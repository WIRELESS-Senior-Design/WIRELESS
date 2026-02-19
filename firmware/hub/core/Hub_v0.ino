#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_types.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_sleep.h"

#include <FastLED.h>

// ----------------------------
// Pins 
// ----------------------------
#define RX_PIN          GPIO_NUM_12   // hub receives from nodes on this pin
#define TX_PIN          GPIO_NUM_13   // hub transmits commands to nodes (later)
#define ADD_BTN_PIN     16            // “Add device” button
#define UI_TOUCH_PIN    17            // UI touch interrupt / activity (example)
#define STATUS_LED_PIN  14
#define NUM_LEDS         1
#define LED_PIN          48
#define FRAME_BITS   18
#define START_BITS   3
#define STOP_BITS    1

#define START_FIELD  0b101
#define STOP_FIELD   0b1
CRGB leds[NUM_LEDS];

//#ifndef LED_BUILTIN
//#define LED_BUILTIN 2
//#endif


typedef struct {
    uint8_t dev_type;   // 2 bits
    uint8_t addr;       // 5 bits
    uint8_t cmd;        // 4 bits
    uint8_t crc3;       // 3 bits
} frame_fields_t;

typedef struct {
    uint8_t mode;   // CMD[3]
    uint8_t topic;  // CMD[2]
    uint8_t action; // CMD[1]
    uint8_t ok;     // CMD[0]
} cmd_fields_t;

static inline void rgbShow(CRGB c, uint32_t ms)
{
  leds[0] = c;
  FastLED.show();
  vTaskDelay(pdMS_TO_TICKS(ms));
}

// ----------------------------
// Timing / protocol assumptions
// ----------------------------
static const uint32_t RMT_RES_HZ = 10 * 1000 * 1000; // 10MHz
static const TickType_t INACTIVITY_TIMEOUT = pdMS_TO_TICKS(30 * 1000); // 30s demo
static const BaseType_t APP_CORE = 0; // choose 0 or 1
static volatile uint32_t lastRxMs = 0;
//static volatile uint32_t lastAddIsrMs = 0;
static volatile int64_t lastAddIsrUs = 0;
static const int64_t DEBOUNCE_US = 50 * 1000;  // 50ms


// ----------------------------
// RMT handles
// ----------------------------
static rmt_channel_handle_t rx_chan = NULL;
static rmt_channel_handle_t tx_chan = NULL;
static rmt_encoder_handle_t copy_encoder = NULL;

// RX task notification + symbol count from ISR callback
static TaskHandle_t rmtRxTaskHandle = NULL;
static volatile size_t rx_num_symbols = 0;


static TaskHandle_t txTaskHandle = NULL;
static volatile bool txRequestPending = false;




// ----------------------------------------------------------------------------
//                                                                Hub events
// ----------------------------------------------------------------------------
enum HubEventType : uint8_t
{
  EV_ADD_BUTTON = 1,
  EV_UI_ACTIVITY,
  EV_INACTIVITY_TIMEOUT,
  EV_NODE_ID_RX,
};

struct HubEvent
{
  HubEventType type;
  uint8_t id;          // for EV_NODE_ID_RX
};

static QueueHandle_t hubQ;

// ----------------------------
// Device table (simple)
// ----------------------------
static const int MAX_DEVICES = 32;
static uint8_t devices[MAX_DEVICES];
static int deviceCount = 0;

static bool deviceExists(uint8_t id)
{
  for (int i = 0; i < deviceCount; i++) if (devices[i] == id) return true;
  return false;
}

static bool addDevice(uint8_t id)
{
  if (deviceExists(id)) return false;
  if (deviceCount >= MAX_DEVICES) return false;
  devices[deviceCount++] = id;
  return true;
}

// ----------------------------------------------------------------------------
//                                                          Mode/state machine
// ----------------------------------------------------------------------------
enum HubMode : uint8_t
{
  MODE_NORMAL = 0,
  MODE_ADD_DEVICE,
  MODE_LOW_POWER
};

static HubMode mode = MODE_NORMAL;


//-------------------------------------------------------------------------------------------
//-------------------------------------     HELPERS     -------------------------------------
//-------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------
//                                                                   post event
// ----------------------------------------------------------------------------

static void postEvent(HubEventType t, uint8_t id = 0)
{
  HubEvent e{t, id};
  xQueueSend(hubQ, &e, 0);
}

static void requestTx()
{
  txRequestPending = true;
  if (txTaskHandle) xTaskNotifyGive(txTaskHandle);
}


// ----------------------------------------------------------------------------
//                                                          GPIO ISRs -> events
// ----------------------------------------------------------------------------
static void IRAM_ATTR addBtnISR()
{

  // fixing debounce
  const int64_t now = esp_timer_get_time(); // us, safe in ISR
  if ((now - lastAddIsrUs) < DEBOUNCE_US) return;
  lastAddIsrUs = now;
  
  BaseType_t hp = pdFALSE;
  HubEvent e{EV_ADD_BUTTON, 0};
  xQueueSendFromISR(hubQ, &e, &hp);

  
  if (hp) portYIELD_FROM_ISR();
}

static void IRAM_ATTR uiTouchISR()
{
  BaseType_t hp = pdFALSE;
  HubEvent e{EV_UI_ACTIVITY, 0};
  xQueueSendFromISR(hubQ, &e, &hp);
  if (hp) portYIELD_FROM_ISR();
}


// ----------------------------------------------------------------------------
// RMT RX done callback (ISR context) -> notify rx task
// ----------------------------------------------------------------------------
static bool IRAM_ATTR on_rx_done(rmt_channel_handle_t channel,
                                const rmt_rx_done_event_data_t *edata,
                                void *user_data)
{
  (void)channel;
  rx_num_symbols = edata->num_symbols;

  BaseType_t hp = pdFALSE;
  vTaskNotifyGiveFromISR((TaskHandle_t)user_data, &hp);
  return hp == pdTRUE;
}


// ---------------------------------------------------------------------------------------------
//                                                                         RMT RX: init + decode
// ---------------------------------------------------------------------------------------------


//------------------------------------------------------------------------
//
//                                                             rmt Rx Init
//
//------------------------------------------------------------------------
static void initRmtRx()
{
  rmt_rx_channel_config_t rx_cfg = {
    .gpio_num = RX_PIN,
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = RMT_RES_HZ,
    .mem_block_symbols = 128,
    .flags = {
      .invert_in = 0,
      .with_dma = 0,
    },
  };

  ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &rx_chan));

  // Register RX done callback
//  rmt_rx_event_callbacks_t cbs = {
//    .on_recv_done = on_rx_done,
//  };
//  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, (void*)rmtRxTaskHandle));

  ESP_ERROR_CHECK(rmt_enable(rx_chan));
  Serial.printf("RMT RX enabled, chan=%p\n", rx_chan);
}


//------------------------------------------------------------------------
//
//                                                             rmt Tx Init
//
//------------------------------------------------------------------------

static void initRmtTx()
{
  rmt_tx_channel_config_t tx_cfg = {
    .gpio_num = TX_PIN,
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = RMT_RES_HZ,
    .mem_block_symbols = 64,
    .trans_queue_depth = 4,
  };

  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &tx_chan));
  ESP_ERROR_CHECK(rmt_enable(tx_chan));

  rmt_copy_encoder_config_t copy_cfg = {};
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &copy_encoder));

  Serial.printf("RMT TX enabled, chan=%p\n", tx_chan);
}


// ---------------------------------------------------------------------------------------------
//                                                                 RMT TX test (loopback TX->RX)
// ---------------------------------------------------------------------------------------------
static inline rmt_symbol_word_t bit_symbol(bool bit)
{
  rmt_symbol_word_t s;
  // Force a transition every bit:
  // 0 = LOW then HIGH
  // 1 = HIGH then LOW
  s.level0 = bit ? 1 : 0;
  s.duration0 = 5;   // 500 ns @ 10MHz
  s.level1 = bit ? 0 : 1;
  s.duration1 = 5;   // 500 ns
  return s;
}

        
    static void sendTestBits4(uint8_t bits4)
    {
      rmt_symbol_word_t syms[4];
    
      for (int i = 0; i < 4; i++)
      {
        bool b = (bits4 >> (3 - i)) & 1;
        syms[i] = bit_symbol(b);
      }
    
      rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    
      ESP_ERROR_CHECK(rmt_transmit(tx_chan, copy_encoder, syms, sizeof(syms), &tx_cfg));
      ESP_ERROR_CHECK(rmt_tx_wait_all_done(tx_chan, portMAX_DELAY));
    }


// ---------------------------------------------------------------------------------------------
//                                                                Decode 4 ID bits from symbols.
//                                                   Assumes each symbol corresponds to one bit.
// ---------------------------------------------------------------------------------------------
//static bool decodeIdFromSymbols(const rmt_symbol_word_t* syms, int count, uint8_t* outId)
//{     
////  if (count < 4)
////  {
////    Serial.println("Count < 4");
////    return false;
////  }
//
//  uint8_t id = 0;
//
//  for (int i = 0; i < 4; i++)
//  {
//    // TX sets both halves same anyway, so either half being HIGH is HIGH
//    bool bit = (syms[i].level0 || syms[i].level1); 
//    id = (id << 1) | (bit ? 1 : 0);                
//
//    //Serial.println("Done Decoding bits"); 
//  }
//
//    
//  *outId = id & 0x0F;
//  return true;
//}

static bool decodeFrame18FromSymbols(const rmt_symbol_word_t* syms, int count, uint32_t* outFrame)
{
  if (count < FRAME_BITS) return false;

  uint32_t frame = 0;
  for (int i = 0; i < FRAME_BITS; i++)
  {
    bool bit = syms[i].level0;          // <-- changed
    frame = (frame << 1) | (bit ? 1 : 0);
  }

  *outFrame = frame & ((1u << FRAME_BITS) - 1u);
  return true;
}



// ---------------------------------------------------------------------------------------------
//                                                              Low power handling (LIGHT SLEEP)
//                                   NOTE: Using GPIO wake (works with non-RTC pins like 16/17).
// ---------------------------------------------------------------------------------------------
static void enterLowPower()
{
  mode = MODE_LOW_POWER;
  digitalWrite(STATUS_LED_PIN, LOW);

  Serial.println("Entering LOW POWER mode");

  // Disable all wake sources, then configure GPIO wake for light sleep
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
  ESP_ERROR_CHECK(gpio_wakeup_enable((gpio_num_t)UI_TOUCH_PIN, GPIO_INTR_LOW_LEVEL));
  ESP_ERROR_CHECK(gpio_wakeup_enable((gpio_num_t)ADD_BTN_PIN,  GPIO_INTR_LOW_LEVEL));

  // Pause RMT while sleeping (optional but good hygiene)
  if (rx_chan) rmt_disable(rx_chan);
  if (tx_chan) rmt_disable(tx_chan);

  esp_light_sleep_start();

  // Woke up
  Serial.println("Woke from LOW POWER");
  mode = MODE_NORMAL;
  digitalWrite(STATUS_LED_PIN, HIGH);

  // Re-enable after wake
  if (rx_chan) ESP_ERROR_CHECK(rmt_enable(rx_chan));
  if (tx_chan) ESP_ERROR_CHECK(rmt_enable(tx_chan));
}


// ---------------------------------------------------------------------------------------------
//                                                              
//                                                                      Cyclic Redundancy Check                                  
//
// ---------------------------------------------------------------------------------------------
// CRC-3 (x^3 + x + 1) => poly bits for the 3 lower terms: 0b011
static inline uint8_t crc3_11bits(uint16_t data11)
{
    // data11 is 11 bits: [10:0] = DEV_TYPE(2) | ADDR(5) | CMD(4)
    // We'll shift MSB-first through a 3-bit register.
    const uint8_t poly = 0b011;   // taps for x^1 and x^0 when MSB xor occurs
    uint8_t crc = 0;             // 3-bit

    for (int i = 10; i >= 0; --i) {
        uint8_t in_bit = (data11 >> i) & 1;
        uint8_t msb = (crc >> 2) & 1;          // top bit of 3-bit crc
        uint8_t fb = msb ^ in_bit;             // feedback bit

        crc = ((crc << 1) & 0b111);            // shift left, keep 3 bits
        if (fb) crc ^= poly;                   // apply polynomial taps
    }
    return (crc & 0b111);
}


// ---------------------------------------------------------------------------------------------
//                                                              
//                                                                              Frame assembling                                  
//
// ---------------------------------------------------------------------------------------------

static inline uint32_t pack_frame18(uint8_t dev_type, uint8_t addr, uint8_t cmd)
{
    dev_type &= 0x3;
    addr     &= 0x1F;
    cmd      &= 0xF;

    // Build 11-bit CRC input: DEV_TYPE(2) | ADDR(5) | CMD(4)
    uint16_t crc_in =
        ((uint16_t)dev_type << 9) |
        ((uint16_t)addr     << 4) |
        ((uint16_t)cmd      << 0);

    uint8_t crc = crc3_11bits(crc_in);

    uint32_t frame =
        ((uint32_t)START_FIELD << 15) |
        ((uint32_t)dev_type    << 13) |
        ((uint32_t)addr        << 8)  |
        ((uint32_t)cmd         << 4)  |
        ((uint32_t)crc         << 1)  |
        ((uint32_t)STOP_FIELD  << 0);

    return frame; // only low 18 bits used
}


// ---------------------------------------------------------------------------------------------
//                                                              
//                                                                                    CMD Decode                                  
//
// ---------------------------------------------------------------------------------------------
static inline cmd_fields_t cmd_decode(uint8_t cmd)
{
    cmd_fields_t c;
    c.mode   = (cmd >> 3) & 1;
    c.topic  = (cmd >> 2) & 1;
    c.action = (cmd >> 1) & 1;
    c.ok     = (cmd >> 0) & 1;
    return c;
}



// ---------------------------------------------------------------------------------------------
//                                                              
//                                                                                    CMD Encode                                  
//
// ---------------------------------------------------------------------------------------------
static inline uint8_t cmd_encode(uint8_t mode, uint8_t topic, uint8_t action, uint8_t ok)
{
    return ((mode & 1) << 3) |
           ((topic & 1) << 2) |
           ((action & 1) << 1) |
           ((ok & 1) << 0);
}

// ---------------------------------------------------------------------------------------------
//                                                              
//                                                                       Send 18 bits, MSB first                                  
//
// ---------------------------------------------------------------------------------------------
static inline void send_frame18_rmt(uint32_t frame18)
{
  rmt_symbol_word_t syms[FRAME_BITS];

  for (int i = 0; i < FRAME_BITS; i++)
  {
    bool b = (frame18 >> (FRAME_BITS - 1 - i)) & 1; // MSB first
    syms[i] = bit_symbol(b);
  }

  rmt_transmit_config_t tx_cfg = { .loop_count = 0 };

  ESP_ERROR_CHECK(rmt_transmit(tx_chan, copy_encoder, syms, sizeof(syms), &tx_cfg));
  ESP_ERROR_CHECK(rmt_tx_wait_all_done(tx_chan, portMAX_DELAY));
}


//-------------------------------------------------------------------------------------------
//--------------------------------------     TASKS     --------------------------------------
//-------------------------------------------------------------------------------------------



// ---------------------------------------------------------------------------------------------
//                                                                        Hub state machine task
// ---------------------------------------------------------------------------------------------
static void hubTask(void* pv)
{
  (void)pv;
  TickType_t lastActivity = xTaskGetTickCount();

  Serial.println("Hub task started");
  digitalWrite(STATUS_LED_PIN, HIGH);

  while (1)
  {
    HubEvent e;

    if (xQueueReceive(hubQ, &e, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      if (e.type == EV_UI_ACTIVITY)
      {
        lastActivity = xTaskGetTickCount();
      }


//------------------------------------------------------------------------
//
//                                                              Add Device
//
//------------------------------------------------------------------------
      if (e.type == EV_ADD_BUTTON)
      {
        lastActivity = xTaskGetTickCount();
        mode = MODE_ADD_DEVICE;
        Serial.println("MODE_ADD_DEVICE: waiting for node ID...");

        // give RX a moment to arm before TX
        vTaskDelay(pdMS_TO_TICKS(2));
        requestTx();
      }




//------------------------------------------------------------------------
//
//                                             Confirmm or Deny Device add
//
//------------------------------------------------------------------------
      if (e.type == EV_NODE_ID_RX)
      {
        lastActivity = xTaskGetTickCount();

        if (mode == MODE_ADD_DEVICE)
        {
          bool ok = addDevice(e.id);
          Serial.print("Enroll ID ");
          Serial.print(e.id, BIN);
          Serial.println(ok ? " -> ADDED" : " -> already exists / table full");
          mode = MODE_NORMAL;
        }
        else
        {
          Serial.print("Heard node ID ");
          Serial.println(e.id, BIN);
        }
      }
    }

    

//------------------------------------------------------------------------
//
//                                                       Setting Low Power
//
//------------------------------------------------------------------------
    // Inactivity timeout logic (only when not already low power)
    if (mode != MODE_LOW_POWER)
    {
      TickType_t now = xTaskGetTickCount();
      if ((now - lastActivity) > INACTIVITY_TIMEOUT)
      {
        // Uncomment when you're ready:
        // enterLowPower();
        lastActivity = xTaskGetTickCount();
      }
    }
  }
}



// ---------------------------------------------------------------------------------------------
//                                                                         TX test task (GPIO13)
// ---------------------------------------------------------------------------------------------
//static void rmtTxTestTask(void* pv)
//{
//  (void)pv;
//   Serial.println("rmt TX Test task started");
//  uint8_t pattern = 0b1010;
//  vTaskDelay(pdMS_TO_TICKS(500));  // let init finish
//
//  while (1)
//  {
//    if (mode != MODE_LOW_POWER)
//    {
//      rgbShow(CRGB::Green, 20);
//      sendTestBits4(pattern);
//      rgbShow(CRGB::Black, 1);
//      
//      Serial.print("TX test sent: ");
//      Serial.println(pattern, BIN);
//      pattern = (pattern + 1) & 0x0F;
//    }
//    vTaskDelay(pdMS_TO_TICKS(1000));
//  }
//}

static void rmtTxTask(void* pv)
{
  (void)pv;
  txTaskHandle = xTaskGetCurrentTaskHandle();
  Serial.println("rmt TX task started");

  uint8_t pattern = 0b1010;

  while (1)
  {
    // sleep forever until someone notifies us (button/UI event)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    

    if (mode == MODE_LOW_POWER) continue;
    if (!tx_chan) continue;

    // do a short burst (you can change count)
    const int burstCount = 1;
    for (int i = 0; i < burstCount; i++)
    {
      //rgbShow(CRGB::Green, 30);      // TX indication
      // Example: HUB->NODE pairing request
      uint8_t dev_type = 0;                  // 00 = hub (your convention)
      uint8_t addr     = 1;                  // target node id for test
      uint8_t cmd      = cmd_encode(1,0,0,0); // 0b1000 pairing request
      
      uint32_t frame18 = pack_frame18(dev_type, addr, cmd);
      
      rgbShow(CRGB::Green, 30);
      send_frame18_rmt(frame18);
      rgbShow(CRGB::Black, 5);

      Serial.println("Start (3), Type (2), Addr (5), CMD (4), CRC3(3), Stop (1)\n");
      Serial.print("TX frame18: ");
      Serial.println(frame18, BIN);

      //rgbShow(CRGB::Black, 5);
//
//      Serial.print("TX sent: ");
//      Serial.println(pattern, BIN);

      //pattern = (pattern + 1) & 0x0F;
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    txRequestPending = false;
  }
}


// ---------------------------------------------------------------------------------------------
//                                                                              RX task (GPIO12)
//                          Proper pattern: start receive -> wait for RX-done callback -> decode
// ---------------------------------------------------------------------------------------------
static void rmtRxTask(void* pv)
{
  (void)pv;
  Serial.println("rmt RX task started");
  // Save task handle so callback can notify us
  rmtRxTaskHandle = xTaskGetCurrentTaskHandle();

  // Re-register callbacks now that we have a valid task handle
  if (rx_chan)
  {
    rmt_rx_event_callbacks_t cbs = { .on_recv_done = on_rx_done };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, (void*)rmtRxTaskHandle));
  }

  static rmt_symbol_word_t rx_buf[18];

  rmt_receive_config_t rx_rcfg = {
    .signal_range_min_ns = 200,
    .signal_range_max_ns = 10 * 1000
  };

  // Ensure enabled
  //if (rx_chan) ESP_ERROR_CHECK(rmt_enable(rx_chan));

  while (1)
  {
    if (mode == MODE_LOW_POWER)
    {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (!rx_chan)
    {
      Serial.println("rx_chan is NULL!");
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    // Start a receive
    esp_err_t err = rmt_receive(rx_chan, rx_buf, sizeof(rx_buf), &rx_rcfg);
    if (err != ESP_OK)
    {
      Serial.printf("rmt_receive start err=%d\n", (int)err);
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // Wait for RX done callback to notify
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Serial.print("RX symbols: ");
    Serial.println(rx_num_symbols);

    lastRxMs = millis();


    // Decode using actual symbol count captured by callback
    // Decode using actual symbol count captured by callback
    uint32_t frame18 = 0;
    frame_fields_t f;
    
    rgbShow(CRGB::Red, 20);
    rgbShow(CRGB::Black, 1);

    if (rx_num_symbols < FRAME_BITS) 
    {
          Serial.println("RX too short (<18 symbols) -> ignore");
        continue;
    }


    
    if (decodeFrame18FromSymbols(rx_buf, (int)rx_num_symbols, &frame18) &&
        unpack_and_check_frame18(frame18, &f))
    {
      postEvent(EV_NODE_ID_RX, f.addr);
    
      Serial.print("RX frame18: ");
      Serial.println(frame18, BIN);
    
      Serial.printf("Parsed: dev_type=%u addr=%u cmd=0x%X crc3=%u\n",
                    f.dev_type, f.addr, f.cmd, f.crc3);
    
      cmd_fields_t c = cmd_decode(f.cmd);
      Serial.printf("CMD: mode=%u topic=%u action=%u ok=%u\n",
                    c.mode, c.topic, c.action, c.ok);
    }
    else
    {
      Serial.println("RX invalid (len/start/stop/crc)");
    }
    
    rgbShow(CRGB::Red, 20);
    rgbShow(CRGB::Black, 1);

  }
}

// ---------------------------------------------------------------------------------------------
//                                                                                 LED Flow task
// ---------------------------------------------------------------------------------------------
static void ledFlowTask(void* pv)
{
  (void)pv;
  Serial.println("LED flow task started");

  uint8_t hue = 0;

  while (1)
  {
    if (mode == MODE_LOW_POWER)
    {
      leds[0] = CRGB::Black;
      FastLED.show();
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // If we recently RX’d or are TX’ing, don’t run the flow (let RX/TX colors show)
    uint32_t age = millis() - lastRxMs;
    if (txRequestPending || age < 200)   // 200ms cooldown after RX
    {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    leds[0] = CHSV(hue++, 255, 80);  // smooth rainbow
    FastLED.show();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}


// ---------------------------------------------------------------------------------------------
//                                                                                Opening packet
// ---------------------------------------------------------------------------------------------
static inline bool unpack_and_check_frame18(uint32_t frame18, frame_fields_t *out)
{
    frame18 &= ((1u << FRAME_BITS) - 1u);

    uint8_t start = (frame18 >> 15) & 0x7;
    uint8_t stop  = (frame18 >> 0)  & 0x1;

    if (start != START_FIELD) return false;
    if (stop  != STOP_FIELD)  return false;

    uint8_t dev_type = (frame18 >> 13) & 0x3;
    uint8_t addr     = (frame18 >> 8)  & 0x1F;
    uint8_t cmd      = (frame18 >> 4)  & 0xF;
    uint8_t crc_rx   = (frame18 >> 1)  & 0x7;

    uint16_t crc_in =
        ((uint16_t)dev_type << 9) |
        ((uint16_t)addr     << 4) |
        ((uint16_t)cmd      << 0);

    Serial.print("RX raw frame: ");
    Serial.println(frame18, BIN);

    uint8_t crc_calc = crc3_11bits(crc_in);
    if (crc_calc != crc_rx) return false;

    if (out) {
        out->dev_type = dev_type;
        out->addr     = addr;
        out->cmd      = cmd;    }
    return true;
}









// ---------------------------------------------------------------------------------------------
//                                                                                         Setup
// ---------------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  delay(1000);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

  // ---- Confirm flash + PSRAM ----
  Serial.println("---- MEMORY INFO ----");
  Serial.printf("Chip model: %s\n", ESP.getChipModel());
  Serial.printf("Flash size: %u MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("PSRAM found: %s\n", psramFound() ? "YES" : "NO");
  Serial.printf("PSRAM size: %u MB\n", ESP.getPsramSize() / (1024 * 1024));
  Serial.println("---------------------");

  //pinMode(STATUS_LED_PIN, OUTPUT);
  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, LOW);
  pinMode(ADD_BTN_PIN, INPUT_PULLUP);
  pinMode(UI_TOUCH_PIN, INPUT_PULLUP);
 
  hubQ = xQueueCreate(16, sizeof(HubEvent));

  attachInterrupt(digitalPinToInterrupt(ADD_BTN_PIN), addBtnISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(UI_TOUCH_PIN), uiTouchISR, FALLING);

  // Init RMT
  initRmtRx();
  initRmtTx();

  // Tasks
xTaskCreatePinnedToCore(rmtRxTask,   "rmtRx",   4096, NULL, 2, NULL, APP_CORE);
xTaskCreatePinnedToCore(rmtTxTask,   "rmtTx",   4096, NULL, 1, NULL, APP_CORE);
xTaskCreatePinnedToCore(hubTask,     "hub",     4096, NULL, 1, NULL, APP_CORE);
xTaskCreatePinnedToCore(ledFlowTask, "ledFlow", 2048, NULL, 1, NULL, APP_CORE);

}

void loop()
{
  // unused
} 
