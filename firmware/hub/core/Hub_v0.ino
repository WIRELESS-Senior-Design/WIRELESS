#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_sleep.h"

#include "rmt_tx.h"
#include "led.h" 
#include "wireless_frame.h"


// ----------------------------
// Pins 
// ----------------------------
#define RX_PIN          GPIO_NUM_12   // hub receives from nodes on this pin
#define TX_PIN          GPIO_NUM_13   // hub transmits commands to nodes (later)
#define ADD_BTN_PIN     16            // “Add device” button
#define UI_TOUCH_PIN    17            // UI touch interrupt / activity (example)
#define STATUS_LED_PIN  14
#define NUM_LEDS        1
#define LED_PIN         48


void initRMT();
void sendFrame(uint32_t packet);

// ----------------------------
// Timing / protocol assumptions
// ----------------------------
static const uint32_t RMT_RES_HZ = 10 * 1000 * 1000; // 10MHz
static const TickType_t INACTIVITY_TIMEOUT = pdMS_TO_TICKS(30 * 1000); // 30s demo
static const BaseType_t APP_CORE = 0; // choose 0 or 1
static volatile uint32_t lastRxMs = 0;
static volatile int64_t lastAddIsrUs = 0;
static const int64_t DEBOUNCE_US = 50 * 1000;  // 50ms


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


  esp_light_sleep_start();

  // Woke up
  Serial.println("Woke from LOW POWER");
  mode = MODE_NORMAL;
  digitalWrite(STATUS_LED_PIN, HIGH);
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
      if (e.type == EV_UI_ACTIVITY) //If there is a ui event
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

        uint32_t packet = 0xC281;
        Serial.print("TX packet (16b): 0x");
        Serial.print((uint16_t)packet, HEX);
        Serial.print("  BIN: ");
        Serial.println((uint16_t)packet, BIN);

        led_off();
        // Tell LED module we are transmitting
        led_notify_tx();

        // give RX a moment to arm before TX
        vTaskDelay(pdMS_TO_TICKS(2));
        sendFrame(packet);
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
        enterLowPower();
        lastActivity = xTaskGetTickCount();
      }
    }
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
      led_off();
      //FastLED.show();
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // If we recently RX’d or are TX’ing, don’t run the flow (let RX/TX colors show)
    uint32_t age = millis() - lastRxMs;
    if (age < 200)   // 200ms cooldown after RX
    {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    led_set_hsv(hue++, 255, 80);  // smooth rainbow
    //FastLED.show();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}




// ---------------------------------------------------------------------------------------------
//                                                                                         Setup
// ---------------------------------------------------------------------------------------------
void setup()
{
  digitalWrite(STATUS_LED_PIN, LOW);

  Serial.begin(115200);
  delay(200);

  // ---- Confirm flash + PSRAM ----
  Serial.println("---- MEMORY INFO ----");
  Serial.printf("Chip model: %s\n", ESP.getChipModel());
  Serial.printf("Flash size: %u MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("PSRAM found: %s\n", psramFound() ? "YES" : "NO");
  Serial.printf("PSRAM size: %u MB\n", ESP.getPsramSize() / (1024 * 1024));
  Serial.println("---------------------");


  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  pinMode(ADD_BTN_PIN, INPUT_PULLUP);
  pinMode(UI_TOUCH_PIN, INPUT_PULLUP);

  hubQ = xQueueCreate(16, sizeof(HubEvent));

  attachInterrupt(digitalPinToInterrupt(ADD_BTN_PIN), addBtnISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(UI_TOUCH_PIN), uiTouchISR, FALLING);

  initRMT();
  led_init();

  xTaskCreatePinnedToCore(hubTask, "hub", 4096, NULL, 1, NULL, APP_CORE);
  xTaskCreatePinnedToCore(ledFlowTask, "ledFlow", 2048, NULL, 1, NULL, APP_CORE);
}

void loop()
{
  // unused
} 
