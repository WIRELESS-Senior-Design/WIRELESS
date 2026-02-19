#include <Arduino.h>
#include "driver/rmt_tx.h"
#include "driver/rmt_types.h"

#define TX_PIN GPIO_NUM_13
#define HALF_BIT 1000
#define DATA_BITS 4
#define FRAME_SYMBOLS 6   // start + data + stop

rmt_channel_handle_t tx_chan = NULL;
rmt_encoder_handle_t copy_encoder = NULL;

// Example ID
uint8_t device_id = 0b1010;

// ------------------------------------------------------------
// Initialize RMT
// ------------------------------------------------------------

void initRMT()
{
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = TX_PIN,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,  // 1 tick = 1 µs
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &tx_chan));
    ESP_ERROR_CHECK(rmt_enable(tx_chan));

    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &copy_encoder));
}

// ------------------------------------------------------------
// Manchester Encoder
// 1 → LOW → HIGH
// 0 → HIGH → LOW
// ------------------------------------------------------------

static inline rmt_symbol_word_t manchester_symbol(bool bit)
{
    rmt_symbol_word_t s;

    if (bit)
    {
        s.level0 = 0;
        s.duration0 = HALF_BIT;
        s.level1 = 1;
        s.duration1 = HALF_BIT;
    }
    else
    {
        s.level0 = 1;
        s.duration0 = HALF_BIT;
        s.level1 = 0;
        s.duration1 = HALF_BIT;
    }

    return s;
}

// ------------------------------------------------------------
// Send Frame
// ------------------------------------------------------------

void sendFrame(uint8_t id)
{
    rmt_symbol_word_t symbols[FRAME_SYMBOLS];

    // Start bit = 1
    symbols[0] = manchester_symbol(0);

    // Data bits (MSB first)
    for (int i = 0; i < DATA_BITS; i++)
    {
        bool bit = (id >> (DATA_BITS - 1 - i)) & 1;
        symbols[i + 1] = manchester_symbol(bit);
    }

    // Stop bit = 1
    symbols[DATA_BITS + 1] = manchester_symbol(0);

    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };

    ESP_ERROR_CHECK(
        rmt_transmit(
            tx_chan,
            copy_encoder,
            symbols,
            sizeof(symbols),
            &tx_cfg)
    );

    ESP_ERROR_CHECK(rmt_tx_wait_all_done(tx_chan, portMAX_DELAY));
}

// ------------------------------------------------------------
// TX Task
// ------------------------------------------------------------

void txTask(void *pv)
{
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1)
    {
        Serial.printf("Sending: %s\n", String(device_id, BIN).c_str());

        sendFrame(device_id);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void setup()
{
    Serial.begin(115200);
    initRMT();

    xTaskCreate(
        txTask,
        "TX Task",
        4096,
        NULL,
        1,
        NULL
    );
}

void loop()
{
}

