#include <Arduino.h>
#include "driver/rmt_rx.h"
#include "driver/rmt_types.h"

// ------------------------------------------------------------
// Configuration
// ------------------------------------------------------------

#define RX_GPIO GPIO_NUM_14
#define HALF_BIT 1000
#define MAX_HALFBITS 256

// ------------------------------------------------------------
// Global RMT handle
// ------------------------------------------------------------

rmt_channel_handle_t rx_chan = NULL;

// ------------------------------------------------------------
// Convert RX symbols into half-bit stream
// ------------------------------------------------------------

int rxToBitStream(rmt_symbol_word_t rx_buffer[],
                  int maxSymbols,
                  uint8_t bitStream[])
{
    int bitIndex = 0;

    for (int i = 0; i < maxSymbols; i++)
    {
        if (rx_buffer[i].duration0 == 0 &&
            rx_buffer[i].duration1 == 0)
            break;

        // First segment
        if (rx_buffer[i].duration0 != 0)
        {
            int count = rx_buffer[i].duration0 / HALF_BIT;

            for (int j = 0; j < count; j++)
            {
                if (bitIndex < MAX_HALFBITS)
                    bitStream[bitIndex++] = rx_buffer[i].level0;
            }
        }

        // Second segment
        if (rx_buffer[i].duration1 != 0)
        {
            int count = rx_buffer[i].duration1 / HALF_BIT;

            for (int j = 0; j < count; j++)
            {
                if (bitIndex < MAX_HALFBITS)
                    bitStream[bitIndex++] = rx_buffer[i].level1;
            }
        }
    }

    return bitIndex;
}

// ------------------------------------------------------------
// RMT Initialization
// ------------------------------------------------------------

void initRMT()
{
    rmt_rx_channel_config_t rx_config = {
        .gpio_num = RX_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,   // Must match TX
        .mem_block_symbols = 64,
    };

    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_config, &rx_chan));
    ESP_ERROR_CHECK(rmt_enable(rx_chan));
}

// ------------------------------------------------------------
// RX Task (FreeRTOS)
// ------------------------------------------------------------

void rxTask(void *pvParameters)
{
    static rmt_symbol_word_t rx_buffer[64];
    uint8_t halfBits[MAX_HALFBITS];

    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 100,
        .signal_range_max_ns = 3000000,
    };

    vTaskDelay(pdMS_TO_TICKS(500));

    while (1)
    {
        ESP_ERROR_CHECK(
            rmt_receive(
                rx_chan,
                rx_buffer,
                sizeof(rx_buffer),
                &receive_config)
        );

        // Give hardware time to capture
        vTaskDelay(pdMS_TO_TICKS(300));

        Serial.println("Received Symbols:");

        for (int i = 0; i < 10; i++)
        {
            if (rx_buffer[i].duration0 == 0 &&
                rx_buffer[i].duration1 == 0)
                break;

            Serial.printf("Symbol %d | L0=%d D0=%d | L1=%d D1=%d\n",
                          i,
                          rx_buffer[i].level0,
                          rx_buffer[i].duration0,
                          rx_buffer[i].level1,
                          rx_buffer[i].duration1);
        }

        int halfCount = rxToBitStream(rx_buffer, 64, halfBits);

        Serial.print("Half-bit Stream: ");
        for (int i = 0; i < halfCount; i++)
        {
            Serial.print(halfBits[i]);
        }
        Serial.println();
        Serial.println("----------------------");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ------------------------------------------------------------
// Arduino Setup
// ------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("RX Starting...");

    initRMT();

    xTaskCreate(
        rxTask,
        "RX Task",
        4096,
        NULL,
        1,
        NULL
    );
}

// ------------------------------------------------------------
// Arduino Loop (unused)
// ------------------------------------------------------------

void loop()
{
}

