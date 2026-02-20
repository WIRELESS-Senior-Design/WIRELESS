#pragma once
#include <stdint.h>

// Call once in setup()
void led_init();

// Simple controls
void led_off();
void led_set_rgb(uint8_t r, uint8_t g, uint8_t b);
void led_set_hsv(uint8_t h, uint8_t s, uint8_t v);

// Event hooks (call these from hub code)
void led_notify_tx();   // “I just transmitted”
void led_notify_rx();   // “I just received” (optional)

// Run this as a FreeRTOS task for background flow
void led_task(void *pv);
