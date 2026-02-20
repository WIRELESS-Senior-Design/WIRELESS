#include "led.h"
#include <Arduino.h>
#include <FastLED.h>

// Match your hardware
#define NUM_LEDS 1
#define LED_PIN  48

static CRGB leds[NUM_LEDS];

// Timing owned by LED module (no cross-file globals)
static volatile uint32_t g_lastTxMs = 0;
static volatile uint32_t g_lastRxMs = 0;

void led_init()
{
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  leds[0] = CRGB::Black;
  FastLED.show();
}

void led_off()
{
  leds[0] = CRGB::Black;
  FastLED.show();
}

void led_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
  leds[0] = CRGB(r, g, b);
  FastLED.show();
}

void led_set_hsv(uint8_t h, uint8_t s, uint8_t v)
{
  leds[0] = CHSV(h, s, v);
  FastLED.show();
}

void led_notify_tx()
{
  g_lastTxMs = millis();
  for(uint8_t i=0; i < 6; i++)
  {
    led_set_rgb(0, 255, 0);
    vTaskDelay(20);
    led_set_rgb(0, 0, 0);
    vTaskDelay(20);
  }
}

void led_notify_rx()
{
  g_lastRxMs = millis();
}

void led_task(void *pv)
{
  (void)pv;

  uint8_t hue = 0;

  for (;;)
  {
    const uint32_t now = millis();

    // TX flash window: show GREEN for a short burst
    if ((uint32_t)(now - g_lastTxMs) < 120)   // 120ms after TX
    {
      leds[0] = CRGB(0, 255, 0);
      FastLED.show();
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // RX cooldown window if you want to reserve RX color later
    if ((uint32_t)(now - g_lastRxMs) < 120)
    {
      // example: blue on RX
      leds[0] = CRGB(0, 0, 255);
      FastLED.show();
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Normal idle: smooth flow
    leds[0] = CHSV(hue++, 255, 80);
    FastLED.show();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
