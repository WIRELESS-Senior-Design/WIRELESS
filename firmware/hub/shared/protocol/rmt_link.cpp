#include <Arduino.h>
#include "rmt_link.h"
#include "driver/rmt_tx.h"
#include "proto18.h"

// These are defined in your .ino
extern rmt_channel_handle_t tx_chan;
extern rmt_encoder_handle_t copy_encoder;

rmt_symbol_word_t bit_symbol(bool bit)
{
  rmt_symbol_word_t s;
  s.level0 = bit;
  s.duration0 = 5;
  s.level1 = bit;
  s.duration1 = 5;
  return s;
}

void send_frame18_rmt(uint32_t frame18)
{
  rmt_symbol_word_t syms[FRAME_BITS];

  for (int i = 0; i < FRAME_BITS; i++) {
    bool b = (frame18 >> (FRAME_BITS - 1 - i)) & 1;
    syms[i] = bit_symbol(b);
  }

  rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
  ESP_ERROR_CHECK(rmt_transmit(tx_chan, copy_encoder, syms, sizeof(syms), &tx_cfg));
  ESP_ERROR_CHECK(rmt_tx_wait_all_done(tx_chan, portMAX_DELAY));
}

bool decodeFrame18FromSymbols(const rmt_symbol_word_t* syms, int count, uint32_t* outFrame)
{
  if (count < FRAME_BITS) return false;

  uint32_t frame = 0;
  for (int i = 0; i < FRAME_BITS; i++) {
    bool bit = (syms[i].level0 || syms[i].level1);
    frame = (frame << 1) | (bit ? 1 : 0);
  }

  *outFrame = frame & ((1u << FRAME_BITS) - 1u);
  return true;
}
