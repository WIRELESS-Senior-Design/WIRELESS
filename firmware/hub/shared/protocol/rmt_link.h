#pragma once
#include <stdint.h>
#include "driver/rmt_types.h"

rmt_symbol_word_t bit_symbol(bool bit);

void send_frame18_rmt(uint32_t frame18); // uses tx_chan/copy_encoder inside .cpp

bool decodeFrame18FromSymbols(const rmt_symbol_word_t* syms, int count, uint32_t* outFrame);
