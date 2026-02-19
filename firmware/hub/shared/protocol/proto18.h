#pragma once
#include <stdint.h>
#include <stdbool.h>

// ---- Protocol constants ----
#define FRAME_BITS   18
#define START_FIELD  0b101
#define STOP_FIELD   0b1

typedef struct {
    uint8_t dev_type;   // 2 bits
    uint8_t addr;       // 5 bits
    uint8_t cmd;        // 4 bits
    uint8_t crc3;       // 3 bits (optional to store)
} frame_fields_t;

typedef struct {
    uint8_t mode;   // CMD[3]
    uint8_t topic;  // CMD[2]
    uint8_t action; // CMD[1]
    uint8_t ok;     // CMD[0]
} cmd_fields_t;

// CRC + pack/unpack
uint8_t  crc3_11bits(uint16_t data11);
uint32_t pack_frame18(uint8_t dev_type, uint8_t addr, uint8_t cmd);
bool     unpack_and_check_frame18(uint32_t frame18, frame_fields_t *out);

// CMD helpers
cmd_fields_t cmd_decode(uint8_t cmd);
uint8_t      cmd_encode(uint8_t mode, uint8_t topic, uint8_t action, uint8_t ok);
