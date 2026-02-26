#pragma once
#include <stdint.h>
#include <stdbool.h>

/*
Frame format (MSB → LSB):

[19:18] START     (2) = 0b10
[17:16] PREAMBLE  (2) = 0b11
[15:14] DEV_TYPE  (2)
[13:9 ] ADDR      (5)
[ 8:5 ] CMD       (4)
[ 4:2 ] ERR       (3)  <-- CRC-3
[ 1:0 ] STOP      (2) = 0b10

Total = 20 bits
*/

#define FRAME_BITS        20

#define START_FIELD       0b10
#define PREAMBLE_FIELD    0b11
#define STOP_FIELD        0b10

// Bit shifts
#define START_SHIFT       18
#define PREAMBLE_SHIFT    16
#define DEVTYPE_SHIFT     14
#define ADDR_SHIFT         9
#define CMD_SHIFT          5
#define ERR_SHIFT          2
#define STOP_SHIFT         0

// ============================================================
// CRC-3 (Polynomial = 1011, x^3 + x + 1)
// ============================================================

static inline uint8_t crc3_compute(uint16_t data, uint8_t bit_len)
{
    uint8_t crc = 0;  // 3-bit remainder register

    for (int i = bit_len - 1; i >= 0; i--)
    {
        uint8_t data_bit = (data >> i) & 1;
        uint8_t msb = (crc >> 2) & 1;

        crc <<= 1;
        crc |= data_bit;

        if (msb)
        {
            crc ^= 0x3;   // lower 3 bits of 1011
        }
    }

    return crc & 0x7;
}

// ============================================================
// Command encode/decode
// ============================================================

static inline uint8_t cmd_encode(uint8_t mode,
                                 uint8_t topic,
                                 uint8_t action,
                                 uint8_t ok)
{
    return ((mode   & 1) << 3) |
           ((topic  & 1) << 2) |
           ((action & 1) << 1) |
           ((ok     & 1) << 0);
}

typedef struct {
    uint8_t mode;
    uint8_t topic;
    uint8_t action;
    uint8_t ok;
} cmd_fields_t;

static inline cmd_fields_t cmd_decode(uint8_t cmd)
{
    cmd_fields_t c;
    c.mode   = (cmd >> 3) & 1;
    c.topic  = (cmd >> 2) & 1;
    c.action = (cmd >> 1) & 1;
    c.ok     = (cmd >> 0) & 1;
    return c;
}

// ============================================================
// Frame pack (uses CRC-3 now)
// ============================================================

static inline uint32_t pack_frame20(uint8_t dev_type,
                                    uint8_t addr,
                                    uint8_t cmd)
{
    dev_type &= 0x3;
    addr     &= 0x1F;
    cmd      &= 0x0F;

    // Build 11-bit protected data
    uint16_t protected_data =
        (dev_type << 9) |
        (addr     << 4) |
        cmd;

    uint8_t crc = crc3_compute(protected_data, 11);

    uint32_t frame =
        ((uint32_t)START_FIELD    << START_SHIFT)    |
        ((uint32_t)PREAMBLE_FIELD << PREAMBLE_SHIFT) |
        ((uint32_t)dev_type       << DEVTYPE_SHIFT)  |
        ((uint32_t)addr           << ADDR_SHIFT)     |
        ((uint32_t)cmd            << CMD_SHIFT)      |
        ((uint32_t)crc            << ERR_SHIFT)      |
        ((uint32_t)STOP_FIELD     << STOP_SHIFT);

    return frame & ((1u << FRAME_BITS) - 1u);
}

// ============================================================
// Frame unpack + FULL validation (including CRC)
// ============================================================

typedef struct {
    uint8_t dev_type;
    uint8_t addr;
    uint8_t cmd;
    uint8_t crc;
} frame20_fields_t;

static inline bool unpack_and_check_frame20(uint32_t frame,
                                            frame20_fields_t *out)
{
    frame &= ((1u << FRAME_BITS) - 1u);

    uint8_t start    = (frame >> START_SHIFT)    & 0x3;
    uint8_t preamble = (frame >> PREAMBLE_SHIFT) & 0x3;
    uint8_t stop     = (frame >> STOP_SHIFT)     & 0x3;

    if (start != START_FIELD) return false;
    if (preamble != PREAMBLE_FIELD) return false;
    if (stop != STOP_FIELD) return false;

    uint8_t dev_type = (frame >> DEVTYPE_SHIFT) & 0x3;
    uint8_t addr     = (frame >> ADDR_SHIFT)    & 0x1F;
    uint8_t cmd      = (frame >> CMD_SHIFT)     & 0x0F;
    uint8_t crc_rx   = (frame >> ERR_SHIFT)     & 0x7;

    // Rebuild protected data
    uint16_t protected_data =
        (dev_type << 9) |
        (addr     << 4) |
        cmd;

    uint8_t crc_expected = crc3_compute(protected_data, 11);

    if (crc_expected != crc_rx)
        return false;   // CRC failed

    if (out) {
        out->dev_type = dev_type;
        out->addr     = addr;
        out->cmd      = cmd;
        out->crc      = crc_rx;
    }

    return true;
}
