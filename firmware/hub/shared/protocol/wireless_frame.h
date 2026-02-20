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
[ 4:2 ] ERR       (3)
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

// ----------------------------
// Command encode/decode
// ----------------------------

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

// ----------------------------
// Simple 3-bit error placeholder
// (Replace with CRC later if desired)
// ----------------------------

static inline uint8_t err3_generate(uint8_t dev_type,
                                    uint8_t addr,
                                    uint8_t cmd)
{
    uint8_t x = (dev_type & 0x3)
              ^ (addr & 0x1F)
              ^ (cmd & 0x0F);

    x ^= (x >> 3);
    x ^= (x >> 2);
    return x & 0x7;
}

// ----------------------------
// Frame pack
// ----------------------------

static inline uint32_t pack_frame20(uint8_t dev_type,
                                    uint8_t addr,
                                    uint8_t cmd)
{
    dev_type &= 0x3;
    addr     &= 0x1F;
    cmd      &= 0x0F;

    uint8_t err3 = err3_generate(dev_type, addr, cmd);

    uint32_t frame =
        ((uint32_t)START_FIELD    << START_SHIFT)    |
        ((uint32_t)PREAMBLE_FIELD << PREAMBLE_SHIFT) |
        ((uint32_t)dev_type       << DEVTYPE_SHIFT)  |
        ((uint32_t)addr           << ADDR_SHIFT)     |
        ((uint32_t)cmd            << CMD_SHIFT)      |
        ((uint32_t)err3           << ERR_SHIFT)      |
        ((uint32_t)STOP_FIELD     << STOP_SHIFT);

    return frame & ((1u << FRAME_BITS) - 1u);
}

// ----------------------------
// Frame unpack + basic validation
// ----------------------------

typedef struct {
    uint8_t dev_type;
    uint8_t addr;
    uint8_t cmd;
    uint8_t err3;
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
    uint8_t err3     = (frame >> ERR_SHIFT)     & 0x7;

    if (out) {
        out->dev_type = dev_type;
        out->addr     = addr;
        out->cmd      = cmd;
        out->err3     = err3;
    }

    return true;
}
