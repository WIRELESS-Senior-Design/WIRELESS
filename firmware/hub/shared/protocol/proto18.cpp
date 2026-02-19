#include "proto18.h"

// CRC-3 (x^3 + x + 1) taps = 0b011
uint8_t crc3_11bits(uint16_t data11)
{
    const uint8_t poly = 0b011;
    uint8_t crc = 0;

    for (int i = 10; i >= 0; --i) {
        uint8_t in_bit = (data11 >> i) & 1;
        uint8_t msb    = (crc >> 2) & 1;
        uint8_t fb     = msb ^ in_bit;

        crc = ((crc << 1) & 0b111);
        if (fb) crc ^= poly;
    }
    return crc & 0b111;
}

uint32_t pack_frame18(uint8_t dev_type, uint8_t addr, uint8_t cmd)
{
    dev_type &= 0x3;
    addr     &= 0x1F;
    cmd      &= 0xF;

    uint16_t crc_in =
        ((uint16_t)dev_type << 9) |
        ((uint16_t)addr     << 4) |
        ((uint16_t)cmd      << 0);

    uint8_t crc = crc3_11bits(crc_in);

    uint32_t frame =
        ((uint32_t)START_FIELD << 15) |
        ((uint32_t)dev_type    << 13) |
        ((uint32_t)addr        << 8)  |
        ((uint32_t)cmd         << 4)  |
        ((uint32_t)crc         << 1)  |
        ((uint32_t)STOP_FIELD  << 0);

    return frame;
}

bool unpack_and_check_frame18(uint32_t frame18, frame_fields_t *out)
{
    frame18 &= ((1u << FRAME_BITS) - 1u);

    uint8_t start = (frame18 >> 15) & 0x7;
    uint8_t stop  = (frame18 >> 0)  & 0x1;
    if (start != START_FIELD) return false;
    if (stop  != STOP_FIELD)  return false;

    uint8_t dev_type = (frame18 >> 13) & 0x3;
    uint8_t addr     = (frame18 >> 8)  & 0x1F;
    uint8_t cmd      = (frame18 >> 4)  & 0xF;
    uint8_t crc_rx   = (frame18 >> 1)  & 0x7;

    uint16_t crc_in =
        ((uint16_t)dev_type << 9) |
        ((uint16_t)addr     << 4) |
        ((uint16_t)cmd      << 0);

    uint8_t crc_calc = crc3_11bits(crc_in);
    if (crc_calc != crc_rx) return false;

    if (out) {
        out->dev_type = dev_type;
        out->addr     = addr;
        out->cmd      = cmd;
        out->crc3     = crc_rx;
    }
    return true;
}

cmd_fields_t cmd_decode(uint8_t cmd)
{
    cmd_fields_t c;
    c.mode   = (cmd >> 3) & 1;
    c.topic  = (cmd >> 2) & 1;
    c.action = (cmd >> 1) & 1;
    c.ok     = (cmd >> 0) & 1;
    return c;
}

uint8_t cmd_encode(uint8_t mode, uint8_t topic, uint8_t action, uint8_t ok)
{
    return ((mode & 1) << 3) |
           ((topic & 1) << 2) |
           ((action & 1) << 1) |
           ((ok & 1) << 0);
}
