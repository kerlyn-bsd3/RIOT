/*
 * MS/TP CRCs — see mstp_crc.h for provenance.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mstp_crc.h"

/*
 * Accumulate 'data_value' into the CRC in 'crc_value'.
 * 'crc_value' must be set to CRC32K_INITIAL_VALUE before initial call.
 * (draft-lynn-6lo-rfc8163-bis-01 reference implementation.)
 */
uint32_t calc_crc32K(uint8_t data_value, uint32_t crc_value)
{
    int b;

    for (b = 0; b < 8; b++) {
        if ((data_value & 1) ^ (crc_value & 1)) {
            crc_value >>= 1;
            crc_value ^= CRC32K_POLY;
        }
        else {
            crc_value >>= 1;
        }
        data_value >>= 1;
    }
    return crc_value;
}

uint32_t mstp_data_crc32k(const uint8_t *data, uint16_t len)
{
    uint32_t crc = CRC32K_INITIAL_VALUE;

    for (uint16_t i = 0; i < len; i++) {
        crc = calc_crc32K(data[i], crc);
    }
    /* 9.10.2 step 5: take the ones' complement for transmission. */
    return ~crc;
}

/*
 * BACnet MS/TP header CRC-8 (135-2024 9.6.1). Per-octet accumulation;
 * 'crc_value' must be initialised to X'FF' before the first call.
 */
uint8_t calc_header_crc(uint8_t data_value, uint8_t crc_value)
{
    uint16_t crc = (uint16_t)(crc_value ^ data_value);

    crc = crc ^ (crc << 1) ^ (crc << 2) ^ (crc << 3)
              ^ (crc << 4) ^ (crc << 5) ^ (crc << 6) ^ (crc << 7);

    return (uint8_t)((crc & 0xFE) ^ ((crc >> 8) & 1));
}

uint8_t mstp_header_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0xFF;

    for (uint16_t i = 0; i < len; i++) {
        crc = calc_header_crc(data[i], crc);
    }
    /* the transmitted Header CRC octet is the ones' complement */
    return (uint8_t)~crc;
}
