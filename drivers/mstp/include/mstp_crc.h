/*
 * MS/TP CRCs.
 *
 * calc_crc32K() and the CRC32K constants are the reference implementation from
 * draft-lynn-6lo-rfc8163-bis-01 (K. Lynn). IETF Trust / Simplified BSD License.
 * The header CRC-8 is the BACnet MS/TP algorithm (ANSI/ASHRAE 135-2024 Clause
 * 9.6.1), verified against a real on-wire capture: header 22 01 02 00 6B -> 0x67,
 * residue X'55'.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MSTP_CRC_H
#define MSTP_CRC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- CRC-32K (Koopman), per draft-lynn-6lo-rfc8163-bis-01 ---------------- */

#define CRC32K_INITIAL_VALUE    (0xFFFFFFFFU)
#define CRC32K_RESIDUE          (0x0843323BU)   /* == 135-2024 9.5.4.9        */
#define CRC32K_POLY             (0xEB31D82EU)   /* 1 + x**1 + ... + x**30 (+ x**32) */

/** Header CRC-8 residue over header + CRC octet (135-2024 9.5.8). */
#define MSTP_HEADER_CRC_RESIDUE (0x55U)

/**
 * @brief   Accumulate @p data_value into the CRC-32K in @p crc_value.
 *
 * @p crc_value must be initialised to @ref CRC32K_INITIAL_VALUE before the
 * first call. Per 135-2024 9.10.2 this is accumulated over the *Encoded Data*
 * field, octet by octet as transmitted/received.
 */
uint32_t calc_crc32K(uint8_t data_value, uint32_t crc_value);

/**
 * @brief   Convenience: CRC-32K over a buffer, returning the transmitted value.
 *
 * Initialises to CRC32K_INITIAL_VALUE, accumulates @p len octets, and returns
 * the ones' complement (9.10.2 step 5). Emit least-significant octet first.
 */
uint32_t mstp_data_crc32k(const uint8_t *data, uint16_t len);

/* --- Header CRC-8 (ANSI/ASHRAE 135-2024 Clause 9.6.1) -------------------- */

/**
 * @brief   Accumulate one header octet into the header CRC.
 * @p crc_value must be initialised to X'FF' before the first call.
 */
uint8_t calc_header_crc(uint8_t data_value, uint8_t crc_value);

/**
 * @brief   Convenience: header CRC-8 over @p len octets, returning the octet to
 *          transmit (the ones' complement of the accumulated value).
 */
uint8_t mstp_header_crc8(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* MSTP_CRC_H */
