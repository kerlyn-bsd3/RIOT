/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file
 * @brief   Build a BACnet MS/TP IPv6-encapsulation (Frame Type 34) frame.
 *
 * Layout per ANSI/ASHRAE 135-2024 Clause 9.3:
 *
 *   55 FF | FT Dst Src LenHi LenLo HdrCRC | Encoded Data | Encoded CRC-32K
 *
 * Frame Type 34 is allocated to the IETF for IPv6-over-MS/TP (RFC 8163) out of
 * ASHRAE's reserved range; per 9.3 all Frame Types 32..127 are COBS-encoded and
 * carry the Encoded Data and Encoded CRC-32K fields.
 *
 * Two rules that are easy to get wrong (both confirmed against 135-2024):
 *
 *  - **Length = E + 3**, where E = len(Encoded Data). 9.3: the Length field is
 *    "the combined length of the Encoded Data and Encoded CRC-32K fields in
 *    octets, minus two" — the Encoded CRC-32K is always five octets, and two are
 *    subtracted for backward compatibility with nodes implementing SKIP_DATA.
 *  - **CRC-32K is computed over the *Encoded Data*** (the COBS output), not the
 *    raw MSDU. 9.5.2: "CRC32K — used ... to accumulate the CRC-32K on the
 *    Encoded Data field." The CRC is then itself COBS-encoded to five octets.
 */

#ifndef MSTP_FRAME_H
#define MSTP_FRAME_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** MS/TP Frame Type for IPv6-over-MS/TP (6LoBAC): 34 (X'22'). */
#define MSTP_FRAME_TYPE_IPV6    (34U)

/** MS/TP preamble octets (9.3). */
#define MSTP_PREAMBLE_1         (0x55U)
#define MSTP_PREAMBLE_2         (0xFFU)

/** Valid Length range for COBS-encoded frames (9.5.3). */
#define MSTP_NMIN_COBS_LENGTH   (5U)
#define MSTP_NMAX_COBS_LENGTH   (2043U)

/** The Encoded CRC-32K field is always five octets (9.3). */
#define MSTP_ENCODED_CRC_LEN    (5U)

/**
 * @brief   Assemble a Frame-Type-34 MS/TP frame carrying @p msdu.
 *
 * @param[in]  src       source MS/TP address (TS)
 * @param[in]  dst       destination MS/TP address (255 = broadcast)
 * @param[in]  msdu      raw MSDU (IPv6 packet / 6LoBAC payload)
 * @param[in]  msdu_len  MSDU length; must be non-zero
 * @param[out] out       output buffer
 * @param[in]  out_max   size of @p out
 *
 * @return total frame length written, or 0 on error.
 */
size_t mstp_build_ipv6_frame(uint8_t src, uint8_t dst,
                             const uint8_t *msdu, uint16_t msdu_len,
                             uint8_t *out, size_t out_max);

#ifdef __cplusplus
}
#endif

#endif /* MSTP_FRAME_H */
