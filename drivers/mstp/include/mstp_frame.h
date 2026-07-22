/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file
 * @brief   Build a BACnet MS/TP IPv6-encapsulation (Frame Type 34) frame.
 *
 * Migrated verbatim (re-homed onto the driver's crc/cobs/fsm headers) from the
 * on-wire-verified builder in examples/networking/lobac_bringup/. Layout per
 * ANSI/ASHRAE 135-2024 Clause 9.3 / 9.10.2:
 *
 *   55 FF | FT Dst Src LenHi LenLo HdrCRC | Encoded Data | Encoded CRC-32K
 *
 * Frame Type 34 is allocated to the IETF for IPv6-over-MS/TP (RFC 8163) out of
 * ASHRAE's reserved range; per 9.3 all Frame Types 32..127 are COBS-encoded and
 * carry the Encoded Data and Encoded CRC-32K fields.
 *
 * Two rules that are easy to get wrong (both confirmed against 135-2024, and
 * exercised end-to-end by tests-host/test_mstp_frame.c against the Receive FSM):
 *
 *  - **Length = E + 3**, where E = len(Encoded Data). 9.3: the Length field is
 *    "the combined length of the Encoded Data and Encoded CRC-32K fields in
 *    octets, minus two" — the Encoded CRC-32K is always five octets, and two are
 *    subtracted for backward compatibility with nodes implementing SKIP_DATA.
 *  - **CRC-32K is computed over the *Encoded Data*** (the COBS output), not the
 *    raw MSDU (9.5.2 / 9.10.2). The CRC is then itself COBS-encoded to five octets.
 */

#ifndef MSTP_FRAME_H
#define MSTP_FRAME_H

#include <stddef.h>
#include <stdint.h>

#include "mstp_fsm.h"    /* MSTP_FRAME_TYPE_IPV6, MSTP_PREAMBLE_*, COBS length bounds */
#include "mstp_cobs.h"   /* COBS_MAX_ENCODED_LEN, MSTP_ENCODED_CRC_LEN, MSTP_COBS_MASK */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Worst-case buffer size to hold a Frame Type 34 carrying @p msdu octets:
 *          preamble(2) + header(5) + header CRC(1) + Encoded Data + Encoded CRC-32K.
 */
#define MSTP_IPV6_FRAME_MAX(msdu) \
    (8U + COBS_MAX_ENCODED_LEN(msdu) + MSTP_ENCODED_CRC_LEN)

/**
 * @brief   Assemble a Frame-Type-34 MS/TP frame carrying @p msdu.
 *
 * @param[in]  src       source MS/TP address (This Station)
 * @param[in]  dst       destination MS/TP address (255 = broadcast)
 * @param[in]  msdu      raw MSDU (IPv6 packet / 6LoBAC payload)
 * @param[in]  msdu_len  MSDU length; must be non-zero (COBS frames can't be empty)
 * @param[out] out       output buffer
 * @param[in]  out_max   size of @p out (see @ref MSTP_IPV6_FRAME_MAX)
 *
 * @return total frame length written, or 0 on error (NULL/empty MSDU, buffer too
 *         small, or resulting Length outside the COBS 9.5.3 bounds).
 */
size_t mstp_build_ipv6_frame(uint8_t src, uint8_t dst,
                             const uint8_t *msdu, uint16_t msdu_len,
                             uint8_t *out, size_t out_max);

#ifdef __cplusplus
}
#endif

#endif /* MSTP_FRAME_H */
