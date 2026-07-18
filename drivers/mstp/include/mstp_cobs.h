/*
 * COBS (Consistent Overhead Byte Stuffing) for MS/TP.
 *
 * Reference implementation taken from draft-lynn-6lo-rfc8163-bis-01
 * (K. Lynn, "Transmission of IPv6 over Multidrop Serial Bus/Token Passing
 * (MS/TP) Networks"), which obsoletes RFC 8163. Code components of IETF
 * documents are provided under the Simplified BSD License; see the draft's
 * Copyright Notice (IETF Trust).
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Normative behaviour: ANSI/ASHRAE 135-2024 Clause 9.10 (+ Annex T).
 * The encoder eliminates X'00' from the stream and then XORs every output
 * octet with the mask (X'55'), so no preamble octet can appear in the
 * Encoded Data or Encoded CRC-32K fields.
 */

#ifndef MSTP_COBS_H
#define MSTP_COBS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** The octet eliminated from the encoded fields: the MS/TP preamble X'55'. */
#define MSTP_COBS_MASK          (0x55U)

/** Worst-case COBS overhead: one octet per 254, plus the leading code octet. */
#define COBS_OVERHEAD(n)        (1U + ((n) / 254U))
#define COBS_MAX_ENCODED_LEN(n) ((n) + COBS_OVERHEAD(n))

/** The Encoded CRC-32K field is always five octets (135-2024 9.3). */
#define MSTP_ENCODED_CRC_LEN    (5U)

/**
 * @brief   COBS-encode @p length octets at @p from into @p to, XORing with @p mask.
 * @return  number of octets written to @p to.
 */
size_t cobs_encode(uint8_t *to, const uint8_t *from, size_t length, uint8_t mask);

/**
 * @brief   COBS-decode @p length octets at @p from into @p to, un-XORing @p mask.
 * @return  number of octets written to @p to, or 0 on an invalid encoding
 *          (see RFC 8163 Errata 5996).
 */
size_t cobs_decode(uint8_t *to, const uint8_t *from, size_t length, uint8_t mask);

#ifdef __cplusplus
}
#endif

#endif /* MSTP_COBS_H */
