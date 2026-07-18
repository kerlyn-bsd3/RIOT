/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mstp_frame.h"
#include "mstp_crc.h"
#include "mstp_cobs.h"

size_t mstp_build_ipv6_frame(uint8_t src, uint8_t dst,
                             const uint8_t *msdu, uint16_t msdu_len,
                             uint8_t *out, size_t out_max)
{
    /* COBS-encoded frames always carry Encoded Data + Encoded CRC-32K (9.3),
     * so a zero-length MSDU is not representable. */
    if (msdu == NULL || msdu_len == 0U) {
        return 0;
    }
    /* The reference cobs_encode() takes no output bound, so check up front:
     * preamble+header (8) + worst-case Encoded Data + Encoded CRC-32K (5). */
    if (out_max < 8U + COBS_MAX_ENCODED_LEN(msdu_len) + MSTP_ENCODED_CRC_LEN) {
        return 0;
    }

    size_t idx = 8;   /* reserve preamble(2) + header(5) + header CRC(1) */

    /* 9.10.2 (1): Encoded Data = COBS(client data), each octet XOR X'55'. */
    size_t enc = cobs_encode(&out[idx], msdu, msdu_len, MSTP_COBS_MASK);
    if (enc == 0) {
        return 0;
    }
    idx += enc;

    /*
     * 9.10.2 (3)(4)(5): init CRC32K to X'FFFFFFFF', accumulate over the
     * *Encoded Data* field, take the ones' complement, order LSB first.
     */
    uint32_t crc = mstp_data_crc32k(&out[8], (uint16_t)enc);
    uint8_t crc_le[4] = {
        (uint8_t)(crc), (uint8_t)(crc >> 8),
        (uint8_t)(crc >> 16), (uint8_t)(crc >> 24)
    };

    /* 9.10.2 (6): COBS-encode the CRC; always five octets. */
    size_t enc_crc = cobs_encode(&out[idx], crc_le, sizeof(crc_le), MSTP_COBS_MASK);
    if (enc_crc != MSTP_ENCODED_CRC_LEN) {
        return 0;
    }
    idx += enc_crc;

    /* 9.10.2 (2): Length = Encoded Data + Encoded CRC-32K - 2 = enc + 3. */
    uint16_t length = (uint16_t)(enc + 3U);
    if ((length < MSTP_NMIN_COBS_LENGTH) || (length > MSTP_NMAX_COBS_LENGTH)) {
        return 0;
    }

    out[0] = MSTP_PREAMBLE_1;
    out[1] = MSTP_PREAMBLE_2;

    uint8_t hdr[5] = {
        MSTP_FRAME_TYPE_IPV6, dst, src,
        (uint8_t)(length >> 8), (uint8_t)(length & 0xFFU)
    };
    out[2] = hdr[0];
    out[3] = hdr[1];
    out[4] = hdr[2];
    out[5] = hdr[3];
    out[6] = hdr[4];
    out[7] = mstp_header_crc8(hdr, sizeof(hdr));

    return idx;
}
