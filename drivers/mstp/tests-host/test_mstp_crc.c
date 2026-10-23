/* Host test: table-driven CRC-32K == the draft's bitwise reference. */
#include <stdio.h>
#include <stdlib.h>
#include "mstp_crc.h"

int main(void)
{
    unsigned fails = 0;
    uint32_t s = 0x12345678U;
    /* every octet against 4096 pseudo-random CRC states (+ the fixed points) */
    for (unsigned k = 0; k < 4096; k++) {
        s = s * 1664525U + 1013904223U;
        uint32_t c = (k == 0) ? CRC32K_INITIAL_VALUE : (k == 1) ? 0U : s;
        for (unsigned d = 0; d < 256; d++) {
            if (calc_crc32K((uint8_t)d, c) != calc_crc32K_ref((uint8_t)d, c)) { fails++; }
        }
    }
    /* buffers: update() == octet-wise reference; residue check over data + CRC */
    static uint8_t buf[1600];
    for (unsigned len = 1; len <= 1520; len += 37) {
        for (unsigned i = 0; i < len; i++) { s = s * 1664525U + 1013904223U; buf[i] = (uint8_t)(s >> 24); }
        uint32_t ref = CRC32K_INITIAL_VALUE;
        for (unsigned i = 0; i < len; i++) { ref = calc_crc32K_ref(buf[i], ref); }
        uint32_t tx = mstp_data_crc32k(buf, (uint16_t)len);
        if (tx != ~ref) { fails++; }
        buf[len] = (uint8_t)tx; buf[len+1] = (uint8_t)(tx >> 8);
        buf[len+2] = (uint8_t)(tx >> 16); buf[len+3] = (uint8_t)(tx >> 24);
        if (mstp_crc32k_update(CRC32K_INITIAL_VALUE, buf, len + 4) != CRC32K_RESIDUE) { fails++; }
    }
    printf("test_mstp_crc: %s (%u failures)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
