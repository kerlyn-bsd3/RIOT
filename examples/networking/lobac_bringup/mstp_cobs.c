/*
 * COBS encoder/decoder for MS/TP.
 *
 * Reference implementation from draft-lynn-6lo-rfc8163-bis-01 (K. Lynn),
 * Appendix "COBS functions". IETF Trust / Simplified BSD License.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mstp_cobs.h"

/*
 * Encodes 'length' octets of client data located at 'from' and
 * writes the COBS-encoded result at 'to', XOR'ing each output
 * octet with 'mask'. Returns the length of the encoded data.
 */
size_t cobs_encode(uint8_t *to, const uint8_t *from, size_t length, uint8_t mask)
{
    size_t code_index = 0;
    size_t read_index = 0;
    size_t write_index = 1;
    uint8_t code = 1;
    uint8_t data, last_code = 0;

    while (read_index < length) {
        data = from[read_index++];
        /*
         * In the case of encountering a non-zero octet in the data,
         * simply copy input to output and increment the code octet.
         */
        if (data != 0) {
            to[write_index++] = data ^ mask;
            code++;
            if (code != 255) {
                continue;
            }
        }
        /*
         * In the case of encountering a zero in the data or having
         * copied the maximum number (254) of non-zero octets, store
         * the code octet and reset the encoder state.
         */
        last_code = code;
        to[code_index] = code ^ mask;
        code_index = write_index++;
        code = 1;
    }
    /*
     * If the last chunk contains exactly 254 non-zero octets, then
     * this exception is handled above (and the returned length must
     * be adjusted). Otherwise, encode the final code octet.
     */
    if ((last_code == 255) && (code == 1)) {
        write_index--;
    }
    else {
        to[code_index] = code ^ mask;
    }

    return write_index;
}

/*
 * Decodes 'length' octets of data located at 'from' and
 * writes the original client data at 'to', restoring any
 * 'mask' octets that may present in the encoded data.
 * Returns the length of the encoded data or zero if error.
 *
 * [NOTE, not in the draft: the return value is the length of the
 *  *decoded* data (write_index). See the editorial nit reported
 *  against draft-lynn-6lo-rfc8163-bis-01.]
 */
size_t cobs_decode(uint8_t *to, const uint8_t *from, size_t length, uint8_t mask)
{
    size_t read_index = 0;
    size_t write_index = 0;
    uint8_t code, last_code;

    while (read_index < length) {
        code = from[read_index] ^ mask;
        last_code = code;
        /*
         * Sanity check the encoding to prevent the while() loop below
         * from overrunning the output buffer. See [Err5996].
         */
        if (code == 0 || read_index + code > length) {
            return 0;
        }

        read_index++;
        while (--code > 0) {
            to[write_index++] = from[read_index++] ^ mask;
        }
        /*
         * Restore the implicit zero at the end of each decoded block
         * except when it contains exactly 254 non-zero octets or the
         * end of the encoded data has been reached.
         */
        if ((last_code != 255) && (read_index < length)) {
            to[write_index++] = 0;
        }
    }

    return write_index;
}
