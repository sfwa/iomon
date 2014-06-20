/*
Copyright (C) 2013 Ben Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "fcsassert.h"
#include "log.h"
#include "cobsr.h"
#include "crc32.h"

/* Initialize a log packet with a type and packet index */
void fcs_log_init(struct fcs_log_t *plog, enum fcs_log_type_t type,
uint16_t frame_id) {
    fcs_assert(plog);
    fcs_assert(((uint32_t)plog & 0x3) == 0);
    fcs_assert(type > FCS_LOG_TYPE_INVALID);
    fcs_assert(type < FCS_LOG_TYPE_LAST);

    plog->canary1 = 0xdeadbeefu;
    plog->length = FCS_LOG_MIN_LENGTH;
    plog->data[0] = (uint8_t)type;
    plog->data[1] = 0;
    plog->data[2] = 0;
    plog->data[3] = (frame_id >> 0u) & 0xFFu;
    plog->data[4] = (frame_id >> 8u) & 0xFFu;
    plog->canary2 = 0xfeedfaceu;
}

/*
Serialize and add COBS-R + framing to log packet, and copy the result to
`out_buf`. Returns the length of the serialized data.

Modifies `log` to include a CRC32.
*/
size_t fcs_log_serialize(uint8_t *out_buf, size_t out_buf_length,
struct fcs_log_t *plog) {
    fcs_assert(out_buf);
    fcs_assert(out_buf_length);
    fcs_assert(plog);
    fcs_assert(((uint32_t)plog & 0x3) == 0);
    fcs_assert(FCS_LOG_MIN_LENGTH <= plog->length &&
               plog->length <= FCS_LOG_MAX_LENGTH);
    fcs_assert(plog->data[0] > (uint8_t)FCS_LOG_TYPE_INVALID);
    fcs_assert(plog->data[0] < (uint8_t)FCS_LOG_TYPE_LAST);
    fcs_assert(plog->canary1 == 0xdeadbeefu && plog->canary2 == 0xfeedfaceu);

    /*
    4 bytes for CRC, 2 + ceil((len+4) / 256) bytes for COBS-R + NUL start/end
    */
    fcs_assert(out_buf_length >= plog->length + 4u + 2u +
                                 ((plog->length + 259u) >> 8u));

    /* Calculate checksum and update the packet with the result */
    uint32_t crc = fcs_crc32(plog->data, plog->length, 0xFFFFFFFFu);
    plog->data[plog->length + 0u] = (crc >> 0u) & 0xFFu;
    plog->data[plog->length + 1u] = (crc >> 8u) & 0xFFu;
    plog->data[plog->length + 2u] = (crc >> 16u) & 0xFFu;
    plog->data[plog->length + 3u] = (crc >> 24u) & 0xFFu;

    /* Write COBS-R encoded result to out_buf */
    struct cobsr_encode_result result;
    result = cobsr_encode(&out_buf[1], out_buf_length - 2u, plog->data,
                          plog->length + 4u);
    fcs_assert(result.status == COBSR_ENCODE_OK);

    /* Add NUL start/end bytes */
    out_buf[0] = 0;
    out_buf[result.out_len + 1u] = 0;

    /* Return the COBS-R encoded length, plus the length of the NUL bytes */
    fcs_assert(result.out_len > 0 && (size_t)result.out_len < SIZE_MAX - 2u);
    return (size_t)(result.out_len + 2u);
}

/* Deserialize a log frame */
bool fcs_log_deserialize(struct fcs_log_t *plog, const uint8_t *in_buf,
size_t in_buf_len) {
    fcs_assert(plog);
    fcs_assert(((uint32_t)plog & 0x3) == 0);
    fcs_assert(in_buf);
    fcs_assert(in_buf_len);

    uint32_t crc, packet_crc;
    struct cobsr_decode_result result;

    /* Validate buffer length */
    if (in_buf_len < 9u || in_buf_len > FCS_LOG_MAX_LENGTH + 10u) {
        goto invalid;
    }

    result = cobsr_decode(plog->data, FCS_LOG_MAX_LENGTH + 4u, &in_buf[1],
                          in_buf_len - 2u);
    if (result.status != COBSR_DECODE_OK || result.out_len < 9u ||
            result.out_len - 4u > FCS_LOG_MAX_LENGTH) {
        goto invalid;
    }

    plog->length = (size_t)result.out_len - 4u;

    crc = fcs_crc32(plog->data, plog->length, 0xFFFFFFFFu);
    packet_crc = (uint32_t)plog->data[plog->length + 0u] +
                 ((uint32_t)plog->data[plog->length + 1u] << 8u) +
                 ((uint32_t)plog->data[plog->length + 2u] << 16u) +
                 ((uint32_t)plog->data[plog->length + 3u] << 24u);

    if (crc != packet_crc) {
        goto invalid;
    }

    plog->canary1 = 0xdeadbeefu;
    plog->canary2 = 0xfeedfaceu;

    return true;

invalid:
    /*
    Since the log is invalid, set the length to a value which will cause an
    assertion failure as quickly as possible if the caller doesn't check our
    return value;
    */
    plog->length = 0xFFFFFFFFu;
    return false;
}
