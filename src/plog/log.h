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

#ifndef _LOG_H_
#define _LOG_H_

#include <stdint.h>

typedef uint16_t float16_t;

#define FCS_LOG_MIN_LENGTH 5u
#define FCS_LOG_MAX_LENGTH 248u
#define FCS_LOG_SERIALIZED_LENGTH 256u

enum fcs_log_type_t {
    FCS_LOG_TYPE_INVALID,
    FCS_LOG_TYPE_MEASUREMENT,
    FCS_LOG_TYPE_SENSOR_HAL,
    FCS_LOG_TYPE_ESTIMATE,
    FCS_LOG_TYPE_CONTROL,
    FCS_LOG_TYPE_COMBINED,
    FCS_LOG_TYPE_LAST
};

/*
Binary log packet (all multi-byte values are LE):
1 byte type
2 bytes reserved (0)
2 bytes frame index
... (sensor readings -- fcs_measurement_t)
4 bytes CRC32
*/
struct fcs_log_t {
    uint32_t canary1;
    size_t length;  /* excludes the size of the CRC32 (4 bytes) */
    uint8_t data[FCS_LOG_MAX_LENGTH + 4u];
    uint32_t canary2;
};

/* Initialize a log packet with a type and packet index */
void fcs_log_init(struct fcs_log_t *plog, enum fcs_log_type_t type,
uint16_t frame_id);

/*
Serialize and add COBS-R + framing to log packet, and copy the result to
`out_buf`. Returns the length of the serialized data.

Modifies `log` to include a CRC32.
*/
size_t fcs_log_serialize(uint8_t *out_buf, size_t out_buf_len,
struct fcs_log_t *plog);

/* Deserialize a log frame */
bool fcs_log_deserialize(struct fcs_log_t *plog, const uint8_t *in_buf,
size_t in_buf_len);

#endif
