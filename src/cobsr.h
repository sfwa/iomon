/*
----------------------------------------------------------------------------
Copyright (c) 2010 Craig McQueen

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
----------------------------------------------------------------------------
*/

/*****************************************************************************
 *
 * cobsr.h
 *
 * Consistent Overhead Byte Stuffing--Reduced (COBS/R)
 *
 ****************************************************************************/

#ifndef COBSR_H_
#define COBSR_H_

#include <stdint.h>

enum cobsr_encode_status{
    COBSR_ENCODE_OK                     = 0x00u,
    COBSR_ENCODE_NULL_POINTER           = 0x01u,
    COBSR_ENCODE_OUT_BUFFER_OVERFLOW    = 0x02u
};

struct cobsr_encode_result {
    uint32_t              out_len;
    enum cobsr_encode_status status;
};


enum cobsr_decode_status {
    COBSR_DECODE_OK                     = 0x00u,
    COBSR_DECODE_NULL_POINTER           = 0x01u,
    COBSR_DECODE_OUT_BUFFER_OVERFLOW    = 0x02u,
    COBSR_DECODE_ZERO_BYTE_IN_INPUT     = 0x04u,
};

struct cobsr_decode_result {
    uint32_t              out_len;
    enum cobsr_decode_status status;
};

struct cobsr_encode_result cobsr_encode(uint8_t *dst_buf_ptr,
uint32_t dst_buf_len, const uint8_t * src_ptr, uint32_t src_len);
struct cobsr_decode_result cobsr_decode(uint8_t *dst_buf_ptr,
uint32_t dst_buf_len, const uint8_t * src_ptr, uint32_t src_len);

#endif
