/*
 * Copyright (c) 2011 Broadcom Corporation
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "crc8.h"

#ifndef Assert
#include <compiler.h>
#include <Assert.h>
#endif

/*
 * crc8_init - fill crc table for given polynomial in regular bit order.
 *
 * table:   table to be filled.
 * polynomial:  polynomial for which table is to be filled.
 */
void crc8_init(uint8_t table[], uint8_t polynomial) {
    uint32_t i, j;
    uint8_t t = 0;

    for (i = 0; i < 256u; i++) {
        t = i;
        for (j = 0; j < 8u; j++) {
            t = (t << 1u) ^ ((t & 0x80u) ? polynomial : 0);
        }
        table[i] = t;
    }
}

/*
 * crc8 - calculate a crc8 over the given input data.
 *
 * table: crc table used for calculation.
 * pdata: pointer to data buffer.
 * nbytes: number of bytes in data buffer.
 * crc: previous returned crc8 value.
 */
uint8_t crc8(const uint8_t table[], const uint8_t const *pdata,
        uint32_t nbytes, uint8_t crc) {
    uint32_t i;

    Assert(nbytes <= 256u);
    Assert(pdata);

    /* loop over the buffer data */
    for (i = 0; i < nbytes; i++) {
        crc = table[(crc ^ pdata[i]) & 0xffu];
    }

    return crc;
}
