/*
Copyright (C) 2013 Ben Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <asf.h>
#include "dfu.h"

/* Config word 1 (flash 0x808001FC) is set up as follows:
7:0 ISP_CRC8_1 -- config word 1 CRC 0x107 (x^8+x^2+x+1)
8 ISP_IO_COND_EN -- set to 0 to ignore external DFU triggers
9 ISP_FORCE -- set to 1 to force DFU mode on startup
15:10 RESERVED -- set to all 1s
31:16 ISP_BOOT_KEY_1 -- set to 0xE11E */

#define BOOTLOADER_1_1_CONFIGWORD1_ISP_EN 0xE11EFED0u
#define BOOTLOADER_1_1_CONFIGWORD1_ISP_DIS 0xE11EFCDEu
#define BOOTLOADER_1_1_CONFIGWORD2 0x929E0E62u

/* Address of configuration word */
volatile uint32_t *configword1 = (uint32_t*)0x808001FC;
volatile uint32_t *configword2 = (uint32_t*)0x808001F8;

void dfu_enable(void) {
    Assert(flashc_erase_user_page(true));
    flashc_memset32(configword1, BOOTLOADER_1_1_CONFIGWORD1_ISP_EN, 4, true);
    flashc_memset32(configword2, BOOTLOADER_1_1_CONFIGWORD2, 4, true);
}
