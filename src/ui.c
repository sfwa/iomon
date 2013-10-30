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
#include "ui.h"

/*
* - Led 0 is on when USB line is in IDLE mode, and blinks while COM open.
* - Led 1 is on until all peripherals are initialized, then stays off
* - Led 2 is on during PWM disable conditions (e.g. asserts, or no CPU)
* - Led 3 is on during assert conditions
*/

void ui_init(void) {
    LED_OFF(LED0_GPIO);
	LED_OFF(LED3_GPIO);
}

void ui_powerdown(void) {
    ui_init();
}

void ui_wakeup(void) {

}

void ui_com_open(void) {
    LED_ON(LED0_GPIO);
}

void ui_com_close(void) {
    LED_OFF(LED0_GPIO);
}

void ui_process(uint16_t framenumber) {
    framenumber %= 500u;
    if (framenumber == 0) {
        LED_ON(LED0_GPIO);
    } else if (framenumber == 250u) {
        LED_OFF(LED0_GPIO);
    }
}
