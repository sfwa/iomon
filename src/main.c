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
#include "fcsassert.h"
#include "comms.h"
#include "peripherals/gp.h"
#include "peripherals/pwm.h"
#include "peripherals/ubx_gps.h"
#include "peripherals/ms5611.h"
#include "peripherals/hmc5883.h"
#include "peripherals/mpu6000.h"
#include "peripherals/ms4525.h"

static void sysclk_init(void);

static void sysclk_init(void) {
    struct pll_config pllcfg;

    cpu_irq_disable();

    if (pll_is_locked(0)) {
        pll_disable(0);
    }

    if (!osc_is_ready(OSC_ID_OSC0)) {
        osc_enable(OSC_ID_OSC0);
        osc_wait_ready(OSC_ID_OSC0);
    }

    pll_config_init(&pllcfg, CONFIG_PLL0_SOURCE, CONFIG_PLL0_DIV,
                    CONFIG_PLL0_MUL);
    pll_enable(&pllcfg, 0);

    while (!pll_is_locked(0));

    /* Set a flash wait state depending on the new cpu frequency. */
    if (CONFIG_MAIN_HZ > 33000000) {
        AVR32_FLASHC.FCR.fws =  1;
    }

    AVR32_PM.unlock = 0xaa000000 | AVR32_PM_MCCTRL;
    AVR32_PM.mcctrl = CONFIG_SYSCLK_SOURCE;

    while (!(AVR32_PM.SR.ckrdy));

    cpu_irq_enable();
}

int main(void) {
	/* Initialize core systems */
    sysclk_init();
	gpio_local_init();

    /* Initialize the core iomon systems */
    comms_init();

    /* Initialize peripheral devices */
    gp_init();
    pwm_init();
    accel_gyro_init();
    barometer_init();
    magnetometer_init();
    pitot_init();
    gps_init();

    /*
    * - Led 0 is on when USB line is in IDLE mode, and blinks while COM open.
    * - Led 1 is on until all peripherals are initialized, then stays off
    * - Led 2 is on during PWM disable conditions (e.g. asserts, or no CPU)
    * - Led 3 is on during assert conditions
    */
    LED_OFF(LED0_GPIO);
    LED_OFF(LED3_GPIO);

    uint32_t counts_per_ms, frame, start_t;
    counts_per_ms = CONFIG_MAIN_HZ / 1000u;
    frame = Get_system_register(AVR32_COUNT) / counts_per_ms;
    while (true) {
		start_t = frame * counts_per_ms;

        /* Input/output procedure */
        gp_tick();

        /* Sensor input procedures */
        accel_gyro_tick();
        barometer_tick();
        magnetometer_tick();
        pitot_tick();
        gps_tick();
        /* Communications input/output */
        comms_tick();
        /* PWM output procedure */
        pwm_tick();

        /* Work out CPU usage for the last frame */
		comms_set_cpu_status(Get_system_register(AVR32_COUNT) - start_t);
        frame++;

        /*
        If we lose an entire frame, skip the next one to avoid compounding the
        issue, and flash LED3 to alert.
        */
        if ((Get_system_register(AVR32_COUNT) - start_t) > counts_per_ms) {
            LED_ON(LED3_GPIO);
        } else {
            LED_OFF(LED3_GPIO);
        }

        while ((Get_system_register(AVR32_COUNT) - start_t) < counts_per_ms) {
            /* FIXME: use udelay instead of busy loop */
            cpu_relax();
        }

		/* Transmit the CPU packet at ~exactly the same time each frame */
        comms_start_transmit();
    }
}
