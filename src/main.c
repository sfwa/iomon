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

int main(void) {
	/* Initialize core systems */
    sysclk_init();
	sleepmgr_init();

    irq_initialize_vectors();
    cpu_irq_disable();

    /* Initialize the WDT as early as possible; initially set timeout to 100ms */
    wdt_opt_t wdt_options = {
        .us_timeout_period = 100000u, /* us */
        .us_timeban_period = 0,
        .cssel = WDT_CLOCK_SOURCE_SELECT_RCSYS,
        .fcd = 1u, /* don't re-calibrate on watchdog reset */
        .sfv = 0, /* if set to 1, the WDT is locked out completely */
        .mode = 0, /* no timeban */
        .dar = 1u /* disable WDT on reset, to prevent runaway reset loops */
    };
#ifdef CONTINUE_ON_ASSERT
    wdt_enable(&wdt_options);
#endif

    /* Initialize the core iomon systems */
    comms_init();

    wdt_clear();

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

    wdt_clear();

    wdt_options.us_timeout_period = 5000u;
    wdt_options.sfv = 1u; /* don't allow any further changes */
#ifdef CONTINUE_ON_ASSERT
    wdt_disable();
    wdt_clear();
    wdt_enable(&wdt_options);
#endif

    cpu_irq_enable();

    uint32_t counts_per_ms, frame;
    counts_per_ms = sysclk_get_cpu_hz() / 1000u;
    frame = Get_system_register(AVR32_COUNT) / counts_per_ms;
    while (true) {
        /* Clear the watchdog timer */
        wdt_clear();

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
        uint32_t start_t = frame * counts_per_ms;
        comms_set_cpu_status(Get_system_register(AVR32_COUNT) - start_t);
        frame++;

        while ((Get_system_register(AVR32_COUNT) - start_t) < counts_per_ms) {
            /* FIXME: use udelay instead of busy loop */
            cpu_relax();
        }

        if ((Get_system_register(AVR32_COUNT) - start_t) > 8u * counts_per_ms / 7u) {
            /* Lost an entire frame! */
            fcs_assert(false);
        }

        /* Transmit the CPU packet at ~exactly the same time each frame */
        comms_start_transmit();
    }
}
