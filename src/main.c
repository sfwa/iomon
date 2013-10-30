/**
* \file
*
* \brief CDC Application Main functions
*
* Copyright (c) 2011-2012 Atmel Corporation. All rights reserved.
*
* \asf_license_start
*
* \page License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. The name of Atmel may not be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* 4. This software may only be redistributed and used in connection with an
*    Atmel microcontroller product.
*
* THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
* EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \asf_license_stop
*
*/

#include <asf.h>
#include "conf_usb.h"
#include "ui.h"
#include "comms.h"
#include "gp.h"
#include "pwm.h"
#include "venus638_gps.h"
/* #include "ubx_gps.h" */
#include "ms5611.h"
#include "hmc5883.h"
#include "mpu6050.h"

static volatile bool main_b_cdc_enable = false;

int main(void) {
    irq_initialize_vectors();
    cpu_irq_enable();

    /* Initialize core systems */
    sleepmgr_init();
    sysclk_init();

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
    gps_init();
	ui_init();

	wdt_clear();

	/* Start USB stack to authorize VBus monitoring */
	udc_start();

    wdt_options.us_timeout_period = 5000u;
    wdt_options.sfv = 1u; /* don't allow any further changes */
#ifdef CONTINUE_ON_ASSERT
    wdt_disable();
    wdt_clear();
    wdt_enable(&wdt_options);
#endif

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

        if ((Get_system_register(AVR32_COUNT) - start_t) > 5u * counts_per_ms / 4u) {
            /* Lost an entire frame! */
            Assert(false);
        }
    }
}

void main_vbus_action(bool b_high) {
    if (b_high) {
        /* Attach USB Device */
        udc_attach();
    } else {
        /* VBUS not present */
        udc_detach();
    }
}

void main_suspend_action(void) {
    ui_powerdown();
}

void main_resume_action(void) {
    ui_wakeup();
}

void main_sof_action(void) {
    if (!main_b_cdc_enable) {
        return;
    }
    ui_process(udd_get_frame_number());
}

bool main_cdc_enable(void) {
    main_b_cdc_enable = true;
    comms_enable_usb();

    return true;
}

void main_cdc_disable(void) {
    main_b_cdc_enable = false;
    comms_disable_usb();
}

void main_cdc_set_dtr(bool b_enable) {
    if (b_enable) {
        /* Host terminal has opened COM */
        ui_com_open();
    } else {
        /* Host terminal has closed COM */
        ui_com_close();
    }
}

void main_usb_rx_notify(void) {

}

void main_usb_config(void* arg) {

}
