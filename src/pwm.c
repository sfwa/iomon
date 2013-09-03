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
#include <avr32/io.h>
#include <string.h>
#include "comms.h"
#include "pwm.h"

static uint32_t pwm_values[PWM_NUM_OUTPUTS];
static bool pwm_is_enabled = false;

void pwm_init(void) {
    /* Configure PWM enable */
    gpio_configure_pin(PWM_ENABLE_PIN, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

    pwm_disable();

    /* Set GPIO mapping, enable PWM clock */
    gpio_enable_module_pin(PWM_0_PIN, PWM_0_FUNCTION);
    gpio_enable_module_pin(PWM_1_PIN, PWM_1_FUNCTION);
    gpio_enable_module_pin(PWM_2_PIN, PWM_2_FUNCTION);
    gpio_enable_module_pin(PWM_3_PIN, PWM_3_FUNCTION);
    sysclk_enable_pba_module(PWM_SYSCLK);

    /* Initialize PWM frequency and mode */
    volatile avr32_pwm_t *pwm = &AVR32_PWM;
    pwm->idr1 = 0xFFFFFFFFu;
    pwm->isr1;
    pwm->idr2 = 0xFFFFFFFFu;
    pwm->isr2;

    /* Set PWM mode register. */
    pwm->clk =
        (1u << AVR32_PWM_DIVA_OFFSET) | /* run CLKA at full CPU clock speed
                                           (48MHz) */
        (0 << AVR32_PWM_DIVB_OFFSET) |  /* disable CLKB */
        (0 << AVR32_PWM_PREA_OFFSET) |  /* no pre-division */
        (0 << AVR32_PWM_PREB_OFFSET) |
        (0 << AVR32_PWM_CLKSEL_OFFSET); /* use CLK_PWM as source */

    /* Delay for at least 2 clock periods (datasheet 33.6.1) */
    for (volatile uint8_t z = 0; z < 5u; z++);

    /* Disable all channels */
    pwm->dis = 0x0fu;

    /* Set all PWM channels to be synchronous */
    pwm->SCM.updm = 0;
    pwm->scm |= (0xfu << AVR32_PWM_SCM_SYNC0_OFFSET);

    for (uint8_t i = 0; i < PWM_NUM_OUTPUTS; i++) {
        /* Set polarity so cycle starts high. */
        pwm->channel[i].cmr = AVR32_PWM_CPOL_MASK;
        /* 1.5ms duty cycle */
        pwm->channel[i].cdtyupd = 75497u;
        /* 48Hz period */
        pwm->channel[i].cprdupd = 1048575u;
    }

    /* Write the channel value update */
    pwm->SCUC.updulock = 1u;

    /* Enable all channels */
    pwm->ena = 0x0fu;
}

void pwm_tick(void) {
    uint16_t pwm_values[PWM_NUM_OUTPUTS];
    bool got_pwm_values = false;
    for (uint8_t i = 0; i < PWM_NUM_OUTPUTS; i++) {
        pwm_values[i] = comms_get_pwm(i);
        if (!got_pwm_values && pwm_values[i]) {
            got_pwm_values = true;
        }
    }

	if (got_pwm_values) {
		pwm_set_values(pwm_values);
	}

    /* Enable PWM if it isn't already, and we've received at least some data
       from the CPU (otherwise the values would still be 0) */
    if (!pwm_is_enabled && got_pwm_values) {
        pwm_enable();
    }
}

void pwm_set_values(uint16_t pwms[PWM_NUM_OUTPUTS]) {
    volatile avr32_pwm_t *pwm = &AVR32_PWM;
    bool updated = false;

    /* Make sure the previous update has been applied */
    if (!pwm->SCUC.updulock) {
        for (uint8_t i = 0; i < PWM_NUM_OUTPUTS; i++) {
            /* Channel period is 1048576 cycles long, at 48Hz; PWM pulse width
               is 1-2ms. A PWM value in the range 42729-108264 means the pulse
               range is 0.85-2.15ms. */
            uint32_t pwm_val = (uint32_t)pwms[i] + 42729u;
            if (pwm_val != pwm_values[i]) {
                /* change PWM duty cycle for the current channel (20-bit) */
                pwm->channel[i].cdtyupd = pwm_val & 0x000fffffu;
                pwm->channel[i].cprdupd = 1048575u;

                pwm_values[i] = pwm_val;
                updated = true;
            }
        }

        if (updated) {
            pwm->SCUC.updulock = 1u;
        }
    }
}

void pwm_enable(void) {
    gpio_local_set_gpio_pin(PWM_ENABLE_PIN);
    LED_OFF(LED2_GPIO);

    pwm_is_enabled = true;
}

void pwm_disable(void) {
    gpio_local_clr_gpio_pin(PWM_ENABLE_PIN);
    memset(pwm_values, 0, sizeof(pwm_values));
    LED_ON(LED2_GPIO);

    pwm_is_enabled = false;
}
