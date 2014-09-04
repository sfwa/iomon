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
#include "fcsassert.h"
#include "comms.h"
#include "pwm.h"
#include "plog/parameter.h"

#define PWM_THROTTLE_FAILSAFE_THRESHOLD 15000u
#define PWM_INTERNAL_TO_EXTERNAL_THRESHOLD 20000u
#define PWM_EXTERNAL_TO_INTERNAL_THRESHOLD 40000u
#define PWM_TRANSITION_PULSE_COUNT 5u
#define PWM_TRIM_MEASUREMENT_TICKS 5000u


#define PWM_FAILSAFE_INTERNAL_TICKS 50u
#define PWM_FAILSAFE_EXTERNAL_TICKS 1500u


static uint32_t pwm_out_values[PWM_NUM_OUTPUTS];
static uint16_t pwm_trim_offsets[PWM_NUM_OUTPUTS];
static bool pwm_is_enabled = false;
static bool pwm_use_internal = false;
static uint32_t pwm_transition_pulses = 0;
static uint32_t pwm_missed_internal_ticks = 0;
static uint32_t pwm_missed_external_ticks = 0;
static uint32_t pwm_trim_measurement_ticks;

static volatile uint32_t pwm_input_state_begin[PWM_NUM_INPUTS];
static volatile uint16_t pwm_input_next[PWM_NUM_INPUTS];
static uint16_t pwm_input_values[PWM_NUM_INPUTS];
static const uint32_t pwm_input_pins[PWM_NUM_INPUTS] =
    {PWM_IN_0_PIN, PWM_IN_1_PIN, PWM_IN_2_PIN, PWM_IN_3_PIN};


/* Interrupt handler for PWM input */
__attribute__((__interrupt__))
static void pwm_input_interrupt_handler(void) {
    const uint32_t port_idx = PWM_IN_0_PIN >> 5u;
    uint32_t cycle_count, i, delta, ifr;

    ifr = AVR32_GPIO.port[port_idx].ifr;
    cycle_count = Get_system_register(AVR32_COUNT);

    /*
    Check PWM input pins; if there's a state change, reset the current count.
    Normally called from interrupt.
    */
    for (i = 0; i < PWM_NUM_INPUTS; i++) {
        if ((ifr >> (pwm_input_pins[i] & 0x1Fu)) & 1u) {
            /*
            If the current pin state is low, update the PWM value according to
            the delta.
            */
            if (!gpio_local_get_pin_value(pwm_input_pins[i])) {
                delta = cycle_count - pwm_input_state_begin[i];

                /*
                Now we have number of cycles the input was high for; we
                need a mapping from 0.85-2.15ms to the range [0, 65535].
                */
                if (delta <= 42829u) {
                    pwm_input_next[i] = 0;
                } else if (delta >= 108264u) {
                    pwm_input_next[i] = 65535u;
                } else {
                    pwm_input_next[i] = (delta - 42829u) & 0xffffu;
                }
            }

            pwm_input_state_begin[i] = cycle_count;
        }
    }

	AVR32_GPIO.port[port_idx].ifrc = ifr;
    AVR32_GPIO.port[port_idx].ifr;
}

void pwm_init(void) {
    /* Configure PWM enable */
    gpio_configure_pin(PWM_ENABLE_PIN, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

    /* Set GPIO mapping, enable PWM clock */
    gpio_enable_module_pin(PWM_0_PIN, PWM_0_FUNCTION);
    gpio_enable_module_pin(PWM_1_PIN, PWM_1_FUNCTION);
    gpio_enable_module_pin(PWM_2_PIN, PWM_2_FUNCTION);
    gpio_enable_module_pin(PWM_3_PIN, PWM_3_FUNCTION);

    /* Set internal mode */
    pwm_transition_pulses = 0;
    pwm_use_internal = false;
    pwm_missed_internal_ticks = 0;
    pwm_missed_external_ticks = 0;
    pwm_trim_measurement_ticks = 0;

    /* Enable PWM input interrupts */
	gpio_configure_pin(pwm_input_pins[0], GPIO_DIR_INPUT | GPIO_PULL_DOWN);
	gpio_configure_pin(pwm_input_pins[1], GPIO_DIR_INPUT | GPIO_PULL_DOWN);
	gpio_configure_pin(pwm_input_pins[2], GPIO_DIR_INPUT | GPIO_PULL_DOWN);
	gpio_configure_pin(pwm_input_pins[3], GPIO_DIR_INPUT | GPIO_PULL_DOWN);

    cpu_irq_disable();
    INTC_init_interrupts();
    INTC_register_interrupt(&pwm_input_interrupt_handler,
                            AVR32_GPIO_IRQ_0 + pwm_input_pins[0] / 8,
                            AVR32_INTC_INT0);
    INTC_register_interrupt(&pwm_input_interrupt_handler,
                            AVR32_GPIO_IRQ_0 + pwm_input_pins[3] / 8,
                            AVR32_INTC_INT0);
    gpio_enable_pin_interrupt(pwm_input_pins[0], GPIO_PIN_CHANGE);
    gpio_enable_pin_interrupt(pwm_input_pins[1], GPIO_PIN_CHANGE);
    gpio_enable_pin_interrupt(pwm_input_pins[2], GPIO_PIN_CHANGE);
    gpio_enable_pin_interrupt(pwm_input_pins[3], GPIO_PIN_CHANGE);
    cpu_irq_enable();

    /* Initialize PWM frequency and mode */
    AVR32_PWM.idr1 = 0xFFFFFFFFu;
    AVR32_PWM.isr1;
    AVR32_PWM.idr2 = 0xFFFFFFFFu;
    AVR32_PWM.isr2;

    /* Set PWM mode register. */
    AVR32_PWM.clk =
        (1u << AVR32_PWM_DIVA_OFFSET) | /* run CLKA at full CPU clock speed
                                           (52MHz) */
        (0 << AVR32_PWM_DIVB_OFFSET) |  /* disable CLKB */
        (0 << AVR32_PWM_PREA_OFFSET) |  /* no pre-division */
        (0 << AVR32_PWM_PREB_OFFSET) |
        (0 << AVR32_PWM_CLKSEL_OFFSET); /* use CLK_PWM as source */

    /* Delay for at least 2 clock periods (datasheet 33.6.1) */
    volatile uint8_t z;
    for (z = 0; z < 5u; z++);

    /* Disable all channels */
    AVR32_PWM.dis = 0x0fu;

    for (z = 0; z < 5u; z++);

    /* Set all PWM channels to be synchronous */
    AVR32_PWM.SCM.updm = 0;
    AVR32_PWM.scm |= (0xfu << AVR32_PWM_SCM_SYNC0_OFFSET);

    for (uint8_t i = 0; i < PWM_NUM_OUTPUTS; i++) {
        /* Set polarity so cycle starts high. */
        AVR32_PWM.channel[i].cmr = AVR32_PWM_CPOL_MASK;
        /* 1.5ms duty cycle */
        AVR32_PWM.channel[i].cdtyupd = 75497u;
        /* 48Hz period */
        AVR32_PWM.channel[i].cprdupd = 1048575u;
    }

    /* Write the channel value update */
    AVR32_PWM.SCUC.updulock = 1u;

    /* Enable all channels */
    AVR32_PWM.ena = 0x0fu;
}

void pwm_tick(void) {
    static uint16_t out_values[PWM_NUM_OUTPUTS];
    struct fcs_parameter_t param;

    /* Get the latest PWM input values without being interrupted */
    cpu_irq_disable();
    pwm_input_values[0] = pwm_input_next[0];
    pwm_input_values[1] = pwm_input_next[1];
    pwm_input_values[2] = pwm_input_next[2];
    pwm_input_values[3] = pwm_input_next[3];
    cpu_irq_enable();

    /* Handle internal/external control and failsafe logic */
    if (pwm_use_internal) {
        if (fcs_parameter_find_by_type_and_device(
                &cpu_conn.in_log, FCS_PARAMETER_CONTROL_SETPOINT, 0, &param)){
            out_values[0] = pwm_trim_offsets[0] + swap_u16(param.data.u16[0]);
            out_values[1] = (pwm_trim_offsets[1] - 32767) +
                            swap_u16(param.data.u16[1]);
            out_values[2] = (pwm_trim_offsets[2] - 32767) +
                            (65535 - swap_u16(param.data.u16[2]));

            pwm_set_values(out_values);
            pwm_missed_internal_ticks = 0;
        } else {
            /* Retain previous output values */
            pwm_missed_internal_ticks++;
        }

        if (pwm_input_values[3] < PWM_INTERNAL_TO_EXTERNAL_THRESHOLD) {
            pwm_transition_pulses++;
        } else {
            pwm_transition_pulses = 0;
        }

        if (pwm_missed_internal_ticks > PWM_FAILSAFE_INTERNAL_TICKS) {
            pwm_terminate_flight();
        }
    } else {
        out_values[0] = pwm_input_values[0];
        out_values[1] = pwm_input_values[1];
        out_values[2] = pwm_input_values[2];

        /*
        Detect R/C failsafe condition -- based on simultaneous throttle
        off and switch to auto.
        */
        if (pwm_input_values[3] > PWM_EXTERNAL_TO_INTERNAL_THRESHOLD) {
            if (pwm_input_values[0] < PWM_THROTTLE_FAILSAFE_THRESHOLD) {
                pwm_missed_external_ticks++;
                pwm_transition_pulses = 0;
            } else {
                pwm_transition_pulses++;
                pwm_missed_external_ticks = 0;
            }
        } else {
            pwm_transition_pulses = 0;
            pwm_missed_external_ticks = 0;
        }

        if (pwm_missed_external_ticks > PWM_FAILSAFE_EXTERNAL_TICKS) {
            pwm_terminate_flight();
        }
    }

    if (pwm_trim_measurement_ticks < PWM_TRIM_MEASUREMENT_TICKS) {
        pwm_trim_offsets[1] +=
            (pwm_input_values[1] - pwm_trim_offsets[1]) >> 2u;
        pwm_trim_offsets[2] +=
            (pwm_input_values[2] - pwm_trim_offsets[2]) >> 2u;
        pwm_trim_measurement_ticks++;
    }

    /*
    We have to call pwm_enable frequently to keep the hardware watchdog
    going.
    */
    pwm_enable();

    /* Output PWM values and send the current positions to the log */
    pwm_set_values(out_values);

    fcs_parameter_set_header(&param, FCS_VALUE_UNSIGNED, 16u, 3u);
    fcs_parameter_set_type(&param, FCS_PARAMETER_CONTROL_POS);
    fcs_parameter_set_device_id(&param, 0);
    param.data.u16[0] = swap_u16(out_values[0] - pwm_trim_offsets[0]);
    param.data.u16[1] = swap_u16(out_values[1] -
                               (pwm_trim_offsets[1] - 32767u));
    param.data.u16[2] = 65535u - swap_u16(out_values[2] -
                                       (pwm_trim_offsets[2] - 32767u));
    (void)fcs_log_add_parameter(&cpu_conn.out_log, &param);

    /* Output control mode -- 1 for internal, 0 for external (R/C) */
    fcs_parameter_set_header(&param, FCS_VALUE_UNSIGNED, 8u, 1u);
    fcs_parameter_set_type(&param, FCS_PARAMETER_CONTROL_MODE);
    fcs_parameter_set_device_id(&param, 0);
    param.data.u8[0] = pwm_use_internal ? 1u : 0;
    (void)fcs_log_add_parameter(&cpu_conn.out_log, &param);

    /*
    Handle internal <-> external control transition -- wait for
    PWM_TRANSITION_PULSE_COUNT pulses higher/lower than the threshold.
    */
    if (pwm_transition_pulses >= PWM_TRANSITION_PULSE_COUNT) {
        pwm_use_internal = !pwm_use_internal;
    }
}

void pwm_set_values(uint16_t pwms[PWM_NUM_OUTPUTS]) {
    size_t i;
    uint32_t pwm_val;
    bool updated = false;

    /* Make sure the previous update has been applied */
    if (!AVR32_PWM.SCUC.updulock) {
        for (i = 0; i < PWM_NUM_OUTPUTS; i++) {
            /*
            Channel period is 1048576 cycles long, at 48Hz; PWM pulse width is
            1-2ms. A PWM value in the range 42729-108264 means the pulse range
            is 0.85-2.15ms.
            */
            pwm_val = (uint32_t)pwms[i] + 42729u;
            if (pwm_val != pwm_out_values[i]) {
                /* Change PWM duty cycle for the current channel (20-bit) */
                AVR32_PWM.channel[i].cdtyupd = pwm_val & 0x000fffffu;
                AVR32_PWM.channel[i].cprdupd = 1048575u;

                pwm_out_values[i] = pwm_val;
                updated = true;
            }
        }

        if (updated) {
            AVR32_PWM.SCUC.updulock = 1u;
        }
    }
}

void pwm_enable(void) {
    gpio_local_tgl_gpio_pin(PWM_ENABLE_PIN);

    pwm_is_enabled = true;
}

void pwm_disable(void) {
    gpio_local_clr_gpio_pin(PWM_ENABLE_PIN);

    pwm_is_enabled = false;
}

void pwm_terminate_flight(void) {
    /*
    Infinite loop to lock out any possibility of recovery -- reset the WDT
    each time as well otherwise the system will restart itself.
    */
    static uint16_t pwm_values[PWM_NUM_OUTPUTS] = {0, 25000u, 25000u, 0};

    LED_ON(LED3_GPIO);

    irqflags_t flags = cpu_irq_save();
    /* Take control of the PWM */
    for (;;) {
		pwm_enable();
        pwm_set_values(pwm_values);

        //pwm_disable();
    }
    cpu_irq_restore(flags);

    LED_OFF(LED3_GPIO);
}
