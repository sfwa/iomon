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
#include "plog/parameter.h"


#define PWM_INTERNAL_TO_EXTERNAL_THRESHOLD 20000u
#define PWM_EXTERNAL_TO_INTERNAL_THRESHOLD 40000u
#define PWM_TRANSITION_PULSE_COUNT 5u


#define PWM_FAILSAFE_INTERNAL_TICKS 10u
#define PWM_FAILSAFE_EXTERNAL_TICKS 1500u


static uint32_t pwm_out_values[PWM_NUM_OUTPUTS];
static bool pwm_is_enabled = false;
static bool pwm_use_internal = false;
static uint32_t pwm_transition_pulses = 0;
static uint32_t pwm_missed_internal_ticks = 0;
static uint32_t pwm_missed_external_ticks = 0;

static uint8_t pwm_input_state;
static uint32_t pwm_input_state_begin[PWM_NUM_INPUTS];
static uint16_t pwm_input_values[PWM_NUM_INPUTS];
static uint32_t pwm_input_pins[PWM_NUM_INPUTS] =
    {PWM_IN_0_PIN, PWM_IN_1_PIN, PWM_IN_2_PIN, PWM_IN_3_PIN};


static void pwm_check_inputs(void);
static void pwm_terminate_flight(void);

/* Interrupt handler for PWM input */
__attribute__((__interrupt__))
static void pwm_input_interrupt_handler (void)
{
    pwm_check_inputs();

    gpio_clear_pin_interrupt_flag(PWM_IN_0_PIN);
    gpio_clear_pin_interrupt_flag(PWM_IN_1_PIN);
    gpio_clear_pin_interrupt_flag(PWM_IN_2_PIN);
    gpio_clear_pin_interrupt_flag(PWM_IN_3_PIN);
}

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

    /* Set internal mode */
    pwm_transition_pulses = 0;
    pwm_use_internal = false;
    pwm_missed_internal_ticks = 0;
    pwm_missed_external_ticks = 0;

    /* Clear out input states and enable PWM input interrupts */
    pwm_input_state = 0;
    for (uint8_t i = 0; i < PWM_NUM_INPUTS; i++) {
        pwm_input_state_begin[i] = 0;
        pwm_input_values[i] = 0;

        gpio_configure_pin(pwm_input_pins[i], GPIO_DIR_INPUT | GPIO_PULL_DOWN);
        gpio_enable_pin_interrupt(pwm_input_pins[i], GPIO_PIN_CHANGE);
        INTC_register_interrupt(&pwm_input_interrupt_handler,
            AVR32_GPIO_IRQ_0 + (pwm_input_pins[i] / 8), AVR32_INTC_INT0);
    }

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
    uint16_t internal_values[PWM_NUM_OUTPUTS];
    struct fcs_parameter_t param;
    size_t i;

    /* Handle internal/external control and failsafe logic */
    if (!pwm_use_internal) {
        if (fcs_parameter_find_by_type_and_device(
                &comms_in_log, FCS_PARAMETER_CONTROL_SETPOINT, 0, &param)) {
            for (i = 0; i < PWM_NUM_OUTPUTS; i++) {
                internal_values[i] = swap16(param.data.u16[i]);
            }

            pwm_set_values(internal_values);
            pwm_missed_internal_ticks = 0;
        } else {
            pwm_missed_internal_ticks++;
        }

        if (pwm_missed_internal_ticks > PWM_FAILSAFE_INTERNAL_TICKS) {
            pwm_terminate_flight();
        }
    } else {
        pwm_set_values(pwm_input_values);
        pwm_missed_external_ticks = 0;
        /* FIXME: detect R/C failsafe condition */

        if (pwm_missed_external_ticks > PWM_FAILSAFE_EXTERNAL_TICKS) {
            pwm_terminate_flight();
        }
    }

    /*
    We keep calling this even after it's enabled because the pin has to
    be toggled low-high-low every 10ms otherwise the board will go into
    bypass.
    */
    pwm_enable();

    /* Output PWM input values */
    fcs_parameter_set_header(&param, FCS_VALUE_UNSIGNED, 16u, PWM_NUM_INPUTS);
    fcs_parameter_set_type(&param, FCS_PARAMETER_CONTROL_POS);
    fcs_parameter_set_device_id(&param, 0);
    for (i = 0; i < PWM_NUM_INPUTS; i++) {
        param.data.u16[i] = swap16(pwm_input_values[i]);
    }
    (void)fcs_log_add_parameter(&comms_out_log, &param);

    /* Output control mode -- 1 for internal, 0 for external (R/C) */
    fcs_parameter_set_header(&param, FCS_VALUE_UNSIGNED, 8u, 1u);
    fcs_parameter_set_type(&param, FCS_PARAMETER_CONTROL_MODE);
    fcs_parameter_set_device_id(&param, 0);
    param.data.u8[0] = pwm_use_internal ? 1u : 0;
    (void)fcs_log_add_parameter(&comms_out_log, &param);

    /*
    Handle internal <-> external control transition -- wait for
    PWM_TRANSITION_PULSE_COUNT pulses higher/lower than the threshold.
    */
    if (pwm_use_internal && pwm_input_values[3] <
            PWM_INTERNAL_TO_EXTERNAL_THRESHOLD) {
        pwm_transition_pulses++;
    } else if (!pwm_use_internal && pwm_input_values[3] >
            PWM_EXTERNAL_TO_INTERNAL_THRESHOLD) {
        pwm_transition_pulses++;
    } else {
        pwm_transition_pulses = 0;
    }

    if (pwm_transition_pulses >= PWM_TRANSITION_PULSE_COUNT) {
        pwm_use_internal = !pwm_use_internal;
    }
}

void pwm_set_values(uint16_t pwms[PWM_NUM_OUTPUTS]) {
    volatile avr32_pwm_t *pwm = &AVR32_PWM;
    size_t i;
    uint32_t pwm_val;
    bool updated = false;

    /* Make sure the previous update has been applied */
    if (!pwm->SCUC.updulock) {
        for (i = 0; i < PWM_NUM_OUTPUTS; i++) {
            /*
            Channel period is 1048576 cycles long, at 48Hz; PWM pulse width is
            1-2ms. A PWM value in the range 42729-108264 means the pulse range
            is 0.85-2.15ms.
            */
            pwm_val = (uint32_t)pwms[i] + 42729u;
            if (pwm_val != pwm_out_values[i]) {
                /* Change PWM duty cycle for the current channel (20-bit) */
                pwm->channel[i].cdtyupd = pwm_val & 0x000fffffu;
                pwm->channel[i].cprdupd = 1048575u;

                pwm_out_values[i] = pwm_val;
                updated = true;
            }
        }

        if (updated) {
            pwm->SCUC.updulock = 1u;
        }
    }
}

void pwm_enable(void) {
    gpio_local_tgl_gpio_pin(PWM_ENABLE_PIN);
    LED_OFF(LED2_GPIO);

    pwm_is_enabled = true;
}

void pwm_disable(void) {
    gpio_local_clr_gpio_pin(PWM_ENABLE_PIN);
    memset(pwm_out_values, 0, sizeof(pwm_out_values));
    LED_ON(LED2_GPIO);

    pwm_is_enabled = false;
}

static void pwm_check_inputs(void) {
    /*
    Check PWM input pins; if there's a state change, reset the current count.
    Normally called from interrupt.
    */
    uint32_t cur_state = 0, pins_changed;

    for (uint8_t i = 0; i < PWM_NUM_INPUTS; i++) {
        cur_state |= gpio_local_get_pin_value(pwm_input_pins[i]) << i;
    }

    pins_changed = cur_state ^ pwm_input_state;

    if (pins_changed) {
        uint32_t count = Get_system_register(AVR32_COUNT);

        for (uint8_t i = 0; i < PWM_NUM_INPUTS; i++) {
            if (pins_changed & (1u << i)) {
                /*
                If the last input state was high, update the PWM value
                according to the delta
                */
                if (pwm_input_state & (1u << i)) {
                    uint32_t delta = count - pwm_input_state_begin[i];

                    /*
                    Now we have number of cycles the input was high for; we
                    need a mapping from 0.85-2.15ms to the range [0, 65535].
                    */
                    if (delta <= 42829u) {
                        pwm_input_values[i] = 0;
                    } else if (delta >= 108264u) {
                        pwm_input_values[i] = 65535u;
                    } else {
                        pwm_input_values[i] = (delta - 42829u) & 0xffffu;
                    }
                }

                pwm_input_state_begin[i] = count;
            }
        }

        /* Update current input state */
        pwm_input_state = cur_state & 0xfu;
    }
}

static void pwm_terminate_flight(void) {
    /*
    Infinite loop to lock out any possibility of recovery -- reset the WDT
    each time as well otherwise the system will restart itself.
    */
    static uint16_t pwm_values[PWM_NUM_OUTPUTS] = {0, 0, 65535u, 0};

    LED_ON(LED3_GPIO);

    irqflags_t flags = cpu_irq_save();
    /* Take control of the PWM */
    for (;;) {
        pwm_enable();
        pwm_set_values(pwm_values);
        wdt_clear();
    }
    cpu_irq_restore(flags);

    LED_OFF(LED3_GPIO);
}
