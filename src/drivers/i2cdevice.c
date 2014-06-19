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
#include "fcsassert.h"
#include "i2cdevice.h"

void i2c_device_init(struct i2c_device_t *dev) {
    fcs_assert(dev);
    fcs_assert(dev->sda_pin_id && dev->sda_function < 8u);
    fcs_assert(dev->scl_pin_id && dev->scl_function < 8u);
    fcs_assert(100000u <= dev->speed && dev->speed <= 400000u);
    fcs_assert(dev->init_sequence);
    fcs_assert(dev->read_sequence);
    fcs_assert(dev->power_delay && dev->read_timeout && dev->init_timeout);

    dev->state = I2C_POWERING_DOWN;
    dev->sequence_idx = 0;
    dev->state_timer = 0;

    /* Set up GPIOs */
    gpio_enable_module_pin(dev->sda_pin_id, dev->sda_function);
    gpio_enable_module_pin(dev->scl_pin_id, dev->scl_function);

    /* Configure power enable pin */
    if (dev->enable_pin_id) {
        gpio_configure_pin(dev->enable_pin_id,
            GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
    }

    twim_pdca_init(&(dev->twim_cfg), dev->speed);
}

void i2c_device_tick(struct i2c_device_t *dev) {
    fcs_assert(dev);
    fcs_assert(dev->read_sequence && dev->init_sequence);
    fcs_assert(dev->power_delay && dev->read_timeout && dev->init_timeout);

    dev->state_timer++;

    if (dev->state == I2C_POWERING_DOWN &&
            dev->state_timer > dev->power_delay) {
        /* Power up */
        if (dev->enable_pin_id) {
            gpio_local_set_gpio_pin(dev->enable_pin_id);
        }
        twim_pdca_init(&(dev->twim_cfg), dev->speed);

        dev->init_sequence[0].txn_status = TWIM_TRANSACTION_STATUS_NONE;
        i2c_device_state_transition(dev, I2C_POWERING_UP);
    } else if (dev->state == I2C_POWERING_UP &&
            dev->state_timer > dev->power_delay) {
        i2c_device_state_transition(dev, I2C_INIT_SEQUENCE);
    } else if (dev->state == I2C_INIT_SEQUENCE) {
        /* Run init sequence commands in order until the sequence is done,
           then transition to read sequence */
        enum twim_transaction_result_t result;
        result = twim_run_sequence(&(dev->twim_cfg), dev->init_sequence,
            dev->sequence_idx);

        if (result == TWIM_TRANSACTION_SEQDONE) {
            dev->read_sequence[0].txn_status = TWIM_TRANSACTION_STATUS_NONE;
            i2c_device_state_transition(dev, I2C_READ_SEQUENCE);
        } else if (result == TWIM_TRANSACTION_EXECUTED) {
            dev->sequence_idx++;
        } else if (result == TWIM_TRANSACTION_ERROR ||
                dev->state_timer > dev->init_timeout) {
            /* Power the device down */
            if (dev->enable_pin_id) {
                gpio_local_clr_gpio_pin(dev->enable_pin_id);
            }

            i2c_device_state_transition(dev, I2C_POWERING_DOWN);
        }
    } else if (dev->state == I2C_READ_SEQUENCE &&
            dev->state_timer > dev->read_timeout) {
        /* Watch for timeouts in the read state -- power down */
        if (dev->enable_pin_id) {
            gpio_local_clr_gpio_pin(dev->enable_pin_id);
        }

        i2c_device_state_transition(dev, I2C_POWERING_DOWN);
    } else {
        /* Either waiting for a timer to expire, or in the main read sequence
           -- either way, do nothing */
    }
}
