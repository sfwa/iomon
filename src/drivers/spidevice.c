/*
Copyright (C) 2014 Ben Dyer

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
#include "spidevice.h"

void spi_device_init(struct spi_device_t *dev) {
    fcs_assert(dev);
    fcs_assert(dev->miso_pin_id && dev->miso_function < 8u);
    fcs_assert(dev->mosi_pin_id && dev->mosi_function < 8u);
    fcs_assert(dev->cs_pin_id && dev->cs_function < 8u);
    fcs_assert(dev->clk_pin_id && dev->clk_function < 8u);
    fcs_assert(1000000u <= dev->speed && dev->speed <= 20000000u);
    fcs_assert(dev->init_sequence);
    fcs_assert(dev->read_sequence);
    fcs_assert(dev->power_delay && dev->read_timeout && dev->init_timeout);

    dev->state = SPI_POWERING_DOWN;
    dev->sequence_idx = 0;
    dev->state_timer = 0;

    /* Set up GPIOs */
    gpio_enable_module_pin(dev->miso_pin_id, dev->miso_function);
    gpio_enable_module_pin(dev->mosi_pin_id, dev->mosi_function);
    gpio_enable_module_pin(dev->cs_pin_id, dev->cs_function);
    gpio_enable_module_pin(dev->clk_pin_id, dev->clk_function);
    sysclk_enable_pba_module(dev->sysclk_id);

    /* Configure power enable pin */
    if (dev->enable_pin_id) {
        gpio_configure_pin(dev->enable_pin_id,
            GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
    }

    spim_pdca_init(&(dev->spim_cfg), dev->speed);
}

void spi_device_tick(struct spi_device_t *dev) {
    fcs_assert(dev->power_delay && dev->read_timeout && dev->init_timeout);

    dev->state_timer++;

    if (dev->state == SPI_POWERING_DOWN &&
            dev->state_timer > dev->power_delay) {
        /* Power up */
        if (dev->enable_pin_id) {
            gpio_local_set_gpio_pin(dev->enable_pin_id);
        }
        spim_pdca_init(&(dev->spim_cfg), dev->speed);

        dev->init_sequence[0].txn_status = SPIM_TRANSACTION_STATUS_NONE;
        spi_device_state_transition(dev, SPI_POWERING_UP);
    } else if (dev->state == SPI_POWERING_UP &&
            dev->state_timer > dev->power_delay) {
        spi_device_state_transition(dev, SPI_INIT_SEQUENCE);
    } else if (dev->state == SPI_INIT_SEQUENCE) {
        /* Run init sequence commands in order until the sequence is done,
           then transition to read sequence */
        enum spim_transaction_result_t result;
        result = spim_run_sequence(&(dev->spim_cfg), dev->init_sequence,
            dev->sequence_idx);

        if (result == SPIM_TRANSACTION_SEQDONE) {
            dev->read_sequence[0].txn_status = SPIM_TRANSACTION_STATUS_NONE;
            spi_device_state_transition(dev, SPI_READ_SEQUENCE);
        } else if (result == SPIM_TRANSACTION_EXECUTED) {
            dev->sequence_idx++;
        } else if (result == SPIM_TRANSACTION_ERROR ||
                dev->state_timer > dev->init_timeout) {
            /* Power the device down */
            if (dev->enable_pin_id) {
                gpio_local_clr_gpio_pin(dev->enable_pin_id);
            }

            spi_device_state_transition(dev, SPI_POWERING_DOWN);
        }
    } else if (dev->state == SPI_READ_SEQUENCE &&
            dev->state_timer > dev->read_timeout) {
        /* Watch for timeouts in the read state -- power down */
        if (dev->enable_pin_id) {
            gpio_local_clr_gpio_pin(dev->enable_pin_id);
        }

        spi_device_state_transition(dev, SPI_POWERING_DOWN);
    } else {
        /* Either waiting for a timer to expire, or in the main read sequence
           -- either way, do nothing */
    }
}
