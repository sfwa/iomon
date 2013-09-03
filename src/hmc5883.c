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
#include "i2cdevice.h"
#include "comms.h"
#include "hmc5883.h"

static uint8_t hmc5883_inbuf[6];
static uint8_t hmc5883_testbuf[6];
static uint16_t hmc5883_last_thermal_comp = 0;
static bool hmc5883_performing_thermal_comp = false;
static uint8_t hmc5883_thermal_comp_sequence_idx = 0;

/* Nominal positive self-test measurements -- 1.16Ga for X and Y, and 1.09Ga
   for Z. Scaled by 8x to account for the exponentially-weighted moving
   average calculations. */
#define HMC5883_NOMINAL_TEST_X (1264 << 3)
#define HMC5883_NOMINAL_TEST_Y (1264 << 3)
#define HMC5883_NOMINAL_TEST_Z (1177 << 3)

/* X, Z, Y to match device order. Scaled by 8x the actual value to simplify
   EWMA filter. */
static int16_t hmc5883_test_results[3] = {HMC5883_NOMINAL_TEST_X,
    HMC5883_NOMINAL_TEST_Z, HMC5883_NOMINAL_TEST_Z};

static struct twim_transaction_t init_sequence[] = {
    /* Device address, TX byte count, TX bytes (0-4), RX byte count, RX buffer
       With this configuration, full-scale is 2Ga (1090LSB/Ga sensitivity),
       8 samples are averaged per measurement, and measurements are done in
       single mode at ~125Hz. */

    /* Write 0x78 to CRA -- 8 samples per measurement, 75Hz nominal, no bias */
    {HMC5883_DEVICE_ADDR, 2u, {0x00u, 0x78u}, 0, NULL},
    /* Write 0x00 to CRB -- gain = 2 (1090LSB/Ga) */
    {HMC5883_DEVICE_ADDR, 2u, {0x01u, 0x20u}, 0, NULL},
    TWIM_TRANSACTION_SENTINEL
};

static struct twim_transaction_t read_sequence[] = {
    /* Write single-measurement start to MODE register (0x01) */
    {HMC5883_DEVICE_ADDR, 2u, {0x02u, 0x01u}, 0, NULL},
    /* Read 6 bytes from DXRA -- returns:
       DXRA, DXRB, DZRA, DZRB, DYRA, DYRB (A=MSB, B=LSB) */
    {HMC5883_DEVICE_ADDR, 1u, {0x03u}, 6u, hmc5883_inbuf},
    TWIM_TRANSACTION_SENTINEL
};

static struct twim_transaction_t thermal_comp_sequence[] = {
    /* Write 0x79 to CRA -- 8 samples per measurement, 75Hz nominal,
       +ve self-test */
    {HMC5883_DEVICE_ADDR, 2u, {0x00u, 0x79u}, 0, NULL},
    /* Write single-measurement start to MODE register (0x01) */
    {HMC5883_DEVICE_ADDR, 2u, {0x02u, 0x01u}, 0, NULL},
    /* Read 6 bytes from DXRA -- returns:
       DXRA, DXRB, DZRA, DZRB, DYRA, DYRB (A=MSB, B=LSB) */
    {HMC5883_DEVICE_ADDR, 1u, {0x03u}, 6u, hmc5883_testbuf},
    /* Write 0x78 to CRA -- 8 samples per measurement, 75Hz nominal, no bias.
       Also repeat CRB and MODE settings just to make sure. */
    {HMC5883_DEVICE_ADDR, 4u, {0x00u, 0x78u, 0x20u, 0x01u}, 0, NULL},
    TWIM_TRANSACTION_SENTINEL
};

static struct i2c_device_t hmc5883 = {
    .speed = 100000u,
    .power_delay = 500u,
    .init_timeout = 600u,
    .read_timeout = 15u,

    .sda_pin_id = HMC5883_TWI_TWD_PIN,
    .sda_function = HMC5883_TWI_TWD_FUNCTION,
    .scl_pin_id = HMC5883_TWI_TWCK_PIN,
    .scl_function = HMC5883_TWI_TWCK_FUNCTION,
    .enable_pin_id = HMC5883_ENABLE_PIN,
    .sysclk_id = HMC5883_TWI_SYSCLK,

    .twim_cfg = {
        .twim = HMC5883_TWI,
        .tx_pdca_num = PDCA_CHANNEL_HMC5883_TX,
        .rx_pdca_num = PDCA_CHANNEL_HMC5883_RX,
        .tx_pid = HMC5883_TWI_PDCA_PID_TX,
        .rx_pid = HMC5883_TWI_PDCA_PID_RX
    },

    .init_sequence = init_sequence,
    .read_sequence = read_sequence
};

#ifndef CONTINUE_ON_ASSERT
#define HMC5883Assert(x) Assert(x)
#else
#define HMC5883Assert(x) if (!(x)) { hmc5883.state_timer = 0xffffu; }
#endif

/* Take a thermal compensation reading every second */
#define HMC5883_THERMAL_COMP_PERIOD 1000u

void hmc5883_measure(void);
void hmc5883_test(void);

void hmc5883_init(void) {
    i2c_device_init(&hmc5883);

    hmc5883_performing_thermal_comp = false;
    hmc5883_thermal_comp_sequence_idx = 0;
    hmc5883_last_thermal_comp = HMC5883_THERMAL_COMP_PERIOD;
}

void hmc5883_tick(void) {
    i2c_device_tick(&hmc5883);
    comms_set_magnetometer_state((uint8_t)hmc5883.state);

    if (hmc5883.state != I2C_READ_SEQUENCE) {
        return;
    }

    /* Switch between measurement and thermal compensation (self-test) --
       one self-test cycle every HMC5883_THERMAL_COMP_PERIOD milliseconds. */
    hmc5883_last_thermal_comp++;
    if (hmc5883_last_thermal_comp >= HMC5883_THERMAL_COMP_PERIOD) {
        hmc5883_performing_thermal_comp = true;
        hmc5883_last_thermal_comp = 0;
        thermal_comp_sequence[0].txn_status = TWIM_TRANSACTION_STATUS_NONE;
    }

    /* Run either the test sequence or the measurement sequence depending on
       current task */
    if (hmc5883_performing_thermal_comp) {
        hmc5883_test();
    }

    if (!hmc5883_performing_thermal_comp) {
        hmc5883_measure();
    }
}

void hmc5883_measure(void) {
    HMC5883Assert(!hmc5883_performing_thermal_comp);
    /* Self-test measurements should already have been made */

    if (hmc5883.state_timer >= 8u) {
        /* Wait until the 8th tick (~7ms after measurement command), then
           execute a read operation to get the latest magnetometer
           measurement. If the command completes, start another measurement. */
        enum twim_transaction_result_t read_result;
        read_result = twim_run_sequence(&(hmc5883.twim_cfg),
            hmc5883.read_sequence, 1);

        if (read_result == TWIM_TRANSACTION_EXECUTED) {
            /* Convert the result and update the comms module */
            int16_t m[3];
            memcpy(m, hmc5883_inbuf, sizeof(m));

            /* Magnetic field over-/underflow -- should maybe adjust
               sensitivity automatically? */
            if (!  (-2048 <= m[0] && m[0] <= 2047 &&
				    -2048 <= m[1] && m[1] <= 2047 &&
                    -2048 <= m[2] && m[2] <= 2047)) {
				/* Power the device down */
				if (hmc5883.enable_pin_id) {
					gpio_local_clr_gpio_pin(hmc5883.enable_pin_id);
				}
                i2c_device_state_transition(&hmc5883, I2C_POWERING_DOWN);
                return;
            }

            /* Apply thermal compensation based on the most recent
               self-test measurement */
            int32_t tmp;
            tmp = (((int32_t)(m[0])) * HMC5883_NOMINAL_TEST_X) /
                ((int32_t)(hmc5883_test_results[0]));
            HMC5883Assert(INT16_MIN <= tmp && tmp <= INT16_MAX);
            m[0] = (int16_t)tmp;

            tmp = (((int32_t)(m[1])) * HMC5883_NOMINAL_TEST_Z) /
                ((int32_t)(hmc5883_test_results[1]));
            HMC5883Assert(INT16_MIN <= tmp && tmp <= INT16_MAX);
            m[1] = (int16_t)tmp;

            tmp = (((int32_t)(m[2])) * HMC5883_NOMINAL_TEST_Y) /
                ((int32_t)(hmc5883_test_results[2]));
            HMC5883Assert(INT16_MIN <= tmp && tmp <= INT16_MAX);
            m[2] = (int16_t)tmp;

            /* Registers are ordered X, Z, Y */
            comms_set_mag_xyz(m[0], m[2], m[1]);

            hmc5883.state_timer = 0;
        }
    }

    if (hmc5883.state_timer == 0) {
        /* At the start of the read state, or after each successful read,
           generate a measurement command */
        hmc5883.read_sequence[0].txn_status = TWIM_TRANSACTION_STATUS_NONE;
        hmc5883.read_sequence[1].txn_status = TWIM_TRANSACTION_STATUS_NONE;
        twim_run_sequence(&(hmc5883.twim_cfg), hmc5883.read_sequence, 0);
    }
}

void hmc5883_test(void) {
    HMC5883Assert(hmc5883_performing_thermal_comp);

    if (hmc5883_thermal_comp_sequence_idx != 2) {
        /* Run init sequence commands in order until the sequence is done,
           then transition to read sequence */
        enum twim_transaction_result_t result;
        result = twim_run_sequence(&(hmc5883.twim_cfg),
            thermal_comp_sequence, hmc5883_thermal_comp_sequence_idx);

        if (result == TWIM_TRANSACTION_EXECUTED) {
            hmc5883.state_timer = 0;
            hmc5883_thermal_comp_sequence_idx++;
        } else if (result == TWIM_TRANSACTION_ERROR ||
                hmc5883.state_timer > hmc5883.init_timeout) {
            /* Power the device down */
            if (hmc5883.enable_pin_id) {
                gpio_local_clr_gpio_pin(hmc5883.enable_pin_id);
            }

            i2c_device_state_transition(&hmc5883, I2C_POWERING_DOWN);
        }
    } else if (hmc5883_thermal_comp_sequence_idx == 2 &&
            hmc5883.state_timer >= 8u) {
        /* Wait until the 8th tick (~7ms after measurement command), then
           execute a read operation to get the latest self-test
           measurement. */
        enum twim_transaction_result_t read_result;
        read_result = twim_run_sequence(&(hmc5883.twim_cfg),
            thermal_comp_sequence, hmc5883_thermal_comp_sequence_idx);

        if (read_result == TWIM_TRANSACTION_EXECUTED) {
            /* Copy the result to a temporary buffer, and then update the
               exponentially-weighted moving average in hmc5883_test_results.
               */
            int16_t results[3];
            memcpy(results, hmc5883_testbuf, sizeof(results));

            /* Self-test field over-/underflow -- should maybe adjust
               sensitivity automatically? */
            if (!  (-2048 <= results[0] && results[0] <= 2047 &&
                    -2048 <= results[1] && results[1] <= 2047 &&
                    -2048 <= results[2] && results[2] <= 2047)) {
				/* Power the device down */
				if (hmc5883.enable_pin_id) {
					gpio_local_clr_gpio_pin(hmc5883.enable_pin_id);
				}
                i2c_device_state_transition(&hmc5883, I2C_POWERING_DOWN);
                return;
            }

            /* Perform EWMA calculation: y(n) = 0.75y(n-1) + 0.25x(n)
               Should reach 95% of difference between original and current
               value within about 15 samples (= seconds).
               */
            hmc5883_test_results[0] = hmc5883_test_results[0] -
                (2 * hmc5883_test_results[0] / 8) + (2 * results[0]);
            hmc5883_test_results[1] = hmc5883_test_results[1] -
                (2 * hmc5883_test_results[1] / 8) + (2 * results[1]);
            hmc5883_test_results[2] = hmc5883_test_results[2] -
                (2 * hmc5883_test_results[2] / 8) + (2 * results[2]);

            /* Sanity check */
            HMC5883Assert(0 <= hmc5883_test_results[0] &&
                hmc5883_test_results[0] <= (2047 << 3));
            HMC5883Assert(0 <= hmc5883_test_results[1] &&
                hmc5883_test_results[1] <= (2047 << 3));
            HMC5883Assert(0 <= hmc5883_test_results[2] &&
                hmc5883_test_results[2] <= (2047 << 3));

            hmc5883.state_timer = 0;
			hmc5883_thermal_comp_sequence_idx++;
        }
    }

    if (hmc5883_thermal_comp_sequence_idx == 4 && hmc5883.state_timer == 8) {
        /* Exit thermal compensation mode, and start a regular read so that
           the hmc5883_measure loop has something to look for when the state
           timer reaches 8. */
        hmc5883_performing_thermal_comp = false;
        hmc5883_thermal_comp_sequence_idx = 0;
        hmc5883.state_timer = 0;
    }
}
