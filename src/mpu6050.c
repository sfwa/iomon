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
#include "mpu6050.h"

static uint8_t mpu6050_inbuf[14];

static struct twim_transaction_t init_sequence[] = {
    /* Device address, TX byte count, TX bytes (0-4), RX byte count, RX buffer
       With this configuration, accel and gyro are sampled at 8kHz with
       accelerometer LPF at 260Hz, gyro LPF at 256Hz, accel delay at 0ms and
       gyro delay at 0.98ms. Full-scale on gyro is 500deg/s, and accel is 4g. */

	/* Write 0x02 to RA_PWR_MGMT_1 -- sets clock source to gyro w/ PLL */
    {MPU6050_DEVICE_ADDR, 2u, {0x6bu, 0x02u}, 0, NULL},
    /* Write 0x00 to RA_SMPLRT_DIV -- 8000/(1+0) = 8kHz */
    {MPU6050_DEVICE_ADDR, 2u, {0x19u, 0x00u}, 0, NULL},
    /* Write 0x00 to RA_CONFIG -- disable FSync, 256Hz low-pass */
    {MPU6050_DEVICE_ADDR, 2u, {0x1au, 0x00u}, 0, NULL},
    /* Write 0x08 to RA_GYRO_CONFIG -- no self test, scale 500deg/s */
    {MPU6050_DEVICE_ADDR, 2u, {0x1bu, 0x08u}, 0, NULL},
    /* Write 0x08 to RA_ACCEL_CONFIG -- no self test, scale of +-4g, no HPF */
    {MPU6050_DEVICE_ADDR, 2u, {0x1cu, 0x08u}, 0, NULL},
    /* Write 0x00 to RA_SIGNAL_PATH_RESET -- reset sensor signal paths */
    {MPU6050_DEVICE_ADDR, 2u, {0x68u, 0x00u}, 0, NULL},
    TWIM_TRANSACTION_SENTINEL
};

static struct twim_transaction_t read_sequence[] = {
    /* Read 14 bytes from RA_ACCEL_XOUT_H -- returns:
       AX.H, AX.L, AY.H, AY.L, AZ.H, AZ.L,
       TEMP.H, TEMP.L,
       GX.H, GX.L, GY.H, GY.L, GZ.H, GZ.L */
    {MPU6050_DEVICE_ADDR, 1u, {0x3bu}, 14u, mpu6050_inbuf},
    TWIM_TRANSACTION_SENTINEL
};

static struct i2c_device_t mpu6050 = {
    .speed = 400000u,
    .power_delay = 100u,
    .init_timeout = 150u,
    .read_timeout = 5u,

    .sda_pin_id = MPU6050_TWI_TWD_PIN,
    .sda_function = MPU6050_TWI_TWD_FUNCTION,
    .scl_pin_id = MPU6050_TWI_TWCK_PIN,
    .scl_function = MPU6050_TWI_TWCK_FUNCTION,
    .enable_pin_id = MPU6050_ENABLE_PIN,
    .sysclk_id = MPU6050_TWI_SYSCLK,

    .twim_cfg = {
        .twim = MPU6050_TWI,
        .tx_pdca_num = PDCA_CHANNEL_MPU6050_TX,
        .rx_pdca_num = PDCA_CHANNEL_MPU6050_RX,
        .tx_pid = MPU6050_TWI_PDCA_PID_TX,
        .rx_pid = MPU6050_TWI_PDCA_PID_RX
    },

    .init_sequence = init_sequence,
    .read_sequence = read_sequence
};

void mpu6050_init(void) {
    i2c_device_init(&mpu6050);
}

void mpu6050_tick(void) {
    i2c_device_tick(&mpu6050);
    comms_set_accel_gyro_state((uint8_t)mpu6050.state);

    if (mpu6050.state == I2C_READ_SEQUENCE) {
        enum twim_transaction_result_t result;
        result = twim_run_sequence(&(mpu6050.twim_cfg), mpu6050.read_sequence,
            0);

        if (result == TWIM_TRANSACTION_EXECUTED) {
            /* Convert the result and update the comms module */
            int16_t data[7];
            memcpy(data, mpu6050_inbuf, sizeof(data));

            comms_set_accel_gyro_temp(data[3]);
            comms_set_accel_xyz(data[0], data[1], data[2]);
            comms_set_gyro_xyz(data[4], data[5], data[6]);

            mpu6050.state_timer = 0;
            /* Start the next read to make sure there are values ready
               next tick */
            twim_run_sequence(&(mpu6050.twim_cfg), mpu6050.read_sequence, 0);
        }
    }
}
