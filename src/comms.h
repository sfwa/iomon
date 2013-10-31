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

/*
Inititalize communications -- set up USART and clear data structures.
*/
uint32_t comms_init(void);

/*
Status register setting
*/
void comms_set_cpu_status(uint32_t cycles_used);
void comms_set_gps_state(uint8_t state);
void comms_set_magnetometer_state(uint8_t state);
void comms_set_accel_gyro_state(uint8_t state);
void comms_set_barometer_state(uint8_t state);

/*
Set packet values following reads from sensors (indirectly determines what
type of packet will be sent).
*/

void comms_set_mag_xyz(int16_t x, int16_t y, int16_t z);
void comms_set_accel_xyz(int16_t x, int16_t y, int16_t z);
void comms_set_gyro_xyz(int16_t x, int16_t y, int16_t z);
void comms_set_accel_gyro_temp(int16_t temp);
void comms_set_barometer_pressure_temp(int32_t pressure, int32_t temp);
void comms_set_pitot(uint16_t v);
void comms_set_range(uint16_t v);
void comms_set_iv(uint16_t i, uint16_t v);
void comms_set_gpin_state(uint8_t v);
void comms_set_gps_pv(int32_t lat, int32_t lng, int32_t alt, int32_t vn,
    int32_t ve, int32_t vd);
void comms_set_gps_info(uint8_t fix_mode, uint8_t pos_err,
    uint8_t num_satellites);

/*
Get packet values from CPU board input.
*/

uint16_t comms_get_pwm(uint8_t pwm_id);
uint8_t comms_get_gpout(void);

/*
Finalize the current tick's packet and send via PDC; handle parsing of input
messages as well.
*/
uint16_t comms_tick(void);
