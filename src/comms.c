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
#include <compiler.h>
#include <string.h>
#include <board.h>
#include "cobsr.h"
#include "crc8.h"
#include "comms.h"
#include "venus638_gps.h"
#include "failsafe.h"
#include "dfu.h"

#ifndef CONTINUE_ON_ASSERT
#define CommsAssert(x) Assert(x)
#else
#define CommsAssert(x) if (!(x)) { wdt_reset_mcu(); }
#endif

/*
See https://github.com/bendyer/uav/wiki/IO-Board-Design
*/

struct sensor_packet_t {
    /* Base fields */
    uint8_t crc;
    uint16_t tick;
    uint32_t status;
    uint8_t sensor_update_flags;

    /* Sensor fields */
    struct {
        int16_t x, y, z; /* [-4,4]g, 8192LSb/g */
    } __attribute__ ((packed)) accel;
    struct {
        int16_t x, y, z; /* [-500,500]deg/s, 65.5LSb/(deg/s) */
    } __attribute__ ((packed)) gyro;
    int16_t accel_gyro_temp; /* -40 to 85degC, 340LSb/degC,
        -512 = 35degC */
    uint16_t pressure; /* 10-1200mbar, 1LSb = 0.02mbar */
    uint16_t barometer_temp; /* -40 to 125degC, in 0.01degC increments,
        0 = -40degC */
    int16_t pitot; /* 16-bit ADC reading -- pitot sensor */
    int16_t i; /* 16-bit ADC reading -- current sensor */
    int16_t v; /* 16-bit ADC reading -- voltage sensor */
    int16_t range; /* 16-bit ADC reading -- rangefinder */
    uint8_t gpin_state_pwm_id; /* & 0x0f for pin state, & 0xf0 for PWM read */
    uint16_t pwm_value; /* Latest PWM value for the indicated channel */

    /* Magnetometer */
    struct {
        int16_t x, y, z; /* [-2,2]Ga, 1090LSb/Ga */
    } __attribute__ ((packed)) mag;

    /* GPS fields */
    struct {
        struct {
            int32_t lat, lng; /* lat, lng in 10^-7 degress */
            int32_t alt; /* alt above msl in  cm */
        } position;
        struct {
            int16_t x, y, z; /* NED in cm/s */
        } __attribute__ ((packed)) velocity;
    } __attribute__ ((packed)) gps;

    struct {
        uint8_t fix_mode_num_satellites; /* 2x 4-bit values */
        uint16_t pdop;
    } __attribute__ ((packed)) gps_info;
} __attribute__ ((packed));

#define SENSOR_PACKET_LEN 37u
#define MAG_DATA_LEN 6u
#define GPS_POS_LEN 18u
#define GPS_INFO_LEN 3u

#define SENSOR_STATUS_TXERR_MASK   0x00000001u
#define SENSOR_STATUS_RXERR_MASK   0x00000002u
#define SENSOR_STATUS_GPS_MASK     0x0000001cu
#define SENSOR_STATUS_GPS_OFFSET   2u
#define SENSOR_STATUS_BAROMETER_MASK  0x000000e0u
#define SENSOR_STATUS_BAROMETER_OFFSET 5u
#define SENSOR_STATUS_ACCEL_GYRO_MASK 0x00000700u
#define SENSOR_STATUS_ACCEL_GYRO_OFFSET 8u
#define SENSOR_STATUS_MAGNETOMETER_MASK 0x00003800u
#define SENSOR_STATUS_MAGNETOMETER_OFFSET 11u
#define SENSOR_STATUS_CPULOAD_MASK 0x0001C000u
#define SENSOR_STATUS_CPULOAD_OFFSET 14u
#define SENSOR_STATUS_TYPE_MASK 0xFF000000u
#define SENSOR_STATUS_TYPE 0x00u
#define SENSOR_STATUS_UNUSED_MASK  0x00FE0000u

#define CMD_KEY_LEN 64u

struct control_packet_t {
    uint8_t crc;
    uint16_t tick;
    uint8_t msg_type;
    uint16_t pwm[4];
    uint8_t gpout;
} __attribute__ ((packed));

struct transfer_packet_t {
    uint8_t crc;
    uint16_t tick;
    uint8_t msg_type;
    uint8_t payload_len;
    uint8_t payload[248];
} __attribute__ ((packed));

struct cmd_packet_t {
    uint8_t crc;
    uint16_t tick;
    uint8_t msg_type;
    uint8_t key[CMD_KEY_LEN];
} __attribute__ ((packed));

enum msg_type_t {
    MSG_TYPE_NONE = 0,
    MSG_TYPE_CONTROL = 1,
    MSG_TYPE_TRANSFER_MB = 2,
    MSG_TYPE_CMD = 3
};

static uint8_t cmd_key_dfu[] = "DFU_DFU_DFU_DFU_";
static uint8_t cmd_key_packet_rate_div_set[] = "PACKET_RATE_DIV_SET";
static uint8_t cmd_packet_rate_div = 1u;

#define MSG_TRANSFER_OVERHEAD 5u

#define TICK_MAX 65535u
/* From http://www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
   0x97 in Koopman notation = 0x12F in MSB-first notation, so excluing implicit
   x^n term we get 2F. */
#define COMM_CRC8_POLY 0x2Fu
#define OUTBUF_LEN 80u
#define INBUF_LEN 256u
#define RX_BUF_LEN 256u

#define UPDATED_ACCEL 0x01u
#define UPDATED_GYRO 0x02u
#define UPDATED_BAROMETER 0x04u
#define UPDATED_MAG 0x08u
#define UPDATED_GPS_POS 0x10u
#define UPDATED_GPS_INFO 0x20u
#define UPDATED_ADC_GPIO 0x40u

/* Wait 5 seconds after receipt of last message before killing CPU, then
   hold reset high for 10 seconds. */
#define CPU_TIMEOUT_TICKS 5000u
#define CPU_RESET_TICKS 10000u

enum rx_buf_parse_state_t {
    RX_NO_MSG = 0,
    RX_START,
    RX_IN_MSG,
    RX_END
};

static struct sensor_packet_t packet;
static struct control_packet_t control;
static struct cmd_packet_t cmd;
static uint8_t outbuf[OUTBUF_LEN];

static uint8_t rx_buf[RX_BUF_LEN];
static uint8_t rx_buf_idx;
static uint8_t inbuf[INBUF_LEN];
static uint8_t msgbuf[INBUF_LEN];
static uint8_t inbuf_idx;
static enum rx_buf_parse_state_t rx_parse_state = RX_NO_MSG;

static uint8_t crc_lookup[CRC8_TABLE_SIZE];
static bool usb_enabled;
static uint16_t last_cpu_packet_tick;
static uint16_t cpu_reset_countdown_tick;

static void comms_process_rx_buf(uint32_t bytes_read);

inline static int16_t clamp_s16(int32_t v) {
    int16_t result;
    if (v < INT16_MIN) {
        result = INT16_MIN;
    } else if (v > INT16_MAX) {
        result = INT16_MAX;
    } else {
        result = (int16_t)v;
    }

    return result;
}

void comms_set_cpu_status(uint32_t cycles_used) {
    packet.status &= ~SENSOR_STATUS_CPULOAD_MASK;

    uint32_t proportion_used = (6000u * cycles_used) / sysclk_get_cpu_hz();
    if (proportion_used > 7u) {
        proportion_used = 7u;
    }
    packet.status |= (proportion_used & 0x7u) << SENSOR_STATUS_CPULOAD_OFFSET;
}

void comms_set_gps_state(uint8_t status) {
    packet.status &= ~SENSOR_STATUS_GPS_MASK;
    packet.status |= status << SENSOR_STATUS_GPS_OFFSET;
}

void comms_set_magnetometer_state(uint8_t status) {
    packet.status &= ~SENSOR_STATUS_MAGNETOMETER_MASK;
    packet.status |= status << SENSOR_STATUS_MAGNETOMETER_OFFSET;
}

void comms_set_accel_gyro_state(uint8_t status) {
    packet.status &= ~SENSOR_STATUS_ACCEL_GYRO_MASK;
    packet.status |= status << SENSOR_STATUS_ACCEL_GYRO_OFFSET;
}

void comms_set_barometer_state(uint8_t status) {
    packet.status &= ~SENSOR_STATUS_BAROMETER_MASK;
    packet.status |= status << SENSOR_STATUS_BAROMETER_OFFSET;
}

void comms_set_mag_xyz(int16_t x, int16_t y, int16_t z) {
    packet.sensor_update_flags |= UPDATED_MAG;

    packet.mag.x = x;
    packet.mag.y = y;
    packet.mag.z = z;
}

void comms_set_accel_xyz(int16_t x, int16_t y, int16_t z) {
    packet.sensor_update_flags |= UPDATED_ACCEL;

    packet.accel.x = x;
    packet.accel.y = y;
    packet.accel.z = z;
}

void comms_set_gyro_xyz(int16_t x, int16_t y, int16_t z) {
    packet.sensor_update_flags |= UPDATED_GYRO;

    packet.gyro.x = x;
    packet.gyro.y = y;
    packet.gyro.z = z;
}

void comms_set_accel_gyro_temp(int16_t temp) {
    packet.sensor_update_flags |= UPDATED_ACCEL;

    packet.accel_gyro_temp = temp;
}

void comms_set_barometer_pressure_temp(int32_t pressure, int32_t temp) {
    CommsAssert(1000 <= pressure && pressure <= 120000);
    CommsAssert(-4000 <= temp && temp <= 8500);

    uint32_t pressure_scaled = pressure >> 1u,
             temp_pos = temp + 4000;

    /* temp_pos in range [0, 12500]; pressure_scaled in range [500, 60000] */
    CommsAssert(0 <= pressure_scaled && pressure_scaled <= 60000u);
    CommsAssert(0 <= temp_pos && temp_pos <= 12500u);

    packet.sensor_update_flags |= UPDATED_BAROMETER;

    packet.pressure = (uint16_t)pressure_scaled;
    packet.barometer_temp = (uint16_t)temp_pos;
}

void comms_set_pitot(uint16_t v) {
    packet.sensor_update_flags |= UPDATED_ADC_GPIO;

    packet.pitot = v;
}

void comms_set_range(uint16_t v) {
    packet.sensor_update_flags |= UPDATED_ADC_GPIO;

    packet.range = v;
}

void comms_set_iv(uint16_t i, uint16_t v) {
    packet.sensor_update_flags |= UPDATED_ADC_GPIO;

    packet.i = i;
    packet.v = v;
}

void comms_set_gpin_state(uint8_t v) {
    CommsAssert(v <= 0x0fu);

    packet.sensor_update_flags |= UPDATED_ADC_GPIO;

    packet.gpin_state_pwm_id =
        (packet.gpin_state_pwm_id & 0xf0u) | (v & 0x0fu);
}

void comms_set_pwm_value(uint8_t pwm_id, uint16_t value) {
    CommsAssert(pwm_id < 4u);

    packet.gpin_state_pwm_id =
        (packet.gpin_state_pwm_id & 0x0fu) | ((pwm_id & 0x0fu) << 4u);
    packet.pwm_value = value;
}

void comms_set_gps_pv(int32_t lat, int32_t lng, int32_t alt, int32_t vx,
        int32_t vy, int32_t vz) {
    packet.sensor_update_flags |= UPDATED_GPS_POS;
    packet.gps.position.lat = lat;
    packet.gps.position.lng = lng;
    packet.gps.position.alt = alt;
    packet.gps.velocity.x = clamp_s16(vx);
    packet.gps.velocity.y = clamp_s16(vy);
    packet.gps.velocity.z = clamp_s16(vz);
}

void comms_set_gps_info(uint8_t fix_mode, uint16_t pdop,
        uint8_t num_satellites) {
    CommsAssert(fix_mode <= GPS_FIX_3D_DGPS);
    CommsAssert(num_satellites < 16u);

    packet.sensor_update_flags |= UPDATED_GPS_INFO;
    packet.gps_info.fix_mode_num_satellites = (fix_mode << 4u) + num_satellites;
    packet.gps_info.pdop = pdop;
}

uint16_t comms_get_pwm(uint8_t pwm_id) {
    CommsAssert(pwm_id < 4u);

    return control.pwm[pwm_id];
}

uint8_t comms_get_gpout(void) {
    return control.gpout;
}

uint32_t comms_init(void) {
    crc8_init(crc_lookup, COMM_CRC8_POLY);

    /* Set up serial comms + CPU board interface */
    gpio_enable_module_pin(CPU_USART_RXD_PIN, CPU_USART_RXD_FUNCTION);
    gpio_enable_module_pin(CPU_USART_TXD_PIN, CPU_USART_TXD_FUNCTION);
    sysclk_enable_pba_module(CPU_USART_SYSCLK);

    /* Configure CPU board reset output */
    gpio_configure_pin(CPU_RESET_PIN, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

	/* Clear out packets */
	memset(&control, 0, sizeof(control));
	memset(&packet, 0, sizeof(packet));

    last_cpu_packet_tick = 0;
    cpu_reset_countdown_tick = 0;

    /* Configure CPU USART */
    static usart_options_t usart_options;
    usart_options.baudrate = 921600u;
    usart_options.charlength = 8u;
    usart_options.paritytype = USART_NO_PARITY;
    usart_options.stopbits = USART_1_STOPBIT;
    usart_options.channelmode = USART_NORMAL_CHMODE;
    CPU_USART->idr = 0xFFFFFFFFu;
    uint32_t result = usart_init_rs232(CPU_USART, &usart_options,
        sysclk_get_pba_hz());
    CommsAssert(result == USART_SUCCESS);

    return 0;
}

void comms_enable_usb(void) {
    CommsAssert(!usb_enabled);
    usb_enabled = true;
}

void comms_disable_usb(void) {
    CommsAssert(usb_enabled);
    usb_enabled = false;
}

uint16_t comms_tick(void) {
    /* Wrap packet values around */
    if (packet.tick == TICK_MAX) {
        packet.tick = 0;
    } else {
        packet.tick++;
    }

    /* Turn LED1 on if we haven't seen each of the sensors updated this second
       */
    static uint8_t sensors_updated = 0;
    if (packet.tick % 1000 == 0) {
        if (sensors_updated == (UPDATED_ACCEL | UPDATED_GYRO |
                UPDATED_BAROMETER | UPDATED_MAG | UPDATED_GPS_POS |
                UPDATED_GPS_INFO | UPDATED_ADC_GPIO)) {
            LED_OFF(LED1_GPIO);
        } else {
            LED_ON(LED1_GPIO);
        }
        sensors_updated = 0;
    } else {
        /* Accumulate the set of sensor updates */
        sensors_updated |= packet.sensor_update_flags;
    }

    /* Work out actual packet length */
    uint8_t packet_len = SENSOR_PACKET_LEN;
    if (packet.sensor_update_flags & UPDATED_GPS_INFO) {
        packet_len += MAG_DATA_LEN + GPS_POS_LEN + GPS_INFO_LEN;
    } else if (packet.sensor_update_flags & UPDATED_GPS_POS) {
        packet_len += MAG_DATA_LEN + GPS_POS_LEN;
    } else if (packet.sensor_update_flags & UPDATED_MAG) {
        packet_len += MAG_DATA_LEN;
    } else {
        /* No packet length modifiers */
    }

    CommsAssert(packet_len <= OUTBUF_LEN - 2u);

    /* Calculate CRC for the first byte of the packet, based on the rest of
       the packet */
    const uint8_t const* packet_data_ptr = ((uint8_t *)&packet) + sizeof(uint8_t);
    packet.crc = crc8(crc_lookup, packet_data_ptr, packet_len - 1, 0);

    /* Encode the whole packet using COBS-R */
    struct cobsr_encode_result encode_result;
    encode_result = cobsr_encode(&(outbuf[1]), OUTBUF_LEN,
        (const uint8_t *)&packet, packet_len);

    /* Check for errors */
    CommsAssert(outbuf[0] == 0);
    CommsAssert(encode_result.status == COBSR_ENCODE_OK);
    CommsAssert(encode_result.out_len <= packet_len + 1u);

    outbuf[1 + encode_result.out_len] = 0; /* Append a null byte */

    /* If USB is enabled and ready, mirror sends from UART to USB, and use
       USB for reads instead of UART. */
    if (usb_enabled) {
        /* Only send one USB packet every cmd_packet_rate_div ticks. */
        if (udi_cdc_is_tx_ready() && packet.tick % cmd_packet_rate_div == 0) {
            udi_cdc_write_buf(outbuf, encode_result.out_len + 2u);
        }

        /* Receive data from USB, via the RX buffer */
        if (udi_cdc_is_rx_ready()) {
            rx_buf_idx = 0;
            comms_process_rx_buf(RX_BUF_LEN -
                udi_cdc_read_buf(rx_buf, RX_BUF_LEN));

            last_cpu_packet_tick = packet.tick;
            failsafe_got_cpu_packet();
        }
    } else {
        /* Receive data from primary UART */
        volatile avr32_pdca_channel_t *pdca_channel =
            &AVR32_PDCA.channel[PDCA_CHANNEL_CPU_RX];

        uint32_t bytes_read = RX_BUF_LEN - pdca_channel->tcr;
        uint8_t bytes_avail = 0;

        CommsAssert(bytes_read <= RX_BUF_LEN);

        /* Work out the number of bytes available in the ring buffer */
        if (bytes_read > rx_buf_idx) {
            bytes_avail = bytes_read - rx_buf_idx;
        } else if (bytes_read < rx_buf_idx) {
            bytes_avail = (RX_BUF_LEN - rx_buf_idx - 1u) + bytes_read;
        } else {
            /* bytes_read == rx_buf_idx, i.e. nothing new has been read */
            bytes_avail = 0;
        }

        if (bytes_avail > RX_BUF_LEN >> 1u) {
            /* Either the PDCA isn't initialized, or there's been a possible
               buffer overflow -- either way, re-initialize the RX PDCA channel. */
            bytes_read = 0;
            bytes_avail = 0;
            rx_buf_idx = 0;
            rx_parse_state = RX_NO_MSG;

            irqflags_t flags = cpu_irq_save();
            pdca_channel->cr = AVR32_PDCA_TDIS_MASK;
            pdca_channel->mar = (uint32_t)rx_buf;
            pdca_channel->tcr = RX_BUF_LEN;
            pdca_channel->marr = (uint32_t)rx_buf;
            pdca_channel->tcrr = RX_BUF_LEN;
            pdca_channel->psr = CPU_USART_PDCA_PID_RX;
            pdca_channel->mr = (AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET)
                | (1 << AVR32_PDCA_RING_OFFSET);
            pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
            pdca_channel->isr;
            cpu_irq_restore(flags);
        }

        if (bytes_avail) {
            comms_process_rx_buf(bytes_avail);
            last_cpu_packet_tick = packet.tick;
            failsafe_got_cpu_packet();
        }
    }

    if (packet.tick % cmd_packet_rate_div == 0) {
        /* Schedule the transfer over CPU UART */
        volatile avr32_pdca_channel_t *pdca_channel =
            &AVR32_PDCA.channel[PDCA_CHANNEL_CPU_TX];

        irqflags_t flags = cpu_irq_save();
        pdca_channel->cr = AVR32_PDCA_TDIS_MASK;
        pdca_channel->mar = (uint32_t)outbuf;
        pdca_channel->tcr = encode_result.out_len + 2u;
        pdca_channel->marr = 0;
        pdca_channel->tcrr = 0;
        pdca_channel->psr = CPU_USART_PDCA_PID_TX;
        pdca_channel->mr = AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET;
        pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
        pdca_channel->isr;
        cpu_irq_restore(flags);

        /* Clear out the packet data after the tick value */
        memset(((uint8_t *)&packet) + sizeof(uint8_t) + sizeof(uint16_t),
            0, sizeof(packet) - sizeof(uint8_t) - sizeof(uint16_t));
    }

    /* Set the CPU reset line if the last received packet was more than 30s
       ago */
    if (cpu_reset_countdown_tick == 0 &&
            packet.tick - last_cpu_packet_tick > CPU_TIMEOUT_TICKS) {
        cpu_reset_countdown_tick = CPU_RESET_TICKS;
        gpio_local_set_gpio_pin(CPU_RESET_PIN);
    } else if (cpu_reset_countdown_tick == 1u) {
        cpu_reset_countdown_tick = 0;
        last_cpu_packet_tick = packet.tick;
        gpio_local_clr_gpio_pin(CPU_RESET_PIN);
    } else if (cpu_reset_countdown_tick > 1u) {
        cpu_reset_countdown_tick--;
    } else {
        /* No timeout conditions to handle */
    }

    return packet.tick;
}

static void comms_process_rx_buf(uint32_t bytes_avail) {
    CommsAssert(bytes_avail <= RX_BUF_LEN);
    CommsAssert(RX_NO_MSG <= rx_parse_state && rx_parse_state <= RX_END);

    for (; bytes_avail; bytes_avail--) {
        uint8_t ch = rx_buf[rx_buf_idx];
        rx_buf_idx++;

        if (ch == 0x00) {
            if (rx_parse_state == RX_IN_MSG) {
                rx_parse_state = RX_END;
            } else {
                rx_parse_state = RX_START;
            }
        } else {
            if (rx_parse_state == RX_START) {
                rx_parse_state = RX_IN_MSG;
                inbuf_idx = 0;
            } else if (rx_parse_state != RX_IN_MSG) {
                /* Didn't get a valid packet -- we're receiving
                   data without having found a start byte. */
                rx_parse_state = RX_NO_MSG;
            } else {
                /* rx_parse_state == RX_IN_MSG; this is handled below */
            }
        }

        if (rx_parse_state == RX_IN_MSG) {
            /* Currently parsing a message, so copy the data to
               the message buffer */
            if (inbuf_idx < INBUF_LEN) {
                inbuf[inbuf_idx] = ch;
                inbuf_idx++;
            } else {
                /* Buffer overrun -- messages are only allowed to
                   be 254B long so something went wrong */
                rx_parse_state = RX_NO_MSG;
            }
        } else if (rx_parse_state == RX_END) {
            /* Message was fully received; decode it now */
            struct cobsr_decode_result dresult;
            dresult = cobsr_decode(msgbuf, INBUF_LEN, inbuf,
                inbuf_idx);

            if (dresult.status == COBSR_DECODE_OK && dresult.out_len > 4u) {
                /* Check CRC */
                uint8_t calc_crc = crc8(crc_lookup, &(msgbuf[1]),
                    dresult.out_len - 1, 0);
                uint8_t packet_crc = msgbuf[0];

                /* Process the message and write into the relevant
                   packet. */
                if (calc_crc == packet_crc) {
                    if (msgbuf[3] == MSG_TYPE_CONTROL &&
                            dresult.out_len == sizeof(control)) {
                        memcpy(&control, msgbuf, sizeof(control));
                    } else if (msgbuf[3] == MSG_TYPE_TRANSFER_MB &&
                            dresult.out_len == msgbuf[4] - MSG_TRANSFER_OVERHEAD) {
                        /* Make sure the payload length corresponds to the
                           total message length excluding overheads */
                        failsafe_set_boundary((int32_t*)(&msgbuf[5]),
							msgbuf[4] >> 3u);
                    } else if (msgbuf[3] == MSG_TYPE_CMD &&
                            dresult.out_len == sizeof(cmd)) {
                        memcpy(&cmd, msgbuf, sizeof(cmd));

                        /* Process command packet */
                        if (!memcmp(cmd_key_dfu, cmd.key,
								sizeof(cmd_key_dfu) - 1u)) {
                            /* Got a DFU packet -- reset into bootloader */
                            dfu_enable();

                            cpu_irq_disable();
                            wdt_opt_t opt = {
                               .us_timeout_period = 1000000
                            };
                            wdt_disable();
                            wdt_clear();
                            wdt_enable(&opt);
                            while (1);
                        } else if (!memcmp(cmd_key_packet_rate_div_set,
								cmd.key,
								sizeof(cmd_key_packet_rate_div_set) - 1u)) {
                            /* Got a packet rate set packet -- set global
                               rate divisor to the new value, assuming it's
                               not 0. */
                            cmd_packet_rate_div = cmd.key[CMD_KEY_LEN - 1u];
                            if (cmd_packet_rate_div == 0) {
                                cmd_packet_rate_div = 1u;
                            }
                        }
                    }
                }
            }
        } else {
            /* rx_parse_state != IN_MSG && rx_parse_state != RX_END
               This was handled in the if/else block above, so nothing more
               to do. */
        }
    }
}
