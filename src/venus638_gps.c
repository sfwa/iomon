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
#include <math.h>
#include "comms.h"
#include "venus638_gps.h"

#define  PI_F 3.14159265358979f

static void venus638_process_latest_msg(void);

enum venus638_state_t {
    VENUS638_POWERING_UP = 0,
    VENUS638_NAVIGATING,
    VENUS638_POWERING_DOWN
};

enum venus638_msg_parser_state_t {
    VENUS638_PARSER_NO_MSG = 0,
    VENUS638_PARSER_PREFIX_A0,
    VENUS638_PARSER_PREFIX_A1,
    VENUS638_PARSER_LENGTH_0,
    VENUS638_PARSER_LENGTH_1,
    VENUS638_PARSER_DONE_MSG
};

struct venus638_nav_data_t {
    uint8_t fix_mode; /* see gps_fix_mode_t */
    uint8_t num_satellites;
    uint16_t gps_week; /* GPS week number */
    uint32_t time_of_week; /* 1LSB = 0.01sec */
    int32_t latitude; /* 1e-7 deg */
    int32_t longitude; /* 1e-7 deg */
    int32_t ellipsoid_altitude; /* cm above ellipsoid */
    int32_t msl_altitude; /* cm above mean sea level */
    uint16_t gdop; /* 1LSB = 0.01 */
    uint16_t pdop; /* 1LSB = 0.01 */
    uint16_t hdop; /* 1LSB = 0.01 */
    uint16_t vdop; /* 1LSB = 0.01 */
    uint16_t tdop; /* 1LSB = 0.01 */
    int32_t ecef_pos[3]; /* X, Y, Z in cm */
    int32_t ecef_v[3]; /* X, Y, Z in cm/s */
} __attribute__ ((packed));

#define VENUS638_INBUF_SIZE 128u
#define VENUS638_MSGBUF_SIZE 64u
/* 2 prefix u8, 1 paylod length u16, one checksum u8 and two suffix u8s */
#define VENUS638_PREFIX_LEN 4u
#define VENUS638_SUFFIX_LEN 3u
#define VENUS638_MSG_OVERHEAD (VENUS638_PREFIX_LEN + VENUS638_SUFFIX_LEN)

/* Timeout values */
#define VENUS638_POWER_DELAY 1000u
#define VENUS638_TIMEOUT 2500u

/* GPS message IDs */
#define VENUS638_MSG_NAV_DATA 0xa8u

/* Sanity checks */
#define Venus638_state_is_valid(x) \
    (VENUS638_POWERING_UP <= (x) && (x) <= VENUS638_POWERING_DOWN)
#define Venus638_parser_state_is_valid(x) \
    (VENUS638_PARSER_NO_MSG <= (x) && (x) <= VENUS638_PARSER_DONE_MSG)

static enum venus638_state_t venus638_state = VENUS638_POWERING_DOWN;
static uint32_t venus638_state_timer; /* tracks time in current state */

static uint8_t venus638_inbuf[VENUS638_INBUF_SIZE];
static uint8_t venus638_msgbuf[VENUS638_MSGBUF_SIZE];

static enum venus638_msg_parser_state_t venus638_inbuf_parse_state =
    VENUS638_PARSER_NO_MSG;
static uint8_t venus638_inbuf_msg_len_remaining, venus638_inbuf_idx,
    venus638_inbuf_msg_crc, venus638_msgbuf_idx;

#ifndef CONTINUE_ON_ASSERT
#define Venus638Assert(x) Assert(x)
#else
#define Venus638Assert(x) if (!(x)) { venus638_state_timer = 0xffffu; }
#endif

static inline void venus638_state_transition(enum venus638_state_t new_state) {
    Venus638Assert(new_state != venus638_state);
    Venus638Assert(Venus638_state_is_valid(new_state));

    venus638_state = new_state;
    venus638_state_timer = 0;
}

void venus638_init(void) {
    venus638_state_timer = 0;
    venus638_inbuf_parse_state = VENUS638_PARSER_NO_MSG;
    venus638_inbuf_msg_len_remaining = 0;
    venus638_inbuf_msg_crc = 0;
    venus638_inbuf_idx = 0;
    venus638_msgbuf_idx = 0;

    /* USART GPIO pin configuration and sysclk setup */
    gpio_enable_module_pin(GPS_USART_RXD_PIN, GPS_USART_RXD_FUNCTION);
    gpio_enable_module_pin(GPS_USART_TXD_PIN, GPS_USART_TXD_FUNCTION);
    sysclk_enable_pba_module(GPS_USART_SYSCLK);

    /* Configure GPS power enable */
    gpio_configure_pin(GPS_ENABLE_PIN, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

    /* Enable USART -- start in 115200 baud mode */
    usart_options_t usart_options;
    usart_options.baudrate = 115200u;
    usart_options.charlength = 8u;
    usart_options.paritytype = USART_NO_PARITY;
    usart_options.stopbits = USART_1_STOPBIT;
    usart_options.channelmode = USART_NORMAL_CHMODE;
    GPS_USART->idr = 0xFFFFFFFFu;
    uint32_t result = usart_init_rs232(GPS_USART, &usart_options,
        sysclk_get_pba_hz());
    Venus638Assert(result == USART_SUCCESS);
}

/* venus638_tick is called once per millisecond to handle message parsing and
   periodic tasks */
void venus638_tick(void) {
    Venus638Assert(Venus638_state_is_valid(venus638_state));
    Venus638Assert(Venus638_parser_state_is_valid(venus638_inbuf_parse_state));

    comms_set_gps_state((uint8_t)venus638_state);

    venus638_state_timer++;

    /* Parse messages appearing in the input buffer */
    volatile avr32_pdca_channel_t *pdca_channel =
        &AVR32_PDCA.channel[PDCA_CHANNEL_GPS_RX];

    /* Reset message done flag */
    if (venus638_inbuf_parse_state == VENUS638_PARSER_DONE_MSG) {
        venus638_inbuf_parse_state = VENUS638_PARSER_NO_MSG;
    }

    uint32_t inbuf_bytes_read = VENUS638_INBUF_SIZE - pdca_channel->tcr;
    uint8_t bytes_avail = 0;

    Venus638Assert(inbuf_bytes_read <= VENUS638_INBUF_SIZE);

    /* Work out the number of bytes available in the ring buffer */
    if (inbuf_bytes_read > venus638_inbuf_idx) {
        bytes_avail = inbuf_bytes_read - venus638_inbuf_idx;
    } else if (inbuf_bytes_read < venus638_inbuf_idx) {
        bytes_avail = (VENUS638_INBUF_SIZE - venus638_inbuf_idx - 1u) +
            inbuf_bytes_read;
    } else {
        /* inbuf_bytes_read == venus638_inbuf_idx, i.e. no new bytes read */
        bytes_avail = 0;
    }

    if (bytes_avail > VENUS638_INBUF_SIZE / 2u) {
        /* Either the PDCA isn't initialized, or there's been a possible
           buffer overflow -- either way, re-initialize the RX PDCA channel. */
        inbuf_bytes_read = 0;
        bytes_avail = 0;
        venus638_inbuf_idx = 0;
        venus638_inbuf_parse_state = VENUS638_PARSER_NO_MSG;

        irqflags_t flags = cpu_irq_save();
        pdca_channel->cr = AVR32_PDCA_TDIS_MASK;
        pdca_channel->mar = (uint32_t)venus638_inbuf;
        pdca_channel->tcr = VENUS638_INBUF_SIZE;
        pdca_channel->marr = (uint32_t)venus638_inbuf;
        pdca_channel->tcrr = VENUS638_INBUF_SIZE;
        pdca_channel->psr = GPS_USART_PDCA_PID_RX;
        pdca_channel->mr = (AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET)
            | (1 << AVR32_PDCA_RING_OFFSET);
        pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
        pdca_channel->isr;
        cpu_irq_restore(flags);
    }

    for (; bytes_avail && venus638_inbuf_parse_state != VENUS638_PARSER_DONE_MSG;
            bytes_avail--) {
        uint8_t ch = venus638_inbuf[venus638_inbuf_idx];
        venus638_inbuf_idx = (venus638_inbuf_idx + 1) & (VENUS638_INBUF_SIZE-1);

        if (venus638_inbuf_parse_state == VENUS638_PARSER_NO_MSG &&
                ch == 0xa0u) {
            /* Check that the previous and current characters form
               a message prefix; if so, change state to
               VENUS638_PARSER_IN_PREFIX */
            venus638_inbuf_parse_state = VENUS638_PARSER_PREFIX_A0;
        } else if (venus638_inbuf_parse_state == VENUS638_PARSER_PREFIX_A0 &&
                ch == 0xa1u) {
            venus638_inbuf_parse_state = VENUS638_PARSER_PREFIX_A1;
        } else if (venus638_inbuf_parse_state == VENUS638_PARSER_PREFIX_A1 &&
                ch == 0x00) {
            /* Ignore any packets with length > 256 (should never happen) */
            venus638_inbuf_parse_state = VENUS638_PARSER_LENGTH_0;
        } else if (venus638_inbuf_parse_state == VENUS638_PARSER_LENGTH_0 &&
                ch <= VENUS638_MSGBUF_SIZE - VENUS638_SUFFIX_LEN) {
            /* Read the lower byte of the message length and enter the main
               parse state */
            venus638_inbuf_msg_len_remaining = ch + VENUS638_SUFFIX_LEN;
            venus638_msgbuf_idx = 0;
            venus638_inbuf_msg_crc = 0;
            venus638_inbuf_parse_state = VENUS638_PARSER_LENGTH_1;
        } else if (venus638_inbuf_parse_state == VENUS638_PARSER_LENGTH_1) {
            /* Copy the current character into the buffer, and update the CRC
               value if the character isn't part of the message suffix */
            venus638_msgbuf[venus638_msgbuf_idx] = ch;
            venus638_msgbuf_idx++;

            if (venus638_inbuf_msg_len_remaining > VENUS638_SUFFIX_LEN) {
                venus638_inbuf_msg_crc ^= ch;
            }
            venus638_inbuf_msg_len_remaining--;

            /* Received all the message data -- check the CRC and suffix. */
            if (venus638_inbuf_msg_len_remaining == 0u) {
                if ((venus638_msgbuf[venus638_msgbuf_idx - 3] != venus638_inbuf_msg_crc) ||
                        (venus638_msgbuf[venus638_msgbuf_idx - 2] != 0x0du) ||
                        (venus638_msgbuf[venus638_msgbuf_idx - 1] != 0x0au)) {
                    /* Message is valid */
                    venus638_inbuf_parse_state = VENUS638_PARSER_NO_MSG;
                } else {
                    venus638_inbuf_parse_state = VENUS638_PARSER_DONE_MSG;
                }
            }
        } else {
            venus638_inbuf_parse_state = VENUS638_PARSER_NO_MSG;
        }
    }

    /* GPS driver state processing */
    if (venus638_state_timer > VENUS638_TIMEOUT &&
			venus638_state != VENUS638_POWERING_DOWN) {
        /* Power GPS down */
        gpio_local_clr_gpio_pin(GPS_ENABLE_PIN);
        venus638_state_transition(VENUS638_POWERING_DOWN);
    } else if (venus638_state_timer > VENUS638_POWER_DELAY) {
        if (venus638_state == VENUS638_POWERING_DOWN) {
            /* The VENUS638_POWERING_DOWN state holds for 500ms, powers the GPS
               unit back up, then transfers to VENUS638_POWERING_UP. */
            gpio_local_set_gpio_pin(GPS_ENABLE_PIN);
            venus638_state_transition(VENUS638_POWERING_UP);
        } else if (venus638_state == VENUS638_POWERING_UP) {
            /* The VENUS638_POWERING_UP holds for 500ms, then transitions to
               VENUS638_NAVIGATING */
            venus638_state_transition(VENUS638_NAVIGATING);
        } else {
            /* Not in a powering up/down state, so VENUS638_POWER_DELAY is
               irrelevant */
        }
    } else {
        /* Not timed out, so continue processing normally */
    }

    /* If a valid message has been received, process it */
    if (venus638_inbuf_parse_state == VENUS638_PARSER_DONE_MSG &&
            venus638_state == VENUS638_NAVIGATING &&
            venus638_msgbuf[0] == VENUS638_MSG_NAV_DATA) {
        venus638_process_latest_msg();
    }
}

static void venus638_process_latest_msg(void) {
    /* VENUS638_NAVIGATING holds until more than VENUS638_TIMEOUT ticks elapse
       between received packets. Approximately every
       VENUS638_NAV_INFO_FREQUENCY ticks, the GPS tracking info is included. */

    struct venus638_nav_data_t msg;
    memcpy(&msg, &(venus638_msgbuf[1]), sizeof(msg));
    if (msg.fix_mode != GPS_FIX_NONE) {
        /* msg.ecef_v is ECEF X, Y, Z in cm/s. We need to convert these to
           N, E, D in cm/s. */
        /*float_t vx = (float_t)msg.ecef_v[0],
                vy = (float_t)msg.ecef_v[1],
                vz = (float_t)msg.ecef_v[2],
                lat_ref = (PI_F / 180.0f) * (float_t)msg.latitude * 1.0e7f,
                lng_ref = (PI_F / 180.0f) * (float_t)msg.longitude * 1.0e7f,
                sin_lat = sin(lat_ref), cos_lat = cos(lat_ref),
                sin_lng = sin(lng_ref), cos_lng = cos(lng_ref),
                vn, ve, vu;

        ve = (-sin_lng * vx) + (cos_lng * vy);
        vn = (-sin_lat * cos_lng * vx) + (-sin_lat * sin_lng * vy) + (cos_lat * vz);
        vu = (cos_lat * cos_lng * vx) + (cos_lat * sin_lng * vy) + (sin_lat * vz); */

        /* Limit to 200m/s in any direction */
        /*Venus638Assert(-20000.0f < vn && vn < 20000.0f);
        Venus638Assert(-20000.0f < ve && ve < 20000.0f);
        Venus638Assert(-20000.0f < vu && vu < 20000.0f);*/

        /*comms_set_gps_pv(msg.latitude, msg.longitude, msg.msl_altitude,
            (int32_t)vn, (int32_t)ve, (int32_t)(-vu));*/
        comms_set_gps_pv(msg.latitude, msg.longitude, msg.msl_altitude,
            msg.ecef_v[0], msg.ecef_v[1], msg.ecef_v[2]);
    }

    comms_set_gps_info(msg.fix_mode, msg.pdop, msg.num_satellites);
    venus638_state_timer = 0;
}
