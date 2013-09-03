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
#include "ubx_gps.h"
#include "failsafe.h"

static void ubx_process_latest_msg(void);

enum ubx_state_t {
    UBX_POWERING_UP = 0,
    UBX_NAVIGATING,
    UBX_POWERING_DOWN
};

enum ubx_msg_parser_state_t {
    UBX_PARSER_NO_MSG = 0,
    UBX_PARSER_PREFIX_B5,
    UBX_PARSER_PREFIX_62,
    UBX_PARSER_MSG_CLASS,
    UBX_PARSER_MSG_ID,
    UBX_PARSER_LENGTH_0,
    UBX_PARSER_LENGTH_1,
    UBX_PARSER_DONE_MSG
};

struct ubx_nav_sol_t {
    /* Actual TOW in seconds is itow * 1e-3 + ftow * 1e-9 */
    uint32_t itow; /* GPS time of week in ms */
    int32_t ftow; /* GPS time of week fractional in ns (+/- 500000) */
    int16_t week; /* GPS week number */
    /* 0x00 = no fix, 0x01 = dead reckoning, 0x02 = 2D, 0x03 = 3D,
       0x04 = GPS + dead reckoning, 0x05 = time only */
    uint8_t gps_fix;
    /* 0LSB = GPS fix OK, 1LSB = differential, 2LSB = valid week, 3LSB = valid
       time of week */
    uint8_t flags;
    int32_t ecef_pos[3]; /* X, Y, Z in cm */
    uint32_t p_acc; /* position accuracy, in cm error */
    int32_t ecef_v[3]; /* X, Y, Z in cm/s */
    uint32_t s_acc; /* speed accuracy, in cm/s error */
    uint16_t pdop; /* PDOP, 1LSB = 0.01 */
    uint8_t reserved1;
    uint8_t num_sv; /* number of sats tracked */
    uint32_t reserved2;
} __attribute__ ((packed));

struct ubx_nav_posllh_t {
    uint32_t itow; /* GPS time of week in ms */
    int32_t lon; /* in 1e-7 deg */
    int32_t lat; /* in 1e-7 deg */
    int32_t height; /* in mm above ellipsoid */
    int32_t h_msl; /* in mm above mean sea level */
    uint32_t h_acc; /* horizontal accuracy, in mm error */
    uint32_t v_acc; /* vertical accuracy, in mm error */
} __attribute__ ((packed));

#define UBX_INBUF_SIZE 128u
#define UBX_MSGBUF_SIZE 64u
/* 2 prefix u8, 1 message class u8, 1 message ID u8, 1 paylod length u16,
   one checksum u16 */
#define UBX_PREFIX_LEN 6u
#define UBX_SUFFIX_LEN 2u
#define UBX_MSG_OVERHEAD (UBX_PREFIX_LEN + UBX_SUFFIX_LEN)

/* Timeout values */
#define UBX_POWER_DELAY 500u
#define UBX_TIMEOUT 1500u
#define UBX_NAV_INFO_FREQUENCY 500u

/* GPS class and message IDs */
#define UBX_MSG_NAV_SOL "\x01\x06"
#define UBX_MSG_NAV_POSLLH "\x01\x02"

/* Sanity checks */
#define Ubx_state_is_valid(x) \
    (UBX_POWERING_UP <= (x) && (x) <= UBX_POWERING_DOWN)
#define Ubx_parser_state_is_valid(x) \
    (UBX_PARSER_NO_MSG <= (x) && (x) <= UBX_PARSER_DONE_MSG)

static enum ubx_state_t ubx_state = UBX_POWERING_DOWN;
static uint32_t ubx_state_timer; /* tracks time in current state */

static uint8_t ubx_inbuf[UBX_INBUF_SIZE];
static uint8_t ubx_msgbuf[UBX_MSGBUF_SIZE];

static enum ubx_msg_parser_state_t ubx_inbuf_parse_state =
    UBX_PARSER_NO_MSG;
static uint8_t ubx_inbuf_msg_len_remaining, ubx_inbuf_idx, ubx_inbuf_msg_ck_a,
    ubx_inbuf_msg_ck_b, ubx_msgbuf_idx, ubx_inbuf_msg_class, ubx_inbuf_msg_id;

static enum gps_fix_mode_t ubx_last_fix_mode = GPS_FIX_NONE;

/* Check GPS message types */
#define Ubx_got_msg(msg_id) (ubx_inbuf_msg_class == msg_id[0] && \
    ubx_inbuf_msg_id == msg_id[1])

#ifndef CONTINUE_ON_ASSERT
#define UbxAssert(x) Assert(x)
#else
#define UbxAssert(x) if (!(x)) { ubx_state_timer = 0xffffu; }
#endif

static inline void ubx_state_transition(enum ubx_state_t new_state) {
    UbxAssert(new_state != ubx_state);
    UbxAssert(Ubx_state_is_valid(new_state));

    ubx_state = new_state;
    ubx_state_timer = 0;
}

void ubx_init(void) {
    ubx_state_timer = 0;
    ubx_inbuf_parse_state = UBX_PARSER_NO_MSG;
    ubx_inbuf_msg_len_remaining = 0;
    ubx_inbuf_idx = 0;
    ubx_msgbuf_idx = 0;

    /* USART GPIO pin configuration and sysclk setup */
    gpio_enable_module_pin(GPS_USART_RXD_PIN, GPS_USART_RXD_FUNCTION);
    gpio_enable_module_pin(GPS_USART_TXD_PIN, GPS_USART_TXD_FUNCTION);
    sysclk_enable_pba_module(GPS_USART_SYSCLK);

    /* Configure GPS power enable */
    gpio_configure_pin(GPS_ENABLE_PIN, GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

    /* Enable USART -- start in 57600 baud mode */
    usart_options_t usart_options;
    usart_options.baudrate = 57600u;
    usart_options.charlength = 8u;
    usart_options.paritytype = USART_NO_PARITY;
    usart_options.stopbits = USART_1_STOPBIT;
    usart_options.channelmode = USART_NORMAL_CHMODE;
    GPS_USART->idr = 0xFFFFFFFFu;
    uint32_t result = usart_init_rs232(GPS_USART, &usart_options,
        sysclk_get_pba_hz());
    UbxAssert(result == USART_SUCCESS);
}

/* ubx_tick is called once per millisecond to handle message parsing and
   periodic tasks */
void ubx_tick(void) {
    UbxAssert(Ubx_state_is_valid(ubx_state));
    UbxAssert(Ubx_parser_state_is_valid(ubx_inbuf_parse_state));

    comms_set_gps_state((uint8_t)ubx_state);

    ubx_state_timer++;

    /* Parse messages appearing in the input buffer */
    volatile avr32_pdca_channel_t *pdca_channel =
        &AVR32_PDCA.channel[PDCA_CHANNEL_GPS_RX];

    /* Reset message done flag */
    if (ubx_inbuf_parse_state == UBX_PARSER_DONE_MSG) {
        ubx_inbuf_parse_state = UBX_PARSER_NO_MSG;
    }

    uint32_t inbuf_bytes_read = UBX_INBUF_SIZE - pdca_channel->tcr;
    uint8_t bytes_avail = 0;

    UbxAssert(inbuf_bytes_read <= UBX_INBUF_SIZE);

    /* Work out the number of bytes available in the ring buffer */
    if (inbuf_bytes_read > ubx_inbuf_idx) {
        bytes_avail = inbuf_bytes_read - ubx_inbuf_idx;
    } else if (inbuf_bytes_read < ubx_inbuf_idx) {
        bytes_avail = (UBX_INBUF_SIZE - ubx_inbuf_idx - 1u) +
            inbuf_bytes_read;
    } else {
        /* inbuf_bytes_read == ubx_inbuf_idx, i.e. no new bytes read */
        bytes_avail = 0;
    }

    if (bytes_avail > UBX_INBUF_SIZE / 2u) {
        /* Either the PDCA isn't initialized, or there's been a possible
           buffer overflow -- either way, re-initialize the RX PDCA channel. */
        inbuf_bytes_read = 0;
        bytes_avail = 0;
        ubx_inbuf_idx = 0;
        ubx_inbuf_parse_state = UBX_PARSER_NO_MSG;

        irqflags_t flags = cpu_irq_save();
        pdca_channel->cr = AVR32_PDCA_TDIS_MASK;
        pdca_channel->mar = (uint32_t)ubx_inbuf;
        pdca_channel->tcr = UBX_INBUF_SIZE;
        pdca_channel->marr = (uint32_t)ubx_inbuf;
        pdca_channel->tcrr = UBX_INBUF_SIZE;
        pdca_channel->psr = GPS_USART_PDCA_PID_RX;
        pdca_channel->mr = (AVR32_PDCA_BYTE << AVR32_PDCA_SIZE_OFFSET)
            | (1 << AVR32_PDCA_RING_OFFSET);
        pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
        pdca_channel->isr;
        cpu_irq_restore(flags);
    }

    for (; bytes_avail && ubx_inbuf_parse_state != UBX_PARSER_DONE_MSG;
            bytes_avail--) {
        uint8_t ch = ubx_inbuf[ubx_inbuf_idx];
        ubx_inbuf_idx = (ubx_inbuf_idx + 1) & (UBX_INBUF_SIZE-1);

        if (ubx_inbuf_parse_state == UBX_PARSER_NO_MSG && ch == 0xb5u) {
            /* Got the first byte of a message prefix */
            ubx_inbuf_parse_state = UBX_PARSER_PREFIX_B5;
        } else if (ubx_inbuf_parse_state == UBX_PARSER_PREFIX_B5 && ch == 0x62u) {
            /* Second byte of a message prefix */
            ubx_inbuf_parse_state = UBX_PARSER_PREFIX_62;
        } else if (ubx_inbuf_parse_state == UBX_PARSER_PREFIX_62) {
            /* Got a class ID */
            ubx_inbuf_msg_ck_a = ubx_inbuf_msg_ck_b = 0;
            ubx_inbuf_msg_class = ch;
            ubx_inbuf_parse_state = UBX_PARSER_MSG_CLASS;

            ubx_inbuf_msg_ck_a += ch;
            ubx_inbuf_msg_ck_b += ubx_inbuf_msg_ck_a;
        } else if (ubx_inbuf_parse_state == UBX_PARSER_MSG_CLASS) {
            /* Got a message ID */
            ubx_inbuf_msg_id = ch;
            ubx_inbuf_parse_state = UBX_PARSER_MSG_ID;

            ubx_inbuf_msg_ck_a += ch;
            ubx_inbuf_msg_ck_b += ubx_inbuf_msg_ck_a;
        } else if (ubx_inbuf_parse_state == UBX_PARSER_MSG_ID &&
				ch <= UBX_MSGBUF_SIZE - UBX_SUFFIX_LEN) {
            /* Got the first length byte */
            ubx_inbuf_parse_state = UBX_PARSER_LENGTH_0;

			/* Read the lower byte of the message length */
            ubx_inbuf_msg_len_remaining = ch + UBX_SUFFIX_LEN;
            ubx_msgbuf_idx = 0;

            ubx_inbuf_msg_ck_a += ch;
            ubx_inbuf_msg_ck_b += ubx_inbuf_msg_ck_a;
        } else if (ubx_inbuf_parse_state == UBX_PARSER_LENGTH_0 &&
				ch == 0x00) {
            ubx_inbuf_parse_state = UBX_PARSER_LENGTH_1;

            ubx_inbuf_msg_ck_a += ch;
            ubx_inbuf_msg_ck_b += ubx_inbuf_msg_ck_a;
        } else if (ubx_inbuf_parse_state == UBX_PARSER_LENGTH_1) {
            /* Copy the current character into the buffer, and update the CRC
               value if the character isn't part of the message suffix */
            ubx_msgbuf[ubx_msgbuf_idx] = ch;
            ubx_msgbuf_idx++;

            if (ubx_inbuf_msg_len_remaining > UBX_SUFFIX_LEN) {
                ubx_inbuf_msg_ck_a += ch;
                ubx_inbuf_msg_ck_b += ubx_inbuf_msg_ck_a;
            }
            ubx_inbuf_msg_len_remaining--;

            if (ubx_inbuf_msg_len_remaining == 0u) {
                /* Received all the message data -- confirm the checksum bytes
                   are correct. */
                if (ubx_inbuf_msg_ck_a == ubx_msgbuf[ubx_msgbuf_idx - 2] &&
                        ubx_inbuf_msg_ck_b == ubx_msgbuf[ubx_msgbuf_idx - 1]) {
                    ubx_inbuf_parse_state = UBX_PARSER_DONE_MSG;
                } else {
                    ubx_inbuf_parse_state = UBX_PARSER_NO_MSG;
                }
            }
        } else {
            ubx_inbuf_parse_state = UBX_PARSER_NO_MSG;
        }
    }

    /* GPS driver state processing */
    if (ubx_state_timer > UBX_TIMEOUT) {
        /* Power GPS down */
        gpio_local_clr_gpio_pin(GPS_ENABLE_PIN);
        ubx_state_transition(UBX_POWERING_DOWN);
    } else if (ubx_state_timer > UBX_POWER_DELAY) {
        if (ubx_state == UBX_POWERING_DOWN) {
            /* The UBX_POWERING_DOWN state holds for 500ms, powers the GPS
               unit back up, then transfers to UBX_POWERING_UP. */
            gpio_local_set_gpio_pin(GPS_ENABLE_PIN);
            ubx_state_transition(UBX_POWERING_UP);
        } else if (ubx_state == UBX_POWERING_UP) {
            /* The UBX_POWERING_UP holds for 500ms, then transitions to the
               navigating state. */
            ubx_last_fix_mode = GPS_FIX_NONE;
            ubx_state_transition(UBX_NAVIGATING);
        } else {
            /* Not in a powering up/down state, so UBX_POWER_DELAY is
               irrelevant */
        }
    } else {
        /* Not timed out, so continue processing normally */
    }

    /* If a valid message has been received, process it */
    if (ubx_inbuf_parse_state == UBX_PARSER_DONE_MSG &&
            ubx_state == UBX_NAVIGATING) {
        ubx_process_latest_msg();
    }
}

static void ubx_process_latest_msg(void) {
    /* UBX_NAVIGATING holds until more than UBX_TIMEOUT ticks elapse
       between received packets. Approximately every
       UBX_NAV_INFO_FREQUENCY ticks, the GPS tracking info is included. */
    if (Ubx_got_msg(UBX_MSG_NAV_SOL)) {
        struct ubx_nav_sol_t msg;
        memcpy(&msg, ubx_msgbuf, sizeof(msg));

        /* Translate fix modes */
        if (msg.gps_fix == 0x02) {
            ubx_last_fix_mode = GPS_FIX_2D;
        } else if (msg.gps_fix == 0x03 || msg.gps_fix == 0x04) {
            ubx_last_fix_mode = GPS_FIX_3D;
        } else {
            ubx_last_fix_mode = GPS_FIX_NONE;
        }

        if (ubx_state_timer >= UBX_NAV_INFO_FREQUENCY) {
            ubx_state_timer = 0;
            comms_set_gps_info(ubx_last_fix_mode, swap16(msg.pdop),
                msg.num_sv);
        }

        if (ubx_last_fix_mode != GPS_FIX_NONE) {
            /* FIXME */
            comms_set_gps_pv(swap32(msg.ecef_pos[0]), swap32(msg.ecef_pos[1]),
                swap32(msg.ecef_pos[2]), swap32(msg.ecef_v[0]),
                swap32(msg.ecef_v[1]), swap32(msg.ecef_v[2]));
        }
    } else if (Ubx_got_msg(UBX_MSG_NAV_POSLLH) &&
            ubx_last_fix_mode != GPS_FIX_NONE) {
        /* If we're currently fixed, use the LLH position messages to
           supply the failsafe boundary checker with data */
        struct ubx_nav_posllh_t msg;
        memcpy(&msg, ubx_msgbuf, sizeof(msg));
        failsafe_set_latlng(swap32(msg.lat), swap32(msg.lon));
    } else {
        /* Some other message type; ignore */
    }
}
