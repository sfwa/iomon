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
#include <string.h>
#include "pwm.h"
#include "coord.h"
#include "failsafe.h"

#ifndef CONTINUE_ON_ASSERT
#define FSAssert(x) Assert(x)
#else
#define FSAssert(x) if (!(x)) { failsafe_enter_cft(); }
#endif

static void failsafe_enter_cft(void);

#define MB_MAX_POINTS 30u
/* No more than 1 second outside the MB before event is triggered */
#define MAX_MB_EXCEED_PACKETS 10u

/* Mission boundary definition */
static int32_t mb_lats[MB_MAX_POINTS];
static int32_t mb_longs[MB_MAX_POINTS];
static uint8_t mb_num_points = 0;
static int16_t mb_exceed_packets = 0;

static uint32_t failsafe_tick_count = 0;
static uint32_t failsafe_last_gps = 0;
static uint32_t failsafe_last_cpu = 0;

/* Track which events should be associated with which responses */
static enum failsafe_termination_mode_t
    failsafe_event_response[FAILSAFE_EVENT_COUNT];

static void failsafe_enter_cft(void) {
    /* Infinite loop to lock out any possibility of recovery -- reset the WDT
       each time as well otherwise the system will restart itself */
    static uint16_t pwm_values[PWM_NUM_OUTPUTS];
    memset(pwm_values, 0x0001, sizeof(pwm_values));

    LED_ON(LED3_GPIO);

    irqflags_t flags = cpu_irq_save();
    /* Take control of the PWM */
    pwm_enable();
    for (;;) {
        pwm_set_values(pwm_values);
        wdt_clear();
    }
    cpu_irq_restore(flags);

    LED_OFF(LED3_GPIO);
}

void failsafe_init(void) {
    for (uint8_t i = 0; i < FAILSAFE_EVENT_COUNT; i++) {
        failsafe_event_response[i] = FAILSAFE_MODE_CFT;
    }

    failsafe_event_response[FAILSAFE_EVENT_CPU_TIMEOUT] = FAILSAFE_MODE_DISABLE_PWM;
	failsafe_event_response[FAILSAFE_EVENT_CPU_FAILURE] = FAILSAFE_MODE_DISABLE_PWM;
    failsafe_event_response[FAILSAFE_EVENT_GPS_TIMEOUT] = FAILSAFE_MODE_IGNORE;
    failsafe_event_response[FAILSAFE_EVENT_GPS_FAILURE] = FAILSAFE_MODE_IGNORE;
    failsafe_event_response[FAILSAFE_EVENT_GROUND_CONTACT] = FAILSAFE_MODE_IGNORE;
}

void failsafe_set_boundary(int32_t coords[], size_t num_coords) {
    FSAssert(num_coords <= MB_MAX_POINTS);

    for (uint8_t i = 0; i < num_coords; i++) {
        mb_lats[i] = coords[(i << 1)];
        mb_longs[i] = coords[(i << 1) + 1];
    }
    mb_num_points = num_coords;
}

void failsafe_set_termination_mode(enum failsafe_event_t event,
    enum failsafe_termination_mode_t mode) {

    FSAssert(FAILSAFE_EVENT_NONE <= event && event < FAILSAFE_EVENT_COUNT);
    FSAssert(FAILSAFE_MODE_DEFAULT <= mode && mode < FAILSAFE_MODE_COUNT);

    failsafe_event_response[event] = mode;
}

void failsafe_set_latlng(int32_t lat, int32_t lng) {
    failsafe_last_gps = failsafe_tick_count;

    if (is_latlng_in_poly(lat, lng, mb_num_points, mb_lats, mb_longs) ==
            OUT_OF_BOUNDS) {
        mb_exceed_packets += 1;

        if (mb_exceed_packets >= MAX_MB_EXCEED_PACKETS) {
            failsafe_trigger_event(FAILSAFE_EVENT_BOUNDARY_CROSSED);
        }
    } else {
        mb_exceed_packets = 0;
    }
}

void failsafe_trigger_event(enum failsafe_event_t event) {
    FSAssert(FAILSAFE_EVENT_NONE <= event && event < FAILSAFE_EVENT_COUNT);

    enum failsafe_termination_mode_t mode = failsafe_event_response[event];

    FSAssert(FAILSAFE_MODE_DEFAULT <= mode && mode < FAILSAFE_MODE_COUNT);

    if (mode == FAILSAFE_MODE_RESTART) {
        /* Restart CPU */
        wdt_reset_mcu();
    } else if (mode == FAILSAFE_MODE_IGNORE) {
        /* Do nothing */
    } else if (mode == FAILSAFE_MODE_DISABLE_PWM) {
        /* Turn PWM off until we get another CPU packet */
        pwm_disable();
    } else {
        /* Any unimplemented mode results in CFT */
        failsafe_enter_cft();
    }
}

void failsafe_got_cpu_packet(void) {
    failsafe_last_cpu = failsafe_tick_count;
}

void failsafe_tick(void) {
    failsafe_tick_count++;

    uint32_t cpu_delta = failsafe_tick_count - failsafe_last_cpu;
    if (cpu_delta > FAILSAFE_EVENT_CPU_FAILURE_TICKS) {
        failsafe_trigger_event(FAILSAFE_EVENT_CPU_FAILURE);
    } else if (cpu_delta > FAILSAFE_EVENT_CPU_TIMEOUT_TICKS) {
        failsafe_trigger_event(FAILSAFE_EVENT_CPU_TIMEOUT);
    } else {
        /* No timeout condition */
    }

    uint32_t gps_delta = failsafe_tick_count - failsafe_last_gps;
    if (gps_delta > FAILSAFE_EVENT_GPS_FAILURE_TICKS) {
        failsafe_trigger_event(FAILSAFE_EVENT_GPS_FAILURE);
    } else if (gps_delta > FAILSAFE_EVENT_GPS_TIMEOUT_TICKS) {
        failsafe_trigger_event(FAILSAFE_EVENT_GPS_TIMEOUT);
    } else {
        /* No timeout condition */
    }
}
