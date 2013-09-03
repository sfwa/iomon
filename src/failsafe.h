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

#ifndef _FAILSAFE_H_
#define _FAILSAFE_H_

#define FAILSAFE_EVENT_CPU_TIMEOUT_TICKS 100u
#define FAILSAFE_EVENT_CPU_FAILURE_TICKS 1000u
#define FAILSAFE_EVENT_GPS_TIMEOUT_TICKS 30000u
#define FAILSAFE_EVENT_GPS_FAILURE_TICKS 120000u

enum failsafe_event_t {
    FAILSAFE_EVENT_NONE = 0,
    FAILSAFE_EVENT_CPU_TIMEOUT,     /* Triggered after 0.1s without CPU */
    FAILSAFE_EVENT_CPU_FAILURE,     /* Triggered after 1s without CPU */
    FAILSAFE_EVENT_GPS_TIMEOUT,     /* Triggered after 30s without GPS */
    FAILSAFE_EVENT_GPS_FAILURE,     /* Triggered after 2m without GPS */
    FAILSAFE_EVENT_ASSERT_FAILURE,
    FAILSAFE_EVENT_BOUNDARY_CROSSED, /* Triggered 1s after boundary crossing */
    FAILSAFE_EVENT_GROUND_CONTACT,   /* Triggered when ground sensor = 0 dist */
    FAILSAFE_EVENT_MANUAL_ABORT,

    /* For array sizing */
    FAILSAFE_EVENT_COUNT
};

enum failsafe_termination_mode_t {
    FAILSAFE_MODE_DEFAULT = 0, /* Equivalent to controlled flight into terrain */
    FAILSAFE_MODE_CFT,
    FAILSAFE_MODE_IGNORE,
    FAILSAFE_MODE_RESTART,
    FAILSAFE_MODE_DISABLE_PWM,

    FAILSAFE_MODE_COUNT
};

void failsafe_init(void);

/*
 * Sets the mission boundary polygon. Coordinates must be specified as
 * lat0, long0, lat1, long1, ..., latN, longN. Up to 30 pairs of (lat,long)
 * coordinates may be used. Coordinates must be given in clockwise order.
 */
void failsafe_set_boundary(int32_t coords[], size_t num_coords);

/*
 * Set failsafe termination mode on a per-event basis.
 */
void failsafe_set_termination_mode(enum failsafe_event_t event,
    enum failsafe_termination_mode_t mode);

/*
 * Set current lat/long from GPS -- must be called at least every
 * 30s or FAILSAFE_EVENT_GPS_TIMEOUT will be triggered.
 */
void failsafe_set_latlng(int32_t lat, int32_t lng);

/*
 * Indicate to the failsafe code that a CPU packet has been received.
 */
void failsafe_got_cpu_packet(void);

/*
 * Trigger a failsafe event.
 */
void failsafe_trigger_event(enum failsafe_event_t event);

/*
 * Process periodic failsafe tasks.
 */
void failsafe_tick(void);

#endif
