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
#include <adcifa/adcifa.h>
#include "fcsassert.h"
#include "comms.h"
#include "gp.h"
#include "plog/parameter.h"

#define GP_NUM_INPUTS 4u
#define GP_NUM_OUTPUTS 4u
#define GP_NUM_ADCS 4u
#define GP_ADC_BUF_SIZE (GP_NUM_ADCS * 4u * 2u)

#define GP_ADC_PITOT 0u
#define GP_ADC_BATTERY_I 1u
#define GP_ADC_BATTERY_V 2u
#define GP_ADC_AUX 3u

static const uint32_t gp_input_pins[GP_NUM_INPUTS] =
    {GPIN_0_PIN, GPIN_1_PIN, GPIN_2_PIN, GPIN_3_PIN};
static const uint32_t gp_output_pins[GP_NUM_OUTPUTS] =
    {GPOUT_0_PIN, GPOUT_1_PIN, GPOUT_2_PIN, GPOUT_3_PIN};
/* Interleaved samples are stored in this buffer (i.e. 0 1 2 3 0 1 2 3) by
   the PDCA. The gp_adc_last_sample_idx value contains the index of the last
   ADC sample read. */
static int16_t gp_adc_samples[GP_ADC_BUF_SIZE * 2u];
static uint32_t gp_adc_last_sample_idx;

static void gp_set_pins(uint32_t pin_values);
static uint32_t gp_get_pins(void);


void gp_init(void) {
    /* Set GPIO pin configuration on outputs */
    for (uint8_t i = 0; i < GP_NUM_OUTPUTS; i++) {
        gpio_configure_pin(gp_output_pins[i], GPIO_DIR_OUTPUT);
    }

    /*
    Enable pull-ups on GPIN 0, the paylod presence detect, which pulls the
    line low when active.
    */
    gpio_configure_pin(gp_input_pins[0], GPIO_DIR_INPUT | GPIO_PULL_UP);
    gpio_configure_pin(gp_input_pins[1], GPIO_DIR_INPUT | GPIO_PULL_UP);
    gpio_configure_pin(gp_input_pins[2], GPIO_DIR_INPUT | GPIO_PULL_UP);
    gpio_configure_pin(gp_input_pins[3], GPIO_DIR_INPUT | GPIO_PULL_UP);

    gp_set_pins(0);

    /* Configure the pins connected to LEDs as output and set their default
       initial state to high (LEDs on). */
    gpio_configure_pin(LED0_GPIO, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
    gpio_configure_pin(LED1_GPIO, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
    gpio_configure_pin(LED2_GPIO, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
    gpio_configure_pin(LED3_GPIO, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);

	return;

    /* Initialize ADCs */

    /* Set GPIOs for channels 0-3 */
    sysclk_enable_pbc_module(GP_ADC_SYSCLK);
    gpio_enable_module_pin(ADC_PITOT_PIN, ADC_PITOT_FUNCTION);
    gpio_enable_module_pin(ADC_AUX_PIN, ADC_AUX_FUNCTION);
    gpio_enable_module_pin(ADC_BATTERY_V_PIN, ADC_BATTERY_V_FUNCTION);
    gpio_enable_module_pin(ADC_BATTERY_I_PIN, ADC_BATTERY_I_FUNCTION);

    /* Just in case */
    ADCIFA_disable();

    adcifa_opt_t adc_opts;
    adcifa_sequencer_opt_t seq_opts;
    adcifa_sequencer_conversion_opt_t conv_opts[8];

    /* Read calibration from factory page in flash and write to ADCCAL.GCAL */
    adcifa_get_calibration_data(GP_ADC, &adc_opts);

    /* Set to DIRECT mode (clear SHD bit in CFG register) */
    adc_opts.sample_and_hold_disable = true;

    /* Set all conversions to occur in sequence (clear SOCB in SEQCFG0) */
    adc_opts.single_sequencer_mode = false;
    adc_opts.sleep_mode_enable = false;
    adc_opts.free_running_mode_enable = false;

    /* Set reference source to AREF1 */
    adc_opts.reference_source = ADCIFA_REF06VDD;

    /* - Set clock divider to 192 (16ks/s) */
    adc_opts.frequency = GP_NUM_ADCS * 2u * 1000u;

    /* Set sequencer to overwrite old data without acknowledge (set SA in SEQCFG0)
       Enable oversampling mode (2 clocks per sample conversion) */
    seq_opts.convnb = 8;
    seq_opts.resolution = ADCIFA_SRES_12B;
    seq_opts.trigger_selection = 3; /* continuous triggering */
    seq_opts.oversampling = 1u;
    seq_opts.software_acknowledge = ADCIFA_SA_NO_EOS_SOFTACK;
    seq_opts.start_of_conversion = ADCIFA_SOCB_ALLSEQ;
    seq_opts.half_word_adjustment = ADCIFA_HWLA_NOADJ;

    for (uint8_t i = 0; i < 8; i++) {
        conv_opts[i].channel_p = AVR32_ADCIFA_INP_ADCIN0 + (i % GP_NUM_ADCS);
        conv_opts[i].channel_n = AVR32_ADCIFA_INN_GNDANA;
        conv_opts[i].gain = ADCIFA_SHG_1;
    }
    adcifa_configure_sequencer(GP_ADC, 0, &seq_opts, conv_opts);

    /* TODO: Measure ADC offset and write to ADCCAL.OCAL ? */

    /* Configure the ADC, and enable it */
    GP_ADC->scr = 0xffffffffu;
    adcifa_configure(GP_ADC, &adc_opts, CONFIG_MAIN_HZ);

    /* Trigger SOC to begin loop */
    GP_ADC->scr = 0xffffffffu;
}

void gp_tick(void) {
    /* Copy output pin values from last comms packet if they've changed */
    struct fcs_parameter_t param;

    if (fcs_parameter_find_by_type_and_device(
            &cpu_conn.in_log, FCS_PARAMETER_GP_OUT, 0, &param)) {
        gp_set_pins(param.data.u8[0]);
    }

    /* Add input values to measurement log */
    fcs_parameter_set_header(&param, FCS_VALUE_UNSIGNED, 8u, 1u);
    fcs_parameter_set_type(&param, FCS_PARAMETER_GP_IN);
    fcs_parameter_set_device_id(&param, 0);
    param.data.u8[0] = (uint8_t)(gp_get_pins() & 0xFFu);
    (void)fcs_log_add_parameter(&cpu_conn.out_log, &param);

	return;

    /* Get ADC values from sample data */
    uint32_t adc_totals[GP_NUM_ADCS];
    uint16_t adc_sample_count[GP_NUM_ADCS];
    memset(adc_totals, 0x00, sizeof(adc_totals));
    memset(adc_sample_count, 0x00, sizeof(adc_sample_count));

    /* Loop over new samples (ring buffer), add totals to the corresponding
       adc_totals value, and increment adc_sample_count for each. Divide each
       adc_totals value by adc_sample_count. */
    volatile avr32_pdca_channel_t *pdca_channel =
        &AVR32_PDCA.channel[PDCA_CHANNEL_ADC_RX];

    uint32_t samples_read = GP_ADC_BUF_SIZE - pdca_channel->tcr;
    uint16_t samples_avail = 0;

    fcs_assert(samples_read <= GP_ADC_BUF_SIZE);

    /* Work out the number of bytes available in the ring buffer */
    if (samples_read > gp_adc_last_sample_idx) {
        samples_avail = samples_read - gp_adc_last_sample_idx;
    } else if (samples_read < gp_adc_last_sample_idx) {
        samples_avail = (GP_ADC_BUF_SIZE - gp_adc_last_sample_idx - 1) +
            samples_read;
    } else {
        /* samples_read == gp_adc_last_sample_idx, i.e. no new samples */
        samples_avail = 0;
    }

    if (samples_avail > GP_ADC_BUF_SIZE - 1u || samples_avail == 0) {
        /* Either the PDCA isn't initialized, or there's been a possible
           buffer overflow -- either way, re-initialize the RX PDCA channel. */
        samples_read = 0;
        samples_avail = 0;
        gp_adc_last_sample_idx = 0;

        ADCIFA_disable();

        /* Configure PDCA transfer in ring buffer mode */
        pdca_channel->cr = AVR32_PDCA_TDIS_MASK;
        pdca_channel->idr = 0xFFFFFFFFu;
        pdca_channel->mar = (uint32_t)gp_adc_samples;
        pdca_channel->tcr = GP_ADC_BUF_SIZE;
        pdca_channel->marr = (uint32_t)gp_adc_samples;
        pdca_channel->tcrr = GP_ADC_BUF_SIZE;
        pdca_channel->psr = ADC_PDCA_PID_RX;
        pdca_channel->mr = (AVR32_PDCA_HALF_WORD << AVR32_PDCA_SIZE_OFFSET)
            | (1 << AVR32_PDCA_RING_OFFSET);
        pdca_channel->cr = AVR32_PDCA_ECLR_MASK | AVR32_PDCA_TEN_MASK;
        pdca_channel->isr;

        ADCIFA_enable();
    }

    for (; samples_avail; samples_avail--) {
        /* If the number of samples is greater than 64 (corresponding to
           buffer size), something has gone badly wrong above. */
        fcs_assert(
            adc_sample_count[gp_adc_last_sample_idx % GP_NUM_ADCS] <= 4);

        int16_t sample = gp_adc_samples[gp_adc_last_sample_idx];
        if (sample < 0) { /* Clamp negative samples to zero */
            sample = 0;
        } else if (sample >= 2047) { /* Clamp samples to +2047 */
            sample = 2047;
        }

        adc_totals[gp_adc_last_sample_idx % GP_NUM_ADCS] += (uint32_t)sample;
        adc_sample_count[gp_adc_last_sample_idx % GP_NUM_ADCS]++;
        gp_adc_last_sample_idx = (gp_adc_last_sample_idx + 1) &
            (GP_ADC_BUF_SIZE - 1);
    }

    for (uint8_t i = 0; i < GP_NUM_ADCS; i++) {
        /* Average and convert each value to the range [0, 32752) */
        if (adc_sample_count[i] > 0) {
            adc_totals[i] <<= 4;
            adc_totals[i] /= adc_sample_count[i];
        } else {
            adc_totals[i] = 0;
        }
        fcs_assert(adc_totals[i] <= 32752);
    }

    /* Add ADC readings to measurement log */
    fcs_parameter_set_header(&param, FCS_VALUE_UNSIGNED, 16u, 2u);
    fcs_parameter_set_type(&param, FCS_PARAMETER_IV);
    fcs_parameter_set_device_id(&param, 0);
    param.data.u16[0] = swap_u16(adc_totals[GP_ADC_BATTERY_I]);
    param.data.u16[1] = swap_u16(adc_totals[GP_ADC_BATTERY_V]);
    (void)fcs_log_add_parameter(&cpu_conn.out_log, &param);
}

static void gp_set_pins(uint32_t pin_values) {
    for (uint8_t i = 0; i < GP_NUM_OUTPUTS; i++) {
        if (pin_values & (1u << i)) {
            gpio_local_set_gpio_pin(gp_output_pins[i]);
        } else {
            gpio_local_clr_gpio_pin(gp_output_pins[i]);
        }
    }
}

static uint32_t gp_get_pins(void) {
    uint32_t result = 0;
    for (uint8_t i = 0; i < GP_NUM_INPUTS; i++) {
        result |= gpio_local_get_pin_value(gp_input_pins[i]) << i;
    }
    return result;
}
