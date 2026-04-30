/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * @file main.c
 * @brief Peripheral: nRF52833 SAADC with low power RTC-triggered sampling.
 *
 * Compatibility: nRF52833, nRF5 SDK 17.1.0
 * Softdevice used: No softdevice
 *
 * This example uses the RTC timer to periodically trigger SAADC sampling on AIN0 (P0.02).
 * RTC is chosen for its low power consumption.
 *
 * Features:
 * - Low Power: SAADC low power mode enabled.
 * - Oversampling: Reduces SAADC noise. See SAADC_OVERSAMPLE.
 * - BURST mode: When enabled, all oversamples are taken as fast as possible with one SAMPLE task trigger.
 * - Offset Calibration: SAADC is periodically calibrated. Calibration interval is set by SAADC_CALIBRATION_INTERVAL.
 *
 * Board indicators:
 * LED1: SAADC sampling triggered
 * LED2: SAADC buffer full/event received
 * LED3: SAADC offset calibration complete
 */

// ========================= Includes =========================
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "boards.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_power.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// ========================= Macros & Config =========================
#define LED_FUNCTIONALITY_ENABLED       1
#define CALIBRATION_FUNCTIONALITY_ENABLED 0
#define SAADC_SAMPLE_INTERVAL_SEC       1
#define SAADC_SAMPLE_INTERVAL_MS        (SAADC_SAMPLE_INTERVAL_SEC * 1000)
#define RTC_FREQUENCY                   32
#define RTC_CC_VALUE                    8
#define SAADC_CALIBRATION_INTERVAL      30
#define SAADC_SAMPLES_IN_BUFFER         1
#define SAADC_OVERSAMPLE                NRF_SAADC_OVERSAMPLE_DISABLED
#define SAADC_BURST_MODE                0
#define SAADC_RESOLUTION_BITS           12
#define SAADC_REFERENCE_VOLTAGE         0.6f
#define SAADC_GAIN                      NRF_SAADC_GAIN1_6
#define VOLTAGE_DIVIDER_RATIO           6.0f

// ========================= Globals =========================
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);
static uint32_t rtc_ticks = RTC_US_TO_TICKS(SAADC_SAMPLE_INTERVAL_MS * 1000, RTC_FREQUENCY);
static nrf_saadc_value_t m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static uint32_t m_adc_evt_counter = 0;
static bool m_saadc_calibrate = false;

#if CALIBRATION_FUNCTIONALITY_ENABLED
#define SHOULD_CALIBRATE() ((m_adc_evt_counter % SAADC_CALIBRATION_INTERVAL) == 0)
#else
#define SHOULD_CALIBRATE() (false)
#endif

// ========================= Function Prototypes =========================
static float calculate_voltage_mV(nrf_saadc_value_t sample);
static void rtc_handler(nrf_drv_rtc_int_type_t int_type);
static void lfclk_config(void);
static void rtc_config(void);
void saadc_callback(nrf_drv_saadc_evt_t const * p_event);
void saadc_init(void);
void init_log(void);
int main(void);

// ========================= Utility Functions =========================
static float calculate_voltage_mV(nrf_saadc_value_t sample)
{
    return ((float)sample / ((1 << SAADC_RESOLUTION_BITS) - 1)) * SAADC_REFERENCE_VOLTAGE * 1000 * VOLTAGE_DIVIDER_RATIO;
}

// ========================= RTC & Clock Functions =========================
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    uint32_t err_code;

    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        nrf_drv_saadc_sample();
#if LED_FUNCTIONALITY_ENABLED
        LEDS_INVERT(BSP_LED_0_MASK);
#endif
        err_code = nrf_drv_rtc_cc_set(&rtc, 0, rtc_ticks, true);
        APP_ERROR_CHECK(err_code);
        nrf_drv_rtc_counter_clear(&rtc);
    }
}

static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

static void rtc_config(void)
{
    uint32_t err_code;
    nrf_drv_rtc_config_t rtc_config;
    rtc_config.prescaler = RTC_FREQ_TO_PRESCALER(RTC_FREQUENCY);
    err_code = nrf_drv_rtc_init(&rtc, &rtc_config, rtc_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_rtc_cc_set(&rtc, 0, rtc_ticks, true);
    APP_ERROR_CHECK(err_code);
    nrf_drv_rtc_enable(&rtc);
}

// ========================= SAADC Functions =========================
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    ret_code_t err_code;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
#if LED_FUNCTIONALITY_ENABLED
        LEDS_INVERT(BSP_LED_1_MASK);
#endif
        if (SHOULD_CALIBRATE())
        {
#if CALIBRATION_FUNCTIONALITY_ENABLED
            m_saadc_calibrate = true;
#endif
        }

        NRF_LOG_INFO("ADC event number: %d\r\n", (int)m_adc_evt_counter);
        for (int i = 0; i < p_event->data.done.size; i++)
        {
            float voltage_mV = calculate_voltage_mV(p_event->data.done.p_buffer[i]);
            NRF_LOG_INFO("%d (%d mV)\r\n", p_event->data.done.p_buffer[i], (int)voltage_mV);
        }

        if (m_saadc_calibrate == false)
        {
            err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);
            APP_ERROR_CHECK(err_code);
        }

        m_adc_evt_counter++;
    }
#if CALIBRATION_FUNCTIONALITY_ENABLED
    else if (p_event->type == NRF_DRV_SAADC_EVT_CALIBRATEDONE)
    {
#if LED_FUNCTIONALITY_ENABLED
        LEDS_INVERT(BSP_LED_2_MASK);
#endif
        err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
        err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAADC_SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_INFO("SAADC calibration complete ! \r\n");
    }
#endif
}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config;

    saadc_config.low_power_mode = true;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;
    saadc_config.oversample = SAADC_OVERSAMPLE;
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
    channel_config.gain = NRF_SAADC_GAIN1_6;
    channel_config.acq_time = NRF_SAADC_ACQTIME_10US;
    channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;
    if (SAADC_BURST_MODE)
    {
        channel_config.burst = NRF_SAADC_BURST_ENABLED;
    }
    channel_config.pin_p = NRF_SAADC_INPUT_AIN0;
    channel_config.pin_n = NRF_SAADC_INPUT_DISABLED;
    channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

// ========================= Logging Functions =========================
void init_log(void)
{
    ret_code_t err_code;
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_DEBUG("Logging initialized\r\n");
}

// ========================= Main Application =========================
int main(void)
{
#if LED_FUNCTIONALITY_ENABLED
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
#endif
    NRF_POWER->DCDCEN = 1;
    init_log();
    NRF_LOG_INFO("SAADC Low Power Example.");
    lfclk_config();
    rtc_config();
    saadc_init();
    while (1)
    {
#if CALIBRATION_FUNCTIONALITY_ENABLED
        if (m_saadc_calibrate == true)
        {
            nrf_drv_saadc_abort();
            NRF_LOG_INFO("SAADC calibration starting...");
            while (nrf_drv_saadc_calibrate_offset() != NRF_SUCCESS);
            m_saadc_calibrate = false;
        }
#endif
        while (NRF_LOG_PROCESS() != NRF_SUCCESS);
        NRF_LOG_FLUSH();
        nrf_pwr_mgmt_run();
    }
}

/** @} */
