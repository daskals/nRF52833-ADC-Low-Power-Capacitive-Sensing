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
 * @brief Peripheral: nRF52 SAADC with low power RTC-triggered sampling and 74HC4051 multiplexer support.
 *
 * Compatibility: nRF52832 rev 1/nRF52840 Eng A, nRF5 SDK 13.0.0
 * Softdevice used: No softdevice
 *
 * This example uses the RTC timer to periodically trigger SAADC sampling. RTC is chosen for its low power consumption.
 * The example samples a single input pin (AIN0, P0.02) via a 74HC4051 multiplexer, allowing up to 8 analog channels.
 *
 * Features:
 * - Low Power: SAADC is initialized only when sampling, and uninitialized after. For lowest power, set UART_PRINTING_ENABLED undefined and SAADC_SAMPLES_IN_BUFFER to 1.
 * - Oversampling: Reduces SAADC noise, especially at higher resolutions. See SAADC_OVERSAMPLE.
 * - BURST mode: When enabled, all oversamples are taken as fast as possible with one SAMPLE task trigger.
 * - Offset Calibration: SAADC is periodically calibrated. Calibration interval is set by SAADC_CALIBRATION_INTERVAL.
 *
 * UART output: The SAADC sample result is printed via UART. Configure a UART terminal (e.g., Realterm) with the settings in uart_config().
 *
 * Board indicators:
 * LED1: SAADC sampling triggered
 * LED2: SAADC sampling buffer full/event received
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
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// ========================= Macros & Config =========================
#define LED_FUNCTIONALITY_ENABLED 1
#define CALIBRATION_FUNCTIONALITY_ENABLED 0
#define SAADC_SAMPLE_INTERVAL_SEC 1
#define SAADC_SAMPLE_INTERVAL_MS (SAADC_SAMPLE_INTERVAL_SEC * 1000)
#define RTC_FREQUENCY 32
#define RTC_CC_VALUE 8
#define SAADC_CALIBRATION_INTERVAL 30
#define SAADC_SAMPLES_IN_BUFFER 1
#define SAADC_OVERSAMPLE NRF_SAADC_OVERSAMPLE_DISABLED  //NRF_SAADC_OVERSAMPLE_4X
#define SAADC_BURST_MODE 0
#define SAADC_RESOLUTION_BITS   12
#define SAADC_REFERENCE_VOLTAGE 0.6f
#define SAADC_GAIN              NRF_SAADC_GAIN1_6
#define VOLTAGE_DIVIDER_RATIO   6.0f
#define NUM_MUX_INPUTS 8
#define MUX_S0_PIN NRF_GPIO_PIN_MAP(0, 3)
#define MUX_S1_PIN NRF_GPIO_PIN_MAP(0, 4)
#define MUX_S2_PIN NRF_GPIO_PIN_MAP(0, 5)
#define ADC_IN_PIN NRF_GPIO_PIN_MAP(0, 2)
#define MUX_ENABLE_PIN NRF_GPIO_PIN_MAP(0, 6)

// ========================= Types & Globals =========================
typedef struct {
    bool active;
    uint8_t channel;
    float results[NUM_MUX_INPUTS];
} mux_scan_t;

static mux_scan_t mux_scan = {0};
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);
static uint32_t rtc_ticks = RTC_US_TO_TICKS(SAADC_SAMPLE_INTERVAL_MS * 1000, RTC_FREQUENCY);
static nrf_saadc_value_t m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static uint32_t m_adc_evt_counter = 0;
static bool m_saadc_calibrate = false;
volatile bool scan_mux_requested = false;

#if CALIBRATION_FUNCTIONALITY_ENABLED
#define SHOULD_CALIBRATE() ((m_adc_evt_counter % SAADC_CALIBRATION_INTERVAL) == 0)
#else
#define SHOULD_CALIBRATE() (false)
#endif

// ========================= Function Prototypes =========================
static float calculate_voltage_mV(nrf_saadc_value_t sample);
void select_mux_channel(uint8_t channel);
static void handle_mux_scan(nrf_saadc_value_t sample);
static void rtc_handler(nrf_drv_rtc_int_type_t int_type);
static void lfclk_config(void);
static void rtc_config(void);
void saadc_callback(nrf_drv_saadc_evt_t const * p_event);
void gpio_init(void);
void saadc_init(void);
void init_log(void);
int main(void);

// ========================= Utility Functions =========================
/**
 * @brief Calculate the voltage in millivolts from an SAADC sample.
 *
 * @param[in] sample The raw SAADC sample value.
 * @return float The calculated voltage in millivolts.
 */
static float calculate_voltage_mV(nrf_saadc_value_t sample)
{
    return ((float)sample / ((1 << SAADC_RESOLUTION_BITS) - 1)) * SAADC_REFERENCE_VOLTAGE * 1000 * VOLTAGE_DIVIDER_RATIO;
}

/**
 * @brief Select the active channel on the 74HC4051 multiplexer.
 *
 * Sets the multiplexer control pins to select the desired channel (0-7).
 *
 * @param[in] channel Channel number to select (0-7).
 */
void select_mux_channel(uint8_t channel)
{
    nrf_gpio_pin_write(MUX_S0_PIN, channel & 0x01);
    nrf_gpio_pin_write(MUX_S1_PIN, (channel >> 1) & 0x01);
    nrf_gpio_pin_write(MUX_S2_PIN, (channel >> 2) & 0x01);
}

/**
 * @brief Handle the MUX scan process and print results when complete.
 *
 * Should be called from the SAADC callback when mux_scan.active is true.
 * Stores the result for the current channel, advances the channel, and triggers the next sample.
 * When all channels are scanned, prints the results and disables the multiplexer.
 *
 * @param[in] sample Raw SAADC sample value for the current channel.
 */
static void handle_mux_scan(nrf_saadc_value_t sample)
{
    mux_scan.results[mux_scan.channel] = calculate_voltage_mV(sample);
    mux_scan.channel++;
    if (mux_scan.channel < NUM_MUX_INPUTS) {
        select_mux_channel(mux_scan.channel);
        nrf_delay_us(10); // Allow MUX to settle
        nrf_drv_saadc_sample();
    } else {
        NRF_LOG_INFO("MUX ADC Results:");
        for (uint8_t i = 0; i < NUM_MUX_INPUTS; i++) {
            NRF_LOG_INFO("Channel %d: %d mV", i, (int)mux_scan.results[i]);
        }
        nrf_gpio_pin_clear(MUX_ENABLE_PIN); // Disable multiplexer to save power
        mux_scan.active = false;
    }
}

// ========================= RTC & Clock Functions =========================
/**
 * @brief RTC interrupt handler.
 *
 * Handles RTC compare events to trigger periodic SAADC sampling and MUX scan.
 * Enables the multiplexer, starts a scan, and resets the RTC counter.
 *
 * @param[in] int_type RTC interrupt type (e.g., NRF_DRV_RTC_INT_COMPARE0).
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    uint32_t err_code;
    
    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        nrf_gpio_pin_set(MUX_ENABLE_PIN); // Enable multiplexer (active high)
        NRF_LOG_INFO("MUX ADC Measurement------");
        mux_scan.active = true;
        mux_scan.channel = 0;
        select_mux_channel(0);
        nrf_delay_us(10); // Allow MUX to settle
        nrf_drv_saadc_sample();
#if LED_FUNCTIONALITY_ENABLED
        LEDS_INVERT(BSP_LED_0_MASK); // Toggle LED1: SAADC sampling start
#endif
        err_code = nrf_drv_rtc_cc_set(&rtc, 0, rtc_ticks, true); // Set RTC compare value
        APP_ERROR_CHECK(err_code);
        nrf_drv_rtc_counter_clear(&rtc); // Restart RTC counter
    }
}

/**
 * @brief Configure the low-frequency clock (LFCLK).
 *
 * Initializes and requests the 32kHz LFCLK source required for RTC operation.
 */
static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();                        //Initialize the clock source specified in the nrf_drv_config.h file, i.e. the CLOCK_CONFIG_LF_SRC constant
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

/**
 * @brief Configure the RTC peripheral for periodic interrupts.
 *
 * Initializes the RTC instance, sets the compare value for periodic interrupts,
 * and enables the RTC. The RTC is used to trigger periodic SAADC sampling.
 */
static void rtc_config(void)
{

    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t rtc_config;
    rtc_config.prescaler = RTC_FREQ_TO_PRESCALER(RTC_FREQUENCY);
    err_code = nrf_drv_rtc_init(&rtc, &rtc_config, rtc_handler);                //Initialize the RTC with callback function rtc_handler. The rtc_handler must be implemented in this applicaiton. Passing NULL here for RTC configuration means that configuration will be taken from the sdk_config.h file.
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_rtc_cc_set(&rtc, 0, rtc_ticks, true);                    //Set RTC compare value to trigger interrupt. Configure the interrupt frequency by adjust RTC_CC_VALUE and RTC_FREQUENCY constant in top of main.c
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);                                                   //Enable RTC
}

// ========================= SAADC Functions =========================
/**
 * @brief SAADC event callback handler.
 *
 * Handles SAADC events such as buffer full and calibration complete.
 * Processes MUX scan results, manages calibration, and logs ADC values.
 *
 * @param[in] p_event Pointer to the SAADC event structure.
 */
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    ret_code_t err_code;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)                                //Capture offset calibration complete event
    {
#if LED_FUNCTIONALITY_ENABLED
        LEDS_INVERT(BSP_LED_1_MASK);                                            //Toggle LED2 to indicate SAADC buffer full		
#endif
        if(SHOULD_CALIBRATE())
        {
#if CALIBRATION_FUNCTIONALITY_ENABLED
            m_saadc_calibrate = true;                                           // Set flag to trigger calibration in main context when SAADC is stopped
#endif
        }

        if (mux_scan.active) {
            handle_mux_scan(p_event->data.done.p_buffer[0]);
        } else {
            NRF_LOG_INFO("ADC event number: %d\r\n",(int)m_adc_evt_counter);
            for (int i = 0; i < p_event->data.done.size; i++)
            {
                float voltage_mV = calculate_voltage_mV(p_event->data.done.p_buffer[i]);
                NRF_LOG_INFO("%d (%d mV)\r\n", p_event->data.done.p_buffer[i], (int)voltage_mV);
            }
        }
        if(m_saadc_calibrate == false)
        {
            err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);             //Set buffer so the SAADC can write to it again. 
            APP_ERROR_CHECK(err_code);
        }
        
        m_adc_evt_counter++;
  
    }
#if CALIBRATION_FUNCTIONALITY_ENABLED
    else if (p_event->type == NRF_DRV_SAADC_EVT_CALIBRATEDONE)
    {
#if LED_FUNCTIONALITY_ENABLED
        LEDS_INVERT(BSP_LED_2_MASK);                                                                    //Toggle LED3 to indicate SAADC calibration complete
#endif
        err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER);             //Set buffer so the SAADC can write to it again. 
        APP_ERROR_CHECK(err_code);
        err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAADC_SAMPLES_IN_BUFFER);             //Need to setup both buffers, as they were both removed with the call to nrf_drv_saadc_abort before calibration.
        APP_ERROR_CHECK(err_code);
        
        NRF_LOG_INFO("SAADC calibration complete ! \r\n");                                              //Print on UART

    }
#endif
}

/**
 * @brief Initialize and configure the SAADC peripheral.
 *
 * Sets up the SAADC in low power mode, configures the channel, and prepares the sample buffers.
 */
void saadc_init(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config;
    // Configure SAADC
    saadc_config.low_power_mode = true; // Enable low power mode
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT; // 12-bit resolution
    saadc_config.oversample = SAADC_OVERSAMPLE; // Oversampling
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW; // Low priority interrupt
    // Initialize SAADC
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);
    // Configure SAADC channel
    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL; // 0.6V internal reference
    channel_config.gain = NRF_SAADC_GAIN1_6; // Gain 1/6 (max input 3.6V)
    channel_config.acq_time = NRF_SAADC_ACQTIME_10US; // Acquisition time
    channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED; // Single-ended mode
    if (SAADC_BURST_MODE)
    {
        channel_config.burst = NRF_SAADC_BURST_ENABLED; // Enable burst mode if configured
    }
    channel_config.pin_p = NRF_SAADC_INPUT_AIN0; // AIN0 (P0.02)
    channel_config.pin_n = NRF_SAADC_INPUT_DISABLED; // Negative pin disabled
    channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    // Initialize SAADC channel
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

// ========================= GPIO Functions =========================
/**
 * @brief Initialize GPIOs for multiplexer and SAADC input.
 *
 * Configures the MUX control and enable pins as outputs, disables the MUX initially,
 * and sets the SAADC input pin to its default (analog) state.
 */
void gpio_init(void)
{
    // Configure multiplexer control pins as outputs
    nrf_gpio_cfg_output(MUX_S0_PIN);
    nrf_gpio_cfg_output(MUX_S1_PIN);
    nrf_gpio_cfg_output(MUX_S2_PIN);
    // Configure multiplexer enable pin as output
    nrf_gpio_cfg_output(MUX_ENABLE_PIN);
    // Configure SAADC input pin (AIN0) as analog input
    nrf_gpio_cfg_default(ADC_IN_PIN); // AIN0 = P0.02
    // Disable multiplexer initially to save power
    nrf_gpio_pin_clear(MUX_ENABLE_PIN); // Set low to disable
    
}

// ========================= Logging Functions =========================
/**
 * @brief Initialize the logging module (SEGGER RTT backend).
 *
 * Sets up the logging backend and outputs a debug message.
 */
void init_log(void)
{
    ret_code_t err_code;
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_DEBUG("Logging initialized\r\n");
}

// ========================= Main Application =========================
/**
 * @brief Main application entry point.
 *
 * Initializes all peripherals, configures the system, and enters the main loop.
 * Handles periodic SAADC sampling, MUX scanning, and calibration.
 */
int main(void)
{
#if LED_FUNCTIONALITY_ENABLED
    LEDS_CONFIGURE(LEDS_MASK); // Configure all LEDs
    LEDS_OFF(LEDS_MASK);       // Turn off all LEDs
#endif
    NRF_POWER->DCDCEN = 1;     // Enable DCDC converter for lower current
    init_log();                // Initialize logging
    gpio_init();               // Initialize GPIOs
    NRF_LOG_INFO("SAADC Low Power Example.");
    lfclk_config();            // Configure 32kHz low frequency clock
    rtc_config();              // Configure RTC for periodic interrupts
    saadc_init();              // Initialize and start SAADC
    while (1)
    {
#if CALIBRATION_FUNCTIONALITY_ENABLED
        if (m_saadc_calibrate == true)
        {
            nrf_drv_saadc_abort(); // Abort ongoing conversions before calibration
            NRF_LOG_INFO("SAADC calibration starting...");
            while (nrf_drv_saadc_calibrate_offset() != NRF_SUCCESS);
            m_saadc_calibrate = false;
        }
#endif
        while (NRF_LOG_PROCESS() != NRF_SUCCESS);
        NRF_LOG_FLUSH(); // Ensure all log messages are output
        nrf_pwr_mgmt_run();
    }
}

/** @} */