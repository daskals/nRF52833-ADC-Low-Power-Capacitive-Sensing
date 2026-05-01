/**
 * @file main.c
 * @brief nRF52833 Capacitive Rain Sensor — system init and main loop.
 *
 * Sensor logic (SAADC, measurement, calibration): rain_sensor.c / rain_sensor.h
 *
 * Author: Humeizi Song — shmz2025@126.com
 * Supervisor: Dr. Spyridon Daskalakis
 * Heriot-Watt University, Edinburgh EH14 4AS, UK
 */

#include <stdbool.h>
#include <stdint.h>
#include "rain_sensor.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define RAIN_MEASUREMENT_INTERVAL_SEC  2   // RTC period between measurements (seconds)
#define RTC_FREQUENCY                  32  // RTC clock frequency (Hz)
#define LED_FUNCTIONALITY_ENABLED      1   // 1 = toggle LED on each measurement

static const nrf_drv_rtc_t rtc      = NRF_DRV_RTC_INSTANCE(2);
static uint32_t            rtc_ticks = RTC_US_TO_TICKS(RAIN_MEASUREMENT_INTERVAL_SEC * 1000000UL, RTC_FREQUENCY);
static volatile bool       g_do_measurement = false;

// -----------------------------------------------------------------------------
// RTC interrupt handler — sets flag only; work runs in main loop
// -----------------------------------------------------------------------------
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type != NRF_DRV_RTC_INT_COMPARE0) return;

    g_do_measurement = true;

    ret_code_t err_code = nrf_drv_rtc_cc_set(&rtc, 0, rtc_ticks, true);
    APP_ERROR_CHECK(err_code);
    nrf_drv_rtc_counter_clear(&rtc);

#if LED_FUNCTIONALITY_ENABLED
    LEDS_INVERT(BSP_LED_0_MASK);
#endif
}

// -----------------------------------------------------------------------------
// Low-frequency clock
// -----------------------------------------------------------------------------
static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("  - Requesting LFCLK...");
    NRF_LOG_FLUSH();
    nrf_drv_clock_lfclk_request(NULL);
    NRF_LOG_INFO("  - Waiting for LFCLK to stabilise...");
    NRF_LOG_FLUSH();
    while (!nrf_drv_clock_lfclk_is_running())
        nrf_delay_ms(1);
    NRF_LOG_INFO("    LFCLK running.");
    NRF_LOG_FLUSH();
}

// -----------------------------------------------------------------------------
// RTC configuration
// -----------------------------------------------------------------------------
static void rtc_config(void)
{
    ret_code_t           err_code;
    nrf_drv_rtc_config_t cfg = { .prescaler = RTC_FREQ_TO_PRESCALER(RTC_FREQUENCY) };

    NRF_LOG_INFO("  - Initializing RTC...");
    NRF_LOG_FLUSH();
    err_code = nrf_drv_rtc_init(&rtc, &cfg, rtc_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_rtc_cc_set(&rtc, 0, rtc_ticks, true);
    APP_ERROR_CHECK(err_code);

    nrf_drv_rtc_enable(&rtc);
    NRF_LOG_INFO("    RTC enabled. Interval: %d s", RAIN_MEASUREMENT_INTERVAL_SEC);
    NRF_LOG_FLUSH();
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("===== nRF52833 Capacitive Rain Sensor =====");
    NRF_LOG_FLUSH();

#if LED_FUNCTIONALITY_ENABLED
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
#endif

    NRF_LOG_INFO("Step 1: LFCLK config");    lfclk_config();
    NRF_LOG_INFO("Step 2: Sensor init");     rain_sensor_init();  NRF_LOG_FLUSH();

#if CALIBRATION_FUNCTIONALITY_ENABLED
    NRF_LOG_INFO("Step 3: Calibration mode"); NRF_LOG_FLUSH();
    perform_calibration();  // does not return
#else
    NRF_LOG_INFO("Step 3: Power management init");
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Step 4: RTC config");
    rtc_config();

    NRF_LOG_INFO("----------------------------------------");
    NRF_LOG_INFO("System ready.");
    NRF_LOG_INFO("Level: 0=NONE  1=LIGHT  2=MODERATE  3=HEAVY");
    NRF_LOG_INFO("----------------------------------------");
    NRF_LOG_FLUSH();

    while (1)
    {
        if (g_do_measurement)
        {
            g_do_measurement = false;
            rain_sensor_update();
            NRF_LOG_FLUSH();
        }
        NRF_LOG_PROCESS();
        nrf_pwr_mgmt_run();
    }
#endif
}
