/**
 * @file main.c
 * @brief Precision Agriculture Capacitive Rain Sensor (nRF52833 SAADC)
 *
 * Measurement principle: Arduino Capacitance Meter small-capacitance method.
 * Reference: https://wordpress.codewrite.co.uk/pic/2014/01/21/cap-meter-with-arduino-uno/index.html
 *
 * Hardware connection:
 *   OUT_PIN = P0.02 (AIN0) — driven high to charge sensor capacitor
 *   IN_PIN  = P0.03 (AIN1) — ADC sense pin, charged via internal pull-up (~13 kΩ)
 *
 * Calibration: two-point calibration using 10 pF and 100 pF standard capacitors.
 *   Set CALIBRATION_FUNCTIONALITY_ENABLED = 1 to run calibration mode.
 *
 * Output: capacitance value (pF) and rain intensity level every 2 seconds via RTT log.
 *
 * Author: Humeizi Song — shmz2025@126.com
 * Supervisor: Dr. Spyridon Daskalakis
 * Heriot-Watt University, Edinburgh EH14 4AS, UK
 *
 * Revision history:
 *   2025-02-20: Initial port from Arduino; basic measurement working.
 *   2025-03-01: Calibration mode added.
 *   2025-03-16: Improved calibration: longer charging time, increased averaging, outlier removal.
 *   2025-03-20: Removed large-capacitance branch; only small capacitance formula used.
 *   2025-03-24: Enhanced stability: increased oversampling, longer stabilisation delay,
 *               larger moving average window, outlier rejection.
 *   2025-03-25: Increased charging time to 1 ms, oversampling to 10,
 *               added 15-repeat measurement with outlier removal before moving average.
 *   2025-03-26: Extended charging time to 1 ms, increased discharge settling.
 *   2025-05-01: Fixed LFCLK startup wait; moved measurement out of ISR; removed
 *               async calibrate_offset race; added nrf_pwr_mgmt_init.
 */

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "boards.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// -----------------------------------------------------------------------------
// Mode selection  (0 = measurement mode, 1 = calibration mode)
// -----------------------------------------------------------------------------
#define CALIBRATION_FUNCTIONALITY_ENABLED  0

// -----------------------------------------------------------------------------
// Hardware pins
// -----------------------------------------------------------------------------
#define RAIN_SENSOR_OUT_PIN          NRF_GPIO_PIN_MAP(0, 2)  // AIN0 — charging output
#define RAIN_SENSOR_IN_PIN           NRF_GPIO_PIN_MAP(0, 3)  // AIN1 — ADC sense input

#define RAIN_SENSOR_ADC_CHANNEL_IN   1   // AIN1
#define RAIN_SENSOR_ADC_CHANNEL_OUT  0   // AIN0

// -----------------------------------------------------------------------------
// Capacitance measurement parameters
// -----------------------------------------------------------------------------
#define IN_STRAY_CAP_TO_GND     99.0f   // Proportionality factor K  (10 pF / 100 pF two-point calibration)
#define C_STRAY                 41.0f   // System stray capacitance (pF)
#define MAX_ADC_VALUE           4095    // 12-bit ADC full scale

// -----------------------------------------------------------------------------
// System / timing parameters
// -----------------------------------------------------------------------------
#define RAIN_MEASUREMENT_INTERVAL_SEC    2    // RTC period between measurements (seconds)
#define RTC_FREQUENCY                    32   // RTC clock frequency (Hz)
#define LED_FUNCTIONALITY_ENABLED        1    // 1 = toggle LED on each measurement
#define USE_LEVEL_STRING                 0    // 0 = numeric level output, 1 = text output

// -----------------------------------------------------------------------------
// Stability parameters
// -----------------------------------------------------------------------------
#define ADC_OVERSAMPLE     10    // ADC samples averaged per charge cycle
#define CHARGE_DELAY_US    1000  // Capacitor charge time (µs)
#define MEASURE_REPEAT     15    // Repeated charge cycles per measurement (outlier rejection)

// -----------------------------------------------------------------------------
// Moving average filter (5-point)
// -----------------------------------------------------------------------------
#define FILTER_SAMPLES  5
static float    g_cap_buffer[FILTER_SAMPLES] = {0};
static uint8_t  g_cap_idx = 0;
static float    g_cap_sum = 0;
static uint8_t  g_cap_cnt = 0;

// -----------------------------------------------------------------------------
// Rain intensity
// -----------------------------------------------------------------------------
typedef enum {
    RAIN_NONE     = 0,
    RAIN_LIGHT    = 1,
    RAIN_MODERATE = 2,
    RAIN_HEAVY    = 3
} rain_intensity_t;

static struct {
    float            capacitance_pF;
    rain_intensity_t intensity_level;
} g_rain_sensor = {0};

// -----------------------------------------------------------------------------
// RTC globals
// -----------------------------------------------------------------------------
static const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);
static uint32_t rtc_ticks = RTC_US_TO_TICKS(RAIN_MEASUREMENT_INTERVAL_SEC * 1000000UL, RTC_FREQUENCY);
static volatile bool g_do_measurement = false;  // set by RTC ISR, consumed in main loop
static volatile bool g_saadc_cal_done = false;  // set by saadc_callback on CALIBRATEDONE

// -----------------------------------------------------------------------------
// Function prototypes
// -----------------------------------------------------------------------------
static float            measure_capacitance(void);
static void             perform_saadc_sample(nrf_saadc_value_t *sample, uint8_t channel);
static void             saadc_callback(nrf_drv_saadc_evt_t const *p_event);
static void             saadc_init(void);
static void             lfclk_config(void);
static void             rtc_handler(nrf_drv_rtc_int_type_t int_type);
static void             rtc_config(void);
static void             gpio_init(void);
static void             init_log(void);
static void             process_rain_measurement(float capacitance);
static rain_intensity_t classify_rain_intensity(float capacitance);

#if CALIBRATION_FUNCTIONALITY_ENABLED
static void     perform_calibration(void);
static uint16_t measure_small_cap_adc(void);
static float    collect_measurements(float cap_value, const char *cap_name,
                                     int num_measurements, int wait_time);
#endif

// -----------------------------------------------------------------------------
// SAADC blocking single-sample helper
// -----------------------------------------------------------------------------
static void perform_saadc_sample(nrf_saadc_value_t *sample, uint8_t channel)
{
    ret_code_t err_code = nrf_drv_saadc_sample_convert(channel, sample);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("SAADC conversion failed ch%d: %d", channel, err_code);
        *sample = 0;
    }
}

// -----------------------------------------------------------------------------
// Capacitance measurement
//   15 charge/discharge cycles, 10x oversampling per cycle,
//   bubble-sort + trim 3 outliers each side, then apply calibration formula.
// -----------------------------------------------------------------------------
static float measure_capacitance(void)
{
    uint32_t          adc_vals[MEASURE_REPEAT];
    nrf_saadc_value_t adc_val;

    for (int repeat = 0; repeat < MEASURE_REPEAT; repeat++)
    {
        // Discharge phase
        nrf_gpio_cfg_output(RAIN_SENSOR_OUT_PIN);
        nrf_gpio_cfg_output(RAIN_SENSOR_IN_PIN);
        nrf_gpio_pin_clear(RAIN_SENSOR_OUT_PIN);
        nrf_gpio_pin_clear(RAIN_SENSOR_IN_PIN);
        nrf_delay_ms(5);

        nrf_gpio_cfg_default(RAIN_SENSOR_OUT_PIN);
        nrf_gpio_cfg_default(RAIN_SENSOR_IN_PIN);
        nrf_delay_us(1000);

        // Charge phase
        nrf_gpio_cfg_output(RAIN_SENSOR_OUT_PIN);
        nrf_gpio_pin_set(RAIN_SENSOR_OUT_PIN);
        nrf_delay_us(CHARGE_DELAY_US);
        nrf_gpio_cfg_default(RAIN_SENSOR_IN_PIN);
        nrf_delay_us(10);

        // Oversampling
        uint32_t adc_sum = 0;
        for (int i = 0; i < ADC_OVERSAMPLE; i++)
        {
            perform_saadc_sample(&adc_val, RAIN_SENSOR_ADC_CHANNEL_IN);
            adc_sum += adc_val;
        }
        adc_vals[repeat] = adc_sum / ADC_OVERSAMPLE;

        nrf_gpio_pin_clear(RAIN_SENSOR_OUT_PIN);
        nrf_gpio_cfg_default(RAIN_SENSOR_OUT_PIN);
        nrf_delay_ms(10);
    }

    // Bubble sort
    for (int i = 0; i < MEASURE_REPEAT - 1; i++)
        for (int j = i + 1; j < MEASURE_REPEAT; j++)
            if (adc_vals[i] > adc_vals[j]) {
                uint32_t tmp = adc_vals[i];
                adc_vals[i]  = adc_vals[j];
                adc_vals[j]  = tmp;
            }

    // Trim 3 from each side
    int trim = (MEASURE_REPEAT >= 7) ? 3 : 1;
    if (MEASURE_REPEAT <= 2 * trim) trim = 0;
    uint32_t sum   = 0;
    int      count = MEASURE_REPEAT - 2 * trim;
    for (int i = trim; i < MEASURE_REPEAT - trim; i++)
        sum += adc_vals[i];
    uint32_t val = sum / count;

    float capacitance = (float)val * IN_STRAY_CAP_TO_GND / (float)(MAX_ADC_VALUE - val) - C_STRAY;
    if (capacitance < 0) capacitance = 0;
    NRF_LOG_DEBUG("Cap: ADC=%d, C=%d pF", val, (int)capacitance);
    return capacitance;
}

// -----------------------------------------------------------------------------
// Rain intensity classification
// -----------------------------------------------------------------------------
static rain_intensity_t classify_rain_intensity(float capacitance)
{
    if      (capacitance < 150.0f) return RAIN_NONE;
    else if (capacitance < 250.0f) return RAIN_LIGHT;
    else if (capacitance < 500.0f) return RAIN_MODERATE;
    else                           return RAIN_HEAVY;
}

static void process_rain_measurement(float capacitance)
{
    g_rain_sensor.capacitance_pF  = capacitance;
    g_rain_sensor.intensity_level = classify_rain_intensity(capacitance);

#if (USE_LEVEL_STRING == 1)
    const char *level_str[] = {"NONE", "LIGHT", "MODERATE", "HEAVY"};
    NRF_LOG_INFO("Capacitance: %d pF, Level: %s",
                 (int)capacitance, level_str[g_rain_sensor.intensity_level]);
#else
    NRF_LOG_INFO("Capacitance: %d pF, Level: %d",
                 (int)capacitance, (int)g_rain_sensor.intensity_level);
#endif
}

// -----------------------------------------------------------------------------
// SAADC callback
// -----------------------------------------------------------------------------
static void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_CALIBRATEDONE)
    {
        g_saadc_cal_done = true;
        NRF_LOG_INFO("    SAADC hardware offset calibration complete.");
        NRF_LOG_FLUSH();
    }
}

// -----------------------------------------------------------------------------
// SAADC initialisation — dual channel, polling (sample_convert) mode
// -----------------------------------------------------------------------------
static void saadc_init(void)
{
    ret_code_t err_code;

    nrf_drv_saadc_config_t saadc_config = {
        .low_power_mode     = true,
        .resolution         = NRF_SAADC_RESOLUTION_12BIT,
        .oversample         = NRF_SAADC_OVERSAMPLE_DISABLED,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };
    NRF_LOG_INFO("  - Initializing SAADC driver...");
    NRF_LOG_FLUSH();
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t ch_cfg = {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain       = NRF_SAADC_GAIN1_6,
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,
        .acq_time   = NRF_SAADC_ACQTIME_10US,
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .burst      = NRF_SAADC_BURST_DISABLED,
        .pin_n      = NRF_SAADC_INPUT_DISABLED
    };

    NRF_LOG_INFO("  - Configuring AIN0 (ch0)...");
    NRF_LOG_FLUSH();
    ch_cfg.pin_p = NRF_SAADC_INPUT_AIN0;
    err_code = nrf_drv_saadc_channel_init(RAIN_SENSOR_ADC_CHANNEL_OUT, &ch_cfg);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("  - Configuring AIN1 (ch1)...");
    NRF_LOG_FLUSH();
    ch_cfg.pin_p = NRF_SAADC_INPUT_AIN1;
    err_code = nrf_drv_saadc_channel_init(RAIN_SENSOR_ADC_CHANNEL_IN, &ch_cfg);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("  - Starting SAADC hardware offset calibration...");
    NRF_LOG_INFO("    (Trims internal ADC offset; completes before first measurement)");
    NRF_LOG_FLUSH();
    g_saadc_cal_done = false;
    APP_ERROR_CHECK(nrf_drv_saadc_calibrate_offset());
    while (!g_saadc_cal_done);
    NRF_LOG_INFO("    SAADC ready.");
    NRF_LOG_FLUSH();
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
// RTC interrupt handler — sets flag only; measurement runs in main loop
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
// RTC configuration
// -----------------------------------------------------------------------------
static void rtc_config(void)
{
    ret_code_t err_code;
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
// GPIO initialisation
// -----------------------------------------------------------------------------
static void gpio_init(void)
{
#if LED_FUNCTIONALITY_ENABLED
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
#endif
    nrf_gpio_cfg_default(RAIN_SENSOR_OUT_PIN);
    nrf_gpio_cfg_default(RAIN_SENSOR_IN_PIN);
}

// -----------------------------------------------------------------------------
// Logging initialisation
// -----------------------------------------------------------------------------
static void init_log(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

// -----------------------------------------------------------------------------
// Calibration mode (compiled only when CALIBRATION_FUNCTIONALITY_ENABLED = 1)
// -----------------------------------------------------------------------------
#if CALIBRATION_FUNCTIONALITY_ENABLED

static uint16_t measure_small_cap_adc(void)
{
    nrf_saadc_value_t adc_val = 0;
    uint32_t          adc_sum = 0;

    nrf_gpio_cfg_output(RAIN_SENSOR_OUT_PIN);
    nrf_gpio_cfg_output(RAIN_SENSOR_IN_PIN);
    nrf_gpio_pin_clear(RAIN_SENSOR_OUT_PIN);
    nrf_gpio_pin_clear(RAIN_SENSOR_IN_PIN);
    nrf_delay_ms(5);
    nrf_gpio_cfg_default(RAIN_SENSOR_OUT_PIN);
    nrf_gpio_cfg_default(RAIN_SENSOR_IN_PIN);
    nrf_delay_us(1000);

    nrf_gpio_cfg_output(RAIN_SENSOR_OUT_PIN);
    nrf_gpio_pin_set(RAIN_SENSOR_OUT_PIN);
    nrf_delay_us(CHARGE_DELAY_US);
    nrf_gpio_cfg_default(RAIN_SENSOR_IN_PIN);
    nrf_delay_us(10);

    for (int i = 0; i < ADC_OVERSAMPLE; i++) {
        perform_saadc_sample(&adc_val, RAIN_SENSOR_ADC_CHANNEL_IN);
        adc_sum += adc_val;
    }
    uint16_t raw_val = adc_sum / ADC_OVERSAMPLE;

    nrf_gpio_pin_clear(RAIN_SENSOR_OUT_PIN);
    nrf_gpio_cfg_default(RAIN_SENSOR_OUT_PIN);
    return raw_val;
}

static float collect_measurements(float cap_value, const char *cap_name,
                                  int num_measurements, int wait_time)
{
    NRF_LOG_INFO("=========================================="); NRF_LOG_FLUSH();
    NRF_LOG_INFO("Connect %.0f pF capacitor between P0.02 and P0.03.", cap_value); NRF_LOG_FLUSH();
    NRF_LOG_INFO("Waiting %d seconds...", wait_time); NRF_LOG_FLUSH();
    for (int i = wait_time; i > 0; i--) { NRF_LOG_INFO("  %d...", i); NRF_LOG_FLUSH(); nrf_delay_ms(1000); }

    if (num_measurements > 50) num_measurements = 50;
    uint16_t measurements[50];

    NRF_LOG_INFO("Measuring %s...", cap_name); NRF_LOG_FLUSH();
    for (int i = 0; i < num_measurements; i++) {
        measurements[i] = measure_small_cap_adc();
        NRF_LOG_INFO("  [%2d] ADC = %4d", i + 1, measurements[i]);
        nrf_delay_ms(100);
    }

    for (int i = 0; i < num_measurements - 1; i++)
        for (int j = i + 1; j < num_measurements; j++)
            if (measurements[i] > measurements[j]) {
                uint16_t tmp    = measurements[i];
                measurements[i] = measurements[j];
                measurements[j] = tmp;
            }

    int trim = (num_measurements >= 5) ? 2 : 0;
    uint32_t sum   = 0;
    int      count = num_measurements - 2 * trim;
    for (int i = trim; i < num_measurements - trim; i++) sum += measurements[i];
    float avg = (float)sum / count;

    NRF_LOG_INFO("Average ADC (trim=%d each side): %d", trim, (int)avg); NRF_LOG_FLUSH();
    return avg;
}

static void perform_calibration(void)
{
    const float C1 = 10.0f, C2 = 100.0f;
    const int   N  = 20, WAIT = 10;

    NRF_LOG_INFO("===== Two-Point Calibration Mode ====="); NRF_LOG_FLUSH();
    NRF_LOG_INFO("Required: 10 pF and 100 pF reference capacitors."); NRF_LOG_FLUSH();

    float val1 = collect_measurements(C1, "10pF",  N, WAIT);
    float val2 = collect_measurements(C2, "100pF", N, WAIT);

    float x1 = val1 / (MAX_ADC_VALUE - val1);
    float x2 = val2 / (MAX_ADC_VALUE - val2);
    float K       = (C2 - C1) / (x2 - x1);
    float C_stray = x1 * K - C1;

    NRF_LOG_INFO("=========================================="); NRF_LOG_FLUSH();
    NRF_LOG_INFO("Calibration results:"); NRF_LOG_FLUSH();
    NRF_LOG_INFO("  #define IN_STRAY_CAP_TO_GND  %d.%df",
                 (int)K, (int)((K - (int)K) * 10)); NRF_LOG_FLUSH();
    NRF_LOG_INFO("  #define C_STRAY              %d.%df",
                 (int)C_stray, (int)((C_stray - (int)C_stray) * 10)); NRF_LOG_FLUSH();
    NRF_LOG_INFO("Update these in main.c then set CALIBRATION_FUNCTIONALITY_ENABLED = 0."); NRF_LOG_FLUSH();
    NRF_LOG_INFO("=========================================="); NRF_LOG_FLUSH();

    while (1) { NRF_LOG_PROCESS(); nrf_pwr_mgmt_run(); }
}

#endif // CALIBRATION_FUNCTIONALITY_ENABLED

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(void)
{
    init_log();
    NRF_LOG_INFO("===== nRF52833 Capacitive Rain Sensor =====");
    NRF_LOG_INFO("K=%d/10, Cstray=%d/10 pF, Interval=%ds",
                 (int)(IN_STRAY_CAP_TO_GND * 10),
                 (int)(C_STRAY * 10),
                 RAIN_MEASUREMENT_INTERVAL_SEC);
    NRF_LOG_FLUSH();

    NRF_LOG_INFO("Step 1: GPIO init");       gpio_init();
    NRF_LOG_INFO("Step 2: LFCLK config");    lfclk_config();
    NRF_LOG_INFO("Step 3: SAADC init");      saadc_init();    NRF_LOG_FLUSH();

#if CALIBRATION_FUNCTIONALITY_ENABLED
    NRF_LOG_INFO("Step 4: Calibration mode"); NRF_LOG_FLUSH();
    perform_calibration();  // does not return
#else
    NRF_LOG_INFO("Step 4: Power management init");
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Step 5: RTC config");
    rtc_config();
    NRF_LOG_INFO("System ready. Output: capacitance (pF) + level (0=NONE 1=LIGHT 2=MODERATE 3=HEAVY)");
    NRF_LOG_FLUSH();

    while (1)
    {
        if (g_do_measurement)
        {
            g_do_measurement = false;

            float raw_cap = measure_capacitance();

            if (raw_cap > 0.1f)
            {
                g_cap_sum              -= g_cap_buffer[g_cap_idx];
                g_cap_buffer[g_cap_idx] = raw_cap;
                g_cap_sum              += raw_cap;
                g_cap_idx               = (g_cap_idx + 1) % FILTER_SAMPLES;
                if (g_cap_cnt < FILTER_SAMPLES) g_cap_cnt++;

                float avg_cap = g_cap_sum / g_cap_cnt;
                process_rain_measurement(avg_cap);
            }
            NRF_LOG_FLUSH();
        }

        NRF_LOG_PROCESS();
        nrf_pwr_mgmt_run();
    }
#endif
}
