#ifndef RAIN_SENSOR_H
#define RAIN_SENSOR_H

// Mode selection: 0 = measurement, 1 = two-point calibration
#define CALIBRATION_FUNCTIONALITY_ENABLED  0

// Output format: 0 = numeric level, 1 = level string
#define USE_LEVEL_STRING  0

typedef enum {
    RAIN_NONE     = 0,
    RAIN_LIGHT    = 1,
    RAIN_MODERATE = 2,
    RAIN_HEAVY    = 3
} rain_intensity_t;

// Initialise SAADC and sensor GPIO
void rain_sensor_init(void);

// One full measurement cycle: measure + moving-average filter + classify + log
void rain_sensor_update(void);

#if CALIBRATION_FUNCTIONALITY_ENABLED
// Two-point calibration mode (does not return)
void perform_calibration(void);
#endif

#endif // RAIN_SENSOR_H
