#ifndef config_h
#define config_h

#define LPP_BOARD_CHANNEL 0
#define LPP_BME280_CHANNEL 1
#define LPP_GPS_CHANNEL 2
#define LPP_BME280_PROBE_COUNT_CHANNEL 101
#define LPP_BME280_ERROR_COUNT_CHANNEL 201
#define PROBE_MINIMUM_SIGNIFICANT_VALUES 3
#define PROBE_MAXIMUM_PROBING_ITERATION 5
#define PROBE_INTERVAL_IN_MS 1000
#define ADC_CORRECTION_RATIO ( 1 )

#define PROBING_PERIOD_IN_SEC 120

#define ADC_BAT_PIN 35

#endif //config_h