#ifndef WATER_SENSOR_H_
#define WATER_SENSOR_H_

#include <stdbool.h>
#include <esp_err.h>
#include <driver/gpio.h>
#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Water level classification states
 */
typedef enum {
    WATER_LEVEL_DRY = 0,     // No water detected (< 10%)
    WATER_LEVEL_LOW,         // Low water level (10% - 35%)
    WATER_LEVEL_MEDIUM,      // Medium water level (35% - 70%)
    WATER_LEVEL_HIGH,        // High water level (70% - 90%)
    WATER_LEVEL_FULL         // Sensor fully submerged (> 90%)
} water_level_state_t;

/**
 * @brief Configuration structure for HW-038 water level sensor
 */
typedef struct {
    adc_oneshot_unit_handle_t adc_unit_handle; // Optional shared ADC1 handle (or NULL to create new)
    adc_channel_t adc_channel;       // ADC1 channel (e.g. ADC_CHANNEL_6 for GPIO 34)
    gpio_num_t    do_gpio_num;       // Digital output pin from LM393 (or GPIO_NUM_NC if unused)
    int           cal_dry_raw;       // Calibration ADC raw reading when dry in air
    int           cal_full_raw;      // Calibration ADC raw reading when fully submerged
    bool          invert_logic;      // True if higher ADC value means dry in air (e.g. HW-038 pull-up)
} water_sensor_config_t;

typedef struct water_sensor_dev_t *water_sensor_handle_t;

/**
 * @brief Initialize the HW-038 water level sensor
 *
 * @param config Pointer to configuration struct
 * @param[out] out_handle Pointer to receive device handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t water_sensor_init(const water_sensor_config_t *config, water_sensor_handle_t *out_handle);

/**
 * @brief Read raw 12-bit ADC value (0 - 4095)
 *
 * @param handle Sensor handle
 * @param[out] raw_out Raw ADC reading
 * @return esp_err_t ESP_OK on success
 */
esp_err_t water_sensor_read_raw(water_sensor_handle_t handle, int *raw_out);

/**
 * @brief Read voltage in millivolts (approximate 0 - 3300 mV)
 *
 * @param handle Sensor handle
 * @param[out] voltage_mv Estimated voltage in mV
 * @return esp_err_t ESP_OK on success
 */
esp_err_t water_sensor_read_voltage(water_sensor_handle_t handle, int *voltage_mv);

/**
 * @brief Read water level percentage (0.0% to 100.0%) based on calibration values
 *
 * @param handle Sensor handle
 * @param[out] percentage Percentage (0.0 - 100.0)
 * @param[out] state Discrete state (DRY, LOW, MEDIUM, HIGH, FULL)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t water_sensor_read_level(water_sensor_handle_t handle, float *percentage, water_level_state_t *state);

/**
 * @brief Read digital output state (from LM393 comparator on HW-038)
 *
 * @param handle Sensor handle
 * @param[out] threshold_exceeded true if digital pin triggered
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_SUPPORTED if DO pin is NC
 */
esp_err_t water_sensor_read_digital(water_sensor_handle_t handle, bool *threshold_exceeded);

/**
 * @brief Convert state enum to human-readable string
 */
const char* water_sensor_state_to_str(water_level_state_t state);

/**
 * @brief Free resources associated with the water sensor
 */
esp_err_t water_sensor_deinit(water_sensor_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // WATER_SENSOR_H_
