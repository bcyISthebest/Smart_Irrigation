#ifndef SOIL_SENSOR_H_
#define SOIL_SENSOR_H_

#include <stdbool.h>
#include <esp_err.h>
#include <driver/gpio.h>
#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Soil moisture status levels for smart irrigation
 */
typedef enum {
    SOIL_MOISTURE_VERY_DRY = 0, // < 20%: Very dry, urgent irrigation needed
    SOIL_MOISTURE_DRY,          // 20% - 40%: Dry, irrigation recommended
    SOIL_MOISTURE_OPTIMAL,      // 40% - 70%: Ideal moisture for plants
    SOIL_MOISTURE_WET,          // 70% - 85%: Moist / well-watered
    SOIL_MOISTURE_SATURATED     // > 85%: Saturated / muddy
} soil_moisture_state_t;

/**
 * @brief Soil sensor configuration structure
 */
typedef struct {
    adc_oneshot_unit_handle_t adc_unit_handle; // Optional shared ADC1 handle (or NULL to create new)
    adc_channel_t             adc_channel;     // ADC1 channel (e.g. ADC_CHANNEL_7 for GPIO 35)
    gpio_num_t                do_gpio_num;     // Digital comparator pin (or GPIO_NUM_NC if unused)
    int                       cal_air_dry_raw; // Raw ADC reading in dry soil / air (~3200-4000)
    int                       cal_water_wet_raw;// Raw ADC reading in wet soil / cup of water (~1200-1800)
    bool                      invert_logic;    // True if higher ADC means dry (Capacitive & resistive pull-up)
} soil_sensor_config_t;

typedef struct soil_sensor_dev_t *soil_sensor_handle_t;

/**
 * @brief Initialize the Soil Moisture Sensor
 *
 * @param config Pointer to configuration struct
 * @param[out] out_handle Pointer to receive device handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t soil_sensor_init(const soil_sensor_config_t *config, soil_sensor_handle_t *out_handle);

/**
 * @brief Read raw 12-bit ADC value (0 - 4095) with averaging
 *
 * @param handle Sensor handle
 * @param[out] raw_out Averaged raw ADC value
 * @return esp_err_t ESP_OK on success
 */
esp_err_t soil_sensor_read_raw(soil_sensor_handle_t handle, int *raw_out);

/**
 * @brief Read estimated voltage in millivolts (0 - 3300 mV)
 *
 * @param handle Sensor handle
 * @param[out] voltage_mv Estimated voltage in mV
 * @return esp_err_t ESP_OK on success
 */
esp_err_t soil_sensor_read_voltage(soil_sensor_handle_t handle, int *voltage_mv);

/**
 * @brief Read soil moisture percentage (0.0% to 100.0%) and status category
 *
 * @param handle Sensor handle
 * @param[out] percentage Soil moisture percentage (0.0 - 100.0%)
 * @param[out] state Soil moisture health state
 * @return esp_err_t ESP_OK on success
 */
esp_err_t soil_sensor_read_moisture(soil_sensor_handle_t handle, float *percentage, soil_moisture_state_t *state);

/**
 * @brief Read digital output state (if DO pin connected on resistive module)
 *
 * @param handle Sensor handle
 * @param[out] digital_triggered True if digital threshold triggered
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_SUPPORTED if DO is NC
 */
esp_err_t soil_sensor_read_digital(soil_sensor_handle_t handle, bool *digital_triggered);

/**
 * @brief Convert soil state enum to descriptive string
 */
const char* soil_sensor_state_to_str(soil_moisture_state_t state);

/**
 * @brief Deinitialize and free resources
 */
esp_err_t soil_sensor_deinit(soil_sensor_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // SOIL_SENSOR_H_
