#ifndef DHT_H_
#define DHT_H_

#include <esp_err.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DHT_TYPE_DHT11 = 0,
    DHT_TYPE_DHT22,
    DHT_TYPE_AM2302 = DHT_TYPE_DHT22
} dht_sensor_type_t;

/**
 * @brief Read relative humidity and temperature from a DHT sensor.
 *
 * @param sensor_type Type of sensor: DHT_TYPE_DHT11 or DHT_TYPE_DHT22
 * @param gpio_num GPIO pin connected to the DHT data pin
 * @param[out] humidity Pointer to float where humidity (%) will be written
 * @param[out] temperature Pointer to float where temperature (°C) will be written
 * @return esp_err_t 
 *         - ESP_OK: Successfully read sensor data
 *         - ESP_ERR_INVALID_ARG: NULL pointer argument
 *         - ESP_ERR_TIMEOUT: Sensor did not respond or timed out during signal
 *         - ESP_ERR_INVALID_CRC: Checksum failed
 */
esp_err_t dht_read_data(dht_sensor_type_t sensor_type, gpio_num_t gpio_num,
                        float *humidity, float *temperature);

#ifdef __cplusplus
}
#endif

#endif // DHT_H_
