#include "dht.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>
#include <esp_log.h>

static const char *TAG = "DHT";

static portMUX_TYPE dht_mux = portMUX_INITIALIZER_UNLOCKED;

static inline esp_err_t dht_wait_for_level(gpio_num_t gpio_num, int level, uint32_t timeout_us)
{
    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(gpio_num) != level) {
        if ((uint32_t)(esp_timer_get_time() - start_time) > timeout_us) {
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}

esp_err_t dht_read_data(dht_sensor_type_t sensor_type, gpio_num_t gpio_num,
                        float *humidity, float *temperature)
{
    if (humidity == NULL || temperature == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[5] = {0};

    // Step 1: Send start signal to DHT
    // Set pin as output
    gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(gpio_num, GPIO_PULLUP_ONLY);

    // Pull low for at least 18-20ms
    gpio_set_level(gpio_num, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Pull high for 30us
    gpio_set_level(gpio_num, 1);
    esp_rom_delay_us(30);

    // Switch pin to input with pullup to read DHT response
    gpio_set_direction(gpio_num, GPIO_MODE_INPUT);

    // Enter critical section to ensure precise microsecond timing
    taskENTER_CRITICAL(&dht_mux);

    // Step 2: Wait for DHT response (80us LOW followed by 80us HIGH)
    // First, wait for line to go LOW
    if (dht_wait_for_level(gpio_num, 0, 100) != ESP_OK) {
        taskEXIT_CRITICAL(&dht_mux);
        ESP_LOGD(TAG, "Timeout waiting for initial response (LOW)");
        return ESP_ERR_TIMEOUT;
    }

    // Wait for line to go HIGH
    if (dht_wait_for_level(gpio_num, 1, 100) != ESP_OK) {
        taskEXIT_CRITICAL(&dht_mux);
        ESP_LOGD(TAG, "Timeout waiting for response signal (HIGH)");
        return ESP_ERR_TIMEOUT;
    }

    // Wait for line to go LOW (start of first bit transmission)
    if (dht_wait_for_level(gpio_num, 0, 100) != ESP_OK) {
        taskEXIT_CRITICAL(&dht_mux);
        ESP_LOGD(TAG, "Timeout waiting for transmission start (LOW)");
        return ESP_ERR_TIMEOUT;
    }

    // Step 3: Read 40 data bits
    for (int i = 0; i < 40; i++) {
        // Each bit begins with ~50us LOW signal
        if (dht_wait_for_level(gpio_num, 1, 75) != ESP_OK) {
            taskEXIT_CRITICAL(&dht_mux);
            ESP_LOGD(TAG, "Timeout waiting for bit %d HIGH", i);
            return ESP_ERR_TIMEOUT;
        }

        // Measure duration of HIGH signal (26-28us = 0, ~70us = 1)
        int64_t pulse_start = esp_timer_get_time();
        if (dht_wait_for_level(gpio_num, 0, 100) != ESP_OK) {
            taskEXIT_CRITICAL(&dht_mux);
            ESP_LOGD(TAG, "Timeout waiting for bit %d LOW", i);
            return ESP_ERR_TIMEOUT;
        }

        uint32_t pulse_duration = (uint32_t)(esp_timer_get_time() - pulse_start);

        // Threshold between 0 and 1 is ~45us
        if (pulse_duration > 45) {
            data[i / 8] |= (1 << (7 - (i % 8)));
        }
    }

    taskEXIT_CRITICAL(&dht_mux);

    // Step 4: Verify checksum
    uint8_t checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
    if (data[4] != checksum) {
        ESP_LOGW(TAG, "Checksum failed: calc=0x%02X, received=0x%02X (raw: %02X %02X %02X %02X %02X)",
                 checksum, data[4], data[0], data[1], data[2], data[3], data[4]);
        return ESP_ERR_INVALID_CRC;
    }

    // Step 5: Convert raw data based on sensor type
    if (sensor_type == DHT_TYPE_DHT11) {
        *humidity = (float)data[0] + (float)data[1] * 0.1f;
        *temperature = (float)data[2] + (float)data[3] * 0.1f;
    } else { // DHT_TYPE_DHT22 / AM2302
        *humidity = (float)(((uint16_t)data[0] << 8) | data[1]) * 0.1f;
        
        int16_t raw_temp = (int16_t)((((uint16_t)data[2] & 0x7F) << 8) | data[3]);
        if (data[2] & 0x80) {
            raw_temp = -raw_temp;
        }
        *temperature = (float)raw_temp * 0.1f;
    }

    return ESP_OK;
}
