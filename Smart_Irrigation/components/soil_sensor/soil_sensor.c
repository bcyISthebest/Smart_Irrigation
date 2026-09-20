#include "soil_sensor.h"
#include <stdlib.h>
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "SOIL_SENSOR";

struct soil_sensor_dev_t {
    adc_oneshot_unit_handle_t adc_handle;
    bool                      owns_adc_handle;
    adc_channel_t             adc_channel;
    gpio_num_t                do_gpio_num;
    int                       cal_air_dry_raw;
    int                       cal_water_wet_raw;
    bool                      invert_logic;
};

esp_err_t soil_sensor_init(const soil_sensor_config_t *config, soil_sensor_handle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(config != NULL && out_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Null pointer argument");

    soil_sensor_handle_t dev = calloc(1, sizeof(struct soil_sensor_dev_t));
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_NO_MEM, TAG, "Memory allocation failed");

    dev->adc_channel = config->adc_channel;
    dev->do_gpio_num = config->do_gpio_num;
    dev->invert_logic = config->invert_logic;

    if (config->invert_logic) {
        dev->cal_air_dry_raw = (config->cal_air_dry_raw > 0) ? config->cal_air_dry_raw : 3500;
        dev->cal_water_wet_raw = (config->cal_water_wet_raw > 0) ? config->cal_water_wet_raw : 1400;
    } else {
        dev->cal_air_dry_raw = (config->cal_air_dry_raw > 0) ? config->cal_air_dry_raw : 1400;
        dev->cal_water_wet_raw = (config->cal_water_wet_raw > 0) ? config->cal_water_wet_raw : 3500;
    }

    // 1. Manage ADC1 Unit
    if (config->adc_unit_handle != NULL) {
        dev->adc_handle = config->adc_unit_handle;
        dev->owns_adc_handle = false;
    } else {
        adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        esp_err_t ret = adc_oneshot_new_unit(&init_config1, &dev->adc_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
            free(dev);
            return ret;
        }
        dev->owns_adc_handle = true;
    }

    // 2. Configure ADC channel
    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    esp_err_t ret = adc_oneshot_config_channel(dev->adc_handle, dev->adc_channel, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        if (dev->owns_adc_handle) {
            adc_oneshot_del_unit(dev->adc_handle);
        }
        free(dev);
        return ret;
    }

    // 3. Configure DO (Digital Output) pin if present
    if (dev->do_gpio_num != GPIO_NUM_NC && dev->do_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << dev->do_gpio_num),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
    }

    *out_handle = dev;
    ESP_LOGI(TAG, "Soil sensor initialized (Chan: %d, DO Pin: %d, Dry: %d, Wet: %d, Invert: %d)",
             dev->adc_channel, dev->do_gpio_num, dev->cal_air_dry_raw, dev->cal_water_wet_raw, dev->invert_logic);
    return ESP_OK;
}

esp_err_t soil_sensor_read_raw(soil_sensor_handle_t handle, int *raw_out)
{
    ESP_RETURN_ON_FALSE(handle != NULL && raw_out != NULL, ESP_ERR_INVALID_ARG, TAG, "Null pointer argument");

    int sum = 0;
    int val = 0;
    const int SAMPLES = 8;

    for (int i = 0; i < SAMPLES; i++) {
        esp_err_t ret = adc_oneshot_read(handle->adc_handle, handle->adc_channel, &val);
        if (ret != ESP_OK) {
            return ret;
        }
        sum += val;
    }

    *raw_out = sum / SAMPLES;
    return ESP_OK;
}

esp_err_t soil_sensor_read_voltage(soil_sensor_handle_t handle, int *voltage_mv)
{
    ESP_RETURN_ON_FALSE(handle != NULL && voltage_mv != NULL, ESP_ERR_INVALID_ARG, TAG, "Null pointer argument");

    int raw = 0;
    esp_err_t ret = soil_sensor_read_raw(handle, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    *voltage_mv = (raw * 3300) / 4095;
    return ESP_OK;
}

esp_err_t soil_sensor_read_moisture(soil_sensor_handle_t handle, float *percentage, soil_moisture_state_t *state)
{
    ESP_RETURN_ON_FALSE(handle != NULL && percentage != NULL, ESP_ERR_INVALID_ARG, TAG, "Null pointer argument");

    int raw = 0;
    esp_err_t ret = soil_sensor_read_raw(handle, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    float pct = 0.0f;
    int dry = handle->cal_air_dry_raw;
    int wet = handle->cal_water_wet_raw;

    if (handle->invert_logic || dry > wet) {
        if (dry != wet) {
            pct = ((float)(dry - raw) / (float)(dry - wet)) * 100.0f;
        }
    } else {
        if (wet != dry) {
            pct = ((float)(raw - dry) / (float)(wet - dry)) * 100.0f;
        }
    }

    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    *percentage = pct;

    if (state != NULL) {
        if (pct < 20.0f) {
            *state = SOIL_MOISTURE_VERY_DRY;
        } else if (pct < 40.0f) {
            *state = SOIL_MOISTURE_DRY;
        } else if (pct < 70.0f) {
            *state = SOIL_MOISTURE_OPTIMAL;
        } else if (pct < 85.0f) {
            *state = SOIL_MOISTURE_WET;
        } else {
            *state = SOIL_MOISTURE_SATURATED;
        }
    }

    return ESP_OK;
}

esp_err_t soil_sensor_read_digital(soil_sensor_handle_t handle, bool *digital_triggered)
{
    ESP_RETURN_ON_FALSE(handle != NULL && digital_triggered != NULL, ESP_ERR_INVALID_ARG, TAG, "Null pointer argument");
    if (handle->do_gpio_num == GPIO_NUM_NC || handle->do_gpio_num < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    int level = gpio_get_level(handle->do_gpio_num);
    *digital_triggered = (level == 0);
    return ESP_OK;
}

const char* soil_sensor_state_to_str(soil_moisture_state_t state)
{
    switch (state) {
        case SOIL_MOISTURE_VERY_DRY:  return "VERY DRY (Water Needed!)";
        case SOIL_MOISTURE_DRY:       return "DRY (Irrigate Soon)";
        case SOIL_MOISTURE_OPTIMAL:   return "OPTIMAL (Healthy)";
        case SOIL_MOISTURE_WET:       return "MOIST / WET";
        case SOIL_MOISTURE_SATURATED: return "SATURATED / MUDDY";
        default:                      return "UNKNOWN";
    }
}

esp_err_t soil_sensor_deinit(soil_sensor_handle_t handle)
{
    if (handle == NULL) {
        return ESP_OK;
    }
    if (handle->owns_adc_handle && handle->adc_handle != NULL) {
        adc_oneshot_del_unit(handle->adc_handle);
    }
    free(handle);
    return ESP_OK;
}
