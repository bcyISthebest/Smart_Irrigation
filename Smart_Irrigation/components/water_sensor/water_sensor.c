#include "water_sensor.h"
#include <stdlib.h>
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "WATER_SENSOR";

struct water_sensor_dev_t {
    adc_oneshot_unit_handle_t adc_handle;
    bool                      owns_adc_handle;
    adc_channel_t             adc_channel;
    gpio_num_t                do_gpio_num;
    int                       cal_dry_raw;
    int                       cal_full_raw;
    bool                      invert_logic;
};

esp_err_t water_sensor_init(const water_sensor_config_t *config, water_sensor_handle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(config != NULL && out_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Null pointer argument");

    water_sensor_handle_t dev = calloc(1, sizeof(struct water_sensor_dev_t));
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_NO_MEM, TAG, "Failed to allocate memory for water sensor");

    dev->adc_channel = config->adc_channel;
    dev->do_gpio_num = config->do_gpio_num;
    dev->invert_logic = config->invert_logic;

    // Use supplied calibration values or apply sane defaults
    if (config->invert_logic) {
        dev->cal_dry_raw = (config->cal_dry_raw > 0) ? config->cal_dry_raw : 3900;
        dev->cal_full_raw = (config->cal_full_raw > 0) ? config->cal_full_raw : 1000;
    } else {
        dev->cal_dry_raw = (config->cal_dry_raw > 0) ? config->cal_dry_raw : 80;
        dev->cal_full_raw = (config->cal_full_raw > 0) ? config->cal_full_raw : 3100;
    }

    // 1. Initialize or assign ADC1 unit
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

    // 2. Configure ADC channel (12-bit resolution, 12dB attenuation for ~0 - 3.3V range)
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

    // 3. Configure DO (Digital Output) pin if provided
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
    ESP_LOGI(TAG, "Water sensor initialized (ADC1 Chan: %d, DO Pin: %d, Cal Dry: %d, Cal Full: %d)",
             dev->adc_channel, dev->do_gpio_num, dev->cal_dry_raw, dev->cal_full_raw);
    return ESP_OK;
}

esp_err_t water_sensor_read_raw(water_sensor_handle_t handle, int *raw_out)
{
    ESP_RETURN_ON_FALSE(handle != NULL && raw_out != NULL, ESP_ERR_INVALID_ARG, TAG, "Null pointer argument");

    // Sample 8 times to average out noise on water sensor traces
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

esp_err_t water_sensor_read_voltage(water_sensor_handle_t handle, int *voltage_mv)
{
    ESP_RETURN_ON_FALSE(handle != NULL && voltage_mv != NULL, ESP_ERR_INVALID_ARG, TAG, "Null pointer argument");

    int raw = 0;
    esp_err_t ret = water_sensor_read_raw(handle, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    // Calculate approximate mV (12-bit ADC max 4095, full scale ~3300 mV with 12dB attenuation)
    *voltage_mv = (raw * 3300) / 4095;
    return ESP_OK;
}

esp_err_t water_sensor_read_level(water_sensor_handle_t handle, float *percentage, water_level_state_t *state)
{
    ESP_RETURN_ON_FALSE(handle != NULL && percentage != NULL, ESP_ERR_INVALID_ARG, TAG, "Null pointer argument");

    int raw = 0;
    esp_err_t ret = water_sensor_read_raw(handle, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    float pct = 0.0f;
    int dry = handle->cal_dry_raw;
    int full = handle->cal_full_raw;

    if (handle->invert_logic || dry > full) {
        // Inverted logic: higher ADC reading = dry in air, lower ADC reading = submerged
        if (dry != full) {
            pct = ((float)(dry - raw) / (float)(dry - full)) * 100.0f;
        }
    } else {
        // Standard logic: lower ADC reading = dry in air, higher ADC reading = submerged
        if (full != dry) {
            pct = ((float)(raw - dry) / (float)(full - dry)) * 100.0f;
        }
    }

    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    *percentage = pct;

    if (state != NULL) {
        if (pct < 10.0f) {
            *state = WATER_LEVEL_DRY;
        } else if (pct < 35.0f) {
            *state = WATER_LEVEL_LOW;
        } else if (pct < 70.0f) {
            *state = WATER_LEVEL_MEDIUM;
        } else if (pct < 90.0f) {
            *state = WATER_LEVEL_HIGH;
        } else {
            *state = WATER_LEVEL_FULL;
        }
    }

    return ESP_OK;
}

esp_err_t water_sensor_read_digital(water_sensor_handle_t handle, bool *threshold_exceeded)
{
    ESP_RETURN_ON_FALSE(handle != NULL && threshold_exceeded != NULL, ESP_ERR_INVALID_ARG, TAG, "Null pointer argument");
    if (handle->do_gpio_num == GPIO_NUM_NC || handle->do_gpio_num < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    // On LM393 comparator modules, DO is typically active-low (goes LOW when threshold is reached)
    int level = gpio_get_level(handle->do_gpio_num);
    *threshold_exceeded = (level == 0);
    return ESP_OK;
}

const char* water_sensor_state_to_str(water_level_state_t state)
{
    switch (state) {
        case WATER_LEVEL_DRY:    return "DRY (No Water)";
        case WATER_LEVEL_LOW:    return "LOW";
        case WATER_LEVEL_MEDIUM: return "MEDIUM";
        case WATER_LEVEL_HIGH:   return "HIGH";
        case WATER_LEVEL_FULL:   return "FULL / SUBMERGED";
        default:                 return "UNKNOWN";
    }
}

esp_err_t water_sensor_deinit(water_sensor_handle_t handle)
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
