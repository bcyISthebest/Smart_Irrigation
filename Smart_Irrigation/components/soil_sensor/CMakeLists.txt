idf_component_register(
    SRCS "soil_sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_adc esp_timer
)
