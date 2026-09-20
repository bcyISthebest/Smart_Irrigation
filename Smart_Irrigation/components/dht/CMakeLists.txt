idf_component_register(
    SRCS "dht.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer esp_rom
)
