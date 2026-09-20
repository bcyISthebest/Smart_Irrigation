/*
 =============================================================================
  Smart Irrigation System - ESP-IDF Stub
  File: src/main.c
 =============================================================================
  This file is the ESP-IDF entry point ONLY.
  It is guarded with #ifndef ARDUINO so that when PlatformIO compiles it
  under the Arduino framework (env:esp32doit-arduino), it emits zero
  symbols and causes no linker conflicts.

  The COMPLETE system firmware lives in:
    -> arduino_src/main.cpp   (PlatformIO Arduino build)
    -> Smart_Irrigation.ino   (Arduino IDE)
 =============================================================================
*/

/* Compile this stub ONLY under the ESP-IDF framework */
#ifndef ARDUINO

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGW(TAG, "=======================================================");
    ESP_LOGW(TAG, " This is the ESP-IDF stub build (component library).");
    ESP_LOGW(TAG, " For the FULL system, flash arduino_src/main.cpp via:");
    ESP_LOGW(TAG, " pio run -e esp32doit-arduino -t upload");
    ESP_LOGW(TAG, "=======================================================");

    /* Nothing to do - the real firmware is in arduino_src/main.cpp */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

#endif /* ARDUINO */
