# 🌱 ESP32 Smart Irrigation System

An intelligent, autonomous irrigation and environmental monitoring system built on the **ESP32 DOIT DevKit v1**. Features a responsive web dashboard, Bluetooth Low Energy (BLE) Nordic UART control, automated soil hydration management, anti-electrolysis sensor protection, and fail-safe watchdog protection.

---

## 🚀 Features

- **Dual Network Operation**:
  - **Access Point (AP)** mode (`Smart_Irrigation_ESP32`, default IP `192.168.4.1`) for direct field connection without a router.
  - **Station (STA)** mode to join an existing home or greenhouse Wi-Fi network.
- **Embedded Web Dashboard**:
  - Real-time sensor telemetry (Soil Moisture, Water Level, Temperature, Humidity).
  - Dynamic visual indicators, animated pump status, and moisture trend tracking.
  - Manual pump toggle and live threshold tuning stored directly in NVS.
- **Bluetooth Low Energy (BLE)**:
  - Nordic UART Service (NUS) for terminal control and monitoring via smartphone apps (e.g., Serial Bluetooth Terminal, nRF Connect).
- **Intelligent Water & Soil Control**:
  - Hysteresis-based watering to prevent pump stutter.
  - Low-water reservoir cutoff to protect submersible pumps from dry running.
  - Configurable max continuous runtime safety watchdog.
- **Hardware Protections**:
  - **Anti-electrolysis water sensor**: VCC is powered only during readings (~100 ms) to eliminate electrode degradation.
  - **High-Z Relay Drive**: Eliminates optocoupler stick-ON common with 5V relay modules connected to 3.3V GPIOs.
- **Persistent Storage (NVS)**: Calibration values and threshold limits persist across reboots.

---

## 📌 Pinout & Wiring Quick Reference

| Component | Pin / Signal | ESP32 GPIO | Notes |
| :--- | :--- | :--- | :--- |
| **DHT11 / DHT22** | DATA | `GPIO 4` | Requires 10 kΩ pull-up resistor to 3.3V |
| **Soil Moisture Sensor** | Analog Out (AO) | `GPIO 35` | Input-only ADC pin |
| **Water Level Sensor** | Analog Out (AO) | `GPIO 34` | Input-only ADC pin |
| **Water Level Sensor** | VCC (Power) | `GPIO 25` | Switched power (Anti-electrolysis) |
| **Relay Module (Pump)**| IN | `GPIO 23` | Active-LOW; uses High-Z state when OFF |

---

## 🛠️ Project Structure

```text
Smart_Irrigation/
├── Smart_Irrigation.ino           # Arduino IDE sketch
├── Smart_Irrigation.code-workspace # VS Code multi-root workspace
├── .gitignore                     # Git ignore rules for build artifacts
└── Smart_Irrigation/              # PlatformIO & ESP-IDF project
    ├── platformio.ini             # PlatformIO build configuration
    ├── arduino_src/
    │   └── main.cpp               # PlatformIO Arduino firmware
    ├── components/                # ESP-IDF sensor driver components
    │   ├── dht/
    │   ├── soil_sensor/
    │   └── water_sensor/
    └── src/
        └── main.c                 # ESP-IDF entry stub
```

---

## 💻 Getting Started

### Option 1: PlatformIO (Recommended)

1. Open this repository in **VS Code** with the **PlatformIO IDE** extension installed.
2. Open the project root or workspace:
   ```bash
   # Build the Arduino firmware
   pio run -d Smart_Irrigation -e esp32doit-arduino

   # Flash to ESP32
   pio run -d Smart_Irrigation -e esp32doit-arduino -t upload

   # Open Serial Monitor (115200 baud)
   pio device monitor -b 115200
   ```

### Option 2: Arduino IDE

1. Open `Smart_Irrigation.ino` in the Arduino IDE.
2. Select board: **DOIT ESP32 DEVKIT V1** (or ESP32 Dev Module).
3. Install required libraries via Arduino Library Manager:
   - `DHT sensor library` by Adafruit
   - `Adafruit Unified Sensor` by Adafruit
   - `NimBLE-Arduino` by h2zero
4. (Optional) Configure Wi-Fi credentials in the sketch:
   ```cpp
   const char* WIFI_SSID = "YOUR_WIFI_SSID";
   const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
   ```
5. Click **Upload**.

---

## 🌐 Connecting to the Dashboard

1. Power on the ESP32.
2. Connect your phone or laptop to the Wi-Fi network:
   - **SSID**: `Smart_Irrigation_ESP32`
   - **Password**: `12345678`
3. Open a browser and navigate to:
   - **`http://192.168.4.1`**

---

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.
