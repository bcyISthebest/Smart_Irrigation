/*
 ================================================================================
  PROJECT : Smart Irrigation System with ESP32
  FILE    : Smart_Irrigation.ino
  TARGET  : ESP32 Development Board (Arduino framework)
  IDE     : Arduino IDE  ─OR─  PlatformIO [env:esp32doit-arduino]
 ================================================================================
  REQUIRED LIBRARIES (Install via Arduino Library Manager or platformio.ini):
    1. "DHT sensor library"         by Adafruit   (^1.4.7)
    2. "Adafruit Unified Sensor"    by Adafruit   (^1.1.14)
    3. "NimBLE-Arduino"             by h2zero      (^1.4.2)
    4. "Blynk"                      by Volodymyr Shymanskyy (^1.3.2)
 ================================================================================
  FEATURES:
    ✔ Wi-Fi Web Server  – Access Point (AP) + optional Home Wi-Fi (STA) mode
    ✔ Bluetooth Low Energy (BLE) – Nordic UART Service for phone/PC control
    ✔ Auto irrigation with hysteresis & low-water safety cutoff
    ✔ Manual pump override via web or BLE
    ✔ On-the-fly threshold tuning (web UI + BLE commands)
    ✔ NVS / Preferences – thresholds survive power cycles
    ✔ Anti-electrolysis water sensor – VCC switched for only ~100 ms per reading
    ✔ Relay High-Z trick – prevents optocoupler stick-ON on 5V/12V modules
    ✔ Safety watchdog – max continuous pump runtime
    ✔ Live dashboard with animated water fill, pump glow & moisture trend
 ================================================================================
  WIRING QUICK REFERENCE:
    GPIO  4  ← DHT11/DHT22 Data (10 kΩ pull-up to 3.3 V)
    GPIO 34  ← Soil Moisture Sensor  AO  (input-only ADC pin)
    GPIO 35  ← Water Level Sensor    AO  (input-only ADC pin)
    GPIO 25  → Water Level Sensor    VCC (switched power – anti-electrolysis!)
    GPIO 23  → Relay module          IN  (active-LOW, High-Z when OFF)
 ================================================================================
*/

// ============================================================================
//  BLYNK IOT CONFIGURATION (Must precede all includes)
// ============================================================================
// Paste your Template ID and Name from Blynk Console -> Developer Zone -> Templates
#define BLYNK_TEMPLATE_ID   "TMPL3Yic5OqHo"
#define BLYNK_TEMPLATE_NAME "Smart Irrigation"
#define BLYNK_AUTH_TOKEN    "Nolq8tRwI0qlmqj9C3dnzPU7NkkRbE5z"

// ============================================================================
//  LIBRARY INCLUDES
// ============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DHT.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <NimBLE2904.h>

// ============================================================================
//  USER CONFIGURATION  ← Edit these values for your setup
// ============================================================================

// --- 1. WI-FI CREDENTIALS ---
// Leave WIFI_SSID as "YOUR_WIFI_SSID" to skip home router connection.
const char* WIFI_SSID = "YAGNIKJIO4A2,4";
const char* WIFI_PASS = "chirchinkoam4@";

// ESP32 built-in Access Point (always active)
const char* AP_SSID = "Smart_Irrigation_ESP32";
const char* AP_PASS = "12345678";   // Min 8 chars; set "" for open network

// --- 2. GPIO PIN ASSIGNMENTS ---
#define DHT_PIN           4     // DHT11/22 data line
#define SOIL_PIN          34    // Soil moisture sensor analogue out (input-only ADC pin)
#define WATER_PIN         35    // Water level sensor analogue out (input-only ADC pin)
#define WATER_POWER_PIN   25    // Water sensor VCC (switched to prevent electrolysis)
#define RELAY_PIN         23    // Relay IN signal
#define LED_PIN           2     // Built-in status LED (solid ON when active, blinks on sensor read)

// --- 3. SENSOR / HARDWARE CONFIG ---
#define DHT_TYPE          DHT11   // Change to DHT22 if you use DHT22 / AM2302
#define RELAY_ACTIVE_LOW  true    // True for standard optocoupler relay boards
#define SOIL_INVERT       true    // True if higher ADC = drier soil (capacitive & pull-up resistive)
#define WATER_INVERT      false   // Standard resistive water sensor: higher ADC = higher water level

// --- 4. DEFAULT CALIBRATION VALUES (12-bit ADC: 0–4095) ---
//  Calibrate by reading raw values in air (dry) and submerged (wet/full).
int SOIL_DRY_RAW    = 3300;  // ADC in open air  (dry)
int SOIL_WET_RAW    = 1400;  // ADC submerged     (wet)
int WATER_EMPTY_RAW = 0;     // ADC with no water (empty tank - reads 0 in air)
int WATER_FULL_RAW  = 2800;  // ADC fully submerged (full tank - reads ~2500-3200 submerged)

// --- 5. IRRIGATION THRESHOLDS (%) – saved to NVS ---
float soilDryThreshold  = 30.0f;  // Pump ON  below this soil moisture %
float soilWetThreshold  = 65.0f;  // Pump OFF above this soil moisture %
float waterMinThreshold = 15.0f;  // Minimum tank % required to allow pumping

// --- 6. TIMING ---
#define SENSOR_INTERVAL_MS      5000    // How often to run the main sensor loop
#define WATER_POWER_SETTLE_MS     80    // Settle time after powering water sensor (ms)
#define WATER_POWER_ON_MS        100    // Total time water sensor VCC is held HIGH
#define MAX_PUMP_RUNTIME_MS   120000    // Safety cutoff: 2 min max continuous pumping

// ============================================================================
//  BLE UUIDs  (Nordic UART Service)
// ============================================================================
#define BLE_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHAR_RX_UUID        "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // Phone → ESP32
#define BLE_CHAR_TX_UUID        "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // ESP32 → Phone
#define BLE_DEVICE_NAME         "SmartIrrigation"

// ============================================================================
//  SYSTEM STATE
// ============================================================================

enum IrrigationMode { MODE_AUTO, MODE_MANUAL };
enum SystemStatus {
  STATUS_SOIL_MOIST,
  STATUS_IRRIGATING,
  STATUS_LOW_WATER,
  STATUS_MANUAL_ON,
  STATUS_SENSOR_ERROR,
  STATUS_SAFETY_TIMEOUT
};

IrrigationMode currentMode = MODE_AUTO;
SystemStatus   currentStatus = STATUS_SOIL_MOIST;

bool pumpActive    = false;
bool bleEnabled    = true;
bool bleConnected  = false;

unsigned long lastSensorTime  = 0;
unsigned long pumpStartTime   = 0;
unsigned long bootTime        = 0;
unsigned long totalPumpMs     = 0;       // Accumulated pump runtime this session

// Cached sensor readings (updated every SENSOR_INTERVAL_MS)
float  cachedTemp      = 0.0f;
float  cachedHumidity  = 0.0f;
float  cachedSoilPct   = 0.0f;
float  cachedWaterPct  = 0.0f;
int    cachedSoilRaw   = 0;
int    cachedWaterRaw  = 0;
bool   dhtValid        = false;

// Moisture history (last 20 readings) for sparkline
#define HISTORY_LEN 20
float soilHistory[HISTORY_LEN];
uint8_t historyIdx = 0;

// ============================================================================
//  OBJECTS
// ============================================================================
DHT        dht(DHT_PIN, DHT_TYPE);
WebServer  server(80);
Preferences prefs;

NimBLEServer*         pServer     = nullptr;
NimBLECharacteristic* pTxChar     = nullptr;
NimBLECharacteristic* pRxChar     = nullptr;

// ============================================================================
//  HARDWARE HELPERS
// ============================================================================

/**
 * Control the relay using the High-Z / Open-Drain trick.
 *
 * Why this is needed for 5 V / 12 V optocoupler relay boards on the ESP32:
 * The optocoupler LED is pulled up to 5 V (or 12 V).  When the ESP32 outputs
 * 3.3 V HIGH the voltage difference (5 V − 3.3 V = 1.7 V) is still enough to
 * keep the LED lit, so the relay NEVER turns fully OFF.
 *
 * Fix:
 *   ON  – drive GPIO OUTPUT LOW  (0 V → sinks current → relay energises)
 *   OFF – switch GPIO to INPUT    (High-Z cuts all current → relay de-energises)
 */
void setPump(bool on) {
  if (on) {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? LOW : HIGH);
    if (!pumpActive) pumpStartTime = millis();
    pumpActive = true;
  } else {
    if (pumpActive) totalPumpMs += (millis() - pumpStartTime);
    if (RELAY_ACTIVE_LOW) {
      // High-Z: switch to floating input – cuts optocoupler current completely
      pinMode(RELAY_PIN, INPUT);
    } else {
      pinMode(RELAY_PIN, OUTPUT);
      digitalWrite(RELAY_PIN, LOW);
    }
    pumpActive = false;
  }
  if (Blynk.connected()) {
    Blynk.virtualWrite(V4, pumpActive ? 1 : 0);
  }
}

/**
 * Average N ADC samples from a pin.
 */
int readADCAvg(int pin, uint8_t samples = 8) {
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  return (int)(sum / samples);
}

/**
 * Convert raw ADC to percentage using two-point calibration.
 * Handles both inverted and non-inverted sensors automatically.
 */
float rawToPercent(int raw, int calDry, int calWet) {
  // Determine direction: if calDry > calWet the sensor is inverted
  float pct;
  if (calDry > calWet) {
    pct = ((float)(calDry - raw) / (float)(calDry - calWet)) * 100.0f;
  } else {
    pct = ((float)(raw - calDry) / (float)(calWet - calDry)) * 100.0f;
  }
  return constrain(pct, 0.0f, 100.0f);
}

/**
 * Read the water level sensor safely:
 *   1. Power the sensor VCC via WATER_POWER_PIN (anti-electrolysis)
 *   2. Wait for the sensor to settle
 *   3. Average ADC samples
 *   4. Immediately cut VCC
 *
 * The sensor carries current for only ~100 ms rather than continuously,
 * which prevents electrolytic corrosion of the sensor traces.
 */
int readWaterLevelRaw() {
  // Power ON
  digitalWrite(WATER_POWER_PIN, HIGH);
  delay(WATER_POWER_SETTLE_MS);          // Allow output to stabilise

  int raw = readADCAvg(WATER_PIN);

  // Power OFF  – cuts corrosive DC immediately
  digitalWrite(WATER_POWER_PIN, LOW);
  return raw;
}

const char* statusString() {
  switch (currentStatus) {
    case STATUS_IRRIGATING:     return "IRRIGATING (AUTO)";
    case STATUS_SOIL_MOIST:     return "SOIL MOIST (IDLE)";
    case STATUS_LOW_WATER:      return "LOW WATER — LOCKED OUT";
    case STATUS_MANUAL_ON:      return "MANUAL OVERRIDE (ON)";
    case STATUS_SENSOR_ERROR:   return "DHT SENSOR WARNING";
    case STATUS_SAFETY_TIMEOUT: return "SAFETY TIMEOUT";
    default:                    return "IDLE";
  }
}

// ============================================================================
//  NVS / PREFERENCES  (survive reboot)
// ============================================================================
void loadPreferences() {
  prefs.begin("irrigation", true);   // read-only namespace
  soilDryThreshold  = prefs.getFloat("soilDry",  soilDryThreshold);
  soilWetThreshold  = prefs.getFloat("soilWet",  soilWetThreshold);
  waterMinThreshold = prefs.getFloat("waterMin",  waterMinThreshold);
  prefs.end();
}

void savePreferences() {
  prefs.begin("irrigation", false);  // read-write
  prefs.putFloat("soilDry",  soilDryThreshold);
  prefs.putFloat("soilWet",  soilWetThreshold);
  prefs.putFloat("waterMin", waterMinThreshold);
  prefs.end();
}

// ============================================================================
//  BLE CALLBACKS
// ============================================================================

/** Called when a BLE client (phone) connects or disconnects */
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pSvr) override {
    bleConnected = true;
    Serial.println(F("[BLE] Client connected"));
  }
  void onDisconnect(NimBLEServer* pSvr) override {
    bleConnected = false;
    Serial.println(F("[BLE] Client disconnected – restarting advertising"));
    NimBLEDevice::startAdvertising();
  }
};

/**
 * Called when the phone writes to the RX characteristic.
 *
 * Supported commands (case-insensitive):
 *   AUTO         – Switch to automatic irrigation mode
 *   MANUAL       – Switch to manual mode
 *   PUMP_ON      – Turn pump ON  (manual mode only, water check enforced)
 *   PUMP_OFF     – Turn pump OFF
 *   SET_DRY=<n>  – Set soil dry threshold %
 *   SET_WET=<n>  – Set soil wet threshold %
 *   SET_MIN=<n>  – Set minimum tank % threshold
 *   STATUS       – Request immediate JSON status reply
 */
class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar) override {
    std::string raw = pChar->getValue();
    String cmd = String(raw.c_str());
    cmd.trim();
    cmd.toUpperCase();

    Serial.printf("[BLE] RX: %s\n", cmd.c_str());

    if (cmd == "AUTO") {
      currentMode = MODE_AUTO;
    } else if (cmd == "MANUAL") {
      currentMode = MODE_MANUAL;
    } else if (cmd == "PUMP_ON") {
      if (currentMode == MODE_MANUAL) {
        if (cachedWaterPct >= waterMinThreshold) {
          setPump(true);
          currentStatus = STATUS_MANUAL_ON;
        } else {
          if (pTxChar) pTxChar->setValue("ERR: Tank too low");
          if (pTxChar) pTxChar->notify();
        }
      }
    } else if (cmd == "PUMP_OFF") {
      setPump(false);
      currentStatus = STATUS_SOIL_MOIST;
    } else if (cmd.startsWith("SET_DRY=")) {
      soilDryThreshold = cmd.substring(8).toFloat();
      savePreferences();
      if (Blynk.connected()) Blynk.virtualWrite(V5, (int)soilDryThreshold);
    } else if (cmd.startsWith("SET_WET=")) {
      soilWetThreshold = cmd.substring(8).toFloat();
      savePreferences();
      if (Blynk.connected()) Blynk.virtualWrite(V6, (int)soilWetThreshold);
    } else if (cmd.startsWith("SET_MIN=")) {
      waterMinThreshold = cmd.substring(8).toFloat();
      savePreferences();
    }
    // STATUS or any unknown command → triggers JSON reply below
  }
};

/** Send current system state as compact JSON over BLE TX characteristic */
void bleSendStatus() {
  if (!bleConnected || !pTxChar) return;

  char buf[220];
  snprintf(buf, sizeof(buf),
    "{\"t\":%.1f,\"h\":%.1f,\"soil\":%.1f,\"water\":%.1f,"
    "\"pump\":%s,\"mode\":\"%s\",\"status\":\"%s\","
    "\"uptime\":%lu}",
    cachedTemp, cachedHumidity,
    cachedSoilPct, cachedWaterPct,
    pumpActive ? "true" : "false",
    currentMode == MODE_AUTO ? "auto" : "manual",
    statusString(),
    (millis() - bootTime) / 1000UL
  );

  pTxChar->setValue(buf);
  pTxChar->notify();
}

void setupBLE() {
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);   // Max TX power

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService* pService = pServer->createService(BLE_SERVICE_UUID);

  // TX characteristic: ESP32 → phone (notify)
  pTxChar = pService->createCharacteristic(
    BLE_CHAR_TX_UUID,
    NIMBLE_PROPERTY::NOTIFY
  );

  // RX characteristic: phone → ESP32 (write)
  pRxChar = pService->createCharacteristic(
    BLE_CHAR_RX_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  pRxChar->setCallbacks(new RxCallbacks());

  pService->start();

  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(BLE_SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->start();

  Serial.println(F("[BLE] Nordic UART Service started – advertising"));
}

// ============================================================================
//  BLYNK CALLBACKS & HANDLERS
// ============================================================================

// V4: Pump Manual Switch (0 = OFF, 1 = ON)
BLYNK_WRITE(V4) {
  int switchVal = param.asInt();
  if (switchVal == 1) {
    if (cachedWaterPct >= waterMinThreshold) {
      currentMode = MODE_MANUAL;
      setPump(true);
      currentStatus = STATUS_MANUAL_ON;
      Serial.println(F("[Blynk] Manual Pump ON"));
    } else {
      Serial.println(F("[Blynk] Pump ON blocked: Water tank level too low!"));
      Blynk.virtualWrite(V4, 0); // Revert switch in Blynk console
    }
  } else {
    setPump(false);
    currentMode = MODE_AUTO; // Resume automatic regulation
    currentStatus = STATUS_SOIL_MOIST;
    Serial.println(F("[Blynk] Pump OFF (Resumed Auto Mode)"));
  }
}

// V5: Soil Moisture Min Threshold (Dry % -> Pump Turns ON)
BLYNK_WRITE(V5) {
  float newMin = param.asFloat();
  if (newMin >= 0.0f && newMin < soilWetThreshold) {
    soilDryThreshold = newMin;
    savePreferences();
    Serial.printf("[Blynk] Min threshold set to: %.0f%%\n", soilDryThreshold);
  } else {
    Serial.println(F("[Blynk] Invalid Min Threshold (must be less than Max)"));
    Blynk.virtualWrite(V5, (int)soilDryThreshold); // Revert slider
  }
}

// V6: Soil Moisture Max Threshold (Wet % -> Pump Turns OFF)
BLYNK_WRITE(V6) {
  float newMax = param.asFloat();
  if (newMax > soilDryThreshold && newMax <= 100.0f) {
    soilWetThreshold = newMax;
    savePreferences();
    Serial.printf("[Blynk] Max threshold set to: %.0f%%\n", soilWetThreshold);
  } else {
    Serial.println(F("[Blynk] Invalid Max Threshold (must be greater than Min)"));
    Blynk.virtualWrite(V6, (int)soilWetThreshold); // Revert slider
  }
}

// Sync values when device connects to Blynk Cloud
BLYNK_CONNECTED() {
  Serial.println(F("[Blynk] Connected to Blynk Cloud!"));
  Blynk.virtualWrite(V4, pumpActive ? 1 : 0);
  Blynk.virtualWrite(V5, (int)soilDryThreshold);
  Blynk.virtualWrite(V6, (int)soilWetThreshold);
}

// ============================================================================
//  WEB DASHBOARD HTML / CSS / JS
// ============================================================================

const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Smart Irrigation Dashboard</title>
  <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600;700;900&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg:           #0b1220;
      --surface:      rgba(18, 28, 48, 0.92);
      --border:       rgba(255,255,255,0.07);
      --primary:      #00e5a0;
      --primary-dim:  rgba(0,229,160,0.15);
      --primary-glow: rgba(0,229,160,0.35);
      --blue:         #3b82f6;
      --blue-dim:     rgba(59,130,246,0.15);
      --amber:        #f59e0b;
      --red:          #ef4444;
      --red-dim:      rgba(239,68,68,0.15);
      --text:         #f1f5f9;
      --text-dim:     #64748b;
      --text-mid:     #94a3b8;
    }
    *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: 'Outfit', sans-serif;
      background: var(--bg);
      background-image:
        radial-gradient(ellipse 80% 50% at 20% -10%, rgba(0,229,160,0.08) 0%, transparent 60%),
        radial-gradient(ellipse 60% 40% at 80% 110%, rgba(59,130,246,0.07) 0%, transparent 60%);
      color: var(--text);
      min-height: 100vh;
      padding: 20px 16px 40px;
    }
    .container { max-width: 960px; margin: 0 auto; }

    /* ── HEADER ─────────────────────────────────────────── */
    header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 28px;
      padding-bottom: 20px;
      border-bottom: 1px solid var(--border);
      flex-wrap: wrap;
      gap: 12px;
    }
    .logo { display: flex; align-items: center; gap: 12px; }
    .logo-icon {
      width: 42px; height: 42px;
      background: var(--primary-dim);
      border: 1px solid var(--primary);
      border-radius: 12px;
      display: flex; align-items: center; justify-content: center;
      font-size: 22px;
      box-shadow: 0 0 18px var(--primary-glow);
    }
    h1 { font-size: 1.45rem; font-weight: 700; letter-spacing: -0.3px; }
    h1 span { color: var(--primary); }
    .header-right { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
    .pill {
      padding: 5px 14px;
      border-radius: 999px;
      font-size: 0.78rem;
      font-weight: 600;
      letter-spacing: 0.8px;
      text-transform: uppercase;
      border: 1px solid var(--border);
      background: var(--surface);
      color: var(--text-mid);
    }
    .pill.green  { background: var(--primary-dim); border-color: var(--primary); color: var(--primary); box-shadow: 0 0 10px var(--primary-glow); }
    .pill.red    { background: var(--red-dim);     border-color: var(--red);     color: var(--red); }
    .pill.blue   { background: var(--blue-dim);    border-color: var(--blue);    color: var(--blue); }

    /* ── SECTION TITLE ──────────────────────────────────── */
    .section-title {
      font-size: 0.7rem;
      font-weight: 700;
      letter-spacing: 2px;
      text-transform: uppercase;
      color: var(--text-dim);
      margin-bottom: 14px;
    }

    /* ── SENSOR GRID ────────────────────────────────────── */
    .sensor-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 14px;
      margin-bottom: 28px;
    }
    .card {
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: 18px;
      padding: 22px;
      transition: transform 0.22s ease, box-shadow 0.22s ease;
      position: relative;
      overflow: hidden;
    }
    .card:hover { transform: translateY(-3px); box-shadow: 0 12px 40px rgba(0,0,0,0.4); }
    .card::before {
      content: '';
      position: absolute;
      top: 0; left: 0; right: 0;
      height: 2px;
      border-radius: 18px 18px 0 0;
      background: var(--accent, var(--primary));
      opacity: 0.7;
    }
    .card.blue-accent  { --accent: var(--blue); }
    .card.amber-accent { --accent: var(--amber); }
    .card.red-accent   { --accent: var(--red); }

    .card-header {
      display: flex; justify-content: space-between; align-items: flex-start;
      margin-bottom: 16px;
    }
    .card-label { font-size: 0.78rem; color: var(--text-dim); font-weight: 600; letter-spacing: 0.8px; text-transform: uppercase; }
    .card-icon  { font-size: 1.3rem; opacity: 0.85; }
    .card-value { font-size: 2.4rem; font-weight: 900; line-height: 1; margin-bottom: 14px; }
    .card-sub   { font-size: 0.82rem; color: var(--text-mid); }

    /* Progress bar */
    .bar-track {
      width: 100%; height: 8px;
      background: rgba(255,255,255,0.05);
      border-radius: 99px;
      overflow: hidden;
      margin-bottom: 8px;
    }
    .bar-fill {
      height: 100%;
      border-radius: 99px;
      background: var(--primary);
      width: 0%;
      transition: width 0.7s cubic-bezier(0.4,0,0.2,1);
    }
    .bar-fill.blue  { background: var(--blue); }
    .bar-fill.amber { background: var(--amber); }
    .bar-fill.red   { background: var(--red); }

    /* Water tank visualiser */
    .water-tank {
      width: 100%; height: 80px;
      background: rgba(255,255,255,0.04);
      border: 1px solid rgba(59,130,246,0.2);
      border-radius: 10px;
      position: relative;
      overflow: hidden;
      margin-bottom: 10px;
    }
    .water-fill {
      position: absolute;
      bottom: 0; left: 0; right: 0;
      background: linear-gradient(180deg, rgba(59,130,246,0.5), rgba(37,99,235,0.8));
      transition: height 0.8s cubic-bezier(0.4,0,0.2,1);
      border-radius: 0 0 9px 9px;
    }
    .water-wave {
      position: absolute;
      top: -8px; left: -20px; right: -20px;
      height: 16px;
      background: rgba(59,130,246,0.4);
      border-radius: 50%;
      animation: wave 2.5s ease-in-out infinite;
    }
    @keyframes wave {
      0%,100% { transform: translateX(0) scaleY(1); }
      50%      { transform: translateX(8px) scaleY(0.6); }
    }

    /* Sparkline canvas */
    canvas#sparkline { width: 100%; height: 40px; margin-top: 10px; border-radius: 6px; }

    /* ── CLIMATE CARD ───────────────────────────────────── */
    .climate-row {
      display: flex; gap: 20px; align-items: flex-end; margin-bottom: 8px;
    }
    .climate-item { flex: 1; }
    .climate-big  { font-size: 2.4rem; font-weight: 900; }
    .climate-lbl  { font-size: 0.75rem; color: var(--text-dim); text-transform: uppercase; letter-spacing: 1px; }

    /* ── CONTROL PANEL ──────────────────────────────────── */
    .control-panel {
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: 18px;
      padding: 24px;
      margin-bottom: 20px;
    }
    .panel-title {
      font-size: 1rem; font-weight: 700; margin-bottom: 18px;
      display: flex; align-items: center; gap: 8px;
    }
    .pump-indicator {
      display: inline-block;
      width: 10px; height: 10px;
      border-radius: 50%;
      background: var(--text-dim);
      margin-right: 6px;
      transition: background 0.3s, box-shadow 0.3s;
    }
    .pump-indicator.on {
      background: var(--primary);
      box-shadow: 0 0 10px var(--primary-glow);
      animation: pulse 1.5s ease-in-out infinite;
    }
    @keyframes pulse {
      0%,100% { box-shadow: 0 0 6px  var(--primary-glow); }
      50%      { box-shadow: 0 0 18px var(--primary-glow); }
    }

    .btn-row { display: flex; flex-wrap: wrap; gap: 10px; margin-bottom: 20px; }
    button {
      padding: 11px 22px;
      border-radius: 10px;
      font-size: 0.9rem;
      font-weight: 600;
      cursor: pointer;
      border: none;
      transition: all 0.2s ease;
      font-family: 'Outfit', sans-serif;
    }
    .btn-green   { background: var(--primary); color: #000; }
    .btn-green:hover { opacity: 0.88; box-shadow: 0 0 18px var(--primary-glow); }
    .btn-red     { background: var(--red); color: #fff; }
    .btn-ghost   { background: rgba(255,255,255,0.07); color: var(--text); border: 1px solid var(--border); }
    .btn-ghost:hover { background: rgba(255,255,255,0.14); }
    .btn-ghost.sel { border-color: var(--primary); color: var(--primary); background: var(--primary-dim); }
    button:active { transform: scale(0.97); }

    /* Divider */
    .divider { border: none; border-top: 1px solid var(--border); margin: 20px 0; }

    /* Threshold settings grid */
    .settings-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
      gap: 14px;
    }
    .input-group label {
      display: block; font-size: 0.78rem; color: var(--text-dim);
      margin-bottom: 6px; text-transform: uppercase; letter-spacing: 0.8px;
    }
    .input-group input[type=number] {
      width: 100%; padding: 10px 12px;
      background: rgba(0,0,0,0.35);
      border: 1px solid var(--border);
      border-radius: 9px;
      color: var(--text);
      font-size: 1rem;
      font-family: 'Outfit', sans-serif;
      transition: border-color 0.2s;
    }
    .input-group input[type=number]:focus {
      outline: none; border-color: var(--primary);
    }

    /* ── BLE / STATS PANEL ──────────────────────────────── */
    .info-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
      gap: 14px;
      margin-bottom: 28px;
    }
    .info-card {
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: 14px;
      padding: 16px 20px;
      display: flex; align-items: center; gap: 14px;
    }
    .info-icon { font-size: 1.6rem; }
    .info-label { font-size: 0.72rem; color: var(--text-dim); text-transform: uppercase; letter-spacing: 1px; }
    .info-val   { font-size: 1.05rem; font-weight: 700; margin-top: 2px; }

    /* ── FOOTER ─────────────────────────────────────────── */
    footer {
      text-align: center;
      color: var(--text-dim);
      font-size: 0.78rem;
      margin-top: 32px;
      padding-top: 20px;
      border-top: 1px solid var(--border);
    }

    /* ── TOAST NOTIFICATION ─────────────────────────────── */
    #toast {
      position: fixed; bottom: 24px; left: 50%; transform: translateX(-50%);
      background: rgba(18,28,48,0.98); border: 1px solid var(--primary);
      color: var(--primary); padding: 12px 24px; border-radius: 12px;
      font-size: 0.88rem; font-weight: 600;
      opacity: 0; transition: opacity 0.3s;
      pointer-events: none; z-index: 999;
    }
    #toast.show { opacity: 1; }
  </style>
</head>
<body>
<div class="container">

  <!-- HEADER -->
  <header>
    <div class="logo">
      <div class="logo-icon">🌱</div>
      <div>
        <h1>Smart <span>Irrigation</span></h1>
        <div style="font-size:0.78rem;color:var(--text-dim);margin-top:2px;">ESP32 Control Dashboard</div>
      </div>
    </div>
    <div class="header-right">
      <div id="ble-pill"  class="pill">BLE –</div>
      <div id="status-pill" class="pill">CONNECTING…</div>
    </div>
  </header>

  <!-- SENSOR CARDS -->
  <div class="section-title">Live Sensor Readings</div>
  <div class="sensor-grid">

    <!-- Soil Moisture -->
    <div class="card">
      <div class="card-header">
        <span class="card-label">🌿 Soil Moisture</span>
        <span id="soil-raw" class="card-icon" style="font-size:0.75rem;color:var(--text-dim);">raw –</span>
      </div>
      <div class="card-value" id="soil-val">– %</div>
      <div class="bar-track"><div class="bar-fill" id="soil-bar"></div></div>
      <div class="card-sub" id="soil-sub">Optimal: 30 – 65 %</div>
      <canvas id="sparkline"></canvas>
    </div>

    <!-- Water Tank -->
    <div class="card blue-accent">
      <div class="card-header">
        <span class="card-label">💧 Water Reservoir</span>
        <span id="water-raw" class="card-icon" style="font-size:0.75rem;color:var(--text-dim);">raw –</span>
      </div>
      <div class="card-value" id="water-val">– %</div>
      <div class="water-tank">
        <div class="water-fill" id="water-fill" style="height:0%">
          <div class="water-wave"></div>
        </div>
      </div>
      <div class="card-sub" id="water-sub">Minimum floor: 15 %</div>
    </div>

    <!-- Climate -->
    <div class="card amber-accent">
      <div class="card-header">
        <span class="card-label">🌡️ Ambient Climate</span>
        <span id="dht-status" class="card-icon" style="font-size:0.75rem;color:var(--text-dim);">DHT –</span>
      </div>
      <div class="climate-row">
        <div class="climate-item">
          <div class="climate-lbl">Temperature</div>
          <div class="climate-big" id="temp-val">– °C</div>
        </div>
        <div class="climate-item">
          <div class="climate-lbl">Humidity</div>
          <div class="climate-big" id="hum-val">– %</div>
        </div>
      </div>
      <div class="card-sub" id="climate-sub">–</div>
    </div>
  </div>

  <!-- INFO STRIP -->
  <div class="info-grid">
    <div class="info-card">
      <div class="info-icon">⏱️</div>
      <div><div class="info-label">Uptime</div><div class="info-val" id="uptime-val">–</div></div>
    </div>
    <div class="info-card">
      <div class="info-icon">💦</div>
      <div><div class="info-label">Pump Runtime (session)</div><div class="info-val" id="pump-time-val">–</div></div>
    </div>
    <div class="info-card">
      <div class="info-icon">📶</div>
      <div><div class="info-label">Wi-Fi Mode</div><div class="info-val" id="wifi-mode-val">–</div></div>
    </div>
    <div class="info-card">
      <div class="info-icon">🔵</div>
      <div><div class="info-label">BLE Status</div><div class="info-val" id="ble-status-val">–</div></div>
    </div>
  </div>

  <!-- CONTROL PANEL -->
  <div class="control-panel">
    <div class="panel-title">
      <span class="pump-indicator" id="pump-dot"></span>
      Pump Control — <span id="pump-text" style="color:var(--text-dim)">OFF</span>
    </div>

    <div class="btn-row">
      <button id="btn-auto"   class="btn-ghost" onclick="setMode('auto')">🤖 Auto Mode</button>
      <button id="btn-manual" class="btn-ghost" onclick="setMode('manual')">🕹️ Manual Mode</button>
      <button id="btn-pump"   class="btn-green" onclick="togglePump()" style="display:none;">Turn Pump ON</button>
    </div>

    <hr class="divider">

    <div style="font-size:0.8rem;color:var(--text-dim);margin-bottom:14px;font-weight:600;text-transform:uppercase;letter-spacing:1px;">⚙️ Irrigation Thresholds</div>
    <div class="settings-grid">
      <div class="input-group">
        <label>Soil Dry Trigger (%)</label>
        <input type="number" id="cfg-dry" min="0" max="100" step="1">
      </div>
      <div class="input-group">
        <label>Soil Wet Target (%)</label>
        <input type="number" id="cfg-wet" min="0" max="100" step="1">
      </div>
      <div class="input-group">
        <label>Min Tank Level (%)</label>
        <input type="number" id="cfg-tank" min="0" max="100" step="1">
      </div>
    </div>
    <button class="btn-ghost" onclick="saveSettings()" style="margin-top:16px;width:100%;">💾 Save &amp; Apply Thresholds</button>
  </div>

  <footer>Smart Irrigation System &mdash; ESP32 &bull; DHT &bull; Soil Sensor &bull; Water Level &bull; BLE + Wi-Fi</footer>
</div>

<div id="toast"></div>

<script>
  // ── State ────────────────────────────────────────────────
  let pumpActive = false;
  let soilHistory = [];
  const HIST = 20;

  // ── Helpers ──────────────────────────────────────────────
  function toast(msg, ms = 2400) {
    const t = document.getElementById('toast');
    t.textContent = msg;
    t.classList.add('show');
    setTimeout(() => t.classList.remove('show'), ms);
  }

  function fmtSeconds(s) {
    s = Math.floor(s);
    if (s < 60) return s + 's';
    const m = Math.floor(s / 60), sec = s % 60;
    if (m < 60) return m + 'm ' + sec + 's';
    const h = Math.floor(m / 60), min = m % 60;
    return h + 'h ' + min + 'm';
  }

  // ── Sparkline ────────────────────────────────────────────
  function drawSparkline(data) {
    const c = document.getElementById('sparkline');
    if (!c) return;
    const ctx = c.getContext('2d');
    c.width  = c.offsetWidth  * window.devicePixelRatio;
    c.height = c.offsetHeight * window.devicePixelRatio;
    ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
    const w = c.offsetWidth, h = c.offsetHeight;
    ctx.clearRect(0, 0, w, h);
    if (data.length < 2) return;

    const min = 0, max = 100;
    const step = w / (data.length - 1);

    // Gradient fill
    const grad = ctx.createLinearGradient(0, 0, 0, h);
    grad.addColorStop(0,   'rgba(0,229,160,0.35)');
    grad.addColorStop(1,   'rgba(0,229,160,0.02)');

    ctx.beginPath();
    data.forEach((v, i) => {
      const x = i * step;
      const y = h - ((v - min) / (max - min)) * h;
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.strokeStyle = '#00e5a0';
    ctx.lineWidth   = 1.8;
    ctx.stroke();

    // Fill below line
    ctx.lineTo(w, h); ctx.lineTo(0, h);
    ctx.closePath();
    ctx.fillStyle = grad;
    ctx.fill();
  }

  // ── Fetch & Update ───────────────────────────────────────
  async function fetchData() {
    try {
      const res = await fetch('/api/data');
      if (!res.ok) throw new Error(res.status);
      const d = await res.json();

      // --- Soil ---
      document.getElementById('soil-val').textContent = d.soilMoisture.toFixed(1) + ' %';
      document.getElementById('soil-bar').style.width = Math.min(100, d.soilMoisture) + '%';
      document.getElementById('soil-raw').textContent = 'raw ' + d.soilRaw;
      const soilPct = Math.min(100, Math.max(0, d.soilMoisture));
      const soilBar = document.getElementById('soil-bar');
      soilBar.className = 'bar-fill' + (soilPct < 30 ? ' red' : soilPct > 70 ? ' blue' : '');
      soilHistory.push(soilPct);
      if (soilHistory.length > HIST) soilHistory.shift();
      drawSparkline(soilHistory);

      // --- Water ---
      const wPct = Math.min(100, Math.max(0, d.waterLevel));
      document.getElementById('water-val').textContent = d.waterLevel.toFixed(1) + ' %';
      document.getElementById('water-fill').style.height = wPct + '%';
      document.getElementById('water-raw').textContent = 'raw ' + d.waterRaw;
      document.getElementById('water-sub').textContent =
        wPct < d.waterMinThresh ? '⚠️ Tank too low — pump locked!' : 'Min floor: ' + d.waterMinThresh + ' %';

      // --- Climate ---
      document.getElementById('temp-val').textContent = d.dhtValid ? d.temp.toFixed(1) + ' °C' : '— °C';
      document.getElementById('hum-val').textContent  = d.dhtValid ? d.humidity.toFixed(1) + ' %' : '— %';
      document.getElementById('dht-status').textContent = d.dhtValid ? '✔' : '⚠';
      const hi = d.temp > 35 ? 'Hot & dry — more watering may be needed'
               : d.temp < 10 ? 'Cold — reduce irrigation frequency'
               : 'Climate normal';
      document.getElementById('climate-sub').textContent = d.dhtValid ? hi : 'DHT sensor read error';

      // --- Status pill ---
      const pill = document.getElementById('status-pill');
      pill.textContent = d.statusText;
      pill.className = 'pill ' + (d.pump ? 'green' : wPct < d.waterMinThresh ? 'red' : '');

      // --- Pump ---
      pumpActive = d.pump;
      const dot  = document.getElementById('pump-dot');
      dot.className = 'pump-indicator' + (pumpActive ? ' on' : '');
      const pt = document.getElementById('pump-text');
      pt.textContent = pumpActive ? 'RUNNING ▶' : 'STOPPED ◼';
      pt.style.color = pumpActive ? 'var(--primary)' : 'var(--text-dim)';

      // --- Mode buttons ---
      document.getElementById('btn-auto').className  = 'btn-ghost' + (d.mode === 'auto'   ? ' sel' : '');
      document.getElementById('btn-manual').className= 'btn-ghost' + (d.mode === 'manual' ? ' sel' : '');
      const pb = document.getElementById('btn-pump');
      if (d.mode === 'manual') {
        pb.style.display  = 'block';
        pb.textContent    = pumpActive ? 'Turn Pump OFF' : 'Turn Pump ON';
        pb.className      = pumpActive ? 'btn-red' : 'btn-green';
      } else { pb.style.display = 'none'; }

      // --- Info strip ---
      document.getElementById('uptime-val').textContent    = fmtSeconds(d.uptime);
      document.getElementById('pump-time-val').textContent = fmtSeconds(d.pumpMs / 1000);
      document.getElementById('wifi-mode-val').textContent = d.wifiMode;
      document.getElementById('ble-status-val').textContent= d.bleConnected ? 'Connected' : (d.bleEnabled ? 'Advertising' : 'Disabled');

      // --- BLE pill ---
      const bp = document.getElementById('ble-pill');
      bp.textContent  = d.bleConnected ? 'BLE ●' : 'BLE ○';
      bp.className    = 'pill' + (d.bleConnected ? ' blue' : '');

      // --- Pre-fill thresholds (first load only) ---
      if (!document.getElementById('cfg-dry').value) {
        document.getElementById('cfg-dry').value  = d.soilDryThresh;
        document.getElementById('cfg-wet').value  = d.soilWetThresh;
        document.getElementById('cfg-tank').value = d.waterMinThresh;
      }
    } catch (err) {
      console.warn('Fetch error:', err);
    }
  }

  async function setMode(m) {
    await fetch('/api/mode?set=' + m);
    fetchData();
  }

  async function togglePump() {
    const next = pumpActive ? 'off' : 'on';
    const res  = await fetch('/api/pump?set=' + next);
    if (!res.ok) { toast('⚠ Tank too low to start pump!'); return; }
    fetchData();
  }

  async function saveSettings() {
    const dry  = document.getElementById('cfg-dry').value;
    const wet  = document.getElementById('cfg-wet').value;
    const tank = document.getElementById('cfg-tank').value;
    if (+dry >= +wet) { toast('⚠ Dry threshold must be below Wet threshold!'); return; }
    await fetch(`/api/config?dry=${dry}&wet=${wet}&tank=${tank}`);
    toast('✔ Thresholds saved to flash!');
    fetchData();
  }

  setInterval(fetchData, 2500);
  fetchData();
</script>
</body>
</html>
)rawliteral";

// ============================================================================
//  WEB SERVER ROUTE HANDLERS
// ============================================================================

void handleRoot() {
  server.send_P(200, "text/html", DASHBOARD_HTML);
}

void handleApiData() {
  // Build Wi-Fi mode string
  String wifiMode = "AP";
  if (WiFi.status() == WL_CONNECTED) {
    wifiMode = "AP + STA (" + WiFi.localIP().toString() + ")";
  }

  unsigned long uptime   = (millis() - bootTime) / 1000UL;
  unsigned long pumpMs   = totalPumpMs + (pumpActive ? (millis() - pumpStartTime) : 0);

  String json = "{";
  json += "\"temp\":"          + String(cachedTemp, 1)       + ",";
  json += "\"humidity\":"      + String(cachedHumidity, 1)   + ",";
  json += "\"soilMoisture\":"  + String(cachedSoilPct, 1)    + ",";
  json += "\"soilRaw\":"       + String(cachedSoilRaw)       + ",";
  json += "\"waterLevel\":"    + String(cachedWaterPct, 1)   + ",";
  json += "\"waterRaw\":"      + String(cachedWaterRaw)      + ",";
  json += "\"pump\":"          + String(pumpActive ? "true" : "false") + ",";
  json += "\"dhtValid\":"      + String(dhtValid   ? "true" : "false") + ",";
  json += "\"bleConnected\":"  + String(bleConnected ? "true" : "false") + ",";
  json += "\"bleEnabled\":"    + String(bleEnabled   ? "true" : "false") + ",";
  json += "\"mode\":\""        + String(currentMode == MODE_AUTO ? "auto" : "manual") + "\",";
  json += "\"statusText\":\""  + String(statusString()) + "\",";
  json += "\"soilDryThresh\":" + String(soilDryThreshold, 0)  + ",";
  json += "\"soilWetThresh\":" + String(soilWetThreshold, 0)  + ",";
  json += "\"waterMinThresh\":" + String(waterMinThreshold, 0) + ",";
  json += "\"uptime\":"        + String(uptime)   + ",";
  json += "\"pumpMs\":"        + String(pumpMs)   + ",";
  json += "\"wifiMode\":\""    + wifiMode + "\"";
  json += "}";

  server.sendHeader("Cache-Control", "no-cache");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void handleApiMode() {
  if (server.hasArg("set")) {
    String m = server.arg("set");
    if (m == "auto") {
      currentMode = MODE_AUTO;
      // If switching back to auto while pump is on, let automation decide
    } else if (m == "manual") {
      currentMode = MODE_MANUAL;
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleApiPump() {
  if (server.hasArg("set") && currentMode == MODE_MANUAL) {
    String p = server.arg("set");
    if (p == "on") {
      if (cachedWaterPct >= waterMinThreshold) {
        setPump(true);
        currentStatus = STATUS_MANUAL_ON;
      } else {
        server.send(400, "text/plain", "ERR: Tank level too low");
        return;
      }
    } else {
      setPump(false);
      currentStatus = STATUS_SOIL_MOIST;
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleApiConfig() {
  bool changed = false;
  if (server.hasArg("dry"))  { soilDryThreshold  = server.arg("dry").toFloat();  changed = true; }
  if (server.hasArg("wet"))  { soilWetThreshold   = server.arg("wet").toFloat();  changed = true; }
  if (server.hasArg("tank")) { waterMinThreshold  = server.arg("tank").toFloat(); changed = true; }
  if (changed) {
    savePreferences();
    if (Blynk.connected()) {
      Blynk.virtualWrite(V5, (int)soilDryThreshold);
      Blynk.virtualWrite(V6, (int)soilWetThreshold);
    }
  }
  server.send(200, "text/plain", "OK");
}

// ============================================================================
//  AUTOMATION LOGIC
// ============================================================================

void runAutomation() {
  bool waterOK = (cachedWaterPct >= waterMinThreshold);
  unsigned long now = millis();

  if (currentMode == MODE_AUTO) {
    if (!waterOK) {
      setPump(false);
      currentStatus = STATUS_LOW_WATER;
      return;
    }

    if (!dhtValid) {
      currentStatus = STATUS_SENSOR_ERROR;
      // Still allow irrigation based on soil moisture alone
    }

    if (cachedSoilPct < soilDryThreshold) {
      setPump(true);
      currentStatus = STATUS_IRRIGATING;
    } else if (cachedSoilPct >= soilWetThreshold) {
      setPump(false);
      currentStatus = STATUS_SOIL_MOIST;
    } else {
      // In hysteresis band – keep current state
      currentStatus = pumpActive ? STATUS_IRRIGATING : STATUS_SOIL_MOIST;
    }

    // Safety watchdog: cut pump if runtime exceeds maximum
    if (pumpActive && (now - pumpStartTime) > MAX_PUMP_RUNTIME_MS) {
      setPump(false);
      currentStatus = STATUS_SAFETY_TIMEOUT;
      Serial.println(F("[SAFETY] Pump runtime watchdog triggered – pump forced OFF"));
    }

  } else {
    // Manual mode: only enforce water safety override
    if (!waterOK && pumpActive) {
      setPump(false);
      currentStatus = STATUS_LOW_WATER;
      Serial.println(F("[SAFETY] Manual mode: tank too low – pump forced OFF"));
    }
  }
}

// ============================================================================
//  SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);

  // Status LED: light up immediately to indicate system is powered and running
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.println(F("\n================================================="));
  Serial.println(F("  SMART IRRIGATION SYSTEM  –  ESP32"));
  Serial.println(F("================================================="));

  bootTime = millis();

  // --- GPIO initialisation ---
  analogReadResolution(12);               // 12-bit ADC (0–4095)
  analogSetAttenuation(ADC_11db);         // Full 0–3.3 V range

  // Water sensor power pin: start LOW (sensor off)
  pinMode(WATER_POWER_PIN, OUTPUT);
  digitalWrite(WATER_POWER_PIN, LOW);

  // Relay: ensure pump is OFF at boot (High-Z)
  setPump(false);

  // --- DHT ---
  // Enable the ESP32 internal pull-up (~45 kΩ) on the DHT data pin.
  // This replaces the need for an external 10 kΩ resistor.
  // NOTE: Works well for short cable runs (< ~30 cm). If you get
  //       frequent checksum errors, add a physical 10 kΩ resistor instead.
  pinMode(DHT_PIN, INPUT_PULLUP);
  dht.begin();

  // --- Initialise soil moisture history buffer ---
  memset(soilHistory, 0, sizeof(soilHistory));

  // --- Load saved thresholds from NVS ---
  loadPreferences();
  Serial.printf("[NVS] Thresholds loaded  Dry=%.0f%%  Wet=%.0f%%  MinTank=%.0f%%\n",
                soilDryThreshold, soilWetThreshold, waterMinThreshold);

  // --- Wi-Fi: Access Point + optional Station mode ---
  WiFi.mode(WIFI_AP_STA);

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[AP]  SSID : %s\n", AP_SSID);
  Serial.printf("[AP]  URL  : http://%s\n", WiFi.softAPIP().toString().c_str());

  if (String(WIFI_SSID) != "YOUR_WIFI_SSID") {
    Serial.printf("[STA] Connecting to '%s' ", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 20) {
      delay(500);
      Serial.print('.');
      tries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("\n[STA] Connected! URL: http://%s\n", WiFi.localIP().toString().c_str());
      if (String(BLYNK_AUTH_TOKEN) != "YOUR_AUTH_TOKEN") {
        Blynk.config(BLYNK_AUTH_TOKEN);
        Blynk.connect();
        Serial.println(F("[Blynk] Connecting to Blynk Cloud..."));
      }
    } else {
      Serial.println(F("\n[STA] Timed out – AP-only mode active"));
    }
  }

  // --- Web Server routes ---
  server.on("/",           handleRoot);
  server.on("/api/data",   handleApiData);
  server.on("/api/mode",   handleApiMode);
  server.on("/api/pump",   handleApiPump);
  server.on("/api/config", handleApiConfig);
  server.begin();
  Serial.println(F("[HTTP] Web server started on port 80"));

  // --- BLE ---
  if (bleEnabled) {
    setupBLE();
  }

  Serial.println(F("\n-------------------------------------------------"));
  Serial.println(F("  System ready!  Connect to dashboard or BLE."));
  Serial.println(F("-------------------------------------------------\n"));
}

// ============================================================================
//  LOOP
// ============================================================================

void loop() {
  // 1. Handle HTTP clients
  server.handleClient();

  // 2. Handle Blynk connection & events (non-blocking)
  if (WiFi.status() == WL_CONNECTED && String(BLYNK_AUTH_TOKEN) != "YOUR_AUTH_TOKEN") {
    Blynk.run();
  }

  unsigned long now = millis();

  // 3. Periodic sensor read + automation cycle
  if (now - lastSensorTime >= SENSOR_INTERVAL_MS) {
    lastSensorTime = now;

    // Heartbeat: brief blink to show active loop execution
    digitalWrite(LED_PIN, LOW);

    // ── DHT (temperature + humidity) ────────────────────
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    dhtValid = (!isnan(t) && !isnan(h));
    if (dhtValid) { cachedTemp = t; cachedHumidity = h; }

    // ── Soil moisture ────────────────────────────────────
    cachedSoilRaw = readADCAvg(SOIL_PIN);
    cachedSoilPct = rawToPercent(cachedSoilRaw, SOIL_DRY_RAW, SOIL_WET_RAW);

    // Update history buffer
    soilHistory[historyIdx] = cachedSoilPct;
    historyIdx = (historyIdx + 1) % HISTORY_LEN;

    // ── Water level (anti-electrolysis pulsed read) ──────
    cachedWaterRaw = readWaterLevelRaw();
    cachedWaterPct = rawToPercent(cachedWaterRaw, WATER_EMPTY_RAW, WATER_FULL_RAW);

    // ── Automation decision ──────────────────────────────
    runAutomation();

    // ── BLE status broadcast ─────────────────────────────
    bleSendStatus();

    // ── Blynk telemetry broadcast ────────────────────────
    if (Blynk.connected()) {
      Blynk.virtualWrite(V0, (int)cachedSoilPct);
      Blynk.virtualWrite(V1, (int)cachedWaterPct);
      if (dhtValid) {
        Blynk.virtualWrite(V2, cachedTemp);
        Blynk.virtualWrite(V3, cachedHumidity);
      }
    }

    // Restore status LED to ON
    digitalWrite(LED_PIN, HIGH);

    // ── Serial monitor summary ───────────────────────────
    Serial.printf("[SENS] Soil: %.1f%% (raw %d) | Water: %.1f%% (raw %d) | "
                  "Temp: %.1f°C | Hum: %.1f%% | Pump: %s | Mode: %s\n",
                  cachedSoilPct, cachedSoilRaw,
                  cachedWaterPct, cachedWaterRaw,
                  cachedTemp, cachedHumidity,
                  pumpActive ? "ON" : "OFF",
                  currentMode == MODE_AUTO ? "AUTO" : "MANUAL");
  }
}
