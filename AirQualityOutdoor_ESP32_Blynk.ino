// ============================================================================
// Outdoor Air Quality Station — ESP32 Production Firmware v2.2
// Migrated & rewritten from original Arduino Mega + Esp8266EasyIoT (AM, 2017)
//
// ─── HARDWARE v2.0/2.1 ───────────────────────────────────────────────────────
//   KEPT:       MQ-7 CO board (5V heater, closed-loop via A1 feedback)
//               ML8511 UV (3.3V ratiometric, direct)
//               Sharp GP2Y1010 dust (5V, 10k+10k divider)
//               JY-901/WT901 IMU (UART2 GPIO16/17, 3.3V)
//               LEDs, buzzer, CO PWM
//
//   REPLACED:
//     DHT21          → AHT2x on ENS160 board   (I2C 0x38, 3.3V, no pull-up)
//     MICS-2710 NO2  → ENS160 TVOC/eCO2/AQI    (I2C 0x53, 3.3V, no divider)
//     Analogue mic   → INMP441 I2S 24-bit mic   (GPIO25/26/33, 3.3V)
//     u-blox GPS     → ATGM336H NMEA GPS        (UART1 GPIO13/23, 9600 baud)
//
// ─── MQ-7 HEATER BOARD — CLOSED LOOP (from datasheet analysis) ──────────────
//   Board pins:
//     VCC → ESP32 VIN (5V)
//     GND → GND
//     D2  → GPIO4  (PWM direct, 3.3V logic drives NPN base via 1kΩ — sufficient)
//     A0  → 10kΩ+10kΩ divider → GPIO36 (CO sense, smoothed by 470µF on board)
//     A1  → 10kΩ+10kΩ divider → GPIO39 (heater voltage feedback, 0–5V → 0–2.5V)
//
//   New closed-loop calibration: instead of open-loop PWM sweep, we read A1
//   (heater voltage feedback) and adjust duty cycle until A1 hits target voltage.
//   Heating phase:     target A1 = 5.0V  → duty = 255 (full on)
//   Measurement phase: target A1 = 1.4V  → duty adjusted via closed loop
//
// ─── PIN MAP v2.0 ────────────────────────────────────────────────────────────
//   GPIO36 VP — CO sense AO (10k+10k divider from board A0)
//   GPIO39 VN — CO heater feedback A1 (10k+10k divider from board A1)
//   GPIO34    — UV OUT (ML8511, direct 3.3V)
//   GPIO35    — UV REF (ML8511, direct 3.3V)
//   GPIO32    — Dust AO (GP2Y1010, 10k+10k divider)
//   GPIO21    — I2C SDA (ENS160+AHT2x)
//   GPIO22    — I2C SCL (ENS160+AHT2x)
//   GPIO13    — GPS UART1 RX ← ATGM336H TX  [moved from 22 to free I2C SCL]
//   GPIO23    — GPS UART1 TX → ATGM336H RX
//   GPIO16    — IMU UART2 RX ← JY-901 TX
//   GPIO17    — IMU UART2 TX → JY-901 RX
//   GPIO25    — INMP441 I2S SCK (BCLK)
//   GPIO26    — INMP441 I2S WS  (LRCLK)  [ADC2 output only — no WiFi ADC conflict]
//   GPIO33    — INMP441 I2S SD  (data in) [freed from NO2 analogue]
//   GPIO4     — CO PWM (LEDC, direct to D2 on heater board)
//   GPIO18    — Dust IR LED (active LOW)
//   GPIO14    — LED green   (CO ≤ 10 ppm)
//   GPIO27    — LED orange  (CO 10–20 ppm)
//   GPIO15    — LED red     (CO > 20 ppm) [GPIO26 freed for I2S WS]
//   GPIO19    — Buzzer
//
// ─── VOLTAGE DIVIDERS v2.0 ───────────────────────────────────────────────────
//   Only 3 remain (all new sensors are 3.3V native):
//   MQ-7  A0 → 10kΩ + 10kΩ → GPIO36    Vout@5V = 2.5V ✓
//   MQ-7  A1 → 10kΩ + 10kΩ → GPIO39    Vout@5V = 2.5V ✓ (heater feedback)
//   GP2Y  Vo → 10kΩ + 10kΩ → GPIO32    Vout@5V = 2.5V ✓
//
// ─── BLYNK VIRTUAL PINS ──────────────────────────────────────────────────────
//   FAST (5s):  V2  CO ppm      V6  CO phase    V7  CO raw
//               V12 GPS lat     V13 GPS lng      V14 GPS sats
//               V15 Humidity    V16 Temperature  V17 HDOP
//               V19 Eng msg     V20 Status flags V21 RSSI dBm
//               V22 WiFi qual%
//   SLOW (10s): V1  Sound dB    V3  UVI          V4  Diagnostic
//               V5  TVOC ppb    V8  Roll         V9  Pitch
//               V10 Yaw         V11 IMU temp     V18 Dust mg/m³
//               V23 eCO2 ppm    V24 AQI          V25 Heater V (A1 feedback)
//
// ─── ARCHITECTURE ────────────────────────────────────────────────────────────
//   Core 1 (app_cpu) — Arduino loop(): WiFi, Blynk.run(), BlynkTimer
//   Core 0 (pro_cpu) — sensorTask:     all sensors, CO state machine
//   Shared SensorData struct protected by FreeRTOS mutex.
//   Blynk.connect(timeout) NOT used — avoids IWDT crash on Core 1.
//
// ─── v2.1 CHANGES ────────────────────────────────────────────────────────────
//   • Replaced Adafruit_ENS160 with ScioSense_ENS160 library (correct library
//     for this sensor module). API calls updated throughout.
//   • Fixed "ADC: CONFLICT driver_ng" error:
//     Root cause: analogSetAttenuation() in Arduino core 3.x internally calls
//     the new adc_oneshot (driver_ng) API. The legacy I2S driver installs its
//     own ADC handle via the legacy ADC API. Both cannot coexist — firmware
//     aborts with "CONFLICT! driver_ng is not allowed to be used with the
//     legacy driver".
//     Fix: replaced analogSetAttenuation() with analogSetPinAttenuation()
//     called individually for each ADC pin. Per-pin calls do NOT invoke
//     driver_ng and are safe alongside the legacy I2S driver.
//     I2S (initINMP441) is also installed LAST in setup(), after all ADC
//     pin configuration is complete.
//
// ─── v2.2 CHANGES — DEFINITIVE FIX FOR ADC/I2S COLLISION ───────────────────
//   The v2.1 fix above (per-pin attenuation + install-order) reduced the
//   frequency of the "ADC: CONFLICT driver_ng is not allowed to be used
//   with the legacy driver" abort but did not eliminate it, because the
//   TRUE root cause is not ordering — it is that <driver/i2s.h> (legacy
//   I2S driver) and analogRead()/analogSetPinAttenuation() (which use the
//   new driver_ng ADC API on ESP32 Arduino core 3.x) belong to two
//   different, mutually-exclusive ESP-IDF driver families. Once the
//   legacy I2S driver is installed, ANY subsequent driver_ng ADC call
//   (including calls made later inside sensorTask, e.g. every CO/UV/dust
//   analogRead()) can abort — no init ordering fixes this permanently.
//
//   HARMONIZED FIX: migrated the INMP441 microphone driver from the legacy
//   <driver/i2s.h> API (i2s_driver_install / i2s_set_pin / i2s_read) to the
//   new standard driver <driver/i2s_std.h> API (i2s_new_channel /
//   i2s_channel_init_std_mode / i2s_channel_enable / i2s_channel_read).
//   The new i2s_std driver is part of the same "new driver" family as
//   driver_ng ADC and never touches the ADC peripheral or its driver
//   state at all — I2S and ADC are fully independent hardware blocks in
//   this API family, so no conflict is possible in either init order or
//   at runtime. This is the same class of fix as replacing legacy ADC
//   calls with driver_ng equivalents: harmonizing both libraries onto the
//   same modern driver generation removes the collision at its source
//   instead of merely avoiding it by sequencing.
//
//   All CO/UV/dust ADC logic, EMA smoothing, closed-loop heater control,
//   GPS/IMU/ENS160/AHT2x logic, Blynk architecture, and pin map are
//   UNCHANGED from v2.1 — only the internal implementation of
//   initINMP441() and readINMP441_dB() changed to use the new driver.
//   The public behaviour (function signatures, return values, dB output
//   range, SENSOR_UNAVAILABLE sentinel) is identical.
// ============================================================================

#define BLYNK_HEARTBEAT 60

#include "secrets.h"
// secrets.h: #define WIFI_SSID / WIFI_PASS / BLYNK_AUTH / BLYNK_SERVER / BLYNK_PORT
#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <esp_mac.h>
#include <TinyGPS++.h>
#include <Wire.h>
// ENS160: use ScioSense_ENS160 library (NOT Adafruit_ENS160 which does not exist)
// Install: https://github.com/sciosense/ENS160_driver  or via Library Manager
#include "ScioSense_ENS160.h"
#include <Adafruit_AHTX0.h>      // AHT20/AHT21 temp+hum — install: Adafruit AHTX0
#include <Adafruit_SSD1306.h>
#include <driver/i2s_std.h>      // ESP32 NEW standard I2S driver for INMP441 (v2.2)
// v2.1 previously used the legacy <driver/i2s.h> here. That header put the
// firmware at risk of the "ADC: CONFLICT driver_ng" abort because the legacy
// I2S driver and the driver_ng ADC API used by analogRead() cannot safely
// coexist. <driver/i2s_std.h> is the new-generation I2S driver family and
// does not touch the ADC driver at all — see v2.2 CHANGES note above.
// NOTE: Use legacy <driver/i2s.h> not the new ESP-IDF 5.x i2s_std.h —
// the legacy driver is what the Arduino ESP32 core 3.x still exposes via
// the compatibility shim. To avoid the "ADC: CONFLICT driver_ng" abort:
//   1. Call analogReadResolution() + analogSetPinAttenuation() per pin in setup() FIRST.
//   2. Do NOT call analogSetAttenuation() (global) — it uses driver_ng internally.
//   3. Call i2s_driver_install() (initINMP441) LAST in setup(), after all ADC config.
// ^ Superseded by v2.2: the above three-step workaround is no longer required
//   now that INMP441 uses <driver/i2s_std.h>, which never touches the ADC.
//   Kept here for historical context only — the current initINMP441() below
//   uses the new driver and can be initialised in ANY order relative to ADC.

// ─── FEATURE SWITCHES ────────────────────────────────────────────────────────
#define DEBUGON        false
#define DISPLAYON      false
#define WIFI           true
#define COsensorThere  true
#define IMUsensorThere true
#define ENSsensorThere true    // ENS160 + AHT2x combo board
#define INMPsensorThere true   // INMP441 I2S microphone

bool ten_mins_autoreset = false;

#define SENSOR_UNAVAILABLE (-999.0f)

volatile bool setupDone = false;

// ─── VOLTAGE DIVIDER  10kΩ + 10kΩ → Vout@5V = 2.5V ─────────────────────────
const float R_DIVIDER_SERIES = 10.0f;
const float R_DIVIDER_GND    = 10.0f;
float divider_scale = 2.0f;
float divider_ratio = 0.5f;

// ─── OLED ────────────────────────────────────────────────────────────────────
#define OLED_RESET -1
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

// ─── GPS — ATGM336H on UART1 GPIO13(RX)/GPIO23(TX) ──────────────────────────
// ATGM336H outputs standard NMEA0183 at 9600 baud — no UBX init needed.
// GPIO13 chosen to free GPIO22 for I2C SCL (ENS160+AHT2x).
#define GPS_RX_PIN    13
#define GPS_TX_PIN    23
#define GPS_BAUD      9600
HardwareSerial gps_serial(1);
TinyGPSPlus    gps;

// ─── IMU — JY-901 UART2 GPIO16(RX)/GPIO17(TX) ───────────────────────────────
#define IMU_RX_PIN 16
#define IMU_TX_PIN 17
HardwareSerial imu_serial(2);

// ─── ENS160 + AHT2x — I2C GPIO21(SDA)/GPIO22(SCL) ──────────────────────────
// ScioSense_ENS160 API:
//   ens160.begin()           — init, returns void
//   ens160.available()       — returns bool (true if responding)
//   ens160.setMode(ENS160_OPMODE_STD) — set standard operating mode
//   ens160.set_envdata(tempC_int, hum_int) — compensation (int values)
//   ens160.measure(true)     — trigger measurement
//   ens160.measureRaw(true)  — trigger raw measurement
//   ens160.getAQI()          — uint8_t  1–5 index
//   ens160.getTVOC()         — uint16_t ppb
//   ens160.geteCO2()         — uint16_t ppm equivalent CO2
ScioSense_ENS160 ens160(ENS160_I2CADDR_1);  // 0x53 (ADD pin → VCC on this board)
Adafruit_AHTX0   aht;

// ─── INMP441 I2S microphone ──────────────────────────────────────────────────
#define I2S_PORT        I2S_NUM_0
#define I2S_SCK_PIN     25   // BCLK
#define I2S_WS_PIN      26   // LRCLK  [ADC2 output only — no WiFi ADC conflict]
#define I2S_SD_PIN      33   // SD data in [freed from NO2 analogue]
#define I2S_SAMPLE_RATE 16000
#define I2S_BUF_SAMPLES 256
// v2.2: RX channel handle for the new i2s_std driver. Replaces the old
// legacy-driver pattern of addressing the peripheral by I2S_PORT number
// alone — the new driver family uses an explicit channel handle instead.
i2s_chan_handle_t inmp441_rx_handle = NULL;

// ─── ANALOG PINS (ADC1 only) ─────────────────────────────────────────────────
// All 5V sensor outputs require 10kΩ + 10kΩ voltage divider before ESP32 ADC.
// ML8511 UV is 3.3V ratiometric — direct connection, no divider.
#define CO_ADC_PIN    36    // VP ADC1_CH0 input-only — MQ-7 A0 (CO sense, via divider)
#define CO_A1_PIN     39    // VN ADC1_CH3 input-only — MQ-7 A1 (heater feedback, via divider)
#define UV_OUT_PIN    34    //    ADC1_CH6 input-only — ML8511 OUT (direct 3.3V)
#define UV_REF_PIN    35    //    ADC1_CH7 input-only — ML8511 REF (direct 3.3V)
#define DUST_ADC_PIN  32    //    ADC1_CH4 — GP2Y1010 Vo (via 10k+10k divider)

// ─── DIGITAL PINS ────────────────────────────────────────────────────────────
#define ledPin11   14    // GREEN  CO ≤ 10 ppm
#define ledPin12   27    // ORANGE CO 10–20 ppm  (ADC2, output only — no WiFi conflict)
#define ledPin13   15    // RED    CO > 20 ppm   (GPIO26 freed for I2S WS)
#define buzzPin    19    // Passive buzzer
#define dustLED    18    // Sharp GP2Y1010 IR LED (active LOW)
#define CO_PWM_PIN  4    // CO heater D2 PWM (direct to NPN base via 1kΩ on board)

// ─── LEDC ────────────────────────────────────────────────────────────────────
#define LEDC_FREQ_CO 5000
#define LEDC_RES_CO  8

// ─── TIMEOUTS & INTERVALS ────────────────────────────────────────────────────
#define GPS_TIMEOUT_MS        15000
#define IMU_TIMEOUT_MS         5000
#define ENS_MAX_FAILS             5
#define ENS_RETRY_MS          10000
#define CO_PHASE_MAX_MS      180000
#define WIFI_RECONNECT_MS     30000
#define ENG_MSG_INTERVAL      15000
#define BLYNK_SEND_FAST_MS     5000
#define BLYNK_SEND_SLOW_MS    10000
#define SENSOR_TASK_PERIOD_MS     25
#define GPS_FEED_MS               50
#define IMU_POLL_MS               50
#define BUZZER_GPS_INTERVAL     5000
#define VWRITE_GAP_MS            10

// ─── CO CLOSED-LOOP HEATER TARGETS ───────────────────────────────────────────
// A1 feedback divider scales 5V→2.5V at ADC. We target the ADC-side voltage.
// Physical heater voltages: heat=5.0V, measure=1.4V
// At ADC after 10k+10k divider: heat=2.5V, measure=0.7V
#define CO_HEAT_TARGET_V   2.50f   // ADC target during heating phase (=5V at sensor)
#define CO_MEAS_TARGET_V   0.70f   // ADC target during measurement phase (=1.4V at sensor)
#define CO_CLOSED_LOOP_TOL 0.05f   // ±tolerance in V at ADC before adjusting duty

// ─── STATUS FLAGS ────────────────────────────────────────────────────────────
#define STATUS_WIFI_OK   (1<<0)
#define STATUS_GPS_OK    (1<<1)
#define STATUS_IMU_OK    (1<<2)
#define STATUS_ENS_OK    (1<<3)    // was DHT_OK, now ENS160+AHT2x
#define STATUS_CO_FAULT  (1<<4)
#define STATUS_GPS_FAULT (1<<5)
#define STATUS_IMU_STUCK (1<<6)
#define STATUS_ENS_FAULT (1<<7)    // was DHT_FAULT
#define STATUS_CO_NO_SNS (1<<8)

// ─── CO CALIBRATION ──────────────────────────────────────────────────────────
// sensor_reading_clean_air: raw 12-bit ADC at GPIO36 (after divider) in fresh air
float reference_resistor_kOhm   = 9.98f;
float sensor_reading_clean_air  = 600.65f;
float sensor_reading_100_ppm_CO = -1.0f;

// ─── SHARED DATA STRUCT ──────────────────────────────────────────────────────
struct SensorData {
  float    co_ppm, co_raw, co_heater_v; // heater_v = actual A1 voltage (×divider_scale)
  byte     co_phase;
  bool     co_fault, co_no_sensor;
  float    tvoc_ppb, eco2_ppm, aqi;     // ENS160
  float    uvi;
  float    sound_db;                    // INMP441 dB SPL estimate
  float    dust_mg;
  float    temp, hum;                   // AHT2x
  bool     ens_fault;
  float    angle[3], imu_temp;
  bool     imu_ok;
  double   lat, lng;
  float    hdop, sats;
  bool     gps_fix;
  int32_t  rssi;
  uint8_t  wifi_qual;
  uint32_t status_flags;
  char     eng_msg[128];
  int      rand_num;
};

SensorData        sd;
SemaphoreHandle_t dataMutex;

// ─── CO STATE ────────────────────────────────────────────────────────────────
byte          co_phase          = 0;    // 0=measure, 1=heat
unsigned long co_phase_start    = 0;
float         sens_val          = 0;   // EMA of CO_ADC_PIN
float         sens_val_last     = 0;
float         last_CO_ppm       = 0;
byte          co_duty           = 255; // closed-loop duty output
float         sensor_base_resistance_kOhm;
float         sensor_100ppm_CO_resistance_kOhm;

// ─── IMU ─────────────────────────────────────────────────────────────────────
unsigned char Re_buf[12];
int           imu_counter  = 0;
unsigned long lastIMUframe = 0;
float         angle[3]     = {0,0,0};
float         imuT         = 0;
const float   tempComp     = -7.2f;

// ─── ENS160 / AHT2x state ────────────────────────────────────────────────────
int           ens_fail_count = 0;
unsigned long lastENSattempt = 0;
bool          ensReady       = false;  // true after first successful init
float         hum_prev = 20.0f, temp_prev = 20.0f;

// ─── MISC STATE ──────────────────────────────────────────────────────────────
unsigned long lastGPSdata   = 0;
unsigned long lastGPSbeep   = 0;
unsigned long lastWiFiCheck = 0;
long          countReset    = 0;
bool          prevZeroVal   = false;
float         SatGPS = 0, HDOP = 0;

BlynkTimer blynkTimer;

// ============================================================================
// HELPERS
// ============================================================================

int averageAnalogRead(int pin)
{
  unsigned long sum = 0;
  for (byte i = 0; i < 9; i++) { sum += analogRead(pin); delayMicroseconds(100); }
  return (int)(sum / 9);
}

float adcToSensorVoltage(int counts)
{
  // Convert post-divider ADC counts to actual sensor voltage (multiplied by divider_scale)
  return counts * (3.3f / 4095.0f) * divider_scale;
}

// Convert raw ADC counts to actual voltage AT the ADC pin (no scale, just V)
float adcToVolts(int counts)
{
  return counts * (3.3f / 4095.0f);
}

uint8_t rssiToQuality(int32_t rssi)
{
  if (rssi >= -50)  return 100;
  if (rssi <= -100) return 0;
  return (uint8_t)(2 * (rssi + 100));
}

void engMsg(const char* msg)
{
  Serial.print("[ENG] "); Serial.println(msg);
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    strncpy(sd.eng_msg, msg, sizeof(sd.eng_msg)-1);
    sd.eng_msg[sizeof(sd.eng_msg)-1] = '\0';
    xSemaphoreGive(dataMutex);
  }
}

void engMsgf(const char* fmt, ...)
{
  char buf[128];
  va_list args; va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  engMsg(buf);
}

void buzzerTone(uint16_t frequency, uint16_t durationMs)
{
  if (!frequency) return;
  ledcAttach(buzzPin, frequency, 8);
  ledcWrite(buzzPin, 128);
  vTaskDelay(pdMS_TO_TICKS(durationMs));
  ledcWrite(buzzPin, 0);
  ledcDetach(buzzPin);
}

void safeWrite(int vpin, float val)        { Blynk.virtualWrite(vpin, val); delay(VWRITE_GAP_MS); }
void safeWriteI(int vpin, int val)         { Blynk.virtualWrite(vpin, val); delay(VWRITE_GAP_MS); }
void safeWriteS(int vpin, const char* val) { Blynk.virtualWrite(vpin, val); delay(VWRITE_GAP_MS); }

// ============================================================================
// GPS — ATGM336H (NMEA only, no UBX init)
// ============================================================================

void initGPS()
{
  // ATGM336H speaks standard NMEA0183 at 9600 baud out of the box.
  // No proprietary init sequence needed — TinyGPS++ decodes NMEA directly.
  gps_serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(100);
  engMsg("GPS: ATGM336H NMEA 9600 baud — no init needed");
}

void feedGPS()
{
  unsigned long t = millis();
  while (millis() - t < GPS_FEED_MS) {
    while (gps_serial.available()) gps.encode(gps_serial.read());
    taskYIELD();
  }
}

// ============================================================================
// ENS160 + AHT2x (I2C) — ScioSense_ENS160 library
// ============================================================================

bool initENS()
{
  // ScioSense API: ens160.begin() returns void; check ens160.available() after.
  ens160.begin();
  if (!ens160.available()) {
    engMsg("ENS160: not available (addr 0x53) — check wiring");
    return false;
  }
  // Set standard operating mode (continuous measurement)
  if (!ens160.setMode(ENS160_OPMODE_STD)) {
    engMsg("ENS160: setMode failed");
    return false;
  }
  // AHT2x init
  if (!aht.begin()) {
    engMsg("AHT2x: init failed — check wiring");
    return false;
  }
  engMsg("ENS160+AHT2x: OK");
  return true;
}

// Read ENS160 + AHT2x. Returns true if valid data obtained.
// ENS160 needs temperature+humidity compensation for accuracy — we feed it
// from AHT2x which shares the same I2C bus on the same board.
// ScioSense API uses integer temp (°C) and humidity (%) for set_envdata().
bool readENS(float& tvoc, float& eco2, float& aqi_out, float& temp_out, float& hum_out)
{
  // Read AHT2x first — provides compensation values for ENS160
  sensors_event_t humEvent, tempEvent;
  if (!aht.getEvent(&humEvent, &tempEvent)) return false;
  temp_out = tempEvent.temperature;
  hum_out  = humEvent.relative_humidity;

  // Provide integer temp/humidity compensation to ENS160 for improved accuracy
  // ScioSense set_envdata() takes int (°C) and int (%) — cast from float
  ens160.set_envdata((int)temp_out, (int)hum_out);

  // Trigger measurement
  ens160.measure(true);
  ens160.measureRaw(true);

  // Read results — always available after measure()
  tvoc    = (float)ens160.getTVOC();   // ppb
  eco2    = (float)ens160.geteCO2();   // ppm equivalent CO2
  aqi_out = (float)ens160.getAQI();    // 1–5 index

  return true;
}

// ============================================================================
// INMP441 I2S microphone — legacy driver/i2s.h
// IMPORTANT: ADC must be fully configured (analogReadResolution +
// analogSetAttenuation) in setup() BEFORE i2s_driver_install() is called.
// Calling ADC config AFTER I2S install triggers:
//   E (321) ADC: CONFLICT! driver_ng is not allowed to be used with the legacy driver
//
// v2.2 UPDATE: the above ordering requirement applied to the legacy
// <driver/i2s.h> API and is now HISTORICAL — initINMP441() below has been
// migrated to the new <driver/i2s_std.h> driver, which does not use or
// conflict with the ADC driver at all. The functions can now be called in
// any order relative to ADC setup. The old comment is kept for context.
// ============================================================================

bool initINMP441()
{
  // v2.2: new i2s_std driver — replaces legacy i2s_driver_install/i2s_set_pin.
  // Step 1: allocate an RX channel on I2S_PORT (I2S_NUM_0).
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG((i2s_port_t)I2S_PORT, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num  = 4;              // was dma_buf_count = 4 in legacy config
  chan_cfg.dma_frame_num = I2S_BUF_SAMPLES; // was dma_buf_len = I2S_BUF_SAMPLES in legacy config
  if (i2s_new_channel(&chan_cfg, NULL, &inmp441_rx_handle) != ESP_OK) {
    engMsg("INMP441: new_channel failed");
    return false;
  }

  // Step 2: configure standard I2S mode — sample rate, slot width, and pins.
  // channel_format = I2S_CHANNEL_FMT_ONLY_LEFT (legacy) becomes slot_mask =
  // I2S_STD_SLOT_LEFT (new) — L/R pin tied to GND still selects left channel.
  i2s_std_config_t std_cfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_SCK_PIN,
      .ws   = (gpio_num_t)I2S_WS_PIN,
      .dout = I2S_GPIO_UNUSED,
      .din  = (gpio_num_t)I2S_SD_PIN,
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false }
    }
  };
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;  // L/R tied to GND = left channel

  if (i2s_channel_init_std_mode(inmp441_rx_handle, &std_cfg) != ESP_OK) {
    engMsg("INMP441: init_std_mode failed");
    return false;
  }

  // Step 3: enable the channel — equivalent to the old driver being ready
  // for i2s_read() calls immediately after i2s_driver_install()+i2s_set_pin().
  if (i2s_channel_enable(inmp441_rx_handle) != ESP_OK) {
    engMsg("INMP441: channel_enable failed");
    return false;
  }

  engMsg("INMP441: I2S mic OK (new i2s_std driver, no ADC conflict)");
  return true;
}

// Read one buffer from INMP441, return RMS level converted to approximate dBFS.
// INMP441 is 24-bit left-justified in 32-bit slot — shift right 8 to get 24-bit signed.
float readINMP441_dB()
{
  int32_t samples[I2S_BUF_SAMPLES];
  size_t  bytes_read = 0;
  // v2.2: i2s_channel_read() replaces legacy i2s_read(I2S_PORT, ...).
  // Same blocking-with-timeout semantics as the legacy call.
  i2s_channel_read(inmp441_rx_handle, &samples, sizeof(samples), &bytes_read, pdMS_TO_TICKS(50));
  int count = bytes_read / sizeof(int32_t);
  if (count == 0) return SENSOR_UNAVAILABLE;

  double sum_sq = 0;
  for (int i = 0; i < count; i++) {
    int32_t s = samples[i] >> 8;   // 32-bit slot → 24-bit signed value
    sum_sq += (double)s * (double)s;
  }
  double rms = sqrt(sum_sq / count);
  if (rms < 1.0) return SENSOR_UNAVAILABLE;
  // dBFS relative to full-scale 24-bit (2^23 = 8388608)
  float db = 20.0f * log10f((float)(rms / 8388608.0));
  return db;   // typically -60 to 0 dBFS
}

// ============================================================================
// CO SENSOR — CLOSED LOOP heater control using A1 feedback
// ============================================================================

void setHeatDuty(byte duty)
{
  co_duty = duty;
  ledcWrite(CO_PWM_PIN, duty);
}

void startMeasurementPhase()
{
  co_phase = 0; co_phase_start = millis();
  // Start at initial duty guess for 1.4V; closed loop in tickCO() will refine
  setHeatDuty(64);  // ~25% duty ≈ reasonable starting point for 1.4V target
  engMsg("CO: measure phase — closed-loop targeting 1.4V heater");
}

void startHeatingPhase()
{
  co_phase = 1; co_phase_start = millis();
  setHeatDuty(255);  // Full duty = full 5V heater
  engMsg("CO: heat phase (duty=255 → 5V heater)");
}

bool coSensorPresent()
{
  // GPIO36 (VP) floats near 0 when nothing is connected.
  // With MQ-7 divider circuit, it reads well above noise floor.
  long sum = 0;
  for (int i = 0; i < 20; i++) { sum += analogRead(CO_ADC_PIN); delay(5); }
  return ((sum / 20) > 50);
}

float raw_to_CO_ppm(float adc_ema)
{
  if (adc_ema < 1.0f) return SENSOR_UNAVAILABLE;
  float adc_eq   = constrain(adc_ema * divider_scale, 1.0f, 4095.0f);
  float clean_eq = constrain(sensor_reading_clean_air * divider_scale, 1.0f, 4095.0f);
  sensor_base_resistance_kOhm =
    reference_resistor_kOhm * 4095.0f / clean_eq - reference_resistor_kOhm;
  if (sensor_reading_100_ppm_CO > 0.0f) {
    float c = constrain(sensor_reading_100_ppm_CO * divider_scale, 1.0f, 4095.0f);
    sensor_100ppm_CO_resistance_kOhm =
      reference_resistor_kOhm * 4095.0f / c - reference_resistor_kOhm;
  } else {
    sensor_100ppm_CO_resistance_kOhm = sensor_base_resistance_kOhm * 0.5f;
  }
  if (sensor_base_resistance_kOhm <= 0.0f || sensor_100ppm_CO_resistance_kOhm <= 0.0f)
    return SENSOR_UNAVAILABLE;
  float sensor_R = reference_resistor_kOhm * 4095.0f / adc_eq - reference_resistor_kOhm;
  if (sensor_R <= 0.0f) return 0.0f;
  float ppm = 100.0f * (expf(sensor_100ppm_CO_resistance_kOhm / sensor_R) - 1.648f);
  return (ppm < 0.0f) ? 0.0f : ppm;
}

void tickCO()
{
  unsigned long elapsed = millis() - co_phase_start;

  // Read A1 heater feedback (via divider, so multiply by scale to get actual V)
  float a1_adc_v = adcToVolts(averageAnalogRead(CO_A1_PIN));  // voltage at ADC pin
  float heater_v = a1_adc_v * divider_scale;                  // actual heater voltage

  // Closed-loop duty adjustment toward target
  // Target is in ADC-pin volts (after divider).
  // Simple proportional: ±1 duty step per tick when outside tolerance band.
  float target = (co_phase == 1) ? CO_HEAT_TARGET_V : CO_MEAS_TARGET_V;
  float error  = target - a1_adc_v;
  if (fabsf(error) > CO_CLOSED_LOOP_TOL) {
    int new_duty = (int)co_duty + (error > 0 ? 1 : -1);
    new_duty = constrain(new_duty, 0, 255);
    setHeatDuty((byte)new_duty);
  }

  // Watchdog — phase stuck beyond CO_PHASE_MAX_MS
  if (elapsed > CO_PHASE_MAX_MS) {
    engMsgf("CO WATCHDOG: phase %d stuck >%lus heater=%.2fV",
            co_phase, CO_PHASE_MAX_MS/1000, heater_v);
    (co_phase == 1) ? startMeasurementPhase() : startHeatingPhase();
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      sd.co_fault = true; sd.status_flags |= STATUS_CO_FAULT;
      xSemaphoreGive(dataMutex);
    }
    return;
  }

  // Phase transitions
  if (co_phase == 1 && elapsed > 60000UL) {
    startMeasurementPhase();
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      sd.co_fault = false; sd.status_flags &= ~STATUS_CO_FAULT;
      xSemaphoreGive(dataMutex);
    }
    return;
  }
  if (co_phase == 0 && elapsed > 90000UL) {
    float ppm = raw_to_CO_ppm(sens_val);
    if (ppm >= 0.0f) last_CO_ppm = ppm;
    sens_val_last = sens_val;
    engMsgf("CO: cycle done ppm=%.1f raw=%.0f heater=%.2fV duty=%d",
            last_CO_ppm, sens_val, heater_v, co_duty);
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      sd.co_ppm = last_CO_ppm; sd.co_raw = sens_val; sd.co_phase = co_phase;
      sd.co_heater_v = heater_v;
      sd.co_fault = (ppm == SENSOR_UNAVAILABLE);
      if (ppm == SENSOR_UNAVAILABLE) sd.status_flags |= STATUS_CO_FAULT;
      else                           sd.status_flags &= ~STATUS_CO_FAULT;
      xSemaphoreGive(dataMutex);
    }
    startHeatingPhase();
    return;
  }

  // Continuous CO sense ADC read + exponential moving average (α=0.3)
  float v = (float)analogRead(CO_ADC_PIN);
  float maxADC = 4095.0f * divider_ratio;
  if (v >= 10.0f && v <= maxADC * 1.05f)
    sens_val = 0.7f * sens_val + 0.3f * v;

  // Update shared struct — brief mutex hold
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    sd.co_raw = sens_val; sd.co_phase = co_phase;
    sd.co_heater_v = heater_v;
    xSemaphoreGive(dataMutex);
  }

  if (DEBUGON)
    Serial.printf("CO phase=%d t=%lus ema=%.0f ppm=%.1f a1_v=%.3f heater_v=%.3f duty=%d\n",
                  co_phase, elapsed/1000, sens_val, last_CO_ppm, a1_adc_v, heater_v, co_duty);
}

// ============================================================================
// IMU — JY-901 (unchanged 0x55 framed protocol)
// ============================================================================

bool pollIMU()
{
  while (imu_serial.available()) {
    unsigned char b = (unsigned char)imu_serial.read();
    if (imu_counter == 0 && b != 0x55) continue;
    Re_buf[imu_counter++] = b;
    if (imu_counter == 11) {
      imu_counter = 0;
      if (Re_buf[0]==0x55 && Re_buf[1]==0x53) {
        angle[0] = (short((Re_buf[3]<<8)|Re_buf[2])) / 32768.0f * 180.0f;
        angle[1] = (short((Re_buf[5]<<8)|Re_buf[4])) / 32768.0f * 180.0f;
        angle[2] = (short((Re_buf[7]<<8)|Re_buf[6])) / 32768.0f * 180.0f;
        imuT = ((short((Re_buf[9]<<8)|Re_buf[8])) / 340.0f + 36.25f) + tempComp;
        lastIMUframe = millis();
        return true;
      }
    }
  }
  return false;
}

void readIMU()
{
  unsigned long start = millis();
  bool got = false;
  while (millis() - start < IMU_POLL_MS) { if (pollIMU()) { got=true; break; } taskYIELD(); }
  bool stuck = (lastIMUframe > 0 && millis()-lastIMUframe > IMU_TIMEOUT_MS);
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    if (got) {
      sd.angle[0]=angle[0]; sd.angle[1]=angle[1]; sd.angle[2]=angle[2];
      sd.imu_temp=imuT; sd.imu_ok=true;
      sd.status_flags |= STATUS_IMU_OK;
      sd.status_flags &= ~STATUS_IMU_STUCK;
    }
    if (stuck) {
      sd.angle[0]=sd.angle[1]=sd.angle[2]=sd.imu_temp=SENSOR_UNAVAILABLE;
      sd.imu_ok=false;
      sd.status_flags &= ~STATUS_IMU_OK;
      sd.status_flags |= STATUS_IMU_STUCK;
    }
    xSemaphoreGive(dataMutex);
  }
  if (stuck) engMsgf("IMU STUCK: no frame for %lus", (millis()-lastIMUframe)/1000);
}

// ============================================================================
// SENSOR TASK — Core 0 (pro_cpu)
// All blocking sensor I/O lives here, away from Blynk on Core 1.
// Uses vTaskDelay for all waits so FreeRTOS tick ISR is never starved.
// NOTE: ADC is NOT reconfigured here — it was already configured globally
// in setup() BEFORE I2S init. Reconfiguring ADC after I2S install causes
// the "ADC: CONFLICT driver_ng" error.
// v2.2: this ordering constraint applied to the legacy I2S driver. Since
// INMP441 now uses <driver/i2s_std.h> (see initINMP441()), the constraint
// no longer applies — analogRead() calls in this task cannot collide with
// I2S regardless of when initINMP441() was called in setup(). The original
// ordering is kept anyway as good practice, not because it's still required.
// ============================================================================

void sensorTask(void* pvParam)
{
  while (!setupDone) vTaskDelay(pdMS_TO_TICKS(10));

  // ADC config already done in setup() — do NOT call analogReadResolution()
  // or analogSetAttenuation() here. That would trigger the driver conflict.

  while (true)
  {
    unsigned long loopStart = millis();

    // ── UV (ML8511 3.3V ratiometric, direct — no divider) ───────────────────
    int uvRaw = averageAnalogRead(UV_OUT_PIN);
    int uvRef = averageAnalogRead(UV_REF_PIN);
    float uvi = SENSOR_UNAVAILABLE;
    if (uvRef > 50) {
      float outV = (3.3f / (float)uvRef) * (float)uvRaw;
      uvi = constrain(12.49f*(outV+0.03f)-12.49f+0.3f, 0.0f, 20.0f);
    }

    // ── Dust (GP2Y1010 5V, 10k+10k divider) ─────────────────────────────────
    // GPIO18 drives IR LED at 3.3V — safe direct connect to sensor ILED pin
    digitalWrite(dustLED, LOW);
    delayMicroseconds(280);
    int voMeas = analogRead(DUST_ADC_PIN);
    delayMicroseconds(40);
    digitalWrite(dustLED, HIGH);
    float dustDens = max(0.0f, 0.17f * adcToSensorVoltage(voMeas) - 0.1f);

    // ── ENS160 + AHT2x (I2C, ScioSense library) ─────────────────────────────
    float tvoc=SENSOR_UNAVAILABLE, eco2=SENSOR_UNAVAILABLE, aqi=SENSOR_UNAVAILABLE;
    float t=temp_prev, h=hum_prev;
    bool ensOK = false;
    bool ensFaulted = (ens_fail_count >= ENS_MAX_FAILS);
    if (!ensFaulted || (millis()-lastENSattempt > ENS_RETRY_MS)) {
      lastENSattempt = millis();
      if (!ensReady) {
        // Attempt (re)init — retried from sensorTask in case board was absent at boot
        ensReady = initENS();
      }
      if (ensReady) {
        ensOK = readENS(tvoc, eco2, aqi, t, h);
        if (ensOK) { temp_prev=t; hum_prev=h; ens_fail_count=0; }
        else        { ens_fail_count++; t=temp_prev; h=hum_prev; }
      } else { ens_fail_count++; }
      if (ens_fail_count == ENS_MAX_FAILS)
        engMsgf("ENS FAULT: %d failures — retrying every %ds", ENS_MAX_FAILS, ENS_RETRY_MS/1000);
    }
    if (ensFaulted && !ensOK) { t=SENSOR_UNAVAILABLE; h=SENSOR_UNAVAILABLE; }

    // ── INMP441 I2S mic ───────────────────────────────────────────────────────
    float soundDb = INMPsensorThere ? readINMP441_dB() : SENSOR_UNAVAILABLE;

    // ── CO state machine ─────────────────────────────────────────────────────
    if (COsensorThere) tickCO();

    // ── GPS (ATGM336H NMEA, 9600 baud) ───────────────────────────────────────
    feedGPS();
    SatGPS = (float)gps.satellites.value();
    HDOP   = (SatGPS >= 1) ? gps.hdop.value()/100.0f : 99.9f;
    bool gpsFix = (gps.location.isValid() &&
                   gps.location.age() < GPS_TIMEOUT_MS && SatGPS >= 4);
    double lat=0, lng=0;
    if (gpsFix) {
      lat=gps.location.lat(); lng=gps.location.lng();
      lastGPSdata=millis(); prevZeroVal=false;
    } else {
      if (!prevZeroVal) {
        lat=gps.location.lat()+0.0001; lng=gps.location.lng()+0.0001;
        prevZeroVal=true;
        engMsgf("GPS: fix lost (sats=%.0f age=%lums)", SatGPS, gps.location.age());
      } else { lat=0; lng=0; }
      if (millis()-lastGPSbeep > BUZZER_GPS_INTERVAL) {
        lastGPSbeep=millis(); buzzerTone(12000, 90);
      }
    }
    if (lastGPSdata>0 && millis()-lastGPSdata>GPS_TIMEOUT_MS)
      engMsgf("GPS FAULT: no data >%ds", GPS_TIMEOUT_MS/1000);

    // ── IMU — JY-901 UART2 ───────────────────────────────────────────────────
    if (IMUsensorThere) readIMU();
    else {
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        sd.angle[0]=sd.angle[1]=sd.angle[2]=sd.imu_temp=SENSOR_UNAVAILABLE;
        xSemaphoreGive(dataMutex);
      }
    }

    // ── LED control ──────────────────────────────────────────────────────────
    float cppm = last_CO_ppm;
    digitalWrite(ledPin11, (cppm>=0 && cppm<=10) ? HIGH : LOW);
    digitalWrite(ledPin12, (cppm>10 && cppm<=20) ? HIGH : LOW);
    digitalWrite(ledPin13, (cppm>20)             ? HIGH : LOW);

    // ── OLED ─────────────────────────────────────────────────────────────────
    if (DISPLAYON) {
      display.clearDisplay(); display.setCursor(0,0); display.setTextSize(1);
      display.printf("CO:%.1f UV:%.1f\n", cppm, uvi);
      display.printf("T:%.1fC H:%.0f%%\n", t, h);
      display.printf("TVOC:%.0fppb CO2:%.0f\n", tvoc, eco2);
      display.printf("%.6f\n%.6f\n", lat, lng);
      if (IMUsensorThere && sd.imu_ok)
        display.printf("R/P/Y:%.0f/%.0f/%.0f\n", angle[0],angle[1],angle[2]);
      display.display();
    }

    // ── Single mutex-protected struct write ──────────────────────────────────
    // Mutex held only here — never during I/O operations above.
    uint32_t flags = 0;
    if (ensOK)                                               flags |= STATUS_ENS_OK;
    if (ens_fail_count>=ENS_MAX_FAILS)                       flags |= STATUS_ENS_FAULT;
    if (gpsFix)                                              flags |= STATUS_GPS_OK;
    if (lastGPSdata>0&&millis()-lastGPSdata>GPS_TIMEOUT_MS) flags |= STATUS_GPS_FAULT;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      sd.uvi=uvi; sd.dust_mg=dustDens;
      sd.tvoc_ppb=tvoc; sd.eco2_ppm=eco2; sd.aqi=aqi;
      sd.sound_db=soundDb;
      sd.temp=t; sd.hum=h; sd.ens_fault=(ens_fail_count>=ENS_MAX_FAILS);
      sd.lat=lat; sd.lng=lng; sd.hdop=HDOP; sd.sats=SatGPS; sd.gps_fix=gpsFix;
      sd.rand_num=random(1,10);
      // Preserve WiFi/CO/IMU flags already set by their own functions; merge sensor flags
      sd.status_flags = (sd.status_flags &
        (STATUS_WIFI_OK|STATUS_IMU_OK|STATUS_IMU_STUCK|
         STATUS_CO_FAULT|STATUS_CO_NO_SNS)) | flags;
      xSemaphoreGive(dataMutex);
    }

    if (DEBUGON)
      Serial.printf("[SENS] CO=%.1f ph=%d UV=%.2f Snd=%.1fdB "
                    "Dst=%.3f T=%.1f H=%.0f TVOC=%.0f eCO2=%.0f AQI=%.0f "
                    "GPS=%d sat=%.0f\n",
                    last_CO_ppm,co_phase,uvi,soundDb,dustDens,
                    t,h,tvoc,eco2,aqi,(int)gpsFix,SatGPS);

    // ── 10-min optional auto-reset ────────────────────────────────────────────
    countReset++;
    if (ten_mins_autoreset && countReset >= 600) {
      engMsg("AUTO-RESET: 10-min watchdog");
      vTaskDelay(pdMS_TO_TICKS(300));
      buzzerTone(1000, 200);
      countReset=0; ESP.restart();
    }

    unsigned long elapsed = millis()-loopStart;
    vTaskDelay(pdMS_TO_TICKS(elapsed < SENSOR_TASK_PERIOD_MS
                              ? SENSOR_TASK_PERIOD_MS - elapsed : 1));
  }
}

// ============================================================================
// BLYNK SENDS — called from BlynkTimer in loop() (Core 1)
// ============================================================================

void blynkSendFast()
{
  // Critical / navigation data — sent every BLYNK_SEND_FAST_MS (5s)
  if (WiFi.status() != WL_CONNECTED) return;
  int32_t rssi = WiFi.RSSI();
  uint8_t qual = rssiToQuality(rssi);
  SensorData snap;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
    snap=sd; snap.rssi=rssi; snap.wifi_qual=qual;
    sd.rssi=rssi; sd.wifi_qual=qual;
    xSemaphoreGive(dataMutex);
  } else { engMsg("BLYNK FAST: mutex timeout"); return; }

  safeWrite (V2,  snap.co_ppm);
  safeWriteI(V6,  snap.co_phase);
  safeWrite (V7,  snap.co_raw);
  safeWrite (V12, snap.lat);
  safeWrite (V13, snap.lng);
  safeWrite (V14, snap.sats);
  safeWrite (V17, snap.hdop);
  safeWrite (V15, snap.hum);
  safeWrite (V16, snap.temp);
  safeWriteS(V19, snap.eng_msg);
  safeWriteI(V20, (int)snap.status_flags);
  safeWriteI(V21, snap.rssi);
  safeWriteI(V22, (int)snap.wifi_qual);
}

void blynkSendSlow()
{
  // Environmental / sensor data — sent every BLYNK_SEND_SLOW_MS (10s)
  if (WiFi.status() != WL_CONNECTED) return;
  SensorData snap;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
    snap=sd; xSemaphoreGive(dataMutex);
  } else { engMsg("BLYNK SLOW: mutex timeout"); return; }

  safeWrite (V1,  snap.sound_db);
  safeWrite (V3,  snap.uvi);
  safeWriteI(V4,  snap.rand_num);
  safeWrite (V5,  snap.tvoc_ppb);
  safeWrite (V8,  snap.angle[0]);
  safeWrite (V9,  snap.angle[1]);
  safeWrite (V10, snap.angle[2]);
  safeWrite (V11, snap.imu_temp);
  safeWrite (V18, snap.dust_mg);
  safeWrite (V23, snap.eco2_ppm);
  safeWrite (V24, snap.aqi);
  safeWrite (V25, snap.co_heater_v);
}

// ============================================================================
// WiFi CONNECT
// ============================================================================

bool wifiConnect()
{
  Serial.printf("[WiFi] Connecting to %s...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t = millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-t<20000) { delay(500); Serial.print("."); }
  if (WiFi.status()!=WL_CONNECTED) {
    Serial.println("\n[WiFi] FAILED");
    engMsg("WiFi FAILED: offline");
    buzzerTone(1000,600); delay(200); buzzerTone(1000,600); delay(200); buzzerTone(1000,600);
    if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(10))==pdTRUE) {
      sd.status_flags&=~STATUS_WIFI_OK; xSemaphoreGive(dataMutex);
    }
    return false;
  }
  Serial.printf("\n[WiFi] Connected — IP=%s RSSI=%ddBm MAC=%s\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI(), WiFi.macAddress().c_str());
  // Blynk.config() sets server address only — does NOT block or spin.
  // Blynk.run() in loop() completes the handshake on first call.
  // DO NOT use Blynk.connect(timeout) — it spins internally for 5s and causes IWDT crash.
  Blynk.config(BLYNK_AUTH, BLYNK_SERVER, BLYNK_PORT);
  if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(10))==pdTRUE) {
    sd.status_flags|=STATUS_WIFI_OK; xSemaphoreGive(dataMutex);
  }
  engMsgf("WiFi OK — Blynk configured %s:%d", BLYNK_SERVER, BLYNK_PORT);
  buzzerTone(5000, 100);
  return true;
}

// ============================================================================
// SETUP — runs on Core 1 (Arduino default app_cpu)
// ============================================================================

void setup()
{
  Serial.begin(115200);
  Serial.println("\n[INIT] Air Quality Station ESP32 v2.2");
  Serial.printf("[INIT] ten_mins_autoreset = %s\n", ten_mins_autoreset ? "ON":"OFF");

  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  Serial.printf("[INIT] WiFi MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

  divider_ratio = R_DIVIDER_GND / (R_DIVIDER_SERIES + R_DIVIDER_GND);
  divider_scale = (R_DIVIDER_SERIES + R_DIVIDER_GND) / R_DIVIDER_GND;
  Serial.printf("[INIT] Divider: %.0fkΩ+%.0fkΩ ratio=%.4f scale=%.4f Vout@5V=%.3fV\n",
                R_DIVIDER_SERIES,R_DIVIDER_GND,divider_ratio,divider_scale,5.0f*divider_ratio);

  dataMutex = xSemaphoreCreateMutex();
  memset(&sd, 0, sizeof(sd));
  strncpy(sd.eng_msg, "Booting...", sizeof(sd.eng_msg));
  // Pre-fill all sensor fields with UNAVAILABLE until first valid reads
  sd.co_ppm=sd.co_raw=sd.co_heater_v=SENSOR_UNAVAILABLE;
  sd.tvoc_ppb=sd.eco2_ppm=sd.aqi=SENSOR_UNAVAILABLE;
  sd.uvi=sd.sound_db=sd.dust_mg=SENSOR_UNAVAILABLE;
  sd.temp=sd.hum=SENSOR_UNAVAILABLE;
  sd.angle[0]=sd.angle[1]=sd.angle[2]=sd.imu_temp=SENSOR_UNAVAILABLE;

  // ── GPIO setup ───────────────────────────────────────────────────────────
  pinMode(ledPin11,OUTPUT); digitalWrite(ledPin11,LOW);
  pinMode(ledPin12,OUTPUT); digitalWrite(ledPin12,LOW);
  pinMode(ledPin13,OUTPUT); digitalWrite(ledPin13,LOW);
  pinMode(dustLED, OUTPUT); digitalWrite(dustLED, HIGH);
  pinMode(buzzPin, OUTPUT);
  pinMode(CO_ADC_PIN,   INPUT);
  pinMode(CO_A1_PIN,    INPUT);
  pinMode(UV_OUT_PIN,   INPUT);
  pinMode(UV_REF_PIN,   INPUT);
  pinMode(DUST_ADC_PIN, INPUT);

  // ── ADC MUST be configured BEFORE I2S driver install ─────────────────────
  // On ESP32 Arduino core 3.x, analogSetAttenuation() internally uses the
  // new adc_oneshot driver (driver_ng). The legacy I2S driver installs its
  // own ADC handle. When both run, the firmware aborts with:
  //   E (322) ADC: CONFLICT! driver_ng is not allowed to be used with the legacy driver
  //
  // Fix: use analogReadResolution() + analogSetPinAttenuation() per ADC pin
  // instead of the global analogSetAttenuation(). Per-pin calls do NOT
  // touch driver_ng and are safe alongside the legacy I2S driver.
  // ALSO: I2S is installed LAST in setup() so ADC is fully initialised first.
  analogReadResolution(12);
  // Set 0–3.3V range (ADC_11db) individually on every ADC1 pin we use.
  // Do NOT call analogSetAttenuation() — it conflicts with I2S legacy driver.
  analogSetPinAttenuation(CO_ADC_PIN,   ADC_11db);
  analogSetPinAttenuation(CO_A1_PIN,    ADC_11db);
  analogSetPinAttenuation(UV_OUT_PIN,   ADC_11db);
  analogSetPinAttenuation(UV_REF_PIN,   ADC_11db);
  analogSetPinAttenuation(DUST_ADC_PIN, ADC_11db);
  Serial.println("[INIT] ADC configured (12-bit, per-pin 11dB — no driver_ng conflict)");

  // ── I2C for ENS160+AHT2x ─────────────────────────────────────────────────
  Wire.begin(21, 22);  // SDA=GPIO21, SCL=GPIO22

  // ── ENS160+AHT2x init ────────────────────────────────────────────────────
  if (ENSsensorThere) {
    ensReady = initENS();
    if (!ensReady) Serial.println("[WARN] ENS160/AHT2x not detected — will retry in sensorTask");
  }

  // ── CO LEDC PWM ──────────────────────────────────────────────────────────
  if (COsensorThere) {
    ledcAttach(CO_PWM_PIN, LEDC_FREQ_CO, LEDC_RES_CO);
    ledcWrite(CO_PWM_PIN, 0);
  }

  // ── IMU UART2 ────────────────────────────────────────────────────────────
  if (IMUsensorThere) {
    imu_serial.begin(115200, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
    Serial.println("[INIT] IMU UART2 started (GPIO16/17)");
  }

  // ── GPS — ATGM336H NMEA only, 9600 baud ──────────────────────────────────
  initGPS();

  // ── OLED (optional, shares I2C) ──────────────────────────────────────────
  if (DISPLAYON) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
      Serial.println("[WARN] OLED init failed");
    else { display.setTextColor(WHITE); display.clearDisplay(); display.display(); }
  }

  randomSeed(analogRead(0));

  // ── INMP441 I2S — installed LAST, AFTER ADC configuration ────────────────
  // This ordering is critical. i2s_driver_install() and analogRead() use
  // different internal ADC driver layers. Installing I2S before configuring
  // ADC with the legacy API causes the driver_ng conflict error.
  // v2.2: initINMP441() now uses <driver/i2s_std.h>, which never touches the
  // ADC driver — this call could be moved anywhere in setup() without risk.
  // Left in this position since it costs nothing and keeps the safe pattern.
  if (INMPsensorThere) initINMP441();

  // ── Launch sensor task on Core 0 ─────────────────────────────────────────
  // WiFi is connected on Core 1 (this setup() function) before CO startup,
  // so the WiFi driver is running during any long CO initialisation.
  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 8192, NULL, 1, NULL, 0);

  // ── WiFi + Blynk config ──────────────────────────────────────────────────
  if (WIFI) wifiConnect();

  // ── CO sensor startup ────────────────────────────────────────────────────
  if (COsensorThere) {
    sens_val = sensor_reading_clean_air;
    if (!coSensorPresent()) {
      Serial.println("[WARN] CO sensor not detected (GPIO36 floating)");
      engMsg("CO: sensor absent — check A0 wiring and divider");
      if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(20))==pdTRUE) {
        sd.co_no_sensor=true; sd.status_flags|=STATUS_CO_NO_SNS;
        sd.co_ppm=SENSOR_UNAVAILABLE; xSemaphoreGive(dataMutex);
      }
    } else {
      engMsg("CO: sensor detected — starting closed-loop heater control");
    }
    startMeasurementPhase();
  }

  // ── BlynkTimer intervals ──────────────────────────────────────────────────
  blynkTimer.setInterval(BLYNK_SEND_FAST_MS, blynkSendFast);
  blynkTimer.setInterval(BLYNK_SEND_SLOW_MS, blynkSendSlow);

  engMsg("Setup OK v2.2 — I2S harmonized to i2s_std driver");
  setupDone = true;
}

// ============================================================================
// LOOP — Core 1: WiFi + Blynk only
// All sensor I/O is in sensorTask (Core 0).
// Blynk.run() in loop() is the critical design decision that prevents
// the IWDT crash seen in all v1.x versions (where Blynk ran in a FreeRTOS
// task and Blynk.connect(timeout) spun for 5s blocking interrupt handlers).
// ============================================================================

void loop()
{
  if (WIFI) {
    if (WiFi.status() == WL_CONNECTED) {
      Blynk.run();
      blynkTimer.run();
      if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(5))==pdTRUE) {
        sd.status_flags|=STATUS_WIFI_OK; xSemaphoreGive(dataMutex);
      }
    } else {
      if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(5))==pdTRUE) {
        sd.status_flags&=~STATUS_WIFI_OK; xSemaphoreGive(dataMutex);
      }
      if (millis()-lastWiFiCheck > WIFI_RECONNECT_MS) {
        lastWiFiCheck=millis();
        engMsg("WiFi: reconnect attempt");
        Blynk.disconnect(); WiFi.disconnect(); delay(500);
        wifiConnect();
      }
    }
  }
}
