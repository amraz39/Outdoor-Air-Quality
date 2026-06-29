// ============================================================================
// Outdoor Air Quality Station — ESP32 Production Firmware v1.2
// Migrated & rewritten from original Arduino Mega + Esp8266EasyIoT (AM, 2017)
//
// Hardware: ESP32 DevKit, MQ-7 CO sensor board, MICS-2710 NO2,
//           DHT21 temp/hum, Sharp GP2Y1010 dust, ML8511 UV,
//           Analog mic, JY-901/WT901 IMU, u-blox GPS (UART)
//
// Backend:  Blynk 0.6.1 LOCAL SERVER (Java 21 patched build)
//           Plain TCP port 8080. For TLS use BlynkSimpleEsp32_SSL.h + port 8441.
//
// Architecture: Core 0 = Blynk / WiFi / reporting (BlynkTimer)
//               Core 1 = All sensor reads + CO state machine
//               FreeRTOS mutex protects shared SensorData struct
//
// Credentials: secrets.h (never commit that file to version control)
// ============================================================================

// ─── CREDENTIALS from secrets.h ──────────────────────────────────────────────
// secrets.h must sit in the same sketch folder. Contents:
//   #pragma once
//   #define WIFI_SSID    "yourSSID"
//   #define WIFI_PASS    "yourPass"
//   #define BLYNK_AUTH   "yourToken"
//   #define BLYNK_SERVER "192.168.x.x"
//   #define BLYNK_PORT   8080
#include "secrets.h"

// ─── BLYNK — local server 0.6.1 ─────────────────────────────────────────────
// NOTE: BLYNK_TEMPLATE_ID / BLYNK_TEMPLATE_NAME are Blynk Cloud concepts only.
// They must NOT be defined when connecting to a local server — 0.6.1 does not
// recognise them and some library versions reject the connection.
#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>   // plain TCP — matches local server port 8080
                                 // Switch to BlynkSimpleEsp32_SSL.h for port 8441

#include <TinyGPS++.h>
#include <DHT.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// ─── FEATURE SWITCHES ────────────────────────────────────────────────────────
#define DEBUGON           false  // verbose serial output
#define DISPLAYON         false  // OLED display (shares SCL/GPIO22 with GPS — see note)
#define WIFI              true   // Blynk / WiFi enabled
#define COsensorThere     true   // MQ-7 CO sensor board present
#define IMUsensorThere    true   // JY-901 / WT901 IMU present

// Set true to enable 10-minute watchdog auto-reset (useful for unattended field use)
bool ten_mins_autoreset = false;

// ─── VOLTAGE DIVIDER CONFIGURATION ───────────────────────────────────────────
// Used on all 5V analogue sensor outputs before they reach 3.3V ESP32 ADC pins.
// Circuit:  Sensor_AO ── R_DIVIDER_SERIES ──┬── ESP32 ADC pin
//                                            │
//                                       R_DIVIDER_GND (can be two resistors in series)
//                                            │
//                                           GND
//
// Current wiring: 10 kΩ series, then 10 kΩ + 10 kΩ = 20 kΩ to GND.
// Set values in kΩ below. The divider_scale factor is computed once in setup().
//
// !! SAFETY WARNING !!
// With R_DIVIDER_SERIES = 10 kΩ and R_DIVIDER_GND = 20 kΩ:
//   Vout = Vin × 20/(10+20) = Vin × 0.6667
//   At Vin = 5.0V → Vout = 3.333V — which is 33mV ABOVE the ESP32 GPIO
//   absolute maximum of 3.3V. This is a marginal configuration.
//   In practice sensor outputs rarely hit the full 5V rail in normal
//   operating conditions, but be aware of the risk.
//   For a safer margin use 10 kΩ + 15 kΩ (Vout_max = 3.0V).
const float R_DIVIDER_SERIES = 10.0f;  // kΩ — series resistor (between sensor and ADC pin)
const float R_DIVIDER_GND    = 20.0f;  // kΩ — ground leg (e.g. two 10kΩ in series)

// Computed once in setup() — do not edit these directly.
float divider_scale  = 1.5f;   // Vin/Vout = (R_series + R_gnd) / R_gnd
float divider_ratio  = 0.667f; // Vout/Vin = R_gnd / (R_series + R_gnd)

// ─── BLYNK VIRTUAL PIN MAP ───────────────────────────────────────────────────
// V1  = Sound level (V peak-to-peak, at actual sensor)
// V2  = CO ppm (calculated)
// V3  = UV index (UVI)
// V4  = Random diagnostic number
// V5  = NO2 actual sensor voltage (V)
// V6  = CO phase (0=measuring 1=heating)
// V7  = CO raw ADC EMA
// V8  = IMU Roll  (°)
// V9  = IMU Pitch (°)
// V10 = IMU Yaw   (°)
// V11 = IMU temperature (°C)
// V12 = GPS Latitude  (decimal degrees)
// V13 = GPS Longitude (decimal degrees)
// V14 = GPS Satellites
// V15 = DHT21 Humidity (%)
// V16 = DHT21 Temperature (°C)
// V17 = GPS HDOP
// V18 = Dust density (mg/m³)
// V19 = Engineering / diagnostic message (string)
// V20 = System status bitmask (int)
//         bit 0 = WiFi OK      bit 1 = GPS fix OK
//         bit 2 = IMU OK       bit 3 = DHT OK
//         bit 4 = CO fault     bit 5 = GPS data fault
//         bit 6 = IMU stuck    bit 7 = DHT fault

// ─── OLED ────────────────────────────────────────────────────────────────────
// OLED I2C SCL = GPIO22. GPS UART1 RX is also remapped to GPIO22.
// DISPLAYON false (default) → no conflict.
// If DISPLAYON=true: remap GPS_RX_PIN to a free GPIO (e.g. GPIO13) first.
#define OLED_RESET -1
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

// ─── GPS — UART1 remapped away from flash pins GPIO9/10 ──────────────────────
#define GPS_RX_PIN    22       // GPS TX → ESP32 GPIO22 (UART1 RX remapped)
#define GPS_TX_PIN    23       // GPS RX ← ESP32 GPIO23 (UART1 TX remapped)
#define GPS_BAUD_INIT  9600
#define GPS_BAUD_FAST  115200
HardwareSerial gps_serial(1);
TinyGPSPlus    gps;

// ─── GPS DIGITAL OUTPUT SAFETY NOTE ─────────────────────────────────────────
// The u-blox GPS module (NEO-6M/7M/8M) UART TX operates at 3.3V logic.
// It is SAFE to connect directly to ESP32 GPIO without a level shifter.
// Most breakout modules include an onboard 3.3V LDO regulator — supply
// the module from the ESP32 3V3 pin to ensure correct logic levels.

// Corrected UBLOX_INIT — all checksums verified, 5 Hz, GGA explicitly enabled,
// stray POLL byte removed, inter-message delay added in initGPS():
//   GLL off · GSV off · VTG off · GSA off · RMC off
//   GGA ON (rate=1) · CFG-RATE 200ms=5Hz · CFG-PRT baud=115200
const unsigned char UBLOX_INIT[] = {
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x06,0x2A, // GLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x01,0x01,0x01,0x01,0x01,0x01,0x08,0x3E, // GSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x01,0x01,0x01,0x01,0x01,0x01,0x0A,0x52, // VTG off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x01,0x01,0x01,0x01,0x01,0x01,0x07,0x34, // GSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x01,0x01,0x01,0x01,0x01,0x01,0x09,0x48, // RMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x05,0x22, // GGA ON
  0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A,           // 5 Hz
  0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,           // baud 115200
  0x00,0xC2,0x01,0x00,0x07,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0xC4,0x96,
};

// ─── IMU — UART2 GPIO16 (RX) / GPIO17 (TX) ──────────────────────────────────
// ─── IMU DIGITAL OUTPUT SAFETY NOTE ─────────────────────────────────────────
// The JY-901/WT901 UART TX level matches its supply voltage.
// POWER THE IMU FROM THE ESP32 3V3 PIN — this makes TX = 3.3V logic,
// safe for direct connection to ESP32 UART2 RX (GPIO16).
// If powered at 5V, the 5V UART TX would damage the ESP32 GPIO.
#define IMU_RX_PIN  16
#define IMU_TX_PIN  17
HardwareSerial imu_serial(2);

// ─── DHT ─────────────────────────────────────────────────────────────────────
// GPIO5 is a strapping pin (must be HIGH at boot). DHT idle line is HIGH → safe.
//
// ─── DHT21 DIGITAL OUTPUT SAFETY NOTE ───────────────────────────────────────
// The DHT21 uses a single open-drain data line pulled up to Vcc.
// POWER THE DHT21 FROM THE ESP32 3V3 PIN — the data line will then idle
// at 3.3V, safe for direct connection to GPIO5.
// If the DHT21 is powered at 5V, the data line idles at 5V and WILL
// damage the ESP32 GPIO. No level shifter is needed when using 3.3V supply.
// The 4.7 kΩ pull-up resistor must connect to 3.3V, not 5V.
#define DHTPIN  5
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);

// ─── ANALOG PINS (ADC1 only — ADC2 unusable while WiFi is active) ────────────
// 5V sensor signals pass through the voltage divider defined above
// before reaching the ESP32 ADC pins. See divider_scale / divider_ratio.
//
// ML8511 UV sensor is 3.3V ratiometric — no divider needed.
// Sharp GP2Y1010 IR LED control pin (GPIO18) is a digital output from the
// ESP32 at 3.3V — safe to connect directly to the sensor's ILED input.
//
// Mega A0/A1 (CO node)  → GPIO36 VP  ADC1_CH0  [input-only, 5V divider]
// Mega A2    (UV OUT)   → GPIO39 VN  ADC1_CH3  [input-only, direct 3.3V]
// Mega A3    (UV REF)   → GPIO34     ADC1_CH6  [input-only, direct 3.3V]
// Mega A4    (Dust AO)  → GPIO35     ADC1_CH7  [input-only, 5V divider]
// Mega A5    (Mic AO)   → GPIO32     ADC1_CH4  [5V divider if module is 5V]
// Mega A7    (NO2 AO)   → GPIO33     ADC1_CH5  [5V divider]
#define CO_ADC_PIN      36
#define UVOUT           39
#define REF_3V3         34
#define dustPin         35
#define micPin          32
#define NO2_SENSOR_PIN  33

// ─── DIGITAL PINS ────────────────────────────────────────────────────────────
// Mega D10 → GPIO14  LED green  (no boot constraint)
// Mega D11 → GPIO27  LED orange (ADC2, output-only — no WiFi conflict)
// Mega D12 → GPIO26  LED red    (ADC2, output-only — no WiFi conflict)
// Mega D7  → GPIO19  Buzzer
// Mega D9  → GPIO4   CO heater PWM via LEDC (avoids all strapping pins)
// Mega D53 → GPIO18  Sharp GP2Y1010 dust IR LED (active LOW, 3.3V safe direct)
#define ledPin11   14   // GREEN  — CO ≤ 10 ppm
#define ledPin12   27   // ORANGE — CO 10–20 ppm
#define ledPin13   26   // RED    — CO > 20 ppm
#define buzzPin    19   // Passive buzzer
#define dustLED    18   // Dust sensor IR LED, active LOW
#define CO_PWM_PIN  4   // CO heater PWM output (LEDC)

// ─── LEDC (replaces Mega Timer2 hardware registers) ──────────────────────────
#define LEDC_CHAN_CO   0
#define LEDC_CHAN_BUZ  1
#define LEDC_FREQ_CO   5000
#define LEDC_RES_CO    8     // 8-bit duty: 0–255

// ─── WATCHDOG / HEALTH TIMEOUTS ──────────────────────────────────────────────
#define GPS_TIMEOUT_MS    15000  // GPS fix considered stale after 15 s
#define IMU_TIMEOUT_MS     5000  // IMU frame gap > 5 s → stuck
#define DHT_MAX_FAILS         5  // consecutive DHT read failures → fault flag
#define CO_PHASE_MAX_MS  180000  // CO phase stuck > 3 min → force transition
#define WIFI_RECONNECT_MS 30000  // retry WiFi every 30 s when dropped
#define ENG_MSG_INTERVAL  10000  // engineering message Blynk flush interval
#define BLYNK_SEND_MS      5000  // sensor data send cadence

// ─── SYSTEM STATUS BITMASK ───────────────────────────────────────────────────
#define STATUS_WIFI_OK   (1<<0)
#define STATUS_GPS_OK    (1<<1)
#define STATUS_IMU_OK    (1<<2)
#define STATUS_DHT_OK    (1<<3)
#define STATUS_CO_FAULT  (1<<4)
#define STATUS_GPS_FAULT (1<<5)
#define STATUS_IMU_STUCK (1<<6)
#define STATUS_DHT_FAULT (1<<7)

// ─── TIMING ──────────────────────────────────────────────────────────────────
#define SLEEP_TIME 25          // sensor loop minimum cadence (ms)
const int sampleSoundWindow = 50;

// ─── CO CALIBRATION ──────────────────────────────────────────────────────────
// !! IMPORTANT: sensor_reading_clean_air is the raw 12-bit ADC value seen at
//    GPIO36 — i.e. the DIVIDED voltage, NOT the actual sensor node voltage.
//    The divider scale is applied internally in raw_value_to_CO_ppm(). !!
//
// To calibrate: power station outdoors in fresh air, wait for the first full
// 90 s measurement phase, then read the raw ADC value from Serial debug output
// and set it here.
float reference_resistor_kOhm   = 9.98;
float sensor_reading_clean_air  = 600.65; // raw 12-bit ADC at GPIO36 in clean air — CALIBRATE
float sensor_reading_100_ppm_CO = -1;     // optional: raw ADC at GPIO36 at known 100 ppm CO

// ─── SHARED DATA STRUCT ──────────────────────────────────────────────────────
// Written exclusively by sensorTask (Core 1).
// Read exclusively by blynkTask (Core 0) after taking dataMutex.
struct SensorData {
  float    co_ppm;
  float    co_raw;          // raw ADC EMA at GPIO36 (after divider)
  byte     co_phase;
  bool     co_fault;
  float    no2_raw;         // raw ADC at GPIO33 (after divider)
  float    no2_voltage;     // actual sensor voltage (divider corrected)
  float    uvi;
  float    sound_v;         // actual peak-to-peak voltage at sensor (corrected if 5V mic)
  float    dust_mg;         // dust density mg/m³ (divider-corrected Vo)
  float    temp;
  float    hum;
  bool     dht_fault;
  float    angle[3];
  float    imu_temp;
  bool     imu_ok;
  double   lat;
  double   lng;
  float    hdop;
  float    sats;
  bool     gps_fix;
  uint32_t status_flags;
  char     eng_msg[128];
  int      rand_num;
};

SensorData        sd;
SemaphoreHandle_t dataMutex;

// ─── CO STATE MACHINE ────────────────────────────────────────────────────────
float opt_voltage              = 0;
byte  opt_width                = 240;
byte  co_phase                 = 0;
unsigned long co_phase_start   = 0;
float sens_val                 = 0;
float sens_val_last            = 0;
float last_CO_ppm              = 0;
float sensor_base_resistance_kOhm;
float sensor_100ppm_CO_resistance_kOhm;

// ─── IMU ─────────────────────────────────────────────────────────────────────
unsigned char  Re_buf[12];
unsigned char  imu_sign    = 0;
int            imu_counter = 0;
unsigned long  lastIMUframe = 0;
float          angle[3]    = {0, 0, 0};
float          imuT        = 0;
const float    tempComp    = -7.2;

// ─── FAULT / TIMING STATE ────────────────────────────────────────────────────
int           dht_fail_count = 0;
unsigned long lastGPSdata    = 0;
unsigned long lastWiFiCheck  = 0;
long          countNum       = 0;
int           counterNum     = 0;
int           countReset     = 0;

// ─── GPS MISC ────────────────────────────────────────────────────────────────
float hum_prev = 0, temp_prev = 0;
bool  prevZeroVal = false;
char  LatGPS[12], LongGPS[12];
char  LatGPS_Prev[12], LongGPS_Prev[12];
float SatGPS = 0, HDOP = 0;
String pin11state = "", pin12state = "", pin13state = "";

// ─── BLYNK TIMER ─────────────────────────────────────────────────────────────
BlynkTimer blynkTimer;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

// 9-sample ADC average — small gaps improve ESP32 ADC linearity
int averageAnalogRead(int pinToRead)
{
  const byte N = 9;
  unsigned long sum = 0;
  for (byte i = 0; i < N; i++) {
    sum += analogRead(pinToRead);
    delayMicroseconds(100);
  }
  return (int)(sum / N);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Convert raw 12-bit ADC reading (after divider) to actual sensor voltage.
// Applies the divider_scale factor computed from R_DIVIDER_SERIES / R_DIVIDER_GND.
//   actual_V = adc_counts × (3.3V / 4095) × divider_scale
float adcToSensorVoltage(int adcCounts)
{
  return (float)adcCounts * (3.3f / 4095.0f) * divider_scale;
}

// Engineering message — mutex-safe, writes to Serial and shared struct
void engMsg(const char* msg)
{
  Serial.print("[ENG] "); Serial.println(msg);
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    strncpy(sd.eng_msg, msg, sizeof(sd.eng_msg) - 1);
    sd.eng_msg[sizeof(sd.eng_msg) - 1] = '\0';
    xSemaphoreGive(dataMutex);
  }
}

void engMsgf(const char* fmt, ...)
{
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  engMsg(buf);
}

// Buzzer — LEDC channel 1, separate from CO PWM channel 0
void buzzerTone(uint16_t frequency, uint16_t durationMs)
{
  if (frequency == 0) return;
  ledcSetup(LEDC_CHAN_BUZ, frequency, 8);
  ledcAttachPin(buzzPin, LEDC_CHAN_BUZ);
  ledcWrite(LEDC_CHAN_BUZ, 128);
  delay(durationMs);
  ledcWrite(LEDC_CHAN_BUZ, 0);
  ledcDetachPin(buzzPin);
}

// ============================================================================
// GPS
// ============================================================================

void initGPS()
{
  gps_serial.begin(GPS_BAUD_INIT, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(200);
  // Send UBX messages one at a time with a 15 ms gap — u-blox needs ~10 ms
  // per CFG command. Sending back-to-back can cause the baud-switch to be lost.
  int i = 0;
  while (i < (int)sizeof(UBLOX_INIT)) {
    if (UBLOX_INIT[i] == 0xB5 && i + 5 < (int)sizeof(UBLOX_INIT)) {
      int paylen = UBLOX_INIT[i + 4] | (UBLOX_INIT[i + 5] << 8);
      int msglen = 6 + paylen + 2;
      gps_serial.write(&UBLOX_INIT[i], msglen);
      gps_serial.flush();
      delay(15);
      i += msglen;
    } else {
      i++;
    }
  }
  delay(200);
  gps_serial.end();
  delay(100);
  gps_serial.begin(GPS_BAUD_FAST, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  engMsg("GPS: UBX config sent, running at 115200 baud 5Hz GGA-only");
}

int feedGPS(unsigned long maxMs)
{
  int fed = 0;
  unsigned long t = millis();
  while (millis() - t < maxMs) {
    while (gps_serial.available()) {
      gps.encode(gps_serial.read());
      fed++;
    }
    if (fed > 0 && !gps_serial.available()) break;
    delay(1);
  }
  return fed;
}

// ============================================================================
// CO SENSOR (MQ-7) — PWM calibration and state machine
// ============================================================================

void setHeatDuty(byte duty) { ledcWrite(LEDC_CHAN_CO, duty); }

void startMeasurementPhase()
{
  co_phase       = 0;
  co_phase_start = millis();
  setHeatDuty(opt_width);
  engMsgf("CO: measurement phase (duty=%d, ~1.4V at sensor)", opt_width);
}

void startHeatingPhase()
{
  co_phase       = 1;
  co_phase_start = millis();
  setHeatDuty(255);
  engMsg("CO: heating phase (duty=255, ~5V at sensor)");
}

// Sweep LEDC duty 0→249 and find the width that produces ~1.4V at the MQ-7
// sense node. Because the sense node is read through the voltage divider,
// the ADC pin will see:
//   Vout_adc = 1.4V × divider_ratio  (= 1.4 × R_gnd / (R_series + R_gnd))
// This target is computed from the user-configured divider values so it
// remains correct regardless of which resistors are fitted.
bool pwm_adjust()
{
  const float target_sensor_V = 1.4f;                        // desired voltage AT the MQ-7 node
  const float target_adc_V    = target_sensor_V * divider_ratio; // what the ADC pin will see
  const float raw2v           = 3.3f / 4095.0f;
  float prev_v = 3.3f;
  opt_width = 240; // safe fallback

  engMsgf("CO-CAL: target %.3fV at sensor → %.3fV at ADC (divider=%.4f)",
          target_sensor_V, target_adc_V, divider_ratio);

  for (int w = 0; w < 250; w++) {
    setHeatDuty((byte)w);
    delay(50);
    float avg = 0;
    for (int x = 0; x < 20; x++) { avg += analogRead(CO_ADC_PIN); delay(2); }
    avg /= 20.0f;
    float v = avg * raw2v;   // voltage at ADC pin (after divider)
    if (DEBUGON) Serial.printf("CO-CAL w=%d Vadc=%.3f Vsensor=%.3f\n",
                               w, v, v * divider_scale);
    if (v < target_adc_V && prev_v >= target_adc_V) {
      float dn = target_adc_V - v, dp = prev_v - target_adc_V;
      opt_width   = (dn < dp) ? (byte)w : (byte)(w > 0 ? w - 1 : 0);
      opt_voltage = ((dn < dp) ? v : prev_v) * divider_scale; // report actual sensor V
      engMsgf("CO-CAL OK: duty=%d sensor_V=%.3fV adc_V=%.3fV",
              opt_width, opt_voltage, opt_voltage * divider_ratio);
      return true;
    }
    prev_v = v;
  }
  engMsg("CO-CAL WARN: 1.4V target not found in sweep, using duty=240");
  return false;
}

// Convert EMA-smoothed raw ADC reading (as seen at GPIO36, after divider) to CO ppm.
// The reading must be scaled back to the actual sensor node voltage before
// computing the MQ-7 resistance, because reference_resistor_kOhm is measured
// in the physical circuit at the sensor node — not at the ADC pin.
float raw_value_to_CO_ppm(float adc_ema)
{
  if (adc_ema < 1.0f) return -1.0f;

  // Scale ADC reading back to what it would be at the actual sensor node voltage.
  // The divider attenuates the sensor voltage:
  //   V_adc = V_sensor × divider_ratio
  // so: V_sensor / V_adc = divider_scale = (R_series + R_gnd) / R_gnd
  // We re-scale the ADC count proportionally:
  //   adc_sensor_equivalent = adc_ema × divider_scale
  //   (this equals the ADC count we'd get if we sampled at the sensor node directly)
  float adc_sensor_equiv = adc_ema * divider_scale;

  // Guard: clamp to 12-bit range after scaling
  if (adc_sensor_equiv > 4095.0f) adc_sensor_equiv = 4095.0f;

  // MQ-7 resistance from voltage-divider bridge formula (unchanged from original):
  //   R_sensor = R_ref × (ADC_max / ADC_reading) − R_ref
  sensor_base_resistance_kOhm =
    reference_resistor_kOhm * 4095.0f / sensor_reading_clean_air - reference_resistor_kOhm;

  // Note: sensor_reading_clean_air is stored as the raw ADC value AT GPIO36 (divided).
  // When used in the formula above it must also be scaled so both sides of the
  // equation are in the same "space" (sensor-node equivalent counts).
  // We pre-scale it here consistently:
  float clean_equiv = sensor_reading_clean_air * divider_scale;
  if (clean_equiv > 4095.0f) clean_equiv = 4095.0f;
  sensor_base_resistance_kOhm =
    reference_resistor_kOhm * 4095.0f / clean_equiv - reference_resistor_kOhm;

  if (sensor_reading_100_ppm_CO > 0.0f) {
    float cal100_equiv = sensor_reading_100_ppm_CO * divider_scale;
    if (cal100_equiv > 4095.0f) cal100_equiv = 4095.0f;
    sensor_100ppm_CO_resistance_kOhm =
      reference_resistor_kOhm * 4095.0f / cal100_equiv - reference_resistor_kOhm;
  } else {
    sensor_100ppm_CO_resistance_kOhm = sensor_base_resistance_kOhm * 0.5f;
  }

  if (sensor_base_resistance_kOhm <= 0.0f || sensor_100ppm_CO_resistance_kOhm <= 0.0f)
    return -1.0f;

  float sensor_R = reference_resistor_kOhm * 4095.0f / adc_sensor_equiv - reference_resistor_kOhm;
  if (sensor_R <= 0.0f) return 0.0f;

  float CO_ppm = 100.0f * (expf(sensor_100ppm_CO_resistance_kOhm / sensor_R) - 1.648f);
  return (CO_ppm < 0.0f) ? 0.0f : CO_ppm;
}

void tickCO()
{
  unsigned long elapsed = millis() - co_phase_start;

  // Watchdog: phase stuck longer than CO_PHASE_MAX_MS
  if (elapsed > CO_PHASE_MAX_MS) {
    engMsgf("CO WATCHDOG: phase %d stuck >%lus — forcing transition",
            co_phase, (unsigned long)(CO_PHASE_MAX_MS / 1000));
    (co_phase == 1) ? startMeasurementPhase() : startHeatingPhase();
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      sd.co_fault = true;
      sd.status_flags |= STATUS_CO_FAULT;
      xSemaphoreGive(dataMutex);
    }
    return;
  }

  // Normal phase transitions
  if (co_phase == 1 && elapsed > 60000UL) {
    startMeasurementPhase();
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      sd.co_fault = false; sd.status_flags &= ~STATUS_CO_FAULT;
      xSemaphoreGive(dataMutex);
    }
  }
  if (co_phase == 0 && elapsed > 90000UL) {
    float ppm = raw_value_to_CO_ppm(sens_val);
    last_CO_ppm   = (ppm >= 0.0f) ? ppm : last_CO_ppm;
    sens_val_last = sens_val;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      sd.co_ppm   = last_CO_ppm;
      sd.co_raw   = sens_val;
      sd.co_phase = co_phase;
      sd.co_fault = (ppm < 0.0f);
      if (ppm < 0.0f) sd.status_flags |= STATUS_CO_FAULT;
      else            sd.status_flags &= ~STATUS_CO_FAULT;
      xSemaphoreGive(dataMutex);
    }
    engMsgf("CO: cycle done ppm=%.1f raw_adc=%.0f sensor_equiv=%.0f",
            last_CO_ppm, sens_val, sens_val * divider_scale);
    startHeatingPhase();
    return;
  }

  // ADC read every tick — guard for open-circuit / rail conditions
  // The rail check uses the ADC-pin range (after divider), so full 5V sensor output
  // maps to 4095 × divider_ratio at the ADC. We check against ±5% of that.
  float v = (float)analogRead(CO_ADC_PIN);
  float expected_max_adc = 4095.0f * divider_ratio;
  if (v < 10.0f || v > (expected_max_adc * 1.05f)) {
    if (DEBUGON) Serial.printf("CO ADC out of range: %.0f (expected 0–%.0f)\n",
                               v, expected_max_adc);
    return;
  }
  sens_val = 0.7f * sens_val + 0.3f * v; // EMA α=0.3 (original coefficients)

  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    sd.co_raw = sens_val; sd.co_phase = co_phase;
    xSemaphoreGive(dataMutex);
  }
  if (DEBUGON)
    Serial.printf("CO phase=%d t=%lus adc=%.0f ema=%.1f sensor_V=%.3fV ppm=%.1f\n",
                  co_phase, elapsed / 1000, v, sens_val,
                  v * (3.3f / 4095.0f) * divider_scale, last_CO_ppm);
}

// ============================================================================
// IMU (JY-901 / WT901 — 0x55 framed 11-byte protocol)
// ============================================================================

bool pollIMU()
{
  while (imu_serial.available()) {
    unsigned char b = (unsigned char)imu_serial.read();
    if (imu_counter == 0 && b != 0x55) continue;
    Re_buf[imu_counter++] = b;
    if (imu_counter == 11) {
      imu_counter = 0;
      if (Re_buf[0] == 0x55 && Re_buf[1] == 0x53) {
        angle[0] = (short((Re_buf[3] << 8) | Re_buf[2])) / 32768.0f * 180.0f;
        angle[1] = (short((Re_buf[5] << 8) | Re_buf[4])) / 32768.0f * 180.0f;
        angle[2] = (short((Re_buf[7] << 8) | Re_buf[6])) / 32768.0f * 180.0f;
        imuT = ((short((Re_buf[9] << 8) | Re_buf[8])) / 340.0f + 36.25f) + tempComp;
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
  while (millis() - start < 200) { if (pollIMU()) { got = true; break; } delay(1); }

  unsigned long age = millis() - lastIMUframe;
  bool stuck = (lastIMUframe > 0 && age > IMU_TIMEOUT_MS);

  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (got) {
      sd.angle[0] = angle[0]; sd.angle[1] = angle[1]; sd.angle[2] = angle[2];
      sd.imu_temp = imuT; sd.imu_ok = true;
      sd.status_flags |= STATUS_IMU_OK;
      sd.status_flags &= ~STATUS_IMU_STUCK;
    }
    if (stuck) {
      sd.imu_ok = false;
      sd.status_flags &= ~STATUS_IMU_OK;
      sd.status_flags |= STATUS_IMU_STUCK;
    }
    xSemaphoreGive(dataMutex);
  }
  if (stuck) engMsgf("IMU STUCK: no frame for %lus", age / 1000);
}

// ============================================================================
// SENSOR TASK — Core 1
// ============================================================================

void sensorTask(void* pvParam)
{
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // full 0–3.3V range on all ADC1 pins

  while (true)
  {
    unsigned long loopStart = millis();

    // ── UV (ML8511 — 3.3V ratiometric, NO voltage divider) ──────────────────
    // The ML8511 outputs a voltage ratiometric to its own 3.3V supply.
    // Both OUT and REF pins connect directly to the ESP32 — no divider, no scaling.
    // Vout/Vref ratio is independent of supply voltage.
    int uvLevel  = averageAnalogRead(UVOUT);
    int refLevel = averageAnalogRead(REF_3V3);
    float outV = (refLevel > 50) ? (3.3f / (float)refLevel) * (float)uvLevel : 0.0f;
    float uvi  = (12.49f * (outV + (1.0f - 0.97f)) - 12.49f + 0.3f);
    if (uvi < 0)  uvi = 0;
    if (uvi > 20) uvi = 20;

    // ── NO2 (MICS-2710 — 5V sensor, analogue output through voltage divider) ─
    // ADC sees: Vadc = Vsensor × divider_ratio
    // Recover actual sensor voltage: Vsensor = Vadc × divider_scale
    int   no2_adc = averageAnalogRead(NO2_SENSOR_PIN);
    float no2v    = adcToSensorVoltage(no2_adc);  // actual sensor voltage 0–5V

    // ── Sound (analogue mic — divider applied if module is 5V) ───────────────
    // Peak-to-peak measurement recovers amplitude; divider_scale restores
    // the actual signal swing at the microphone output.
    unsigned int sigMax = 0, sigMin = 4095;
    unsigned long sw = millis();
    while (millis() - sw < (unsigned long)sampleSoundWindow) {
      unsigned int s = (unsigned int)analogRead(micPin);
      if (s < 4095) { if (s > sigMax) sigMax = s; if (s < sigMin) sigMin = s; }
    }
    // Convert ADC peak-to-peak counts to actual sensor voltage swing.
    // If the mic module runs at 3.3V, remove the × divider_scale term and
    // connect it directly without a divider — then set MIC_IS_5V false below.
    const bool MIC_IS_5V = true;  // set false if mic module powered at 3.3V
    float soundV = 0;
    if (sigMax > sigMin) {
      float adc_pp_V = (sigMax - sigMin) * (3.3f / 4095.0f);
      soundV = MIC_IS_5V ? (adc_pp_V * divider_scale) : adc_pp_V;
    }

    // ── Dust (Sharp GP2Y1010 — 5V sensor, analogue output through divider) ───
    // GPIO18 (digital output, 3.3V) drives the ILED input — safe direct connect.
    // The Vo analogue output goes through the voltage divider; divider_scale
    // restores the actual sensor Vo before applying the calibration curve.
    digitalWrite(dustLED, LOW);
    delayMicroseconds(280);
    int voMeas = analogRead(dustPin);
    delayMicroseconds(40);
    digitalWrite(dustLED, HIGH);
    float calcV    = adcToSensorVoltage(voMeas);   // actual Vo at sensor output
    float dustDens = 0.17f * calcV - 0.1f;
    if (dustDens < 0) dustDens = 0;

    // ── DHT21 — digital, 3.3V supply, direct GPIO connection ─────────────────
    float h = dht.readHumidity(), t = dht.readTemperature();
    bool dhtOK = (!isnan(h) && !isnan(t) && h >= 0 && h <= 100 && t > -40 && t < 80);
    if (dhtOK) {
      hum_prev = h; temp_prev = t; dht_fail_count = 0;
    } else {
      dht_fail_count++;
      h = hum_prev; t = temp_prev;
      if (dht_fail_count == DHT_MAX_FAILS)
        engMsgf("DHT FAULT: %d consecutive read failures", dht_fail_count);
    }

    // ── CO state machine ─────────────────────────────────────────────────────
    if (COsensorThere) tickCO();

    // ── GPS (digital UART 3.3V — direct connection) ──────────────────────────
    feedGPS(150);
    SatGPS = (float)gps.satellites.value();
    HDOP   = (SatGPS >= 1) ? (float)(gps.hdop.value()) / 100.0f : 99.9f;
    memcpy(LatGPS_Prev, LatGPS, sizeof(LatGPS));
    memcpy(LongGPS_Prev, LongGPS, sizeof(LongGPS));

    bool gpsFix = (gps.location.isValid() && gps.location.age() < GPS_TIMEOUT_MS && SatGPS >= 4);
    double lat = 0, lng = 0;
    if (gpsFix) {
      lat = gps.location.lat(); lng = gps.location.lng();
      lastGPSdata = millis(); prevZeroVal = false;
    } else {
      if (!prevZeroVal) {
        lat = gps.location.lat() + 0.0001; lng = gps.location.lng() + 0.0001;
        prevZeroVal = true;
        engMsgf("GPS: fix lost (sats=%.0f age=%lums)", SatGPS, gps.location.age());
      } else { lat = 0; lng = 0; }
      buzzerTone(12000, 90);
    }
    if (lastGPSdata > 0 && millis() - lastGPSdata > GPS_TIMEOUT_MS)
      engMsgf("GPS FAULT: no valid data for >%ds", GPS_TIMEOUT_MS / 1000);

    dtostrf(lat, 9, 6, LatGPS);
    dtostrf(lng, 9, 6, LongGPS);

    // ── IMU (digital UART 3.3V — direct connection) ──────────────────────────
    if (IMUsensorThere) readIMU();

    // ── Status flags ─────────────────────────────────────────────────────────
    uint32_t flags = 0;
    if (dhtOK)                                          flags |= STATUS_DHT_OK;
    else if (dht_fail_count >= DHT_MAX_FAILS)           flags |= STATUS_DHT_FAULT;
    if (gpsFix)                                         flags |= STATUS_GPS_OK;
    if (lastGPSdata > 0 && millis() - lastGPSdata > GPS_TIMEOUT_MS)
                                                        flags |= STATUS_GPS_FAULT;

    // ── LED control ──────────────────────────────────────────────────────────
    float cppm = last_CO_ppm;
    digitalWrite(ledPin11, (cppm <= 10)              ? HIGH : LOW);
    digitalWrite(ledPin12, (cppm > 10 && cppm <= 20) ? HIGH : LOW);
    digitalWrite(ledPin13, (cppm > 20)               ? HIGH : LOW);
    pin11state = digitalRead(ledPin11);
    pin12state = digitalRead(ledPin12);
    pin13state = digitalRead(ledPin13);

    // ── OLED ─────────────────────────────────────────────────────────────────
    if (DISPLAYON) {
      display.clearDisplay(); display.setCursor(0, 0); display.setTextSize(1);
      display.printf("CO:%.1fppm UV:%.1f\n", cppm, uvi);
      display.printf("T:%.1fC H:%.0f%%\n", t, h);
      display.printf("NO2:%.2fV Dust:%.3f\n", no2v, dustDens);
      display.printf("%s\n%s\n", LatGPS, LongGPS);
      if (IMUsensorThere)
        display.printf("IMU:%.0f/%.0f/%.0f\n", angle[0], angle[1], angle[2]);
      display.display();
    }

    // ── Write all results to shared struct ───────────────────────────────────
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      sd.uvi         = uvi;
      sd.no2_raw     = (float)no2_adc;
      sd.no2_voltage = no2v;
      sd.sound_v     = soundV;
      sd.dust_mg     = dustDens;
      sd.temp        = t;
      sd.hum         = h;
      sd.dht_fault   = (dht_fail_count >= DHT_MAX_FAILS);
      sd.lat         = lat;
      sd.lng         = lng;
      sd.hdop        = HDOP;
      sd.sats        = SatGPS;
      sd.gps_fix     = gpsFix;
      sd.rand_num    = random(1, 10);
      sd.status_flags = (sd.status_flags & (STATUS_WIFI_OK | STATUS_IMU_OK |
                         STATUS_IMU_STUCK | STATUS_CO_FAULT)) | flags;
      xSemaphoreGive(dataMutex);
    }

    if (DEBUGON)
      Serial.printf("[SENS] CO=%.1fppm phase=%d NO2=%.3fV UV=%.2f Snd=%.3fV "
                    "Dust=%.4f T=%.1f H=%.0f GPS=%d sats=%.0f "
                    "IMU=%.1f/%.1f/%.1f divScale=%.4f\n",
                    last_CO_ppm, co_phase, no2v, uvi, soundV, dustDens,
                    t, h, (int)gpsFix, SatGPS,
                    angle[0], angle[1], angle[2], divider_scale);

    // ── 10-minute auto-reset (optional) ──────────────────────────────────────
    countNum++; counterNum++; countReset++;
    if (ten_mins_autoreset && countReset >= 600) {
      engMsg("AUTO-RESET: 10-minute watchdog triggered");
      delay(200);
      buzzerTone(1000, 200);
      countReset = 0;
      ESP.restart();
    }

    unsigned long elapsed = millis() - loopStart;
    if (elapsed < SLEEP_TIME) delay(SLEEP_TIME - elapsed);
  }
}

// ============================================================================
// BLYNK SEND — called by BlynkTimer on Core 0
// ============================================================================

void blynkSendAll()
{
  if (!WIFI || WiFi.status() != WL_CONNECTED) return;

  SensorData snap;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    snap = sd;
    xSemaphoreGive(dataMutex);
  } else return;

  Blynk.virtualWrite(V1,  snap.sound_v);
  Blynk.virtualWrite(V2,  snap.co_ppm);
  Blynk.virtualWrite(V3,  snap.uvi);
  Blynk.virtualWrite(V4,  snap.rand_num);
  Blynk.virtualWrite(V5,  snap.no2_voltage);  // actual sensor voltage (divider corrected)
  Blynk.virtualWrite(V6,  snap.co_phase);
  Blynk.virtualWrite(V7,  snap.co_raw);
  Blynk.virtualWrite(V8,  snap.angle[0]);
  Blynk.virtualWrite(V9,  snap.angle[1]);
  Blynk.virtualWrite(V10, snap.angle[2]);
  Blynk.virtualWrite(V11, snap.imu_temp);
  Blynk.virtualWrite(V12, snap.lat);
  Blynk.virtualWrite(V13, snap.lng);
  Blynk.virtualWrite(V14, snap.sats);
  Blynk.virtualWrite(V15, snap.hum);
  Blynk.virtualWrite(V16, snap.temp);
  Blynk.virtualWrite(V17, snap.hdop);
  Blynk.virtualWrite(V18, snap.dust_mg);
  Blynk.virtualWrite(V19, snap.eng_msg);
  Blynk.virtualWrite(V20, (int)snap.status_flags);

  if (DEBUGON) Serial.printf("[BLYNK] sent vPins, status=0x%02X\n", snap.status_flags);
}

// ============================================================================
// BLYNK / WIFI TASK — Core 0
// ============================================================================

void blynkTask(void* pvParam)
{
  Serial.printf("[WiFi] Connecting to %s...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long wStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wStart < 20000) {
    delay(500); Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected, IP=%s\n", WiFi.localIP().toString().c_str());
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      sd.status_flags |= STATUS_WIFI_OK;
      xSemaphoreGive(dataMutex);
    }
    Blynk.config(BLYNK_AUTH, BLYNK_SERVER, BLYNK_PORT);
    Blynk.connect(5000);
    engMsgf("Blynk connected to %s:%d", BLYNK_SERVER, BLYNK_PORT);
    buzzerTone(5000, 100);
  } else {
    Serial.println("\n[WiFi] FAILED — running offline");
    engMsg("WiFi FAILED: running offline");
    buzzerTone(1000, 800); delay(300);
    buzzerTone(1000, 800); delay(300);
    buzzerTone(1000, 800);
  }

  blynkTimer.setInterval(BLYNK_SEND_MS, blynkSendAll);

  blynkTimer.setInterval(ENG_MSG_INTERVAL, []() {
    if (!WIFI || WiFi.status() != WL_CONNECTED) return;
    char msg[128];
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      strncpy(msg, sd.eng_msg, sizeof(msg));
      xSemaphoreGive(dataMutex);
    }
    if (strlen(msg) > 0) Blynk.virtualWrite(V19, msg);
  });

  while (true) {
    if (WIFI) {
      if (WiFi.status() != WL_CONNECTED && millis() - lastWiFiCheck > WIFI_RECONNECT_MS) {
        lastWiFiCheck = millis();
        Serial.println("[WiFi] Reconnecting...");
        engMsg("WiFi: reconnect attempt");
        WiFi.reconnect();
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          sd.status_flags &= ~STATUS_WIFI_OK;
          xSemaphoreGive(dataMutex);
        }
      } else if (WiFi.status() == WL_CONNECTED) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          sd.status_flags |= STATUS_WIFI_OK;
          xSemaphoreGive(dataMutex);
        }
      }
      Blynk.run();
      blynkTimer.run();
    }
    delay(10);
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup()
{
  Serial.begin(115200);
  Serial.println("\n[INIT] Air Quality Station ESP32 v1.2");
  Serial.printf("[INIT] ten_mins_autoreset = %s\n", ten_mins_autoreset ? "ON" : "OFF");

  // ── Compute divider factors from user-configured resistor values ──────────
  divider_ratio = R_DIVIDER_GND / (R_DIVIDER_SERIES + R_DIVIDER_GND);
  divider_scale = (R_DIVIDER_SERIES + R_DIVIDER_GND) / R_DIVIDER_GND;
  float vout_at_5v = 5.0f * divider_ratio;
  Serial.printf("[INIT] Divider: R_series=%.0fkΩ R_gnd=%.0fkΩ  "
                "ratio=%.4f  scale=%.4f  Vout@5V=%.3fV\n",
                R_DIVIDER_SERIES, R_DIVIDER_GND,
                divider_ratio, divider_scale, vout_at_5v);
  if (vout_at_5v > 3.3f) {
    Serial.printf("[WARN] Vout at 5V sensor = %.3fV exceeds ESP32 GPIO max (3.3V)! "
                  "Risk of GPIO damage at full sensor output.\n", vout_at_5v);
    engMsg("WARN: divider Vout@5V exceeds 3.3V GPIO max — check resistors");
  }

  dataMutex = xSemaphoreCreateMutex();
  memset(&sd, 0, sizeof(sd));
  strncpy(sd.eng_msg, "Booting...", sizeof(sd.eng_msg));

  pinMode(ledPin11, OUTPUT); digitalWrite(ledPin11, LOW);
  pinMode(ledPin12, OUTPUT); digitalWrite(ledPin12, LOW);
  pinMode(ledPin13, OUTPUT); digitalWrite(ledPin13, LOW);
  pinMode(dustLED,  OUTPUT); digitalWrite(dustLED,  HIGH); // active LOW → off
  pinMode(buzzPin,  OUTPUT);
  pinMode(UVOUT,          INPUT);
  pinMode(REF_3V3,        INPUT);
  pinMode(micPin,         INPUT);
  pinMode(NO2_SENSOR_PIN, INPUT);
  pinMode(CO_ADC_PIN,     INPUT);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  if (COsensorThere) {
    ledcSetup(LEDC_CHAN_CO, LEDC_FREQ_CO, LEDC_RES_CO);
    ledcAttachPin(CO_PWM_PIN, LEDC_CHAN_CO);
    ledcWrite(LEDC_CHAN_CO, 0);
  }

  dht.begin();

  if (IMUsensorThere) {
    imu_serial.begin(115200, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
    Serial.println("[INIT] IMU UART2 started (GPIO16/17)");
  }

  initGPS();

  if (DISPLAYON) {
    Wire.begin();
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
      Serial.println("[WARN] OLED init failed");
    else {
      display.setTextColor(WHITE);
      display.clearDisplay();
      display.display();
    }
  }

  randomSeed(analogRead(0));

  if (COsensorThere) {
    sens_val = sensor_reading_clean_air;
    Serial.println("[INIT] CO PWM calibration...");
    bool calOK = pwm_adjust();
    Serial.printf("[INIT] CO cal %s duty=%d sensor_V=%.3f\n",
                  calOK ? "OK" : "WARN", opt_width, opt_voltage);
    delay(2000);
    startMeasurementPhase();
  }

  if (DEBUGON) {
    Serial.printf("[INIT] Pins: CO_ADC=%d CO_PWM=%d UV=%d REF=%d "
                  "Dust=%d dustLED=%d Mic=%d NO2=%d DHT=%d Buzz=%d\n",
                  CO_ADC_PIN, CO_PWM_PIN, UVOUT, REF_3V3,
                  dustPin, dustLED, micPin, NO2_SENSOR_PIN, DHTPIN, buzzPin);
    Serial.printf("[INIT]       LED g=%d o=%d r=%d | "
                  "GPS RX=%d TX=%d | IMU RX=%d TX=%d\n",
                  ledPin11, ledPin12, ledPin13,
                  GPS_RX_PIN, GPS_TX_PIN, IMU_RX_PIN, IMU_TX_PIN);
  }

  engMsgf("Setup OK v1.2 — divider %.0f+%.0fkΩ scale=%.3f Vout@5V=%.3fV",
          R_DIVIDER_SERIES, R_DIVIDER_GND, divider_scale, vout_at_5v);

  xTaskCreatePinnedToCore(blynkTask,  "blynkTask",  8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 8192, NULL, 1, NULL, 1);
}

// loop() is intentionally empty — all work runs in FreeRTOS tasks.
void loop() { vTaskDelete(NULL); }
