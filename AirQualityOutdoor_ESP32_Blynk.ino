// ============================================================================
// Outdoor Air Quality Station — ESP32 Production Firmware v2.6
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
//   GPIO36 VP — CO sense AO (10k+10k divider from board A0) - Suitable for reading analog signals
//   GPIO39 VN — CO heater feedback A1 (10k+10k divider from board A1) - Suitable for reading analog signals
//   GPIO34    — UV OUT (ML8511, direct 3.3V)
//   GPIO35    — UV REF (ML8511, direct 3.3V)
//   GPIO32    — Dust AO (GP2Y1010, 10k+10k divider)
//   GPIO21    — I2C SDA (ENS160+AHT2x)
//   GPIO22    — I2C SCL (ENS160+AHT2x)
//   GPIO13    — GPS UART1 RX ← ATGM336H TX  [moved from 22 to free I2C SCL]
//   GPIO23    — GPS UART1 TX → ATGM336H RX
//   GPIO16    — [SUPERSEDED v2.4] was JY-901 UART2 RX — now free/unused
//   GPIO17    — [SUPERSEDED v2.4] was JY-901 UART2 TX — now free/unused
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
//               V34 ESP reset reason (diagnostic)
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
//
// ─── v2.3 CHANGES — GPS PPS + GPS/IMU INERTIAL NAVIGATION (DEAD RECKONING) ──
//   1) GPS 1PPS (pulse-per-second) output wired to GPIO5 (free since DHT21
//      was removed in v2.0) via a rising-edge interrupt. PPS gives a hard
//      diagnostic of whether the GPS has a real timing lock (a GPS can
//      output stale/repeated NMEA without a genuine fix; a clean ~1.000s
//      PPS train is a much stronger lock indicator). Reported on Blynk V26.
//
//   2) New GY-BMI160 6-DOF IMU (I2C, shares the ENS160/AHT2x bus on
//      GPIO21/22, address 0x69) driven with a minimal register-level driver
//      (no external library dependency — BMI160 registers are simple and
//      well documented, and a hand-rolled driver avoids adding another
//      library that could collide with the ADC/I2S drivers the way v2.1/2.2
//      had to work around). Provides raw gyro (dps) and accel (g) samples.
//
//   3) New function inertialNav() fuses GPS position (and, when available,
//      GPS-derived speed+course from RMC/VTG) with BMI160 gyro/accel using
//      a compact 4-state Extended Kalman Filter (state = East, North,
//      Speed, Heading — a standard "2D ground-vehicle EKF" pattern widely
//      used for exactly this GPS+MEMS-gyro dead-reckoning problem on
//      resource-constrained MCUs). Design rationale, taken from common
//      embedded GNSS/INS fusion practice:
//        • Position is NOT double-integrated from the accelerometer.
//          Double-integrating cheap MEMS accelerometer noise/bias is a
//          well-known way to get position error that grows with t^2 —
//          essentially unusable within seconds. Instead, velocity is
//          treated as a slowly-varying state that is corrected directly
//          by GPS (position deltas AND, when moving fast enough, GPS
//          Doppler speed) and held constant between fixes.
//        • Heading is propagated by integrating BMI160 gyro Z between GPS
//          fixes (classic gyro dead-reckoning) and corrected by GPS course
//          only when GPS speed exceeds INAV_MIN_COURSE_SPEED_MPS, because
//          GPS course-over-ground is meaningless / noisy near zero speed.
//        • Gyro bias is NOT estimated inside the main filter (that would
//          need it to be observable, which a position/speed/heading-only
//          GPS update does not cleanly provide). Instead it is calibrated
//          separately using a Zero-velocity Update (ZUPT): whenever the
//          accelerometer magnitude reads ~1g and the gyro reads ~0 dps for
//          a sustained period, the platform is known to be stationary, so
//          the live gyro Z reading during that window IS the bias — a
//          textbook robust technique for MEMS gyro bias tracking that is
//          far simpler and more reliable than trying to make the bias
//          observable inside the position/velocity/heading EKF.
//        • ZUPT also forces the velocity state back toward zero via a
//          tiny-variance pseudo-measurement, preventing the classic
//          "GPS jitter looks like slow drift" creep while parked.
//      All matrix operations are hand-written fixed-size (4x4 max) loops —
//      no dynamic allocation, no external linear-algebra library — so the
//      filter is cheap enough to run at the user-configurable output rate
//      (inav_update_hz, default 10 Hz) on Core 0 alongside every other
//      sensor read without measurably affecting loop timing.
//
//   4) GPS-loss state machine (independent of, and reported alongside, the
//      existing STATUS_GPS_FAULT bit which is unchanged):
//        NO_FIX_YET  — no GPS fix has ever been received since boot;
//                      inertialNav() is idle until an origin can be set.
//        GPS_FIX     — a fresh GPS fix was just used to correct the filter.
//        IMU_RECENT  — GPS fix missing, but fewer than inav_stale_after_
//                      misses "expected" fixes (based on the adaptively
//                      measured GPS update interval) have been missed.
//                      Interpolated position is trusted almost as much as
//                      a real fix.
//        IMU_STALE   — GPS has been missing longer than that, but still
//                      within inav_max_loss_minutes. Interpolation
//                      continues but accuracy is explicitly flagged as
//                      degrading.
//        LOST        — GPS missing beyond inav_max_loss_minutes.
//                      Interpolation is HALTED (position held, not
//                      advanced further) until GPS reacquires, exactly as
//                      requested, to avoid reporting arbitrarily-drifted
//                      dead-reckoned position after a long outage.
//      Engineering messages are emitted only on state TRANSITIONS (never
//      every loop) so they don't drown out other diagnostics.
//
//   5) New Blynk virtual pins (all additive — no existing pin renumbered):
//      V26 PPS locked (0/1)         V27 INAV state (0-4, see enum above)
//      V28 INAV latitude            V29 INAV longitude
//      V30 INAV altitude (m, held from last valid GPS fix — BMI160 has no
//          barometer, so altitude is intentionally NOT dead-reckoned)
//      V31 INAV speed (m/s)         V32 INAV heading (deg, 0=N, clockwise)
//      These are entirely separate from the existing raw-GPS V12/V13 —
//      V12/V13 still report exactly what they did before (raw GPS lat/lng,
//      snapped to 0 or offset when no fix, as originally implemented).
//
// ─── v2.4 CHANGES — BMI160 REPLACES JY-901 ENTIRELY [SUPERSEDES v2.3] ───────
//   JY-901/WT901 (UART2, GPIO16/17) has been removed completely. All earlier
//   references to it in this file (the v2.0 "KEPT" list, the v2.3 changelog
//   text saying "existing JY-901 ... is untouched", the old PIN MAP entries
//   for GPIO16/17) are now historical — BMI160 is the only IMU in the system.
//   GPIO16/GPIO17 are free again (no longer used for anything).
//
//   BMI160 now does double duty:
//     1) Still the sole inertial source for inertialNav() (unchanged EKF).
//     2) ALSO now feeds V8 Roll / V9 Pitch / V10 Yaw / V11 IMU temperature —
//        the job JY-901's onboard fusion used to do — via a lightweight
//        complementary filter computed right here, since raw BMI160 has no
//        onboard sensor fusion of its own:
//          • Roll/Pitch: classic 98/2 complementary filter — gyro-integrated
//            angle corrected toward the gravity-vector accelerometer angle
//            each cycle. Gravity gives an absolute, non-drifting reference
//            for these two axes, which is why a simple complementary filter
//            (rather than a full AHRS) is sufficient and cheap here.
//          • Yaw: BMI160 has no magnetometer, so gyro-only yaw would drift
//            unboundedly. Rather than duplicate a second yaw estimator,
//            V10 simply reports the SAME heading state (inavX[3]) already
//            being maintained and GPS-corrected inside inertialNav() — one
//            state, two consumers, no drift, no extra code.
//          • IMU temperature: read from BMI160's internal die-temperature
//            registers (0x20/0x21) — informational only, expect it to read
//            a few degrees above ambient due to self-heating.
//     STATUS_IMU_OK / STATUS_IMU_STUCK now reflect BMI160 I2C read health
//     (a failed/absent BMI160 read) instead of JY-901 UART frame timeout —
//     same two status bits, same meaning to anything consuming V20.
// ============================================================================
// v2.7 LONG-TERM RELIABILITY IMPROVEMENTS — additive, existing behaviour kept
//   • GPS PPS ISR now records only timing/event state; D2 LED control runs in
//     sensorTask so the LED reflects the normal application context as well as
//     PPS activity. Atomic event exchange prevents PPS events being lost.
//   • WiFi reconnect was changed to a non-blocking 20s association state
//     machine; the original blocking wifiConnect() helper remains available.
//   • sensorTask heartbeat + ESP task-watchdog subscription provide an actual
//     recovery path for a stalled sensor task. The existing optional 10-minute
//     auto-reset remains intact and disabled by default.
//   • Periodic heap/minimum-heap/largest-free-block health telemetry is emitted
//     once per minute for long-duration burn-in and memory-fragmentation checks.
//   • Existing sensor, GPS, INAV, CO, Blynk, map, display, buzzer, and status
//     logic is preserved; changes are limited to reliability/monitoring paths.
// ============================================================================
// v3.7 LONG-TERM RELIABILITY IMPROVEMENTS — additive, existing behaviour kept
//   • Added ESP32 reset-cause diagnostics at startup, including human-readable 
//     reset reason reporting.
//   • Added Blynk V34 diagnostic output for the ESP32 reset reason.
//   • Added explicit ESP32 task-watchdog configuration with a 10-second timeout 
//     instead of relying on the Arduino core default.
//   • Added explicit watchdog registration and feeding for the sensor task only 
//     after a complete sensor cycle.
//   • Added sensor-task heartbeat monitoring to detect a task that is alive but 
//     no longer completing its work cycle.
//   • Added asynchronous non-blocking buzzer control so buzzer tones no longer 
//     block sensor processing or Blynk/WiFi operation.
//   • Added an asynchronous buzzer pattern state machine for multi-beep alerts 
//     without using blocking delays.
//   • Added persistent sensor-health monitoring for the INMP441 microphone with 
//     fault and recovery reporting.
//   • Added persistent sensor-health monitoring for the ML8511 UV sensor with fault 
//     and recovery reporting.
//   • Added persistent sensor-health monitoring for the GP2Y1010 dust sensor with 
//     fault and recovery reporting.
//   • Added `SENSOR_UNAVAILABLE` handling for failed INMP441, UV, and dust sensors 
//     so invalid sensors do not provide misleading measurements.
//   • Added fault persistence thresholds to prevent a single bad sensor reading 
//     from generating a false alarm.
//   • Added automatic sensor-recovery reporting when a previously failed sensor 
//     starts providing valid data again.
//   • Ensured sensor-health faults are isolated so a malfunctioning sensor does not 
//     stop the sensor task or prevent the remaining sensors from operating.
//   • Preserved all existing sensor-specific fault detection, retry, watchdog, 
//     GPS, CO, Blynk, WiFi, display, LED, buzzer, and status logic.
// ============================================================================


#define BLYNK_HEARTBEAT 60

// v3.3 long-term reliability: Blynk v0.6.1 uses BLYNK_TIMEOUT_MS for the
// underlying WiFiClient read timeout.  The library performs blocking reads
// inside Blynk.run(), so the historical 6s default is too close to the ESP32
// task/interrupt watchdog window for an unattended device.  Keep the timeout
// at the library-supported minimum.  A handshake may therefore require more
// than one Blynk.run() call, but each individual call remains bounded and the
// Core 1 idle task can continue to be serviced between calls.
#define BLYNK_TIMEOUT_MS 1000UL

#include "secrets.h"
// secrets.h: #define WIFI_SSID / WIFI_PASS / BLYNK_AUTH / BLYNK_SERVER / BLYNK_PORT
#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <esp_mac.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <esp_heap_caps.h>
#include <TinyGPS++.h>
#include <Wire.h>
// ENS160: use ScioSense_ENS160 library (NOT Adafruit_ENS160 which does not exist)
// Install: https://github.com/sciosense/ENS160_driver  or via Library Manager
#include "ScioSense_ENS160.h"
#include <Adafruit_AHTX0.h>      // AHT20/AHT21 temp+hum — install: Adafruit AHTX0
#include <Adafruit_SSD1306.h>
#include <driver/i2s_std.h>      // ESP32 NEW standard I2S driver for INMP441 (v2.2)

// ─── PERFORMANCE / I2C DIAGNOSTICS — declarations must precede PPS ISR ───
// Internal counters/timers run continuously; V34-V50 are published only
// by the 10-second diagnostic pathway.
volatile uint32_t perfPpsLastUs = 0;
volatile uint32_t perfMaxLoopGapMs = 0;
volatile uint32_t perfMaxSensorCycleMs = 0;
volatile uint32_t perfMaxBlynkRunMs = 0;
volatile uint32_t perfMaxGpsFeedMs = 0;
volatile uint32_t perfMaxI2cPathMs = 0;
volatile uint32_t perfMaxFastSendMs = 0;
volatile uint32_t perfMaxSlowSendMs = 0;
volatile uint32_t perfMaxPpsLedResponseMs = 0;
volatile uint32_t perfLastI2cDevice = 0;
volatile uint32_t perfLastI2cOperation = 0;
volatile uint32_t perfI2cTimeoutCount = 0;
volatile uint32_t perfMaxEns160Ms = 0;
volatile uint32_t perfMaxAht21Ms = 0;
volatile uint32_t perfMaxBmi160Ms = 0;

static inline void perfRecordMax(volatile uint32_t& dst, uint32_t value)
{
  if (value > dst) dst = value;
}

static inline uint32_t perfPpsAgeMs()
{
  uint32_t lastUs = perfPpsLastUs;
  if (lastUs == 0) return 0;
  return (uint32_t)((micros() - lastUs) / 1000UL);
}

static inline void perfI2cBegin(uint32_t device, uint32_t operation)
{
  perfLastI2cDevice = device;
  perfLastI2cOperation = operation;
}

static inline void perfI2cTimeout()
{
  perfI2cTimeoutCount++;
}

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
// [SUPERSEDED v2.4] IMUsensorThere (JY-901 UART) removed. BMI160sensorThere
// below now gates the ONLY IMU in the system — both inertial nav AND the
// Roll/Pitch/Yaw/temp feeding V8-V11.
#define ENSsensorThere true    // ENS160 + AHT2x combo board
#define INMPsensorThere true   // INMP441 I2S microphone
#define BMI160sensorThere true // GY-BMI160 6-DOF IMU for inertial dead-reckoning (v2.3)
#define PPSsensorThere    true // GPS 1PPS lock-status interrupt (v2.3)

bool ten_mins_autoreset = false;

// ─── INERTIAL NAVIGATION — user configurable (v2.3) ─────────────────────────
float inav_update_hz          = 10.0f; // interpolation output frequency (Hz)
int   inav_stale_after_misses = 3;     // consecutive missed GPS fixes before "stale" state
int   inav_max_loss_minutes   = 5;     // stop interpolating after this many minutes without GPS

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

// v3.5: Capture TinyGPS++ location-update state before any normal GPS
// latitude/longitude access below. TinyGPS++ clears location.isUpdated()
// when lat()/lng() are read, so the raw-GPS reporting path must not consume
// the update flag before inertialNav() gets a chance to use the same fix.
bool gpsLocationUpdatedThisCycle = false;

// [SUPERSEDED v2.4] JY-901 UART2 (GPIO16/17) removed entirely — BMI160 (I2C,
// declared below near the ENS160/AHT2x section) is now the only IMU.
// GPIO16/GPIO17 are free.

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
#define PPS_PIN     5    // GPS 1PPS output → ESP32 GPIO5 (free since DHT21 removed in v2.0)
                          // 3.3V push-pull, direct wire, no divider needed
#define PPS_LED_PIN 2     // ESP-32 board D2 LED — flashes for 250ms on each GPS PPS
// Some ESP32 boards wire the onboard D2/GPIO2 LED active-LOW. The current
// station board uses that polarity, so LOW means LED ON and HIGH means OFF.
// Change to true only if your particular board has an active-HIGH D2 LED.
#define PPS_LED_ACTIVE_HIGH true

// ─── PPS ISR state (v2.3) ────────────────────────────────────────────────────
// Single-word volatiles are atomic on Xtensa — no critical section needed for
// the simple diagnostic reads in ppsIsLocked().
volatile unsigned long ppsLastMicros  = 0;
volatile unsigned long ppsIntervalUs  = 0;
volatile uint32_t      ppsPulseCount  = 0;
// PPS LED requests are handled by sensorTask, not directly by the ISR.
// This keeps the ISR limited to timestamp/counter/event work and also means
// the LED is a useful indication that the normal sensor application is alive.
volatile uint32_t      ppsLedPending = 0;
portMUX_TYPE ppsLedMux = portMUX_INITIALIZER_UNLOCKED;

// PPS LED timing is owned by sensorTask (Core 0).  No cross-core LED state is
// modified by the ISR beyond the atomic pending-event counter above.
// The LED uses an absolute OFF deadline so each PPS creates one fixed 250ms
// pulse and a later PPS cannot accidentally extend an already active pulse.
unsigned long ppsLedOffMillis = 0;
bool ppsLedIsOn = false;

// v3.6: PPS LED service runs in its own small FreeRTOS task.
// This keeps the 250ms LED pulse independent of the sensorTask timing.
// A slow I2C/sensor cycle can therefore never turn a 250ms PPS pulse into
// a continuously illuminated LED.
TaskHandle_t ppsLedTaskHandle = NULL;

// Keep the LED polarity in one place. GPIO2/D2 is a board LED output, and
// different ESP32 boards wire that LED with different polarities.
// Using a helper prevents the PPS state machine from accidentally assuming
// HIGH=ON when the physical LED is active-LOW.
void setPpsLed(bool on)
{
  digitalWrite(PPS_LED_PIN, PPS_LED_ACTIVE_HIGH ? (on ? HIGH : LOW)
                                                 : (on ? LOW : HIGH));
}

// Forward declarations for performance diagnostics used by the PPS path.
static inline void perfRecordMax(volatile uint32_t& dst, uint32_t value);
static inline uint32_t perfPpsAgeMs();

void IRAM_ATTR ppsISR()
{
  unsigned long now = micros();
  if (ppsLastMicros != 0) ppsIntervalUs = now - ppsLastMicros;
  ppsLastMicros = now;
  ppsPulseCount++;
  // Performance diagnostic: timestamp the actual PPS ISR event.
  perfPpsLastUs = now;

  // Do NOT call digitalWrite() here. The ISR only records the PPS event and
  // wakes the dedicated LED task. The LED operation itself remains in normal
  // application context.
  portENTER_CRITICAL_ISR(&ppsLedMux);
  ppsLedPending++;
  portEXIT_CRITICAL_ISR(&ppsLedMux);

  if (ppsLedTaskHandle != NULL) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(ppsLedTaskHandle, &higherPriorityTaskWoken);
    if (higherPriorityTaskWoken) portYIELD_FROM_ISR();
  }
}

// v3.6: Dedicated PPS LED task. Every electrical PPS event received on GPIO5
// produces one fixed 250ms LED pulse. The LED is OFF at all other times.
// This is intentionally independent of sensorTask so slow sensor/I2C work
// cannot stretch the LED pulse or make it appear continuously ON.
void ppsLedTask(void *parameter)
{
  (void)parameter;

  for (;;) {
    // Sleep until the GPIO5 PPS ISR reports an event.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    uint32_t pendingPps;
    portENTER_CRITICAL(&ppsLedMux);
    pendingPps = ppsLedPending;
    ppsLedPending = 0;
    portEXIT_CRITICAL(&ppsLedMux);

    if (pendingPps == 0) continue;

    // Performance diagnostic: measure scheduling latency from the PPS ISR
    // to the dedicated LED task. This does not affect the LED state machine.
    uint32_t perfPpsLedResponseStartUs = micros();
    uint32_t perfPpsEventUs = perfPpsLastUs;
    if (perfPpsEventUs != 0)
      perfRecordMax(perfMaxPpsLedResponseMs,
                    (uint32_t)((perfPpsLedResponseStartUs - perfPpsEventUs) / 1000UL));

    // One PPS event = one 250ms ON pulse.
    setPpsLed(true);
    ppsLedIsOn = true;
    ppsLedOffMillis = millis() + 250UL;

    vTaskDelay(pdMS_TO_TICKS(250));

    // Always switch OFF after the fixed pulse, regardless of PPS lock state.
    setPpsLed(false);
    ppsLedIsOn = false;

    // If multiple PPS events accumulated while the 250ms pulse was active,
    // service them one-by-one. Normal GPS PPS is one event per second, so
    // under normal operation this path is not normally used.
    while (pendingPps > 1) {
      pendingPps--;

      setPpsLed(true);
      ppsLedIsOn = true;
      ppsLedOffMillis = millis() + 250UL;

      vTaskDelay(pdMS_TO_TICKS(250));

      setPpsLed(false);
      ppsLedIsOn = false;
    }
  }
}

// Locked = a pulse arrived within the last 2s AND its interval was within
// ±10% of the expected 1.000s. Tolerant enough for jitter, tight enough to
// reject a GPS that is powered but not yet timing-locked.
bool ppsIsLocked()
{
  if (ppsLastMicros == 0) return false;
  unsigned long sinceLast = micros() - ppsLastMicros;  // unsigned sub handles wrap
  if (sinceLast > 2000000UL) return false;
  unsigned long iv = ppsIntervalUs;
  if (iv < 900000UL || iv > 1100000UL) return false;
  return true;
}

// ─── LEDC ────────────────────────────────────────────────────────────────────
#define LEDC_FREQ_CO 5000
#define LEDC_RES_CO  8

// ─── GY-BMI160 6-DOF IMU — I2C, shares bus with ENS160/AHT2x (v2.3) ─────────
// Minimal register-level driver — no external library, avoids adding another
// dependency that could collide with the ADC/I2S driver families (see v2.1/
// v2.2 notes above for why that risk is taken seriously in this project).
#define BMI160_ADDR         0x69   // 0x69 --- SDO/SA0 pulled HIGH on the GY-BMI160 board
#define BMI160_REG_CHIPID   0x00
#define BMI160_REG_DATA     0x0C   // GYR_X_L .. ACC_Z_H, 12 bytes contiguous
#define BMI160_REG_TEMP     0x20   // die temperature, 2 bytes (v2.4, feeds V11)
#define BMI160_REG_ACC_CONF  0x40
#define BMI160_REG_ACC_RANGE 0x41
#define BMI160_REG_GYR_CONF  0x42
#define BMI160_REG_GYR_RANGE 0x43
#define BMI160_REG_CMD       0x7E
#define BMI160_CMD_SOFTRESET   0xB6
#define BMI160_CMD_ACC_NORMAL  0x11
#define BMI160_CMD_GYR_NORMAL  0x15
#define BMI160_CHIPID_EXPECTED 0xD1
#define BMI160_ACC_RANGE_G     4.0f    // ±4g   (ACC_RANGE=0x05)
#define BMI160_GYR_RANGE_DPS   500.0f  // ±500dps (GYR_RANGE=0x02)

// ─── INERTIAL NAV — EKF tuning constants (v2.3) ─────────────────────────────
// State x = [E(m), N(m), v(m/s), heading(rad, 0=North, clockwise+)]
#define EARTH_RADIUS_M         6371000.0f
#define INAV_R_POS             25.0f     // GPS position meas. variance (m^2), ~5m std
#define INAV_R_SPEED           0.25f     // GPS speed meas. variance (m/s)^2, ~0.5 m/s std
#define INAV_R_HEADING         0.00762f  // GPS course meas. variance (rad^2), ~5deg std
#define INAV_Q_POS             0.02f     // process noise, position (m^2/predict step)
#define INAV_Q_SPEED           0.05f     // process noise, speed ((m/s)^2/predict step)
#define INAV_Q_HEADING         0.000306f // process noise, heading (rad^2/step), ~1deg std
#define INAV_ZUPT_ACCEL_TOL_G  0.03f     // |accel_mag - 1g| under this = "not accelerating"
#define INAV_ZUPT_GYRO_TOL_DPS 2.0f      // |gyro| under this = "not rotating"
#define INAV_ZUPT_DURATION_MS  500       // sustained stillness before ZUPT fires
#define INAV_ZUPT_R_V          0.01f     // tight variance on the zero-velocity pseudo-measurement
#define INAV_MIN_COURSE_SPEED_MPS 0.5f   // below this, GPS course is unreliable — skip heading update
#define INAV_STATE_GPS_FIX     0
#define INAV_STATE_IMU_RECENT  1
#define INAV_STATE_IMU_STALE   2
#define INAV_STATE_LOST        3
#define INAV_STATE_NO_FIX_YET  4

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
#define MAP_SEND_INTERVAL_MS  30000   // V33 Map widget point rate — deliberately slower
                                      // than FAST/SLOW telemetry to avoid flooding the
                                      // Blynk Map widget with a marker every few seconds
#define SENSOR_TASK_PERIOD_MS     25
#define GPS_FEED_MS               50
#define IMU_POLL_MS               50
#define BUZZER_GPS_INTERVAL     5000
#define VWRITE_GAP_MS            10

// ─── PERFORMANCE DIAGNOSTIC VIRTUAL PINS ───────────────────────────────────
// Internal timing/counters run continuously; these pins are published
// only once every 10 seconds by blynkSendDiagnostics().
#define V35 35  // PPS event count
#define V36 36  // milliseconds since last PPS
#define V37 37  // maximum Arduino loop gap (ms)
#define V38 38  // maximum sensor-task cycle duration (ms)
#define V39 39  // maximum Blynk.run() duration (ms)
#define V40 40  // maximum GPS feed duration (ms)
#define V41 41  // maximum I2C sensor-path duration (ms)
#define V42 42  // maximum FAST telemetry duration (ms)
#define V43 43  // maximum SLOW telemetry duration (ms)
#define V44 44  // maximum PPS ISR-to-LED-task response (ms)
#define V45 45  // last I2C device ID (0=none, 1=ENS160, 2=AHT21, 3=BMI160)
#define V46 46  // last I2C operation ID (0=none, 1=read, 2=measurement/command, 3=configuration)
#define V47 47  // cumulative I2C transaction error count
#define V48 48  // maximum AHT21 read duration (ms)
#define V49 49  // maximum ENS160 operation/read duration (ms)
#define V50 50  // maximum BMI160 raw-read duration (ms)

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
  long     gps_unix_time;     // v2.5: UNIX epoch seconds from GPS UTC date/time, used as
                              // the Blynk Map (V33) point index — 0 if GPS date/time invalid
  int32_t  rssi;
  uint8_t  wifi_qual;
  uint32_t status_flags;
  char     eng_msg[128];
  int      rand_num;
  // v2.3 additions — additive only, nothing above this line changed
  bool     pps_locked;
  int      inav_state;                           // see INAV_STATE_* defines
  double   inav_lat, inav_lng;                   // EKF-fused position
  float    inav_alt;                             // held from last valid GPS fix (no baro on BMI160)
  float    inav_speed_mps, inav_heading_deg;     // EKF-fused speed/heading
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

// ─── IMU — BMI160 fusion state (v2.4, replaces JY-901) ──────────────────────
// angle[3] = Roll, Pitch, Yaw (deg). Roll/Pitch from complementary filter,
// Yaw mirrors the EKF heading state (inavX[3]) — see inertialNav().
// imuT = BMI160 die temperature (°C).
float         angle[3]         = {0,0,0};
float         imuT             = 0;
unsigned long lastBmiOkMillis  = 0;   // last successful BMI160 read, for stuck detection
unsigned long lastFusionMillis = 0;   // for complementary filter dt

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

// ─── RESET-CAUSE DIAGNOSTICS ────────────────────────────────────────────────
// Additive diagnostic only: records why the ESP32 last restarted.
esp_reset_reason_t bootResetReason = ESP_RST_UNKNOWN;
const char* bootResetReasonText = "UNKNOWN";

// ─── SENSOR HEALTH DIAGNOSTICS — additive reliability layer ───────────────────
// These counters only detect persistent invalid/no-data conditions. They never
// stop sensorTask and never prevent the other sensors from being serviced.
// Existing sensor-specific fault/retry logic remains unchanged.
unsigned int inmpFaultCount = 0;
unsigned int uvFaultCount   = 0;
unsigned int dustFaultCount = 0;
bool inmpFaultReported = false;
bool uvFaultReported   = false;
bool dustFaultReported = false;
bool inmpLastReadHadData = false;
unsigned long lastInmpHealthMsg = 0;
unsigned long lastUvHealthMsg   = 0;
unsigned long lastDustHealthMsg = 0;

#define SENSOR_HEALTH_FAIL_LIMIT 5
#define SENSOR_HEALTH_MSG_INTERVAL 60000UL

// Report a persistent fault without ever stopping the sensor task. The existing
// engineering-message path (Blynk V19) is deliberately reused so no new Blynk
// data channel is required and no existing virtual-pin mapping changes.
void sensorHealthMsg(const char* sensor, const char* reason, unsigned long& lastMsg)
{
  unsigned long now = millis();
  if (now - lastMsg >= SENSOR_HEALTH_MSG_INTERVAL || lastMsg == 0) {
    lastMsg = now;
    engMsgf("%s FAULT: %s — other sensors continue", sensor, reason);
  }
}

// ─── ASYNCHRONOUS BUZZER STATE ───────────────────────────────────────────────
// Additive reliability change: buzzerTone() starts a tone and returns immediately.
// buzzerService() advances the tone without blocking either FreeRTOS task.
volatile bool buzzerActive = false;
volatile uint16_t buzzerActiveFrequency = 0;
volatile unsigned long buzzerStopMillis = 0;

#define BUZZER_PATTERN_MAX_STEPS 3
struct BuzzerPatternStep {
  uint16_t frequency;
  uint16_t durationMs;
  uint16_t pauseMs;
};
BuzzerPatternStep buzzerPattern[BUZZER_PATTERN_MAX_STEPS];
volatile uint8_t buzzerPatternCount = 0;
volatile uint8_t buzzerPatternIndex = 0;
volatile bool buzzerPatternRunning = false;
volatile unsigned long buzzerPatternNextMillis = 0;

// ─── LONG-TERM HEALTH MONITORING ─────────────────────────────────────────────
// Sensor-task heartbeat is updated once per completed sensor cycle.  The
// application watchdog below uses it as an additional diagnostic, while the
// ESP task watchdog protects against a genuinely stuck sensor task.
volatile uint32_t sensorTaskHeartbeat = 0;
unsigned long lastHealthReport = 0;
unsigned long lastHeartbeatChangeMillis = 0;
uint32_t lastHealthHeartbeat = 0;

// ─── PERFORMANCE DIAGNOSTICS — internal counters/timers ─────────────────────
// These values run continuously but are published to Blynk only every 10s.
// They are diagnostic-only and never trigger a reset or alter normal control.
// ─── ADDITIONAL I2C DIAGNOSTICS — internal state ────────────────────────────
// These values run continuously. V45-V50 are published only by the existing
// 10-second diagnostic pathway, so the diagnostic channel remains low-overhead.
// Device IDs: 0=none, 1=ENS160, 2=AHT21, 3=BMI160.
// Operation IDs: 0=none, 1=read, 2=measurement/command, 3=configuration.
volatile uint32_t perfI2cErrorCount = 0;

static inline void perfI2cError()
{
  perfI2cErrorCount++;
}




void perfReport()
{
  Serial.printf(
    "[PERF] PPS=%lu age=%lums loopMax=%lums sensorMax=%lums "
    "BlynkRunMax=%lums GPSmax=%lums I2Cmax=%lums FASTmax=%lums "
    "SLOWmax=%lums PPSledMax=%lums I2Cdev=%lu I2Cop=%lu "
    "I2Cerrors=%lu AHTmax=%lums ENSmax=%lums BMImax=%lums\n",
    (unsigned long)ppsPulseCount,
    (unsigned long)perfPpsAgeMs(),
    (unsigned long)perfMaxLoopGapMs,
    (unsigned long)perfMaxSensorCycleMs,
    (unsigned long)perfMaxBlynkRunMs,
    (unsigned long)perfMaxGpsFeedMs,
    (unsigned long)perfMaxI2cPathMs,
    (unsigned long)perfMaxFastSendMs,
    (unsigned long)perfMaxSlowSendMs,
    (unsigned long)perfMaxPpsLedResponseMs,
    (unsigned long)perfLastI2cDevice,
    (unsigned long)perfLastI2cOperation,
    (unsigned long)perfI2cErrorCount,
    (unsigned long)perfMaxAht21Ms,
    (unsigned long)perfMaxEns160Ms,
    (unsigned long)perfMaxBmi160Ms
  );
}


#define SENSOR_TASK_HEALTH_TIMEOUT_MS  30000UL
#define HEALTH_REPORT_INTERVAL_MS      60000UL

// ─── INERTIAL NAV — EKF state (v2.3) ────────────────────────────────────────
float         inavX[4]    = {0,0,0,0};  // [E(m), N(m), v(m/s), heading(rad)]
float         inavP[4][4] = {{0}};      // covariance, initialised on first GPS fix
float         inavGyroBiasDps = 0.0f;   // gyro Z bias, calibrated via ZUPT
unsigned long zuptStartMillis = 0;
float         inavLastAlt = SENSOR_UNAVAILABLE;
bool          bmi160Ready = false;

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

const char* resetReasonToString(esp_reset_reason_t reason)
{
  switch (reason) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXTERNAL";
    case ESP_RST_SW:        return "SOFTWARE";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "OTHER";
  }
}

void buzzerTone(uint16_t frequency, uint16_t durationMs)
{
  if (!frequency || !durationMs) return;
  // Non-blocking: only schedule the tone. buzzerService() performs the actual
  // LEDC stop later, so no sensor/Blynk task is held up by a beep.
  buzzerPatternRunning = false;
  buzzerPatternCount = 0;
  buzzerPatternIndex = 0;
  ledcAttach(buzzPin, frequency, 8);
  ledcWrite(buzzPin, 128);
  buzzerActiveFrequency = frequency;
  buzzerActive = true;
  buzzerStopMillis = millis() + durationMs;
}

void buzzerPattern3(uint16_t frequency, uint16_t durationMs, uint16_t pauseMs)
{
  if (!frequency || !durationMs) return;
  buzzerPattern[0] = {frequency, durationMs, pauseMs};
  buzzerPattern[1] = {frequency, durationMs, pauseMs};
  buzzerPattern[2] = {frequency, durationMs, 0};
  buzzerPatternCount = 3;
  buzzerPatternIndex = 0;
  buzzerPatternRunning = true;
  buzzerActive = false;
  buzzerPatternNextMillis = millis();
}

void buzzerService()
{
  unsigned long now = millis();

  if (buzzerActive) {
    if ((long)(now - buzzerStopMillis) >= 0) {
      ledcWrite(buzzPin, 0);
      ledcDetach(buzzPin);
      buzzerActive = false;
      if (buzzerPatternRunning) {
        buzzerPatternNextMillis = now + buzzerPattern[buzzerPatternIndex].pauseMs;
      }
    }
    return;
  }

  if (buzzerPatternRunning && (long)(now - buzzerPatternNextMillis) >= 0) {
    BuzzerPatternStep step = buzzerPattern[buzzerPatternIndex];
    ledcAttach(buzzPin, step.frequency, 8);
    ledcWrite(buzzPin, 128);
    buzzerActiveFrequency = step.frequency;
    buzzerActive = true;
    buzzerStopMillis = now + step.durationMs;
    buzzerPatternIndex++;
    if (buzzerPatternIndex >= buzzerPatternCount) {
      buzzerPatternRunning = false;
    }
  }
}

void safeWrite(int vpin, double val)       { Blynk.virtualWrite(vpin, val); delay(VWRITE_GAP_MS); }
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
  uint32_t perfGpsStartUs = micros();

  unsigned long t = millis();
  while (millis() - t < GPS_FEED_MS) {
    while (gps_serial.available()) gps.encode(gps_serial.read());
    taskYIELD();
  }

  // GPS DEBUG ONLY — no effect on GPS processing or normal operation.
  if (DEBUGON) {
    static unsigned long lastGPSDebug = 0;
    if (millis() - lastGPSDebug >= 5000) {
      lastGPSDebug = millis();
      Serial.printf("[GPS DEBUG] chars=%lu sats=%lu hdop=%.2f location=%s age=%lu ms\n",
                    gps.charsProcessed(),
                    gps.satellites.value(),
                    gps.hdop.value()/100.0f,
                    gps.location.isValid() ? "VALID" : "INVALID",
                    gps.location.age());
      Serial.printf("[GPS DEBUG] lat=%.6f lng=%.6f fix=%s PPS=%d\n",
                    gps.location.isValid() ? gps.location.lat() : 0.0,
                    gps.location.isValid() ? gps.location.lng() : 0.0,
                    (gps.location.isValid() &&
                     gps.location.age() < GPS_TIMEOUT_MS &&
                     gps.satellites.value() >= 4) ? "VALID" : "INVALID",
                    ppsIsLocked() ? 1 : 0);
    }
  }

  perfRecordMax(perfMaxGpsFeedMs,
                (uint32_t)((micros() - perfGpsStartUs) / 1000UL));
}

// v2.5: Convert GPS UTC date+time (from NMEA RMC/GGA, already parsed by
// TinyGPS++) to a UNIX epoch timestamp — used as the point index for the
// Blynk Map widget on V33. Standard proleptic-Gregorian "days_from_civil"
// calendar math (Howard Hinnant's well-known constexpr algorithm); no RTC,
// no NTP, no external library needed — the GPS module is already an
// accurate UTC time source once it has a fix.
//
// IMPORTANT: this reads the global `gps` (TinyGPSPlus) object directly, so
// it must ONLY ever be called from sensorTask on Core 0 — the same core
// that owns and writes to `gps` via feedGPS(). Calling it from loop()/Core 1
// would be an unprotected cross-core access to a non-thread-safe object.
// The result is cached into sd.gps_unix_time (mutex-protected) so Core 1
// can read it safely without ever touching `gps` itself.
long gpsUnixTime()
{
  if (!gps.date.isValid() || !gps.time.isValid()) return 0;
  int y  = gps.date.year();
  int m  = gps.date.month();
  int d  = gps.date.day();
  int hh = gps.time.hour();
  int mm = gps.time.minute();
  int ss = gps.time.second();
  if (y < 2020 || y > 2100) return 0;   // sanity guard against a bad/uninitialised NMEA fix

  y -= (m <= 2) ? 1 : 0;
  long era = (y >= 0 ? y : y - 399) / 400;
  long yoe = y - era * 400;
  long doy = (153*(m + (m > 2 ? -3 : 9)) + 2)/5 + d - 1;
  long doe = yoe*365 + yoe/4 - yoe/100 + doy;
  long days = era*146097 + doe - 719468;
  return days*86400L + (long)hh*3600L + (long)mm*60L + (long)ss;
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
  uint32_t perfEnsStartUs = micros();

  // Read AHT2x first — provides compensation values for ENS160
  sensors_event_t humEvent, tempEvent;
  uint32_t perfAht21StartUs = micros();
  perfI2cBegin(2, 1);
  bool perfAht21Ok = aht.getEvent(&humEvent, &tempEvent);
  perfRecordMax(perfMaxAht21Ms,
                (uint32_t)((micros() - perfAht21StartUs) / 1000UL));
  if (!perfAht21Ok) {
    perfRecordMax(perfMaxI2cPathMs,
                  (uint32_t)((micros() - perfEnsStartUs) / 1000UL));
    return false;
  }
  temp_out = tempEvent.temperature;
  hum_out  = humEvent.relative_humidity;

  // Provide integer temp/humidity compensation to ENS160 for improved accuracy
  // ScioSense set_envdata() takes int (°C) and int (%) — cast from float
  uint32_t perfEns160OnlyStartUs = micros();
  perfI2cBegin(1, 3);
  ens160.set_envdata((int)temp_out, (int)hum_out);

  // Trigger measurement
  perfI2cBegin(1, 2);
  ens160.measure(true);
  ens160.measureRaw(true);

  // Read results — always available after measure()
  perfI2cBegin(1, 1);
  tvoc    = (float)ens160.getTVOC();   // ppb
  eco2    = (float)ens160.geteCO2();   // ppm equivalent CO2
  aqi_out = (float)ens160.getAQI();    // 1–5 index
  perfRecordMax(perfMaxEns160Ms,
                (uint32_t)((micros() - perfEns160OnlyStartUs) / 1000UL));

  perfRecordMax(perfMaxI2cPathMs,


                (uint32_t)((micros() - perfEnsStartUs) / 1000UL));


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
  inmpLastReadHadData = false;
  // v2.2: i2s_channel_read() replaces legacy i2s_read(I2S_PORT, ...).
  // Same blocking-with-timeout semantics as the legacy call.
  i2s_channel_read(inmp441_rx_handle, &samples, sizeof(samples), &bytes_read, pdMS_TO_TICKS(50));
  int count = bytes_read / sizeof(int32_t);
  if (count == 0) return SENSOR_UNAVAILABLE;
  inmpLastReadHadData = true;

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
// GY-BMI160 6-DOF IMU — minimal I2C register-level driver
// [SUPERSEDED v2.4] JY-901 (UART, pre-fused Roll/Pitch/Yaw) has been removed
// entirely. BMI160 is now the ONLY IMU — its raw gyro/accel feed both
// inertialNav() (EKF dead-reckoning) AND, via a complementary filter inside
// inertialNav() itself, the Roll/Pitch/Yaw/temp values sent to V8-V11.
// feeding V8-V11 exactly as before.
// ============================================================================

bool bmi160WriteReg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.write(val);
  uint8_t err = Wire.endTransmission();
  if (err != 0) perfI2cError();
  return (err == 0);
}

bool bmi160ReadRegs(uint8_t reg, uint8_t* buf, uint8_t len)
{
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { perfI2cError(); return false; }   // repeated start
  uint8_t got = Wire.requestFrom((int)BMI160_ADDR, (int)len);
  if (got != len) { perfI2cError(); return false; }
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool initBMI160()
{
  uint8_t chipId = 0;
  if (!bmi160ReadRegs(BMI160_REG_CHIPID, &chipId, 1) || chipId != BMI160_CHIPID_EXPECTED) {
    engMsgf("BMI160: chip ID 0x%02X (expected 0x%02X) — check wiring/address",
            chipId, BMI160_CHIPID_EXPECTED);
    return false;
  }
  bmi160WriteReg(BMI160_REG_CMD, BMI160_CMD_SOFTRESET);
  delay(50);
  // ODR=100Hz (0x08) | bandwidth=normal (0x02<<4=0x20) = 0x28, for both acc & gyro
  bmi160WriteReg(BMI160_REG_ACC_CONF, 0x28);
  bmi160WriteReg(BMI160_REG_ACC_RANGE, 0x05);   // ±4g
  bmi160WriteReg(BMI160_REG_GYR_CONF, 0x28);
  bmi160WriteReg(BMI160_REG_GYR_RANGE, 0x02);   // ±500 dps
  bmi160WriteReg(BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL);
  delay(50);
  bmi160WriteReg(BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL);
  delay(100);
  engMsg("BMI160: init OK (±4g, ±500dps, 100Hz)");
  return true;
}

// Returns physical units: gx/gy/gz in dps, ax/ay/az in g.
bool bmi160ReadRaw(float& gx, float& gy, float& gz, float& ax, float& ay, float& az)
{
  uint32_t perfBmi160RawStartUs = micros();
  perfI2cBegin(3, 1);
  uint8_t buf[12];
  if (!bmi160ReadRegs(BMI160_REG_DATA, buf, 12)) {
    perfI2cError();
    perfRecordMax(perfMaxBmi160Ms,
                  (uint32_t)((micros() - perfBmi160RawStartUs) / 1000UL));
    return false;
  }
  int16_t rgx = (int16_t)((buf[1]<<8)|buf[0]);
  int16_t rgy = (int16_t)((buf[3]<<8)|buf[2]);
  int16_t rgz = (int16_t)((buf[5]<<8)|buf[4]);
  int16_t rax = (int16_t)((buf[7]<<8)|buf[6]);
  int16_t ray = (int16_t)((buf[9]<<8)|buf[8]);
  int16_t raz = (int16_t)((buf[11]<<8)|buf[10]);
  gx = rgx / 32768.0f * BMI160_GYR_RANGE_DPS;
  gy = rgy / 32768.0f * BMI160_GYR_RANGE_DPS;
  gz = rgz / 32768.0f * BMI160_GYR_RANGE_DPS;
  ax = rax / 32768.0f * BMI160_ACC_RANGE_G;
  ay = ray / 32768.0f * BMI160_ACC_RANGE_G;
  az = raz / 32768.0f * BMI160_ACC_RANGE_G;

  if (DEBUGON) {
    static unsigned long lastGPSDebug = 0;
    if (millis() - lastGPSDebug >= 5000) {
      lastGPSDebug = millis();
      Serial.printf("[IMU DEBUG] gx=%.1f gy=%.1f gz=%.1f | ax=%.1f ay=%.1f az=%.1f \n",
                      gx,
                      gy,
                      gz,
                      ax,
                      ay,
                      az);
    }
  }

  perfRecordMax(perfMaxBmi160Ms,
                (uint32_t)((micros() - perfBmi160RawStartUs) / 1000UL));
  return true;
}

// ============================================================================
// INERTIAL NAVIGATION — GPS + BMI160 4-state EKF (v2.3)
// See the v2.3 CHANGES header comment block at the top of this file for the
// full design rationale (why velocity isn't double-integrated from the
// accelerometer, why heading uses gyro dead-reckoning + GPS-course
// correction only above INAV_MIN_COURSE_SPEED_MPS, and why gyro bias is
// calibrated via ZUPT instead of being a filter state).
// ============================================================================

float inavWrapAngle(float a)
{
  while (a >  (float)M_PI) a -= 2.0f*(float)M_PI;
  while (a < -(float)M_PI) a += 2.0f*(float)M_PI;
  return a;
}

// General 4x4 matrix inverse via Gauss-Jordan with partial pivoting.
// Only used by the "full" GPS update (position+speed+heading, H=Identity),
// which fires at most a few times per second — cost is negligible.
bool mat4Inverse(const float M[4][4], float Inv[4][4])
{
  float A[4][8];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) A[i][j] = M[i][j];
    for (int j = 0; j < 4; j++) A[i][4+j] = (i==j) ? 1.0f : 0.0f;
  }
  for (int col = 0; col < 4; col++) {
    int piv = col;
    float maxAbs = fabsf(A[col][col]);
    for (int r = col+1; r < 4; r++) {
      if (fabsf(A[r][col]) > maxAbs) { maxAbs = fabsf(A[r][col]); piv = r; }
    }
    if (maxAbs < 1e-9f) return false;   // singular — caller should skip this update
    if (piv != col) {
      for (int k = 0; k < 8; k++) { float tmp=A[col][k]; A[col][k]=A[piv][k]; A[piv][k]=tmp; }
    }
    float d = A[col][col];
    for (int k = 0; k < 8; k++) A[col][k] /= d;
    for (int r = 0; r < 4; r++) {
      if (r == col) continue;
      float f = A[r][col];
      if (f == 0.0f) continue;
      for (int k = 0; k < 8; k++) A[r][k] -= f*A[col][k];
    }
  }
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      Inv[i][j] = A[i][4+j];
  return true;
}

// Prediction step — nonlinear unicycle model, linearised via Jacobian F.
// dt in seconds. gyroZ_dps is the RAW BMI160 gyro Z reading; bias is
// subtracted here using the ZUPT-calibrated inavGyroBiasDps.
void inavPredict(float dt, float gyroZ_dps)
{
  float yawRate = (gyroZ_dps - inavGyroBiasDps) * DEG_TO_RAD;  // rad/s
  float psi = inavX[3];
  float v   = inavX[2];
  float s = sinf(psi), c = cosf(psi);

  inavX[0] += v * s * dt;
  inavX[1] += v * c * dt;
  inavX[3]  = inavWrapAngle(psi + yawRate * dt);
  // v (inavX[2]) held constant here — corrected only by GPS or ZUPT.

  float F[4][4] = {
    {1, 0,  s*dt,   v*c*dt},
    {0, 1,  c*dt,  -v*s*dt},
    {0, 0,  1,      0     },
    {0, 0,  0,      1     }
  };

  float FP[4][4];
  for (int i=0;i<4;i++) for (int j=0;j<4;j++) {
    float acc=0; for (int k=0;k<4;k++) acc += F[i][k]*inavP[k][j];
    FP[i][j]=acc;
  }
  float FPFt[4][4];
  for (int i=0;i<4;i++) for (int j=0;j<4;j++) {
    float acc=0; for (int k=0;k<4;k++) acc += FP[i][k]*F[j][k];  // F^T[k][j]=F[j][k]
    FPFt[i][j]=acc;
  }
  static const float Q[4] = {INAV_Q_POS, INAV_Q_POS, INAV_Q_SPEED, INAV_Q_HEADING};
  for (int i=0;i<4;i++) for (int j=0;j<4;j++)
    inavP[i][j] = FPFt[i][j] + ((i==j) ? Q[i] : 0.0f);
}

// GPS update — position only (H = [[1,0,0,0],[0,1,0,0]]). Hand-expanded
// 2-row special case instead of a generic H matrix — cheaper and clearer.
void inavKalmanUpdatePos(float zE, float zN, float rPos)
{
  float yE = zE - inavX[0];
  float yN = zN - inavX[1];
  float S00 = inavP[0][0] + rPos, S01 = inavP[0][1];
  float S10 = inavP[1][0],        S11 = inavP[1][1] + rPos;
  float det = S00*S11 - S01*S10;
  if (fabsf(det) < 1e-9f) return;   // singular — skip this update
  float invDet = 1.0f/det;
  float Si00 =  S11*invDet, Si01 = -S01*invDet;
  float Si10 = -S10*invDet, Si11 =  S00*invDet;

  float K[4][2];
  for (int i=0;i<4;i++) {
    float PHt0 = inavP[i][0], PHt1 = inavP[i][1];
    K[i][0] = PHt0*Si00 + PHt1*Si10;
    K[i][1] = PHt0*Si01 + PHt1*Si11;
  }
  for (int i=0;i<4;i++) inavX[i] += K[i][0]*yE + K[i][1]*yN;
  inavX[3] = inavWrapAngle(inavX[3]);

  float HP0[4], HP1[4];
  for (int j=0;j<4;j++) { HP0[j]=inavP[0][j]; HP1[j]=inavP[1][j]; }
  for (int i=0;i<4;i++) for (int j=0;j<4;j++)
    inavP[i][j] -= K[i][0]*HP0[j] + K[i][1]*HP1[j];
}

// GPS update — full observation (H = Identity4), used only when GPS speed
// and course are both valid AND speed exceeds INAV_MIN_COURSE_SPEED_MPS.
void inavKalmanUpdateFull(float zE, float zN, float zSpeed, float zHeadingRad,
                           float rPos, float rSpeed, float rHeading)
{
  float y[4] = { zE - inavX[0], zN - inavX[1], zSpeed - inavX[2],
                 inavWrapAngle(zHeadingRad - inavX[3]) };
  float S[4][4];
  for (int i=0;i<4;i++) for (int j=0;j<4;j++) S[i][j] = inavP[i][j];
  S[0][0]+=rPos; S[1][1]+=rPos; S[2][2]+=rSpeed; S[3][3]+=rHeading;

  float Sinv[4][4];
  if (!mat4Inverse(S, Sinv)) return;   // singular — skip this update

  float K[4][4];
  for (int i=0;i<4;i++) for (int j=0;j<4;j++) {
    float acc=0; for (int k=0;k<4;k++) acc += inavP[i][k]*Sinv[k][j];
    K[i][j]=acc;
  }
  for (int i=0;i<4;i++) {
    float dx=0; for (int j=0;j<4;j++) dx += K[i][j]*y[j];
    inavX[i] += dx;
  }
  inavX[3] = inavWrapAngle(inavX[3]);

  float KP[4][4];
  for (int i=0;i<4;i++) for (int j=0;j<4;j++) {
    float acc=0; for (int k=0;k<4;k++) acc += K[i][k]*inavP[k][j];
    KP[i][j]=acc;
  }
  for (int i=0;i<4;i++) for (int j=0;j<4;j++) inavP[i][j] -= KP[i][j];
}

// Zero-velocity update (ZUPT) — the standard robust technique for keeping a
// dead-reckoning filter from drifting while stationary, and for calibrating
// gyro bias without needing it to be a filter state (see header rationale).
void inavZUPT(float ax_g, float ay_g, float az_g, float gyroZ_dps)
{
  float accelMag = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  bool stationary = (fabsf(accelMag - 1.0f) < INAV_ZUPT_ACCEL_TOL_G) &&
                     (fabsf(gyroZ_dps) < INAV_ZUPT_GYRO_TOL_DPS);
  unsigned long now = millis();
  if (!stationary) { zuptStartMillis = 0; return; }
  if (zuptStartMillis == 0) { zuptStartMillis = now; return; }
  if (now - zuptStartMillis < INAV_ZUPT_DURATION_MS) return;

  // Gyro bias: slow EMA toward the current (true-rate-is-zero) reading.
  inavGyroBiasDps = 0.98f*inavGyroBiasDps + 0.02f*gyroZ_dps;

  // Zero-velocity pseudo-measurement — 1-D Kalman update on state[2] only.
  float y = 0.0f - inavX[2];
  float S = inavP[2][2] + INAV_ZUPT_R_V;
  if (S <= 1e-9f) return;
  float K[4];
  for (int i=0;i<4;i++) K[i] = inavP[i][2]/S;
  for (int i=0;i<4;i++) inavX[i] += K[i]*y;
  float P2row[4]; for (int j=0;j<4;j++) P2row[j]=inavP[2][j];
  for (int i=0;i<4;i++) for (int j=0;j<4;j++) inavP[i][j] -= K[i]*P2row[j];
}

// Master entry point — called once per sensorTask loop iteration. Internally
// rate-limits its own prediction step to inav_update_hz; GPS updates run
// whenever TinyGPS++ reports a fresh location this cycle (no rate limiting
// needed there — GPS naturally updates at its own, much lower, rate).
void inertialNav()
{
  uint32_t perfInavStartUs = micros();

  static bool          originSet            = false;
  static double        lat0 = 0, lon0 = 0;
  static float         cosLat0              = 1.0f;
  static int           lastInavState        = -1;
  static unsigned long lastGoodFixMillis    = 0;
  static unsigned long gpsFixIntervalMsLoc  = 1000;  // adaptive nominal GPS interval
  static unsigned long lastPredictMillis    = 0;
  static unsigned long lastBmiRetryMillis   = 0;
  static bool          firstBmiAttempt      = true;   // bypass the 10s gate on the very first call

  if (!bmi160Ready) {
    if (firstBmiAttempt || millis() - lastBmiRetryMillis > 10000) {
      firstBmiAttempt = false;
      lastBmiRetryMillis = millis();
      bmi160Ready = initBMI160();
    }
    if (!bmi160Ready) perfRecordMax(perfMaxI2cPathMs,
               (uint32_t)((micros() - perfInavStartUs) / 1000UL));
 return;   // device fully absent — nothing to do this cycle
  }

  float gx=0,gy=0,gz=0,ax=0,ay=0,az=0;
  bool bmiOk = bmi160ReadRaw(gx,gy,gz,ax,ay,az);
  if (bmiOk) lastBmiOkMillis = millis();
  // Unlike a hard I2C absence, a single failed read no longer aborts the whole
  // cycle — GPS/EKF processing below still runs (gz=0 is a safe one-cycle
  // fallback for the predict step). "Stuck" means sustained failure.
  bool imuStuck = (lastBmiOkMillis > 0 && millis() - lastBmiOkMillis > IMU_TIMEOUT_MS);

  if (bmiOk) {
    inavZUPT(ax,ay,az,gz);

    // ── Roll/Pitch complementary filter (v2.4, replaces JY-901 fusion) ──────
    // Gravity-vector accel angle blended with gyro-integrated angle at 98/2.
    // Gravity gives an absolute, non-drifting reference for these two axes,
    // so a simple complementary filter (not a full AHRS) is sufficient here.
    unsigned long nowFusion = millis();
    float dtFusion = (lastFusionMillis == 0) ? 0.0f
                    : constrain((nowFusion - lastFusionMillis) / 1000.0f, 0.0f, 0.5f);
    lastFusionMillis = nowFusion;

    float rollAcc  = atan2f(ay, az) * RAD_TO_DEG;
    float pitchAcc = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;
    const float ALPHA = 0.98f;
    angle[0] = ALPHA * (angle[0] + gx*dtFusion) + (1.0f-ALPHA) * rollAcc;   // Roll
    angle[1] = ALPHA * (angle[1] + gy*dtFusion) + (1.0f-ALPHA) * pitchAcc;  // Pitch
    // angle[2] (Yaw) is set further below, from the EKF heading state —
    // BMI160 has no magnetometer, so free-integrating yaw here would drift
    // unboundedly; the EKF heading is already GPS-corrected, so we reuse it.

    // BMI160 die temperature (registers 0x20/0x21) — informational, feeds V11
    uint8_t tbuf[2];
    if (bmi160ReadRegs(BMI160_REG_TEMP, tbuf, 2)) {
      int16_t rawT = (int16_t)((tbuf[1]<<8) | tbuf[0]);
      imuT = (rawT == (int16_t)0x8000) ? SENSOR_UNAVAILABLE : (23.0f + rawT/512.0f);
    }
  }

  // v3.5: feedGPS/raw-GPS processing may already have read lat()/lng(),
  // which clears TinyGPS++'s location.isUpdated() flag. Use the snapshot
  // captured immediately after feedGPS() so INAV never misses a fresh fix.
  bool newFix = gpsLocationUpdatedThisCycle;
  int state;

  if (newFix && gps.location.isValid()) {
    double glat = gps.location.lat();
    double glon = gps.location.lng();

    if (!originSet) {
      lat0 = glat; lon0 = glon; cosLat0 = cosf((float)lat0 * DEG_TO_RAD);
      inavX[0]=0; inavX[1]=0; inavX[2]=0; inavX[3]=0;
      memset(inavP, 0, sizeof(inavP));
      inavP[0][0]=inavP[1][1]=100.0f;      // ~10m initial position uncertainty
      inavP[2][2]=4.0f;                     // ~2 m/s initial speed uncertainty
      inavP[3][3]=(float)(M_PI*M_PI);       // heading fully unknown initially
      originSet = true;
      engMsg("INAV: origin set from first GPS fix");
    }

    if (lastGoodFixMillis > 0) {
      unsigned long dtFix = millis() - lastGoodFixMillis;
      if (dtFix > 50 && dtFix < 10000)
        gpsFixIntervalMsLoc = (unsigned long)(0.7f*gpsFixIntervalMsLoc + 0.3f*dtFix);
    }
    lastGoodFixMillis = millis();

    float zE = (float)((glon - lon0) * DEG_TO_RAD * cosLat0 * EARTH_RADIUS_M);
    float zN = (float)((glat - lat0) * DEG_TO_RAD * EARTH_RADIUS_M);

    bool haveSpeedCourse = gps.speed.isValid() && gps.course.isValid();
    float gpsSpeedMps = haveSpeedCourse ? (float)gps.speed.mps() : 0.0f;

    if (haveSpeedCourse && gpsSpeedMps > INAV_MIN_COURSE_SPEED_MPS) {
      float zHeading = (float)gps.course.deg() * DEG_TO_RAD;
      inavKalmanUpdateFull(zE, zN, gpsSpeedMps, zHeading,
                            INAV_R_POS, INAV_R_SPEED, INAV_R_HEADING);
    } else {
      inavKalmanUpdatePos(zE, zN, INAV_R_POS);
    }
    state = INAV_STATE_GPS_FIX;
  }
  else if (!originSet) {
    state = INAV_STATE_NO_FIX_YET;
  }
  else {
    unsigned long sinceFix = millis() - lastGoodFixMillis;
    unsigned long missedEq = sinceFix / gpsFixIntervalMsLoc;
    unsigned long maxLossMs = (unsigned long)inav_max_loss_minutes * 60000UL;
    if      (sinceFix >= maxLossMs)                    state = INAV_STATE_LOST;
    else if ((int)missedEq >= inav_stale_after_misses) state = INAV_STATE_IMU_STALE;
    else                                                state = INAV_STATE_IMU_RECENT;
  }

  // Prediction — rate-limited to the user-configured inav_update_hz. Frozen
  // entirely once LOST, exactly as requested: no further advancement of the
  // dead-reckoned position after the max-loss timeout.
  if (originSet && state != INAV_STATE_LOST) {
    unsigned long now = millis();
    unsigned long periodMs = (unsigned long)(1000.0f / max(1.0f, inav_update_hz));
    if (lastPredictMillis == 0) lastPredictMillis = now;
    if (now - lastPredictMillis >= periodMs) {
      float dt = (now - lastPredictMillis) / 1000.0f;
      inavPredict(dt, gz);
      lastPredictMillis = now;
    }
  }

  if (state != lastInavState) {
    switch (state) {
      case INAV_STATE_NO_FIX_YET: engMsg("INAV: waiting for first GPS fix"); break;
      case INAV_STATE_GPS_FIX:    engMsg("INAV: position acquired by GPS"); break;
      case INAV_STATE_IMU_RECENT: engMsg("INAV: GPS gap — interpolating from recent GPS+IMU"); break;
      case INAV_STATE_IMU_STALE:  engMsgf("INAV: GPS lost >=%d fixes — interpolating (stale baseline)",
                                           inav_stale_after_misses); break;
      case INAV_STATE_LOST:       engMsgf("INAV: GPS lost >%d min — interpolation stopped, holding position",
                                           inav_max_loss_minutes); break;
    }
    lastInavState = state;
  }

  double outLat = lat0, outLon = lon0;
  float  outAlt = SENSOR_UNAVAILABLE, outSpeed = 0, outHeadingDeg = 0;
  if (originSet) {
    outLat = lat0 + (inavX[1] / EARTH_RADIUS_M) * RAD_TO_DEG;
    outLon = lon0 + (inavX[0] / (EARTH_RADIUS_M * cosLat0)) * RAD_TO_DEG;
    outSpeed = inavX[2];
    outHeadingDeg = inavX[3] * RAD_TO_DEG;
    if (outHeadingDeg < 0) outHeadingDeg += 360.0f;
    angle[2] = outHeadingDeg;   // Yaw for V10 mirrors the EKF heading — no separate drift source
    if (gps.altitude.isValid()) inavLastAlt = (float)gps.altitude.meters();
    outAlt = inavLastAlt;   // held from last valid GPS fix — no IMU altitude dead-reckoning
  }

  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    // sd.pps_locked is NOT written here anymore — it's updated unconditionally
    // in sensorTask's main mutex block, since PPS is independent of BMI160
    // and this function returns early whenever BMI160 isn't ready.
    sd.inav_state      = state;
    sd.inav_lat        = outLat;
    sd.inav_lng        = outLon;
    sd.inav_alt        = outAlt;
    sd.inav_speed_mps  = outSpeed;
    sd.inav_heading_deg= outHeadingDeg;
    // v2.4: BMI160 also feeds V8-V11 (Roll/Pitch/Yaw/temp), replacing JY-901.
    if (bmiOk && !imuStuck) {
      sd.angle[0] = angle[0]; sd.angle[1] = angle[1]; sd.angle[2] = angle[2];
      sd.imu_temp = imuT;
      sd.imu_ok = true;
      sd.status_flags |= STATUS_IMU_OK;
      sd.status_flags &= ~STATUS_IMU_STUCK;
    }
    if (imuStuck) {
      sd.angle[0]=sd.angle[1]=sd.angle[2]=sd.imu_temp=SENSOR_UNAVAILABLE;
      sd.imu_ok = false;
      sd.status_flags &= ~STATUS_IMU_OK;
      sd.status_flags |= STATUS_IMU_STUCK;
    }
    xSemaphoreGive(dataMutex);
  }
  if (imuStuck) engMsgf("IMU STUCK: BMI160 no good read for %lus", (millis()-lastBmiOkMillis)/1000);
  perfRecordMax(perfMaxI2cPathMs,
                (uint32_t)((micros() - perfInavStartUs) / 1000UL));
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

  // Subscribe the sensor task to the ESP task watchdog. The task feeds it only
  // after a complete sensor cycle, so a genuinely stuck sensor/I2C operation
  // can no longer leave the station silently wedged for an unlimited time.
  esp_err_t wdtResult = esp_task_wdt_add(NULL);
  if (wdtResult != ESP_OK && wdtResult != ESP_ERR_INVALID_STATE) {
    Serial.printf("[WATCHDOG] sensorTask registration failed: %d\n", (int)wdtResult);
  }

  // ADC config already done in setup() — do NOT call analogReadResolution()
  // or analogSetAttenuation() here. That would trigger the driver conflict.

  while (true)
  {
    uint32_t perfSensorCycleStartUs = micros();

    buzzerService();
    unsigned long loopStart = millis();

    // ── GPS PPS LED — keep the ESP-32 board D2 LED on for exactly 250ms ───────
    // Historical implementation note retained from v2.6:
    // The PPS ISR turns the LED on immediately; this task turns it off after
    // the requested 0.25 second pulse. PPS timing itself remains ISR-driven.
    // In v2.7 the ISR no longer drives the LED directly; it only records the
    // event, and this task performs both the ON and OFF operations.
    // The PPS ISR only records the event. This task performs the actual LED
    // operation, so the LED now also proves that the normal sensor application
    // is running rather than merely proving that the interrupt still fires.
    // A counter is used instead of a boolean so closely spaced/queued events
    // cannot be silently lost; normal GPS PPS is one event per second.
    // ── GPS PPS LED ───────────────────────────────────────────────────────────
    // v3.6: LED timing is no longer serviced by sensorTask. A dedicated PPS LED
    // task is woken by the GPIO5 PPS interrupt and generates the exact 250ms
    // pulse. This prevents slow sensor/I2C processing from stretching the pulse.
    // The LED is strictly event-driven: it is ON only for the fixed 250ms PPS
    // indication period, and is never held ON merely because PPS remains locked.
    // PPS lock state is deliberately NOT used here: every physical PPS event
    // produces one LED pulse, while loss of PPS naturally leaves the LED OFF.

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

    // ── Additive sensor-health diagnostics ────────────────────────────────────
    // Persistent rail/no-data conditions are reported, but the corresponding
    // sensor value remains SENSOR_UNAVAILABLE and all other sensors continue.
    // A single bad sample is deliberately ignored to avoid false alarms.
    if (!inmpLastReadHadData) {
      if (inmpFaultCount < SENSOR_HEALTH_FAIL_LIMIT) inmpFaultCount++;
    } else {
      if (inmpFaultReported) engMsg("INMP441: data recovered");
      inmpFaultCount = 0;
      inmpFaultReported = false;
    }
    if (inmpFaultCount >= SENSOR_HEALTH_FAIL_LIMIT) {
      inmpFaultReported = true;
      sensorHealthMsg("INMP441", "no I2S samples", lastInmpHealthMsg);
    }

    bool uvInvalid = (uvRef <= 50);
    if (uvInvalid) {
      if (uvFaultCount < SENSOR_HEALTH_FAIL_LIMIT) uvFaultCount++;
    } else {
      if (uvFaultReported) engMsg("UV: sensor data recovered");
      uvFaultCount = 0;
      uvFaultReported = false;
    }
    if (uvFaultCount >= SENSOR_HEALTH_FAIL_LIMIT) {
      uvFaultReported = true;
      sensorHealthMsg("UV", "invalid reference signal", lastUvHealthMsg);
    }

    bool dustRailFault = (voMeas <= 2 || voMeas >= 4093);
    if (dustRailFault) {
      if (dustFaultCount < SENSOR_HEALTH_FAIL_LIMIT) dustFaultCount++;
    } else {
      if (dustFaultReported) engMsg("Dust: ADC data recovered");
      dustFaultCount = 0;
      dustFaultReported = false;
    }
    if (dustFaultCount >= SENSOR_HEALTH_FAIL_LIMIT) {
      dustFaultReported = true;
      sensorHealthMsg("Dust", "ADC input at rail", lastDustHealthMsg);
      dustDens = SENSOR_UNAVAILABLE;
    }

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
    // v3.5: Snapshot this BEFORE gps.location.lat()/lng() are read below.
    // TinyGPS++ clears location.isUpdated() when lat()/lng() are accessed,
    // and INAV needs to know that this cycle contains a fresh GPS position.
    gpsLocationUpdatedThisCycle = gps.location.isUpdated();
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

    // v2.5: computed here (Core 0, sensorTask) where `gps` is safely owned —
    // never called from loop()/Core 1. Cached to sd.gps_unix_time below.
    long gpsUnixT = gpsUnixTime();

    // ── Inertial navigation — GPS + BMI160 EKF (v2.4) ────────────────────────
    // BMI160 is now the ONLY IMU. This one call handles everything: EKF
    // dead-reckoning, PPS/lat/lng/speed/heading, AND (via the complementary
    // filter inside inertialNav()) Roll/Pitch/Yaw/temp for sd.angle[]/
    // sd.imu_temp — the job JY-901's readIMU() used to do, now gone.
    if (BMI160sensorThere) inertialNav();
    else if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      sd.angle[0]=sd.angle[1]=sd.angle[2]=sd.imu_temp=SENSOR_UNAVAILABLE;
      xSemaphoreGive(dataMutex);
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
      if (BMI160sensorThere && sd.imu_ok)
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
      sd.gps_unix_time = gpsUnixT;
      sd.rand_num=random(1,10);
      // BUGFIX: sd.pps_locked was previously only ever written inside
      // inertialNav(), which returns early (before reaching that line)
      // whenever BMI160 isn't ready. PPS hardware has nothing to do with
      // BMI160, so it must be updated here unconditionally instead.
      sd.pps_locked = ppsIsLocked();
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

    // Sensor-task health heartbeat. Updated only after the complete cycle
    // finishes, so a blocked sensor operation cannot falsely report health.
    sensorTaskHeartbeat++;

    // Feed the ESP task watchdog after a successful full sensor cycle. If the
    // task becomes stuck in an I2C/sensor operation beyond the watchdog window,
    // the hardware/software watchdog infrastructure can recover the ESP32.
    esp_task_wdt_reset();

    unsigned long elapsed = millis()-loopStart;
    perfRecordMax(perfMaxSensorCycleMs,
                  (uint32_t)((micros() - perfSensorCycleStartUs) / 1000UL));
    vTaskDelay(pdMS_TO_TICKS(elapsed < SENSOR_TASK_PERIOD_MS
                              ? SENSOR_TASK_PERIOD_MS - elapsed : 1));
  }
}

// ============================================================================
// BLYNK SENDS — called from BlynkTimer in loop() (Core 1)
// ============================================================================

void blynkSendFast()
{
  uint32_t perfFastStartUs = micros();

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
  // v2.3: PPS lock + inertial-nav fused position/speed/heading
  safeWriteI(V26, snap.pps_locked ? 1 : 0);
  safeWriteI(V27, snap.inav_state);
  safeWrite (V28, snap.inav_lat);
  safeWrite (V29, snap.inav_lng);
  safeWrite (V30, snap.inav_alt);
  safeWrite (V31, snap.inav_speed_mps);
  safeWrite (V32, snap.inav_heading_deg);
  perfRecordMax(perfMaxFastSendMs,
                (uint32_t)((micros() - perfFastStartUs) / 1000UL));
}

void blynkSendSlow()
{
  uint32_t perfSlowStartUs = micros();

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
  perfRecordMax(perfMaxSlowSendMs,
                (uint32_t)((micros() - perfSlowStartUs) / 1000UL));
}

// ─── DIAGNOSTIC VIRTUAL PINS — every 10 seconds ──────────────────────────────
// Internal counters/timers continue constantly. Only their Blynk publication
// is rate-limited to one snapshot every 10 seconds. This prevents diagnostic
// traffic from being added to the 5-second FAST telemetry pathway.
void blynkSendDiagnostics()
{
  if (WiFi.status() != WL_CONNECTED) return;
  if (!Blynk.connected()) return;

  // Existing V34 diagnostic channel: ESP32 reset cause from boot.
  safeWriteS(V34, bootResetReasonText);

  // Performance diagnostics V35-V44. These are retained maxima since boot
  // except V35 (count) and V36 (current PPS age).
  safeWriteI(V35, (int)ppsPulseCount);
  safeWriteI(V36, (int)perfPpsAgeMs());
  safeWriteI(V37, (int)perfMaxLoopGapMs);
  safeWriteI(V38, (int)perfMaxSensorCycleMs);
  safeWriteI(V39, (int)perfMaxBlynkRunMs);
  safeWriteI(V40, (int)perfMaxGpsFeedMs);
  safeWriteI(V41, (int)perfMaxI2cPathMs);
  safeWriteI(V42, (int)perfMaxFastSendMs);
  safeWriteI(V43, (int)perfMaxSlowSendMs);
  safeWriteI(V44, (int)perfMaxPpsLedResponseMs);
  safeWriteI(V45, (int)perfLastI2cDevice);
  safeWriteI(V46, (int)perfLastI2cOperation);
  safeWriteI(V47, (int)perfI2cErrorCount);
  safeWriteI(V48, (int)perfMaxAht21Ms);
  safeWriteI(V49, (int)perfMaxEns160Ms);
  safeWriteI(V50, (int)perfMaxBmi160Ms);
}


// v2.5: Blynk Map widget on V33. The Map widget's virtualWrite convention
// is Blynk.virtualWrite(vPin, index, lat, lon, value):
//   • index — identifies/updates a marker; reusing the same index moves an
//     existing marker, a new index drops a new one. We use the GPS UNIX
//     timestamp (sd.gps_unix_time, computed on Core 0 from GPS UTC date/time
//     — see gpsUnixTime()) so every send is naturally a distinct, monotonic,
//     human-meaningful index, and the map accumulates a trail of points over
//     the deployment rather than a single marker that keeps moving. Kept as
//     the GPS timestamp regardless of which position source below is used —
//     it's the only reliable real-world clock in the system either way.
//   • lat/lon — v2.6: source is now selected automatically each send:
//       - If inertial-nav data EXISTS (BMI160 present and inav_state is one
//         of GPS_FIX / IMU_RECENT / IMU_STALE — i.e. an origin has been set
//         from a real GPS fix and the EKF hasn't been forced into LOST),
//         use the fused sd.inav_lat/sd.inav_lng. This keeps the trail going
//         through brief GPS dropouts via BMI160 dead-reckoning.
//       - Otherwise (no BMI160 wired, or inav_state is NO_FIX_YET/LOST —
//         i.e. inertial-nav data does NOT exist/isn't trustworthy right
//         now), fall back to raw GPS sd.lat/sd.lng, gated on sd.gps_fix,
//         exactly as before. This is also the behaviour with no IMU wired
//         at all, since inav_state then never leaves NO_FIX_YET.
//   • value — a fixed label string identifying this device on the map.
// Deliberately its own timer tier (MAP_SEND_INTERVAL_MS, default 30s) rather
// than piggybacking on FAST(5s)/SLOW(10s) — a marker every few seconds would
// flood the Map widget on any deployment longer than a few minutes.
void blynkSendMap()
{
  if (WiFi.status() != WL_CONNECTED) return;

  bool   gpsFixNow;
  double gpsLat, gpsLng;
  int    inavState;
  double inavLat, inavLng;
  long   unixT;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
    gpsFixNow = sd.gps_fix;
    gpsLat    = sd.lat;
    gpsLng    = sd.lng;
    inavState = sd.inav_state;
    inavLat   = sd.inav_lat;
    inavLng   = sd.inav_lng;
    unixT     = sd.gps_unix_time;
    xSemaphoreGive(dataMutex);
  } else { engMsg("BLYNK MAP: mutex timeout"); return; }

  if (unixT <= 0) return;   // no valid GPS time yet — skip this cycle, try again later

  // Does inertial-nav data exist and can it be trusted right now?
  bool inavExists = (inavState == INAV_STATE_GPS_FIX ||
                      inavState == INAV_STATE_IMU_RECENT ||
                      inavState == INAV_STATE_IMU_STALE);

  double lat, lng;
  const char* src;
  if (inavExists) {
    lat = inavLat; lng = inavLng; src = "INAV";
  } else if (gpsFixNow) {
    lat = gpsLat; lng = gpsLng; src = "GPS";
  } else {
    return;   // neither source is trustworthy this cycle — skip, try again later
  }

  Blynk.virtualWrite(V33, unixT, lat, lng, "AOQ-ESP32");
  delay(VWRITE_GAP_MS);

  if (DEBUGON)
    Serial.printf("[BLYNK MAP] idx=%ld lat=%.6f lng=%.6f src=%s\n", unixT, lat, lng, src);
}

// ─── NON-BLOCKING WiFi recovery state ───────────────────────────────────────
bool wifiConnectInProgress = false;
unsigned long wifiConnectStarted = 0;
unsigned long wifiFailureAlertAt = 0;
// Blynk connection is deliberately managed separately from WiFi.
// Blynk.run() must NEVER be allowed to initiate an unbounded connection
// attempt on Core 1, because that can starve the ESP32 interrupt watchdog
// when the Blynk server is unreachable.  We therefore make a bounded TCP
// probe first, use the library-supported 1s Blynk I/O timeout, and service
// the same handshake incrementally until it completes.
bool blynkConfigured = false;
unsigned long lastBlynkConnectAttempt = 0;
// Initial Blynk handshake state.  Once the TCP probe succeeds, keep servicing
// Blynk.run() on every loop pass until the handshake completes instead of
// waiting for the next 10s connection-attempt interval.
bool blynkHandshakeActive = false;
unsigned long blynkHandshakeStarted = 0;
#define BLYNK_CONNECT_RETRY_MS 10000UL
#define BLYNK_CONNECT_TIMEOUT_S 5UL
#define BLYNK_HANDSHAKE_TIMEOUT_MS 10000UL
#define BLYNK_SERVER_PROBE_TIMEOUT_MS 250UL

void wifiStartConnect()
{
  Serial.printf("[WiFi] Connecting to %s...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  wifiConnectStarted = millis();
  wifiConnectInProgress = true;
}

void wifiMarkConnected()
{
  wifiConnectInProgress = false;
  Serial.printf("\n[WiFi] Connected — IP=%s RSSI=%ddBm MAC=%s\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI(), WiFi.macAddress().c_str());
  // Blynk.config() sets server address only — does NOT block or spin.
  // IMPORTANT: do NOT rely on the first Blynk.run() to establish the server
  // connection. With Blynk v0.6.1, Blynk.run() can invoke the default connection
  // timeout when the server is not reachable. That can hold Core 1 long enough
  // to trigger the ESP32 Interrupt WDT. We explicitly probe the server first,
  // then allow Blynk.run() to advance the handshake in bounded increments.
  Blynk.config(BLYNK_AUTH, BLYNK_SERVER, BLYNK_PORT);
  blynkConfigured = true;
  blynkHandshakeActive = false;
  blynkHandshakeStarted = 0;
  lastBlynkConnectAttempt = millis() - BLYNK_CONNECT_RETRY_MS;
  if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(10))==pdTRUE) {
    sd.status_flags|=STATUS_WIFI_OK; xSemaphoreGive(dataMutex);
  }
  engMsgf("WiFi OK — Blynk configured %s:%d", BLYNK_SERVER, BLYNK_PORT);
  buzzerTone(5000, 100);
}

void wifiService()
{
  if (!WIFI) return;

  if (WiFi.status() == WL_CONNECTED) {
    if (wifiConnectInProgress) wifiMarkConnected();
    return;
  }

  if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(5))==pdTRUE) {
    sd.status_flags&=~STATUS_WIFI_OK; xSemaphoreGive(dataMutex);
  }

  unsigned long now = millis();
  if (wifiConnectInProgress) {
    // Do not block Core 1 while the WiFi driver performs its association.
    // If it takes too long, abandon this attempt and allow a later retry.
    if (now - wifiConnectStarted >= 20000UL) {
      wifiConnectInProgress = false;
      WiFi.disconnect();
      Serial.println("\n[WiFi] FAILED (20s timeout)");
      engMsg("WiFi FAILED: offline");
      // Preserve the original audible failure indication. It is emitted only
      // once per failed connection attempt; the connection process itself is
      // still non-blocking for the full 20-second association window.
      wifiFailureAlertAt = now;
      buzzerPattern3(1000, 600, 200);
    }
    return;
  }

  if (now - lastWiFiCheck >= WIFI_RECONNECT_MS) {
    lastWiFiCheck = now;
    engMsg("WiFi: reconnect attempt");
    Blynk.disconnect();
    WiFi.disconnect();
    wifiStartConnect();
  }
}

// ============================================================================
// WiFi CONNECT
// ============================================================================

// Kept as the original blocking helper for compatibility with the existing
// firmware interface. Normal operation now uses wifiStartConnect()/wifiService()
// so Core 1 is never held in a 20-second connection loop.
bool wifiConnect()
{
  Serial.printf("[WiFi] Connecting to %s...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t = millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-t<20000) { delay(500); Serial.print("."); }
  if (WiFi.status()!=WL_CONNECTED) {
    Serial.println("\n[WiFi] FAILED");
    engMsg("WiFi FAILED: offline");
    buzzerPattern3(1000, 600, 200);
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
  Serial.println("\n[INIT] Air Quality Station ESP32 v3.7");
  Serial.printf("[INIT] ten_mins_autoreset = %s\n", ten_mins_autoreset ? "ON":"OFF");

  // ── ESP reset-cause diagnostic ───────────────────────────────────────────
  bootResetReason = esp_reset_reason();
  bootResetReasonText = resetReasonToString(bootResetReason);
  Serial.printf("[INIT] ESP reset reason: %s (%d)\n",
                bootResetReasonText, (int)bootResetReason);

  // ── Explicit ESP task-watchdog configuration ─────────────────────────────
  // Do not rely on the Arduino core's default task-WDT settings. The sensor
  // task is subscribed below and is expected to feed this watchdog only after
  // completing a full sensor cycle. A 10s window leaves ample margin for the
  // existing bounded I2C operations while still recovering a genuinely stuck
  // sensor task.
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = 10000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_err_t wdtConfigResult = esp_task_wdt_reconfigure(&wdtConfig);
  if (wdtConfigResult == ESP_ERR_INVALID_STATE) {
    wdtConfigResult = esp_task_wdt_init(&wdtConfig);
  }
  Serial.printf("[WATCHDOG] explicit task-WDT config: %s (%d), timeout=%lums\n",
                wdtConfigResult == ESP_OK ? "OK" : "ERROR",
                (int)wdtConfigResult, (unsigned long)wdtConfig.timeout_ms);

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
  // Keep the reset cause in the normal engineering-message path so it is
  // also visible on the existing Blynk V19 diagnostic channel.
  engMsgf("ESP reset reason: %s", bootResetReasonText);
  // Pre-fill all sensor fields with UNAVAILABLE until first valid reads
  sd.co_ppm=sd.co_raw=sd.co_heater_v=SENSOR_UNAVAILABLE;
  sd.gps_unix_time = 0;
  sd.tvoc_ppb=sd.eco2_ppm=sd.aqi=SENSOR_UNAVAILABLE;
  sd.uvi=sd.sound_db=sd.dust_mg=SENSOR_UNAVAILABLE;
  sd.temp=sd.hum=SENSOR_UNAVAILABLE;
  sd.angle[0]=sd.angle[1]=sd.angle[2]=sd.imu_temp=SENSOR_UNAVAILABLE;
  // v2.3: pre-fill inertial-nav fields until origin is set from first GPS fix
  sd.pps_locked = false;
  sd.inav_state = INAV_STATE_NO_FIX_YET;
  sd.inav_lat = sd.inav_lng = SENSOR_UNAVAILABLE;
  sd.inav_alt = sd.inav_speed_mps = sd.inav_heading_deg = SENSOR_UNAVAILABLE;

  // ── GPIO setup ───────────────────────────────────────────────────────────
  pinMode(ledPin11,OUTPUT); digitalWrite(ledPin11,LOW);
  pinMode(ledPin12,OUTPUT); digitalWrite(ledPin12,LOW);
  pinMode(ledPin13,OUTPUT); digitalWrite(ledPin13,LOW);
  pinMode(PPS_LED_PIN, OUTPUT); setPpsLed(false);
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

  // ── I2C for ENS160+AHT2x+BMI160 ──────────────────────────────────────────
  Wire.begin(21, 22);  // SDA=GPIO21, SCL=GPIO22
  // CRITICAL: without a bounded timeout, Wire.requestFrom()/endTransmission()
  // can block indefinitely if a device doesn't ACK (e.g. BMI160 not yet
  // wired, floating SDA/SCL, bad joint). 50ms bounds every Wire call for the
  // rest of the sketch — belt-and-suspenders alongside the fix below.
  Wire.setTimeOut(50);

  // ── ENS160+AHT2x / BMI160 init — DELIBERATELY NOT DONE HERE ──────────────
  // Earlier versions called initENS()/initBMI160() directly in setup(), which
  // runs entirely on Core 1 BEFORE wifiConnect() below. If either device
  // failed to ACK on the bus (not yet wired, floating SDA/SCL, bad joint),
  // the I2C call could block long enough for the ESP32's own system
  // watchdog (TG0WDT — separate from the app-level task/interrupt
  // watchdogs discussed elsewhere in this file) to fire, producing an
  // endless "rst:0x7 (TG0WDT_SYS_RESET)" boot loop that never reached WiFi.
  // The Wire.setTimeOut(50) above bounds that specific hang, but the more
  // fundamental fix is architectural: I2C sensor init now happens EXCLUSIVELY
  // inside sensorTask (Core 0) — see the ENS160 block in sensorTask and the
  // BMI160 lazy-init block at the top of inertialNav(), both of which already
  // attempt initialisation on their very first call and retry periodically
  // thereafter if the device is absent. This guarantees WiFi on Core 1 can
  // NEVER be blocked by I2C, no matter what is or isn't wired to the bus —
  // matching the same principle already used for the CO sensor: nothing that
  // can stall waiting on hardware runs before WiFi starts.

  // ── GPS 1PPS interrupt (v2.3) ─────────────────────────────────────────────
  if (PPSsensorThere) {
    pinMode(PPS_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsISR, RISING);
    Serial.println("[INIT] GPS PPS interrupt attached (GPIO5)");
  }

  // ── CO LEDC PWM ──────────────────────────────────────────────────────────
  if (COsensorThere) {
    ledcAttach(CO_PWM_PIN, LEDC_FREQ_CO, LEDC_RES_CO);
    ledcWrite(CO_PWM_PIN, 0);
  }

  // [SUPERSEDED v2.4] JY-901 UART2 init removed — GPIO16/17 are free. BMI160
  // (the only IMU now) is initialised in the ENS160/BMI160 block below.

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

  // ── Launch PPS LED task ───────────────────────────────────────────────────
  // v3.6: Keep the PPS LED timing independent from sensorTask. The task is
  // woken directly by the GPIO5 PPS ISR and produces one fixed 250ms pulse.
  xTaskCreatePinnedToCore(ppsLedTask, "ppsLedTask", 2048, NULL, 2,
                          &ppsLedTaskHandle, 1);

  // ── Launch sensor task on Core 0 ─────────────────────────────────────────
  // WiFi is connected on Core 1 (this setup() function) before CO startup,
  // so the WiFi driver is running during any long CO initialisation.
  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 8192, NULL, 1, NULL, 0);

  // ── WiFi + Blynk config ──────────────────────────────────────────────────
  // Start WiFi asynchronously. The previous wifiConnect() helper remains
  // available, but normal operation uses the non-blocking state machine so
  // Core 1 remains responsive while the station associates with the AP.
  if (WIFI) {
    lastWiFiCheck = millis() - WIFI_RECONNECT_MS;
    wifiStartConnect();
  }

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
  blynkTimer.setInterval(10000UL, blynkSendDiagnostics);
  blynkTimer.setInterval(MAP_SEND_INTERVAL_MS, blynkSendMap);

  engMsg("Setup OK v3.7 — long-term reliability monitoring enabled");
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
  static uint32_t perfLastLoopStartUs = 0;
  uint32_t perfLoopStartUs = micros();
  if (perfLastLoopStartUs != 0)
    perfRecordMax(perfMaxLoopGapMs,
                  (uint32_t)((perfLoopStartUs - perfLastLoopStartUs) / 1000UL));
  perfLastLoopStartUs = perfLoopStartUs;


  buzzerService();

  if (WIFI) {
    wifiService();

    if (WiFi.status() == WL_CONNECTED) {
      // Blynk v0.6.1 may attempt a long server connection from inside
      // Blynk.run() when the configured server is not yet connected. Do not
      // call Blynk.run() while the server is completely unreachable. Once the
      // bounded TCP probe succeeds, however, keep calling Blynk.run() on every
      // loop pass until the existing handshake finishes. This is important: the
      // first Blynk.run() starts the handshake, but it may return before the
      // handshake is complete. The previous code waited for the next retry
      // interval, which is why your second handshake was always ~7-10 seconds
      // later. No second handshake is needed — the same Blynk session is simply
      // serviced again until it reaches Blynk.connected().
      unsigned long nowBlynk = millis();

      if (blynkConfigured && !Blynk.connected() && !blynkHandshakeActive &&
          nowBlynk - lastBlynkConnectAttempt >= BLYNK_CONNECT_RETRY_MS) {
        lastBlynkConnectAttempt = nowBlynk;

        // IMPORTANT: Blynk v0.6.1 Blynk.connect(timeout) is a blocking loop.
        // Even a nominal 1-second timeout can monopolize Core 1 long enough
        // to trigger the ESP32 Interrupt WDT when the TCP connection is not
        // immediately usable. Probe the local Blynk TCP port first with a
        // bounded WiFiClient timeout; only start Blynk.run() when the server
        // has actually accepted a TCP connection. This avoids the failure mode
        // where an unavailable/half-open Blynk server causes the CPU1 IWDT.
        WiFiClient blynkProbe;
        blynkProbe.setTimeout(BLYNK_SERVER_PROBE_TIMEOUT_MS);
        bool serverReachable = blynkProbe.connect(BLYNK_SERVER, BLYNK_PORT);
        blynkProbe.stop();

        if (!serverReachable) {
          Serial.println("[Blynk] Server TCP probe failed — retry later");
        } else {
          // IMPORTANT:
          // Do NOT call Blynk.connect(timeout) here. Blynk v0.6.1 implements
          // connect() as a blocking loop and it has proved unreliable with this
          // local server even when the TCP port is reachable.
          //
          // The original firmware successfully established the Blynk session
          // through Blynk.run(). We preserve that behavior, but only allow
          // Blynk.run() to start the initial handshake after the bounded TCP
          // probe above has confirmed that the server is accepting connections.
          //
          // Once started, the handshake is NOT restarted every 10 seconds.
          // Blynk.run() is serviced on each loop pass until Blynk.connected()
          // becomes true. This is the normal way to advance the same Blynk
          // protocol session after its initial connection attempt.
          blynkHandshakeActive = true;
          blynkHandshakeStarted = millis();
          Serial.println("[Blynk] TCP port open — starting Blynk.run() handshake");
        }
      }

      if (blynkHandshakeActive && !Blynk.connected()) {
        // v3.3: Blynk v0.6.1 performs blocking socket reads inside Blynk.run().
        // BLYNK_TIMEOUT_MS is deliberately limited to 1s above, so one call
        // cannot monopolise Core 1 long enough to trip the ESP32 idle/interrupt
        // watchdog. If the server needs more time, the SAME Blynk session is
        // serviced again on the next loop pass — this is not a new handshake.
        // The short delay/yield below also gives the Arduino/ESP32 system tasks
        // a scheduling opportunity between successive protocol steps.
        {
          uint32_t perfBlynkRunStartUs = micros();
          Blynk.run();
          perfRecordMax(perfMaxBlynkRunMs,
                        (uint32_t)((micros() - perfBlynkRunStartUs) / 1000UL));
        }
        yield();

        if (Blynk.connected()) {
          blynkHandshakeActive = false;
          Serial.println("[Blynk] Connected");
        } else if (millis() - blynkHandshakeStarted >= BLYNK_HANDSHAKE_TIMEOUT_MS) {
          // The TCP probe succeeded, but the Blynk protocol handshake did not
          // finish in a bounded period. Stop servicing the failed session and
          // return to the normal retry state. This prevents an indefinitely
          // stuck connection attempt while still giving the working server
          // enough time to complete its handshake.
          blynkHandshakeActive = false;
          Blynk.disconnect();
          Serial.println("[Blynk] Handshake timeout — retry later");
        }
      }

      if (Blynk.connected()) {
        // Normal connected-state servicing. BLYNK_TIMEOUT_MS=1s also bounds
        // any waiting read if the server disappears between loop iterations.
        {
          uint32_t perfBlynkRunStartUs = micros();
          Blynk.run();
          perfRecordMax(perfMaxBlynkRunMs,
                        (uint32_t)((micros() - perfBlynkRunStartUs) / 1000UL));
        }
        yield();
        blynkTimer.run();
      }

      if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(5))==pdTRUE) {
        sd.status_flags|=STATUS_WIFI_OK; xSemaphoreGive(dataMutex);
      }
    }
  }

  // ── Application health monitor ───────────────────────────────────────────
  // This does not replace the task watchdog. It provides a second, visible
  // diagnostic path and catches a sensor task that is alive but no longer
  // completing its normal work cycle.
  uint32_t hb = sensorTaskHeartbeat;
  unsigned long now = millis();
  if (hb != lastHealthHeartbeat) {
    lastHealthHeartbeat = hb;
    lastHeartbeatChangeMillis = now;
  } else if (setupDone && lastHeartbeatChangeMillis != 0 &&
             now - lastHeartbeatChangeMillis >= SENSOR_TASK_HEALTH_TIMEOUT_MS) {
    // Only report/recover once per timeout window. The task watchdog is the
    // primary recovery mechanism; this is a belt-and-suspenders safeguard.
    Serial.println("[HEALTH] sensorTask heartbeat stalled — requesting restart");
    engMsg("HEALTH FAULT: sensor task stalled");
    lastHeartbeatChangeMillis = now;
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP.restart();
  }

  if (now - lastHealthReport >= HEALTH_REPORT_INTERVAL_MS) {
    perfReport();

    lastHealthReport = now;
    Serial.printf("[HEALTH] heap=%u minHeap=%u largest=%u sensorHB=%lu WiFi=%s Blynk=%s PPS=%d\n",
                  ESP.getFreeHeap(), ESP.getMinFreeHeap(),
                  heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
                  (unsigned long)sensorTaskHeartbeat,
                  WiFi.status() == WL_CONNECTED ? "OK" : "OFF",
                  Blynk.connected() ? "OK" : "OFF",
                  ppsIsLocked() ? 1 : 0);
  }
}
