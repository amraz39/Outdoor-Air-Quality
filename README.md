# Outdoor Air Quality Station

> *A portable environmental monitor that logs CO, NO₂, UV, particulate matter,
> sound, temperature, humidity, and orientation — with live GPS coordinates
> streaming to a self-hosted Blynk dashboard for real-time air-quality mapping.*

**ESP32 · FreeRTOS dual-core · Blynk 0.6.1 local server · ATGM336H GPS · 8 sensing subsystems**

---

## Table of contents

1. [The story](#the-story)
2. [Sensor suite](#sensor-suite)
3. [Latest update to v3.7](#latest-update-to-v37)
4. [Why the hardware changed](#why-the-hardware-changed)
5. [Pin mapping](#pin-mapping)
6. [Wiring and voltage dividers](#wiring-and-voltage-dividers)
7. [Firmware architecture](#firmware-architecture)
8. [CO sensor — calibration and state machine](#co-sensor--calibration-and-state-machine)
9. [GPS initialisation](#gps-initialisation)
10. [IMU protocol](#imu-protocol)
11. [Fault detection and engineering messages](#fault-detection-and-engineering-messages)
12. [Blynk setup](#blynk-setup)
13. [Configuration reference](#configuration-reference)
14. [Required libraries](#required-libraries)
15. [Air quality mapping](#air-quality-mapping)
16. [Known limitations and future work](#known-limitations-and-future-work)
17. [System Health Diagnostics](#system-health-diagnostics)

---

## The story

### 2017 — the original build

This project began as a DIY answer to a simple question: *what is the air actually
like outside, and where is it worst?*

The first version was built around an **Arduino Mega 2560** — the natural choice
at the time for a project needing multiple hardware UARTs, many analogue inputs,
and enough flash to hold a complex sketch. The station was designed to ride on a
vehicle or backpack, measuring urban pollutants in real time and pushing data over
WiFi to a server so readings could be plotted on a map.

The CO measurement presented the most interesting engineering challenge. The MQ-7
sensor demands an alternating power cycle — 60 seconds at ~5 V to heat and
regenerate the sensing element, then 90 seconds at ~1.4 V while the actual
resistance measurement is taken. On the Mega this was handled through a custom
transistor switching board driven by Timer2 hardware PWM registers (`TCCR2A`,
`OCR2B`). A duty-cycle calibration sweep ran at startup to find the exact PWM
width that produced the correct low-heat voltage on the sensor node.

A separate **ESP8266 module** on UART3 handled WiFi using the `Esp8266EasyIoT`
library. The GPS and IMU each occupied their own hardware UART. Everything —
sensor reads, GPS parsing, IMU protocol, WiFi transmission, display updates —
ran sequentially in a single `loop()`, with a soft-reset watchdog calling
`asm volatile("jmp 0")` every ten minutes as a field reliability backstop.

Despite its architecture the system worked. It logged real data from real drives
through real city air.

### 2024 — the migration

Seven years on, the limitations of the original design had become production
blockers rather than acceptable trade-offs:

- The single sequential loop meant GPS data was always several hundred milliseconds
  stale. IMU frames were routinely dropped during long sensor reads.
- The ESP8266 was an additional failure point, consumed an entire UART, and
  required careful baud-rate handshaking to stay in sync with the Mega.
- The Mega's 10-bit ADC at 5 V limited precision on the ratiometric UV sensor and
  produced noisier readings throughout.
- Timer2 register manipulation was AVR-specific — the CO PWM logic could not be
  ported to any other architecture without a complete rewrite.
- There was no fault detection. When something went wrong in the field, the only
  diagnostic was silence.

The **ESP32 DevKit** addressed every one of these. Two Xtensa LX6 cores running
FreeRTOS allow sensor work and network work to run concurrently, with a proper
mutex protecting shared data. The built-in WiFi eliminates the ESP8266 entirely.
The LEDC peripheral replaces Timer2. A 12-bit ADC at 3.3 V improves resolution and
reduces noise. And a structured engineering-message system means the station can
now tell you exactly what it is doing and what has gone wrong.

The backend also moved from the original proprietary IoT library to a
**self-hosted Blynk 0.6.1 server** (patched for Java 21 compatibility), running
on local home infrastructure. No data leaves the local network.

---

## Sensor suite

| Sensor | Parameter measured | Interface | Supply voltage | Firmware role |
|---|---|---|---|---|
| MQ-7 (custom heater board) | Carbon monoxide (CO ppm) | Analogue + PWM heater control | 5 V | Closed-loop heater control and CO estimation |
| ML8511 | UV index (UVI) | Analogue ratiometric | 3.3 V | UV measurement with reference compensation |
| Sharp GP2Y1010AU0F | Particulate matter / dust density | Analogue + IR LED control | 5 V | Dust-density estimate |
| ENS160 | TVOC · eCO₂ · AQI | I²C 0x53 | 3.3 V | VOC/equivalent-CO₂/air-quality processing |
| AHT2x | Temperature · relative humidity | I²C 0x38 | 3.3 V | Ambient data and ENS160 compensation |
| INMP441 | Sound level estimate | I²S | 3.3 V | 24-bit digital microphone, dBFS estimate |
| GY-BMI160 | Roll · pitch · yaw/heading · IMU temperature · raw gyro/accel | I²C 0x69 | 3.3 V | IMU health, attitude estimate and INAV |
| ATGM336H GPS | Position · satellite count · HDOP · UTC time · 1PPS | UART1 + GPIO PPS | 3.3 V | Raw GPS, INAV correction and PPS lock indication |

> The current firmware no longer uses the former MICS-2710, DHT21/AM2301, JY-901/WT901, analogue microphone, or u-blox GPS. Those references in the historical migration sections are retained only to document the project's evolution.

---

## Latest update to v3.7

The current firmware (`AirQualityOutdoor_ESP32_Blynk(5).ino`) adds the following long-term reliability and diagnostic features while preserving the existing sensor, GPS, INAV, CO, Blynk, WiFi, display, LED, buzzer, and status logic:

* Added ESP32 reset-cause diagnostics at startup using `esp_reset_reason()`.
* Added **Blynk V34** for the human-readable ESP32 reset reason. V34 is a boot diagnostic, not a live sensor-health value.
* Added explicit ESP32 task-watchdog configuration with a **10-second timeout**.
* The sensor task is explicitly registered with the task watchdog and feeds it **only after a complete sensor cycle**.
* Added a sensor-task heartbeat counter and health monitoring for a task that is alive but stops completing its work cycle.
* Added asynchronous, non-blocking buzzer tones and a three-step buzzer pattern state machine; the existing buzzer calls remain compatible.
* Added persistent health monitoring and recovery reporting for INMP441, UV, and dust sensors.
* Added `SENSOR_UNAVAILABLE` handling for persistent invalid/no-data conditions so failed sensors do not silently produce plausible measurements.
* Added bounded I²C timeout (`Wire.setTimeOut(50)`) and sensor retry behaviour.
* Added non-blocking WiFi association/reconnect handling and bounded Blynk server probing/handshake handling.
* Preserved the GPS PPS interrupt and dedicated PPS LED task: every received PPS event produces a fixed **250 ms** pulse on the ESP32 board's D2/GPIO2 LED.
* Preserved the BMI160-based INAV EKF and the GPS-loss states `NO_FIX_YET`, `GPS_FIX`, `IMU_RECENT`, `IMU_STALE`, and `LOST`.

### Important diagnostic distinction

* **V4** is the changing application/telemetry diagnostic value.
* **V19** is the latest human-readable engineering/status message generated during operation.
* **V20** is the live subsystem-status bitmask.
* **V26** is GPS PPS lock (`0`/`1`).
* **V27** is the INAV state (`0`–`4`).
* **V34** is the ESP32 **reset cause from the most recent boot**.
* **V35–V50** are performance/I²C diagnostics (PPS count/age, retained-max timings, last I²C device/operation, I²C error count) for tracking down stalls.

---

## Why the hardware changed

| Problem on the Mega + ESP8266 | Solution on the ESP32 |
|---|---|
| Single `loop()` — all subsystems compete for time | Two FreeRTOS tasks pinned to separate cores |
| Separate ESP8266 WiFi module on UART3 | Built-in WiFi — external module removed entirely |
| 10-bit ADC at 5 V | 12-bit ADC at 3.3 V (ADC1 only with WiFi active) |
| Timer2 registers for CO PWM — AVR-specific | LEDC peripheral — `ledcSetup()` / `ledcWrite()` |
| `serialEvent2()` ISR — no equivalent on ESP32 | Manual `pollIMU()` polling per sensor loop cycle |
| `asm volatile("jmp 0")` watchdog reset | `ESP.restart()` — clean software reset on Xtensa |
| No fault detection or telemetry | Per-subsystem watchdogs + engineering messages to Blynk V19 |
| UART3 consumed by WiFi link | UART1 (GPS) and UART2 (IMU) both fully available |

---

## Pin mapping

### Analogue inputs — ADC1 only

| Signal | ESP32 GPIO | Notes |
|---|---:|---|
| MQ-7 CO sense A0 | **36 (VP)** | Input-only · 10kΩ + 10kΩ divider |
| MQ-7 heater feedback A1 | **39 (VN)** | Input-only · 10kΩ + 10kΩ divider |
| ML8511 UV OUT | **34** | Input-only · direct 3.3 V |
| ML8511 UV REF | **35** | Input-only · direct 3.3 V |
| GP2Y1010 dust AO | **32** | 10kΩ + 10kΩ divider |

### Digital / peripheral I/O

| Signal | ESP32 GPIO | Direction / notes |
|---|---:|---|
| CO heater PWM | **4** | LEDC output |
| GPS 1PPS | **5** | Rising-edge interrupt input |
| ESP32 board D2 LED / PPS LED | **2** | Output · 250 ms pulse per PPS event |
| Dust IR LED | **18** | Active LOW output |
| CO green LED | **14** | Output |
| CO orange LED | **27** | Output |
| CO red LED | **15** | Output |
| Buzzer | **19** | LEDC output |
| I²C SDA | **21** | ENS160 + AHT2x + BMI160 |
| I²C SCL | **22** | ENS160 + AHT2x + BMI160 |
| GPS UART1 RX | **13** | ATGM336H TX → ESP32 RX |
| GPS UART1 TX | **23** | ESP32 TX → ATGM336H RX |
| INMP441 BCLK | **25** | I²S |
| INMP441 LRCLK/WS | **26** | I²S |
| INMP441 data | **33** | I²S input |

> GPIO16/GPIO17 are no longer used by the former JY-901. The BMI160 is the only IMU and is connected through I²C.

---

## Wiring and voltage dividers

Several sensors run on a 5 V supply and their analogue outputs can swing up to
the full 5 V rail — well above the ESP32 GPIO's 3.3 V absolute maximum. A
resistor voltage divider on each affected signal line scales the worst-case
output down to a safe 3.0 V, leaving a comfortable 0.3 V margin.

<p align="left">
  <img src="esp32_airquality_wiring_full.png" alt="ESP-323 Wiring diagarm for outdoor air quality monitoring" width="500"/>
</p>

### Divider schematic
Stanard:

```
    Sensor AO
    (5 V max)
        │
      10 kΩ  ← R1 (series)
        │
        ├──────────────────── → ESP32 ADC pin
        │                       Vout = 5 V × 15/(10+15) = 3.00 V  ✓
      15 kΩ  ← R2 (to GND)
        │
       GND
```

My case (based on available **10 kΩ** resistors):

```
    Sensor AO
    (5 V max)
        │
      10 kΩ  ← R1 (series)
        │
        ├──────────────────── → ESP32 ADC pin
        │                       Vout = 5 V × 20/(10+(10+10)) = 3.33 V  ✓
      10 kΩ  ← R2 (to GND)
        │
      10 kΩ  ← R3 (to GND)
        │
       GND
```

Use **E24 series 1 % metal-film resistors** or in my case **0805 0.5% chip resistor**. The total divider impedance 
of **30 kΩ** (my case) or 25 kΩ (standard) is negligible compared to the ESP32 ADC's input impedance (~1 MΩ), so loading
error is immeasurably small.

### Which signals need a divider

| Signal | ESP32 GPIO | Divider | Reason |
|---|---:|---|---|
| MQ-7 CO analogue out | **36** | 10 kΩ + 10 kΩ | MQ-7 board can present a 5 V-referenced analogue signal |
| MQ-7 heater feedback A1 | **39** | 10 kΩ + 10 kΩ | 5 V heater feedback is scaled to the ESP32 ADC |
| GP2Y1010 dust Vo | **32** | 10 kΩ + 10 kΩ | Sensor is powered from 5 V; analogue output must stay within ESP32 limits |
| ML8511 UV OUT / REF | **34 / 35** | None | Native 3.3 V ratiometric outputs |
| ENS160 / AHT2x / BMI160 I²C | **21 / 22** | None | 3.3 V I²C bus |
| ATGM336H GPS UART | **13 / 23** | None | 3.3 V UART |
| INMP441 I²S | **25 / 26 / 33** | None | 3.3 V digital interface |

> The current firmware no longer uses the former MICS-2710, DHT21, analogue
> microphone, or JY-901 connections.

> **Dust-density scaling:** the GP2Y1010 analogue voltage is divided before the
> ESP32 ADC. The firmware applies the configured divider scale before using its
> `0.17 × Vo − 0.1` estimate.

---

## Firmware architecture

### Dual-core FreeRTOS task split

```text
Core 0 — sensorTask
  UV, dust, ENS160/AHT2x, INMP441, CO state machine
  GPS UART parsing and GPS validity
  BMI160 reads + complementary attitude filter
  INAV EKF and GPS-loss state machine
  Per-sensor health checks
  Sensor-task heartbeat
  Explicit task-WDT registration/feed

Core 1 — Arduino loop()
  WiFi state machine
  Blynk.run() and BlynkTimer
  Blynk telemetry writes
  Bounded Blynk TCP probe/handshake
  Non-blocking buzzer service

Dedicated PPS task
  Woken by the GPIO5 PPS ISR
  Generates exactly one 250 ms D2/GPIO2 LED pulse per PPS event

Shared state
  SensorData is protected by a FreeRTOS mutex.
```

The sensor task is isolated from Blynk/WiFi activity. A Blynk outage therefore does
not stop sensor acquisition. Conversely, a failed sensor is isolated and does not
stop the remaining sensors. I²C operations are bounded by a 50 ms Wire timeout.

---

## CO sensor — calibration and state machine

### How the MQ-7 works

The MQ-7 tin-oxide element must alternate between two operating voltages. High
voltage burns off contamination and resets the surface; low voltage is when
the resistance is actually sensitive to CO concentration:

```
 ┌──────────────────────────┐     60 s      ┌──────────────────────────┐
 │      Heating phase       │ ────────────▶ │    Measurement phase     │
 │   duty = 255  (~5 V)     │               │  duty = opt_width (~1.4V) │
 │      phase = 1           │ ◀──────────── │      phase = 0           │
 └──────────────────────────┘     90 s      └──────────────────────────┘
```

At startup `pwm_adjust()` sweeps the LEDC duty from 0 to 249, sampling the
voltage on the MQ-7 sense node via GPIO 36 after a 50 ms settle at each step.
It stops when the voltage crosses 1.4 V and stores the nearest duty value as
`opt_width`. This approach is hardware-independent — it compensates automatically
for any variation in transistor gain on the custom CO board.

### Resistance-to-ppm conversion

```
R_sensor  = R_ref × (4095 / ADC_reading) − R_ref
CO_ppm    = 100 × (exp(R_100ppm / R_sensor) − 1.648)
```

`R_ref` = 9.98 kΩ (reference resistor). `R_100ppm` is derived from either a
known-concentration calibration source, or approximated as `R_clean_air × 0.5`
using the MQ-7 datasheet ratio for clean air vs 100 ppm CO.

### Calibration — required before first deployment

Power the station outdoors in clean air. Wait for the first full measurement
phase to complete (90 s). Read the raw ADC value from the Serial monitor
and enter it as `sensor_reading_clean_air` in the sketch.

> A fresh MQ-7 element needs **24–48 hours** of continuous operation at working
> voltage before readings stabilise. Do not calibrate on a brand-new sensor.

### Watchdog protection

If either phase runs longer than `CO_PHASE_MAX_MS` (3 minutes), the firmware
flags `STATUS_CO_FAULT`, sends an engineering message, and forces a phase
transition to break any deadlock. ADC readings at the rail (below 10 or above
4085 counts) are silently rejected — they indicate an open circuit or missing
sensor — and the exponential moving average is not updated.

---

## GPS initialisation

The current GPS is an **ATGM336H** using standard NMEA output at **9600 baud**.
No u-blox UBX configuration sequence is used. TinyGPS++ parses the NMEA stream on
Core 0.

The GPS also supplies a **1PPS signal on GPIO5**. The PPS ISR records timing/event
state only; a dedicated FreeRTOS task generates a fixed 250 ms pulse on the ESP32
board's D2/GPIO2 LED. This makes the LED a direct visual indication that GPS PPS
events are reaching the ESP32.

### PPS lock criteria

V26 reports PPS lock as:

```text
0 = not locked
1 = locked
```

A PPS lock requires a pulse within the last 2 seconds and a measured PPS interval
between **0.900 s and 1.100 s**. The LED is event-driven: it does not remain on
while PPS is locked.

---

## IMU protocol

The current IMU is the **GY-BMI160** at I²C address `0x69`. It is read directly by
the firmware; no external BMI160 library is required. The configured ranges are:

```text
Accelerometer: ±4 g
Gyroscope:     ±500 dps
ODR:           100 Hz
```

The BMI160 supplies raw gyro/accelerometer data for INAV and a lightweight
complementary filter. Roll and pitch use gyro integration corrected by the
gravity vector. Yaw is the INAV/GPS-corrected heading state because the BMI160 has
no magnetometer. The BMI160 die temperature is reported on V11.

A failed individual I²C read does not immediately declare the IMU dead.
`STATUS_IMU_STUCK` is raised only after no successful BMI160 read has been seen for
`IMU_TIMEOUT_MS` (5 s).

---

## Fault detection and engineering messages

The firmware exposes three different diagnostic concepts:

- **V19 — Engineering message:** the latest human-readable runtime event.
- **V20 — System status:** a live bitmask of subsystem health.
- **V34 — Reset diagnostic:** the human-readable ESP32 reset cause recorded at boot.
- **V35–V50 — Performance/I²C diagnostics:** PPS count/age, retained-maximum timings for the loop, sensor task, `Blynk.run()`, GPS feed, I²C sensor path, FAST/SLOW telemetry sends, and PPS→LED response, plus last-I²C-device/operation and a cumulative I²C error count. Counters run continuously; publication to Blynk (and a matching `[PERF]` Serial line) is rate-limited — see the [System Health Diagnostics](README_System_Health_Diagnostics.md) reference for the full pin table.

### V20 status bitmask — current firmware

| Bit | Value | Flag constant | Meaning |
|---:|---:|---|---|
| 0 | `1` | `STATUS_WIFI_OK` | WiFi connected to the local network |
| 1 | `2` | `STATUS_GPS_OK` | GPS position valid with age < 15 s and at least 4 satellites |
| 2 | `4` | `STATUS_IMU_OK` | BMI160 has a recent successful read |
| 3 | `8` | `STATUS_ENS_OK` | ENS160 + AHT2x read succeeded |
| 4 | `16` | `STATUS_CO_FAULT` | CO phase watchdog/invalid CO condition |
| 5 | `32` | `STATUS_GPS_FAULT` | No valid GPS data for more than 15 s |
| 6 | `64` | `STATUS_IMU_STUCK` | No successful BMI160 read for more than 5 s |
| 7 | `128` | `STATUS_ENS_FAULT` | ENS160/AHT2x failed repeatedly |
| 8 | `256` | `STATUS_CO_NO_SNS` | MQ-7 sensor not detected at startup |

A normal fully connected system therefore normally has:

```text
V20 = 15
```

because `1 + 2 + 4 + 8 = 15`.

### V34 — ESP32 reset reason

V34 is updated every 10 seconds with `bootResetReasonText`, which is captured once
at startup from `esp_reset_reason()`. It therefore tells you **why the current ESP32
boot occurred**; it is not a continuously changing health counter.

#### All possible V34 text outputs in the current firmware

| V34 value | Meaning | Typical interpretation |
|---|---|---|
| `UNKNOWN` | Reset reason could not be identified | Investigate if unexpected |
| `POWERON` | Normal power-on reset | Power was applied or the ESP32 restarted from a power-on condition |
| `EXTERNAL` | External reset | Reset pin/external reset circuit was asserted |
| `SOFTWARE` | Software-requested reset | Firmware or another software component requested a restart |
| `PANIC` | ESP32 panic/crash reset | Fatal software exception/panic; investigate Serial output |
| `INT_WDT` | Interrupt watchdog reset | CPU/interrupt handling was blocked too long |
| `TASK_WDT` | Task watchdog reset | A subscribed task failed to feed the task watchdog |
| `WDT` | Other watchdog reset | Watchdog subsystem caused the restart |
| `DEEPSLEEP` | Deep-sleep wake/reset | Reset associated with deep-sleep operation |
| `BROWNOUT` | Brownout reset | Supply voltage dropped below the ESP32 brownout threshold |
| `SDIO` | SDIO reset | Reset caused by the SDIO subsystem |
| `OTHER` | Unrecognised ESP32 reset code | New/unsupported reset reason for this firmware's mapping |

### How to use V34 in the field

```text
POWERON / EXTERNAL / SOFTWARE
    → usually intentional or expected; verify the circumstances.

PANIC / INT_WDT / TASK_WDT / WDT
    → abnormal; inspect Serial diagnostics and the preceding V19 messages.

BROWNOUT
    → investigate power supply, wiring, regulator capacity, bulk capacitance,
      and voltage drops before blaming application software.

UNKNOWN / OTHER
    → investigate the reset history and ESP32/core version.
```

V34 is especially useful for unattended operation: if the station appears to have
restarted overnight, the next boot reports the previous reset classification rather
than leaving only a silent reboot.

### Other runtime diagnostic messages

The following are representative messages generated by the current firmware on
Serial and/or V19. They are event-driven rather than a fixed enumerated V34 list:

```text
[ENG] ESP reset reason: POWERON
[ENG] Setup OK v3.7 — long-term reliability monitoring enabled
[ENG] ENS160+AHT2x: OK
[ENG] BMI160: init OK (±4g, ±500dps, 100Hz)
[ENG] INMP441: I2S mic OK (new i2s_std driver, no ADC conflict)
[ENG] GPS: ATGM336H NMEA 9600 baud — no init needed
[ENG] INAV: waiting for first GPS fix
[ENG] INAV: position acquired by GPS
[ENG] INAV: GPS gap — interpolating from recent GPS+IMU
[ENG] INAV: GPS lost >=3 fixes — interpolating (stale baseline)
[ENG] INAV: GPS lost >5 min — interpolation stopped, holding position
[ENG] GPS: fix lost (...)
[ENG] GPS FAULT: no data >15s
[ENG] IMU STUCK: BMI160 no good read for ...s
[ENG] ENS FAULT: 5 failures — retrying every 10s
[ENG] INMP441 FAULT: no I2S samples — other sensors continue
[ENG] UV FAULT: invalid reference signal — other sensors continue
[ENG] Dust FAULT: ADC input at rail — other sensors continue
[ENG] INMP441: data recovered
[ENG] UV: sensor data recovered
[ENG] Dust: ADC data recovered
[ENG] WiFi: reconnect attempt
[ENG] WiFi FAILED: offline
[ENG] WiFi OK — Blynk configured ...
[ENG] BLYNK FAST: mutex timeout
[ENG] BLYNK SLOW: mutex timeout
[ENG] BLYNK MAP: mutex timeout
[ENG] CO WATCHDOG: phase ... stuck >180s heater=...V
[ENG] CO: sensor absent — check A0 wiring and divider
[ENG] AUTO-RESET: 10-min watchdog
```

> The exact numeric values in messages such as satellite count, elapsed time,
> heater voltage, or CO concentration vary with the live system state.

### Long-term recovery mechanisms

| Mechanism | Current behaviour |
|---|---|
| I²C | `Wire.setTimeOut(50)` bounds I²C operations to 50 ms |
| ENS160/AHT2x | Retries after repeated failures; sensor task continues |
| BMI160 | Retries initialization; persistent read loss raises `STATUS_IMU_STUCK` |
| INMP441 | Five consecutive no-data cycles trigger a persistent diagnostic; recovery is reported |
| UV | Five consecutive invalid reference readings trigger a diagnostic; recovery is reported |
| Dust | Five consecutive rail readings trigger a diagnostic and `SENSOR_UNAVAILABLE` |
| Sensor task | Heartbeat plus explicit task watchdog; watchdog feed occurs after a complete cycle |
| Task watchdog | Explicit **10 s** timeout with panic/reset recovery enabled |
| WiFi | Non-blocking 20 s association attempt, then retry |
| Blynk | Bounded 250 ms TCP server probe and 1 s Blynk I/O timeout; handshake serviced incrementally |
| Buzzer | Non-blocking tone/pattern state machine |
| GPS PPS LED | Dedicated task; one fixed 250 ms D2 pulse per PPS event |

---

## Blynk setup

### Local server — Blynk 0.6.1 (Java 21 patched)

This firmware targets a **self-hosted Blynk 0.6.1 server**, not Blynk Cloud.
Three API differences that were corrected from the previous Mega code:

| | Blynk Cloud | Local server 0.6.1 |
|---|---|---|
| Template defines | `BLYNK_TEMPLATE_ID` required | **Omit entirely** — local server has no template concept |
| `Blynk.config()` | `Blynk.config(auth)` | `Blynk.config(auth, server_ip, port)` |
| Default port | 443 (TLS) | **8080** plain TCP · 8441 TLS |
| `virtualWrite(pin, value, n)` | Not standard API | Not standard API — trailing int removed throughout |

For TLS on the local server, replace `#include <BlynkSimpleEsp32.h>` with
`#include <BlynkSimpleEsp32_SSL.h>` and set `BLYNK_PORT 8441` in `secrets.h`.

### Credentials — `secrets.h`

All sensitive values are stored in a separate `secrets.h` file that lives in the
same sketch folder. **Never commit this file to version control** — add it to
`.gitignore` immediately.

```cpp
// secrets.h
#pragma once

#define WIFI_SSID    "YourNetworkName"
#define WIFI_PASS    "YourPassword"
#define BLYNK_AUTH   "YourBlynkAuthToken"
#define BLYNK_SERVER "192.168.x.x"        // local server IP
#define BLYNK_PORT   8080                 // plain TCP; use 8441 for TLS
```

### Virtual pin reference

| Pin | Signal | Unit / values | Update tier | Notes |
|---|---|---|---|---|
| V1 | Sound level | dBFS estimate | 10 s | INMP441 |
| V2 | CO concentration | ppm | 5 s | Last completed CO measurement |
| V3 | UV index | UVI | 10 s | ML8511 |
| V4 | Diagnostic | integer 1–9 | 10 s | Random activity value; **changing = application/telemetry path is alive** |
| V5 | TVOC | ppb | 10 s | ENS160 |
| V6 | CO heater phase | `0` measuring / `1` heating | 5 s | MQ-7 state machine |
| V7 | CO raw | ADC/EMA counts | 5 s | MQ-7 raw diagnostic |
| V8 | Roll | ° | 10 s | BMI160 complementary filter |
| V9 | Pitch | ° | 10 s | BMI160 complementary filter |
| V10 | Yaw / heading | ° | 10 s | Mirrors INAV heading |
| V11 | BMI160 temperature | °C | 10 s | IMU die temperature |
| V12 | Raw GPS latitude | decimal degrees | 5 s | Raw GPS path |
| V13 | Raw GPS longitude | decimal degrees | 5 s | Raw GPS path |
| V14 | GPS satellites | count | 5 s | GPS fix requires ≥ 4 |
| V15 | Relative humidity | % RH | 5 s | AHT2x |
| V16 | Temperature | °C | 5 s | AHT2x |
| V17 | GPS HDOP | — | 5 s | Lower is generally better |
| V18 | Dust density | mg/m³ | 10 s | GP2Y1010 estimate |
| V19 | Engineering message | string | 5 s | Latest human-readable event |
| V20 | System status | integer bitmask | 5 s | See V20 table above |
| V21 | WiFi RSSI | dBm | 5 s | Signal strength |
| V22 | WiFi quality | % | 5 s | Derived from RSSI |
| V23 | eCO₂ | ppm | 10 s | ENS160 equivalent CO₂, not direct NDIR CO₂ |
| V24 | AQI | `1`–`5` | 10 s | ENS160 gas/VOC air-quality index |
| V25 | MQ-7 heater voltage | V | 10 s | Actual heater voltage from A1 feedback |
| V26 | GPS PPS lock | `0` / `1` | 5 s | `1` = recent 1 Hz PPS interval is valid |
| V27 | INAV state | `0`–`4` | 5 s | See INAV table below |
| V28 | INAV latitude | decimal degrees | 5 s | GPS/IMU fused position |
| V29 | INAV longitude | decimal degrees | 5 s | GPS/IMU fused position |
| V30 | INAV altitude | m | 5 s | Held from last valid GPS altitude; not dead-reckoned |
| V31 | INAV speed | m/s | 5 s | EKF state |
| V32 | INAV heading | ° | 5 s | 0° = north, clockwise |
| V33 | Blynk Map trail | map tuple | 30 s | Uses GPS/INAV position with GPS UTC timestamp |
| **V34** | **ESP32 reset reason** | string | 10 s | **Boot diagnostic; see complete V34 table above** |
| V35 | PPS event count | integer | 10 s | Cumulative GPS 1PPS pulses since boot |
| V36 | Time since last PPS | ms | 10 s | Large value + V26=0 indicates PPS loss |
| V37 | Max `loop()` gap | ms | 10 s | Retained max since boot; Core 1 |
| V38 | Max sensor-task cycle | ms | 10 s | Retained max since boot; Core 0 |
| V39 | Max `Blynk.run()` duration | ms | 10 s | Retained max since boot |
| V40 | Max GPS feed duration | ms | 10 s | Retained max since boot |
| V41 | Max I²C sensor-path duration | ms | 10 s | Retained max since boot; ENS160+AHT2x+BMI160 combined |
| V42 | Max FAST telemetry duration | ms | 10 s | Retained max since boot |
| V43 | Max SLOW telemetry duration | ms | 10 s | Retained max since boot |
| V44 | Max PPS→LED response | ms | 10 s | Retained max since boot |
| V45 | Last I²C device | `0`–`3` | 10 s | `0`=none, `1`=ENS160, `2`=AHT21, `3`=BMI160 |
| V46 | Last I²C operation | `0`–`3` | 10 s | `0`=none, `1`=read, `2`=measurement/command, `3`=configuration |
| V47 | Cumulative I²C error count | integer | 10 s | Retained count since boot |
| V48 | Max AHT21 read duration | ms | 10 s | Retained max since boot |
| V49 | Max ENS160 operation duration | ms | 10 s | Retained max since boot |
| V50 | Max BMI160 raw-read duration | ms | 10 s | Retained max since boot |

> V35–V50 counters run continuously; their Blynk publication (and a matching `[PERF]` Serial line) is rate-limited to every 10 s so the diagnostic channel stays low-overhead. Full detail and a worked stall-diagnosis example are in [System Health Diagnostics](README_System_Health_Diagnostics.md).

### Quick health interpretation

For a normally operating station:

```text
V4  → changing
V20 → 15
V26 → 1
V27 → 0
D2  → blinking once per second when GPS PPS is present
V34 → normally POWERON after a power-up; other values explain the preceding reset
```

---

## Configuration reference

### Feature switches

```cpp
#define DEBUGON           false   // Verbose Serial output — enable for bench testing
#define DISPLAYON         false   // SSD1306 OLED, shares the I2C bus
#define WIFI              true    // Blynk / WiFi connectivity
#define COsensorThere     true    // MQ-7 CO sensor board is physically connected
#define ENSsensorThere    true    // ENS160 + AHT2x combo board
#define INMPsensorThere   true    // INMP441 I2S microphone
#define BMI160sensorThere true    // GY-BMI160 IMU
#define PPSsensorThere    true    // GPS 1PPS input on GPIO5

bool ten_mins_autoreset = false; // Optional periodic ESP.restart() backstop
```

### INAV settings

```cpp
float inav_update_hz          = 10.0f; // INAV prediction/output rate
int   inav_stale_after_misses = 3;     // missed expected fixes before IMU_STALE
int   inav_max_loss_minutes   = 5;     // after this, INAV becomes LOST and holds position
```

### Reliability settings

| Constant | Default | Purpose |
|---|---:|---|
| `BLYNK_TIMEOUT_MS` | 1000 ms | Bounds individual Blynk socket I/O |
| `BLYNK_SERVER_PROBE_TIMEOUT_MS` | 250 ms | Bounds pre-Blynk TCP reachability probe |
| `WIFI_RECONNECT_MS` | 30 s | WiFi retry interval |
| `ENS_RETRY_MS` | 10 s | ENS160/AHT2x retry interval |
| `SENSOR_HEALTH_FAIL_LIMIT` | 5 | Persistent sensor fault threshold |
| `IMU_TIMEOUT_MS` | 5 s | Persistent BMI160 read-loss threshold |
| `SENSOR_TASK_HEALTH_TIMEOUT_MS` | 30 s | Sensor-task heartbeat diagnostic threshold |
| Task watchdog | **10 s** | Explicit sensor-task watchdog timeout |
| `Wire.setTimeOut()` | **50 ms** | I²C operation timeout |
| PPS LED pulse | **250 ms** | D2/GPIO2 indication per PPS event |

### CO calibration

```cpp
float reference_resistor_kOhm   = 9.98;   // Measure your actual resistor with a multimeter
float sensor_reading_clean_air  = 600.65; // ← Set this in clean outdoor air (see section above)
float sensor_reading_100_ppm_CO = -1;     // Optional: raw ADC at a known 100 ppm CO source
```

### Timing constants

| Constant | Default | Purpose |
|---|---:|---|
| `BLYNK_SEND_FAST_MS` | 5 000 ms | Critical/navigation telemetry interval |
| `BLYNK_SEND_SLOW_MS` | 10 000 ms | Environmental/slow telemetry interval |
| `MAP_SEND_INTERVAL_MS` | 30 000 ms | Blynk Map trail point interval |
| `GPS_TIMEOUT_MS` | 15 000 ms | GPS validity timeout |
| `IMU_TIMEOUT_MS` | 5 000 ms | Persistent BMI160 read-loss threshold |
| `CO_PHASE_MAX_MS` | 180 000 ms | Maximum CO phase duration before watchdog recovery |
| `WIFI_RECONNECT_MS` | 30 000 ms | WiFi reconnect interval |
| `ENS_RETRY_MS` | 10 000 ms | ENS160/AHT2x retry interval |
| `SENSOR_TASK_PERIOD_MS` | 25 ms | Sensor-task target cadence |
| `SENSOR_TASK_HEALTH_TIMEOUT_MS` | 30 000 ms | Heartbeat health threshold |
| `BLYNK_TIMEOUT_MS` | 1 000 ms | Individual Blynk I/O timeout |
| `BLYNK_SERVER_PROBE_TIMEOUT_MS` | 250 ms | Bounded Blynk TCP probe |
| `Wire.setTimeOut()` | 50 ms | I²C operation timeout |
| Task watchdog | **10 s** | Explicit sensor-task watchdog timeout |

## Required libraries

Install the following libraries compatible with the ESP32 Arduino core used to
compile the firmware:

| Library | Purpose |
|---|---|
| **Blynk** / `BlynkSimpleEsp32` | Self-hosted Blynk 0.6.1 local-server client |
| **TinyGPS++** | ATGM336H NMEA parsing |
| **ScioSense_ENS160** | ENS160 TVOC/eCO₂/AQI driver |
| **Adafruit AHTX0** | AHT20/AHT21 temperature/humidity driver |
| **Adafruit SSD1306** | Optional OLED display |
| **Adafruit GFX Library** | SSD1306 dependency |

The BMI160 driver and INMP441 I²S implementation are handled directly in the
firmware using the ESP32 IDF/Arduino interfaces; no separate BMI160 library is
required.

## Air quality mapping

With GPS coordinates streaming to V12 and V13 every 5 seconds, the **Blynk Map
widget** plots each reading as a labelled pin on a live map. Configure the widget
with V12 as latitude, V13 as longitude, and V2 (CO ppm) or V18 (dust density) as
the pin label. As the station moves through an urban environment the dashboard
builds a continuous spatial track that can be replayed or exported.

For deeper offline analysis, export the Blynk data CSV and import it into:

- **QGIS** — spatial interpolation, heat-map overlay on OpenStreetMap basemap
- **Google My Maps** — quick shareable visualisation, no software required
- **Python + pandas + folium** — programmatic choropleth maps, scriptable pipelines

---

## Known limitations and future work

**MQ-7 humidity sensitivity**
The MQ-7 resistance changes with ambient humidity and temperature. AHT2x data is
available in firmware, but the current CO ppm calculation does not apply a full
humidity/temperature compensation model. Treat CO ppm as an indicative estimate
unless calibrated against a suitable reference.

**ENS160 eCO₂ is equivalent CO₂**
ENS160 `eCO2` is an equivalent-CO₂ estimate derived from the gas/VOC response; it
is not a direct NDIR CO₂ measurement. ENS160 AQI is likewise a gas/VOC-oriented
index, not a PM2.5/PM10 regulatory AQI.

**ENS160 gas cross-sensitivity**
TVOC, eCO₂ and AQI are processed gas-sensor outputs. Individual gas estimates
should not be interpreted as laboratory-grade concentrations without appropriate
calibration and environmental control.

**GPS cold-start latency**
The ATGM336H may need tens of seconds to acquire its first outdoor fix. Until a
valid fix exists, INAV remains `NO_FIX_YET`.

**INAV dead reckoning**
INAV is deliberately conservative. It stops advancing the estimated position after
`inav_max_loss_minutes` and enters `LOST`, holding the last position rather than
allowing unbounded drift.

**Reset cause is historical, not a live alarm**
V34 reports the reset reason captured at boot. If the device has not rebooted, V34
continues to show the same value. Use V19/V20/V26/V27 for current operating health.

**Power integrity remains critical**
Firmware watchdogs can recover many software/sensor stalls, but they cannot prevent
brownouts caused by an inadequate regulator, wiring, connectors, WiFi current
spikes, MQ-7 heater transients, or insufficient bulk capacitance. A `BROWNOUT` value
on V34 should therefore trigger a hardware power investigation first.

**10-minute auto-reset is optional**
`ten_mins_autoreset` defaults to `false`. The explicit task watchdog and sensor-task
heartbeat provide fault recovery without requiring periodic scheduled reboots.

---

*Firmware: `AirQualityOutdoor_ESP32_Blynk(5).ino` v3.7 · Credentials: `secrets.h` · Backend: self-hosted Blynk 0.6.1 local server*

---

## System Health Diagnostics

The most useful unattended-operation indicators are:

| Indicator | Normal value / behaviour | Meaning |
|---|---|---|
| **V4** | Changes over time | Application/telemetry path is alive |
| **V20** | `15` | WiFi + GPS + BMI160 + ENS160/AHT2x healthy |
| **V26** | `1` | GPS 1PPS is locked |
| **V27** | `0` | INAV currently has a GPS fix |
| **D2 LED** | One 250 ms flash per second | Physical GPS PPS events are reaching the ESP32 |
| **V34** | Reset-cause text | Explains the reset that produced the current boot |
| **V35–V50** | Retained-max timings, PPS count, I²C error count | Detailed performance/I²C diagnostics for tracking down stalls |

### V4 vs V34

These two diagnostics answer different questions:

- **V4 changing:** *Is the application/telemetry path still running?*
- **V34 value:** *Why did the ESP32 last boot/restart?*

V34 is therefore expected to remain unchanged during normal continuous operation.
It changes only after the ESP32 boots again.

For the complete V34 reset-cause table, the V35–V50 performance/I²C diagnostics
(including a worked stall-diagnosis example and the periodic `[PERF]` Serial
report), see the [System Health Diagnostics](README_System_Health_Diagnostics.md)
reference if it is kept alongside this README.
