# Outdoor Air Quality Station

> *A portable environmental monitor that logs CO, NO₂, UV, particulate matter,
> sound, temperature, humidity, and orientation — with live GPS coordinates
> streaming to a self-hosted Blynk dashboard for real-time air-quality mapping.*

**ESP32 · Arduino-ESP32 Core 3.3.11 · FreeRTOS dual-core · Blynk 0.6.1 local server · ATGM336H GPS · 8 sensing subsystems**

---

## Table of contents

1. [The story](#the-story)
2. [Sensor suite](#sensor-suite)
3. [Latest firmware update — v3.9.52 / Core 3.3.11](#latest-firmware-update--v3952--core-3311)
4. [Why the hardware changed](#why-the-hardware-changed)
5. [Pin mapping](#pin-mapping)
6. [Wiring and voltage dividers](#wiring-and-voltage-dividers)
7. [Firmware architecture](#firmware-architecture)
8. [CO sensor — calibration and state machine](#co-sensor--calibration-and-state-machine)
9. [GPS initialisation](#gps-initialisation)
10. [IMU protocol](#imu-protocol)
11. [Fault detection and engineering messages](#fault-detection-and-engineering-messages)
12. [Blynk setup](#blynk-setup)
13. [V55 remote hard reset](#v55-remote-hard-reset)
14. [Blynk server systemd service](#blynk-server-systemd-service)
15. [Configuration reference](#configuration-reference)
16. [Required libraries](#required-libraries)
17. [Air quality mapping](#air-quality-mapping)
18. [Known limitations and future work](#known-limitations-and-future-work)
19. [System Health Diagnostics](#system-health-diagnostics)

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

## Latest firmware update — v3.9.52 / Core 3.3.11

The current sketch is:

`AirQualityOutdoor_ESP32_Blynk_v3.9.52_ATGM336_PCAS_GPS_Core3.3.11.ino`

It is specifically migrated for **Arduino-ESP32 Core 3.3.11**. The migration is intentionally minimal: application behaviour, sensor logic, pins, timing, GPS policy, Blynk transport, WiFi recovery, I²C recovery, INAV, diagnostics, and scheduling are preserved. Only APIs that changed between Arduino-ESP32 2.x and 3.x were migrated.

### Arduino-ESP32 Core 3.3.11 requirements

- Install **ESP32 by Espressif Systems / Arduino-ESP32 Core 3.3.11**.
- Do **not** compile this sketch with the old Core 2.0.17 package.
- The sketch contains a compile-time guard that rejects Arduino-ESP32 versions below 3.3.x.
- Core 3.3.x uses the ESP-IDF 5.x driver/API generation required by this sketch.

### Core 3.x API migrations

**LEDC / PWM**

The old 2.x channel-first API was removed in Core 3.x. The sketch now uses the pin-based 3.x API:

- `ledcAttachChannel()`
- `ledcChangeFrequency(pin, ...)`
- `ledcWrite(pin, duty)`
- `ledcDetach(pin)`

The original explicit LEDC channel separation is retained:

- **Channel 0 / group 0:** MQ-7 CO heater PWM
- **Channel 8 / group 1:** buzzer

LEDC access is protected by the existing cross-core mutex because the CO heater is serviced by the sensor task while the buzzer is serviced from the application side.

**INMP441 / I²S**

The microphone was migrated from the legacy `driver/i2s.h` API to the **ESP-IDF 5.x standard I²S driver**:

`driver/i2s_std.h`

The electrical configuration is unchanged:

- BCLK GPIO25
- WS/LRCLK GPIO26
- DATA GPIO33
- 16 kHz
- 32-bit stereo capture

The existing two-slot RMS processing is retained. This migration is important because the old legacy I²S driver could conflict with the modern ADC driver used by Arduino-ESP32 3.x. The standard I²S driver avoids that legacy-driver/ADC collision.

**Task watchdog**

The old Core-2.x call:

`esp_task_wdt_init(10, true)`

was migrated to the Core-3.x / ESP-IDF 5.x `esp_task_wdt_config_t` API while retaining:

- 10-second timeout
- panic/reset action
- explicit sensor-task watchdog subscription
- watchdog feed only after a completed sensor cycle

### GPS — ATGM336H / AT6558 PCAS configuration

The GPS is **not configured with PMTK commands** in the current firmware. Field testing showed that this ATGM336H/AT6558 receiver responds to the **PCAS** command family instead.

The firmware uses:

- `PCAS03` — NMEA sentence selection
- `PCAS01` — UART baud rate
- `PCAS02` — update interval
- `PCAS00` — save configuration to flash

The intended persistent configuration is:

- **115200 baud**
- **10 Hz / 100 ms update interval**
- **GGA + RMC only**

The checksums used by the sketch are:

```text
PCAS03,1,0,0,0,1,0,0,0*02
PCAS01,5*19
PCAS02,100*1E
PCAS00*01
```

The `PCAS02,100` checksum is deliberately `*1E`; a reference that lists `*1D` for that exact command is incorrect under the standard XOR checksum calculation.

### GPS persistence policy — important

The firmware deliberately avoids unnecessary flash writes.

1. GPS auto-baud detection runs at startup.
2. If the GPS is detected at **9600 baud**, it is treated as the known factory/default state and the firmware performs the migration:
   - reduce NMEA output to GGA+RMC,
   - request 115200 with `PCAS01`,
   - verify real NMEA reception at 115200,
   - request 10 Hz with `PCAS02`,
   - save once with `PCAS00`.
3. If the GPS is detected at **any baud above 9600**, it is treated as already configured:
   - no `PCAS03`,
   - no `PCAS01`,
   - no `PCAS02`,
   - **no `PCAS00`**.
4. If the 9600 → 115200 migration cannot be verified, the firmware falls back to the confirmed working baud and **does not save** the incomplete configuration.
5. An unexpected baud below 9600 is left untouched rather than being reconfigured blindly.

This means `PCAS00` is a one-time persistence operation for the deliberate 9600 → 115200 + 10 Hz migration, not a flash write on every ESP32 boot.

### GPS UART and PPS

The ATGM336H is connected as:

- GPS TX → ESP32 GPIO13 (UART1 RX)
- ESP32 GPIO23 (UART1 TX) → GPS RX
- GPS 1PPS → GPIO5

The UART RX buffer is enlarged before `begin()` to tolerate bursts of NMEA traffic while the sensor task is busy.

PPS remains independent of the NMEA stream. The PPS interrupt records the event/timing state and the dedicated PPS task generates one fixed **250 ms pulse** on the ESP32 board D2/GPIO2 LED for each PPS event.

**V26** reports PPS lock; the LED is event-driven and does not remain continuously ON merely because PPS is locked.

### Existing reliability fixes retained in v3.9.52

The Core-3.3.11 migration does not remove the reliability work developed in the earlier firmware versions:

- reset-cause diagnostics via V34
- sensor-task heartbeat
- explicit task watchdog
- bounded I²C transactions
- bounded ENS160 measurement waiting
- I²C recovery and diagnostics
- BMI160 health monitoring
- INMP441/UV/dust persistent fault detection and recovery reporting
- non-blocking WiFi reconnect state machine
- disabled duplicate WiFi auto-reconnect
- bounded/non-blocking Blynk TCP probing
- raw non-blocking Blynk transport
- Blynk TX pacing and post-reconnect grace period
- Blynk transport diagnostics
- conservative INAV GPS-loss/dead-reckoning state machine
- asynchronous buzzer state machine
- V55 remote `esp_restart()` recovery

See the detailed historical changelog inside the `.ino` for the reasoning and field evidence behind these changes.

## Why the hardware changed

| Problem on the Mega + ESP8266 | Solution on the ESP32 |
|---|---|
| Single `loop()` — all subsystems compete for time | Two FreeRTOS tasks pinned to separate cores |
| Separate ESP8266 WiFi module on UART3 | Built-in WiFi — external module removed entirely |
| 10-bit ADC at 5 V | 12-bit ADC at 3.3 V (ADC1 only with WiFi active) |
| Timer2 registers for CO PWM — AVR-specific | LEDC peripheral — Core 3.x pin-based API |
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

The current GPS is an **ATGM336H / AT6558** using NMEA output. The firmware auto-detects the working UART baud and then applies the PCAS policy described in the [Latest firmware update](#latest-firmware-update--v3952--core-3311).

The intended persistent configuration is:

```text
UART:      115200 baud
Update:    10 Hz / 100 ms
Sentences: GGA + RMC
```

**Do not replace the PCAS commands with PMTK commands.** The current receiver was field-tested and the working configuration path is PCAS-based.

The firmware verifies a baud change using **real NMEA reception** at the new speed rather than relying on a PMTK-style ACK. `115201` may occasionally appear in diagnostic output when 115200 is configured; this is normal ESP32 UART clock-divider rounding and is not a separate GPS setting.

### GPS auto-detection and persistence

- **Detected 9600:** perform the controlled migration to 115200 + 10 Hz and save with PCAS00 only after successful verification.
- **Detected >9600:** assume the GPS is already configured; leave it alone and do not issue PCAS configuration or save commands.
- **Detected <9600 but not the expected 9600 state:** leave the receiver untouched.
- **Migration verification failure:** return to the known working baud; do not issue PCAS00.

### PPS lock criteria

V26 reports PPS lock as:

```text
0 = not locked
1 = locked
```

A PPS lock requires a pulse within the last 2 seconds and a measured PPS interval between **0.900 s and 1.100 s**.

The LED is event-driven: every rising-edge PPS event produces a fixed **250 ms** D2/GPIO2 pulse. It does not stay on continuously while PPS is locked.

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
[ENG] Setup OK v3.9.52 — Arduino-ESP32 Core 3.3.11
[ENG] ENS160+AHT2x: OK
[ENG] BMI160: init OK (±4g, ±500dps, 100Hz)
[ENG] INMP441: I2S mic OK (new i2s_std driver, no ADC conflict)
[ENG] GPS: ATGM336H NMEA/PCAS — auto-baud and persistence policy active
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

## V55 remote hard reset

The firmware provides **Blynk virtual pin V55** as a remote ESP32 restart control. Configure
a Blynk **Button** widget on V55; **Push** mode is recommended. When the widget sends
`1`, the firmware calls `esp_restart()`.

### V55 setup

| Setting | Value |
|---|---|
| Virtual pin | **V55** |
| Widget | **Button** |
| Mode | **Push** recommended |
| ON / pressed value | `1` |
| Purpose | Remote ESP32 software restart |

The V55 reset is intentionally performed directly from the `BLYNK_WRITE(V55)` callback.
Interrupts remain enabled and the firmware does **not** call `noInterrupts()` before
`esp_restart()`. Disabling interrupts immediately before the ESP32 restart sequence
can prevent the restart machinery from completing and trigger an **Interrupt-WDT**
panic.

A V55 restart is a **software reset**, so after reboot V34 should report:

```text
SOFTWARE
```

V55 is useful both for remote recovery and for controlled restart testing. Trigger V55,
wait for the station to return, and then check V34 and the startup Serial diagnostics.

### Why V55 was added

V55 was added as a diagnostic/recovery control while investigating freezes during
unattended operation. It provides a way to restart the ESP32 remotely without
physically pressing the reset button or removing power. It does not replace the task
watchdog or sensor fault-recovery mechanisms.

## Blynk server systemd service

The station uses a **self-hosted Blynk 0.6.1 server** on the Raspberry Pi. The ESP32 firmware contains its own bounded/non-blocking Blynk recovery logic, but the server-side Java process must also be configured with enough memory and appropriate G1GC settings.

Field diagnostics identified intermittent Blynk socket stalls as a **server-side JVM allocation/GC pressure issue**, not an ESP32 sensor or WiFi fault. The following `blynk.service` configuration was the server-side stability fix that was applied and field-confirmed.

> Keep this unit in sync with the actual server deployment. The `server-0.41.18-java21.jar` filename must match the installed Blynk server JAR.

```ini
[Unit]
Description=Blynk Server
After=network-online.target
Wants=network-online.target

StartLimitIntervalSec=120
StartLimitBurst=5
# StartLimitAction=none means no additional action beyond the restart logic below.
StartLimitAction=none

[Service]
Type=simple

User=ptut
WorkingDirectory=/opt/blynk

ExecStart=/usr/bin/java \
    -server \
    -Xms128m \
    -Xmx700m \
    -Xss512k \
    -XX:+UseG1GC \
    -XX:G1HeapRegionSize=4m \
    -XX:MaxGCPauseMillis=300 \
    -XX:InitiatingHeapOccupancyPercent=45 \
    -XX:+ExplicitGCInvokesConcurrent \
    -XX:+UseStringDeduplication \
    -XX:+ExitOnOutOfMemoryError \
    -XX:NativeMemoryTracking=summary \
    -Xlog:gc*:file=/opt/blynk/gc.log:time,uptime,level,tags:filecount=5,filesize=10M \
    -Djava.security.egd=file:/dev/urandom \
    --add-opens=java.base/java.lang=ALL-UNNAMED \
    -jar /opt/blynk/server-0.41.18-java21.jar \
    -dataFolder /opt/blynk

Restart=always
RestartSec=15

StandardOutput=journal
StandardError=journal
SyslogIdentifier=blynk
# Prevent journal flooding from repeated log lines.
LogRateLimitIntervalSec=30
LogRateLimitBurst=1000

MemoryHigh=650M
MemoryMax=800M
MemorySwapMax=0
# Lower OOM score = killed later. Kill Blynk before other critical services under memory pressure.
OOMScoreAdjust=200
TasksMax=128
Nice=5

# Hardening
NoNewPrivileges=true
# ProtectSystem=strict was previously "full"; strict is more restrictive.
ProtectSystem=strict
ProtectHome=true
ReadWritePaths=/opt/blynk
PrivateTmp=true

# these added for security
PrivateDevices=true
ProtectKernelTunables=true
ProtectKernelModules=true
ProtectControlGroups=true
RestrictNamespaces=true
RestrictSUIDSGID=true
LockPersonality=true

[Install]
WantedBy=multi-user.target
```

### Why these Blynk server settings matter

The important stability changes are:

| Setting | Value | Purpose |
|---|---:|---|
| `-Xms` | 128 MB | Conservative initial Java heap |
| `-Xmx` | 700 MB | More heap headroom than the previous 500 MB setting |
| `G1HeapRegionSize` | 4 MB | Valid power-of-two G1 region size; reduces humongous-object pressure compared with the previous 3 MB setting |
| `InitiatingHeapOccupancyPercent` | 45 | Makes G1 mixed-cycle initiation explicit |
| `ExplicitGCInvokesConcurrent` | enabled | Avoids unnecessarily disruptive full-heap collection behaviour |
| `UseStringDeduplication` | enabled | Reduces duplicate String memory |
| `MaxGCPauseMillis` | 300 ms | Bounds the G1 pause target |
| `MemoryHigh` | 650 MB | systemd memory-pressure threshold |
| `MemoryMax` | 800 MB | Hard memory ceiling with headroom above `-Xmx` |
| `MemorySwapMax` | 0 | Prevents Blynk from moving its memory workload into swap |
| `Restart=always` | enabled | Restarts the service after an unexpected process exit |
| `RestartSec` | 15 s | Avoids an immediate restart loop |
| `OOMScoreAdjust` | 200 | Makes Blynk more expendable than services with a lower OOM score |
| `TasksMax` | 128 | Limits process/thread count |
| `Nice` | 5 | Gives the Blynk process a slightly lower CPU scheduling priority |
| systemd hardening | enabled | Restricts unnecessary kernel/device/namespace capabilities |

The earlier server-side tuning also used explicit `ConcGCThreads=1`, `ParallelGCThreads=4`, and `G1ReservePercent=15` during the investigation. The current unit above is the **actual unit supplied for the stabilized deployment**; do not add those older investigation flags unless deliberately testing another configuration.

### Applying the service

After changing the unit:

```bash
sudo systemctl daemon-reload
sudo systemctl restart blynk
sudo systemctl status blynk
```

For live service logs:

```bash
journalctl -u blynk -f
```

For the G1GC log:

```bash
tail -f /opt/blynk/gc.log
```

The ESP32 firmware should continue to use the local Blynk server address and port defined in `secrets.h`; the current sketch calls `Blynk.config(BLYNK_AUTH, BLYNK_SERVER, BLYNK_PORT)` and deliberately avoids `Blynk.connect(timeout)` because the latter can block Core 1 long enough to trigger the ESP32 Interrupt WDT.

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
| `ENS_MEASURE_TIMEOUT_MS` | 1.5 s | Maximum wait for ENS160 new-data polling |
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

Install **Arduino-ESP32 Core 3.3.11** first, then install the following libraries compatible with that core:

| Library | Purpose |
|---|---|
| **Blynk** / `BlynkSimpleEsp32` | Self-hosted Blynk 0.6.1 local-server client |
| **TinyGPS++** | ATGM336H NMEA parsing |
| **ScioSense_ENS160** | ENS160 TVOC/eCO₂/AQI driver |
| **Adafruit AHTX0** | AHT20/AHT21 temperature/humidity driver |
| **Adafruit SSD1306** | Optional OLED display |
| **Adafruit GFX Library** | SSD1306 dependency |
| **Arduino-ESP32 Core 3.3.11** | ESP32 board package, FreeRTOS, WiFi, UART, ADC, LEDC and ESP-IDF 5.x interfaces |

The BMI160 driver is handled directly in the firmware using register-level I²C; no separate BMI160 library is required. The INMP441 uses the ESP-IDF 5.x standard I²S driver (`driver/i2s_std.h`) supplied by Arduino-ESP32 Core 3.3.11.

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

### Core 3.3.11 compatibility

This README and the current v3.9.52 sketch describe the **Arduino-ESP32 Core 3.3.11** build. Historical changelog entries inside the `.ino` may mention Core 2.0.17 and legacy APIs; those entries document previous investigation and are not instructions for the current build. The active v3.9.52 code uses the Core-3.x LEDC API, ESP-IDF 5.x standard I²S driver, and ESP-IDF 5.x task-WDT configuration.


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

*Firmware: `AirQualityOutdoor_ESP32_Blynk` v3.9.52 · Arduino-ESP32 Core 3.3.11 · Credentials: `secrets.h` · Backend: self-hosted Blynk 0.6.1 local server*

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
