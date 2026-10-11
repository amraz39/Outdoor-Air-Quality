# Outdoor Air Quality Station — ESP32

Current firmware: **v3.9.213**
Current Windows dashboard: **v1.9** (`aq_dashboard.py`)
Target board package: **Arduino-ESP32 3.3.12**
Backend: **self-hosted Blynk 0.41.18 / legacy Blynk protocol 0.6.1**
Primary sketch: `AirQualityOutdoor_ESP32_Blynk.ino`

This project is a dual-core ESP32 outdoor air-quality and navigation station. It acquires GPS, environmental, inertial and acoustic data while maintaining a resilient local connection to a Raspberry Pi 5 Blynk server.

The current design deliberately separates sensor acquisition from networking. It also separates bulk V61/V62 records from the ordinary Blynk hardware socket so that a large string upload cannot block Blynk control, heartbeat or dashboard traffic.

## Current status

v3.9.213 is the current validated build. The important behavior is:

- Core 0 exclusively owns WiFi, lwIP-facing TCP work, `Blynk.run()`, the coherent HTTP client and physical Serial output.
- Core 1 owns sensor initialization, sensor sampling, GPS parsing, IMU/INAV processing and immutable snapshot production. Core 1 never opens or drives a TCP socket.
- V61 and V62 are produced as one coherent pair from the same snapshot and uploaded every second through `PUT /{token}/telemetry/coherent`.
- Ordinary Blynk virtual pins use a coalescing, paced queue. Stale values are replaced instead of creating an unbounded backlog.
- V4 is a small random-looking, non-repeating application heartbeat queued once per second. It does not read or lock the sensor snapshot.
- Exact Blynk PING responses and application checkpoints provide end-to-end receive-path evidence.
- HTTP is paused immediately when the exact Blynk PING becomes overdue. It resumes only after a 60-second stable quiet window.
- A Blynk-only sequence hole receives socket-first recovery. A sequence hole corroborated by an independent recent HTTP transport failure is treated as a shared TCP/lwIP-path failure and performs one bounded full MCU restart.
- WiFi teardown is not used as a generic Blynk repair. WiFi recovery is reserved for actual association/IP failures.
- The optional ten-minute periodic reset remains disabled.

The v3.9.211 field trace showed one proven shared-path reset, a return to Blynk in about 14 seconds, and continued operation without repeated Blynk restarts or WiFi teardowns. v3.9.212 refined reset-domain corroboration and startup behavior; v3.9.213 adds the observable one-second V4 heartbeat used by dashboard v1.9.

## Why v3.9.211 changed recovery scope

Earlier firmware repeatedly restarted only the Blynk protocol object after the established stream became one-way. That did not help when both independent transports were already affected:

1. coherent HTTP uploads timed out;
2. the established Blynk socket stopped receiving an exact PING response;
3. replacement Blynk TCP connections also timed out or authenticated into another one-way session;
4. a full MCU reset immediately restored service.

v3.9.211 therefore uses corroboration rather than timing alone:

- **Blynk failure only:** rotate only the owner-managed Blynk socket and leave WiFi intact.
- **HTTP failure only:** open the HTTP circuit breaker; do not reset Blynk, WiFi or the MCU.
- **Exact Blynk sequence hole plus recent HTTP transport evidence:** restart the complete MCU reset domain once, without first spending time on the already-disproved Blynk-only and WiFi-teardown paths.

This is prevention-oriented: application writes stop early, HTTP is isolated at the first overdue exact PING, and recovery is selected from observed failure scope.

## Task and ownership model

```text
Core 0 — communications owner
  Arduino setup()/loop()
  ESP32 WiFi events and recovery state machine
  Blynk protocol, PINGs, callbacks and paced virtual writes
  Persistent nonblocking HTTP client for V61/V62
  Serial output and network health diagnostics

Core 1 — sensor and processing owner
  ADC, I2C, I2S and UART sensor initialization
  ENS160/AHT2x, BMI160, INMP441, GPS and optional analogue sensors
  GPS/IMU inertial-navigation processing
  Immutable SensorData and V61/V62 snapshot production
  No WiFiClient, HTTP or Blynk calls

Cross-core communication
  One-slot overwrite queues carry the newest immutable snapshots
  Diagnostic/log queues are nonblocking
  A short mutex protects shared sensor data
```

Build-time checks reject configurations where the Arduino loop, Arduino events, ESP32 WiFi task or lwIP TCP/IP task are not assigned to Core 0.

The two cores monitor each other's progress. Recovery requests are cooperative; an unresponsive task cannot be independently restarted safely because ESP-IDF driver and heap ownership crosses task boundaries. If a cooperative recovery is not acknowledged, the final safe recovery is a full MCU reset.

## Data paths

### Blynk control and dashboard socket

The persistent Blynk 0.6.1 hardware socket carries:

- Blynk login, INFO and protocol PING/RESPONSE traffic;
- application callbacks, including V55 remote restart;
- V4 application-liveness value;
- dashboard pins V1–V60 and V63–V65 at paced, round-robin rates;
- V33 map updates.

The queue sends at most one application item per pass and normally spaces writes by 500 ms. Weak links use a slower interval. Application writes pause when inbound responses become stale, but `Blynk.run()` and exact PING recovery continue.

### Coherent V61/V62 HTTP stream

Core 1 seals both strings from the same snapshot. Core 0 uploads the newest available pair once per second as a JSON array:

```http
PUT /<token>/telemetry/coherent HTTP/1.1
Connection: keep-alive
Content-Type: application/json

["<V61 CSV>","<V62 CSV>"]
```

V61 fields:

```text
gpsUnix,deviceMs,lat,lng,sats,hdop,gpsFix,co,coRaw,heaterV,
coPhase,coFault,coNoSensor,tvoc,eCO2,aqi,uvi,sound,dust
```

V62 fields:

```text
gpsUnix,deviceMs,temp,hum,ensFault,roll,pitch,yaw,imuTemp,imuOk,
pps,inavState,inavLat,inavLng,inavAlt,inavSpeed,inavHeading,status,rssi,wifiQuality
```

The server stores both values under one request timestamp and does not reflect them back to the hardware connection. The HTTP client reuses a connection for at most 60 requests, then rotates it preventively. Its transaction timeout is 5 seconds; repeated failures open a 90-second circuit breaker. Blynk health always has priority over bulk HTTP.

## Patched Blynk library

The project is updated for a locally patched **Blynk 0.6.1** client library. `BLYNK_LIB` intentionally contains only the files changed by this project:

```text
BLYNK_LIB/src/Adapters/BlynkArduinoClient.h
BLYNK_LIB/src/Blynk/BlynkProtocol.h
```

The protocol patch adds:

- transport service and pending-TX hooks used by the ESP32 deferred writer;
- an authoritative last-inbound timestamp;
- exact tracked protocol PING ID, age, sent count and verified count;
- bounded application ACK checkpoints;
- protection against the field-observed 128-byte application-write boundary;
- transport-aware heartbeat decisions that do not mistake a deferred write for a dead peer;
- accessors used by the recovery telemetry and isolation gate.

These behaviors are guarded by `BLYNK_TRANSPORT_AWARE` and `BLYNK_APPLICATION_ACK_BARRIER`, which are defined by this ESP32 sketch. An ESP8266 sketch that does not define those macros retains the normal library behavior. This avoids changing the operating behavior of the existing ESP8266 sensor.

Before replacing installed library files, back them up. Copy the two files to the matching locations below the installed Blynk library `src` directory. Do not copy the whole project folder over the library.

Typical Windows installation:

```text
C:\Users\<user>\Documents\Arduino\libraries\Blynk\src\Adapters\BlynkArduinoClient.h
C:\Users\<user>\Documents\Arduino\libraries\Blynk\src\Blynk\BlynkProtocol.h
```

## Arduino-ESP32 requirements

Use Arduino-ESP32 **3.3.12** with:

- board: ESP32 Dev Module or the matching classic ESP32 target;
- flash mode: DIO;
- Arduino loop: Core 0;
- Arduino events: Core 0;
- WiFi/lwIP task affinity: Core 0, as supplied by the selected 3.3.12 configuration;
- partition large enough for the approximately 1.06 MB application.

The packaged v3.9.211 application image validates as ESP32/DIO with a valid checksum and validation hash. A 40 MHz flash setting is suitable for conservative board compatibility; rebuilding at another flash frequency produces a different binary and should be treated as a separate test build.

`ESP32_CORE_PATCH/3.3.12/libraries/WiFi/src/STA.cpp` contains the single targeted Arduino-ESP32 patch retained by this project: authentication-expiry events set `WL_CONNECTION_LOST`. It affects ESP32 Core 3.3.12 only and has no effect on ESP8266. Back up the installed core file before applying it.

## Required libraries

| Component | Purpose |
|---|---|
| Arduino-ESP32 3.3.12 | ESP32, FreeRTOS, WiFi/lwIP, ADC, UART, LEDC and ESP-IDF 5.x drivers |
| Blynk 0.6.1 plus the two project patches | Local-server protocol and transport health evidence |
| TinyGPS++ | ATGM336H NMEA parsing |
| ScioSense ENS160 | TVOC, eCO2 and AQI |
| Adafruit AHTX0 | Temperature and humidity |
| Adafruit GFX + SSD1306 | Optional OLED |

BMI160 is driven directly through I2C registers. INMP441 uses the ESP-IDF 5.x standard I2S driver.

## Credentials

Create `secrets.h` beside the INO and do not commit it:

```cpp
#pragma once
#define WIFI_SSID    "YourNetworkName"
#define WIFI_PASS    "YourPassword"
#define BLYNK_AUTH   "YourLegacyBlynkToken"
#define BLYNK_SERVER "192.168.3.4"
#define BLYNK_PORT   8084
```

This firmware targets the self-hosted legacy server, not current Blynk Cloud. Do not add Blynk IoT template definitions.

## Windows dashboard (`aq_dashboard.py`)

Dashboard v1.9 is a read-mostly Windows monitoring application for the local legacy Blynk server. It displays the current sensor/navigation values, the coherent V61/V62 record, diagnostics, maps and bounded history graphs. All Tk widget changes are applied on the main GUI thread; independent HTTP workers have their own `requests.Session` instances.

Install the required packages:

```powershell
python -m pip install requests customtkinter matplotlib numpy tkintermapview
```

Run the dashboard from the project directory so it can read the same `secrets.h` used by the firmware:

```powershell
cd C:\Users\PTuT\Documents\Arduino\AirQualityOutdoor_ESP32_Blynk
python .\aq_dashboard.py
```

### Connection-state rules

`CONNECTED` is deliberately a two-factor state. Both conditions must be true:

1. the Blynk `isHardwareConnected` endpoint reports that the ESP32 hardware session is connected; and
2. the V4 application heartbeat is actively advancing.

The firmware queues a non-repeating V4 value once per second. The dashboard polls V4 once per second and continuously keeps a moving three-arrival window. The first value is a baseline and the next two must arrive within four seconds to confirm the live stream. After confirmation, V4 evidence expires 3.5 seconds after the last observed transition, so a powered-off or stalled ESP32 normally changes from green `CONNECTED` to yellow `NO LIVE DATA` on the next approximately four-second health check even if the server has not yet closed the hardware session.

An HTTP GET can return a cached V-pin value after the device stops. Receiving an HTTP response is therefore not heartbeat evidence; the observed V4 value must advance. V61/V62 is used as fallback liveness evidence only when the V4 HTTP read itself is unavailable. A readable but stale V4 cannot be masked by the slower V61/V62 path.

Dashboard states:

| Display | Meaning |
|---|---|
| `CONNECTED` | Blynk reports the ESP32 connected and live telemetry is confirmed. |
| `NO LIVE DATA` | Blynk still reports a hardware session, but V4 is no longer advancing. |
| `DEVICE OFFLINE` | Blynk explicitly reports the ESP32 hardware session disconnected. |
| `SERVER UNAVAILABLE` | The dashboard cannot obtain a valid Blynk hardware-status response. |

Internal evidence such as `Blynk + V4 heartbeat` is used by the worker but is not displayed while the dashboard is connected. The header shows only the most recent live-telemetry time.

### Dashboard refresh rates

| Dashboard activity | Rate | Scope |
|---|---:|---|
| V4 health poll and `DIAG (App/Telem. RND# Alive)` | 1 second | This is the only data widget redrawn by the one-second path. |
| Full virtual-pin data sweep | 5 seconds plus HTTP duration | All ordinary sensor, navigation, message and diagnostic widgets. |
| Live graph drawing | 5 seconds | Redraws already cached samples. |
| Server history refresh | 60 seconds | Retrieves historical graph data. |

V4 is intentionally removed from the five-second full-data sweep to avoid duplicate reads and competing widget updates. Other firmware publication rates remain independent; a dashboard poll reads the newest value cached by Blynk and does not imply that every intermediate publication was observed.

### Coherent messages and reset control

V61 and V62 are read independently by the HTTP API. The dashboard caches both halves and changes the two message widgets only when their embedded `(gpsUnix, deviceMs)` keys match. It never intentionally displays a mixed record.

The remote-reset button sends V55 as `0 -> 1 -> 0`. Firmware requires the leading LOW to arm the edge and blocks V55 reset requests during the first 30 seconds after boot. This prevents a retained or replayed HIGH value from causing a reset loop.

`secrets.h` contains WiFi and Blynk credentials. Keep it out of source control and do not include credentials in screenshots, logs or bug reports.

## Active feature defaults

```cpp
#define DEBUGON          false
#define DISPLAYON        false
#define WIFI             true
#define WIFI_SLEEP       false
#define COsensorThere    false
#define DUSTsensorThere  false
#define ENSsensorThere   true
#define INMPsensorThere  true
#define BMI160sensorThere true
#define PPSsensorThere   true

bool ten_mins_autoreset = false;
```

Do not enable an absent analogue sensor. Floating ADC inputs must never be interpreted as a connected MQ-7 or dust sensor.

## Sensor suite and pin mapping

| Sensor or signal | ESP32 pins | Interface | Current default |
|---|---|---|---|
| ENS160 | SDA 21, SCL 22, address 0x53 | I2C | enabled |
| AHT2x | SDA 21, SCL 22, address 0x38 | I2C | enabled |
| BMI160 | SDA 21, SCL 22, address 0x69/0x68 | I2C | enabled |
| ATGM336H GPS | RX 13, TX 23, PPS 5 | UART1 + interrupt | enabled |
| INMP441 | BCLK 25, WS 26, data 33 | I2S | enabled |
| PPS indicator LED | GPIO 2 | digital output | enabled |
| MQ-7 sense / heater feedback | GPIO 36 / 39 | ADC1 through divider | disabled |
| MQ-7 heater PWM | GPIO 4 | LEDC | disabled |
| GP2Y1010 analogue / LED | GPIO 32 / 18 | ADC1 + digital output | disabled |
| ML8511 OUT / REF | GPIO 34 / 35 | ADC1 | firmware input path retained |
| Buzzer | GPIO 19 | LEDC | enabled when required |
| CO status LEDs | GPIO 14, 27, 15 | digital outputs | disabled with MQ-7 |

All ADC inputs use 12-bit resolution and per-pin 11 dB attenuation. The 5 V analogue inputs require the configured 10 kΩ / 10 kΩ divider (`divider_scale = 2.0`). Never apply 5 V directly to an ESP32 GPIO.

## Publication cadence

| Path | Current cadence | Notes |
|---|---:|---|
| Coherent V61/V62 HTTP record | 1 second | newest complete snapshot; no backlog replay |
| V4 application liveness | 1 second | priority, non-repeating Blynk heartbeat value |
| Dashboard round robin | 4 seconds per selected pin | spreads V1–V32 load |
| Diagnostic round robin | 15 seconds per selected pin | spreads V34–V65 load |
| Map V33 | 120 seconds | avoids excessive map points |
| Protocol PING target | 10 seconds | exact response tracked; 15-second proof deadline |
| Blynk heartbeat hard deadline | 189 seconds | final library-level backstop |

V55 is an edge-armed remote hard-reset control. After the 30-second boot lockout, a LOW arms the input and the following HIGH requests `esp_restart()`; the dashboard sends `0 -> 1 -> 0`. The next boot reports `SOFTWARE` on V34.

## Important diagnostics

| Pins/log group | Meaning |
|---|---|
| V4 | application value path is advancing |
| V19 | latest engineering message |
| V20 | system-status bitmask |
| V21/V22 | WiFi RSSI and quality |
| V26 | valid recent 1 PPS lock |
| V27–V32 | INAV state and fused position |
| V34 | reset cause captured at boot |
| V35–V50 | timing, PPS and I2C retained maxima/counters |
| V51 | previous-run RTC forensic checkpoint |
| V52–V54 | Core 1 stage and stall diagnostics |
| V56–V60, V63–V65 | Blynk queue/write diagnostics |

Serial health blocks include `[PERF]`, `[SERVER]`, `[SERVER RX/PONG]`, `[SYNC]`, `[TELEMETRY HTTP]`, `[Blynk PROTOCOL PING]` and `[Blynk APPLICATION ACK]`.

Useful interpretation:

- `protocol PING sent == verified`, zero queue backlog and recent inbound activity indicate a healthy Blynk path.
- HTTP `paused=1` immediately after a PING recovery is expected until the 60-second quiet window completes.
- `ESP reset reason: SOFTWARE` together with `[RESET] ... PROVEN_SHARED_TCP_PATH` is a deliberate recovery.
- `ESP reset reason: BROWNOUT` and `Brownout detector was triggered` indicate real supply undervoltage, not a Blynk or INO recovery. Do not disable the brownout detector; power-cycle and correct the supply/cable/load.

## Recovery ladder

1. Pause application writes when inbound responses become stale.
2. Close/pause the HTTP data plane when the exact Blynk PING is overdue.
3. If only Blynk is affected, rotate the Blynk owner socket and run heartbeat-only probation.
4. If both Blynk and HTTP provide recent transport-failure evidence, perform one complete MCU restart.
5. If WiFi actually loses association/IP, use the separate single-owner WiFi recovery state machine.
6. Task-WDT and cross-core viability supervision remain final protection against a true task stall.

Raw PCB aborts are disabled in the normal Blynk cleanup path. Owner-managed socket close is used instead.

## Build and validation

The v3.9.213 build used Arduino-ESP32 3.3.12 and compiled successfully:

```text
Sketch:           1,056,770 bytes (80%)
Global variables:    75,888 bytes (23%)
Dynamic RAM left:   251,792 bytes
```

The application binary passed Espressif image checksum and validation-hash checks. The reset-domain regression model passed:

```text
1000 modeled sessions / 2000 modeled hours
1000 shared failures -> direct bounded full reset
1000 Blynk-only failures -> socket-first recovery
3552 isolated HTTP failures -> no MCU reset
0 false resets
```

The authoritative simulation is:

```text
simulation/blynk_reset_domain_v319211_2hour_sim.py
```

Run it from the project root:

```powershell
python .\simulation\blynk_reset_domain_v319211_2hour_sim.py
```

The copy inside the release folder is archival; its relative source lookup assumes the original simulation-directory layout.

## Related files

- `AirQualityOutdoor_ESP32_Blynk.ino` — current firmware.
- `aq_dashboard.py` — Windows dashboard v1.9 with strict Blynk + live-data connection health.
- `BLYNK_LIB/` — only the two modified Blynk library files.
- `ESP32_CORE_PATCH/3.3.12/` — targeted ESP32-only WiFi status patch.
- `blynk.service` — current Raspberry Pi 5 Java 21 service unit.
- `dist/firmware-3.9.211-esp32-3.3.12/` — reference source, binaries, build manifest, hashes and simulation.
- `README_System_Health_Diagnostics.md` — historical diagnostic detail; where it conflicts with this README or v3.9.211 source, the source and this current README take precedence.

## Change safety

Always back up the current INO, patched library/core file, README, service unit or server JAR before replacing it. Keep the firmware, patched Blynk files, Java server JAR and service unit as one versioned deployment set; changing only one layer can invalidate recovery assumptions measured across the whole stack.
