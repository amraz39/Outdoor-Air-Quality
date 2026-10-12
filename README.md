# Outdoor Air Quality Station — ESP32

Current firmware: **v3.9.217**
Current Windows dashboard: **v1.9** (`aq_dashboard.py`)
Target board package: **Arduino-ESP32 3.3.12**
Backend: **self-hosted Blynk 0.41.18 / legacy Blynk protocol 0.6.1**
Primary sketch: `AirQualityOutdoor_ESP32_Blynk.ino`

This project is a dual-core ESP32 outdoor air-quality and navigation station. It acquires GPS, environmental, inertial and acoustic data while maintaining a resilient local connection to a Raspberry Pi 5 Blynk server.

The current design separates sensor acquisition from networking and now also
separates the complete outbound telemetry data plane from the persistent Blynk
control connection.

## Current status

v3.9.217 is the current compiled and simulation-validated build. The supplied
v3.9.216 trace showed a 24.179-second delayed application acknowledgement,
then 10.879-second and 4-second delays, followed by one unrecovered checkpoint.
WiFi remained associated, Blynk writes returned in at most 13 ms, heap stayed
stable, and HTTP had 83/83 successful transactions. This is ordered TCP
head-of-line blocking after an ESP32-to-server segment is delayed or lost.
Recovery can detect that event, but only data-plane separation removes routine
telemetry from the affected persistent control stream.

The important behavior is:

- Core 0 exclusively owns WiFi, lwIP-facing TCP work, `Blynk.run()`, the coherent HTTP client and physical Serial output.
- Core 1 owns sensor initialization, sensor sampling, GPS parsing, IMU/INAV processing and immutable snapshot production. Core 1 never opens or drives a TCP socket.
- The Blynk TCP connection is control-only: login, inbound widget commands and protocol PING/PONG. Normal firmware operation sends zero application `virtualWrite()` frames on it.
- V4, V1-V65 outbound dashboard/diagnostic values, map updates, and V61/V62 use `PUT /{token}/telemetry/batch` on an independent persistent HTTP connection.
- Ordinary virtual pins retain the bounded 64-slot coalescing queue. Stale values are replaced, and one HTTP request can commit several due pins with one server timestamp.
- V4 remains a small non-repeating value generated every second. It is given first priority in each HTTP batch.
- V61 and V62 are still produced from one immutable Core-1 snapshot. They are added to the same HTTP batch every five seconds and retain an identical GPS/device timestamp key.
- Blynk control PING responses provide end-to-end control-stream evidence. Their exact timeout is 30 seconds, above the measured 24.179-second recovered delay.
- A Blynk control-path warning no longer closes a healthy HTTP telemetry connection; the two paths are independent failure domains.
- A Blynk-only sequence hole receives socket-first recovery. Full shared-path recovery requires a fresh unresolved streak of at least two HTTP transport failures within 60 seconds. The HTTP circuit-breaker timer alone is not failure proof.
- WiFi teardown is not used as a generic Blynk repair. WiFi recovery is reserved for actual association/IP failures.
- The optional ten-minute periodic reset remains disabled.

The 7,200-second model runs at 100-ms resolution with the measured 24.179 s,
10.879 s and 4 s delays plus independent HTTP outages. It produced 720 control
PINGs, zero Blynk application frames, zero control disconnects, 7,040 successful
HTTP batches, 1,409 coherent V61/V62 records and zero timestamp mismatches.

## Why v3.9.214 changed the quiet-recovery policy

The v3.9.213 logs exposed two policy defects around a real but transient ESP32
TCP/WiFi delay:

1. an application ACK checkpoint was pending, which correctly stopped later
   application writes and suppressed the redundant protocol PING;
2. the HTTP isolation gate watched only the suppressed protocol PING, so the
   bulk HTTP socket continued sending and retrying (`isolations=0`);
3. the 25-second application deadline expired before the previously measured
   47-58-second TCP retransmission recovery window;
4. an old 90-second HTTP circuit-breaker state could be treated as current
   shared-path proof even when only one recovered failure remained.

v3.9.214 makes recovery event-driven. Either exact Blynk proof being 3 seconds
late immediately removes HTTP from the shared path. Blynk then has a quiet
70-second retransmission window. An open HTTP circuit is retained as a pause
mechanism but cannot by itself authorize an MCU restart.

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
  Blynk control protocol, PING/PONG and inbound callbacks
  Persistent nonblocking HTTP batch client for all outbound virtual pins
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

### Blynk control socket

The persistent Blynk 0.6.1 hardware socket carries only:

- Blynk login, INFO and protocol PING/RESPONSE traffic;
- inbound application callbacks, including V55 remote restart.

Normal operation sends no outbound hardware/application frames on this stream.
This removes routine telemetry packets from the TCP sequence space whose one
missing segment previously blocked all later Blynk commands.

### Outbound telemetry HTTP batch stream

The existing per-pin queue is now an HTTP latest-value queue. V4 is queued once
per second; dashboard, diagnostic and map values retain their existing timer
cadences. Core 1 seals V61/V62 from the same snapshot and Core 0 adds both to
one batch every five seconds.

```http
PUT /<token>/telemetry/batch HTTP/1.1
Connection: keep-alive
Content-Type: application/json

["4","7","21","-54","61","<V61 CSV>","62","<V62 CSV>"]
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

The server validates up to 64 pin/value pairs, stores the complete request under
one timestamp, and pushes updates to active applications without reflecting
them back to the hardware connection. The HTTP client reuses a connection for
at most 60 requests, then rotates it preventively. Its transaction timeout is
5 seconds; repeated failures open a 90-second circuit breaker. HTTP failure
does not pause or reset the Blynk control stream, and Blynk control warnings do
not close a healthy telemetry socket.

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
- flash mode: QIO at 40 MHz (the setting used for the validated v3.9.214 build);
- Arduino loop: Core 0;
- Arduino events: Core 0;
- WiFi/lwIP task affinity: Core 0, as supplied by the selected 3.3.12 configuration;
- partition large enough for the approximately 1.06 MB application.

The v3.9.217 application compiles for the classic ESP32 with QIO/40 MHz flash,
Core-0 Arduino loop and Core-0 Arduino events. Rebuilding with another flash
mode or frequency produces a different binary and should be treated as a
separate test build.

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

A brief yellow/no-live-data indication can therefore be legitimate while the
firmware remains connected. The application ACK barrier deliberately pauses
new V4 writes while an exact checkpoint is outstanding. Field response gaps
have reached 7-12 seconds and then recovered on the same Blynk session. With
the requested strict four-second dashboard window, such a delay can produce a
one- or two-second dashboard indication even though the firmware log contains
no `SESSION END`, `Blynk LOSS` or `Disconnected` event.

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
| Protocol PING target | 10 seconds | exact response tracked; 15-second protocol-proof deadline |
| Application ACK checkpoint | every 4 Blynk application frames | HTTP isolated after 3 seconds late; socket deadline 70 seconds |
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
- HTTP `paused=1` after an application ACK or protocol PING recovery is expected until the 60-second quiet window completes.
- `[Blynk/HTTP ISOLATION] exact Blynk proof overdue` followed by `proof recovered` without `SESSION END` is a successful quiet recovery, not a Blynk reconnect.
- A short dashboard `NO LIVE DATA` indication without a firmware `SESSION END`, `Blynk LOSS` or `Disconnected` record is a strict liveness-window result, not a Blynk session loss.
- `ESP reset reason: SOFTWARE` together with `[RESET] ... PROVEN_SHARED_TCP_PATH` is a deliberate recovery.
- `ESP reset reason: BROWNOUT` and `Brownout detector was triggered` indicate real supply undervoltage, not a Blynk or INO recovery. Do not disable the brownout detector; power-cycle and correct the supply/cable/load.

## Recovery ladder

1. The application ACK barrier pauses later application writes while its exact checkpoint is outstanding.
2. Close/pause the HTTP data plane when either an application ACK checkpoint or protocol PING is 3 seconds overdue.
3. Allow up to 70 seconds of quiet TCP retransmission before declaring an application-path sequence hole.
4. If only Blynk is affected, rotate the Blynk owner socket and run heartbeat-only probation.
5. If Blynk fails and HTTP has a fresh unresolved streak of at least two transport failures, perform one complete MCU restart. An open circuit timer alone is insufficient evidence.
6. If WiFi actually loses association/IP, use the separate single-owner WiFi recovery state machine.
7. Task-WDT and cross-core viability supervision remain final protection against a true task stall.

Raw PCB aborts are disabled in the normal Blynk cleanup path. Owner-managed socket close is used instead.

## Build and validation

The v3.9.214 build used Arduino-ESP32 3.3.12, QIO/40 MHz flash,
Arduino Runs On Core 0 and Events Run On Core 0. It compiled successfully:

```text
Sketch:           1,057,142 bytes (80%)
Global variables:    78,536 bytes (23%)
Dynamic RAM left:   251,792 bytes
```

The new quiet-recovery regression executes a complete two-hour timeline at
100 ms resolution. One hundred deterministic/randomized sessions produced
2,000 modeled hours:

```text
199 late 47-58-second ACK recoveries
409 HTTP quiet-path isolations
0 Blynk socket rotations
0 MCU resets
0 V61/V62 timestamp-pair mismatches
0 Core-1 network calls
0 watchdog resets
0 false shared-path resets
```

The authoritative simulation is:

```text
simulation/blynk_quiet_recovery_v319214_2hour_sim.py
```

Run it from the project root:

```powershell
python .\simulation\blynk_quiet_recovery_v319214_2hour_sim.py
```

The earlier v3.9.210 post-run and v3.9.211 reset-domain models were also rerun
as compatibility checks. V61/V62 atomic pairing, Core ownership, queue bounds,
V4 priority, watchdog safety and narrow/shared recovery selection remain
covered.

## Related files

- `AirQualityOutdoor_ESP32_Blynk.ino` — current firmware.
- `aq_dashboard.py` — Windows dashboard v1.9 with strict Blynk + live-data connection health.
- `BLYNK_LIB/` — only the two modified Blynk library files.
- `ESP32_CORE_PATCH/3.3.12/` — targeted ESP32-only WiFi status patch.
- `blynk.service` — current Raspberry Pi 5 Java 21 service unit.
- `simulation/blynk_quiet_recovery_v319214_2hour_sim.py` — current two-hour/100-ms recovery and regression model.
- `AirQualityOutdoor_ESP32_Blynk.ino.20261011_pre_v3.9.214.bak` — rollback copy immediately before the quiet-recovery update.
- `README_System_Health_Diagnostics.md` — historical diagnostic detail; where it conflicts with this README or v3.9.214 source, the source and this current README take precedence.

## Change safety

Always back up the current INO, patched library/core file, README, service unit or server JAR before replacing it. Keep the firmware, patched Blynk files, Java server JAR and service unit as one versioned deployment set; changing only one layer can invalidate recovery assumptions measured across the whole stack.
