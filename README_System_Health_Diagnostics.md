# System Health & Diagnostics

The ESP32 exposes several status values through Blynk. Together they provide a simple way to determine whether the system is operating normally.

## Quick Health Check

| Indicator | Expected | Meaning |
|---|---:|---|
| **V4 — DIAG** | Changing over time | Application/telemetry is running |
| **V19 — Engineering Message** | Latest status text | Most recent human-readable status/fault message |
| **V20 — System Status** | `15` | Wi-Fi, GPS, IMU and environmental sensors OK |
| **V26 — GPS PPS** | `1` | GPS 1PPS signal is locked |
| **V27 — INAV** | `0` | INAV currently has a valid GPS fix |
| **V34 — Reset Cause** | `POWERON` (or expected cause) | Reason the ESP32 last booted/restarted |
| **D2 LED** | Blinks once per second | GPS PPS signal is reaching the ESP32 |

When these indicators are normal, the system is generally operating correctly. If something looks wrong, the performance channel (V35–V50, below) pinpoints *where* time is being lost.

---

## V4 — DIAG

**V4 is an application activity indicator.**

The firmware periodically generates a random value from **1 to 9** and sends it to V4.

Example:

```text
4 → 8 → 2 → 7 → 1 → 9 → 5 → ...
```

The actual number has **no specific meaning**.

What matters is that the value **continues to change**.

### Interpretation

- **V4 changing** → application/telemetry path is running.
- **V4 frozen** → investigate the ESP32, Wi-Fi/Blynk connection, or application execution.

> V4 changing does **not** prove that every sensor is healthy. Use V20 for subsystem health.

---

## V19 — Engineering Message

V19 carries the latest human-readable status/fault message generated during operation (e.g. sensor faults and recoveries, WiFi/Blynk connection events, INAV state changes, GPS status). It is a rolling log value — it always shows the most recent message, not a history.

### Interpretation

- Use V19 to see **what just happened** on the device, in plain text.
- V19 is complementary to V20's bitmask: V20 tells you *what* is currently wrong, V19 often tells you *why* or *when* it changed.

---

## V20 — System Status

V20 is a **bitmask** containing the status of the major subsystems.

| Bit | Value | Meaning |
|---:|---:|---|
| 0 | `1` | Wi-Fi OK |
| 1 | `2` | GPS OK |
| 2 | `4` | IMU OK |
| 3 | `8` | Environmental sensor OK |
| 4 | `16` | CO sensor fault |
| 5 | `32` | GPS fault/timeout |
| 6 | `64` | IMU appears stuck |
| 7 | `128` | Environmental sensor fault |
| 8 | `256` | CO sensor not detected |

### Normal value

```text
V20 = 15
```

because:

```text
1 + 2 + 4 + 8 = 15
```

This means:

```text
Wi-Fi            OK
GPS              OK
IMU              OK
Environmental    OK
```

### Important

V20 is a **bitmask**, not a single state number. Multiple conditions can be active at the same time.

For example:

```text
V20 = 79
```

means:

```text
1 + 2 + 4 + 8 + 64
```

so the IMU-stuck condition is also active.

---

## V26 — GPS PPS Lock

V26 indicates whether the GPS **1PPS timing signal** is currently locked.

| Value | Meaning |
|---:|---|
| `0` | PPS not locked |
| `1` | PPS locked |

A value of:

```text
V26 = 1
```

means the ESP32 is receiving the GPS 1PPS signal correctly.

### D2 LED

The ESP32 board's **D2 LED** provides the same information visually.

When a GPS PPS pulse is received:

- D2 turns **ON**
- remains ON for approximately **250 ms**
- turns OFF
- repeats once per second while PPS is present

Therefore:

```text
D2 blinking once per second
        ↓
GPS PPS signal is reaching the ESP32
```

---

## V27 — INAV Status

V27 indicates the current navigation state.

| Value | State | Meaning |
|---:|---|---|
| `0` | `GPS_FIX` | GPS position is available |
| `1` | `IMU_RECENT` | GPS temporarily missing; recent GPS + IMU used |
| `2` | `IMU_STALE` | GPS missing for several fixes; stale GPS baseline used |
| `3` | `LOST` | GPS unavailable for too long; dead reckoning stopped |
| `4` | `NO_FIX_YET` | No initial GPS fix has ever been obtained |

The normal operating state is:

```text
V27 = 0
```

---

## V34 — Reset Cause

V34 reports the **human-readable reason the ESP32 last booted or restarted**, captured once at startup via `esp_reset_reason()`. It is a boot-time diagnostic, not a live sensor-health value — it stays the same throughout a run and only changes after the next reboot.

| V34 text | ESP-IDF reason | Meaning |
|---|---|---|
| `POWERON` | `ESP_RST_POWERON` | Normal power-on reset |
| `EXTERNAL` | `ESP_RST_EXT` | External reset pin triggered |
| `SOFTWARE` | `ESP_RST_SW` | Reset via `ESP.restart()` / software |
| `PANIC` | `ESP_RST_PANIC` | Crash / exception (e.g. unhandled fault) |
| `INT_WDT` | `ESP_RST_INT_WDT` | Interrupt watchdog triggered |
| `TASK_WDT` | `ESP_RST_TASK_WDT` | Task watchdog triggered (a task stalled) |
| `WDT` | `ESP_RST_WDT` | Other watchdog reset |
| `DEEPSLEEP` | `ESP_RST_DEEPSLEEP` | Woke from deep sleep |
| `BROWNOUT` | `ESP_RST_BROWNOUT` | Brownout detector triggered — investigate power supply first |
| `SDIO` | `ESP_RST_SDIO` | Reset via SDIO |
| `UNKNOWN` / `OTHER` | `ESP_RST_UNKNOWN` / default | Reason not determined |

### Interpretation

- **V34 = `POWERON`** on first boot after a deliberate power cycle is expected and not a fault.
- **V34 = `TASK_WDT` or `INT_WDT`** means a watchdog had to recover a stalled task — check V19 for surrounding context.
- **V34 = `BROWNOUT`** points to a hardware power problem (regulator, wiring, WiFi current spikes, MQ-7 heater transients, or insufficient bulk capacitance) rather than a firmware issue.
- V34 does **not** update on its own — it only changes the next time the device boots.

---

## Performance & I²C Diagnostics (V35–V50)

These channels are internal timing/counter diagnostics. The underlying counters run continuously and never trigger a reset or alter normal control; only their **publication to Blynk is rate-limited to once every 10 seconds**, keeping the diagnostic channel low-overhead. Most are retained maxima **since boot** (they only ever grow, until the next reboot resets them).

| Pin | Value | Meaning |
|---|---|---|
| **V35** | PPS event count | Cumulative count of GPS 1PPS pulses received since boot |
| **V36** | ms since last PPS | Time elapsed since the most recent PPS event — large values with V26=0 confirm PPS loss |
| **V37** | Max `loop()` gap (ms) | Longest observed gap between Arduino `loop()` iterations on Core 1 |
| **V38** | Max sensor-task cycle (ms) | Longest observed full sensor-task cycle time on Core 0 |
| **V39** | Max `Blynk.run()` duration (ms) | Longest observed single `Blynk.run()` call |
| **V40** | Max GPS feed duration (ms) | Longest observed `feedGPS()` processing time |
| **V41** | Max I²C sensor-path duration (ms) | Longest observed I²C sensor access (ENS160/AHT2x/BMI160 combined path) |
| **V42** | Max FAST telemetry duration (ms) | Longest observed FAST Blynk send cycle |
| **V43** | Max SLOW telemetry duration (ms) | Longest observed SLOW Blynk send cycle |
| **V44** | Max PPS→LED response (ms) | Longest observed delay between a PPS event and the D2 LED task responding to it |
| **V45** | Last I²C device | `0`=none/unknown, `1`=ENS160, `2`=AHT21, `3`=BMI160 |
| **V46** | Last I²C operation | `0`=none/unknown, `1`=read, `2`=measurement/command, `3`=configuration |
| **V47** | Cumulative I²C error count | Total I²C transaction errors since boot |
| **V48** | Max AHT21 read duration (ms) | Longest observed AHT21 read |
| **V49** | Max ENS160 operation duration (ms) | Longest observed ENS160 read/operation |
| **V50** | Max BMI160 raw-read duration (ms) | Longest observed BMI160 raw register read |

### Periodic `[PERF]` serial report

In addition to the Blynk channels above, the same set of values is printed to the Serial console every **60 seconds** (alongside the existing `[HEALTH]` heap report), in a single line of the form:

```text
[PERF] PPS=<count> age=<ms>ms loopMax=<ms>ms sensorMax=<ms>ms BlynkRunMax=<ms>ms
       GPSmax=<ms>ms I2Cmax=<ms>ms FASTmax=<ms>ms SLOWmax=<ms>ms PPSledMax=<ms>ms
       I2Cdev=<id> I2Cop=<id> I2Cerrors=<count> AHTmax=<ms>ms ENSmax=<ms>ms BMImax=<ms>ms
```

This is useful for USB-attached debugging sessions where reading the Blynk dashboard isn't convenient.

### Reading a stall from these values

Because most of these are **retained maxima since boot**, a single large value doesn't necessarily mean the device is stalled *right now* — check whether it keeps growing. A worked example, diagnosing a watchdog-triggered reset (V34 = `TASK_WDT`) from a prior run:

| Diagnostic | Value | Interpretation |
|---|---:|---|
| PPS count (V35) | `0` | No PPS events registered since boot |
| Since last PPS (V36) | `0` | No PPS timestamp available |
| Max `loop()` gap (V37) | `2963 ms` | Main loop was blocked for almost 3 seconds |
| Max sensor-task cycle (V38) | `1710 ms` | Sensor task was blocked for 1.71 s |
| Max `Blynk.run()` (V39) | `68 ms` | Not the main problem |
| Max GPS feed (V40) | `0 ms` | GPS did not cause the observed stall |
| Max I²C sensor path (V41) | `997 ms` | Very suspicious |
| Max FAST telemetry (V42) | `1351 ms` | Very slow, but likely a consequence of blocking elsewhere |
| Max SLOW telemetry (V43) | `808 ms` | Also slow |
| PPS→LED response (V44) | `0 ms` | No evidence of PPS/LED-task scheduling being the cause |

The combination that matters here is **I²C max (997 ms) → sensor-task max (1710 ms) → loop max (2963 ms)**: the delay originates on the I²C bus, stalls the sensor task, and that stall then propagates into the main loop. V45–V47 (last I²C device/operation/error count) are the next place to look to identify *which* I²C device caused it.

---

## Recommended Overall Health Check

For a normally operating outdoor station, the preferred combination is:

```text
V4  → changing
V19 → latest message consistent with normal operation
V20 → 15
V26 → 1
V27 → 0
V34 → POWERON (or the expected reset cause for this deployment)
D2  → blinking once per second
```

This indicates:

```text
Application/telemetry    running
Engineering log          current
Wi-Fi                    OK
GPS                      OK
IMU                      OK
Environmental sensor     OK
GPS PPS                  locked
INAV                     has GPS fix
Last reset               as expected
ESP32 PPS indicator      active
```

### In short

**V4 changing** = the application is alive.

**V19** = the latest plain-text status/fault message.

**V20 = 15** = the primary subsystems are healthy.

**V26 = 1** = GPS PPS is locked.

**V27 = 0** = INAV has a GPS fix.

**V34** = why the device last rebooted (historical, not live).

**D2 blinking once per second** = GPS PPS is physically reaching the ESP32.

**V35–V50** = detailed timing/I²C diagnostics used to pinpoint *where* a stall occurred, not needed for routine checks.
