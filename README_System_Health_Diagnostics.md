# System Health & Diagnostics

The ESP32 exposes several status values through Blynk. Together they provide a simple way to determine whether the system is operating normally.

## Quick Health Check

| Indicator | Expected | Meaning |
|---|---:|---|
| **V4 — DIAG** | Changing over time | Application/telemetry is running |
| **V20 — System Status** | `15` | Wi-Fi, GPS, IMU and environmental sensors OK |
| **V26 — GPS PPS** | `1` | GPS 1PPS signal is locked |
| **V27 — INAV** | `0` | INAV currently has a valid GPS fix |
| **D2 LED** | Blinks once per second | GPS PPS signal is reaching the ESP32 |

When these indicators are normal, the system is generally operating correctly.

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

## Recommended Overall Health Check

For a normally operating outdoor station, the preferred combination is:

```text
V4  → changing
V20 → 15
V26 → 1
V27 → 0
D2  → blinking once per second
```

This indicates:

```text
Application/telemetry    running
Wi-Fi                    OK
GPS                      OK
IMU                      OK
Environmental sensor     OK
GPS PPS                  locked
INAV                     has GPS fix
ESP32 PPS indicator      active
```

### In short

**V4 changing** = the application is alive.

**V20 = 15** = the primary subsystems are healthy.

**V26 = 1** = GPS PPS is locked.

**V27 = 0** = INAV has a GPS fix.

**D2 blinking once per second** = GPS PPS is physically reaching the ESP32.
