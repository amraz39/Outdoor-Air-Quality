# Outdoor Air Quality Station — ESP32 Dashboard for Windows PC
# v2.0 — Rolling 24-hour diagnostic event journal
#
# v2.0 changes:
#   - Creates diag/aq_station_events.csv beside the script/executable.
#   - Records transition episodes for server, Blynk-session, V4 telemetry,
#     coherent-record, ESP32 reset, status-flag, RSSI and reset-command events.
#   - Uses UTC timestamps, confidence labels and START/END episode IDs so a
#     telemetry pause is not mislabeled as a Blynk reconnect.
#   - Retains only the newest 24 hours through periodic atomic CSV pruning.
#
# v1.9 — Four-second V4 loss indication and clean connected text
#
# v1.9 changes:
#   - A reachable but non-advancing V4 is authoritative evidence of a stalled
#     stream; the slower V61/V62 path can no longer mask it for 12 seconds.
#   - V61/V62 is used only when the V4 HTTP read itself is unavailable.
#   - CONNECTED no longer displays the internal ``check: ...`` evidence text.
#
# v1.8 — Isolated one-second V4 diagnostic display
#
# v1.8 changes:
#   - The one-second health worker now sends V4 through its own GUI queue item.
#   - Only DIAG (App/Telem. RND# Alive) is redrawn from that one-second value.
#   - V4 is removed from the five-second full-data sweep, preventing duplicate
#     reads and preventing the general UI refresh from overwriting the widget.
#
# v1.7 — One-second V4 heartbeat with V61/V62 fallback
#
# v1.7 changes:
#   - A dedicated health worker polls deterministic V4 once per second.
#   - Three consecutive V4 arrivals are buffered; the first is a baseline and
#     the next two confirm that the stream continues to advance.
#   - An advancing V4 is the primary live-stream proof; a newly matched
#     V61/V62 record key is the fallback if V4 is delayed or unavailable.
#   - CONNECTED still requires isHardwareConnected=TRUE as an independent
#     condition. Cached values or either liveness signal alone are insufficient.
#
# v1.6 — Strict Blynk-link AND live-telemetry connection health
#
# v1.6 changes:
#   - CONNECTED now requires BOTH isHardwareConnected=TRUE from the Blynk
#     server AND a changing ESP32 datastream within its freshness window.
#   - Cached virtual-pin values and a server TRUE response alone can no longer
#     make a stalled device appear connected.
#   - The UI distinguishes an unavailable/offline Blynk link from a connected
#     Blynk session that is not delivering fresh ESP32 telemetry.
#   - Live graph samples are recorded only when the ESP32 stream advances.
#
# v1.5 — Safe edge-armed V55 reset sequence for ESP32 firmware v3.9.212
#
# v1.5 changes:
#   - Remote reset now sends 0 -> 1 -> 0. The leading LOW arms the firmware,
#     preventing a retained/replayed V55 HIGH from causing a reset loop.
#
# v1.4 — Live-graph and connection-health refactor
#
# v1.4 changes:
#   - Graphs merge server history with bounded live samples from this session.
#   - Connection state honors isHardwareConnected beyond startup and falls
#     back to actual telemetry changes when the endpoint is unavailable.
#   - Data, status, graph, and control workers use separate HTTP sessions.
#   - Tk variables are no longer read from background threads, and a graph
#     exception can no longer permanently suppress future refreshes.
#
# Connects to your local Blynk server and displays every virtual pin
# published by the ESP32 firmware:
#
#   - GPS / IMU / INAV navigation block
#   - Live environmental + air-quality sensor tiles
#   - Messages section (V61, V62) shown above Sensor History
#   - 2x2 history graphs for TVOC / eCO2 / CO / Dust with a shared
#     selectable time range (1h / 3h / 24h / 48h / 168h)
#   - Full engineering/diagnostic pin readout (V19, V20, V34, V35-V51),
#     with V19 (Engineering Message), V34 (Reset Reason), and V51 (RTC
#     Checkpoint) each shown as their own titled label + single value line
#   - Local-only CLEAR button + automatic on-startup clear: resets all
#     displayed numeric values to 0.0 and text values to "NaN" in this
#     application only (no Blynk/ESP32 traffic; next poll repopulates
#     real data as usual)
#   - Remote hard-reset button (V55: 0 -> 1 -> 0)
#   - Location panel (GPS + fused INAV position) with a simple trail plot
#     and an "Open in Google Maps" shortcut
#
# ============================================================
# REQUIRED FILES
# ============================================================
#
# Place next to this script a secrets.h containing at least:
#
#   #define WIFI_SSID           "yourssid"
#   #define WIFI_PASS           "yourpass"
#   #define BLYNK_AUTH          "yourtoken"
#   #define BLYNK_SERVER        "192.168.x.x"
#   #define BLYNK_PORT          8084
#
# secrets.h contains credentials, so add it to .gitignore before pushing
# this project to GitHub — only aq_dashboard.py and requirements.txt
# are meant to be public.
#
# ============================================================
# REQUIRED PYTHON PACKAGES
# ============================================================
#
#   pip install requests customtkinter matplotlib numpy
#
# ============================================================

import os
import re
import csv
import gzip
import json
import queue
import sys
import threading
import time
import uuid
import webbrowser
from collections import deque
from datetime import datetime, timedelta, timezone
from pathlib import Path

import requests
import numpy as np
import customtkinter as ctk
from tkinter import messagebox

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.dates as mdates
import tkintermapview

# ============================================================
# SECRETS.H PARSING
# ============================================================


def _read_secrets_value(key, quoted=True):
    if not os.path.exists("secrets.h"):
        raise FileNotFoundError("secrets.h not found")
    with open("secrets.h", "r", encoding="utf-8") as f:
        content = f.read()
    if quoted:
        pattern = rf'{key}\s+"([^"]+)"'
    else:
        pattern = rf'{key}\s+"?([0-9]+)"?'
    match = re.search(pattern, content)
    if not match:
        raise ValueError(f"{key} not found in secrets.h")
    return match.group(1)


WIFI_SSID = _read_secrets_value("WIFI_SSID", quoted=True)
BLYNK_AUTH = _read_secrets_value("BLYNK_AUTH", quoted=True)
BLYNK_SERVER = _read_secrets_value("BLYNK_SERVER", quoted=True)
BLYNK_PORT = _read_secrets_value("BLYNK_PORT", quoted=False)

# ============================================================
# CONFIGURATION
# ============================================================

FAST_POLL_MS = 5000          # matches FAST telemetry tier on the ESP32
HEALTH_POLL_MS = 1000        # lightweight V4 + Blynk status check
GRAPH_REFRESH_MS = 60000     # how often server-side history is re-fetched
LIVE_GRAPH_DRAW_MS = 5000    # redraw cached history + current session samples
LIVE_GRAPH_SAMPLE_S = 30.0   # bound graph memory while preserving trends
HTTP_TIMEOUT_S = 4
HEALTH_HTTP_TIMEOUT_S = 1.0

# Rolling diagnostic journal. Event rows are retained for exactly 24 hours;
# continuous telemetry samples are intentionally not written here.
EVENT_RETENTION_HOURS = 24
EVENT_PRUNE_INTERVAL_S = 300.0
EVENT_STARTUP_GRACE_S = 5.0
EVENT_RSSI_WEAK_DBM = -72
EVENT_RSSI_RECOVER_DBM = -68
EVENT_PAIR_MISMATCH_POLLS = 2


def _application_directory():
    """Return a stable writable directory for script and packaged-EXE runs."""
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parent


DIAG_DIRECTORY = _application_directory() / "diag"
EVENT_CSV_PATH = DIAG_DIRECTORY / "aq_station_events.csv"

# V4 is a rolling diagnostic/telemetry counter sent by the ESP32.
# Firmware v3.9.214 publishes a deterministic V4 heartbeat once per second.
# The dashboard checks it once per second and allows normal LAN/poll jitter.
# The full data worker polls every five seconds, so matching V61/V62 keys use a
# wider fallback window. A successful GET or an initial cached value is never
# sufficient: the observed value/key must advance after its baseline.
LIVE_TELEM_PIN = 4
HEARTBEAT_TIMEOUT_S = 3.5
HEARTBEAT_CONFIRM_WINDOW_S = 4.0
PAIR_FALLBACK_TIMEOUT_S = 12.0

UNAVAILABLE_THRESHOLD = -998.0   # firmware uses -999.0f as SENSOR_UNAVAILABLE

# ============================================================
# VIRTUAL PIN MAP  (must match AirQualityOutdoor_ESP32_Blynk.ino)
# ============================================================

GPS_PINS = {"lat": 12, "lng": 13, "sats": 14, "hdop": 17, "pps": 26}
IMU_PINS = {"roll": 8, "pitch": 9, "yaw": 10, "temp": 11}
INAV_PINS = {"state": 27, "lat": 28, "lng": 29, "alt": 30, "speed": 31, "heading": 32}

SENSOR_PINS = {
    "co_ppm": 2, "co_phase": 6, "co_raw": 7, "co_heater_v": 25,
    "humidity": 15, "temperature": 16,
    "sound_db": 1, "uvi": 3, "tvoc": 5, "dust": 18, "eco2": 23, "aqi": 24,
    # RSSI/quality are useful only while the ESP32 connection is live.
    # Blynk may return their last cached values after disconnect.
    "rssi": 21, "wifi_qual": 22,
}

MISC_PINS = {"eng_msg": 19, "status_flags": 20, "msg_v61": 61, "msg_v62": 62}

# V4 is used by the app/telemetry layer as a rolling diagnostic counter.
# It is displayed in the top-right diagnostic card and is always replaced
# with the newest value rather than accumulated as history.
APP_TELEM_RND_PIN = LIVE_TELEM_PIN

DIAG_PINS = {
    "reset_reason": 34,
    "pps_count": 35, "pps_age_ms": 36, "loop_gap_max": 37, "sensor_cycle_max": 38,
    "blynk_run_max": 39, "gps_feed_max": 40, "i2c_path_max": 41,
    "fast_send_max": 42, "slow_send_max": 43, "pps_led_max": 44,
    "i2c_device": 45, "i2c_op": 46, "i2c_errors": 47,
    "aht21_max": 48, "ens160_max": 49, "bmi160_max": 50,
    "rtc_checkpoint": 51,
}

GRAPH_PINS = {
    "TVOC (ppb)": SENSOR_PINS["tvoc"],
    "eCO2 (ppm)": SENSOR_PINS["eco2"],
    "CO (ppm)": SENSOR_PINS["co_ppm"],
    "Dust (mg/m³)": SENSOR_PINS["dust"],
}

# Reverse lookup used when live polled values are added to the graphs.
GRAPH_VALUE_KEYS = {
    pin: key for key, pin in SENSOR_PINS.items() if pin in GRAPH_PINS.values()
}

RESET_PIN = 55

ALL_READ_PINS = {}
ALL_READ_PINS.update(GPS_PINS)
for k, v in IMU_PINS.items():
    ALL_READ_PINS[f"imu_{k}"] = v
for k, v in INAV_PINS.items():
    ALL_READ_PINS[f"inav_{k}"] = v
ALL_READ_PINS.update(SENSOR_PINS)
ALL_READ_PINS.update(MISC_PINS)
ALL_READ_PINS.update(DIAG_PINS)

INAV_STATE_NAMES = {
    0: "GPS FIX", 1: "IMU RECENT", 2: "IMU STALE", 3: "LOST", 4: "NO FIX YET",
}
INAV_STATE_COLORS = {
    0: "#22c55e", 1: "#22d3ee", 2: "#f59e0b", 3: "#ef4444", 4: "#64748b",
}
I2C_DEVICE_NAMES = {0: "NONE", 1: "ENS160", 2: "AHT21", 3: "BMI160"}
I2C_OP_NAMES = {0: "NONE", 1: "READ", 2: "MEAS/CMD", 3: "CONFIG"}

STATUS_FLAGS = [
    (1 << 0, "WiFi OK"),
    (1 << 1, "GPS OK"),
    (1 << 2, "IMU OK"),
    (1 << 3, "ENS160 OK"),
    (1 << 4, "CO FAULT"),
    (1 << 5, "GPS FAULT"),
    (1 << 6, "IMU STUCK"),
    (1 << 7, "ENS160 FAULT"),
    (1 << 8, "CO NO SENSOR"),
]
# Flags where the bit being SET is a good/OK condition (green when set).
STATUS_GOOD_WHEN_SET = {1 << 0, 1 << 1, 1 << 2, 1 << 3}

# ============================================================
# ROLLING 24-HOUR DIAGNOSTIC EVENT JOURNAL
# ============================================================

EVENT_CSV_FIELDS = (
    "timestamp_utc", "episode_id", "event_phase", "event_type", "severity",
    "duration_seconds", "blynk_status", "v4_available", "v4_fresh",
    "v4_value", "pair_gps_unix", "pair_device_ms", "reset_reason",
    "status_flags", "rssi_dbm", "confidence", "details",
)


def _utc_now_text():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace(
        "+00:00", "Z")


def _parse_utc_text(value):
    try:
        return datetime.fromisoformat(str(value).replace("Z", "+00:00"))
    except (TypeError, ValueError):
        return None


class EventJournal:
    """Serialize diagnostic events and atomically enforce 24-hour retention."""

    def __init__(self, path):
        self.path = Path(path)
        self._queue = queue.Queue()
        self._accepting = True
        self._close_lock = threading.Lock()
        self._thread = threading.Thread(
            target=self._run, name="aq-event-journal", daemon=True)
        self._thread.start()

    def log(self, event_type, event_phase="OBSERVED", severity="INFO",
            episode_id="", duration_seconds="", confidence="confirmed",
            **context):
        if not self._accepting:
            return
        row = {name: "" for name in EVENT_CSV_FIELDS}
        row.update({
            "timestamp_utc": _utc_now_text(),
            "episode_id": episode_id or uuid.uuid4().hex,
            "event_phase": event_phase,
            "event_type": event_type,
            "severity": severity,
            "duration_seconds": duration_seconds,
            "confidence": confidence,
        })
        for key in EVENT_CSV_FIELDS:
            if key in context and context[key] is not None:
                value = context[key]
                if isinstance(value, bool):
                    value = "true" if value else "false"
                row[key] = str(value).replace("\r", " ").replace("\n", " ")
        self._queue.put(row)

    def close(self):
        """Flush a final shutdown event without delaying normal GUI work."""
        with self._close_lock:
            if not self._accepting:
                return
            self.log("DASHBOARD_STOPPED", details="Normal GUI shutdown")
            self._accepting = False
            self._queue.put(None)
        self._thread.join(timeout=2.0)

    def _prepare_file(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.exists() and self.path.stat().st_size:
            try:
                with self.path.open("r", newline="", encoding="utf-8") as handle:
                    header = next(csv.reader(handle), [])
                if tuple(header) != EVENT_CSV_FIELDS:
                    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
                    self.path.replace(
                        self.path.with_name(f"{self.path.stem}.incompatible-{stamp}.csv"))
            except OSError:
                pass
        if not self.path.exists() or self.path.stat().st_size == 0:
            with self.path.open("w", newline="", encoding="utf-8") as handle:
                csv.DictWriter(handle, fieldnames=EVENT_CSV_FIELDS).writeheader()
        self._prune()

    def _append(self, row):
        with self.path.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=EVENT_CSV_FIELDS)
            writer.writerow(row)
            handle.flush()

    def _prune(self):
        if not self.path.exists():
            return
        cutoff = datetime.now(timezone.utc) - timedelta(hours=EVENT_RETENTION_HOURS)
        retained = []
        try:
            with self.path.open("r", newline="", encoding="utf-8") as handle:
                for row in csv.DictReader(handle):
                    stamp = _parse_utc_text(row.get("timestamp_utc"))
                    if stamp is not None and stamp >= cutoff:
                        retained.append({key: row.get(key, "") for key in EVENT_CSV_FIELDS})
            temporary = self.path.with_suffix(self.path.suffix + ".tmp")
            with temporary.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=EVENT_CSV_FIELDS)
                writer.writeheader()
                writer.writerows(retained)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, self.path)
        except OSError as exc:
            print(f"Diagnostic event journal prune failed: {exc}", file=sys.stderr)

    def _run(self):
        try:
            self._prepare_file()
            next_prune = time.monotonic() + EVENT_PRUNE_INTERVAL_S
            while True:
                timeout = max(0.1, next_prune - time.monotonic())
                try:
                    row = self._queue.get(timeout=timeout)
                except queue.Empty:
                    row = "PRUNE"
                if row is None:
                    break
                if row == "PRUNE":
                    self._prune()
                    next_prune = time.monotonic() + EVENT_PRUNE_INTERVAL_S
                    continue
                self._append(row)
                if time.monotonic() >= next_prune:
                    self._prune()
                    next_prune = time.monotonic() + EVENT_PRUNE_INTERVAL_S
            self._prune()
        except Exception as exc:
            # Diagnostics must never take down the monitoring dashboard.
            print(f"Diagnostic event journal disabled: {exc}", file=sys.stderr)


class DiagnosticEventMonitor:
    """Convert polled values into clearly classified transition episodes."""

    def __init__(self, journal):
        self.journal = journal
        self.lock = threading.Lock()
        self.started_at = time.monotonic()
        self.health_enabled = False
        self.active = {}
        self.context = {
            "blynk_status": "unknown", "v4_available": "", "v4_fresh": "",
            "v4_value": "", "pair_gps_unix": "", "pair_device_ms": "",
            "reset_reason": "", "status_flags": "", "rssi_dbm": "",
        }
        self.last_device_ms = None
        self.last_reset_reason = None
        self.last_status_flags = None
        self.last_pair_key = None
        self.last_pair_change = None
        self.pair_mismatch_polls = 0
        self.last_engineering_event = None

    def _snapshot(self, details=""):
        result = dict(self.context)
        result["details"] = details
        return result

    def _episode(self, event_type, condition, severity, confidence, details):
        current = self.active.get(event_type)
        if condition and current is None:
            episode_id = uuid.uuid4().hex
            started = time.monotonic()
            self.active[event_type] = (episode_id, started)
            self.journal.log(event_type, "START", severity, episode_id=episode_id,
                             confidence=confidence, **self._snapshot(details))
        elif not condition and current is not None:
            episode_id, started = self.active.pop(event_type)
            duration = f"{max(0.0, time.monotonic() - started):.3f}"
            self.journal.log(event_type, "END", "INFO", episode_id=episode_id,
                             duration_seconds=duration, confidence=confidence,
                             **self._snapshot(f"Recovered: {details}"))

    def close_all(self):
        """Close open episodes so the CSV remains easy to analyse."""
        with self.lock:
            for event_type, (episode_id, started) in list(self.active.items()):
                duration = f"{max(0.0, time.monotonic() - started):.3f}"
                self.journal.log(
                    event_type, "END", "INFO", episode_id=episode_id,
                    duration_seconds=duration, confidence="observed",
                    **self._snapshot("Dashboard stopped while episode was active"))
            self.active.clear()

    def observe_health(self, blynk_state, heartbeat_available,
                       heartbeat_fresh, heartbeat_value, pair_fresh, source):
        with self.lock:
            self.context.update({
                "blynk_status": ("unknown" if blynk_state is None
                                  else "connected" if blynk_state else "offline"),
                "v4_available": heartbeat_available,
                "v4_fresh": heartbeat_fresh,
                "v4_value": heartbeat_value if heartbeat_value is not None else "",
            })
            if not self.health_enabled:
                if time.monotonic() - self.started_at < EVENT_STARTUP_GRACE_S:
                    return
                self.health_enabled = True

            data_fresh = heartbeat_fresh or (
                not heartbeat_available and pair_fresh)
            self._episode(
                "BLYNK_STATUS_UNAVAILABLE", blynk_state is None, "ERROR",
                "confirmed", f"Hardware-status endpoint unavailable; {source}")
            self._episode(
                "BLYNK_SESSION_OFFLINE", blynk_state is False, "ERROR",
                "confirmed", "Blynk isHardwareConnected reported false")
            self._episode(
                "V4_READ_UNAVAILABLE", not heartbeat_available, "WARNING",
                "confirmed", "V4 HTTP read unavailable")
            self._episode(
                "TELEMETRY_STALL", blynk_state is True and not data_fresh,
                "WARNING", "confirmed",
                "Blynk session present but V4/V61-V62 liveness is stale")

    def observe_data(self, values, key61, key62, matched_pair_key):
        with self.lock:
            reset_reason = values.get("reset_reason")
            status_flags = values.get("status_flags")
            rssi = values.get("rssi")
            self.context.update({
                "reset_reason": reset_reason or "",
                "status_flags": status_flags or "",
                "rssi_dbm": rssi or "",
            })

            if reset_reason is not None:
                reset_text = str(reset_reason).strip()
                if (self.last_reset_reason is not None and reset_text and
                        reset_text != self.last_reset_reason):
                    self.journal.log(
                        "RESET_REASON_CHANGED", severity="WARNING",
                        confidence="confirmed",
                        **self._snapshot(
                            f"V34 changed {self.last_reset_reason!r} -> {reset_text!r}"))
                if reset_text:
                    self.last_reset_reason = reset_text

            if status_flags is not None:
                try:
                    flags = int(float(status_flags))
                except (TypeError, ValueError):
                    flags = None
                if flags is not None:
                    if self.last_status_flags is not None and flags != self.last_status_flags:
                        changed = self.last_status_flags ^ flags
                        self.journal.log(
                            "STATUS_FLAGS_CHANGED", severity="WARNING",
                            confidence="confirmed",
                            **self._snapshot(
                                f"V20 {self.last_status_flags} -> {flags}; changedMask=0x{changed:X}"))
                    self.last_status_flags = flags

            try:
                rssi_number = float(rssi) if rssi is not None else None
            except (TypeError, ValueError):
                rssi_number = None
            weak_active = "WEAK_WIFI_SIGNAL" in self.active
            weak_now = (rssi_number is not None and
                        (rssi_number <= EVENT_RSSI_WEAK_DBM or
                         (weak_active and rssi_number < EVENT_RSSI_RECOVER_DBM)))
            self._episode("WEAK_WIFI_SIGNAL", weak_now, "WARNING", "observed",
                          f"RSSI={rssi_number} dBm")

            pair_mismatch = key61 is not None and key62 is not None and key61 != key62
            self.pair_mismatch_polls = (
                self.pair_mismatch_polls + 1 if pair_mismatch else 0)
            self._episode(
                "V61_V62_MISMATCH",
                self.pair_mismatch_polls >= EVENT_PAIR_MISMATCH_POLLS,
                "WARNING", "observed",
                f"V61 key={key61}; V62 key={key62}")

            if matched_pair_key is not None:
                gps_unix, device_ms_text = matched_pair_key
                self.context["pair_gps_unix"] = gps_unix
                self.context["pair_device_ms"] = device_ms_text
                try:
                    device_ms = int(device_ms_text)
                except (TypeError, ValueError):
                    device_ms = None
                if device_ms is not None:
                    if (self.last_device_ms is not None and
                            device_ms + 5000 < self.last_device_ms):
                        rollover = (self.last_device_ms >= 0xF0000000 and
                                    device_ms <= 0x0FFFFFFF)
                        self.journal.log(
                            "DEVICE_MILLIS_ROLLOVER" if rollover else "ESP32_RESET",
                            severity="INFO" if rollover else "ERROR",
                            confidence="confirmed",
                            **self._snapshot(
                                f"deviceMs {self.last_device_ms} -> {device_ms}"))
                    self.last_device_ms = device_ms

                if matched_pair_key != self.last_pair_key:
                    self.last_pair_key = matched_pair_key
                    self.last_pair_change = time.monotonic()

            pair_stalled = (
                self.last_pair_change is not None and
                time.monotonic() - self.last_pair_change > PAIR_FALLBACK_TIMEOUT_S)
            self._episode(
                "COHERENT_STREAM_STALL", pair_stalled, "WARNING", "observed",
                "Matched V61/V62 record key did not advance")

            eng_msg = str(values.get("eng_msg") or "").strip()
            if eng_msg and eng_msg != self.last_engineering_event and re.search(
                    r"disconnect|timeout|reset|failed|recovery|connected", eng_msg,
                    flags=re.IGNORECASE):
                self.last_engineering_event = eng_msg
                self.journal.log(
                    "ENGINEERING_MESSAGE", severity="INFO", confidence="observed",
                    **self._snapshot(eng_msg))


_event_journal = EventJournal(EVENT_CSV_PATH)
_event_monitor = DiagnosticEventMonitor(_event_journal)
_event_journal.log(
    "DASHBOARD_STARTED", details=f"Dashboard v2.0; journal={EVENT_CSV_PATH.name}")

# ============================================================
# COLOR THEME
# ============================================================

BG = "#090d16"   # Very dark (mostly black) blue
CARD = "#0f1623"   # Very dark (mostly black) blue
BORDER = "#2a3a52"   #  Very dark desaturated blue
TXT_MAIN = "#f1f5f9"   # Light gray
TXT_SUB = "#94a3b8"   # Medium gray
TXT_DIM = "#475569"   # Dark gray
ACCENT = "#22d3ee"   # Cyan
GREEN = "#22c55e"   # Green
AMBER = "#f59e0b"   # Amber
RED = "#ef4444"   # Red

FONT_TITLE = ("Segoe UI", 22, "bold")
FONT_SECTION = ("Segoe UI", 15, "bold")
FONT_LABEL = ("Segoe UI", 11)
FONT_VALUE = ("Segoe UI", 18, "bold")
FONT_SMALL = ("Segoe UI", 10)

# ============================================================
# BLYNK HTTP API
# ============================================================
# IMPORTANT: Blynk virtual-pin GETs return the server's last known value.
# They are therefore NOT a valid hardware-online test. CONNECTED requires the
# logical AND of isHardwareConnected=TRUE and a recently changing ESP32
# datastream. The legacy endpoint is tried first; the external API form is also
# supported for Blynk installations that expose it.


_http_local = threading.local()


def _session():
    """Return one persistent HTTP session per worker thread.

    ``requests.Session`` is not guaranteed to be thread-safe. The dashboard
    has independent data, status, graph, and reset workers, so each gets its
    own connection pool.
    """
    session = getattr(_http_local, "session", None)
    if session is None:
        session = requests.Session()
        _http_local.session = session
    return session


def read_pin(pin, timeout=HTTP_TIMEOUT_S):
    url = f"http://{BLYNK_SERVER}:{BLYNK_PORT}/{BLYNK_AUTH}/get/V{pin}"
    try:
        r = _session().get(url, timeout=timeout)
        if r.status_code != 200:
            return None
        text = r.text.strip()
        try:
            decoded = json.loads(text)
            if isinstance(decoded, list):
                return str(decoded[0]) if decoded else None
            return str(decoded)
        except (ValueError, TypeError):
            # Compatibility with older/non-standard local-server responses.
            return text.replace('["', "").replace('"]', "")
    except Exception:
        return None


def write_pin(pin, value):
    url = f"http://{BLYNK_SERVER}:{BLYNK_PORT}/{BLYNK_AUTH}/update/V{pin}?value={value}"
    try:
        r = _session().get(url, timeout=HTTP_TIMEOUT_S)
        return r.status_code == 200
    except Exception:
        return False


def _timestamp_from_server(raw_timestamp):
    """Accept Unix seconds, milliseconds, microseconds, or nanoseconds."""
    value = float(raw_timestamp)
    magnitude = abs(value)
    if magnitude >= 1e17:
        value /= 1e9
    elif magnitude >= 1e14:
        value /= 1e6
    elif magnitude >= 1e11:
        value /= 1e3
    return datetime.fromtimestamp(value)


def fetch_history(pin, hours):
    """Return ``(points, error)`` for the requested server-side history."""
    url = f"http://{BLYNK_SERVER}:{BLYNK_PORT}/{BLYNK_AUTH}/data/V{pin}"
    try:
        r = _session().get(url, timeout=15)
        if r.status_code != 200:
            return [], f"HTTP {r.status_code}"
        try:
            text = gzip.decompress(r.content).decode("utf-8", errors="ignore")
        except Exception:
            text = r.text
    except Exception as exc:
        return [], type(exc).__name__

    cutoff = datetime.now() - timedelta(hours=hours)
    points = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split(",")
        if len(parts) < 2:
            continue
        try:
            value = float(parts[0])
            ts = _timestamp_from_server(parts[1])
            if ts >= cutoff:
                points.append((ts, value))
        except Exception:
            continue
    points.sort(key=lambda p: p[0])
    return points, None


# ============================================================
# HELPERS
# ============================================================


def fmt(value, unit="", decimals=1):
    if value is None:
        return "N/A"
    try:
        f = float(value)
    except Exception:
        return "N/A"
    if f <= UNAVAILABLE_THRESHOLD:
        return "N/A"
    return f"{f:.{decimals}f}{unit}"


def fmt_int(value, unit=""):
    if value is None:
        return "N/A"
    try:
        return f"{int(float(value))}{unit}"
    except Exception:
        return "N/A"


def rssi_to_quality(rssi_dbm):
    return max(0, min(100, 2 * (rssi_dbm + 100)))


def wifi_color_for(quality):
    if quality >= 75:
        return GREEN
    if quality >= 50:
        return ACCENT
    if quality >= 25:
        return AMBER
    return RED


# ============================================================
# APP WINDOW
# ============================================================

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

app = ctk.CTk()
app.title("Outdoor Air Quality Station — ESP32 Dashboard")
app.geometry("1500x980")
app.configure(fg_color=BG)

# ---- Header ----------------------------------------------------------

header = ctk.CTkFrame(app, fg_color="transparent")
header.pack(fill="x", padx=20, pady=(16, 6))

ctk.CTkLabel(header, text="🌤️  Outdoor Air Quality Station", font=FONT_TITLE,
             text_color=TXT_MAIN).pack(side="left")
ctk.CTkLabel(header, text=f"({WIFI_SSID})", font=FONT_LABEL,
             text_color=TXT_DIM).pack(side="left", padx=(10, 0))

status_wrap = ctk.CTkFrame(header, fg_color="transparent")
status_wrap.pack(side="right")

label_status = ctk.CTkLabel(status_wrap, text="⬤  OFFLINE", font=FONT_SECTION,
                             text_color=RED)
label_status.pack(side="left", padx=(0, 16))

label_wifi = ctk.CTkLabel(status_wrap, text="📶  WiFi: —", font=FONT_LABEL,
                           text_color=TXT_DIM)
label_wifi.pack(side="left", padx=(0, 16))

label_last_update = ctk.CTkLabel(status_wrap, text="Last live data: —",
                                  font=FONT_LABEL, text_color=TXT_DIM)
label_last_update.pack(side="left")

# ---- Scrollable body ---------------------------------------------------

body = ctk.CTkScrollableFrame(app, fg_color="transparent")
body.pack(fill="both", expand=True, padx=12, pady=(0, 12))


def section_title(parent, icon, text):
    ctk.CTkLabel(parent, text=f"{icon}  {text}", font=FONT_SECTION,
                 text_color=TXT_MAIN, anchor="w").pack(fill="x", padx=8, pady=(14, 6))


def make_card(parent, side=None, **pack_kwargs):
    card = ctk.CTkFrame(parent, fg_color=CARD, corner_radius=14,
                         border_width=1, border_color=BORDER)
    defaults = dict(padx=6, pady=6)
    defaults.update(pack_kwargs)
    if side:
        card.pack(side=side, **defaults)
    else:
        card.pack(**defaults)
    return card


def add_tile(parent, icon, title, initial="N/A"):
    """A small titled value tile. Returns the CTkLabel showing the value."""
    tile = ctk.CTkFrame(parent, fg_color="#131b2b", corner_radius=10,
                         border_width=1, border_color=BORDER)
    tile.pack(side="left", fill="both", expand=True, padx=5, pady=5)
    ctk.CTkLabel(tile, text=f"{icon}  {title}", font=FONT_LABEL,
                 text_color=TXT_SUB, anchor="w").pack(fill="x", padx=10, pady=(8, 0))
    value_label = ctk.CTkLabel(tile, text=initial, font=FONT_VALUE,
                                text_color=TXT_MAIN, anchor="w")
    value_label.pack(fill="x", padx=10, pady=(0, 8))
    return value_label


# ============================================================
# SECTION 1 — GPS / IMU / INAV
# ============================================================

section_title(body, "🛰️", "Navigation — GPS · IMU · INAV")
nav_row = ctk.CTkFrame(body, fg_color="transparent")
nav_row.pack(fill="x", padx=4)

gps_card = make_card(nav_row, side="left", fill="both", expand=True)
ctk.CTkLabel(gps_card, text="🛰️  GPS (raw)", font=FONT_SECTION,
             text_color=ACCENT).pack(anchor="w", padx=12, pady=(10, 4))
gps_tiles_frame = ctk.CTkFrame(gps_card, fg_color="transparent")
gps_tiles_frame.pack(fill="both", expand=True, padx=6, pady=(0, 10))
v_gps_lat = add_tile(gps_tiles_frame, "📍", "Latitude")
v_gps_lng = add_tile(gps_tiles_frame, "📍", "Longitude")
gps_row2 = ctk.CTkFrame(gps_card, fg_color="transparent")
gps_row2.pack(fill="both", expand=True, padx=6, pady=(0, 10))
v_gps_sats = add_tile(gps_row2, "🔢", "Satellites")
v_gps_hdop = add_tile(gps_row2, "🎚️", "HDOP")
v_gps_pps = add_tile(gps_row2, "⏱️", "PPS Lock")

imu_card = make_card(nav_row, side="left", fill="both", expand=True)
ctk.CTkLabel(imu_card, text="🧭  IMU (BMI160)", font=FONT_SECTION,
             text_color=ACCENT).pack(anchor="w", padx=12, pady=(10, 4))
imu_tiles_frame = ctk.CTkFrame(imu_card, fg_color="transparent")
imu_tiles_frame.pack(fill="both", expand=True, padx=6, pady=(0, 10))
v_imu_roll = add_tile(imu_tiles_frame, "↩️", "Roll (°)")
v_imu_pitch = add_tile(imu_tiles_frame, "↕️", "Pitch (°)")
imu_row2 = ctk.CTkFrame(imu_card, fg_color="transparent")
imu_row2.pack(fill="both", expand=True, padx=6, pady=(0, 10))
v_imu_yaw = add_tile(imu_row2, "🧭", "Yaw (°)")
v_imu_temp = add_tile(imu_row2, "🌡️", "IMU Temp (°C)")

inav_card = make_card(nav_row, side="left", fill="both", expand=True)
ctk.CTkLabel(inav_card, text="🎯  INAV (fused)", font=FONT_SECTION,
             text_color=ACCENT).pack(anchor="w", padx=12, pady=(10, 4))
inav_tiles_frame = ctk.CTkFrame(inav_card, fg_color="transparent")
inav_tiles_frame.pack(fill="both", expand=True, padx=6, pady=(0, 10))
v_inav_state = add_tile(inav_tiles_frame, "🚦", "State")
v_inav_speed = add_tile(inav_tiles_frame, "💨", "Speed (m/s)")
inav_row2 = ctk.CTkFrame(inav_card, fg_color="transparent")
inav_row2.pack(fill="both", expand=True, padx=6, pady=(0, 10))
v_inav_heading = add_tile(inav_row2, "🧭", "Heading (°)")
v_inav_alt = add_tile(inav_row2, "⛰️", "Altitude (m)")

# ============================================================
# SECTION 2 — SENSOR TILES
# ============================================================

section_title(body, "🫁", "Environment & Air Quality")
sensor_grid = ctk.CTkFrame(body, fg_color="transparent")
sensor_grid.pack(fill="x", padx=4)

sensor_row1 = ctk.CTkFrame(sensor_grid, fg_color="transparent")
sensor_row1.pack(fill="x", pady=2)
v_temp = add_tile(sensor_row1, "🌡️", "Temperature")
v_hum = add_tile(sensor_row1, "💧", "Humidity")
v_co = add_tile(sensor_row1, "☁️", "CO")
v_uvi = add_tile(sensor_row1, "☀️", "UV Index")

sensor_row2 = ctk.CTkFrame(sensor_grid, fg_color="transparent")
sensor_row2.pack(fill="x", pady=2)
v_tvoc = add_tile(sensor_row2, "🧪", "TVOC")
v_eco2 = add_tile(sensor_row2, "🫧", "eCO2")
v_aqi = add_tile(sensor_row2, "📈", "AQI (1-5)")
v_dust = add_tile(sensor_row2, "🌫️", "Dust")

sensor_row3 = ctk.CTkFrame(sensor_grid, fg_color="transparent")
sensor_row3.pack(fill="x", pady=2)
v_sound = add_tile(sensor_row3, "🔊", "Sound")
v_heater = add_tile(sensor_row3, "🔥", "CO Heater V")
v_co_phase = add_tile(sensor_row3, "🔁", "CO Phase")
v_co_raw = add_tile(sensor_row3, "📟", "CO Raw ADC")

# ============================================================
# SECTION 2b — V61 / V62 MESSAGES
# ============================================================

section_title(body, "💬", "Messages")

msgs_v61_v62_card = make_card(body, fill="x", padx=8, pady=(0, 6))

ctk.CTkLabel(msgs_v61_v62_card, text="📨  Message (V61)", font=FONT_LABEL,
             text_color=TXT_SUB).pack(anchor="w", padx=12, pady=(10, 2))
v_msg_v61 = ctk.CTkLabel(msgs_v61_v62_card, text="—", font=("Segoe UI", 13),
                          text_color=TXT_MAIN, anchor="w", wraplength=900,
                          justify="left")
v_msg_v61.pack(anchor="w", padx=12, pady=(0, 8))

ctk.CTkLabel(msgs_v61_v62_card, text="📨  Message (V62)", font=FONT_LABEL,
             text_color=TXT_SUB).pack(anchor="w", padx=12, pady=(0, 2))
v_msg_v62 = ctk.CTkLabel(msgs_v61_v62_card, text="—", font=("Segoe UI", 13),
                          text_color=TXT_MAIN, anchor="w", wraplength=900,
                          justify="left")
v_msg_v62.pack(anchor="w", padx=12, pady=(0, 10))

# ============================================================
# SECTION 3 — HISTORY GRAPHS (2x2) WITH SHARED TIME RANGE
# ============================================================

section_title(body, "📊", "Sensor History")

graph_controls = ctk.CTkFrame(body, fg_color=CARD, corner_radius=12,
                               border_width=1, border_color=BORDER)
graph_controls.pack(fill="x", padx=8, pady=(0, 6))
ctk.CTkLabel(graph_controls, text="Time Range", font=FONT_LABEL,
             text_color=TXT_SUB).pack(side="left", padx=(14, 8), pady=10)

history_var = ctk.StringVar(value="24")
HISTORY_LABELS = {"1": "1h", "3": "3h", "24": "24h", "48": "48h", "168": "168h (7d)"}


def _on_history_change(_choice_label):
    hours = next(h for h, lbl in HISTORY_LABELS.items() if lbl == _choice_label)
    history_var.set(hours)
    trigger_graph_refresh()


option_history = ctk.CTkOptionMenu(
    graph_controls, values=list(HISTORY_LABELS.values()),
    command=_on_history_change,
    fg_color="#1a2332", button_color="#1e3a5f", button_hover_color=ACCENT,
    dropdown_fg_color=CARD, text_color=TXT_MAIN, font=FONT_LABEL,
)
option_history.set(HISTORY_LABELS["24"])
option_history.pack(side="left", padx=8, pady=10)

label_graph_status = ctk.CTkLabel(graph_controls, text="", font=FONT_SMALL,
                                   text_color=TXT_DIM)
label_graph_status.pack(side="left", padx=14)

graph_card = ctk.CTkFrame(body, fg_color=CARD, corner_radius=14,
                           border_width=1, border_color=BORDER)
graph_card.pack(fill="both", expand=True, padx=8, pady=(0, 6))

fig = Figure(figsize=(13, 7), dpi=100)
fig.patch.set_facecolor(BG)
axes = fig.subplots(2, 2)
GRAPH_ORDER = list(GRAPH_PINS.items())
GRAPH_COLORS = ["#00bfff", "#a78bfa", "#f97316", "#34d399"]


def _style_axis(ax):
    ax.set_facecolor(CARD)
    ax.tick_params(colors=TXT_SUB, labelsize=8)
    ax.xaxis.label.set_color(TXT_SUB)
    ax.yaxis.label.set_color(TXT_SUB)
    ax.title.set_color(TXT_MAIN)
    ax.grid(True, color="#1a2332", linestyle="--", linewidth=0.6)
    for spine in ax.spines.values():
        spine.set_color(BORDER)


for ax in axes.flat:
    _style_axis(ax)
fig.tight_layout(pad=2.4)

graph_canvas = FigureCanvasTkAgg(fig, master=graph_card)
graph_canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=10)

# ============================================================
# SECTION 4 — REMOTE CONTROL + DIAGNOSTICS
# ============================================================

section_title(body, "🛠️", "Diagnostics & Remote control")

diag_top = ctk.CTkFrame(body, fg_color="transparent")
diag_top.pack(fill="x", padx=4)

# Keep the engineering message as a single current-state message.
# Updating the label replaces the previous value; it never appends/stack messages.
msg_card = make_card(diag_top, side="left", fill="both", expand=True)
ctk.CTkLabel(msg_card, text="📝  Engineering Message (V19)", font=FONT_LABEL,
             text_color=TXT_SUB).pack(anchor="w", padx=12, pady=(10, 2))
v_eng_msg = ctk.CTkLabel(msg_card, text="—", font=("Segoe UI", 13),
                          text_color=TXT_MAIN, anchor="w", wraplength=520,
                          justify="left")
v_eng_msg.pack(anchor="w", padx=12, pady=(0, 6))

# V34 and V51 follow the same pattern as V19: a titled label line plus a
# single-value line below it. The value widget is updated with
# configure(text=...) only, so a new value replaces the previous one
# instead of stacking another label/message below it.
ctk.CTkLabel(msg_card, text="🔧  Reset Reason (V34)", font=FONT_LABEL,
             text_color=TXT_SUB).pack(anchor="w", padx=12, pady=(6, 2))
v_reset_reason = ctk.CTkLabel(msg_card, text="—", font=("Segoe UI", 13),
                               text_color=TXT_MAIN, anchor="w", wraplength=520,
                               justify="left")
v_reset_reason.pack(anchor="w", padx=12, pady=(0, 6))

ctk.CTkLabel(msg_card, text="🕒  RTC Checkpoint (V51)", font=FONT_LABEL,
             text_color=TXT_SUB).pack(anchor="w", padx=12, pady=(6, 2))
v_rtc_checkpoint = ctk.CTkLabel(msg_card, text="—", font=("Segoe UI", 13),
                                 text_color=TXT_MAIN, anchor="w", wraplength=520,
                                 justify="left")
v_rtc_checkpoint.pack(anchor="w", padx=12, pady=(0, 10))

# Right-side controls are kept in their own vertical column so the diagnostic
# value stays at the top and the hard-reset control is clearly separated below.
# No fixed height/propagation lock: the column sizes itself to its three
# stacked cards, and msg_card (fill="both") on the left matches that height
# dynamically, so the left card's bottom always lines up with Reset Station.
diag_right = ctk.CTkFrame(diag_top, fg_color="transparent", width=250)
diag_right.pack(side="left", fill="y", padx=(6, 0), pady=6)

# App / telemetry diagnostic card (V4)
diag_rnd_card = ctk.CTkFrame(diag_right, fg_color=CARD, corner_radius=14,
                             border_width=1, border_color=BORDER)
diag_rnd_card.pack(fill="x", pady=(6, 6))

ctk.CTkLabel(diag_rnd_card, text="DIAG (App/Telem. RND# Alive)",
             font=("Segoe UI", 14, "bold"), text_color=TXT_MAIN,
             anchor="w", justify="left", wraplength=210).pack(
                 anchor="w", padx=12, pady=(12, 8))

v_app_telem_rnd = ctk.CTkLabel(
    diag_rnd_card, text="—", font=("Segoe UI", 18, "bold"),
    text_color=TXT_SUB, anchor="w"
)
v_app_telem_rnd.pack(anchor="w", padx=12, pady=(0, 14))

# Clear card — resets all displayed values to their blank state, locally only.
# Placed above the hard-reset card, below the DIAG (App/Telem. RND# Alive) card.
clear_card = ctk.CTkFrame(diag_right, fg_color=CARD, corner_radius=14,
                          border_width=1, border_color=BORDER)
clear_card.pack(fill="x", pady=(5, 6), after=diag_rnd_card)

ctk.CTkLabel(clear_card, text="🧹  Clear Displayed Values", font=FONT_LABEL,
             text_color=TXT_SUB).pack(anchor="w", padx=12, pady=(10, 6))

button_clear = ctk.CTkButton(clear_card, text="🧹  Clear", command=lambda: clear_dashboard(),
                              fg_color="#1e3a5f", hover_color="#2a4a75",
                              font=FONT_LABEL, corner_radius=8, width=180)
button_clear.pack(padx=12, pady=(0, 14))

# Remote hard reset is intentionally lower on the right side.
reset_card = ctk.CTkFrame(diag_right, fg_color=CARD, corner_radius=14,
                          border_width=1, border_color=BORDER)
reset_card.pack(fill="x", pady=(5, 6), after=clear_card)

ctk.CTkLabel(reset_card, text="🔁  Remote Hard Reset (V55)", font=FONT_LABEL,
             text_color=TXT_SUB).pack(anchor="w", padx=12, pady=(10, 6))


def do_remote_reset():
    if not messagebox.askyesno("Confirm Reset",
                                "Send a hard-reset command to the ESP32?"):
        return

    def _worker():
        # v1.5 / firmware v3.9.212: explicitly arm the edge-triggered reset.
        # LOW-before-HIGH also clears a stale server-side HIGH left by an
        # interrupted earlier request. The trailing LOW prepares the next use.
        armed = write_pin(RESET_PIN, 0)
        time.sleep(0.2)
        triggered = False
        released = False
        try:
            triggered = write_pin(RESET_PIN, 1)
            time.sleep(0.5)
        finally:
            released = write_pin(RESET_PIN, 0)
        delivered = armed and triggered and released
        _event_journal.log(
            "REMOTE_RESET_COMMAND", severity="WARNING",
            confidence="confirmed" if delivered else "observed",
            details=("V55 LOW-HIGH-LOW delivered"
                     if delivered else
                     f"V55 delivery incomplete: arm={armed}, "
                     f"trigger={triggered}, release={released}"))

    threading.Thread(target=_worker, daemon=True).start()


button_reset = ctk.CTkButton(reset_card, text="⚠  Reset Station", command=do_remote_reset,
                              fg_color="#5f1e1e", hover_color="#7a2828",
                              font=FONT_LABEL, corner_radius=8, width=180)
button_reset.pack(padx=12, pady=(0, 14))

# Status flags chips
flags_card = ctk.CTkFrame(body, fg_color=CARD, corner_radius=14,
                           border_width=1, border_color=BORDER)
flags_card.pack(fill="x", padx=8, pady=6)
ctk.CTkLabel(flags_card, text="🚩  Status Flags (V20)", font=FONT_LABEL,
             text_color=TXT_SUB).pack(anchor="w", padx=12, pady=(10, 4))
flags_wrap = ctk.CTkFrame(flags_card, fg_color="transparent")
flags_wrap.pack(fill="x", padx=8, pady=(0, 12))
flag_chip_labels = []
for bit, name in STATUS_FLAGS:
    chip = ctk.CTkLabel(flags_wrap, text=name, font=FONT_SMALL,
                         text_color="#0f1623", fg_color=TXT_DIM,
                         corner_radius=8, padx=10, pady=4)
    chip.pack(side="left", padx=4, pady=4)
    flag_chip_labels.append((bit, chip))

# Diagnostic pin table
diag_table_card = ctk.CTkFrame(body, fg_color=CARD, corner_radius=14,
                                border_width=1, border_color=BORDER)
diag_table_card.pack(fill="x", padx=8, pady=6)
ctk.CTkLabel(diag_table_card, text="📟  Performance / I²C Diagnostics (V35-V50)",
             font=FONT_LABEL, text_color=TXT_SUB).pack(anchor="w", padx=12, pady=(10, 6))

diag_grid = ctk.CTkFrame(diag_table_card, fg_color="transparent")
diag_grid.pack(fill="x", padx=8, pady=(0, 12))

DIAG_LABELS = [
    ("PPS count", "pps_count", ""),
    ("PPS age", "pps_age_ms", " ms"),
    ("Max loop gap", "loop_gap_max", " ms"),
    ("Max sensor cycle", "sensor_cycle_max", " ms"),
    ("Max Blynk.run()", "blynk_run_max", " ms"),
    ("Max GPS feed", "gps_feed_max", " ms"),
    ("Max I2C path", "i2c_path_max", " ms"),
    ("Max FAST send", "fast_send_max", " ms"),
    ("Max SLOW send", "slow_send_max", " ms"),
    ("Max PPS→LED", "pps_led_max", " ms"),
    ("Last I2C device", "i2c_device", ""),
    ("Last I2C op", "i2c_op", ""),
    ("I2C errors", "i2c_errors", ""),
    ("Max AHT21 read", "aht21_max", " ms"),
    ("Max ENS160 read", "ens160_max", " ms"),
    ("Max BMI160 read", "bmi160_max", " ms"),
]

diag_value_labels = {}
COLS = 4
for idx, (label, key, unit) in enumerate(DIAG_LABELS):
    r, c = divmod(idx, COLS)
    cell = ctk.CTkFrame(diag_grid, fg_color="#131b2b", corner_radius=8,
                         border_width=1, border_color=BORDER)
    cell.grid(row=r, column=c, padx=5, pady=5, sticky="nsew")
    diag_grid.grid_columnconfigure(c, weight=1)
    ctk.CTkLabel(cell, text=label, font=FONT_SMALL, text_color=TXT_SUB,
                 anchor="w").pack(fill="x", padx=8, pady=(6, 0))
    val = ctk.CTkLabel(cell, text="N/A", font=("Segoe UI", 13, "bold"),
                        text_color=TXT_MAIN, anchor="w")
    val.pack(fill="x", padx=8, pady=(0, 6))
    diag_value_labels[key] = (val, unit)

# ============================================================
# SECTION 5 — LOCATION / MAP
# ============================================================

section_title(body, "🗺️", "Location")

map_row = ctk.CTkFrame(body, fg_color="transparent")
map_row.pack(fill="both", expand=True, padx=4, pady=(0, 20))

map_info_card = make_card(map_row, side="left", fill="both")
ctk.CTkLabel(map_info_card, text="📍  Current Position", font=FONT_SECTION,
             text_color=ACCENT).pack(anchor="w", padx=14, pady=(12, 6))

v_map_source = ctk.CTkLabel(map_info_card, text="Source: —", font=FONT_LABEL,
                             text_color=TXT_SUB, anchor="w")
v_map_source.pack(anchor="w", padx=14, pady=2)

v_map_coords = ctk.CTkLabel(map_info_card, text="Lat: —\nLng: —", font=("Segoe UI", 16, "bold"),
                             text_color=TXT_MAIN, anchor="w", justify="left")
v_map_coords.pack(anchor="w", padx=14, pady=8)

v_map_extra = ctk.CTkLabel(map_info_card,
                            text="Sats: —   HDOP: —   Speed: —   Heading: —",
                            font=FONT_SMALL, text_color=TXT_DIM, anchor="w")
v_map_extra.pack(anchor="w", padx=14, pady=(0, 8))

_current_coords = {"lat": None, "lng": None}


def open_in_maps():
    lat, lng = _current_coords["lat"], _current_coords["lng"]
    if lat is None or lng is None:
        messagebox.showinfo("No fix", "No valid location available yet.")
        return
    webbrowser.open(f"https://www.google.com/maps?q={lat},{lng}")


ctk.CTkButton(map_info_card, text="🌐  Open in Google Maps", command=open_in_maps,
              fg_color="#1e3a5f", hover_color=ACCENT, font=FONT_LABEL,
              corner_radius=8).pack(anchor="w", padx=14, pady=(0, 14))

map_plot_card = ctk.CTkFrame(map_row, fg_color=CARD, corner_radius=14,
                              border_width=1, border_color=BORDER,
                              height=640)
map_plot_card.pack(side="left", fill="both", expand=True, padx=6, pady=6)
map_plot_card.pack_propagate(False)

map_toolbar = ctk.CTkFrame(map_plot_card, fg_color="transparent")
map_toolbar.pack(fill="x", padx=10, pady=(10, 0))


def show_all_points():
    if len(_position_trail) < 2:
        messagebox.showinfo("Not enough data", "Need at least two recorded fixes.")
        return
    lats = [p[0] for p in _position_trail]
    lngs = [p[1] for p in _position_trail]
    top_left = (max(lats), min(lngs))
    bottom_right = (min(lats), max(lngs))
    map_widget.fit_bounding_box(top_left, bottom_right)


ctk.CTkButton(map_toolbar, text="🔎  Show All Points", command=show_all_points,
              fg_color="#1e3a5f", hover_color=ACCENT, font=FONT_LABEL,
              corner_radius=8, width=170).pack(side="left")

# Real OpenStreetMap tiles — free, no API key required.
map_widget = tkintermapview.TkinterMapView(map_plot_card, corner_radius=10)
map_widget.pack(fill="both", expand=True, padx=10, pady=10)
map_widget.set_position(0, 0)
map_widget.set_zoom(3)

_position_trail = []      # in-memory trail of (lat, lng) collected this session
MAX_TRAIL_POINTS = 500
_map_marker = None
_map_path = None
_map_has_centered = False


# ============================================================
# GRAPH REFRESH (background thread, blocking HTTP calls)
# ============================================================

_graph_lock = threading.Lock()
_graph_busy = False
_graph_history_cache = {pin: [] for pin in GRAPH_PINS.values()}
_live_graph_lock = threading.Lock()
# 168 hours sampled every 30 seconds, plus margin for scheduler jitter.
_live_graph_points = {
    pin: deque(maxlen=22000) for pin in GRAPH_PINS.values()
}
_last_live_graph_sample = 0.0
_graph_source_note = "waiting for data"


def trigger_graph_refresh():
    global _graph_busy
    # Tk variables are main-thread objects. Pass this value into the worker.
    hours = int(history_var.get())
    with _graph_lock:
        if _graph_busy:
            return
        _graph_busy = True
    label_graph_status.configure(text="Refreshing…", text_color=TXT_DIM)
    threading.Thread(target=_graph_worker, args=(hours,), daemon=True).start()


def _combined_graph_points(pin, hours):
    """Merge archived server history with live samples from this app run."""
    cutoff = datetime.now() - timedelta(hours=hours)
    with _graph_lock:
        archived = list(_graph_history_cache.get(pin, ()))
    with _live_graph_lock:
        live = list(_live_graph_points.get(pin, ()))

    merged = {
        int(point_time.timestamp() * 1000): (point_time, value)
        for point_time, value in archived + live
        if point_time >= cutoff
    }
    return sorted(merged.values(), key=lambda point: point[0])


def _record_live_graph_samples(values, now_monotonic):
    """Store one bounded, in-memory graph sample while telemetry is live."""
    global _last_live_graph_sample
    if now_monotonic - _last_live_graph_sample < LIVE_GRAPH_SAMPLE_S:
        return

    sample_time = datetime.now()
    pending = []
    for pin, key in GRAPH_VALUE_KEYS.items():
        try:
            value = float(values.get(key))
        except (TypeError, ValueError):
            continue
        if value <= UNAVAILABLE_THRESHOLD:
            continue
        pending.append((pin, (sample_time, value)))

    if not pending:
        return
    with _live_graph_lock:
        for pin, point in pending:
            _live_graph_points[pin].append(point)
    _last_live_graph_sample = now_monotonic


def _graph_worker(hours):
    global _graph_busy, _graph_source_note
    fetched = {}
    errors = []
    try:
        for name, pin in GRAPH_ORDER:
            points, error = fetch_history(pin, hours)
            fetched[pin] = points
            if error:
                errors.append(f"{name}: {error}")

        with _graph_lock:
            _graph_history_cache.update(fetched)

        results = [
            (name, _combined_graph_points(pin, hours))
            for name, pin in GRAPH_ORDER
        ]
        archived_count = sum(len(points) for points in fetched.values())
        if errors:
            _graph_source_note = "live data; server history unavailable"
        elif archived_count == 0:
            _graph_source_note = "live data; server history has no recent points"
        else:
            _graph_source_note = "server history + live data"
        _result_queue.put(("graph", (results, hours, _graph_source_note)))
    except Exception as exc:
        _graph_source_note = f"graph refresh failed: {type(exc).__name__}"
        _result_queue.put(("graph_error", _graph_source_note))
    finally:
        # An exception must not leave all future graph refreshes suppressed.
        with _graph_lock:
            _graph_busy = False


def _redraw_graph_from_cache():
    """Fast UI-only redraw; no HTTP requests are made here."""
    hours = int(history_var.get())
    results = [
        (name, _combined_graph_points(pin, hours))
        for name, pin in GRAPH_ORDER
    ]
    _apply_graph_data(results, hours, _graph_source_note)


def _apply_graph_data(results, hours, source_note=""):
    for idx, (name, points) in enumerate(results):
        ax = axes.flat[idx]
        ax.clear()
        _style_axis(ax)
        if points:
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            ax.plot(xs, ys, color=GRAPH_COLORS[idx % len(GRAPH_COLORS)],
                    linewidth=1.4, marker="o", markersize=2)
            ax.set_title(f"{name} — {hours}h", fontsize=11, fontweight="bold")
            span = xs[-1] - xs[0] if len(xs) > 1 else timedelta(hours=1)
            if span.total_seconds() < 86400:
                fmt_str = "%H:%M"
            elif span.days < 31:
                fmt_str = "%d %H:%M"
            else:
                fmt_str = "%d/%m"
            ax.xaxis.set_major_formatter(mdates.DateFormatter(fmt_str))
            for label in ax.get_xticklabels():
                label.set_rotation(25)
        else:
            ax.set_title(f"{name} — no data", fontsize=11, fontweight="bold")
            ax.text(0.5, 0.5, "No history yet", ha="center", va="center",
                    color=TXT_DIM, transform=ax.transAxes)
    fig.tight_layout(pad=2.4)
    graph_canvas.draw()
    suffix = f" — {source_note}" if source_note else ""
    label_graph_status.configure(
        text=f"Updated {datetime.now().strftime('%H:%M:%S')}{suffix}",
        text_color=TXT_DIM)


# ============================================================
# FAST POLL (background thread, queue -> main thread)
# ============================================================

_result_queue = queue.Queue()

# Shared live-data watchdog.  This is deliberately separate from the GUI
# thread so the dashboard can keep rendering values even if a status endpoint
# is unavailable.
_activity_lock = threading.Lock()
_last_data_signature = None
_heartbeat_samples = deque(maxlen=3)
_last_heartbeat_activity = None
_last_pair_key = None
_last_pair_activity = None
_last_data_wallclock = None


def _poll_data_worker():
    """Read all dashboard datastreams exactly as before.

    This worker is deliberately independent of the connection-status check.
    A broken/unsupported Blynk connection-status endpoint must never prevent
    sensor values from reaching the dashboard.
    """
    global _last_data_signature, _last_pair_key, _last_pair_activity
    global _last_data_wallclock

    while True:
        values = {}
        for name, pin in ALL_READ_PINS.items():
            values[name] = read_pin(pin)

        # Any changing datastream proves that fresh data is reaching the
        # dashboard. This is a fallback for firmware versions where the
        # dedicated heartbeat pins are not changing.
        signature = tuple(
            (name, values.get(name))
            for name in sorted(values)
            if values.get(name) is not None
        )
        now = time.monotonic()
        key61 = _message_pair_key(values.get("msg_v61"))
        key62 = _message_pair_key(values.get("msg_v62"))
        matched_pair_key = key61 if key61 is not None and key61 == key62 else None

        changed = False
        with _activity_lock:
            if _last_data_signature is None:
                _last_data_signature = signature
            elif signature != _last_data_signature:
                _last_data_signature = signature
                changed = True

            # V61/V62 are fallback stream evidence only after both halves have
            # the same record key. A mixed pair must never refresh liveness.
            if matched_pair_key is not None:
                if _last_pair_key is None:
                    _last_pair_key = matched_pair_key
                elif matched_pair_key != _last_pair_key:
                    _last_pair_key = matched_pair_key
                    _last_pair_activity = now
                    _last_data_wallclock = datetime.now()

        _event_monitor.observe_data(values, key61, key62, matched_pair_key)

        # Record graph samples only when the stream advances. A server TRUE
        # response by itself is not data and must not duplicate cached values.
        if changed:
            _record_live_graph_samples(values, now)

        _result_queue.put(("data", values))
        threading.Event().wait(FAST_POLL_MS / 1000.0)


def _parse_blynk_bool(text):
    """Parse plain, quoted, or JSON-ish Blynk boolean responses."""
    if text is None:
        return None
    s = str(text).strip().strip('"').strip("'").strip()
    # Legacy servers may wrap the scalar in a one-item JSON array.
    if s.startswith("[") and s.endswith("]"):
        s = s[1:-1].strip().strip('"').strip("'").strip()
    s = s.lower()
    if s in ("true", "1", "yes", "online"):
        return True
    if s in ("false", "0", "no", "offline"):
        return False
    return None


def _status_endpoint():
    """Try the local Blynk hardware-status endpoint without blocking data."""
    urls = (
        f"http://{BLYNK_SERVER}:{BLYNK_PORT}/{BLYNK_AUTH}/isHardwareConnected",
        f"http://{BLYNK_SERVER}:{BLYNK_PORT}/external/api/isHardwareConnected"
        f"?token={BLYNK_AUTH}",
    )
    for url in urls:
        try:
            r = _session().get(url, timeout=HEALTH_HTTP_TIMEOUT_S)
            if r.status_code == 200:
                result = _parse_blynk_bool(r.text)
                if result is not None:
                    return result
        except Exception:
            pass
    return None


def _decide_connection(blynk_state, heartbeat_available,
                       heartbeat_fresh, pair_fresh):
    """Return ``(connected, evidence)`` using a strict two-factor test.

    Factor 1: Blynk says the ESP32 hardware session is connected.
    Factor 2: V4 advanced recently. A matched V61/V62 key is fallback only
    when V4 cannot be read at all; it must not mask a reachable, stale V4.

    Neither factor is sufficient alone: isHardwareConnected can outlive a
    stalled data plane briefly, while cached V-pin reads can remain available
    after the ESP32 session has disappeared.
    """
    if blynk_state is None:
        return False, "server status unavailable"
    if blynk_state is False:
        return False, "device not connected to Blynk"
    if heartbeat_fresh:
        return True, "Blynk + V4 heartbeat"
    if not heartbeat_available and pair_fresh:
        return True, "Blynk + V61/V62 fallback"
    return False, "Blynk connected; no live telemetry"


def _poll_health_worker():
    """Poll primary V4 liveness and Blynk status once per second.

    CONNECTED is deliberately strict: the server must report the ESP32 session
    connected AND V4 must have advanced. V61/V62 is fallback only if the V4
    GET itself is unavailable; a readable but unchanged V4 is stale evidence
    and becomes NO LIVE DATA after the short heartbeat freshness deadline.
    """
    global _last_heartbeat_activity
    global _last_data_wallclock

    while True:
        cycle_started = time.monotonic()
        # Read the smallest, fastest stream marker first. This call has a short
        # independent timeout so a health cycle cannot block the full data poll.
        heartbeat_value = read_pin(LIVE_TELEM_PIN, timeout=HEALTH_HTTP_TIMEOUT_S)
        heartbeat_available = heartbeat_value is not None
        blynk_state = _status_endpoint()
        now = time.monotonic()

        with _activity_lock:
            if heartbeat_value is None:
                # The next successful value must establish a new baseline;
                # do not join observations across a failed HTTP read.
                _heartbeat_samples.clear()
            else:
                heartbeat_value = str(heartbeat_value).strip()
                while (_heartbeat_samples and
                       now - _heartbeat_samples[0][1] > HEARTBEAT_CONFIRM_WINDOW_S):
                    _heartbeat_samples.popleft()
                if not _heartbeat_samples:
                    _heartbeat_samples.append((heartbeat_value, now))
                elif heartbeat_value != _heartbeat_samples[-1][0]:
                    _heartbeat_samples.append((heartbeat_value, now))
                    # Three arrivals (baseline + two subsequent transitions)
                    # must fit in one bounded window before V4 proves a
                    # continuing live stream.
                    if (len(_heartbeat_samples) == 3 and
                            now - _heartbeat_samples[0][1]
                            <= HEARTBEAT_CONFIRM_WINDOW_S):
                        _last_heartbeat_activity = now
                        _last_data_wallclock = datetime.now()

            heartbeat_fresh = (
                _last_heartbeat_activity is not None
                and (now - _last_heartbeat_activity) <= HEARTBEAT_TIMEOUT_S
            )
            pair_fresh = (
                _last_pair_activity is not None
                and (now - _last_pair_activity) <= PAIR_FALLBACK_TIMEOUT_S
            )
            last_live = _last_data_wallclock

        device_connected, source = _decide_connection(
            blynk_state, heartbeat_available, heartbeat_fresh, pair_fresh)

        _event_monitor.observe_health(
            blynk_state, heartbeat_available, heartbeat_fresh,
            heartbeat_value, pair_fresh, source)

        # This is the only one-second data-widget update. It is applied on the
        # Tk main thread by _poll_queue; no other dashboard value is included.
        if heartbeat_value is not None:
            _result_queue.put(("heartbeat", heartbeat_value))

        _result_queue.put(("status", {
            "connected": device_connected,
            "source": source,
            "last_live": last_live,
            "server_connected": blynk_state,
            "data_fresh": heartbeat_fresh or
                          (not heartbeat_available and pair_fresh),
            "heartbeat_available": heartbeat_available,
            "heartbeat_fresh": heartbeat_fresh,
            "pair_fresh": pair_fresh,
        }))

        elapsed = time.monotonic() - cycle_started
        threading.Event().wait(max(0.0, HEALTH_POLL_MS / 1000.0 - elapsed))


def _poll_worker():
    """Start independent full-data and one-second health workers."""
    threading.Thread(target=_poll_data_worker, daemon=True).start()
    threading.Thread(target=_poll_health_worker, daemon=True).start()


def _poll_queue():
    try:
        while True:
            kind, payload = _result_queue.get_nowait()
            if kind == "data":
                _apply_values(payload)
            elif kind == "heartbeat":
                _apply_heartbeat(payload)
            elif kind == "status":
                _apply_connection_status(payload)
            elif kind == "graph":
                results, hours, source_note = payload
                _apply_graph_data(results, hours, source_note)
            elif kind == "graph_error":
                label_graph_status.configure(text=payload, text_color=RED)
    except queue.Empty:
        pass
    app.after(200, _poll_queue)

def _as_float(values, key):
    v = values.get(key)
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


_device_connected = False
_server_link_state = None
_last_values = {}

# V61 and V62 are independent Blynk pins. A poll can read one before its
# update and the other after it. Cache both halves and change both labels only
# after the embedded gpsUnix/deviceMs record keys match.
_pending_message_pair = {"msg_v61": None, "msg_v62": None}
_displayed_message_pair_key = None


def _apply_heartbeat(value):
    """Update only the dedicated V4 Alive widget from the one-second worker."""
    v_app_telem_rnd.configure(text=str(value) if value is not None else "—")


def _message_pair_key(value):
    """Return the canonical (gpsUnix, deviceMs) key from a telemetry CSV."""
    if value is None:
        return None
    fields = str(value).split(",", 2)
    if len(fields) < 2:
        return None
    try:
        return str(int(fields[0].strip())), str(int(fields[1].strip()))
    except (TypeError, ValueError):
        return None


def _apply_message_pair(values):
    """Display only a complete V61/V62 record; never show mixed cycles."""
    global _displayed_message_pair_key

    for name in ("msg_v61", "msg_v62"):
        value = values.get(name)
        if value:
            _pending_message_pair[name] = str(value)

    value61 = _pending_message_pair["msg_v61"]
    value62 = _pending_message_pair["msg_v62"]
    key61 = _message_pair_key(value61)
    key62 = _message_pair_key(value62)
    if key61 is None or key61 != key62:
        # One pin advanced first. Keep displaying the previous complete pair.
        return
    if key61 == _displayed_message_pair_key:
        return

    v_msg_v61.configure(text=value61)
    v_msg_v62.configure(text=value62)
    _displayed_message_pair_key = key61


def _apply_connection_status(status):
    """Update connection widgets without inventing a fresh-data timestamp."""
    global _device_connected, _server_link_state
    if isinstance(status, dict):
        _device_connected = bool(status.get("connected"))
        source = status.get("source", "unknown")
        last_live = status.get("last_live")
        _server_link_state = status.get("server_connected")
    else:
        # Compatibility for any already-queued result during a code reload.
        _device_connected = bool(status)
        source = "unknown"
        last_live = None
        _server_link_state = None

    if _device_connected:
        label_status.configure(text="⬤  CONNECTED", text_color=GREEN)
        if last_live is not None:
            live_text = last_live.strftime("%H:%M:%S")
            label_last_update.configure(
                text=f"Last telemetry change: {live_text}")
        else:
            label_last_update.configure(text="Live telemetry confirmed")
    else:
        if _server_link_state is True:
            label_status.configure(text="⬤  NO LIVE DATA", text_color=AMBER)
            label_wifi.configure(text="📶  WiFi: NO LIVE DATA", text_color=AMBER)
        elif _server_link_state is False:
            label_status.configure(text="⬤  DEVICE OFFLINE", text_color=RED)
            label_wifi.configure(text="📶  WiFi: OFFLINE", text_color=RED)
        else:
            label_status.configure(text="⬤  SERVER UNAVAILABLE", text_color=RED)
            label_wifi.configure(text="📶  WiFi: UNKNOWN", text_color=RED)
        if last_live is not None:
            label_last_update.configure(
                text=f"Last telemetry change: {last_live.strftime('%H:%M:%S')}"
                     f"  ·  check: {source}")
        else:
            label_last_update.configure(text=f"No live telemetry yet  ·  check: {source}")


def _apply_values(values):
    global _last_values
    _last_values = values

    # Connection state is maintained separately. Never infer it from cached
    # virtual-pin values.
    rssi = _as_float(values, "rssi")
    if _device_connected:
        if rssi is not None and rssi > UNAVAILABLE_THRESHOLD:
            quality = rssi_to_quality(int(rssi))
            label_wifi.configure(text=f"📶  {int(rssi)} dBm ({quality}%)",
                                  text_color=wifi_color_for(quality))
        else:
            label_wifi.configure(text="📶  WiFi: —", text_color=AMBER)
    elif _server_link_state is True:
        label_wifi.configure(text="📶  WiFi: NO LIVE DATA", text_color=AMBER)
    elif _server_link_state is None:
        label_wifi.configure(text="📶  WiFi: UNKNOWN", text_color=RED)
    else:
        label_wifi.configure(text="📶  WiFi: OFFLINE", text_color=RED)

    # ---- GPS ----
    v_gps_lat.configure(text=fmt(values.get("lat"), decimals=6))
    v_gps_lng.configure(text=fmt(values.get("lng"), decimals=6))
    v_gps_sats.configure(text=fmt_int(values.get("sats")))
    v_gps_hdop.configure(text=fmt(values.get("hdop"), decimals=2))
    pps = _as_float(values, "pps")
    v_gps_pps.configure(text=("LOCKED" if pps == 1 else "NO LOCK" if pps == 0 else "N/A"),
                         text_color=(GREEN if pps == 1 else RED if pps == 0 else TXT_MAIN))

    # ---- IMU ----
    v_imu_roll.configure(text=fmt(values.get("imu_roll")))
    v_imu_pitch.configure(text=fmt(values.get("imu_pitch")))
    v_imu_yaw.configure(text=fmt(values.get("imu_yaw")))
    v_imu_temp.configure(text=fmt(values.get("imu_temp")))

    # ---- INAV ----
    inav_state = _as_float(values, "inav_state")
    if inav_state is not None and int(inav_state) in INAV_STATE_NAMES:
        s = int(inav_state)
        v_inav_state.configure(text=INAV_STATE_NAMES[s], text_color=INAV_STATE_COLORS[s])
    else:
        v_inav_state.configure(text="N/A", text_color=TXT_MAIN)
    v_inav_speed.configure(text=fmt(values.get("inav_speed")))
    v_inav_heading.configure(text=fmt(values.get("inav_heading")))
    v_inav_alt.configure(text=fmt(values.get("inav_alt")))

    # ---- Sensors ----
    v_temp.configure(text=fmt(values.get("temperature"), " °C"))
    v_hum.configure(text=fmt(values.get("humidity"), " %"))
    v_co.configure(text=fmt(values.get("co_ppm"), " ppm"))
    v_uvi.configure(text=fmt(values.get("uvi")))
    v_tvoc.configure(text=fmt(values.get("tvoc"), " ppb", 0))
    v_eco2.configure(text=fmt(values.get("eco2"), " ppm", 0))
    v_aqi.configure(text=fmt(values.get("aqi"), "", 0))
    v_dust.configure(text=fmt(values.get("dust"), " mg/m³", 3))
    v_sound.configure(text=fmt(values.get("sound_db"), " dB"))
    v_heater.configure(text=fmt(values.get("co_heater_v"), " V", 2))
    v_co_phase.configure(text=("HEAT" if _as_float(values, "co_phase") == 1
                                else "MEASURE" if _as_float(values, "co_phase") == 0
                                else "N/A"))
    v_co_raw.configure(text=fmt(values.get("co_raw"), "", 0))

    # ---- Debug / diagnostics ----
    # Replace the current widgets in-place. Do not append new messages/labels.
    v_eng_msg.configure(text=values.get("eng_msg") or "—")
    _apply_message_pair(values)

    # V34 is a single current reset-reason value. If the server ever returns
    # a multiline value, keep only the newest non-empty line so old messages
    # cannot visually stack in the dashboard.
    reset_reason = values.get("reset_reason")
    if reset_reason is None:
        # Keep V34 visually empty until the first actual value is read.
        reset_reason_text = "—"
    else:
        reset_reason_lines = [line.strip() for line in str(reset_reason).splitlines()
                              if line.strip()]
        reset_reason_text = reset_reason_lines[-1] if reset_reason_lines else "—"
    v_reset_reason.configure(text=reset_reason_text)

    v_rtc_checkpoint.configure(text=values.get("rtc_checkpoint") or "—")

    status_flags = _as_float(values, "status_flags")
    flags_int = int(status_flags) if status_flags is not None else 0
    for bit, chip in flag_chip_labels:
        is_set = bool(flags_int & bit)
        good = is_set if bit in STATUS_GOOD_WHEN_SET else not is_set
        chip.configure(fg_color=(GREEN if good else RED))

    for key, (label_widget, unit) in diag_value_labels.items():
        if key in ("i2c_device",):
            v = _as_float(values, key)
            label_widget.configure(text=I2C_DEVICE_NAMES.get(int(v), "N/A") if v is not None else "N/A")
        elif key in ("i2c_op",):
            v = _as_float(values, key)
            label_widget.configure(text=I2C_OP_NAMES.get(int(v), "N/A") if v is not None else "N/A")
        else:
            label_widget.configure(text=fmt_int(values.get(key), unit))

    # ---- Location / map ----
    inav_lat = _as_float(values, "inav_lat")
    inav_lng = _as_float(values, "inav_lng")
    gps_lat = _as_float(values, "lat")
    gps_lng = _as_float(values, "lng")

    lat = lng = None
    source = "—"
    if inav_lat is not None and inav_lng is not None and inav_lat > UNAVAILABLE_THRESHOLD:
        lat, lng, source = inav_lat, inav_lng, "INAV (fused)"
    elif gps_lat is not None and gps_lng is not None and gps_lat != 0:
        lat, lng, source = gps_lat, gps_lng, "GPS (raw)"

    if lat is not None and lng is not None:
        _current_coords["lat"], _current_coords["lng"] = lat, lng
        v_map_source.configure(text=f"Source: {source}")
        v_map_coords.configure(text=f"Lat: {lat:.6f}\nLng: {lng:.6f}")
        v_map_extra.configure(
            text=(f"Sats: {fmt_int(values.get('sats'))}   "
                  f"HDOP: {fmt(values.get('hdop'), decimals=2)}   "
                  f"Speed: {fmt(values.get('inav_speed'), ' m/s')}   "
                  f"Heading: {fmt(values.get('inav_heading'), '°', 0)}"))
        _position_trail.append((lat, lng))
        if len(_position_trail) > MAX_TRAIL_POINTS:
            del _position_trail[0]
        _update_map(lat, lng)


def _update_map(lat, lng):
    """Move the marker/path on the live OSM map widget to the latest fix."""
    global _map_marker, _map_path, _map_has_centered

    if _map_marker is None:
        _map_marker = map_widget.set_marker(lat, lng, text="Station")
    else:
        _map_marker.set_position(lat, lng)

    if len(_position_trail) >= 2:
        if _map_path is None:
            _map_path = map_widget.set_path(_position_trail)
        else:
            _map_path.set_position_list(_position_trail)

    if not _map_has_centered:
        map_widget.set_position(lat, lng)
        map_widget.set_zoom(16)
        _map_has_centered = True


# ============================================================
# CLEAR (LOCAL-ONLY RESET OF DISPLAYED VALUES)
# ============================================================
# Resets every value shown in the dashboard back to a blank state.
# This never talks to Blynk or the ESP32 — it only changes what is
# currently displayed in this running application. The next successful
# poll will immediately repopulate real values as usual.

NUM0 = "0.0"
TXT_NA = "NaN"

_NUMERIC_TILES = [
    v_gps_lat, v_gps_lng, v_gps_sats, v_gps_hdop,
    v_imu_roll, v_imu_pitch, v_imu_yaw, v_imu_temp,
    v_inav_speed, v_inav_heading, v_inav_alt,
    v_temp, v_hum, v_co, v_uvi, v_tvoc, v_eco2, v_aqi, v_dust,
    v_sound, v_heater, v_co_raw,
]

_TEXT_LABELS = [
    v_gps_pps, v_inav_state, v_co_phase,
    v_eng_msg, v_msg_v61, v_msg_v62, v_app_telem_rnd,
    v_reset_reason, v_rtc_checkpoint,
]


def clear_dashboard():
    """Reset all displayed values to 0.0 (numeric) / NaN (text) locally."""
    global _displayed_message_pair_key
    _pending_message_pair["msg_v61"] = None
    _pending_message_pair["msg_v62"] = None
    _displayed_message_pair_key = None
    for tile in _NUMERIC_TILES:
        tile.configure(text=NUM0, text_color=TXT_MAIN)

    for label in _TEXT_LABELS:
        label.configure(text=TXT_NA, text_color=TXT_MAIN)

    for bit, chip in flag_chip_labels:
        chip.configure(fg_color=TXT_DIM)

    for key, (label_widget, unit) in diag_value_labels.items():
        if key in ("i2c_device", "i2c_op"):
            label_widget.configure(text=TXT_NA)
        else:
            label_widget.configure(text=NUM0)

    v_map_source.configure(text=f"Source: {TXT_NA}")
    v_map_coords.configure(text=f"Lat: {TXT_NA}\nLng: {TXT_NA}")
    v_map_extra.configure(
        text=f"Sats: {NUM0}   HDOP: {NUM0}   Speed: {NUM0}   Heading: {NUM0}")

    label_wifi.configure(text=f"📶  WiFi: {TXT_NA}", text_color=TXT_SUB)
    label_last_update.configure(text=f"Last live data: {TXT_NA}")


# ============================================================
# STARTUP
# ============================================================

clear_dashboard()
threading.Thread(target=_poll_worker, daemon=True).start()
app.after(200, _poll_queue)
app.after(500, trigger_graph_refresh)


def _schedule_live_graph_redraw():
    _redraw_graph_from_cache()
    app.after(LIVE_GRAPH_DRAW_MS, _schedule_live_graph_redraw)


def _schedule_graph_refresh():
    trigger_graph_refresh()
    app.after(GRAPH_REFRESH_MS, _schedule_graph_refresh)


app.after(LIVE_GRAPH_DRAW_MS, _schedule_live_graph_redraw)
app.after(GRAPH_REFRESH_MS, _schedule_graph_refresh)


def _on_dashboard_close():
    # app.mainloop()'s finally block performs the ordered journal flush.
    app.destroy()


app.protocol("WM_DELETE_WINDOW", _on_dashboard_close)
try:
    app.mainloop()
finally:
    _event_monitor.close_all()
    _event_journal.close()
