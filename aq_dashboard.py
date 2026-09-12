# Outdoor Air Quality Station — ESP32 Dashboard for Windows PC
# v1.0 — Companion dashboard for AirQualityOutdoor_ESP32_Blynk.ino (v3.9.5)
#
# Connects to your local Blynk server and displays every virtual pin
# published by the ESP32 firmware:
#
#   - GPS / IMU / INAV navigation block
#   - Live environmental + air-quality sensor tiles
#   - 2x2 history graphs for TVOC / eCO2 / CO / Dust with a shared
#     selectable time range (1h / 3h / 24h / 48h / 168h)
#   - Full engineering/diagnostic pin readout (V19, V20, V34, V35-V51)
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
import gzip
import queue
import threading
import webbrowser
from datetime import datetime, timedelta

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
GRAPH_REFRESH_MS = 60000     # how often the 4 history graphs re-fetch
HTTP_TIMEOUT_S = 4

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
    "rssi": 21, "wifi_qual": 22,
}

MISC_PINS = {"eng_msg": 19, "status_flags": 20}

# V4 is used by the app/telemetry layer as a rolling diagnostic counter.
# It is displayed in the top-right diagnostic card and is always replaced
# with the newest value rather than accumulated as history.
APP_TELEM_RND_PIN = 4

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

RESET_PIN = 55

ALL_READ_PINS = {}
ALL_READ_PINS.update(GPS_PINS)
for k, v in IMU_PINS.items():
    ALL_READ_PINS[f"imu_{k}"] = v
for k, v in INAV_PINS.items():
    ALL_READ_PINS[f"inav_{k}"] = v
ALL_READ_PINS.update(SENSOR_PINS)
ALL_READ_PINS.update(MISC_PINS)
ALL_READ_PINS["app_telem_rnd"] = APP_TELEM_RND_PIN
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

_session = requests.Session()


def read_pin(pin):
    url = f"http://{BLYNK_SERVER}:{BLYNK_PORT}/{BLYNK_AUTH}/get/V{pin}"
    try:
        r = _session.get(url, timeout=HTTP_TIMEOUT_S)
        if r.status_code != 200:
            return None
        text = r.text.strip().replace('["', "").replace('"]', "")
        return text
    except Exception:
        return None


def write_pin(pin, value):
    url = f"http://{BLYNK_SERVER}:{BLYNK_PORT}/{BLYNK_AUTH}/update/V{pin}?value={value}"
    try:
        r = _session.get(url, timeout=HTTP_TIMEOUT_S)
        return r.status_code == 200
    except Exception:
        return False


def fetch_history(pin, hours):
    """Return list of (datetime, float) tuples for the last `hours` hours."""
    url = f"http://{BLYNK_SERVER}:{BLYNK_PORT}/{BLYNK_AUTH}/data/V{pin}"
    try:
        r = _session.get(url, timeout=30)
        if r.status_code != 200:
            return []
        try:
            text = gzip.decompress(r.content).decode("utf-8", errors="ignore")
        except Exception:
            text = r.text
    except Exception:
        return []

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
            ts = datetime.fromtimestamp(int(parts[1]) / 1000)
            if ts >= cutoff:
                points.append((ts, value))
        except Exception:
            continue
    points.sort(key=lambda p: p[0])
    return points


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

label_last_update = ctk.CTkLabel(status_wrap, text="Last update: —",
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

# V34 is deliberately a single replacement value.
# The widget is updated with configure(text=...) only, so a new reset reason
# replaces the previous one instead of creating another label/message below it.
v_reset_reason = ctk.CTkLabel(msg_card, text="",
                               font=FONT_SMALL, text_color=TXT_DIM, anchor="w",
                               wraplength=520, justify="left")
v_reset_reason.pack(anchor="w", padx=12, pady=(0, 6))

v_rtc_checkpoint = ctk.CTkLabel(msg_card, text="RTC checkpoint (V51): —",
                                 font=FONT_SMALL, text_color=TXT_DIM, anchor="w",
                                 wraplength=520, justify="left")
v_rtc_checkpoint.pack(anchor="w", padx=12, pady=(0, 10))

# Right-side controls are kept in their own vertical column so the diagnostic
# value stays at the top and the hard-reset control is clearly separated below.
diag_right = ctk.CTkFrame(diag_top, fg_color="transparent", width=250)
diag_right.pack(side="left", fill="y", padx=(6, 0))
diag_right.pack_propagate(False)

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

# Remote hard reset is intentionally lower on the right side.
reset_card = ctk.CTkFrame(diag_right, fg_color=CARD, corner_radius=14,
                          border_width=1, border_color=BORDER)
reset_card.pack(fill="x", pady=(5, 6), after=diag_rnd_card)

ctk.CTkLabel(reset_card, text="🔁  Remote Hard Reset (V55)", font=FONT_LABEL,
             text_color=TXT_SUB).pack(anchor="w", padx=12, pady=(10, 6))


def do_remote_reset():
    if not messagebox.askyesno("Confirm Reset",
                                "Send a hard-reset command to the ESP32?"):
        return

    def _worker():
        # Press software button (set pin HIGH)
        write_pin(RESET_PIN, 1)
        import time
        # Hold HIGH for 0.5 s to trigger reset
        time.sleep(0.5)
        # Release button (set pin LOW)
        write_pin(RESET_PIN, 0)

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


def trigger_graph_refresh():
    global _graph_busy
    with _graph_lock:
        if _graph_busy:
            return
        _graph_busy = True
    threading.Thread(target=_graph_worker, daemon=True).start()


def _graph_worker():
    global _graph_busy
    hours = int(history_var.get())
    app.after(0, lambda: label_graph_status.configure(text="Refreshing…"))
    results = []
    for name, pin in GRAPH_ORDER:
        results.append((name, fetch_history(pin, hours)))
    app.after(0, lambda: _apply_graph_data(results, hours))
    with _graph_lock:
        _graph_busy = False


def _apply_graph_data(results, hours):
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
    label_graph_status.configure(
        text=f"Updated {datetime.now().strftime('%H:%M:%S')}")


# ============================================================
# FAST POLL (background thread, queue -> main thread)
# ============================================================

_result_queue = queue.Queue()


def _poll_worker():
    while True:
        values = {}
        for name, pin in ALL_READ_PINS.items():
            values[name] = read_pin(pin)
        _result_queue.put(values)
        threading.Event().wait(FAST_POLL_MS / 1000.0)


def _poll_queue():
    try:
        while True:
            values = _result_queue.get_nowait()
            _apply_values(values)
    except queue.Empty:
        pass
    app.after(200, _poll_queue)


def _as_float(values, key):
    v = values.get(key)
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _apply_values(values):
    connected = values.get("temperature") is not None or values.get("co_ppm") is not None

    if connected:
        label_status.configure(text="⬤  CONNECTED", text_color=GREEN)
        label_last_update.configure(text=f"Last update: {datetime.now().strftime('%H:%M:%S')}")
    else:
        label_status.configure(text="⬤  OFFLINE", text_color=RED)

    rssi = _as_float(values, "rssi")
    if rssi is not None and rssi > UNAVAILABLE_THRESHOLD:
        quality = rssi_to_quality(int(rssi))
        label_wifi.configure(text=f"📶  {int(rssi)} dBm ({quality}%)",
                              text_color=wifi_color_for(quality))
    else:
        label_wifi.configure(text="📶  WiFi: —", text_color=TXT_DIM)

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

    # V34 is a single current reset-reason value. If the server ever returns
    # a multiline value, keep only the newest non-empty line so old messages
    # cannot visually stack in the dashboard.
    reset_reason = values.get("reset_reason")
    if reset_reason is None:
        # Keep V34 visually empty until the first actual value is read.
        reset_reason_text = ""
    else:
        reset_reason_lines = [line.strip() for line in str(reset_reason).splitlines()
                              if line.strip()]
        reset_reason_text = reset_reason_lines[-1] if reset_reason_lines else ""
    v_reset_reason.configure(
        text=f"Reset reason (V34): {reset_reason_text}" if reset_reason_text else ""
    )

    v_rtc_checkpoint.configure(
        text=f"RTC checkpoint (V51): {values.get('rtc_checkpoint') or '—'}"
    )

    # V4 diagnostic value is also replacement-only: each poll overwrites the
    # previous value in the same widget.
    v_app_telem_rnd.configure(text=values.get("app_telem_rnd") or "—")

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
# STARTUP
# ============================================================

threading.Thread(target=_poll_worker, daemon=True).start()
app.after(200, _poll_queue)
app.after(500, trigger_graph_refresh)


def _schedule_graph_refresh():
    trigger_graph_refresh()
    app.after(GRAPH_REFRESH_MS, _schedule_graph_refresh)


app.after(GRAPH_REFRESH_MS, _schedule_graph_refresh)

app.mainloop()
