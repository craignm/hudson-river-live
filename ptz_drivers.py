"""
PTZ Hardware & Virtual Control Drivers (`ptz_drivers.py`)
Pure Python 3.9+ standard library — zero external pip dependencies.

Provides a unified `BasePTZDriver` interface for:
  1. `VirtualPTZDriver`  — Smooth virtual pan/tilt/zoom state used by `/?obs=1` digital 4K zoom
                           and Mission Control telemetry when operating with a fixed 4K camera.
  2. `ViscaOverIPDriver` — Hybrid USB 3.0 UVC Endpoint 0 (`uvc-util --stdin-loop`) + UDP VISCA driver:
                           • Calibrated empirical FoMaKo 30x `(zoom-abs <-> HFOV)` optical lookup curve
                           • Full 3D SO(3) gimbal alignment (yaw, pitch, windowsill roll, scale, baseline parallax,
                             and lens zoom collimation offset)
                           • Calibrated zoom-focus tracking + distance curve (`focus-abs` in manual focus lock,
                             eliminating through-glass autofocus hunting day & night)
                           • Solar-aware outdoor daylight iris/shutter exposure vs low-light night AE
  3. `AxisVapixDriver`   — HTTP Digest-authenticated `/axis-cgi/com/ptz.cgi` driver for Axis PTZs.
"""

from collections import deque
import datetime as dt
import hashlib
import http.client
import math
import os
from pathlib import Path
import re
import select
import socket
import struct
import subprocess
import threading
import time
from typing import Dict, List, Optional, Tuple
import urllib.parse
import urllib.request


# Empirically verified on Sony SRG-A40 (`0x0000` = 1x 68.0° HFOV, `0x4000` = 20x optical,
# `0x5556` = 30x 4K Clear Image Zoom). Refined automatically during 30x auto-calibration.
DEFAULT_SONY_A40_ZOOM_HFOV_TABLE: List[Tuple[int, float]] = [
    (0, 68.00),      # 1.00x wide optical stop (0x0000)
    (5792, 34.20),   # 1.99x (0x16A0)
    (8912, 19.21),   # 3.54x (0x22D0)
    (10672, 14.47),  # 4.70x (0x29B0)
    (11840, 11.40),  # 5.96x (0x2E40)
    (13376, 9.32),   # 7.30x (0x3440)
    (14688, 7.20),   # 9.44x (0x3960)
    (15584, 5.95),   # 11.4x (0x3CE0)
    (16384, 4.01),   # 17.0x-20.0x optical tele stop (0x4000)
    (19114, 3.21),   # 25.0x 4K Clear Image Zoom (0x4AAA)
    (21846, 2.67),   # 30.0x 4K Clear Image Zoom stop (0x5556)
]


# Empirically measured on FoMaKo 4K 30x Optical Zoom PTZ (`UV602A4`, USB `0x33f1:0x10c4`)
# Maps hardware UVC/VISCA zoom register [0 .. 16384] -> horizontal field of view (degrees)
DEFAULT_FOMAKO_ZOOM_HFOV_TABLE: List[Tuple[int, float]] = [
    (0, 61.38),      # 1.00x wide optical stop
    (4096, 38.76),   # 1.58x
    (8192, 20.52),   # 2.99x
    (10240, 14.19),  # 4.33x (coarse calibration lock)
    (12288, 8.66),   # 7.09x
    (13824, 6.21),   # 9.88x (medium calibration lock)
    (15104, 3.76),   # 16.32x
    (16384, 2.22),   # ~27.7x-30x tele optical stop
]

# Empirically measured 18-stop varifocal rear-focus optical infinity curve on FoMaKo 30x (`UV602A4`):
# Rises smoothly from ~2000 at 1x to cam apex 2980 at 7.1x (`z=12294`), then plunges down the
# concave-down telephoto cam curve (-3.5 focus ticks/zoom tick near 30x) to 495 at `z=16384`.
DEFAULT_FOMAKO_FOCUS_ZOOM_TABLE: List[Tuple[int, int]] = [
    (0, 2000),
    (2996, 2100),
    (5599, 2270),
    (6738, 2400),
    (8208, 2530),
    (10208, 2780),
    (12294, 2980),
    (13854, 2860),
    (14422, 2690),
    (14886, 2450),
    (15128, 2310),
    (15340, 2140),
    (15536, 1970),
    (15712, 1780),
    (15881, 1580),
    (16038, 1330),
    (16185, 1060),
    (16324, 710),
    (16384, 495),
]


def solar_position_deg_utc(unix_ts: float, lat_deg: float, lon_deg: float) -> Tuple[float, float]:
    """NOAA solar `(elevation_deg, azimuth_deg)` in degrees (with atmospheric refraction) using stdlib math."""
    when_utc = dt.datetime.fromtimestamp(unix_ts, tz=dt.timezone.utc)
    jd = unix_ts / 86400.0 + 2440587.5
    jc = (jd - 2451545.0) / 36525.0

    geom_mean_lon = (280.46646 + jc * (36000.76983 + jc * 0.0003032)) % 360.0
    geom_mean_anom = 357.52911 + jc * (35999.05029 - 0.0001537 * jc)
    eccent = 0.016708634 - jc * (0.000042037 + 0.0000001267 * jc)

    anom_rad = math.radians(geom_mean_anom)
    sun_eq_ctr = (
        math.sin(anom_rad) * (1.914602 - jc * (0.004817 + 0.000014 * jc))
        + math.sin(2.0 * anom_rad) * (0.019993 - 0.000101 * jc)
        + math.sin(3.0 * anom_rad) * 0.000289
    )
    sun_true_lon = geom_mean_lon + sun_eq_ctr
    omega = 125.04 - 1934.136 * jc
    sun_app_lon = sun_true_lon - 0.00569 - 0.00478 * math.sin(math.radians(omega))

    mean_obliq_sec = 21.448 - jc * (46.815 + jc * (0.00059 - jc * 0.001813))
    mean_obliq = 23.0 + (26.0 + (mean_obliq_sec / 60.0)) / 60.0
    obliq_rad = math.radians(mean_obliq + 0.00256 * math.cos(math.radians(omega)))
    app_lon_rad = math.radians(sun_app_lon)

    sin_decl = math.sin(obliq_rad) * math.sin(app_lon_rad)
    decl_rad = math.asin(max(-1.0, min(1.0, sin_decl)))

    y = math.tan(obliq_rad / 2.0) ** 2
    lon_rad = math.radians(geom_mean_lon)
    eq_time_min = 4.0 * math.degrees(
        y * math.sin(2.0 * lon_rad)
        - 2.0 * eccent * math.sin(anom_rad)
        + 4.0 * eccent * y * math.sin(anom_rad) * math.cos(2.0 * lon_rad)
        - 0.5 * y * y * math.sin(4.0 * lon_rad)
        - 1.25 * eccent * eccent * math.sin(2.0 * anom_rad)
    )

    utc_minutes = (
        when_utc.hour * 60.0
        + when_utc.minute
        + when_utc.second / 60.0
        + when_utc.microsecond / 60_000_000.0
    )
    true_solar_time_min = (utc_minutes + eq_time_min + 4.0 * lon_deg) % 1440.0
    hour_angle_deg = true_solar_time_min / 4.0 - 180.0
    if hour_angle_deg < -180.0:
        hour_angle_deg += 360.0

    lat_rad = math.radians(lat_deg)
    ha_rad = math.radians(hour_angle_deg)
    cos_zenith = max(
        -1.0,
        min(
            1.0,
            math.sin(lat_rad) * math.sin(decl_rad)
            + math.cos(lat_rad) * math.cos(decl_rad) * math.cos(ha_rad),
        ),
    )
    sin_zenith = math.sqrt(max(1e-12, 1.0 - cos_zenith * cos_zenith))
    cos_az = max(
        -1.0,
        min(
            1.0,
            (math.sin(decl_rad) - math.sin(lat_rad) * cos_zenith)
            / max(1e-6, math.cos(lat_rad) * sin_zenith),
        ),
    )
    az_deg = (360.0 - math.degrees(math.acos(cos_az))) if ha_rad > 0.0 else math.degrees(math.acos(cos_az))

    elev_deg = 90.0 - math.degrees(math.acos(cos_zenith))
    if elev_deg <= 85.0:
        te = math.tan(math.radians(elev_deg))
        if elev_deg > 5.0:
            refrac = (58.1 / te - 0.07 / (te ** 3) + 0.000086 / (te ** 5)) / 3600.0
        elif elev_deg > -0.575:
            refrac = (
                1735.0 + elev_deg * (-518.2 + elev_deg * (103.4 + elev_deg * (-12.79 + elev_deg * 0.711)))
            ) / 3600.0
        else:
            refrac = (-20.772 / te) / 3600.0
        elev_deg += refrac
    return elev_deg, az_deg % 360.0


def solar_elevation_deg_utc(unix_ts: float, lat_deg: float, lon_deg: float) -> float:
    """NOAA solar elevation angle in degrees (with atmospheric refraction) using stdlib math."""
    return solar_position_deg_utc(unix_ts, lat_deg, lon_deg)[0]


def project_synoptic_to_ptz_gimbal(
    pan_syn_deg: float,
    tilt_syn_deg: float,
    yaw_offset_deg: float,
    pitch_offset_deg: float,
    roll_offset_deg: float = 0.0,
    pan_scale: float = 1.0,
    tilt_scale: float = 1.0,
    zoom_x: float = 30.0,
    collimation_pan_10x_deg: float = 0.0,
    collimation_tilt_10x_deg: float = 0.0,
    dist_km: Optional[float] = None,
    baseline_cam_m: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Tuple[float, float]:
    """
    Maps a target ray `(pan_syn_deg, tilt_syn_deg)` in the Synoptic camera frame (+X right, +Y up, +Z fwd)
    through baseline parallax and 3D rotation `R_y(yaw) * R_x(pitch) * R_z(roll)` into exact FoMaKo
    hardware gimbal motor angles `(pan_hw_deg, tilt_hw_deg)`.
    """
    ps = math.radians(pan_syn_deg)
    ts = math.radians(tilt_syn_deg)
    vx = math.sin(ps) * math.cos(ts)
    vy = math.sin(ts)
    vz = math.cos(ps) * math.cos(ts)

    bx, by, bz = baseline_cam_m
    if dist_km is not None and dist_km > 0.05 and (abs(bx) > 1e-6 or abs(by) > 1e-6 or abs(bz) > 1e-6):
        d_m = float(dist_km) * 1000.0
        px = d_m * vx - bx
        py = d_m * vy - by
        pz = d_m * vz - bz
        norm = math.sqrt(px * px + py * py + pz * pz)
        if norm > 1.0:
            vx, vy, vz = px / norm, py / norm, pz / norm

    # 1) Base roll mismatch around forward Z axis (captures windowsill slant across wide pan excursions)
    r_rad = math.radians(roll_offset_deg)
    cr, sr = math.cos(r_rad), math.sin(r_rad)
    x1 = vx * cr - vy * sr
    y1 = vx * sr + vy * cr
    z1 = vz

    # 2) Pitch/tilt offset around horizontal X axis
    p_rad = math.radians(pitch_offset_deg)
    cp, sp = math.cos(p_rad), math.sin(p_rad)
    x2 = x1
    y2 = y1 * cp + z1 * sp
    z2 = -y1 * sp + z1 * cp

    # 3) Yaw/pan offset around vertical Y axis
    y_rad = math.radians(yaw_offset_deg)
    cy, sy = math.cos(y_rad), math.sin(y_rad)
    x3 = x2 * cy + z2 * sy
    y3 = y2
    z3 = -x2 * sy + z2 * cy

    pan_geom = math.degrees(math.atan2(x3, z3))
    tilt_geom = math.degrees(math.atan2(y3, math.sqrt(max(1e-12, x3 * x3 + z3 * z3))))

    pan_hw = yaw_offset_deg + pan_scale * (pan_geom - yaw_offset_deg)
    tilt_hw = pitch_offset_deg + tilt_scale * (tilt_geom - pitch_offset_deg)

    # Smooth zoom-dependent optical center collimation (0 at 30x where 3D bore-sight is fit, 1.0 at 10x)
    if abs(collimation_pan_10x_deg) > 1e-4 or abs(collimation_tilt_10x_deg) > 1e-4:
        z_eff = max(1.0, min(30.0, float(zoom_x)))
        w_coll = max(0.0, min(2.2, (1.0 / z_eff - 1.0 / 30.0) / (1.0 / 10.0 - 1.0 / 30.0)))
        pan_hw += w_coll * collimation_pan_10x_deg
        tilt_hw += w_coll * collimation_tilt_10x_deg

    return pan_hw, tilt_hw


class BasePTZDriver:
    def move_absolute(self, pan_deg: float, tilt_deg: float, zoom_x: float):
        raise NotImplementedError

    def move_velocity(self, pan_deg_per_s: float, tilt_deg_per_s: float, zoom_x: float):
        raise NotImplementedError

    def lock_focus_infinity(self):
        pass

    def get_state(self) -> dict:
        return {}


class VirtualPTZDriver(BasePTZDriver):
    """Maintains smoothed pan/tilt/zoom state for Virtual PTZ & UI telemetry."""

    def __init__(self):
        self.pan_deg = 0.0      # Relative to synoptic center (+right, -left)
        self.tilt_deg = 0.0     # Relative to synoptic center (+up, -down)
        self.zoom_x = 1.0       # 1.0x .. 40.0x
        self.pan_rate = 0.0
        self.tilt_rate = 0.0
        self.mode = "virtual"
        self.last_update = time.time()

    def move_absolute(self, pan_deg: float, tilt_deg: float, zoom_x: float):
        self.pan_deg = max(-170.0, min(170.0, pan_deg))
        self.tilt_deg = max(-30.0, min(90.0, tilt_deg))
        self.zoom_x = max(1.0, min(40.0, zoom_x))
        self.last_update = time.time()

    def move_velocity(self, pan_deg_per_s: float, tilt_deg_per_s: float, zoom_x: float):
        now = time.time()
        dt_s = min(0.25, max(0.0, now - self.last_update))
        self.pan_rate = pan_deg_per_s
        self.tilt_rate = tilt_deg_per_s
        self.pan_deg = max(-170.0, min(170.0, self.pan_deg + pan_deg_per_s * dt_s))
        self.tilt_deg = max(-30.0, min(90.0, self.tilt_deg + tilt_deg_per_s * dt_s))
        self.zoom_x = max(1.0, min(40.0, zoom_x))
        self.last_update = now

    def get_state(self) -> dict:
        return {
            "driver": "virtual",
            "pan_deg": round(self.pan_deg, 3),
            "tilt_deg": round(self.tilt_deg, 3),
            "zoom_x": round(self.zoom_x, 2),
            "pan_rate_dps": round(self.pan_rate, 3),
            "tilt_rate_dps": round(self.tilt_rate, 3),
        }


# Empirically measured FoMaKo UV602A4 UVC `iris-abs` [0..12] optical gain in EV stops relative to
# sweet-spot `iris-abs=7` (~f/5.6). Note: FoMaKo's UVC `iris-abs` is inverted (`0` = wide-open f/1.8,
# `7` = crisp f/5.6 sweet spot, `9..10` = stopped-down f/8..f/11).
FOMAKO_IRIS_GAIN_STOPS = {
    12: -2.10,
    11: -1.60,
    10: -1.15,
    9: -0.65,
    8: -0.32,
    7: 0.00,
    6: 0.30,
    5: 0.58,
    4: 0.85,
    3: 1.05,
    2: 1.20,
    1: 1.30,
    0: 1.38,
}


class ViscaOverIPDriver(BasePTZDriver):
    """
    Hybrid USB-UVC + VISCA-over-IP PTZ driver supporting:
      - Direct USB 3.0 UVC Endpoint 0 hardware control via persistent `uvc-util -V 0x33f1 --stdin-loop`
        when the FoMaKo 4K 30x (`0x33f1:0x10c4`) is attached locally (zero Ethernet cable needed,
        zero DHCP drift, zero macOS Sequoia LaunchAgent Local Network privacy issues)
      - Automatic fallback to VISCA-over-IP UDP (`raw_udp=True` port 5678/1259 for FoMaKo/PTZOptics,
        or `raw_udp=False` port 52381 w/ 8-byte header for Sony SRG/BRC) when running over LAN
    """

    def __init__(
        self,
        host: str = "",
        port: int = 5678,
        raw_udp: bool = True,
        steps_per_deg: float = 14.4,
        pan_offset_deg: float = 0.0,
        tilt_offset_deg: float = 0.0,
        roll_offset_deg: float = 0.0,
        pan_scale: float = 1.0,
        tilt_scale: float = 1.0,
        max_zoom_x: float = 30.0,
        usb_vendor: str = "0x33f1",
        calibration: Optional[dict] = None,
    ):
        self.host = host or "127.0.0.1"
        self.camera_model = "fomako"
        self.port = port
        self.raw_udp = raw_udp
        self.steps_per_deg = steps_per_deg
        self.pan_offset_deg = pan_offset_deg
        self.tilt_offset_deg = tilt_offset_deg
        self.roll_offset_deg = roll_offset_deg
        self.pan_scale = pan_scale
        self.tilt_scale = tilt_scale
        self.collimation_pan_10x_deg = 0.0
        self.collimation_tilt_10x_deg = 0.0
        self.baseline_cam_m = (0.25, 0.0, 0.0)
        self.lat_deg = 40.744823
        self.lon_deg = -74.008063
        self.syn_az_deg = 207.186
        self.syn_tilt_deg = 3.171
        self.syn_roll_deg = -0.945
        self.zoom_hfov_table: List[Tuple[int, float]] = list(DEFAULT_FOMAKO_ZOOM_HFOV_TABLE)
        self.focus_zoom_table: List[Tuple[int, int]] = list(DEFAULT_FOMAKO_FOCUS_ZOOM_TABLE)
        self.focus_inv_km_slope: float = 21.84
        self.focus_trim_offset: int = 0
        self.has_calibrated_focus: bool = True
        self.day_iris_abs: int = 7
        self.day_shutter_abs: int = 4
        self.day_manual_glare: bool = True
        self.ambient_ev_bias: float = 0.0
        self.local_ev_trim: float = 0.0
        self.last_ev_applied: Optional[float] = None
        self.last_exp_iris: Optional[int] = None
        self.last_exp_shutter: Optional[int] = None
        self.last_exp_gain: Optional[int] = None
        self.last_view_pan_syn: float = 0.0
        self.last_view_tilt_syn: float = -2.0
        self.last_world_el_deg: float = 1.0
        self.exposure_profile: str = "init"

        self.max_zoom_x = max(2.0, float(max_zoom_x))
        self.usb_vendor = usb_vendor
        self.seq = 1
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.last_pan_steps: Optional[int] = None
        self.last_tilt_steps: Optional[int] = None
        self.last_pan_cmd_deg: float = 0.0
        self.last_tilt_cmd_deg: float = 0.0
        self.actual_pan_deg: Optional[float] = None
        self.actual_tilt_deg: Optional[float] = None
        self.vel_step_x: int = 0
        self.vel_step_y: int = 0
        self.last_vel_check_ts: float = 0.0
        self.slew_until_ts: float = 0.0
        self.smooth_hw_pan_dps: float = 0.0
        self.smooth_hw_tilt_dps: float = 0.0
        self.last_vel_ema_ts: float = 0.0
        self.last_target_hw_pan: Optional[float] = None
        self.last_target_hw_tilt: Optional[float] = None
        self.last_zoom_pos: Optional[int] = None
        self.actual_zoom_pos: Optional[int] = None
        self.prev_polled_zoom_pos: Optional[int] = None
        self.pending_focus_sync: bool = True
        self.last_dist_km: Optional[float] = None
        self.last_zoom_x: float = 1.0
        self.last_focus_abs: Optional[int] = None
        self.last_pt_send_ts = 0.0
        self.last_large_slew_ts = 0.0
        self.last_zoom_send_ts = 0.0
        self.last_zoom_target_change_ts = 0.0
        self.last_focus_send_ts = 0.0
        self.last_exposure_sync_ts = 0.0
        self.last_any_send_ts = 0.0
        self.sent_packets = 0
        self.last_error = ""
        self.focus_mode = "auto_low"
        self.usb_proc = None
        self.usb_ready = False
        self.last_usb_spawn_ts = 0.0
        self._one_push_timer = None

        if calibration:
            self.apply_calibration_dict(calibration)
        if self.raw_udp and self.usb_vendor:
            with self.lock:
                self._ensure_usb_pipe()

    def apply_calibration_dict(self, cal: dict):
        """Hot-reloads 3D SO(3) gimbal alignment, empirical zoom/focus curves, and daylight exposure."""
        self.lat_deg = float(cal.get("lat", self.lat_deg))
        self.lon_deg = float(cal.get("lon", self.lon_deg))
        self.syn_az_deg = float(cal.get("az_deg", self.syn_az_deg))
        self.syn_tilt_deg = float(cal.get("tilt_deg", self.syn_tilt_deg))
        self.syn_roll_deg = float(cal.get("roll_deg", self.syn_roll_deg))
        self.pan_offset_deg = float(cal.get("ptz_pan_offset_deg", self.pan_offset_deg))
        self.tilt_offset_deg = float(cal.get("ptz_tilt_offset_deg", self.tilt_offset_deg))
        self.roll_offset_deg = float(cal.get("ptz_roll_offset_deg", 0.0))
        self.pan_scale = float(cal.get("ptz_pan_scale", 1.0))
        self.tilt_scale = float(cal.get("ptz_tilt_scale", 1.0))
        self.collimation_pan_10x_deg = float(cal.get("ptz_collimation_pan_10x_deg", 0.0))
        self.collimation_tilt_10x_deg = float(cal.get("ptz_collimation_tilt_10x_deg", 0.0))
        self.steps_per_deg = float(cal.get("ptz_steps_per_deg", self.steps_per_deg))
        self.max_zoom_x = max(2.0, float(cal.get("ptz_max_zoom_x", self.max_zoom_x)))

        # Convert ENU windowsill baseline into Synoptic camera frame (+X right, +Y up, +Z fwd)
        az = math.radians(self.syn_az_deg)
        el = math.radians(self.syn_tilt_deg)
        be = float(cal.get("ptz_offset_east_m", 0.25))
        bn = float(cal.get("ptz_offset_north_m", 0.0))
        bu = float(cal.get("ptz_offset_up_m", 0.0))
        bx = be * math.cos(az) - bn * math.sin(az)
        by = -be * math.sin(az) * math.sin(el) - bn * math.cos(az) * math.sin(el) + bu * math.cos(el)
        bz = be * math.sin(az) * math.cos(el) + bn * math.cos(az) * math.cos(el) + bu * math.sin(el)
        self.baseline_cam_m = (bx, by, bz)

        z_tbl = cal.get("ptz_zoom_hfov_table")
        if isinstance(z_tbl, list) and len(z_tbl) >= 3:
            parsed_z = sorted((int(p[0]), float(p[1])) for p in z_tbl)
            self.zoom_hfov_table = parsed_z

        f_model = cal.get("ptz_focus_model")
        if isinstance(f_model, dict):
            f_tbl = f_model.get("zoom_focus_table")
            if isinstance(f_tbl, list) and len(f_tbl) >= 2:
                self.focus_zoom_table = sorted((int(p[0]), int(p[1])) for p in f_tbl)
                self.focus_inv_km_slope = float(f_model.get("inv_km_slope_30x", 0.0))
                self.has_calibrated_focus = bool(f_model.get("enabled", True))

        exp_cfg = cal.get("ptz_day_exposure")
        if isinstance(exp_cfg, dict):
            if "ambient_ev_bias" in exp_cfg and exp_cfg["ambient_ev_bias"] is not None:
                self.ambient_ev_bias = max(-3.5, min(4.5, float(exp_cfg["ambient_ev_bias"])))
            self.day_iris_abs = max(0, min(12, int(exp_cfg.get("iris_abs", self.day_iris_abs))))
            self.day_shutter_abs = max(1, min(10000, int(exp_cfg.get("shutter_abs", self.day_shutter_abs))))

        # Force next tracking tick to re-apply updated bore-sight & focus
        self.last_pan_steps = None
        self.last_focus_abs = None
        self.pending_focus_sync = True
        self.focus_trim_offset = 0

    def _find_uvc_bin(self) -> Optional[Path]:
        here = Path(__file__).resolve().parent / "uvc-util"
        if here.is_file():
            return here
        alt = Path.home() / "Library/Application Support/HudsonRiverUVC/uvc-util"
        if alt.is_file():
            return alt
        return None

    def _ensure_usb_pipe(self) -> bool:
        if self.usb_ready and self.usb_proc and self.usb_proc.poll() is None:
            return True
        now = time.time()
        if (now - self.last_usb_spawn_ts) < 3.0:
            return False
        self.last_usb_spawn_ts = now
        self._close_usb_pipe()
        uvc_bin = self._find_uvc_bin()
        if not uvc_bin or not self.usb_vendor:
            return False
        try:
            proc = subprocess.Popen(
                [str(uvc_bin), "-V", self.usb_vendor, "--stdin-loop"],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
            )
            rlist, _, _ = select.select([proc.stdout], [], [], 1.0)
            if rlist:
                banner = proc.stdout.readline().strip()
                if banner.startswith("READY "):
                    self.usb_proc = proc
                    self.usb_ready = True
                    self.last_error = ""
                    return True
            proc.terminate()
            try:
                proc.wait(timeout=0.3)
            except Exception:
                proc.kill()
        except Exception:
            pass
        return False

    def _close_usb_pipe(self):
        proc = self.usb_proc
        self.usb_proc = None
        self.usb_ready = False
        if proc is not None:
            try:
                proc.terminate()
                proc.wait(timeout=0.3)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass

    def _send_usb_uvc(self, expr: str) -> bool:
        with self.lock:
            if not self._ensure_usb_pipe():
                return False
            # FoMaKo ARM ISP I2C bus needs ~45ms spacing on exposure/iris/gain/focus writes so
            # back-to-back Endpoint 0 packets never get overwritten by mode-switch register reloads.
            min_gap = 0.015 if expr.startswith("pan-tilt-") else 0.050
            wait_s = min_gap - (time.time() - self.last_any_send_ts)
            if wait_s > 0.0:
                time.sleep(wait_s)
            try:
                self.usb_proc.stdin.write(expr + "\n")
                self.usb_proc.stdin.flush()
                rlist, _, _ = select.select([self.usb_proc.stdout], [], [], 0.8)
                if not rlist:
                    self._close_usb_pipe()
                    return False
                reply = self.usb_proc.stdout.readline().strip()
                self.last_any_send_ts = time.time()
                if reply.startswith("OK "):
                    self.sent_packets += 1
                    self.last_error = ""
                    return True
                self.last_error = f"UVC {reply}"
                return False
            except Exception as exc:
                self.last_error = f"UVC {type(exc).__name__}: {exc}"
                self._close_usb_pipe()
                return False

    def query_usb_uvc(self, ctrl_name: str) -> Optional[str]:
        """Queries current value of a UVC control over the persistent stdin-loop pipe."""
        with self.lock:
            if not self._ensure_usb_pipe():
                return None
            try:
                self.usb_proc.stdin.write(f"?{ctrl_name}\n")
                self.usb_proc.stdin.flush()
                rlist, _, _ = select.select([self.usb_proc.stdout], [], [], 0.7)
                if not rlist:
                    self._close_usb_pipe()
                    return None
                reply = self.usb_proc.stdout.readline().strip()
                if reply.startswith("OK ") and "=" in reply:
                    return reply.split("=", 1)[1].strip()
                return None
            except Exception:
                self._close_usb_pipe()
                return None

    def _send_visca(self, payload: bytes):
        with self.lock:
            wait_s = 0.040 - (time.time() - self.last_any_send_ts)
            if wait_s > 0.0:
                time.sleep(wait_s)
            if self.raw_udp:
                packet = payload
            else:
                header = struct.pack(">HH", 0x0100, len(payload)) + struct.pack(">I", self.seq)
                self.seq = (self.seq + 1) & 0xFFFFFFFF
                packet = header + payload
            try:
                while True:
                    try:
                        self.sock.recvfrom(64)
                    except OSError:
                        break
                self.sock.sendto(packet, (self.host, self.port))
                self.last_any_send_ts = time.time()
                self.sent_packets += 1
                self.last_error = ""
            except OSError as exc:
                self.last_error = f"{type(exc).__name__}: {exc}"

    @staticmethod
    def _encode_nibble_16(val: int) -> bytes:
        val = val & 0xFFFF
        return bytes([
            (val >> 12) & 0x0F,
            (val >> 8) & 0x0F,
            (val >> 4) & 0x0F,
            val & 0x0F,
        ])

    def register_to_hfov_deg(self, z_pos: int) -> float:
        """Piecewise-logarithmic interpolation from hardware `zoom-abs` [0..16384] -> HFOV (deg)."""
        tbl = self.zoom_hfov_table
        if z_pos <= tbl[0][0]:
            return tbl[0][1]
        if z_pos >= tbl[-1][0]:
            return tbl[-1][1]
        for i in range(len(tbl) - 1):
            z0, h0 = tbl[i]
            z1, h1 = tbl[i + 1]
            if z0 <= z_pos <= z1:
                t = (z_pos - z0) / max(1.0, float(z1 - z0))
                return math.exp(math.log(h0) * (1.0 - t) + math.log(h1) * t)
        return tbl[-1][1]

    def zoom_x_to_register(self, zoom_x: float) -> int:
        """Maps requested optical magnification `zoom_x` (relative to 1x wide stop) -> exact `zoom-abs`."""
        tbl = self.zoom_hfov_table
        wide_hfov = tbl[0][1]
        min_hfov = tbl[-1][1]
        clamped_z = max(1.0, min(self.max_zoom_x, float(zoom_x)))
        desired_hfov = max(min_hfov, min(wide_hfov, wide_hfov / clamped_z))
        if desired_hfov >= wide_hfov:
            return tbl[0][0]
        if desired_hfov <= min_hfov:
            return tbl[-1][0]
        log_d = math.log(desired_hfov)
        for i in range(len(tbl) - 1):
            z0, h0 = tbl[i]
            z1, h1 = tbl[i + 1]
            if h1 <= desired_hfov <= h0:
                lh0, lh1 = math.log(h0), math.log(h1)
                frac = (lh0 - log_d) / max(1e-6, lh0 - lh1)
                return max(0, min(16384, int(round(z0 + frac * (z1 - z0)))))
        return tbl[-1][0]

    def compute_calibrated_focus_abs(
        self, z_pos: int, dist_km: Optional[float] = None, include_trim: bool = True
    ) -> int:
        """
        Computes exact manual `focus-abs` [0..3371] for physical zoom register `z_pos` and 3D slant
        distance `dist_km`, combining the 18-stop empirical rear-focus varifocal cam curve with
        rear-focus near-distance displacement and a strictly bounded [-55..+55] thermal micro-trim.
        """
        ftbl = self.focus_zoom_table
        if z_pos <= ftbl[0][0]:
            f_val = float(ftbl[0][1])
        elif z_pos >= ftbl[-1][0]:
            f_val = float(ftbl[-1][1])
        else:
            f_val = float(ftbl[-1][1])
            for i in range(len(ftbl) - 1):
                z0, f0 = ftbl[i]
                z1, f1 = ftbl[i + 1]
                if z0 <= z_pos <= z1:
                    t = (z_pos - z0) / max(1.0, float(z1 - z0))
                    f_val = f0 + t * (f1 - f0)
                    break

        if dist_km is not None and dist_km > 0.08 and abs(self.focus_inv_km_slope) > 1e-3:
            hfov_cur = self.register_to_hfov_deg(z_pos)
            wide_hfov = self.zoom_hfov_table[0][1]
            zoom_eff = wide_hfov / max(0.5, hfov_cur)
            # In a rear-focus varifocal zoom, rear focus group sensitivity (ticks per diopter)
            # stays nearly constant from ~23x..30x and tapers smoothly toward wide hyperfocal zooms.
            w_zoom = min(1.0, max(0.0, (zoom_eff - 3.0) / 20.0))
            inv_d_delta = (1.0 / max(0.20, float(dist_km))) - (1.0 / 4.0)
            f_val += self.focus_inv_km_slope * inv_d_delta * w_zoom

        if include_trim:
            f_val += max(-55, min(55, int(self.focus_trim_offset)))

        return max(0, min(3371, int(round(f_val))))

    def set_focus_trim_offset(self, offset_ticks: int):
        """Strictly constrains any closed-loop contrast trim within +-55 ticks of the 3D distance model."""
        self.focus_trim_offset = max(-55, min(55, int(round(offset_ticks))))
        self.pending_focus_sync = True

    def set_focus_abs(self, focus_abs: int, force: bool = False) -> bool:
        f_clamped = max(0, min(3371, int(round(focus_abs))))
        now = time.time()
        if (
            not force
            and self.last_focus_abs is not None
            and abs(f_clamped - self.last_focus_abs) < 12
            and (now - self.last_focus_send_ts) < 4.0
        ):
            return True
        if self._focus_mode_needs_manual_switch():
            self._send_usb_uvc("auto-focus=0")
            self.focus_mode = "calibrated_curve"
        ok = self._send_usb_uvc(f"focus-abs={f_clamped}")
        if ok:
            self.last_focus_abs = f_clamped
            self.last_focus_send_ts = now
        return ok

    def _focus_mode_needs_manual_switch(self) -> bool:
        return self.focus_mode not in ("calibrated_curve", "manual_lock")

    def compute_world_ray_deg(self, pan_syn_deg: float, tilt_syn_deg: float) -> Tuple[float, float]:
        """Projects Synoptic ray `(pan_syn_deg, tilt_syn_deg)` into ENU `(world_az_deg, world_el_deg)`."""
        az = math.radians(self.syn_az_deg)
        el = math.radians(self.syn_tilt_deg)
        roll = math.radians(self.syn_roll_deg)
        cr, sr = math.cos(roll), math.sin(roll)

        # Synoptic camera basis vectors in ENU (+E, +N, +U)
        fe = math.sin(az) * math.cos(el)
        fn = math.cos(az) * math.cos(el)
        fu = math.sin(el)

        re0 = math.cos(az)
        rn0 = -math.sin(az)
        ru0 = 0.0

        ue0 = -math.sin(az) * math.sin(el)
        un0 = -math.cos(az) * math.sin(el)
        uu0 = math.cos(el)

        re = cr * re0 + sr * ue0
        rn = cr * rn0 + sr * un0
        ru = cr * ru0 + sr * uu0

        ue = -sr * re0 + cr * ue0
        un = -sr * rn0 + cr * un0
        uu = -sr * ru0 + cr * uu0

        pr = math.radians(pan_syn_deg)
        tr = math.radians(tilt_syn_deg)
        xc = math.sin(pr) * math.cos(tr)
        yc = math.sin(tr)
        zc = math.cos(pr) * math.cos(tr)

        de = xc * re + yc * ue + zc * fe
        dn = xc * rn + yc * un + zc * fn
        du = max(-1.0, min(1.0, xc * ru + yc * uu + zc * fu))
        world_az = math.degrees(math.atan2(de, dn)) % 360.0
        world_el = math.degrees(math.asin(du))
        return world_az, world_el

    def compute_outdoor_ev_index(
        self,
        pan_syn_deg: float,
        tilt_syn_deg: float,
        zoom_x: float,
        now: Optional[float] = None,
    ) -> Optional[float]:
        """
        3D Solar + Horizon/Water Elevation + FOV Sky-Fraction Feed-Forward Exposure Model.
        Returns `None` at night (`sun_elev < -3.5 deg`) so FoMaKo switches cleanly to `AE=8`.
        In daylight/twilight, returns continuous exposure index `ev_idx` where +1.0 = 2x integration time:
          - Pure sky / horizon (`world_el >= +2.25 deg` or wide FOV containing bright sky): stopped down
            around `iris-abs=7..9` (`f/5.6..f/8`) + `exposure-time-abs=1..2` -> zero sky/horizon clipping!
          - Skyline rooftops (`+0.45..+2.25 deg`): `iris-abs=7` + `exposure-time-abs=3..5`
          - Dark Hudson River water (`world_el < +0.45 deg`, no sky in FOV): opens up `+1.5..+4.2` stops
            (`iris-abs=4..5`, `exposure-time-abs=12..20`) so hulls & wakes stay crisp and bright.
        """
        ts = now if now is not None else time.time()
        sun_elev, sun_az = solar_position_deg_utc(ts, self.lat_deg, self.lon_deg)
        if sun_elev < -3.5:
            return None

        if sun_elev >= 25.0:
            sky_base_ev = 1.20
        elif sun_elev >= 10.0:
            sky_base_ev = 2.35 - 1.15 * ((sun_elev - 10.0) / 15.0)
        elif sun_elev >= 0.0:
            sky_base_ev = 5.05 - 2.70 * (sun_elev / 10.0)
        else:
            sky_base_ev = 8.35 - 3.30 * ((sun_elev + 3.5) / 3.5)

        world_az, world_el = self.compute_world_ray_deg(pan_syn_deg, tilt_syn_deg)
        hfov = self.zoom_hfov_table[0][1] / max(1.0, float(zoom_x))
        vfov = max(1.20, hfov * 0.5625)
        top_el = world_el + 0.45 * vfov

        if world_el >= 2.25:
            raw_elev_ev = 0.0
        elif world_el >= 0.45:
            raw_elev_ev = 1.45 * ((2.25 - world_el) / 1.80)
        elif world_el >= -7.55:
            raw_elev_ev = 1.45 + 2.70 * ((0.45 - world_el) / 8.00)
        else:
            raw_elev_ev = min(4.45, 4.15 + 0.30 * ((-7.55 - world_el) / 8.50))

        # Protect upper-frame sky whenever the top edge of the current zoom FOV rises above rooftops!
        if top_el > 1.25:
            sky_frac = min(1.0, max(0.0, (top_el - 1.25) / max(1.40, vfov)))
            safe_cap = 1.45 * (1.0 - sky_frac) + 0.25 * (1.0 - sky_frac)
            elev_ev = min(raw_elev_ev, safe_cap)
        else:
            elev_ev = raw_elev_ev

        glare_ev = 0.0
        if sun_elev > 0.5:
            az1, el1 = math.radians(world_az), math.radians(max(0.0, world_el))
            az2, el2 = math.radians(sun_az), math.radians(sun_elev)
            cos_sep = max(
                -1.0,
                min(1.0, math.sin(el1) * math.sin(el2) + math.cos(el1) * math.cos(el2) * math.cos(az1 - az2)),
            )
            sep_deg = math.degrees(math.acos(cos_sep))
            if sep_deg < 60.0:
                sky_glare = -0.85 * (((60.0 - sep_deg) / 60.0) ** 1.35)
                if world_el >= -0.2:
                    glare_ev = sky_glare
                else:
                    # On water: within +-28 deg of solar azimuth, afternoon sun creates an intense
                    # specular glitter corridor on Hudson waves (~1.5 stops brighter than off-axis water)
                    w_sky = max(0.0, 1.0 - (-0.2 - world_el) / 2.5)
                    az_diff = abs((world_az - sun_az + 180.0) % 360.0 - 180.0)
                    glitter = (
                        -1.55 * ((1.0 - az_diff / 28.0) ** 1.3)
                        if (az_diff < 28.0 and sun_elev > 3.0)
                        else 0.0
                    )
                    glare_ev = w_sky * sky_glare + (1.0 - w_sky) * glitter

        total_ev = sky_base_ev + elev_ev + glare_ev + self.ambient_ev_bias + self.local_ev_trim
        return max(-0.9, min(11.5, total_ev))

    @staticmethod
    def ev_to_uvc_registers(ev_idx: float) -> Tuple[int, int, int]:
        """
        Maps continuous `ev_idx` -> FoMaKo `(iris-abs, exposure-time-abs, gain)` along the optical
        sweet-spot program curve (`iris=7` f/5.6 for sky/skyline, opening smoothly `7->4->0` for dark
        water & twilight, stopping down `8..10` only for direct solar sky glare).
        """
        if ev_idx < 0.20:
            iris = min(10, max(8, int(round(7.5 - ev_idx * 2.2))))
        elif ev_idx <= 2.45:
            iris = 7
        elif ev_idx <= 4.35:
            iris = int(round(7.0 - 3.0 * ((ev_idx - 2.45) / 1.90)))
        elif ev_idx <= 6.95:
            iris = int(round(4.0 - 4.0 * ((ev_idx - 4.35) / 2.60)))
        else:
            iris = 0
        iris = max(0, min(10, iris))

        iris_gain = FOMAKO_IRIS_GAIN_STOPS.get(iris, 0.0)
        shutter_ev = (ev_idx - 0.10) - iris_gain
        if shutter_ev <= 8.41:
            shutter = max(1, min(340, int(round(2.0 ** shutter_ev))))
            gain = 0
        else:
            shutter = 340
            gain = max(0, min(10, int(round((shutter_ev - 8.41) * 3.0))))
        return iris, shutter, gain

    def sync_outdoor_exposure(
        self,
        force: bool = False,
        pan_syn_deg: Optional[float] = None,
        tilt_syn_deg: Optional[float] = None,
        zoom_x: Optional[float] = None,
        allow_before_zoom: bool = False,
    ):
        """
        Applies 3D solar/horizon feed-forward + highlight-protected closed-loop exposure over USB UVC.
        Strict motor interlock: never writes UVC exposure registers while `zoom-abs` is in flight!
        """
        if not self._has_usb_uvc():
            return
        if not force and not allow_before_zoom and self.pending_focus_sync and self.last_zoom_pos is not None:
            return

        now = time.time()
        p_syn = self.last_view_pan_syn if pan_syn_deg is None else float(pan_syn_deg)
        t_syn = self.last_view_tilt_syn if tilt_syn_deg is None else float(tilt_syn_deg)
        z_x = self.last_zoom_x if zoom_x is None else float(zoom_x)

        _, world_el = self.compute_world_ray_deg(p_syn, t_syn)
        hfov = self.zoom_hfov_table[0][1] / max(1.0, z_x)
        top_el = world_el + 0.45 * max(1.20, hfov * 0.5625)

        # When tilting upward from dark water into horizon/sky, immediately shed positive water trim
        # so the sky NEVER flashes white on upward slews!
        if top_el > 0.8 and world_el > self.last_world_el_deg + 1.2 and self.local_ev_trim > 0.0:
            self.local_ev_trim = 0.0
        elif math.hypot(p_syn - self.last_view_pan_syn, t_syn - self.last_view_tilt_syn) > 6.0:
            self.local_ev_trim *= 0.25

        self.last_view_pan_syn = p_syn
        self.last_view_tilt_syn = t_syn
        self.last_world_el_deg = world_el

        ev_idx = self.compute_outdoor_ev_index(p_syn, t_syn, z_x, now=now)
        if ev_idx is None:
            if force or self.exposure_profile != "night_ae8":
                self._send_usb_uvc("power-line-frequency=2")
                self._send_usb_uvc("backlight-compensation=0")
                self._send_usb_uvc("auto-exposure-mode=8")
                self.exposure_profile = "night_ae8"
                self.last_ev_applied = None
                self.last_exp_iris = None
                self.last_exp_shutter = None
                self.last_exp_gain = None
                self.last_exposure_sync_ts = time.time()
            return

        if (
            not force
            and self.last_ev_applied is not None
            and self.exposure_profile.startswith("ev")
            and abs(ev_idx - self.last_ev_applied) < 0.20
            and (now - self.last_exposure_sync_ts) < 45.0
        ):
            return

        iris, shutter, gain = self.ev_to_uvc_registers(ev_idx)
        entering_manual = force or not self.exposure_profile.startswith("ev")
        if entering_manual:
            self._send_usb_uvc("power-line-frequency=0")
            self._send_usb_uvc("backlight-compensation=0")
            self._send_usb_uvc("auto-exposure-mode=1")
            time.sleep(0.10)

        if entering_manual or self.last_exp_iris != iris:
            self._send_usb_uvc(f"iris-abs={iris}")
            self.last_exp_iris = iris
        if entering_manual or self.last_exp_gain != gain:
            self._send_usb_uvc(f"gain={gain}")
            self.last_exp_gain = gain
        if entering_manual or self.last_exp_shutter != shutter:
            self._send_usb_uvc(f"exposure-time-abs={shutter}")
            self.last_exp_shutter = shutter

        self.day_iris_abs = iris
        self.day_shutter_abs = shutter
        self.last_ev_applied = ev_idx
        self.exposure_profile = f"ev{ev_idx:.1f}_i{iris}_s{shutter}" + (f"_g{gain}" if gain > 0 else "")
        self.last_exposure_sync_ts = time.time()

    def apply_luma_feedback(self, mean_luma: float, clipped_frac: float) -> bool:
        """
        Closed-loop highlight-weighted OBS frame exposure feedback:
          - Strict highlight guard: if `clipped_frac > 1.5%` anywhere in the frame OR `mean_luma > 146`,
            immediately steps down exposure so sky/horizon clouds & white hulls never blow out.
          - Shadow recovery: only brightens (`mean_luma < 100`) when highlights have headroom (`clipped_frac < 0.5%`).
        """
        if self.exposure_profile == "night_ae8":
            return False
        # Ignore occluded indoor window-mullion frames in broad daylight (`mean < 18`)
        if mean_luma < 18.0 and solar_elevation_deg_utc(time.time(), self.lat_deg, self.lon_deg) > 2.0:
            return False
        if clipped_frac > 0.015 or mean_luma > 146.0:
            over_luma = max(0.0, (mean_luma - 122.0) / 40.0)
            over_clip = max(0.0, (clipped_frac - 0.008) * 3.2)
            ev_delta = -max(0.26, min(2.20, max(over_luma, over_clip) + 0.35 * min(over_luma, over_clip)))
        elif mean_luma < 100.0 and clipped_frac < 0.005:
            ev_delta = min(1.10, max(0.24, (118.0 - mean_luma) / 46.0))
        else:
            return False

        self.ambient_ev_bias = max(-1.6, min(1.6, self.ambient_ev_bias + 0.10 * ev_delta))
        self.local_ev_trim = max(-2.6, min(2.6, self.local_ev_trim + 0.90 * ev_delta))
        if not self.pending_focus_sync:
            self.sync_outdoor_exposure(force=False)
        return True

    def set_daylight_exposure_params(
        self, iris_abs: int, shutter_abs: int, apply_now: bool = True, manual_glare: bool = True
    ):
        self.day_iris_abs = max(0, min(12, int(iris_abs)))
        self.day_shutter_abs = max(1, min(10000, int(shutter_abs)))
        self.day_manual_glare = bool(manual_glare)
        if apply_now and self._has_usb_uvc():
            self.sync_outdoor_exposure(force=True)

    def home(self):
        if not self._send_usb_uvc("pan-tilt-arcsec=0,0"):
            self._send_visca(bytes([0x81, 0x01, 0x06, 0x04, 0xFF]))
        self.last_pan_steps = 0
        self.last_tilt_steps = 0
        self.last_pan_cmd_deg = 0.0
        self.last_tilt_cmd_deg = 0.0

    def configure_outdoor_window_defaults(self):
        """
        Configures solar- & horizon-aware outdoor exposure and calibrated 3D distance optical focus
        (or AF fallback if not yet calibrated). Uses direct USB 3.0 UVC Endpoint 0 when attached locally.
        """
        if self._has_usb_uvc():
            self.sync_outdoor_exposure(force=True)
            if self.has_calibrated_focus:
                self._send_usb_uvc("auto-focus=0")
                self.focus_mode = "calibrated_curve"
                q = self.query_usb_uvc("zoom-abs")
                try:
                    z_cur = int(q) if q is not None else (self.last_zoom_pos or 0)
                except ValueError:
                    z_cur = self.last_zoom_pos or 0
                self.actual_zoom_pos = z_cur
                self.pending_focus_sync = False
                self.set_focus_abs(
                    self.compute_calibrated_focus_abs(z_cur, self.last_dist_km or 3.0), force=True
                )
            else:
                self._send_usb_uvc("auto-focus=1")
                self.focus_mode = "auto_low"
            return
        self._send_visca(bytes([0x81, 0x01, 0x04, 0x23, 0x02, 0xFF]))
        self._send_visca(bytes([0x81, 0x0A, 0x01, 0x32, 0x00, 0x00, 0x03, 0x00, 0xFF]))
        self._send_visca(bytes([0x81, 0x01, 0x04, 0x38, 0x02, 0xFF]))
        self._send_visca(bytes([0x81, 0x01, 0x04, 0x58, 0x03, 0xFF]))
        self.focus_mode = "auto_low"

    def trigger_one_push_focus(self):
        """Re-applies calibrated 3D distance focus immediately (never uses unconstrained through-glass AF)."""
        if self._one_push_timer is not None:
            self._one_push_timer.cancel()
            self._one_push_timer = None
        if self.has_calibrated_focus and self._has_usb_uvc():
            self._send_usb_uvc("auto-focus=0")
            self.focus_mode = "calibrated_curve"
            q = self.query_usb_uvc("zoom-abs")
            try:
                z_cur = int(q) if q is not None else (self.actual_zoom_pos or self.last_zoom_pos or 0)
            except ValueError:
                z_cur = self.actual_zoom_pos or self.last_zoom_pos or 0
            self.actual_zoom_pos = z_cur
            self.set_focus_abs(self.compute_calibrated_focus_abs(z_cur, self.last_dist_km), force=True)
            return
        if self._send_usb_uvc("auto-focus=1"):
            self.focus_mode = "one_push"
            t = threading.Timer(1.25, self._finish_one_push_lock)
            t.daemon = True
            t.start()
            self._one_push_timer = t
            return
        self._send_visca(bytes([0x81, 0x01, 0x04, 0x38, 0x04, 0xFF]))
        self.focus_mode = "one_push"

    def _finish_one_push_lock(self):
        self._send_usb_uvc("auto-focus=0")
        self.focus_mode = "manual_lock"

    def set_auto_focus(self, enabled: bool = True):
        if self._one_push_timer is not None:
            self._one_push_timer.cancel()
            self._one_push_timer = None
        if enabled:
            if self.has_calibrated_focus:
                self._send_usb_uvc("auto-focus=0")
                self.focus_mode = "calibrated_curve"
                self.focus_trim_offset = 0
                q = self.query_usb_uvc("zoom-abs")
                try:
                    z_cur = int(q) if q is not None else (self.actual_zoom_pos or self.last_zoom_pos or 0)
                except ValueError:
                    z_cur = self.actual_zoom_pos or self.last_zoom_pos or 0
                self.actual_zoom_pos = z_cur
                self.set_focus_abs(
                    self.compute_calibrated_focus_abs(z_cur, self.last_dist_km or 2.5), force=True
                )
                return
            if not self._send_usb_uvc("auto-focus=1"):
                self._send_visca(bytes([0x81, 0x01, 0x04, 0x38, 0x02, 0xFF]))
                self._send_visca(bytes([0x81, 0x01, 0x04, 0x58, 0x03, 0xFF]))
            self.focus_mode = "auto_low"
        else:
            if not self._send_usb_uvc("auto-focus=0"):
                self._send_visca(bytes([0x81, 0x01, 0x04, 0x38, 0x03, 0xFF]))
            self.focus_mode = "manual_lock"

    def lock_focus_infinity(self):
        self.configure_outdoor_window_defaults()

    def _poll_settled_zoom_register(self, target_z: int) -> Optional[int]:
        """
        Polls actual hardware `?zoom-abs` over USB UVC and returns the physical register `zv`
        ONLY when the zoom stepper has arrived near `target_z` or physically stopped moving.
        Returns `None` while the zoom motor is still in flight so `focus-abs` never aborts zoom!
        """
        now = time.time()
        if not self._has_usb_uvc():
            return target_z if (now - self.last_zoom_target_change_ts) >= 3.5 else None
        q = self.query_usb_uvc("zoom-abs")
        if q is None:
            return target_z if (now - self.last_zoom_target_change_ts) >= 5.8 else None
        try:
            zv = int(q)
        except ValueError:
            return target_z
        prev = self.prev_polled_zoom_pos
        self.prev_polled_zoom_pos = zv
        self.actual_zoom_pos = zv
        if abs(zv - target_z) <= 45:
            return zv
        if (now - self.last_zoom_target_change_ts) >= 0.75 and prev is not None and abs(zv - prev) <= 4:
            return zv
        if (now - self.last_zoom_target_change_ts) >= 6.2:
            return zv
        return None

    def set_zoom_register(
        self, z_pos: int, force: bool = False, dist_km: Optional[float] = None, sync_focus: bool = True
    ):
        z_clamped = max(0, min(0x4000, int(round(z_pos))))
        if dist_km is not None:
            self.last_dist_km = float(dist_km)
        now = time.time()
        target_changed = self.last_zoom_pos is None or abs(z_clamped - self.last_zoom_pos) >= 24
        # Never fire periodic refresh while zoom is still in flight (prevents motor stutter)
        periodic_refresh = (now - self.last_zoom_send_ts) >= 12.0 and not self.pending_focus_sync
        if not force and not periodic_refresh and not target_changed:
            if sync_focus and self.has_calibrated_focus and self.focus_mode == "calibrated_curve":
                # Evaluate manual focus at ACTUAL hardware `?zoom-abs` readback once zoom has settled!
                zv_settled = self._poll_settled_zoom_register(self.last_zoom_pos)
                if zv_settled is not None:
                    f_want = self.compute_calibrated_focus_abs(zv_settled, dist_km)
                    if self.set_focus_abs(f_want, force=self.pending_focus_sync):
                        self.pending_focus_sync = False
            return
        if target_changed or force:
            self.last_zoom_target_change_ts = now
            self.pending_focus_sync = True
            self.prev_polled_zoom_pos = None
        if not self._send_usb_uvc(f"zoom-abs={z_clamped}"):
            cmd = bytes([0x81, 0x01, 0x04, 0x47]) + self._encode_nibble_16(z_clamped) + bytes([0xFF])
            self._send_visca(cmd)
        self.last_zoom_pos = z_clamped
        eff_hfov = self.register_to_hfov_deg(z_clamped)
        self.last_zoom_x = self.zoom_hfov_table[0][1] / max(0.5, eff_hfov)
        self.last_zoom_send_ts = time.time()
        if (
            sync_focus
            and not target_changed
            and self.has_calibrated_focus
            and self.focus_mode == "calibrated_curve"
            and self._has_usb_uvc()
        ):
            zv_settled = self._poll_settled_zoom_register(z_clamped)
            if zv_settled is not None:
                if self.set_focus_abs(self.compute_calibrated_focus_abs(zv_settled, dist_km), force=False):
                    self.pending_focus_sync = False

    def wait_zoom_settled(self, z_pos: int, timeout: float = 6.5) -> int:
        """Commands `zoom-abs` and polls hardware readback until the optical zoom motor physically arrives."""
        z_clamped = max(0, min(0x4000, int(round(z_pos))))
        self.set_zoom_register(z_clamped, force=True, sync_focus=False)
        if not self._has_usb_uvc():
            time.sleep(2.5)
            return z_clamped
        t0 = time.time()
        last_z = -99999
        still_cnt = 0
        while (time.time() - t0) < timeout:
            time.sleep(0.16)
            q = self.query_usb_uvc("zoom-abs")
            if q is None:
                continue
            try:
                zv = int(q)
            except ValueError:
                continue
            if abs(zv - z_clamped) <= 55:
                time.sleep(0.12)
                return zv
            if zv == last_z:
                still_cnt += 1
                if still_cnt >= 2:
                    self._send_usb_uvc(f"zoom-abs={z_clamped}")
                    still_cnt = 0
            else:
                still_cnt = 0
            last_z = zv
        return last_z if last_z >= 0 else z_clamped

    def wait_focus_settled(self, focus_abs: int, timeout: float = 3.5) -> int:
        """Commands manual `focus-abs` (after zoom is stationary) and polls until the focus lens arrives."""
        f_clamped = max(0, min(3371, int(round(focus_abs))))
        self.set_focus_abs(f_clamped, force=True)
        self.pending_focus_sync = False
        if not self._has_usb_uvc():
            time.sleep(0.45)
            return f_clamped
        t0 = time.time()
        last_f = f_clamped
        while (time.time() - t0) < timeout:
            time.sleep(0.14)
            q = self.query_usb_uvc("focus-abs")
            if q is None:
                continue
            try:
                fv = int(q)
                last_f = fv
                if abs(fv - f_clamped) <= 22:
                    time.sleep(0.16)
                    return fv
            except ValueError:
                pass
        return last_f

    def set_zoom(self, zoom_x: float, force: bool = False, dist_km: Optional[float] = None):
        z_pos = self.zoom_x_to_register(zoom_x)
        self.set_zoom_register(z_pos, force=force, dist_km=dist_km)
        self.last_zoom_x = max(1.0, min(self.max_zoom_x, float(zoom_x)))

    def _has_usb_uvc(self) -> bool:
        if not (self.raw_udp and self.usb_vendor):
            return False
        if self.usb_ready:
            return True
        with self.lock:
            return self._ensure_usb_pipe()

    def move_raw_hardware_deg(self, pan_hw_deg: float, tilt_hw_deg: float) -> Tuple[float, float]:
        """
        Commands raw FoMaKo hardware gimbal angles directly (bypassing Synoptic bore-sight offsets).
        Returns the exact quantized `(pan_cmd_deg, tilt_cmd_deg)` sent to the stepper motors.
        """
        adj_pan = max(-168.0, min(168.0, float(pan_hw_deg)))
        adj_tilt = max(-26.8, min(88.0, float(tilt_hw_deg)))
        if self._has_usb_uvc():
            pan_arc = int(round((adj_pan * 3600.0) / 500.0)) * 500
            tilt_arc = int(round((adj_tilt * 3600.0) / 500.0)) * 500
            self._send_usb_uvc(f"pan-tilt-arcsec={pan_arc},{tilt_arc}")
            self.last_pan_steps = pan_arc // 250
            self.last_tilt_steps = tilt_arc // 250
            self.last_pan_cmd_deg = pan_arc / 3600.0
            self.last_tilt_cmd_deg = tilt_arc / 3600.0
            self.last_pt_send_ts = time.time()
            return self.last_pan_cmd_deg, self.last_tilt_cmd_deg

        pan_steps = int(round(adj_pan * self.steps_per_deg))
        tilt_steps = int(round(adj_tilt * self.steps_per_deg))
        cmd = (
            bytes([0x81, 0x01, 0x06, 0x02, 0x14, 0x10])
            + self._encode_nibble_16(pan_steps)
            + self._encode_nibble_16(tilt_steps)
            + bytes([0xFF])
        )
        self._send_visca(cmd)
        self.last_pan_steps = pan_steps
        self.last_tilt_steps = tilt_steps
        self.last_pan_cmd_deg = pan_steps / max(1.0, self.steps_per_deg)
        self.last_tilt_cmd_deg = tilt_steps / max(1.0, self.steps_per_deg)
        self.last_pt_send_ts = time.time()
        return self.last_pan_cmd_deg, self.last_tilt_cmd_deg

    def compute_hw_angles(
        self,
        pan_syn_deg: float,
        tilt_syn_deg: float,
        zoom_x: float = 30.0,
        dist_km: Optional[float] = None,
    ) -> Tuple[float, float]:
        return project_synoptic_to_ptz_gimbal(
            pan_syn_deg=pan_syn_deg,
            tilt_syn_deg=tilt_syn_deg,
            yaw_offset_deg=self.pan_offset_deg,
            pitch_offset_deg=self.tilt_offset_deg,
            roll_offset_deg=self.roll_offset_deg,
            pan_scale=self.pan_scale,
            tilt_scale=self.tilt_scale,
            zoom_x=zoom_x,
            collimation_pan_10x_deg=self.collimation_pan_10x_deg,
            collimation_tilt_10x_deg=self.collimation_tilt_10x_deg,
            dist_km=dist_km,
            baseline_cam_m=self.baseline_cam_m,
        )

    def move_absolute(
        self,
        pan_deg: float,
        tilt_deg: float,
        zoom_x: float,
        pan_dps: float = 0.0,
        tilt_dps: float = 0.0,
        dist_km: Optional[float] = None,
        target_kind: str = "",
        force: bool = False,
    ):
        now = time.time()

        # 0.25s kinematic lead compensates for stepper acceleration latency
        lead_pan = pan_deg + pan_dps * 0.25
        lead_tilt = tilt_deg + tilt_dps * 0.25
        raw_pan, raw_tilt = self.compute_hw_angles(lead_pan, lead_tilt, zoom_x=zoom_x, dist_km=dist_km)
        adj_pan = max(-168.0, min(168.0, raw_pan))
        adj_tilt = max(-26.8, min(88.0, raw_tilt))

        z_want = self.zoom_x_to_register(zoom_x)
        will_change_zoom = self.last_zoom_pos is None or abs(z_want - self.last_zoom_pos) >= 24 or force
        if will_change_zoom:
            # Set feed-forward exposure for destination BEFORE starting `zoom-abs` so exposure writes
            # never preempt an in-flight zoom stepper move!
            self.sync_outdoor_exposure(
                force=False,
                pan_syn_deg=lead_pan,
                tilt_syn_deg=lead_tilt,
                zoom_x=zoom_x,
                allow_before_zoom=True,
            )

        if self._has_usb_uvc():
            # FoMaKo UVC CT_PANTILT_ABSOLUTE_CONTROL uses 500-arcsec (2-VISCA-step = 0.139 deg) increments
            pan_arc = int(round((adj_pan * 3600.0) / 500.0)) * 500
            tilt_arc = int(round((adj_tilt * 3600.0) / 500.0)) * 500
            pan_steps = pan_arc // 250
            tilt_steps = tilt_arc // 250
            periodic_refresh = (now - self.last_pt_send_ts) >= 4.0 and not self.pending_focus_sync
            if force or periodic_refresh or self.last_pan_steps is None or (now - self.last_pt_send_ts) >= 0.18:
                dp = abs(pan_steps - (self.last_pan_steps or 0))
                dt_st = abs(tilt_steps - (self.last_tilt_steps or 0))
                if force or periodic_refresh or self.last_pan_steps is None or dp >= 2 or dt_st >= 2:
                    is_large_slew = self.last_pan_steps is None or dp >= 28 or dt_st >= 28
                    if self._send_usb_uvc(f"pan-tilt-arcsec={pan_arc},{tilt_arc}"):
                        self.last_pan_steps = pan_steps
                        self.last_tilt_steps = tilt_steps
                        self.last_pan_cmd_deg = pan_arc / 3600.0
                        self.last_tilt_cmd_deg = tilt_arc / 3600.0
                        now_pt = time.time()
                        self.last_pt_send_ts = now_pt
                        if is_large_slew:
                            self.last_large_slew_ts = now_pt
            self.set_zoom(zoom_x, force=force, dist_km=dist_km)
            if not will_change_zoom and not self.pending_focus_sync:
                self.sync_outdoor_exposure(
                    force=False, pan_syn_deg=lead_pan, tilt_syn_deg=lead_tilt, zoom_x=zoom_x
                )
            return

        pan_steps = int(round(adj_pan * self.steps_per_deg))
        tilt_steps = int(round(adj_tilt * self.steps_per_deg))

        periodic_refresh = (now - self.last_pt_send_ts) >= 2.5
        if force or periodic_refresh or self.last_pan_steps is None or (now - self.last_pt_send_ts) >= 0.16:
            dp = abs(pan_steps - (self.last_pan_steps or 0))
            dt_st = abs(tilt_steps - (self.last_tilt_steps or 0))
            if force or periodic_refresh or self.last_pan_steps is None or dp >= 1 or dt_st >= 1:
                deg_jump = math.hypot(dp, dt_st) / max(1.0, self.steps_per_deg)
                if deg_jump > 6.0:
                    v_pan, v_tilt = 0x14, 0x10
                elif deg_jump > 1.5:
                    v_pan, v_tilt = 0x0B, 0x09
                else:
                    req_spd = max(abs(pan_dps), abs(tilt_dps))
                    v_pan = max(0x03, min(0x09, int(round(3 + req_spd * 1.4))))
                    v_tilt = max(0x03, min(0x08, int(round(3 + req_spd * 1.4))))

                cmd = (
                    bytes([0x81, 0x01, 0x06, 0x02, v_pan, v_tilt])
                    + self._encode_nibble_16(pan_steps)
                    + self._encode_nibble_16(tilt_steps)
                    + bytes([0xFF])
                )
                self._send_visca(cmd)
                self.last_pan_steps = pan_steps
                self.last_tilt_steps = tilt_steps
                self.last_pan_cmd_deg = pan_steps / max(1.0, self.steps_per_deg)
                self.last_tilt_cmd_deg = tilt_steps / max(1.0, self.steps_per_deg)
                self.last_pt_send_ts = time.time()

        self.set_zoom(zoom_x, force=force, dist_km=dist_km)

    def move_celestial_hw(self, pan_hw_deg: float, tilt_hw_deg: float, zoom_x: float, force: bool = False):
        """Spherical Alt-Azimuth fallback for FoMaKo PTZ."""
        now = time.time()
        if force or (now - self.last_pt_send_ts) >= 6.0 or abs(pan_hw_deg - self.last_pan_cmd_deg) >= 0.05:
            self.move_raw_hardware_deg(pan_hw_deg, tilt_hw_deg)
        self.set_zoom(zoom_x, force=force, dist_km=384000.0)

    def get_state(self) -> dict:
        return {
            "enabled": True,
            "camera_model": "fomako",
            "transport": "usb_uvc" if self.usb_ready else "visca_udp",
            "host": "USB 3.0 UVC" if self.usb_ready else f"{self.host}:{self.port}",
            "visca_host": f"{self.host}:{self.port}",
            "steps_per_deg": self.steps_per_deg,
            "pan_offset_deg": round(self.pan_offset_deg, 3),
            "tilt_offset_deg": round(self.tilt_offset_deg, 3),
            "roll_offset_deg": round(self.roll_offset_deg, 3),
            "pan_scale": round(self.pan_scale, 4),
            "tilt_scale": round(self.tilt_scale, 4),
            "max_zoom_x": self.max_zoom_x,
            "zoom_x": round(self.last_zoom_x, 2),
            "zoom_pos": self.actual_zoom_pos if self.actual_zoom_pos is not None else self.last_zoom_pos,
            "focus_abs": self.last_focus_abs,
            "focus_trim": self.focus_trim_offset,
            "dist_km": round(self.last_dist_km, 2) if self.last_dist_km is not None else None,
            "exposure_profile": self.exposure_profile,
            "exp_iris": self.last_exp_iris,
            "exp_shutter": self.last_exp_shutter,
            "exp_gain": self.last_exp_gain,
            "ambient_ev_bias": round(self.ambient_ev_bias, 2),
            "local_ev_trim": round(self.local_ev_trim, 2),
            "pan_steps": self.last_pan_steps,
            "tilt_steps": self.last_tilt_steps,
            "vel_steps": [self.vel_step_x, self.vel_step_y],
            "sent_packets": self.sent_packets,
            "last_error": self.last_error,
            "focus_mode": self.focus_mode,
        }

    def move_velocity(self, pan_deg_per_s: float, tilt_deg_per_s: float, zoom_x: float):
        pan_spd = min(0x18, max(0x00, int(round(abs(pan_deg_per_s) * 0.6))))
        tilt_spd = min(0x14, max(0x00, int(round(abs(tilt_deg_per_s) * 0.6))))
        pan_dir = 0x02 if pan_deg_per_s > 0.05 else (0x01 if pan_deg_per_s < -0.05 else 0x03)
        tilt_dir = 0x01 if tilt_deg_per_s > 0.05 else (0x02 if tilt_deg_per_s < -0.05 else 0x03)
        cmd = bytes([0x81, 0x01, 0x06, 0x01, pan_spd, tilt_spd, pan_dir, tilt_dir, 0xFF])
        self._send_visca(cmd)
        if abs(zoom_x - self.last_zoom_x) > 0.25:
            self.set_zoom(zoom_x)


class AxisVapixDriver(BasePTZDriver):
    """Axis VAPIX HTTP REST PTZ driver for Axis V5938 / Q62 / Q61 cameras."""

    def __init__(self, host: str, username: str = "root", password: str = ""):
        self.base_url = f"http://{host}/axis-cgi/com/ptz.cgi"
        mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()
        mgr.add_password(None, self.base_url, username, password)
        auth = urllib.request.HTTPDigestAuthHandler(mgr)
        self.opener = urllib.request.build_opener(auth)

    def _call(self, params: dict):
        qs = urllib.parse.urlencode(params)
        try:
            self.opener.open(f"{self.base_url}?{qs}", timeout=0.5).read()
        except Exception:
            pass

    def lock_focus_infinity(self):
        self._call({"autofocus": "off", "focus": 9999})

    def move_absolute(self, pan_deg: float, tilt_deg: float, zoom_x: float):
        zoom_val = int(1 + min(1.0, max(0.0, (zoom_x - 1.0) / 19.0)) * 9998)
        self._call({"pan": round(pan_deg, 2), "tilt": round(tilt_deg, 2), "zoom": zoom_val, "speed": 85})

    def move_velocity(self, pan_deg_per_s: float, tilt_deg_per_s: float, zoom_x: float):
        p = max(-100, min(100, int(round(pan_deg_per_s * 1.5))))
        t = max(-100, min(100, int(round(tilt_deg_per_s * 1.5))))
        self._call({"continuouspantiltmove": f"{p},{t}"})


class SonyCGIClient:
    """
    Persistent HTTP/1.1 Keep-Alive + RFC 2617 Digest Authentication client for Sony SRG-A40 / SRG-X400.
    Caches the Digest challenge (`realm`, `nonce`, `qop`, `opaque`) and increments `nc` over a single
    persistent TCP socket (`HttpMaxKeepAliveIdle=70`) so PTZF commands execute in ~1.5 ms on Gigabit LAN,
    while honoring Sony's `Referer` check and brute-force protection (`BruteForceAttackCount=8`).
    """

    def __init__(self, host: str, username: str = "admin", password: str = "", port: int = 80):
        self.host = host
        self.port = port
        self.username = username
        self.password = password
        self.lock = threading.Lock()
        self._conn: Optional[http.client.HTTPConnection] = None
        self._realm = ""
        self._nonce = ""
        self._qop = ""
        self._opaque = ""
        self._nc = 0
        self.auth_failed = False
        self.last_auth_fail_ts = 0.0

    def update_credentials(self, host: Optional[str] = None, username: Optional[str] = None, password: Optional[str] = None):
        with self.lock:
            changed = False
            if host and host != self.host:
                self.host = host
                changed = True
            if username is not None and username != self.username:
                self.username = username
                changed = True
            if password is not None and password != self.password:
                self.password = password
                changed = True
            if changed:
                self._close_conn()
                self._nonce = ""
                self._nc = 0
                self.auth_failed = False
                self.last_auth_fail_ts = 0.0

    def _close_conn(self):
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:
                pass
            self._conn = None

    def _parse_digest_header(self, www_auth: str):
        self._realm = ""
        self._nonce = ""
        self._qop = ""
        self._opaque = ""
        self._nc = 0
        for m in re.finditer(r'([a-zA-Z0-9_-]+)\s*=\s*(?:"([^"]*)"|([^\s,]+))', www_auth):
            k = m.group(1).lower()
            v = m.group(2) if m.group(2) is not None else (m.group(3) or "")
            if k == "realm":
                self._realm = v
            elif k == "nonce":
                self._nonce = v
            elif k == "qop":
                self._qop = "auth" if "auth" in [x.strip() for x in v.split(",")] else v
            elif k == "opaque":
                self._opaque = v

    def _build_digest_header(self, method: str, path: str) -> str:
        self._nc += 1
        nc_str = f"{self._nc:08x}"
        cnonce = os.urandom(8).hex()
        ha1 = hashlib.md5(f"{self.username}:{self._realm}:{self.password}".encode("utf-8")).hexdigest()
        ha2 = hashlib.md5(f"{method}:{path}".encode("utf-8")).hexdigest()
        if self._qop:
            resp = hashlib.md5(
                f"{ha1}:{self._nonce}:{nc_str}:{cnonce}:{self._qop}:{ha2}".encode("utf-8")
            ).hexdigest()
        else:
            resp = hashlib.md5(f"{ha1}:{self._nonce}:{ha2}".encode("utf-8")).hexdigest()
        parts = [
            f'username="{self.username}"',
            f'realm="{self._realm}"',
            f'nonce="{self._nonce}"',
            f'uri="{path}"',
            f'response="{resp}"',
            "algorithm=MD5",
        ]
        if self._qop:
            parts.append(f"qop={self._qop}")
            parts.append(f"nc={nc_str}")
            parts.append(f'cnonce="{cnonce}"')
        if self._opaque:
            parts.append(f'opaque="{self._opaque}"')
        return "Digest " + ", ".join(parts)

    def _raw_get(self, path: str, auth_hdr: Optional[str], timeout: float) -> Tuple[int, Dict[str, str], str]:
        for attempt in range(2):
            try:
                if self._conn is None:
                    self._conn = http.client.HTTPConnection(self.host, self.port, timeout=timeout)
                else:
                    self._conn.timeout = timeout
                headers = {
                    "User-Agent": "HudsonRiverPTZ/1.0",
                    "Referer": f"http://{self.host}/",
                    "Connection": "keep-alive",
                }
                if auth_hdr:
                    headers["Authorization"] = auth_hdr
                self._conn.request("GET", path, headers=headers)
                resp = self._conn.getresponse()
                body = resp.read().decode("utf-8", errors="replace")
                hdrs = {k.lower(): v for k, v in resp.getheaders()}
                if hdrs.get("connection", "").lower() == "close":
                    self._close_conn()
                return resp.status, hdrs, body
            except Exception:
                self._close_conn()
                if attempt == 1:
                    raise
        raise ConnectionError("Sony CGI connection failed")

    def get(self, path: str, auth: bool = True, timeout: float = 1.6) -> Tuple[int, str]:
        with self.lock:
            if not auth:
                st, _, body = self._raw_get(path, None, timeout)
                return st, body

            # Never hammer wrong credentials against Sony's 8-strike brute-force lockout!
            if self.auth_failed and (time.time() - self.last_auth_fail_ts) < 90.0:
                return 401, ""

            auth_hdr = self._build_digest_header("GET", path) if self._nonce else None
            st, hdrs, body = self._raw_get(path, auth_hdr, timeout)
            if st != 401:
                if st in (200, 204):
                    self.auth_failed = False
                return st, body

            www_auth = hdrs.get("www-authenticate", "")
            if "digest" not in www_auth.lower():
                self.auth_failed = True
                self.last_auth_fail_ts = time.time()
                return 401, body

            self._parse_digest_header(www_auth)
            auth_hdr2 = self._build_digest_header("GET", path)
            st2, _, body2 = self._raw_get(path, auth_hdr2, timeout)
            if st2 == 401:
                self.auth_failed = True
                self.last_auth_fail_ts = time.time()
            else:
                self.auth_failed = False
            return st2, body2


class SonySRGA40Driver(BasePTZDriver):
    """
    Production PTZ driver for the Sony SRG-A40 (4K 20x Optical + 30x Clear Image Zoom PTZ):
      - Primary Control: Persistent HTTP/1.1 Keep-Alive CGI (`/command/ptzf.cgi`, `imaging.cgi`,
        `ircf.cgi`, `inquiry.cgi`) over Gigabit Ethernet — zero serial adapters or DIP switches needed!
      - Optional Secondary Control: Native Sony VISCA-over-IP (`UDP port 52381`, 8-byte header)
        if bottom DIP Switch 3 is enabled.
      - Hardware Coordinate Precision: `51.2` steps/deg (`0.0195°` pan/tilt resolution, 7x finer
        than FoMaKo!) with 4K Clear Image Zoom (`0x0000..0x5556` = `1x..30x`).
      - Through-Window Optical Focus: Hardware `FocusNearLimit=3000` (`20 meters`) physically prevents
        autofocus from ever hunting onto indoor window glass while supporting manual `AbsoluteFocus`.
      - Exmor R Outdoor Exposure & True Day/Night: 1/10,000s shutter ceiling + `VisibilityEnhancer`
        dynamic tone-mapping + closed-loop `ExposureCompensation` highlight guard + auto IR-cut night mode.
    """

    SONY_STEPS_PER_DEG = 51.2

    def __init__(
        self,
        host: str = "192.168.1.136",
        username: str = "admin",
        password: str = "",
        visca_port: int = 52381,
        pan_offset_deg: float = 0.0,
        tilt_offset_deg: float = 0.0,
        roll_offset_deg: float = 0.0,
        pan_scale: float = 1.0,
        tilt_scale: float = 1.0,
        max_zoom_x: float = 30.0,
        calibration: Optional[dict] = None,
    ):
        self.camera_model = "sony_a40"
        self.host = (host or "192.168.1.136").strip()
        self.username = (username or "admin").strip()
        self.password = password or ""
        self.port = visca_port
        self.steps_per_deg = self.SONY_STEPS_PER_DEG

        self.pan_offset_deg = pan_offset_deg
        self.tilt_offset_deg = tilt_offset_deg
        self.roll_offset_deg = roll_offset_deg
        self.pan_scale = pan_scale
        self.tilt_scale = tilt_scale
        self.collimation_pan_10x_deg = 0.0
        self.collimation_tilt_10x_deg = 0.0
        self.baseline_cam_m = (0.25, 0.0, 0.0)
        self.lat_deg = 40.744823
        self.lon_deg = -74.008063
        self.syn_az_deg = 207.186
        self.syn_tilt_deg = 3.171
        self.syn_roll_deg = -0.945

        self.zoom_hfov_table: List[Tuple[int, float]] = list(DEFAULT_SONY_A40_ZOOM_HFOV_TABLE)
        self.focus_zoom_table: List[Tuple[int, int]] = [(0, 0x2000), (16384, 0x2000), (21846, 0x2000)]
        self.focus_inv_km_slope: float = 0.0
        self.focus_trim_offset: int = 0
        # Sony's broadcast lens block has hardware parfocal zoom-compensated focus with a 20m near-limit
        self.has_calibrated_focus: bool = False
        self.focus_mode: str = "sony_af_near1300"
        self.focus_near_limit_hex: str = "1300"  # ~160m near limit prevents macro/sky AF hunting & window glass lock

        self.max_zoom_x = max(2.0, min(30.0, float(max_zoom_x)))
        self.max_zoom_reg = 0x5556  # 30x Clear Image Zoom in 4K (0x4000 = 20x optical)

        self.cgi = SonyCGIClient(self.host, self.username, self.password)
        self.lock = threading.Lock()
        self.visca_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.visca_sock.setblocking(False)
        self.visca_seq = 1
        self.visca_ip_ready = False

        # Compatibility flags used by server.py loops & calibrator
        self.usb_ready = True
        self.pending_focus_sync = False

        self.model_name = "SRG-A40"
        self.serial_num = ""
        self.firmware_ver = ""
        self.reachable = False
        self.needs_password_init = False
        self.defaults_configured = False
        self.last_status_probe_ts = 0.0

        self.exposure_comp: int = 7  # Sony 0..14 scale where 7 = 0.0 EV
        self.ambient_ev_bias: float = 0.0
        self.local_ev_trim: float = 0.0
        self.exposure_profile: str = "sony_auto_ve_comp7"

        self.last_pan_steps: Optional[int] = None
        self.last_tilt_steps: Optional[int] = None
        self.last_pan_cmd_deg: float = 0.0
        self.last_tilt_cmd_deg: float = 0.0
        self.actual_pan_deg: Optional[float] = None
        self.actual_tilt_deg: Optional[float] = None
        self.vel_step_x: int = 0
        self.vel_step_y: int = 0
        self.last_vel_check_ts: float = 0.0
        self.slew_until_ts: float = 0.0
        self.smooth_hw_pan_dps: float = 0.0
        self.smooth_hw_tilt_dps: float = 0.0
        self.last_vel_ema_ts: float = 0.0
        self.last_target_hw_pan: Optional[float] = None
        self.last_target_hw_tilt: Optional[float] = None
        self.last_zoom_pos: Optional[int] = None
        self.actual_zoom_pos: Optional[int] = None
        self.last_zoom_x: float = 1.0
        self.last_focus_abs: Optional[int] = 0x2000
        self.last_dist_km: Optional[float] = None

        self.last_pt_send_ts = 0.0
        self.last_large_slew_ts = 0.0
        self.last_zoom_send_ts = 0.0
        self.last_zoom_target_change_ts = 0.0
        self.last_focus_send_ts = 0.0
        self.last_exposure_sync_ts = 0.0
        self.sent_packets = 0
        self.last_error = ""

        # Pose history & closed-loop visual guidance tracking state
        self.pose_lock = threading.Lock()
        self.pose_history: deque = deque(maxlen=96)
        self._pose_anchor_ts: float = 0.0
        self._pose_anchor_pan: float = 0.0
        self._pose_anchor_tilt: float = 0.0
        self._needs_post_slew_query: bool = False
        self._last_vis_recenter_seq: int = 0
        self._active_track_id: Optional[str] = None

        if calibration:
            self.apply_calibration_dict(calibration)
        self.probe_connection(force=True)

    def set_credentials(self, host: Optional[str] = None, username: Optional[str] = None, password: Optional[str] = None):
        if host:
            self.host = host.strip()
        if username is not None:
            self.username = username.strip() or "admin"
        if password is not None:
            self.password = password
        self.cgi.update_credentials(self.host, self.username, self.password)
        self.defaults_configured = False
        self.probe_connection(force=True)
        if self.reachable and not self.needs_password_init and not self.cgi.auth_failed:
            self.configure_outdoor_window_defaults()

    def apply_calibration_dict(self, cal: dict):
        self.lat_deg = float(cal.get("lat", self.lat_deg))
        self.lon_deg = float(cal.get("lon", self.lon_deg))
        self.syn_az_deg = float(cal.get("az_deg", self.syn_az_deg))
        self.syn_tilt_deg = float(cal.get("tilt_deg", self.syn_tilt_deg))
        self.syn_roll_deg = float(cal.get("roll_deg", self.syn_roll_deg))

        prof = (cal.get("ptz_profiles") or {}).get("sony_a40")
        src = prof if isinstance(prof, dict) else cal

        new_host = os.environ.get("SONY_PTZ_HOST", "").strip() or str(
            cal.get("ptz_sony_host") or src.get("host") or self.host
        ).strip()
        new_user = os.environ.get("SONY_PTZ_USER", "").strip() or str(
            cal.get("ptz_sony_user") or src.get("username") or self.username
        ).strip()
        new_pass = os.environ.get("SONY_PTZ_PASS")
        if new_pass is None:
            new_pass = cal.get("ptz_sony_pass")
        if new_pass is None and isinstance(prof, dict):
            new_pass = prof.get("password")
        if new_pass is None:
            new_pass = self.password

        self.cgi.update_credentials(new_host, new_user, str(new_pass))
        self.host = new_host
        self.username = new_user
        self.password = str(new_pass)

        self.pan_offset_deg = float(src.get("ptz_pan_offset_deg", self.pan_offset_deg))
        self.tilt_offset_deg = float(src.get("ptz_tilt_offset_deg", self.tilt_offset_deg))
        self.roll_offset_deg = float(src.get("ptz_roll_offset_deg", 0.0))
        self.pan_scale = float(src.get("ptz_pan_scale", 1.0))
        self.tilt_scale = float(src.get("ptz_tilt_scale", 1.0))
        self.collimation_pan_10x_deg = float(src.get("ptz_collimation_pan_10x_deg", 0.0))
        self.collimation_tilt_10x_deg = float(src.get("ptz_collimation_tilt_10x_deg", 0.0))
        self.max_zoom_x = max(2.0, min(30.0, float(src.get("ptz_max_zoom_x", self.max_zoom_x))))

        az = math.radians(self.syn_az_deg)
        el = math.radians(self.syn_tilt_deg)
        be = float(src.get("ptz_offset_east_m", 0.25))
        bn = float(src.get("ptz_offset_north_m", 0.0))
        bu = float(src.get("ptz_offset_up_m", 0.0))
        bx = be * math.cos(az) - bn * math.sin(az)
        by = -be * math.sin(az) * math.sin(el) - bn * math.cos(az) * math.sin(el) + bu * math.cos(el)
        bz = be * math.sin(az) * math.cos(el) + bn * math.cos(az) * math.cos(el) + bu * math.sin(el)
        self.baseline_cam_m = (bx, by, bz)

        z_tbl = src.get("ptz_zoom_hfov_table")
        if isinstance(z_tbl, list) and len(z_tbl) >= 3:
            self.zoom_hfov_table = sorted((int(p[0]), float(p[1])) for p in z_tbl)

        self.last_pan_steps = None

    @staticmethod
    def _parse_kv_body(body: str) -> Dict[str, str]:
        out: Dict[str, str] = {}
        for part in body.strip().replace("\r", "").replace("\n", "&").split("&"):
            if "=" in part:
                k, v = part.split("=", 1)
                out[k.strip()] = v.strip()
        return out

    @classmethod
    def _deg_to_hex4(cls, deg: float) -> Tuple[str, int, float]:
        steps = int(round(deg * cls.SONY_STEPS_PER_DEG))
        raw16 = steps & 0xFFFF
        return f"{raw16:04x}", steps, steps / cls.SONY_STEPS_PER_DEG

    @classmethod
    def _hex_to_deg(cls, hex_str: str) -> float:
        val = int(hex_str, 16)
        if val > 0x7FFF:
            val -= 0x10000
        return val / cls.SONY_STEPS_PER_DEG

    def probe_connection(self, force: bool = False) -> bool:
        now = time.time()
        if not force and (now - self.last_status_probe_ts) < 6.0:
            return self.reachable and not self.needs_password_init and not self.cgi.auth_failed
        self.last_status_probe_ts = now
        try:
            st, sys_body = self.cgi.get("/command/inquiry.cgi?inq=system", auth=False, timeout=1.5)
            if st == 200 and "ModelName=" in sys_body:
                kv = self._parse_kv_body(sys_body)
                self.model_name = kv.get("ModelName", "SRG-A40")
                self.serial_num = kv.get("Serial", "")
                self.firmware_ver = kv.get("BuildNumber", "")
                self.reachable = True
            else:
                self.reachable = False
                self.last_error = f"Sony CGI HTTP {st}"
                return False

            st_info, info_body = self.cgi.get("/command/inquiry.cgi?inq=sysinfo", auth=True, timeout=1.8)
            if st_info == 401:
                self.needs_password_init = False
                self.last_error = (
                    f"Sony SRG-A40 auth needed: enter admin password for {self.host} in Mission Control"
                )
                return False
            if st_info == 200:
                kv_info = self._parse_kv_body(info_body)
                pw_changed = kv_info.get("PasswordChanged", "1")
                if pw_changed == "0":
                    self.needs_password_init = True
                    self.defaults_configured = False
                    self.last_error = (
                        f"First-time Sony setup: open http://{self.host}/ to set an admin password, "
                        f"then enter it in Mission Control"
                    )
                    return False
                was_uninit = self.needs_password_init
                self.needs_password_init = False
                self.last_error = ""
                if was_uninit or not self.defaults_configured:
                    self.configure_outdoor_window_defaults()
                return True
        except Exception as exc:
            self.reachable = False
            eno = getattr(exc, "errno", None)
            if eno == 65:
                self.last_error = (
                    f"macOS Local Network permission needed on Mac mini to reach Sony {self.host} "
                    f"(System Settings -> Privacy & Security -> Local Network)"
                )
            else:
                self.last_error = f"Sony {self.host}: {type(exc).__name__} ({exc})"
        return False

    def _send_cgi(self, cgi_name: str, params_qs: str, timeout: float = 1.4) -> bool:
        if not self.probe_connection(force=False):
            return False
        path = f"/command/{cgi_name}?{params_qs}"
        try:
            st, _ = self.cgi.get(path, auth=True, timeout=timeout)
            if st in (200, 204):
                self.sent_packets += 1
                self.last_error = ""
                return True
            if st == 503:
                # 503 indicates initial password setup is still pending or camera is in standby
                self.needs_password_init = True
                self.last_error = (
                    f"Sony SRG-A40 HTTP 503 — set initial admin password at http://{self.host}/"
                )
                return False
            self.last_error = f"Sony {cgi_name} HTTP {st}"
            return False
        except Exception as exc:
            self.last_error = f"Sony CGI {type(exc).__name__}"
            return False

    def query_ptzf(self) -> Optional[Dict[str, str]]:
        """Queries live `AbsolutePTZF` (pan, tilt, zoom, focus) & `PtzfStatus` from the SRG-A40."""
        try:
            st, body = self.cgi.get("/command/inquiry.cgi?inq=ptzf", auth=True, timeout=1.6)
            if st == 200 and body:
                kv = self._parse_kv_body(body)
                aptzf = kv.get("AbsolutePTZF", "")
                parts = aptzf.split(",")
                if len(parts) >= 4:
                    try:
                        t_q = time.time()
                        self.actual_pan_deg = self._hex_to_deg(parts[0])
                        self.actual_tilt_deg = self._hex_to_deg(parts[1])
                        self.actual_zoom_pos = int(parts[2], 16)
                        self.last_focus_abs = int(parts[3], 16)
                        self._record_pose_anchor(
                            t_q,
                            self.actual_pan_deg,
                            self.actual_tilt_deg,
                            self.register_to_hfov_deg(self.actual_zoom_pos),
                        )
                    except ValueError:
                        pass
                hw_fm = kv.get("FocusMode", "")
                if hw_fm and hw_fm != "manual" and self.focus_mode == "manual_lock":
                    self.focus_mode = "sony_af_20m_limit"
                z_range = kv.get("ZoomMovementRange", "")
                zr_parts = z_range.split(",")
                if len(zr_parts) >= 3:
                    try:
                        self.max_zoom_reg = int(zr_parts[2], 16)
                    except ValueError:
                        pass
                return kv
        except Exception:
            pass
        return None

    def _has_usb_uvc(self) -> bool:
        return False

    def query_usb_uvc(self, ctrl_name: str) -> Optional[str]:
        return None

    def register_to_hfov_deg(self, z_pos: int) -> float:
        tbl = self.zoom_hfov_table
        if z_pos <= tbl[0][0]:
            return tbl[0][1]
        if z_pos >= tbl[-1][0]:
            return tbl[-1][1]
        for i in range(len(tbl) - 1):
            z0, h0 = tbl[i]
            z1, h1 = tbl[i + 1]
            if z0 <= z_pos <= z1:
                t = (z_pos - z0) / max(1.0, float(z1 - z0))
                return math.exp(math.log(h0) * (1.0 - t) + math.log(h1) * t)
        return tbl[-1][1]

    def zoom_x_to_register(self, zoom_x: float) -> int:
        tbl = self.zoom_hfov_table
        wide_hfov = tbl[0][1]
        min_hfov = tbl[-1][1]
        clamped_z = max(1.0, min(self.max_zoom_x, float(zoom_x)))
        desired_hfov = max(min_hfov, min(wide_hfov, wide_hfov / clamped_z))
        if desired_hfov >= wide_hfov:
            return tbl[0][0]
        if desired_hfov <= min_hfov:
            return min(self.max_zoom_reg, tbl[-1][0])
        log_d = math.log(desired_hfov)
        for i in range(len(tbl) - 1):
            z0, h0 = tbl[i]
            z1, h1 = tbl[i + 1]
            if h1 <= desired_hfov <= h0:
                lh0, lh1 = math.log(h0), math.log(h1)
                frac = (lh0 - log_d) / max(1e-6, lh0 - lh1)
                return max(0, min(self.max_zoom_reg, int(round(z0 + frac * (z1 - z0)))))
        return min(self.max_zoom_reg, tbl[-1][0])

    # Empirically calibrated 9-stop 4K infinity `AbsoluteFocus` cam curve for Sony SRG-A40
    # from 1.0x (`0x0000`) through 30.0x (`0x5556`):
    SONY_INFINITY_FOCUS_TABLE: Tuple[Tuple[int, int], ...] = (
        (0x0000, 0x6000),  # 1.0x wide (rear-focus cam curves up sharply at 1x; pinpoint stars + skyline!)
        (0x0C00, 0x3200),  # 1.3x
        (0x16A0, 0x1550),  # 1.8x
        (0x22B8, 0x1425),  # 2.7x
        (0x2A00, 0x1020),  # 4.0x
        (0x3180, 0x1028),  # 7.0x
        (0x3500, 0x1078),  # 10.0x
        (0x4000, 0x10D8),  # 20.0x optical
        (0x5556, 0x10F0),  # 30.0x Clear Image Zoom
    )

    @classmethod
    def compute_sony_focus_reg_for_zoom_and_dist(cls, z_pos: int, dist_km: Optional[float] = None) -> int:
        """
        Piecewise-linear 4K infinity cam interpolation across `SONY_INFINITY_FOCUS_TABLE`
        plus telephoto-weighted `1/d` near-distance diopter pull above 4.5x (`0x2cc0`).
        """
        zp = max(0, min(0x5556, int(z_pos)))
        tbl = cls.SONY_INFINITY_FOCUS_TABLE
        if zp <= tbl[0][0]:
            f_inf = float(tbl[0][1])
        elif zp >= tbl[-1][0]:
            f_inf = float(tbl[-1][1])
        else:
            f_inf = float(tbl[-1][1])
            for i in range(len(tbl) - 1):
                z0, f0 = tbl[i]
                z1, f1 = tbl[i + 1]
                if z0 <= zp <= z1:
                    t = (zp - z0) / max(1.0, float(z1 - z0))
                    f_inf = f0 + t * (f1 - f0)
                    break

        w_zoom = min(1.0, max(0.0, (zp - 0x2CC0) / float(0x4000 - 0x2CC0)))
        d_m = max(140.0, float(dist_km) * 1000.0) if (dist_km is not None and dist_km > 0.05) else 4500.0
        diopter_delta = ((105000.0 / d_m) - 36.0) * w_zoom
        return max(0x0F80, min(0x8800, int(round(f_inf + diopter_delta))))

    @staticmethod
    def dist_km_to_sony_focus_reg(dist_km: Optional[float]) -> int:
        return SonySRGA40Driver.compute_sony_focus_reg_for_zoom_and_dist(0x4000, dist_km)

    def _sync_target_focus(
        self,
        z_pos: int,
        tilt_syn_deg: float,
        dist_km: Optional[float],
        target_kind: str = "",
    ):
        """
        Syncs calibrated focus AFTER any active `AbsoluteZoom` move finishes settling.
        Unlocks `FocusNearLimit=9000` in manual mode so wide-angle cam values (`0x6000` at 1x)
        are never clamped by Sony's firmware near-limit wall!
        """
        if getattr(self, "celestial_mode", None):
            return
        now = time.time()
        if now < getattr(self, "zoom_settling_until_ts", 0.0):
            return
        elev = solar_elevation_deg_utc(now, self.lat_deg, self.lon_deg)
        use_manual_infinity = (
            z_pos <= 0x2CC0
            or elev < -2.0
            or target_kind in ("aircraft", "helicopter")
            or tilt_syn_deg > 1.2
            or (dist_km is not None and dist_km >= 1.6)
        )
        if use_manual_infinity:
            want_f = self.compute_sony_focus_reg_for_zoom_and_dist(z_pos, dist_km)
            # Periodically read back hardware `AbsolutePTZF` after zoom moves so if Sony ignored an
            # `AbsoluteFocus` command while its zoom motor was still settling, we immediately correct it.
            if (
                getattr(self, "_focus_verify_due_ts", 0.0) > 0.0
                and now >= self._focus_verify_due_ts
                and (now - self.last_focus_send_ts) >= 0.65
            ):
                self._focus_verify_due_ts = 0.0
                self.query_ptzf()
            need_mode = self.focus_mode != "manual_lock"
            need_val = (
                self.pending_focus_sync
                or self.last_focus_abs is None
                or abs(want_f - self.last_focus_abs) >= 16
            )
            if need_mode:
                if self._send_cgi("ptzf.cgi", "FocusNearLimit=9000&FocusMode=manual"):
                    self.focus_mode = "manual_lock"
            if self.focus_mode == "manual_lock" and (need_mode or (need_val and (now - self.last_focus_send_ts) >= 0.40)):
                if self._send_cgi("ptzf.cgi", f"AbsoluteFocus={want_f:04x}"):
                    self.last_focus_abs = want_f
                    self.last_focus_send_ts = now
                    self.pending_focus_sync = False
                    self._focus_verify_due_ts = now + 1.80
        else:
            if self.focus_mode != "sony_af_near1300" and (now - self.last_focus_send_ts) >= 1.0:
                if self._send_cgi(
                    "ptzf.cgi",
                    f"FocusNearLimit={self.focus_near_limit_hex}&AFSensitivity=low&FocusMode=auto",
                ):
                    self.focus_mode = "sony_af_near1300"
                    self.last_focus_send_ts = now
                    self.pending_focus_sync = False

    def compute_calibrated_focus_abs(
        self, z_pos: int, dist_km: Optional[float] = None, include_trim: bool = True
    ) -> int:
        base = self.dist_km_to_sony_focus_reg(dist_km)
        if include_trim:
            base += max(-512, min(512, int(self.focus_trim_offset)))
        return max(0x1000, min(0x3000, base))

    def set_focus_trim_offset(self, offset_ticks: int):
        self.focus_trim_offset = max(-512, min(512, int(round(offset_ticks))))

    def set_focus_abs(self, focus_abs: int, force: bool = False) -> bool:
        f_clamped = max(0x1000, min(0x4000, int(round(focus_abs))))
        if self.focus_mode != "manual_lock":
            self._send_cgi("ptzf.cgi", "FocusMode=manual")
            self.focus_mode = "manual_lock"
        ok = self._send_cgi("ptzf.cgi", f"AbsoluteFocus={f_clamped:04x}")
        if ok:
            self.last_focus_abs = f_clamped
            self.last_focus_send_ts = time.time()
        return ok

    def _apply_solar_exposure_mode(self, force: bool = False, target_kind: str = ""):
        elev = solar_elevation_deg_utc(time.time(), self.lat_deg, self.lon_deg)
        is_night = elev < -2.5
        is_dark_mover = target_kind in ("vessel", "aircraft", "helicopter")
        want_tag = ("sony_night_mover_g8" if is_dark_mover else "sony_night_scenic_g7") if is_night else "sony_day"
        if not force and self.exposure_profile == want_tag:
            return
        if is_night:
            # Clean Goldilocks Exmor R night midpoint (between Nov 20's too-dark `Gain=6`/+15 dB `VE=off`
            # and factory/slow-shutter noise): lock shutter to 1/30s (`ExposureMaxExposureTime=14`,
            # `AutoSlowShutterMode=off` so Sony never drops to 1/4s), use `ExposureMaxGain=8` (+21 dB)
            # on dark river boats & aircraft (`7` = +18 dB on lit skyline/landmarks), plus gentle level-1
            # `VisibilityEnhancer=on (1,1,0)` and `-0.5 EV` (`ExposureCompensation=6`) so night water & sky
            # stay deep low-noise black while boat hulls and aircraft fuselages remain clearly visible:
            self.exposure_comp = 6
            max_gain = 8 if is_dark_mover else 7
            qs = (
                f"ExposureMode=auto&HighSensitivityMode=off&AutoSlowShutterMode=off"
                f"&ExposureMaxExposureTime=14&ExposureMinExposureTime=29&ExposureMaxGain={max_gain}"
                f"&LowLightBasisBrightness=off"
                f"&VisibilityEnhancer=on&VisibilityEnhancerLevel=1,1,0&ChromaSuppressLevel=2"
                f"&BacklightCompensationMode=off&FlickerReduction=on"
                f"&NoiseReductionMode=simple&NoiseReduction=5&ExposureCompensation=6"
            )
            self._send_cgi("imaging.cgi", qs)
            self.exposure_profile = want_tag
        else:
            # Daylight / twilight: Exmor R VisibilityEnhancer tone-mapping + 1/60s..1/125s smooth broadcast shutter
            # (`ExposureMinExposureTime=18` avoids 1/6,000s staccato pan strobing & stops down iris into sweet spot)
            self.exposure_comp = 7
            self._send_cgi(
                "imaging.cgi",
                "ExposureMode=auto&HighSensitivityMode=off&AutoSlowShutterMode=off"
                "&ExposureMaxExposureTime=16&ExposureMinExposureTime=18"
                "&ExposureMaxGain=9&LowLightBasisBrightness=off&BacklightCompensationMode=off"
                "&VisibilityEnhancer=on&VisibilityEnhancerLevel=3,2,1&ChromaSuppressLevel=1"
                "&FlickerReduction=on&NoiseReductionMode=simple&NoiseReduction=3&ExposureCompensation=7",
            )
            self.exposure_profile = "sony_day"
        self.last_exposure_sync_ts = time.time()

    def configure_outdoor_window_defaults(self):
        """
        Configures the Sony SRG-A40 for 24/7 through-window Hudson River operation:
          1. Enables 4K Clear Image Zoom (`ZoomMode=clearimage` -> 1x..30x at 2160p, 1x..40x at 1080p)
          2. Sets hardware `FocusNearLimit=3000` (20m) + `AFSensitivity=low` so window glass (<0.5m)
             is physically excluded from Sony's hybrid autofocus range!
          3. Applies solar-aware full-color Exmor R exposure (`VisibilityEnhancer=on` in daylight,
             clean `-1.5 EV` low-noise dark-sky exposure at night).
        """
        if not self.reachable or self.needs_password_init or self.cgi.auth_failed:
            return
        ok1 = self._send_cgi("ptzf.cgi", "ZoomMode=clearimage")
        if not ok1:
            self._send_cgi("imaging.cgi", "ZoomMode=clearimage")
        self._send_cgi(
            "ptzf.cgi",
            f"FocusNearLimit={self.focus_near_limit_hex}&AFSensitivity=low&FocusMode=auto&SlowPanTiltMode=on&ZoomSpeedScale=45",
        )
        self.focus_mode = "sony_af_20m_limit"
        self._send_cgi("ircf.cgi", "IrCutFilterMode=manual&IrCutFilterManual=off")
        self._apply_solar_exposure_mode(force=True)
        self.query_ptzf()
        self.defaults_configured = True

    def lock_focus_infinity(self):
        self.configure_outdoor_window_defaults()

    def trigger_one_push_focus(self):
        if self._send_cgi("ptzf.cgi", f"FocusNearLimit={self.focus_near_limit_hex}&Move=onepushaf,0"):
            self.focus_mode = "one_push"

    def set_auto_focus(self, enabled: bool = True):
        if enabled:
            if self._send_cgi(
                "ptzf.cgi",
                f"FocusNearLimit={self.focus_near_limit_hex}&AFSensitivity=low&FocusMode=auto",
            ):
                self.focus_mode = "sony_af_20m_limit"
        else:
            if self._send_cgi("ptzf.cgi", "FocusMode=manual"):
                self.focus_mode = "manual_lock"

    def sync_outdoor_exposure(
        self,
        force: bool = False,
        pan_syn_deg: Optional[float] = None,
        tilt_syn_deg: Optional[float] = None,
        zoom_x: Optional[float] = None,
        allow_before_zoom: bool = False,
    ):
        if getattr(self, "celestial_mode", None):
            return
        now = time.time()
        if not force and (now - self.last_exposure_sync_ts) < 45.0:
            return
        if not self.probe_connection(force=False):
            return
        if not self.defaults_configured or force:
            self.configure_outdoor_window_defaults()
            return
        self._apply_solar_exposure_mode(force=False)
        self.last_exposure_sync_ts = now

    def set_celestial_exposure(self, body_id: str, force: bool = False):
        """
        Configures non-saturating lunar / planetary astrophotography shutter & infinity focus
        (`0x106d`) when tracking the Moon, Saturn, Jupiter, or Venus at night/twilight.
        """
        prev = getattr(self, "celestial_mode", None)
        if prev == body_id and not force:
            return
        if not self.probe_connection(force=False):
            return
        self.celestial_mode = body_id
        # Lock focus to calibrated astronomical infinity (0x106d) so AF never hunts on empty black sky
        self._send_cgi("ptzf.cgi", "FocusMode=manual")
        self._send_cgi("ptzf.cgi", "AbsoluteFocus=106d")
        self.focus_mode = "celestial_infinity_106d"
        self.last_focus_abs = 0x106D

        elev = solar_elevation_deg_utc(time.time(), self.lat_deg, self.lon_deg)
        if elev > 0.0:
            # Daytime blue sky Moon / Venus: keep auto exposure slightly stopped down so sky doesn't clip
            self._send_cgi(
                "imaging.cgi",
                "ExposureMode=auto&ExposureCompensation=5&VisibilityEnhancer=off&HighSensitivityMode=off",
            )
            self.exposure_profile = f"celestial_day_{body_id}"
        else:
            # Night / twilight calibrated unsaturated planetary & lunar shutter presets
            presets = {
                "moon": "ExposureMode=manual&ExposureGain=1&ExposureExposureTime=22&HighSensitivityMode=off&VisibilityEnhancer=off",
                "venus": "ExposureMode=manual&ExposureGain=1&ExposureExposureTime=23&HighSensitivityMode=off&VisibilityEnhancer=off",
                "jupiter": "ExposureMode=manual&ExposureGain=2&ExposureExposureTime=19&HighSensitivityMode=off&VisibilityEnhancer=off",
                "saturn": "ExposureMode=manual&ExposureGain=4&ExposureExposureTime=16&HighSensitivityMode=off&VisibilityEnhancer=off",
            }
            qs = presets.get(
                body_id,
                "ExposureMode=manual&ExposureGain=4&ExposureExposureTime=16&HighSensitivityMode=off&VisibilityEnhancer=off",
            )
            self._send_cgi("imaging.cgi", qs)
            self.exposure_profile = f"celestial_{body_id}"
        self.last_exposure_sync_ts = time.time()

    def clear_celestial_exposure(self):
        if not getattr(self, "celestial_mode", None):
            return
        self.celestial_mode = None
        if self.reachable and not self.needs_password_init and not self.cgi.auth_failed:
            self._send_cgi("ptzf.cgi", f"FocusNearLimit={self.focus_near_limit_hex}&AFSensitivity=low&FocusMode=auto")
            self.focus_mode = "sony_af_20m_limit"
            self._apply_solar_exposure_mode(force=True)

    def apply_luma_feedback(self, mean_luma: float, clipped_frac: float) -> bool:
        """
        Gently trims Sony's hardware `ExposureCompensation` [0..14, default 7 = 0 EV] in daylight
        when extreme solar horizon/water glare clips >1.8% of pixels or washes out the frame.
        """
        if getattr(self, "celestial_mode", None):
            return False
        if not self.reachable or self.needs_password_init or self.cgi.auth_failed:
            return False
        elev = solar_elevation_deg_utc(time.time(), self.lat_deg, self.lon_deg)
        if elev < -2.5:
            self._apply_solar_exposure_mode(force=False)
            return False
        if mean_luma < 18.0 and elev > 2.0:
            return False

        new_comp = self.exposure_comp
        if clipped_frac > 0.018 or mean_luma > 154.0:
            step = 2 if (clipped_frac > 0.08 or mean_luma > 185.0) else 1
            new_comp = max(2, self.exposure_comp - step)
        elif mean_luma < 92.0 and clipped_frac < 0.004:
            new_comp = min(10, self.exposure_comp + 1)

        if new_comp != self.exposure_comp:
            if self._send_cgi("imaging.cgi", f"ExposureCompensation={new_comp}"):
                self.exposure_comp = new_comp
                self.exposure_profile = f"sony_day_ve_comp{new_comp}"
                self.last_exposure_sync_ts = time.time()
                return True
        return False

    def compute_hw_angles(
        self,
        pan_syn_deg: float,
        tilt_syn_deg: float,
        zoom_x: float = 30.0,
        dist_km: Optional[float] = None,
    ) -> Tuple[float, float]:
        return project_synoptic_to_ptz_gimbal(
            pan_syn_deg=pan_syn_deg,
            tilt_syn_deg=tilt_syn_deg,
            yaw_offset_deg=self.pan_offset_deg,
            pitch_offset_deg=self.tilt_offset_deg,
            roll_offset_deg=self.roll_offset_deg,
            pan_scale=self.pan_scale,
            tilt_scale=self.tilt_scale,
            zoom_x=zoom_x,
            collimation_pan_10x_deg=self.collimation_pan_10x_deg,
            collimation_tilt_10x_deg=self.collimation_tilt_10x_deg,
            dist_km=dist_km,
            baseline_cam_m=self.baseline_cam_m,
        )

    def stop_motion(self):
        """Immediately halts any active continuous `PanTiltMove` velocity drive."""
        if self.vel_step_x != 0 or self.vel_step_y != 0:
            self._send_cgi("ptzf.cgi", "PanTiltMove=stop,0,0")
            self.vel_step_x = 0
            self.vel_step_y = 0

    def move_raw_hardware_deg(self, pan_hw_deg: float, tilt_hw_deg: float, speed: int = 24) -> Tuple[float, float]:
        self.stop_motion()
        adj_pan = max(-169.5, min(169.5, float(pan_hw_deg)))
        adj_tilt = max(-19.8, min(89.5, float(tilt_hw_deg)))
        p_hex, p_steps, p_exact = self._deg_to_hex4(adj_pan)
        t_hex, t_steps, t_exact = self._deg_to_hex4(adj_tilt)
        spd = max(1, min(24, int(speed)))
        if self._send_cgi("ptzf.cgi", f"AbsolutePanTilt={p_hex},{t_hex},{spd}"):
            self.last_pan_steps = p_steps
            self.last_tilt_steps = t_steps
            self.last_pan_cmd_deg = p_exact
            self.last_tilt_cmd_deg = t_exact
            now_t = time.time()
            self.last_pt_send_ts = now_t
            self.last_large_slew_ts = now_t
        return p_exact, t_exact

    def wait_pantilt_settled(self, target_pan_deg: float, target_tilt_deg: float, timeout: float = 4.5):
        t0 = time.time()
        while (time.time() - t0) < timeout:
            time.sleep(0.16)
            kv = self.query_ptzf()
            if not kv:
                continue
            st_parts = kv.get("PtzfStatus", "").split(",")
            aptzf = kv.get("AbsolutePTZF", "").split(",")
            if len(aptzf) >= 2:
                try:
                    cp = self._hex_to_deg(aptzf[0])
                    ct = self._hex_to_deg(aptzf[1])
                    pt_idle = len(st_parts) >= 2 and st_parts[0] == "idle" and st_parts[1] == "idle"
                    if pt_idle and abs(cp - target_pan_deg) <= 0.12 and abs(ct - target_tilt_deg) <= 0.12:
                        time.sleep(0.18)
                        return
                except ValueError:
                    pass
        time.sleep(0.25)

    def set_zoom_register(
        self, z_pos: int, force: bool = False, dist_km: Optional[float] = None, sync_focus: bool = True
    ):
        z_clamped = max(0, min(self.max_zoom_reg, int(round(z_pos))))
        if dist_km is not None:
            self.last_dist_km = float(dist_km)
        now = time.time()
        min_z_step = 32 if (z_clamped == 0 or z_clamped >= self.max_zoom_reg - 16) else 180
        target_changed = self.last_zoom_pos is None or abs(z_clamped - self.last_zoom_pos) >= min_z_step
        periodic_refresh = (now - self.last_zoom_send_ts) >= 15.0
        if not force and not target_changed and not periodic_refresh:
            return
        prev_z = self.last_zoom_pos if self.last_zoom_pos is not None else z_clamped
        if self._send_cgi("ptzf.cgi", f"AbsoluteZoom={z_clamped:04x}"):
            if target_changed or force:
                self.last_zoom_target_change_ts = now
                dz = abs(z_clamped - prev_z)
                # At ZoomSpeedScale=45, 0x0000->0x5556 takes ~3.6s; wait for motor stop so Sony accepts AbsoluteFocus
                self.zoom_settling_until_ts = now + min(3.9, max(0.60, dz / 6000.0 + 0.40))
                self.pending_focus_sync = True
            self.last_zoom_pos = z_clamped
            self.actual_zoom_pos = z_clamped
            eff_hfov = self.register_to_hfov_deg(z_clamped)
            self.last_zoom_x = self.zoom_hfov_table[0][1] / max(0.5, eff_hfov)
            self.last_zoom_send_ts = now

    def wait_zoom_settled(self, z_pos: int, timeout: float = 5.5) -> int:
        z_clamped = max(0, min(self.max_zoom_reg, int(round(z_pos))))
        self.set_zoom_register(z_clamped, force=True, sync_focus=False)
        t0 = time.time()
        last_zv = z_clamped
        while (time.time() - t0) < timeout:
            time.sleep(0.16)
            kv = self.query_ptzf()
            if not kv:
                continue
            st_parts = kv.get("PtzfStatus", "").split(",")
            aptzf = kv.get("AbsolutePTZF", "").split(",")
            if len(aptzf) >= 3:
                try:
                    zv = int(aptzf[2], 16)
                    last_zv = zv
                    z_idle = len(st_parts) >= 3 and st_parts[2] == "idle"
                    if abs(zv - z_clamped) <= 48 and z_idle:
                        time.sleep(0.14)
                        return zv
                except ValueError:
                    pass
        return last_zv

    def wait_focus_settled(self, focus_abs: int, timeout: float = 3.0) -> int:
        f_clamped = max(0x1000, min(0x4000, int(round(focus_abs))))
        self.set_focus_abs(f_clamped, force=True)
        t0 = time.time()
        last_fv = f_clamped
        while (time.time() - t0) < timeout:
            time.sleep(0.14)
            kv = self.query_ptzf()
            if not kv:
                continue
            st_parts = kv.get("PtzfStatus", "").split(",")
            aptzf = kv.get("AbsolutePTZF", "").split(",")
            if len(aptzf) >= 4:
                try:
                    fv = int(aptzf[3], 16)
                    last_fv = fv
                    f_idle = len(st_parts) >= 4 and st_parts[3] == "idle"
                    if abs(fv - f_clamped) <= 64 and f_idle:
                        time.sleep(0.12)
                        return fv
                except ValueError:
                    pass
        return last_fv

    def set_zoom(self, zoom_x: float, force: bool = False, dist_km: Optional[float] = None):
        z_pos = self.zoom_x_to_register(zoom_x)
        self.set_zoom_register(z_pos, force=force, dist_km=dist_km)
        self.last_zoom_x = max(1.0, min(self.max_zoom_x, float(zoom_x)))

    # Empirically calibrated steady-state angular velocity (deg/s) for Sony SRG-A40 `PanTiltMove`
    # with `SlowPanTiltMode=on` (measured directly against `AbsolutePTZF` 1/144.89° optical encoders):
    SONY_VEL_TABLE: Tuple[Tuple[int, float], ...] = (
        (0, 0.000),
        (1, 0.521),
        (2, 0.692),
        (3, 0.937),
        (4, 1.128),
        (5, 1.313),
        (6, 1.569),
        (7, 2.001),
        (8, 2.313),
        (9, 2.778),
        (10, 3.168),
        (11, 3.680),
        (12, 4.205),
        (13, 4.800),
        (14, 5.394),
        (15, 6.080),
        (16, 6.774),
        (18, 8.800),
        (20, 11.50),
    )

    @classmethod
    def _vel_step_to_dps(cls, signed_step: int) -> float:
        if signed_step == 0:
            return 0.0
        mag = abs(int(signed_step))
        sgn = 1.0 if signed_step > 0 else -1.0
        best_dps = 0.521
        best_d = 999
        for st, dps in cls.SONY_VEL_TABLE:
            if abs(st - mag) < best_d:
                best_d = abs(st - mag)
                best_dps = dps
        return sgn * best_dps

    def _record_pose_anchor(self, ts: float, pan_hw: float, tilt_hw: float, hfov_deg: float):
        with self.pose_lock:
            self._pose_anchor_ts = ts
            self._pose_anchor_pan = pan_hw
            self._pose_anchor_tilt = tilt_hw
            self.pose_history.append((ts, pan_hw, tilt_hw, hfov_deg, ts < self.slew_until_ts))

    def _advance_pose_estimate(self, ts: float):
        with self.pose_lock:
            z_reg = self.actual_zoom_pos if self.actual_zoom_pos is not None else (self.last_zoom_pos or 0)
            hfov = self.register_to_hfov_deg(z_reg)
            if self._pose_anchor_ts <= 0.0:
                p0 = self.actual_pan_deg if self.actual_pan_deg is not None else self.last_pan_cmd_deg
                t0 = self.actual_tilt_deg if self.actual_tilt_deg is not None else self.last_tilt_cmd_deg
                self._pose_anchor_ts = ts
                self._pose_anchor_pan = p0
                self._pose_anchor_tilt = t0
            else:
                dt_p = max(0.0, min(2.0, ts - self._pose_anchor_ts))
                vx = self._vel_step_to_dps(self.vel_step_x)
                vy = self._vel_step_to_dps(self.vel_step_y)
                self._pose_anchor_pan = max(-169.5, min(169.5, self._pose_anchor_pan + vx * dt_p))
                self._pose_anchor_tilt = max(-19.8, min(89.5, self._pose_anchor_tilt + vy * dt_p))
                self._pose_anchor_ts = ts
            self.pose_history.append(
                (ts, self._pose_anchor_pan, self._pose_anchor_tilt, hfov, ts < self.slew_until_ts)
            )

    def get_estimated_hw_pose(self, at_ts: Optional[float] = None) -> Tuple[float, float, float, bool]:
        """
        Returns `(pan_hw_deg, tilt_hw_deg, hfov_deg, was_slewing)` interpolated at `at_ts`
        for latency-compensated Inertial Line-of-Sight (LOS) visual target reconstruction.
        """
        t_q = time.time() if at_ts is None else float(at_ts)
        with self.pose_lock:
            z_reg = self.actual_zoom_pos if self.actual_zoom_pos is not None else (self.last_zoom_pos or 0)
            cur_hfov = self.register_to_hfov_deg(z_reg)
            if not self.pose_history:
                p0 = self.actual_pan_deg if self.actual_pan_deg is not None else self.last_pan_cmd_deg
                t0 = self.actual_tilt_deg if self.actual_tilt_deg is not None else self.last_tilt_cmd_deg
                return p0, t0, cur_hfov, (t_q < self.slew_until_ts)

            hist = list(self.pose_history)
            if t_q <= hist[0][0]:
                _, p, t, hf, sl = hist[0]
                return p, t, hf, sl
            if t_q >= hist[-1][0]:
                ts_last, p, t, hf, sl = hist[-1]
                dt_ext = max(0.0, min(1.0, t_q - ts_last))
                vx = self._vel_step_to_dps(self.vel_step_x)
                vy = self._vel_step_to_dps(self.vel_step_y)
                return p + vx * dt_ext, t + vy * dt_ext, cur_hfov, (sl or t_q < self.slew_until_ts)

            for i in range(len(hist) - 1):
                t0, p0, tl0, hf0, sl0 = hist[i]
                t1, p1, tl1, hf1, sl1 = hist[i + 1]
                if t0 <= t_q <= t1:
                    f = (t_q - t0) / max(1e-5, t1 - t0)
                    return (
                        p0 + f * (p1 - p0),
                        tl0 + f * (tl1 - tl0),
                        hf0 + f * (hf1 - hf0),
                        bool(sl0 or sl1),
                    )
            _, p, t, hf, sl = hist[-1]
            return p, t, hf, sl

    @classmethod
    def _dps_to_signed_vel_step(
        cls,
        want_dps: float,
        prev_step: int = 0,
        forbid_stop: bool = False,
        fine_center_trim: bool = False,
    ) -> int:
        """
        Maps desired signed angular rate (deg/s) onto Sony SRG-A40 `PanTiltMove` steps [-20..+20]
        with strong hysteresis (`0.16°/s`) and optional `forbid_stop` so active continuous pursuit
        never drops to `0` (`stop`) mid-shot.
        """
        mag = abs(float(want_dps))
        if not forbid_stop:
            if fine_center_trim:
                stop_thresh = 0.18 if prev_step != 0 else 0.24
            else:
                stop_thresh = 0.22 if prev_step != 0 else 0.44
            if mag < stop_thresh:
                return 0
        sgn = 1 if want_dps >= 0.0 else -1
        if mag < 0.02 and forbid_stop and prev_step != 0:
            sgn = 1 if prev_step > 0 else -1
        best_step = 1
        best_cost = 999.0
        for step, table_dps in cls.SONY_VEL_TABLE:
            if step == 0:
                continue
            cost = abs(mag - table_dps)
            # Strong stickiness bonus for keeping the currently running motor step uninterrupted
            if sgn * step == prev_step:
                cost -= 0.165
            if cost < best_cost:
                best_cost = cost
                best_step = step
        return sgn * best_step

    def _send_pantilt_velocity_steps(self, sx: int, sy: int) -> bool:
        """
        Sends `PanTiltMove=<dir>,<|sx|>,<|sy|>` ONLY when `(sx, sy)` actually changes, allowing
        the SRG-A40 gimbal to run completely uninterrupted between ~1 Hz course/speed checks.
        """
        sx = max(-20, min(20, int(sx)))
        sy = max(-20, min(20, int(sy)))
        if sx == self.vel_step_x and sy == self.vel_step_y:
            return True
        if sx == 0 and sy == 0:
            qs = "PanTiltMove=stop,0,0"
        else:
            direction = (
                "right" if (sy == 0 and sx > 0)
                else "left" if (sy == 0 and sx < 0)
                else "up" if (sx == 0 and sy > 0)
                else "down" if (sx == 0 and sy < 0)
                else "up-right" if (sx > 0 and sy > 0)
                else "up-left" if (sx < 0 and sy > 0)
                else "down-right" if (sx > 0 and sy < 0)
                else "down-left"
            )
            qs = f"PanTiltMove={direction},{abs(sx)},{abs(sy)}"
        if self._send_cgi("ptzf.cgi", qs):
            now_v = time.time()
            self._advance_pose_estimate(now_v)
            self.vel_step_x = sx
            self.vel_step_y = sy
            self.last_pt_send_ts = now_v
            return True
        return False

    def _issue_absolute_slew(self, target_hw_pan: float, target_hw_tilt: float, now: float) -> bool:
        adj_pan = max(-169.5, min(169.5, float(target_hw_pan)))
        adj_tilt = max(-19.8, min(89.5, float(target_hw_tilt)))
        p_hex, p_steps, p_exact = self._deg_to_hex4(adj_pan)
        t_hex, t_steps, t_exact = self._deg_to_hex4(adj_tilt)
        ref_p = self.actual_pan_deg if self.actual_pan_deg is not None else self.last_pan_cmd_deg
        ref_t = self.actual_tilt_deg if self.actual_tilt_deg is not None else self.last_tilt_cmd_deg
        deg_jump = (
            math.hypot(p_exact - ref_p, t_exact - ref_t)
            if self.last_pan_steps is not None
            else 15.0
        )
        if self.last_pan_steps is None or deg_jump > 5.0:
            spd, est_rate = 18, 8.8
        elif deg_jump > 1.8:
            spd, est_rate = 14, 5.4
        else:
            spd, est_rate = 10, 3.1
        if self._send_cgi("ptzf.cgi", f"AbsolutePanTilt={p_hex},{t_hex},{spd}"):
            now_pt = time.time()
            self.vel_step_x = 0
            self.vel_step_y = 0
            self.last_pan_steps = p_steps
            self.last_tilt_steps = t_steps
            self.last_pan_cmd_deg = p_exact
            self.last_tilt_cmd_deg = t_exact
            self.actual_pan_deg = p_exact
            self.actual_tilt_deg = t_exact
            self.last_pt_send_ts = now_pt
            self.slew_until_ts = now_pt + min(2.3, max(0.42, deg_jump / est_rate + 0.24))
            self._needs_post_slew_query = True
            z_reg = self.actual_zoom_pos if self.actual_zoom_pos is not None else (self.last_zoom_pos or 0)
            self._record_pose_anchor(now_pt, p_exact, t_exact, self.register_to_hfov_deg(z_reg))
            if deg_jump >= 2.0:
                self.last_large_slew_ts = now_pt
            return True
        return False

    def move_absolute(
        self,
        pan_deg: float,
        tilt_deg: float,
        zoom_x: float,
        pan_dps: float = 0.0,
        tilt_dps: float = 0.0,
        dist_km: Optional[float] = None,
        target_kind: str = "",
        force: bool = False,
        visual_guidance: Optional[dict] = None,
    ):
        now = time.time()
        if not self.probe_connection(force=False):
            return
        if not self.defaults_configured:
            self.configure_outdoor_window_defaults()
        else:
            self._apply_solar_exposure_mode(force=False, target_kind=target_kind)

        self._advance_pose_estimate(now)

        # Project instantaneous RF pose and +1.0s ahead RF pose into Sony gimbal angles & angular rates
        rf_now_p, rf_now_t = self.compute_hw_angles(pan_deg, tilt_deg, zoom_x=zoom_x, dist_km=dist_km)
        rf_1s_p, rf_1s_t = self.compute_hw_angles(
            pan_deg + pan_dps, tilt_deg + tilt_dps, zoom_x=zoom_x, dist_km=dist_km
        )
        inst_hw_pan_dps = rf_1s_p - rf_now_p
        inst_hw_tilt_dps = rf_1s_t - rf_now_t

        vis_locked = bool(visual_guidance and visual_guidance.get("locked"))
        vis_tid = str(visual_guidance.get("target_id") or "") if visual_guidance else ""
        vis_recenter_seq = int(visual_guidance.get("recenter_seq", 0)) if visual_guidance else 0
        if vis_locked and visual_guidance:
            raw_now_p = rf_now_p + float(visual_guidance.get("bias_pan_hw", 0.0))
            raw_now_t = rf_now_t + float(visual_guidance.get("bias_tilt_hw", 0.0))
            inst_hw_pan_dps += float(visual_guidance.get("bias_pan_dps", 0.0))
            inst_hw_tilt_dps += float(visual_guidance.get("bias_tilt_dps", 0.0))
        else:
            raw_now_p, raw_now_t = rf_now_p, rf_now_t

        tid_changed = bool(vis_tid and vis_tid != self._active_track_id)
        if vis_tid:
            self._active_track_id = vis_tid
        elif not visual_guidance:
            self._active_track_id = None

        target_jumped = (
            tid_changed
            or self.last_target_hw_pan is None
            or self.last_target_hw_tilt is None
            or math.hypot(rf_now_p - self.last_target_hw_pan, rf_now_t - self.last_target_hw_tilt) > 3.5
        )
        if target_jumped:
            self._last_vis_recenter_seq = vis_recenter_seq

        if target_jumped or self.last_vel_ema_ts <= 0.0:
            self.smooth_hw_pan_dps = inst_hw_pan_dps
            self.smooth_hw_tilt_dps = inst_hw_tilt_dps
        else:
            dt_ema = max(0.02, min(2.0, now - self.last_vel_ema_ts))
            tau = 0.60 if vis_locked else 1.8
            alpha = 1.0 - math.exp(-dt_ema / tau)
            self.smooth_hw_pan_dps += alpha * (inst_hw_pan_dps - self.smooth_hw_pan_dps)
            self.smooth_hw_tilt_dps += alpha * (inst_hw_tilt_dps - self.smooth_hw_tilt_dps)
        self.last_vel_ema_ts = now
        self.last_target_hw_pan = rf_now_p
        self.last_target_hw_tilt = rf_now_t

        hw_v = math.hypot(self.smooth_hw_pan_dps, self.smooth_hw_tilt_dps)
        hfov_cur = max(2.67, self.zoom_hfov_table[0][1] / max(1.0, float(zoom_x)))
        vfov_cur = math.degrees(2.0 * math.atan(math.tan(math.radians(hfov_cur * 0.5)) * (9.0 / 16.0)))
        want_z_pos = self.zoom_x_to_register(zoom_x)

        # 1) Let an in-flight acquisition slew finish cleanly unless switching targets
        if not force and not target_jumped and now < self.slew_until_ts:
            self.set_zoom(zoom_x, force=force, dist_km=dist_km)
            self._sync_target_focus(want_z_pos, tilt_deg, dist_km, target_kind=target_kind)
            return

        # Anchor exact hardware stepper registers right when an acquisition slew finishes
        if self._needs_post_slew_query and now >= self.slew_until_ts:
            self._needs_post_slew_query = False
            self.query_ptzf()

        # 2) Initial target acquisition / target switch slew (predictive intercept)
        if force or target_jumped or self.last_pan_steps is None:
            self.stop_motion()
            ref_p = self.actual_pan_deg if self.actual_pan_deg is not None else self.last_pan_cmd_deg
            ref_t = self.actual_tilt_deg if self.actual_tilt_deg is not None else self.last_tilt_cmd_deg
            approx_jump = math.hypot(raw_now_p - ref_p, raw_now_t - ref_t) if self.last_pan_steps is not None else 15.0
            est_slew_s = min(2.2, max(0.45, approx_jump / 8.5 + 0.25))
            if hw_v >= 0.44:
                aim_p = raw_now_p + self.smooth_hw_pan_dps * est_slew_s
                aim_t = raw_now_t + self.smooth_hw_tilt_dps * est_slew_s
            elif hw_v >= 0.015:
                lead_s = min(5.5, (0.24 * hfov_cur) / max(0.015, hw_v))
                aim_p = raw_now_p + self.smooth_hw_pan_dps * lead_s
                aim_t = raw_now_t + self.smooth_hw_tilt_dps * lead_s
            else:
                aim_p, aim_t = raw_now_p, raw_now_t
            self._issue_absolute_slew(aim_p, aim_t, now)
            self.set_zoom(zoom_x, force=force, dist_km=dist_km)
            self._sync_target_focus(want_z_pos, tilt_deg, dist_km, target_kind=target_kind)
            return

        # 2b) Visual lock centering OR post-occlusion AIS/ADS-B re-acquisition slew trigger
        vis_phase = str(visual_guidance.get("phase") or "") if visual_guidance else ""
        need_vis_recenter = (vis_locked or vis_phase == "reacquiring_rf") and (
            vis_recenter_seq != self._last_vis_recenter_seq
        )
        if need_vis_recenter:
            self._last_vis_recenter_seq = vis_recenter_seq
            if hw_v < 0.44 or vis_phase == "reacquiring_rf":
                # Execute crisp centering / post-occlusion AIS re-find slew!
                self.stop_motion()
                lead_s = min(4.8, (0.20 * hfov_cur) / max(0.015, hw_v)) if hw_v >= 0.02 else 0.0
                if vis_phase == "reacquiring_rf" and hw_v >= 0.44:
                    lead_s = 0.85
                self._issue_absolute_slew(
                    raw_now_p + self.smooth_hw_pan_dps * lead_s,
                    raw_now_t + self.smooth_hw_tilt_dps * lead_s,
                    now,
                )
                self.set_zoom(zoom_x, force=force, dist_km=dist_km)
                self._sync_target_focus(want_z_pos, tilt_deg, dist_km, target_kind=target_kind)
                return

        # 3) Non-stop Continuous Constant-Velocity Pursuit (`PanTiltMove`) for targets >= 0.44°/s
        if hw_v >= 0.44:
            need_handoff = self.vel_step_x == 0 and self.vel_step_y == 0
            check_interval = 0.55 if vis_locked else 1.30
            if need_handoff or need_vis_recenter or (now - self.last_vel_check_ts) >= check_interval:
                self.last_vel_check_ts = now
                self.query_ptzf()
                act_p = self.actual_pan_deg if self.actual_pan_deg is not None else self.last_pan_cmd_deg
                act_t = self.actual_tilt_deg if self.actual_tilt_deg is not None else self.last_tilt_cmd_deg
                _, ps_act, _ = self._deg_to_hex4(act_p)
                _, ts_act, _ = self._deg_to_hex4(act_t)
                self.last_pan_steps = ps_act
                self.last_tilt_steps = ts_act
                self.last_pan_cmd_deg = act_p
                self.last_tilt_cmd_deg = act_t

                if abs(act_p) > 167.5 or act_t < -19.2 or act_t > 88.5:
                    self.stop_motion()
                else:
                    err_p = raw_now_p - act_p
                    err_t = raw_now_t - act_t
                    reacq_thresh = max(2.2 if vis_locked else 4.2, 0.44 * hfov_cur)
                    if math.hypot(err_p, err_t) > reacq_thresh:
                        # Target near/outside edge — crisp predictive intercept onto visual LOS!
                        self.stop_motion()
                        est_s = min(1.5, max(0.45, math.hypot(err_p, err_t) / 6.5 + 0.22))
                        self._issue_absolute_slew(
                            raw_now_p + self.smooth_hw_pan_dps * est_s,
                            raw_now_t + self.smooth_hw_tilt_dps * est_s,
                            now,
                        )
                    else:
                        kp = 0.55 if vis_locked else 0.22
                        want_p_dps = self.smooth_hw_pan_dps + kp * err_p
                        want_t_dps = self.smooth_hw_tilt_dps + kp * err_t
                        dom_pan = abs(self.smooth_hw_pan_dps) >= abs(self.smooth_hw_tilt_dps)
                        fine_p = vis_locked and abs(err_p) > 0.16 * hfov_cur
                        fine_t = vis_locked and abs(err_t) > 0.16 * vfov_cur
                        sx = self._dps_to_signed_vel_step(
                            want_p_dps, self.vel_step_x, forbid_stop=dom_pan, fine_center_trim=fine_p
                        )
                        sy = self._dps_to_signed_vel_step(
                            want_t_dps, self.vel_step_y, forbid_stop=not dom_pan, fine_center_trim=fine_t
                        )
                        self._send_pantilt_velocity_steps(sx, sy)
            self.set_zoom(zoom_x, force=force, dist_km=dist_km)
            self._sync_target_focus(want_z_pos, tilt_deg, dist_km, target_kind=target_kind)
            return

        # 4) Sub-floor boat / distant plane (< 0.44°/s, below SRG-A40's 0.52°/s physical motor floor):
        #    Visual centering places the boat upstream of center; only re-frames if drifting near frame margin.
        self.stop_motion()
        if hw_v >= 0.015 or vis_locked:
            drift_deg = math.hypot(raw_now_p - self.last_pan_cmd_deg, raw_now_t - self.last_tilt_cmd_deg)
            reframe_thresh = (0.32 * hfov_cur) if vis_locked else max(1.50, 0.47 * hfov_cur)
            min_gap_s = 3.2 if vis_locked else 8.0
            if drift_deg >= reframe_thresh and (now - self.last_pt_send_ts) >= min_gap_s:
                lead_s = min(5.5, (0.22 * hfov_cur) / max(0.015, hw_v)) if hw_v >= 0.02 else 0.0
                self._issue_absolute_slew(
                    raw_now_p + self.smooth_hw_pan_dps * lead_s,
                    raw_now_t + self.smooth_hw_tilt_dps * lead_s,
                    now,
                )
            self.set_zoom(zoom_x, force=force, dist_km=dist_km)
            self._sync_target_focus(want_z_pos, tilt_deg, dist_km, target_kind=target_kind)
            return

        # 5) Static landmark / manual harbor point
        adj_pan = max(-169.5, min(169.5, raw_now_p))
        adj_tilt = max(-19.8, min(89.5, raw_now_t))
        _, p_steps, _ = self._deg_to_hex4(adj_pan)
        _, t_steps, _ = self._deg_to_hex4(adj_tilt)
        dp = abs(p_steps - (self.last_pan_steps or 0))
        dt_st = abs(t_steps - (self.last_tilt_steps or 0))
        if (dp >= 2 or dt_st >= 2) and (now - self.last_pt_send_ts) >= 0.25:
            self._issue_absolute_slew(adj_pan, adj_tilt, now)

        self.set_zoom(zoom_x, force=force, dist_km=dist_km)
        self._sync_target_focus(want_z_pos, tilt_deg, dist_km, target_kind=target_kind)

    def move_celestial_hw(self, pan_hw_deg: float, tilt_hw_deg: float, zoom_x: float, force: bool = False):
        """
        Direct spherical Alt-Azimuth gimbal pointing & smooth sidereal tracking (15"/sec)
        for celestial bodies up to +85° elevation above the horizon.
        """
        now = time.time()
        if not self.probe_connection(force=False):
            return
        if not self.defaults_configured:
            self.configure_outdoor_window_defaults()
        self.stop_motion()

        adj_pan = max(-169.5, min(169.5, float(pan_hw_deg)))
        adj_tilt = max(-19.8, min(89.5, float(tilt_hw_deg)))
        p_hex, p_steps, p_exact = self._deg_to_hex4(adj_pan)
        t_hex, t_steps, t_exact = self._deg_to_hex4(adj_tilt)

        dp = abs(p_steps - (self.last_pan_steps or 0))
        dt_st = abs(t_steps - (self.last_tilt_steps or 0))
        if force or self.last_pan_steps is None or ((dp >= 2 or dt_st >= 2) and (now - self.last_pt_send_ts) >= 3.5):
            deg_jump = math.hypot(dp, dt_st) / self.SONY_STEPS_PER_DEG
            spd = 17 if (self.last_pan_steps is None or deg_jump > 5.0) else (13 if deg_jump > 1.5 else 1)
            if self._send_cgi("ptzf.cgi", f"AbsolutePanTilt={p_hex},{t_hex},{spd}"):
                self.last_pan_steps = p_steps
                self.last_tilt_steps = t_steps
                self.last_pan_cmd_deg = p_exact
                self.last_tilt_cmd_deg = t_exact
                self.last_pt_send_ts = time.time()
                if deg_jump >= 2.0:
                    self.last_large_slew_ts = self.last_pt_send_ts

        self.set_zoom(zoom_x, force=force, dist_km=384000.0)

    def move_velocity(self, pan_deg_per_s: float, tilt_deg_per_s: float, zoom_x: float):
        if not self.probe_connection(force=False):
            return
        sx = self._dps_to_signed_vel_step(pan_deg_per_s, self.vel_step_x)
        sy = self._dps_to_signed_vel_step(tilt_deg_per_s, self.vel_step_y)
        self._send_pantilt_velocity_steps(sx, sy)
        if abs(zoom_x - self.last_zoom_x) > 0.25:
            self.set_zoom(zoom_x)

    def get_state(self) -> dict:
        return {
            "enabled": True,
            "camera_model": "sony_a40",
            "model_name": self.model_name,
            "serial": self.serial_num,
            "firmware": self.firmware_ver,
            "transport": "sony_cgi_ethernet",
            "host": self.host,
            "visca_host": f"{self.host}:{self.port}",
            "reachable": self.reachable,
            "needs_password_init": self.needs_password_init,
            "auth_failed": self.cgi.auth_failed,
            "steps_per_deg": self.steps_per_deg,
            "pan_offset_deg": round(self.pan_offset_deg, 3),
            "tilt_offset_deg": round(self.tilt_offset_deg, 3),
            "roll_offset_deg": round(self.roll_offset_deg, 3),
            "pan_scale": round(self.pan_scale, 4),
            "tilt_scale": round(self.tilt_scale, 4),
            "max_zoom_x": self.max_zoom_x,
            "zoom_x": round(self.last_zoom_x, 2),
            "zoom_pos": self.actual_zoom_pos if self.actual_zoom_pos is not None else self.last_zoom_pos,
            "focus_abs": f"0x{self.last_focus_abs:04x}" if isinstance(self.last_focus_abs, int) else self.last_focus_abs,
            "focus_trim": self.focus_trim_offset,
            "dist_km": round(self.last_dist_km, 2) if self.last_dist_km is not None else None,
            "exposure_profile": self.exposure_profile,
            "exp_comp": self.exposure_comp,
            "ambient_ev_bias": round(self.ambient_ev_bias, 2),
            "local_ev_trim": round(self.local_ev_trim, 2),
            "pan_steps": self.last_pan_steps,
            "tilt_steps": self.last_tilt_steps,
            "vel_steps": [self.vel_step_x, self.vel_step_y],
            "sent_packets": self.sent_packets,
            "last_error": self.last_error,
            "focus_mode": self.focus_mode,
        }
