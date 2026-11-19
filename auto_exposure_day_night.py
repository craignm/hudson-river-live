#!/usr/bin/env python3
"""
Automatic Day / Solar-Glare Dehaze / Twilight / Night UVC Controller
for Insta360 Link / Link 2 / Link 2C / Link 2C Pro (0x2e1a) on macOS.

Adjusts Insta360 12-bit hardware ISP parameters cooperatively over USB Endpoint 0
via ./uvc-util without opening AVCaptureSession or interrupting a live 24/7 OBS stream:
  - Keeps on-camera multi-exposure HDR disabled (`insta360-hdr=0`) to prevent 4K thermal stalls.
  - Solar-elevation EV curve over Hudson River window (40.744823 N, -74.008063 W):
      * Sun >= -1.0 deg (Daytime + Golden Hour + Sunset):     0.00 EV baseline
      * -1.0 deg > Sun > -6.0 deg (Civil Twilight, ~28 min):  smooth ramp 0.00 -> -1.80 EV
      * Sun <= -6.0 deg (Civil Dusk through Civil Dawn):     -1.80 EV (prevents pier floodlight bloom)
  - Cloud-Aware Midday Solar Glare Dehaze (SSW window, camera azimuth ~204 deg):
      * When the Sun is shining onto the SSW window (elev >= 6 deg, |az - cam_az| <= 72 deg),
        samples a tiny 160x90 frame from OBS WebSocket to verify actual sunny veiling glare
        vs. overcast clouds, and smoothly dials 12-bit camera ISP dehaze:
          - exposure-comp-ev:  0.00 -> -0.36 EV  (shrinks solar/river highlight clipping)
          - brightness:          50 -> 44        (subtracts milky window-glass veiling glare floor)
          - contrast:            50 -> 62        (restores skyline & river midtone separation)
          - saturation:          56 -> 61        (recovers color washed out by white glare)
      * Automatically relaxes back to baseline (50 / 50 / 56 / 0.00 EV) when clouds roll in,
        during golden hour, and overnight — with zero USB or OBS traffic outside glare hours.
"""

import argparse
import base64
import datetime as dt
import hashlib
import json
import math
import os
import re
import socket
import struct
import subprocess
import sys
import urllib.parse
import zlib
from pathlib import Path
from typing import Dict, Optional, Tuple

# Default Chelsea Hudson River window coordinates & SSW camera azimuth (overridden by calibration.json)
DEFAULT_LAT = 40.74482298892839
DEFAULT_LON = -74.00806308139002
DEFAULT_CAM_AZ_DEG = 204.2

DEFAULT_DAY_EV = 0.0
DEFAULT_NIGHT_EV = -1.8
DEFAULT_RAMP_START_DEG = -1.0   # Just after upper-limb sunset (~0.83 deg below geometric horizon)
DEFAULT_RAMP_END_DEG = -6.0     # Civil dusk / civil dawn
INSTA360_STEP_EV = 0.06         # Link 2 firmware step size (6 / 100 EV)

# Baseline vs. full sunny-window 12-bit UVC ISP settings
BASE_BRIGHTNESS = 50
BASE_CONTRAST = 50
BASE_SATURATION = 56

DEHAZE_MAX_EV_DELTA = -0.36
DEHAZE_MIN_BRIGHTNESS = 44
DEHAZE_MAX_CONTRAST = 62
DEHAZE_MAX_SATURATION = 61

DEFAULT_OBS_URL = os.environ.get("OBS_URL", "obs://127.0.0.1:4455")


def load_camera_config(base_dir: Path) -> Tuple[float, float, float]:
    """Loads (lat, lon, azimuth_deg) from calibration.json (checking local and PTZ dirs)."""
    candidates = [
        base_dir / "calibration.json",
        Path.home() / "Library" / "Application Support" / "HudsonRiverPTZ" / "calibration.json",
    ]
    for cal_path in candidates:
        if cal_path.is_file():
            try:
                data = json.loads(cal_path.read_text(encoding="utf-8"))
                cam = data.get("camera", data)
                lat = float(cam.get("lat", DEFAULT_LAT))
                lon = float(cam.get("lon", DEFAULT_LON))
                az = float(cam.get("azimuth", data.get("azimuth", DEFAULT_CAM_AZ_DEG)))
                return lat, lon, az
            except Exception:
                pass
    return DEFAULT_LAT, DEFAULT_LON, DEFAULT_CAM_AZ_DEG


def solar_position_deg(when_utc: dt.datetime, lat_deg: float, lon_deg: float) -> Tuple[float, float]:
    """
    NOAA / Meeus solar (elevation_deg, azimuth_deg) using pure Python stdlib math,
    including standard atmospheric refraction near the horizon.
    """
    if when_utc.tzinfo is None:
        when_utc = when_utc.replace(tzinfo=dt.timezone.utc)
    else:
        when_utc = when_utc.astimezone(dt.timezone.utc)

    unix_ts = when_utc.timestamp()
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
    obliq_corr = mean_obliq + 0.00256 * math.cos(math.radians(omega))

    obliq_rad = math.radians(obliq_corr)
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
    cos_zenith = (
        math.sin(lat_rad) * math.sin(decl_rad)
        + math.cos(lat_rad) * math.cos(decl_rad) * math.cos(ha_rad)
    )
    cos_zenith = max(-1.0, min(1.0, cos_zenith))
    zenith_rad = math.acos(cos_zenith)
    zenith_deg = math.degrees(zenith_rad)
    elev_deg = 90.0 - zenith_deg

    # Solar azimuth (0=N, 90=E, 180=S, 270=W)
    sin_zenith = math.sin(zenith_rad)
    if sin_zenith > 1e-6:
        cos_az = (math.sin(lat_rad) * cos_zenith - math.sin(decl_rad)) / (
            math.cos(lat_rad) * sin_zenith
        )
        cos_az = max(-1.0, min(1.0, cos_az))
        az_from_south = math.degrees(math.acos(cos_az))
        az_deg = (az_from_south + 180.0) % 360.0 if hour_angle_deg > 0.0 else (540.0 - az_from_south) % 360.0
    else:
        az_deg = 180.0

    if elev_deg > 85.0:
        refrac = 0.0
    else:
        te = math.tan(math.radians(elev_deg))
        if elev_deg > 5.0:
            refrac = (58.1 / te - 0.07 / (te ** 3) + 0.000086 / (te ** 5)) / 3600.0
        elif elev_deg > -0.575:
            refrac = (
                1735.0
                + elev_deg * (-518.2 + elev_deg * (103.4 + elev_deg * (-12.79 + elev_deg * 0.711)))
            ) / 3600.0
        else:
            refrac = (-20.772 / te) / 3600.0

    return elev_deg + refrac, az_deg


def solar_elevation_deg(when_utc: dt.datetime, lat_deg: float, lon_deg: float) -> float:
    elev, _ = solar_position_deg(when_utc, lat_deg, lon_deg)
    return elev


def angle_diff_deg(a: float, b: float) -> float:
    d = (a - b + 180.0) % 360.0 - 180.0
    return abs(d)


def compute_solar_window_glare_envelope(elev_deg: float, sun_az_deg: float, cam_az_deg: float) -> float:
    """
    Geometric envelope [0.0 .. 1.0] indicating whether the Sun is positioned to cast
    direct/grazing veiling glare through the west/SSW Hudson window & lens (`cam_az ~ 204 deg`):
      - Building shades the window in the morning until `sun_az >= 158 deg` (~10:25 AM).
      - Ramps to 1.0 by `sun_az >= 173 deg` (~11:20 AM) while `elev_deg >= 12 deg`.
      - Tapers smoothly back to 0.0 as the afternoon sun drops toward golden hour (`elev <= 6 deg`).
    """
    if elev_deg <= 6.0:
        return 0.0
    az_min = cam_az_deg - 46.0   # ~158 deg (morning building wall shadow edge)
    az_max = cam_az_deg + 64.0   # ~268 deg (west horizon over New Jersey)
    if sun_az_deg <= az_min or sun_az_deg >= az_max:
        return 0.0
    elev_w = min(1.0, (elev_deg - 6.0) / 6.0)
    az_in = min(1.0, (sun_az_deg - az_min) / 15.0)
    az_out = min(1.0, (az_max - sun_az_deg) / 15.0)
    return round(elev_w * min(az_in, az_out), 3)


def snap_ev(ev: float) -> float:
    snapped = round(ev / INSTA360_STEP_EV) * INSTA360_STEP_EV
    if abs(snapped) < 1e-6:
        return 0.0
    return round(max(-3.0, min(3.0, snapped)), 2)


def compute_target_ev(
    elev_deg: float,
    day_ev: float = DEFAULT_DAY_EV,
    night_ev: float = DEFAULT_NIGHT_EV,
    ramp_start_deg: float = DEFAULT_RAMP_START_DEG,
    ramp_end_deg: float = DEFAULT_RAMP_END_DEG,
) -> Tuple[float, str]:
    if elev_deg >= ramp_start_deg:
        return snap_ev(day_ev), "DAY"
    if elev_deg <= ramp_end_deg:
        return snap_ev(night_ev), "NIGHT"
    span = ramp_start_deg - ramp_end_deg
    frac = (ramp_start_deg - elev_deg) / span if span > 1e-6 else 1.0
    ev = day_ev + frac * (night_ev - day_ev)
    return snap_ev(ev), "TWILIGHT"


# ==============================================================================
# Minimal Pure-Stdlib OBS Screenshot & PNG Histogram Helper (for Cloud Sensing)
# ==============================================================================

def _ws_send_text(sock: socket.socket, text: str) -> None:
    payload = text.encode("utf-8")
    mask = os.urandom(4)
    n = len(payload)
    hdr = bytearray([0x81])
    if n < 126:
        hdr.append(0x80 | n)
    elif n < 65536:
        hdr.append(0x80 | 126)
        hdr.extend(struct.pack("!H", n))
    else:
        hdr.append(0x80 | 127)
        hdr.extend(struct.pack("!Q", n))
    hdr.extend(mask)
    masked = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
    sock.sendall(bytes(hdr) + masked)


def _ws_recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("OBS socket closed")
        buf.extend(chunk)
    return bytes(buf)


def _ws_recv_json(sock: socket.socket) -> dict:
    while True:
        b0, b1 = _ws_recv_exact(sock, 2)
        opcode = b0 & 0x0F
        masked = bool(b1 & 0x80)
        length = b1 & 0x7F
        if length == 126:
            length = struct.unpack("!H", _ws_recv_exact(sock, 2))[0]
        elif length == 127:
            length = struct.unpack("!Q", _ws_recv_exact(sock, 8))[0]
        mask = _ws_recv_exact(sock, 4) if masked else b""
        data = _ws_recv_exact(sock, length)
        if masked:
            data = bytes(b ^ mask[i & 3] for i, b in enumerate(data))
        if opcode == 0x8:
            raise ConnectionError("OBS WebSocket closed")
        if opcode == 0x9:
            pong = bytearray([0x8A, 0x80 | len(data)]) + os.urandom(4)
            sock.sendall(bytes(pong))
            continue
        if opcode in (0x1, 0x2):
            return json.loads(data.decode("utf-8"))


def _decode_png_lumas(png: bytes) -> list:
    if len(png) < 33 or png[:8] != b"\x89PNG\r\n\x1a\n":
        return []
    pos = 8
    width = height = color_type = 0
    idat = bytearray()
    while pos + 8 <= len(png):
        ln, ctype = struct.unpack("!I4s", png[pos : pos + 8])
        cdata = png[pos + 8 : pos + 8 + ln]
        pos += 12 + ln
        if ctype == b"IHDR":
            width, height, bit_depth, color_type = struct.unpack("!IIBB", cdata[:10])
            if bit_depth != 8 or color_type not in (2, 6):
                return []
        elif ctype == b"IDAT":
            idat.extend(cdata)
        elif ctype == b"IEND":
            break
    if not width or not height or not idat:
        return []
    bpp = 4 if color_type == 6 else 3
    stride = width * bpp
    raw = zlib.decompress(bytes(idat))
    prev = bytearray(stride)
    cur = bytearray(stride)
    lumas = []
    p = 0
    for _ in range(height):
        ftype = raw[p]
        p += 1
        row = raw[p : p + stride]
        p += stride
        if ftype == 0:
            cur[:] = row
        elif ftype == 1:
            for i in range(stride):
                cur[i] = (row[i] + (cur[i - bpp] if i >= bpp else 0)) & 0xFF
        elif ftype == 2:
            for i in range(stride):
                cur[i] = (row[i] + prev[i]) & 0xFF
        elif ftype == 3:
            for i in range(stride):
                a = cur[i - bpp] if i >= bpp else 0
                cur[i] = (row[i] + ((a + prev[i]) >> 1)) & 0xFF
        elif ftype == 4:
            for i in range(stride):
                a = cur[i - bpp] if i >= bpp else 0
                b = prev[i]
                c = prev[i - bpp] if i >= bpp else 0
                pa, pb, pc = abs(b - c), abs(a - c), abs(a + b - 2 * c)
                pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                cur[i] = (row[i] + pr) & 0xFF
        for x in range(0, stride, bpp):
            lumas.append(0.2126 * cur[x] + 0.7152 * cur[x + 1] + 0.0722 * cur[x + 2])
        prev, cur = cur, prev
    return lumas


def sample_obs_camera_glare_clarity(obs_url: str, applied_strength: float) -> Optional[Tuple[float, str]]:
    """
    Grabs a 160x90 PNG from OBS's macos-avcapture camera input and computes a
    compensation-invariant solar glare clarity score in [0.0 .. 1.0]:
      - Overcast / cloudy scenes (`p99 < 218`, zero specular sun/river glitter): 0.0
      - Direct sun / window veiling glare (`est_raw_b230` glitter or `est_raw_p02` veil): up to 1.0
    Returns None if OBS is unreachable so caller can fall back cleanly.
    """
    parsed = urllib.parse.urlparse(obs_url if "://" in obs_url else f"obs://{obs_url}")
    host = parsed.hostname or "127.0.0.1"
    port = parsed.port or 4455
    password = urllib.parse.unquote(parsed.password or "")

    sock = socket.create_connection((host, port), timeout=2.5)
    try:
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        req = (
            f"GET / HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n"
            f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
        )
        sock.sendall(req.encode("ascii"))
        buf = b""
        while b"\r\n\r\n" not in buf:
            chunk = sock.recv(1024)
            if not chunk:
                return None
            buf += chunk

        hello = _ws_recv_json(sock)
        auth_info = (hello.get("d") or {}).get("authentication")
        ident: Dict[str, object] = {"rpcVersion": 1, "eventSubscriptions": 0}
        if auth_info:
            secret = base64.b64encode(
                hashlib.sha256((password + auth_info["salt"]).encode("utf-8")).digest()
            ).decode("ascii")
            ident["authentication"] = base64.b64encode(
                hashlib.sha256((secret + auth_info["challenge"]).encode("utf-8")).digest()
            ).decode("ascii")
        _ws_send_text(sock, json.dumps({"op": 1, "d": ident}))
        if _ws_recv_json(sock).get("op") != 2:
            return None

        _ws_send_text(sock, json.dumps({"op": 6, "d": {"requestType": "GetInputList", "requestId": "1"}}))
        cam_name = ""
        while True:
            msg = _ws_recv_json(sock)
            if msg.get("op") == 7 and (msg.get("d") or {}).get("requestId") == "1":
                for inp in ((msg["d"].get("responseData") or {}).get("inputs") or []):
                    kind = inp.get("unversionedInputKind") or inp.get("inputKind") or ""
                    if kind.startswith("macos-avcapture") or kind.startswith("av_capture_input"):
                        cam_name = inp.get("inputName") or ""
                        break
                break
        if not cam_name:
            return None

        _ws_send_text(
            sock,
            json.dumps(
                {
                    "op": 6,
                    "d": {
                        "requestType": "GetSourceScreenshot",
                        "requestId": "2",
                        "requestData": {
                            "sourceName": cam_name,
                            "imageFormat": "png",
                            "imageWidth": 160,
                            "imageHeight": 90,
                        },
                    },
                }
            ),
        )
        b64 = ""
        while True:
            msg = _ws_recv_json(sock)
            if msg.get("op") == 7 and (msg.get("d") or {}).get("requestId") == "2":
                b64 = ((msg["d"].get("responseData") or {}).get("imageData")) or ""
                break
        if "," in b64:
            b64 = b64.split(",", 1)[1]
        if not b64:
            return None
        lumas = _decode_png_lumas(base64.b64decode(b64))
        if len(lumas) < 1000:
            return None

        sl = sorted(lumas)
        n = len(sl)
        p02 = sl[int(n * 0.02)]
        p99 = sl[min(n - 1, int(n * 0.99))]

        # Reconstruct pre-dehaze equivalents so already-applied ISP dehaze never causes oscillation
        est_raw_p02 = p02 + 28.0 * applied_strength
        glitter_thresh = 230.0 - 16.0 * applied_strength
        est_b230_pct = 100.0 * sum(1 for v in sl if v >= glitter_thresh) / n
        est_raw_p99 = min(255.0, p99 + 14.0 * applied_strength)

        # Overcast guard: cloudy days have p99 ~ 195-210 and 0% specular river/sun glitter
        if est_raw_p99 < 218.0 and est_b230_pct < 0.15:
            clarity = 0.0
        else:
            c_glitter = min(1.0, max(0.0, (est_b230_pct - 0.15) / 1.10))
            c_veil = min(1.0, max(0.0, (est_raw_p02 - 23.0) / 10.0))
            clarity = max(c_glitter, 0.85 * c_veil)

        diag = f"p02={p02:.1f}(raw~{est_raw_p02:.1f}) p99={p99:.0f} glitter={est_b230_pct:.2f}%"
        return round(clarity, 3), diag
    except Exception:
        return None
    finally:
        try:
            sock.close()
        except Exception:
            pass


def compute_isp_targets_from_strength(base_day_ev: float, strength: float) -> Tuple[float, int, int, int]:
    """Interpolates 12-bit UVC ISP registers for `strength` in [0.0 .. 1.0]."""
    s = max(0.0, min(1.0, strength))
    ev = snap_ev(base_day_ev + s * DEHAZE_MAX_EV_DELTA)
    bri = int(round(BASE_BRIGHTNESS + s * (DEHAZE_MIN_BRIGHTNESS - BASE_BRIGHTNESS)))
    con = int(round(BASE_CONTRAST + s * (DEHAZE_MAX_CONTRAST - BASE_CONTRAST)))
    sat = int(round(BASE_SATURATION + s * (DEHAZE_MAX_SATURATION - BASE_SATURATION)))
    return ev, bri, con, sat


# ==============================================================================
# Kernel USB Session Cache & Cooperative uvc-util I/O
# ==============================================================================

def query_insta360_usb_session_id() -> Optional[int]:
    """
    Reads kernel IORegistry (zero USB bus packets!) to get the current USB sessionID
    of the connected Insta360 camera (idVendor = 11802 / 0x2e1a).
    """
    try:
        proc = subprocess.run(
            ["/usr/sbin/ioreg", "-r", "-c", "IOUSBHostDevice", "-d", "1"],
            capture_output=True,
            text=True,
            timeout=3.0,
        )
        if proc.returncode != 0:
            return None
        for block in proc.stdout.split("+-o "):
            if '"idVendor" = 11802' in block:
                m = re.search(r'"sessionID"\s*=\s*(\d+)', block)
                if m:
                    return int(m.group(1))
    except Exception:
        pass
    return None


def load_ev_cache(cache_path: Path) -> dict:
    try:
        if cache_path.is_file():
            data = json.loads(cache_path.read_text(encoding="utf-8"))
            if isinstance(data, dict):
                return data
    except Exception:
        pass
    return {}


def save_ev_cache(
    cache_path: Path,
    session_id: Optional[int],
    applied_ev: float,
    bri: int,
    con: int,
    sat: int,
    dehaze_strength: float,
    ts: str,
    hdr_disabled: bool = True,
) -> None:
    if session_id is None:
        return
    try:
        payload = {
            "session_id": int(session_id),
            "applied_ev": float(applied_ev),
            "applied_brightness": int(bri),
            "applied_contrast": int(con),
            "applied_saturation": int(sat),
            "dehaze_strength": round(float(dehaze_strength), 3),
            "hdr_disabled": bool(hdr_disabled),
            "updated_at": ts,
        }
        cache_path.write_text(json.dumps(payload) + "\n", encoding="utf-8")
    except Exception:
        pass


def run_uvc_util(uvc_bin: Path, args_list):
    cmd = [str(uvc_bin), "-V", "0x2e1a"] + args_list
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=8.0)
    return proc.returncode, proc.stdout.strip(), proc.stderr.strip()


def ensure_hdr_disabled(uvc_bin: Path, ts: str) -> bool:
    rc, out, _ = run_uvc_util(uvc_bin, ["-g", "insta360-hdr"])
    if rc != 0:
        return False
    for line in out.splitlines():
        if line.startswith("insta360-hdr="):
            try:
                if int(line.split("=", 1)[1].strip()) == 0:
                    return True
            except ValueError:
                pass
    rc2, out2, err2 = run_uvc_util(uvc_bin, ["-s", "insta360-hdr=0"])
    if rc2 == 0:
        print(f"[{ts}] UPDATED | Disabled on-camera HDR (insta360-hdr: 1 -> 0) to reduce 4K SoC heat")
        return True
    print(f"[{ts}] WARN: Could not disable insta360-hdr ({err2 or out2})", file=sys.stderr)
    return False


def read_current_isp(uvc_bin: Path) -> Tuple[Optional[dict], Optional[str]]:
    rc, out, err = run_uvc_util(
        uvc_bin, ["-g", "exposure-comp-ev", "-g", "brightness", "-g", "contrast", "-g", "saturation"]
    )
    if rc != 0:
        return None, err or out
    vals: dict = {}
    for line in out.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            k = k.strip()
            try:
                vals[k] = float(v.strip()) if k == "exposure-comp-ev" else int(float(v.strip()))
            except ValueError:
                pass
    if "exposure-comp-ev" not in vals:
        return None, f"Unexpected uvc-util output: {out}"
    vals.setdefault("brightness", BASE_BRIGHTNESS)
    vals.setdefault("contrast", BASE_CONTRAST)
    vals.setdefault("saturation", BASE_SATURATION)
    return vals, None


def apply_isp_settings(
    uvc_bin: Path, target_ev: float, target_bri: int, target_con: int, target_sat: int
) -> Tuple[Optional[float], Optional[str]]:
    rc, out, err = run_uvc_util(
        uvc_bin,
        [
            "-s", f"exposure-comp-ev={target_ev:.2f}",
            "-s", f"brightness={target_bri}",
            "-s", f"contrast={target_con}",
            "-s", f"saturation={target_sat}",
        ],
    )
    if rc != 0:
        return None, err or out
    applied_ev = target_ev
    for line in out.splitlines():
        if line.startswith("exposure-comp-ev="):
            try:
                applied_ev = float(line.split("=", 1)[1])
            except ValueError:
                pass
    return applied_ev, None


def print_today_schedule(lat: float, lon: float, cam_az: float, args):
    now_local = dt.datetime.now().astimezone()
    day_start = now_local.replace(hour=0, minute=0, second=0, microsecond=0)
    print(
        f"24-Hour Exposure & Solar-Glare Dehaze Schedule for {day_start.strftime('%Y-%m-%d %Z')} "
        f"({lat:.5f} N, {lon:.5f} W, cam_az={cam_az:.1f} deg):"
    )
    print("-" * 88)
    last_key = None
    for minute in range(0, 24 * 60, 6):
        t_local = day_start + dt.timedelta(minutes=minute)
        elev, az = solar_position_deg(t_local.astimezone(dt.timezone.utc), lat, lon)
        ev, phase = compute_target_ev(
            elev, args.day_ev, args.night_ev, args.ramp_start_deg, args.ramp_end_deg
        )
        glare_env = 0.0 if args.no_dehaze else compute_solar_window_glare_envelope(elev, az, cam_az)
        q_env = round(glare_env * 5.0) / 5.0
        if q_env > 0.0:
            sunny_ev, bri, con, sat = compute_isp_targets_from_strength(ev, q_env)
            phase_disp = f"SUN-GLARE({int(q_env*100):3d}%)"
            isp_disp = f"Clear-sun ISP: {sunny_ev:+.2f} EV, B={bri}, C={con}, S={sat} (cloudy: {ev:+.2f} EV)"
        else:
            phase_disp = phase
            isp_disp = f"Target {ev:+.2f} EV, B={BASE_BRIGHTNESS}, C={BASE_CONTRAST}, S={BASE_SATURATION}"
        key = (phase_disp, round(ev, 2), q_env)
        if key != last_key or minute in (0, 12 * 60, 13 * 60 + 30):
            print(
                f"  {t_local.strftime('%H:%M %Z')} | Sun {elev:+6.2f} deg az {az:5.1f} | "
                f"{phase_disp:<15s} | {isp_disp}"
            )
            last_key = key
    print("-" * 88)


def main() -> int:
    base_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="Automatic sunrise/sunset & midday solar-dehaze UVC controller for Insta360 Link 2C Pro."
    )
    parser.add_argument("--uvc-bin", type=Path, default=base_dir / "uvc-util", help="Path to uvc-util binary")
    parser.add_argument("--obs-url", default=DEFAULT_OBS_URL, help="OBS WebSocket URL for cloud-aware glare sensing")
    parser.add_argument("--day-ev", type=float, default=DEFAULT_DAY_EV, help="Daytime & golden-hour EV (default: 0.0)")
    parser.add_argument("--night-ev", type=float, default=DEFAULT_NIGHT_EV, help="Nighttime anti-bloom EV (default: -1.8)")
    parser.add_argument("--ramp-start-deg", type=float, default=DEFAULT_RAMP_START_DEG, help="Solar elevation to start dusk ramp (default: -1.0 deg)")
    parser.add_argument("--ramp-end-deg", type=float, default=DEFAULT_RAMP_END_DEG, help="Solar elevation to finish dusk ramp (default: -6.0 deg)")
    parser.add_argument("--no-dehaze", action="store_true", help="Disable midday solar window glare dehazing")
    parser.add_argument("--force-ev", type=float, default=None, help="Immediately set a specific EV (-3.0 to +3.0) with baseline ISP and exit")
    parser.add_argument("--force-sync", action="store_true", help="Bypass sessionID cache and verify/re-apply ISP over USB")
    parser.add_argument("--state-file", type=Path, default=base_dir / ".ev_state.json", help="Path to cached USB sessionID + ISP state")
    parser.add_argument("--dry-run", action="store_true", help="Compute target EV & dehaze without sending USB commands")
    parser.add_argument("--schedule-today", action="store_true", help="Print today's local transition timetable and exit")
    parser.add_argument("--quiet-unchanged", action="store_true", help="Only log when camera UVC settings actually change")
    args = parser.parse_args()

    lat, lon, cam_az = load_camera_config(base_dir)

    if args.schedule_today:
        print_today_schedule(lat, lon, cam_az, args)
        return 0

    now_local = dt.datetime.now().astimezone()
    elev, sun_az = solar_position_deg(now_local.astimezone(dt.timezone.utc), lat, lon)
    ts = now_local.strftime("%Y-%m-%d %H:%M:%S %Z")

    cache = load_ev_cache(args.state_file)
    prev_strength = float(cache.get("dehaze_strength", 0.0) or 0.0)

    glare_diag = ""
    dehaze_strength = 0.0
    if args.force_ev is not None:
        target_ev = snap_ev(args.force_ev)
        target_bri, target_con, target_sat = BASE_BRIGHTNESS, BASE_CONTRAST, BASE_SATURATION
        phase = "MANUAL"
    else:
        base_ev, phase = compute_target_ev(
            elev, args.day_ev, args.night_ev, args.ramp_start_deg, args.ramp_end_deg
        )
        glare_env = 0.0 if args.no_dehaze else compute_solar_window_glare_envelope(elev, sun_az, cam_az)
        if phase == "DAY" and glare_env > 0.0:
            obs_sample = sample_obs_camera_glare_clarity(args.obs_url, prev_strength)
            if obs_sample is not None:
                clarity, diag = obs_sample
                raw_target = glare_env * clarity
                # Smooth EMA over 3-min ticks and quantize to 20% steps to avoid chatter
                smoothed = 0.65 * raw_target + 0.35 * prev_strength
                if raw_target >= 0.85 and prev_strength == 0.0:
                    smoothed = raw_target  # Immediate crisp engagement on initial startup in full sun
                dehaze_strength = round(smoothed * 5.0) / 5.0
                if dehaze_strength < 0.15:
                    dehaze_strength = 0.0
                glare_diag = f" | glare_env={glare_env:.2f} clarity={clarity:.2f} ({diag})"
            else:
                dehaze_strength = prev_strength * 0.5 if prev_strength >= 0.4 else 0.0
            target_ev, target_bri, target_con, target_sat = compute_isp_targets_from_strength(
                base_ev, dehaze_strength
            )
            if dehaze_strength > 0.0:
                phase = f"DAY-DEHAZE {int(dehaze_strength * 100)}%"
        else:
            target_ev = base_ev
            target_bri, target_con, target_sat = BASE_BRIGHTNESS, BASE_CONTRAST, BASE_SATURATION

    if args.dry_run:
        print(
            f"[{ts}] DRY-RUN | Sun={elev:+.2f} deg az={sun_az:.1f} ({phase}){glare_diag} -> "
            f"EV={target_ev:+.2f}, B={target_bri}, C={target_con}, S={target_sat}"
        )
        return 0

    usb_session_id = query_insta360_usb_session_id()
    cached_sid = cache.get("session_id")
    cached_ev = cache.get("applied_ev")
    cached_bri = cache.get("applied_brightness")
    cached_con = cache.get("applied_contrast")
    cached_sat = cache.get("applied_saturation")
    cached_hdr_off = bool(cache.get("hdr_disabled", False))

    if (
        args.force_ev is None
        and not args.force_sync
        and usb_session_id is not None
        and cached_sid == usb_session_id
        and cached_hdr_off
        and cached_ev is not None
        and abs(float(cached_ev) - target_ev) < 0.04
        and cached_bri == target_bri
        and cached_con == target_con
        and cached_sat == target_sat
    ):
        # Keep smoothed dehaze_strength fresh if sampled during glare hours
        if abs(dehaze_strength - prev_strength) > 1e-3:
            save_ev_cache(
                args.state_file, usb_session_id, float(cached_ev), target_bri, target_con, target_sat,
                dehaze_strength, ts, hdr_disabled=True
            )
        if not args.quiet_unchanged:
            print(
                f"[{ts}] CACHED-OK | Sun={elev:+.2f} deg az={sun_az:.1f} ({phase}){glare_diag} | "
                f"session={usb_session_id} already at {float(cached_ev):+.2f} EV, "
                f"B={target_bri}, C={target_con}, S={target_sat}, HDR=off (zero USB traffic)"
            )
        return 0

    if not args.uvc_bin.is_file() or not os.access(args.uvc_bin, os.X_OK):
        print(f"[{ts}] ERROR: uvc-util binary not found or not executable at {args.uvc_bin}", file=sys.stderr)
        return 1

    hdr_off = ensure_hdr_disabled(args.uvc_bin, ts)

    cur_isp, err = read_current_isp(args.uvc_bin)
    if cur_isp is None:
        print(f"[{ts}] WARN: Could not read Insta360 camera ISP ({err})", file=sys.stderr)
        return 1

    cur_ev = float(cur_isp["exposure-comp-ev"])
    cur_bri = int(cur_isp["brightness"])
    cur_con = int(cur_isp["contrast"])
    cur_sat = int(cur_isp["saturation"])

    if (
        args.force_ev is None
        and abs(cur_ev - target_ev) < 0.04
        and cur_bri == target_bri
        and cur_con == target_con
        and cur_sat == target_sat
    ):
        save_ev_cache(
            args.state_file, usb_session_id, cur_ev, cur_bri, cur_con, cur_sat,
            dehaze_strength, ts, hdr_disabled=hdr_off
        )
        if not args.quiet_unchanged:
            print(
                f"[{ts}] OK | Sun={elev:+.2f} deg az={sun_az:.1f} ({phase}){glare_diag} | "
                f"camera already at {cur_ev:+.2f} EV, B={cur_bri}, C={cur_con}, S={cur_sat}, HDR=off"
            )
        return 0

    applied_ev, werr = apply_isp_settings(args.uvc_bin, target_ev, target_bri, target_con, target_sat)
    if applied_ev is None:
        print(f"[{ts}] ERROR: Failed applying ISP settings ({werr})", file=sys.stderr)
        return 1

    save_ev_cache(
        args.state_file, usb_session_id, applied_ev, target_bri, target_con, target_sat,
        dehaze_strength, ts, hdr_disabled=hdr_off
    )
    print(
        f"[{ts}] UPDATED | Sun={elev:+.2f} deg az={sun_az:.1f} ({phase}){glare_diag} | "
        f"EV {cur_ev:+.2f}->{applied_ev:+.2f}, B {cur_bri}->{target_bri}, "
        f"C {cur_con}->{target_con}, S {cur_sat}->{target_sat}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
