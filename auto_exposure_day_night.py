#!/usr/bin/env python3
"""
Automatic Day / Twilight / Night UVC Exposure Compensation Controller
for Insta360 Link / Link 2 / Link 2C / Link 2C Pro (0x2e1a) on macOS.

Adjusts Insta360 hardware Exposure Compensation (Extension Unit 9, Selector 0x09)
cooperatively over USB Endpoint 0 via ./uvc-util without opening AVCaptureSession
or interrupting a live 24/7 OBS stream.

Solar-elevation curve over Hudson River window (40.744823 N, -74.008063 W):
  - Sun >= -1.0 deg (Daytime + Late Afternoon + Sunset):   0.00 EV  (bright late afternoons!)
  - -1.0 deg > Sun > -6.0 deg (Civil Twilight, ~28 min):   smooth linear ramp 0.00 -> -1.80 EV
  - Sun <= -6.0 deg (Civil Dusk through Civil Dawn):      -1.80 EV  (prevents night floodlight blooming)
"""

import argparse
import datetime as dt
import json
import math
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Optional, Tuple

# Default Chelsea Hudson River window coordinates (overridden by calibration.json if present)
DEFAULT_LAT = 40.74482298892839
DEFAULT_LON = -74.00806308139002

DEFAULT_DAY_EV = 0.0
DEFAULT_NIGHT_EV = -1.8
DEFAULT_RAMP_START_DEG = -1.0   # Just after upper-limb sunset (~0.83 deg below geometric horizon)
DEFAULT_RAMP_END_DEG = -6.0     # Civil dusk / civil dawn
INSTA360_STEP_EV = 0.06         # Link 2 firmware step size (6 / 100 EV)


def load_coordinates(base_dir: Path):
    cal_path = base_dir / "calibration.json"
    if cal_path.is_file():
        try:
            data = json.loads(cal_path.read_text(encoding="utf-8"))
            cam = data.get("camera", data)
            lat = float(cam.get("lat", DEFAULT_LAT))
            lon = float(cam.get("lon", DEFAULT_LON))
            return lat, lon
        except Exception:
            pass
    return DEFAULT_LAT, DEFAULT_LON


def solar_elevation_deg(when_utc: dt.datetime, lat_deg: float, lon_deg: float) -> float:
    """
    NOAA / Meeus solar elevation angle in degrees (including standard atmospheric
    refraction near the horizon), using pure Python stdlib math.
    """
    if when_utc.tzinfo is None:
        when_utc = when_utc.replace(tzinfo=dt.timezone.utc)
    else:
        when_utc = when_utc.astimezone(dt.timezone.utc)

    # Julian Day & Julian Century from J2000.0
    unix_ts = when_utc.timestamp()
    jd = unix_ts / 86400.0 + 2440587.5
    jc = (jd - 2451545.0) / 36525.0

    # Geometric mean longitude & anomaly of the Sun (deg)
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

    # Solar declination
    sin_decl = math.sin(obliq_rad) * math.sin(app_lon_rad)
    decl_rad = math.asin(max(-1.0, min(1.0, sin_decl)))

    # Equation of time (minutes)
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
    elev_deg = 90.0 - math.degrees(math.acos(cos_zenith))

    # Standard atmospheric refraction correction
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

    return elev_deg + refrac


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
):
    if elev_deg >= ramp_start_deg:
        return snap_ev(day_ev), "DAY"
    if elev_deg <= ramp_end_deg:
        return snap_ev(night_ev), "NIGHT"
    span = ramp_start_deg - ramp_end_deg
    frac = (ramp_start_deg - elev_deg) / span if span > 1e-6 else 1.0
    ev = day_ev + frac * (night_ev - day_ev)
    return snap_ev(ev), "TWILIGHT"


def query_insta360_usb_session_id() -> Optional[int]:
    """
    Reads kernel IORegistry (zero USB bus packets!) to get the current USB sessionID
    of the connected Insta360 camera (idVendor = 11802 / 0x2e1a).
    Any physical replug or IOKit USB re-enumeration assigns a brand-new sessionID.
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


def load_ev_cache(cache_path: Path) -> Tuple[Optional[int], Optional[float]]:
    try:
        if cache_path.is_file():
            data = json.loads(cache_path.read_text(encoding="utf-8"))
            sid = data.get("session_id")
            ev = data.get("applied_ev")
            if sid is not None and ev is not None:
                return int(sid), float(ev)
    except Exception:
        pass
    return None, None


def save_ev_cache(cache_path: Path, session_id: Optional[int], applied_ev: float, ts: str) -> None:
    if session_id is None:
        return
    try:
        payload = {"session_id": int(session_id), "applied_ev": float(applied_ev), "updated_at": ts}
        cache_path.write_text(json.dumps(payload) + "\n", encoding="utf-8")
    except Exception:
        pass


def run_uvc_util(uvc_bin: Path, args_list):
    cmd = [str(uvc_bin), "-V", "0x2e1a"] + args_list
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=8.0)
    return proc.returncode, proc.stdout.strip(), proc.stderr.strip()


def read_current_ev(uvc_bin: Path):
    rc, out, err = run_uvc_util(uvc_bin, ["-g", "exposure-comp-ev"])
    if rc != 0:
        return None, err or out
    for line in out.splitlines():
        if line.startswith("exposure-comp-ev="):
            try:
                return float(line.split("=", 1)[1]), None
            except ValueError:
                pass
    return None, f"Unexpected uvc-util output: {out}"


def write_target_ev(uvc_bin: Path, target_ev: float):
    rc, out, err = run_uvc_util(uvc_bin, ["-s", f"exposure-comp-ev={target_ev:.2f}"])
    if rc != 0:
        return None, err or out
    for line in out.splitlines():
        if line.startswith("exposure-comp-ev="):
            try:
                return float(line.split("=", 1)[1]), None
            except ValueError:
                pass
    return target_ev, None


def print_today_schedule(lat: float, lon: float, args):
    now_local = dt.datetime.now().astimezone()
    tz = now_local.tzinfo
    day_start = now_local.replace(hour=0, minute=0, second=0, microsecond=0)
    print(
        f"24-Hour Exposure Schedule for {day_start.strftime('%Y-%m-%d %Z')} "
        f"({lat:.5f} N, {lon:.5f} W):"
    )
    print("-" * 68)
    last_ev = None
    last_phase = None
    for minute in range(0, 24 * 60, 3):
        t_local = day_start + dt.timedelta(minutes=minute)
        elev = solar_elevation_deg(t_local.astimezone(dt.timezone.utc), lat, lon)
        ev, phase = compute_target_ev(
            elev, args.day_ev, args.night_ev, args.ramp_start_deg, args.ramp_end_deg
        )
        if ev != last_ev or phase != last_phase or minute in (0, 12 * 60, 16 * 60, 16 * 60 + 30):
            print(
                f"  {t_local.strftime('%H:%M %Z')}  |  Sun {elev:+6.2f} deg  |  "
                f"{phase:<8s}  |  Target {ev:+.2f} EV"
            )
            last_ev = ev
            last_phase = phase
    print("-" * 68)


def main() -> int:
    base_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="Automatic sunrise/sunset UVC exposure compensation controller for Insta360 Link 2C Pro."
    )
    parser.add_argument("--uvc-bin", type=Path, default=base_dir / "uvc-util", help="Path to uvc-util binary")
    parser.add_argument("--day-ev", type=float, default=DEFAULT_DAY_EV, help="Daytime & late-afternoon EV (default: 0.0)")
    parser.add_argument("--night-ev", type=float, default=DEFAULT_NIGHT_EV, help="Nighttime anti-bloom EV (default: -1.8)")
    parser.add_argument("--ramp-start-deg", type=float, default=DEFAULT_RAMP_START_DEG, help="Solar elevation to start dusk ramp (default: -1.0 deg)")
    parser.add_argument("--ramp-end-deg", type=float, default=DEFAULT_RAMP_END_DEG, help="Solar elevation to finish dusk ramp (default: -6.0 deg)")
    parser.add_argument("--force-ev", type=float, default=None, help="Immediately set a specific EV (-3.0 to +3.0) and exit")
    parser.add_argument("--force-sync", action="store_true", help="Bypass sessionID cache and verify/re-apply EV over USB")
    parser.add_argument("--state-file", type=Path, default=base_dir / ".ev_state.json", help="Path to cached USB sessionID + EV state")
    parser.add_argument("--dry-run", action="store_true", help="Compute target EV without sending USB commands")
    parser.add_argument("--schedule-today", action="store_true", help="Print today's local transition timetable and exit")
    parser.add_argument("--quiet-unchanged", action="store_true", help="Only log when camera EV actually changes")
    args = parser.parse_args()

    lat, lon = load_coordinates(base_dir)

    if args.schedule_today:
        print_today_schedule(lat, lon, args)
        return 0

    now_local = dt.datetime.now().astimezone()
    elev = solar_elevation_deg(now_local.astimezone(dt.timezone.utc), lat, lon)

    if args.force_ev is not None:
        target_ev = snap_ev(args.force_ev)
        phase = "MANUAL"
    else:
        target_ev, phase = compute_target_ev(
            elev, args.day_ev, args.night_ev, args.ramp_start_deg, args.ramp_end_deg
        )

    ts = now_local.strftime("%Y-%m-%d %H:%M:%S %Z")

    if args.dry_run:
        print(f"[{ts}] DRY-RUN | Sun={elev:+.2f} deg ({phase}) -> target exposure-comp-ev={target_ev:+.2f} EV")
        return 0

    # Zero-USB fast path: during steady DAY (+0.00 EV) or steady NIGHT (-1.80 EV), skip
    # spawning uvc-util entirely if the exact same USB camera sessionID already has target_ev.
    usb_session_id = query_insta360_usb_session_id()
    if args.force_ev is None and not args.force_sync and usb_session_id is not None:
        cached_sid, cached_ev = load_ev_cache(args.state_file)
        if cached_sid == usb_session_id and cached_ev is not None and abs(cached_ev - target_ev) < 0.04:
            if not args.quiet_unchanged:
                print(
                    f"[{ts}] CACHED-OK | Sun={elev:+.2f} deg ({phase}) | "
                    f"session={usb_session_id} already at {cached_ev:+.2f} EV (zero USB traffic)"
                )
            return 0

    if not args.uvc_bin.is_file() or not os.access(args.uvc_bin, os.X_OK):
        print(f"[{ts}] ERROR: uvc-util binary not found or not executable at {args.uvc_bin}", file=sys.stderr)
        return 1

    cur_ev, err = read_current_ev(args.uvc_bin)
    if cur_ev is None:
        print(f"[{ts}] WARN: Could not read Insta360 camera EV ({err})", file=sys.stderr)
        return 1

    if args.force_ev is None and abs(cur_ev - target_ev) < 0.04:
        save_ev_cache(args.state_file, usb_session_id, cur_ev, ts)
        if not args.quiet_unchanged:
            print(f"[{ts}] OK | Sun={elev:+.2f} deg ({phase}) | camera already at {cur_ev:+.2f} EV (no USB write needed)")
        return 0

    applied_ev, werr = write_target_ev(args.uvc_bin, target_ev)
    if applied_ev is None:
        print(f"[{ts}] ERROR: Failed setting EV to {target_ev:+.2f} ({werr})", file=sys.stderr)
        return 1

    save_ev_cache(args.state_file, usb_session_id, applied_ev, ts)
    print(
        f"[{ts}] UPDATED | Sun={elev:+.2f} deg ({phase}) | "
        f"exposure-comp-ev: {cur_ev:+.2f} EV -> {applied_ev:+.2f} EV"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
