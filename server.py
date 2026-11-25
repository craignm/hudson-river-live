#!/usr/bin/env python3
"""
Hudson River 24/7 AR Broadcast & PTZ Director Server (`server.py`)
Runs out-of-the-box on macOS Python 3.9+ standard library (zero pip installs).

Core Responsibilities:
  1. 3D Geodetic -> Camera Projection:
     Converts live WGS84 (lat, lon, altitude) telemetry into a local East-North-Up (ENU)
     Cartesian frame, then projects 3D position + 3D velocity vectors through a calibrated
     pinhole + radial barrel-distortion (`k1`) camera model to compute exact normalized
     image coordinates `(u, v)` and analytical image-plane velocities `(du_s, dv_s)`.
  2. Multi-Sensor Telemetry Ingestion & Anisotropic CTRV Filtering:
     - ADS-B aircraft & helicopters (`local_readsb` on `:8080`, failing over across
       `adsb.fi`, `adsb.lol`, and `airplanes.live`), with NYC EGM2008 geoid correction.
     - Maritime AIS vessels (local VHF RTL-SDR via `AIS-catcher` on `:8100`, fused with
       `aisstream.io` for outer NY Harbor coverage + Pier 40 GPS multipath suppression).
     - Optional Computer Vision / YOLOv11 visual target association (`/api/cv_detections`).
  3. Autonomous Broadcast Director & PTZ Control:
     Ranks visible river/sky targets by rarity, proximity, motion, and recency cooldowns;
     drives either virtual 4K PTZ zoom (`/?obs=1`), OBS's zero-copy transparent 60fps AR
     overlay (`/?hud=1`), or physical VISCA-over-IP / Axis VAPIX optical PTZ cameras.
"""

import hmac
import json
import math
import os
import signal
import sys
import threading
import time
import urllib.request
from collections import deque
from datetime import datetime, timezone
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import urlparse

from aisstream_client import (
    AISStreamTracker,
    classify_ais_ship_type,
    identify_staten_island_ferry,
    is_blacklisted_vessel,
)
from monthly_youtube_uploader import MonthlyYouTubeUploader
from obs_camera_bridge import OBSCameraBridge
from ptz_calibrator import PTZAutoCalibrator
from ptz_visual_tracker import PTZVisualTracker
from ptz_drivers import (
    DEFAULT_SONY_A40_ZOOM_HFOV_TABLE,
    AxisVapixDriver,
    SonySRGA40Driver,
    VirtualPTZDriver,
    ViscaOverIPDriver,
    solar_elevation_deg_utc,
    solar_position_deg_utc,
)

BASE_DIR = Path(__file__).resolve().parent
CALIB_PATH = BASE_DIR / "calibration.json"
STATIC_DIR = BASE_DIR / "static"
ENV_PATH = BASE_DIR / ".env"


def load_dotenv():
    """Loads optional `.env` file into `os.environ` without external `python-dotenv`."""
    if not ENV_PATH.exists():
        return
    for line in ENV_PATH.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))


def upsert_dotenv_var(key: str, value: str):
    """Updates or appends `key=value` in `.env` (`0600`) and `os.environ`."""
    os.environ[key] = value
    lines = ENV_PATH.read_text().splitlines() if ENV_PATH.exists() else []
    out = []
    found = False
    for ln in lines:
        s = ln.strip()
        if s and not s.startswith("#") and "=" in s and s.split("=", 1)[0].strip() == key:
            out.append(f'{key}="{value}"')
            found = True
        else:
            out.append(ln)
    if not found:
        out.append(f'{key}="{value}"')
    ENV_PATH.write_text("\n".join(out) + "\n")
    try:
        os.chmod(ENV_PATH, 0o600)
    except OSError:
        pass


load_dotenv()

# WGS84 constants & NYC EGM2008 Geoid undulation (N = -33.0 m in Lower Manhattan)
# ADS-B alt_geom is height above WGS84 ellipsoid; H_MSL = alt_geom - N = alt_geom + 33.0 m (+108.3 ft)
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
NYC_GEOID_OFFSET_FT = 108.3

# Typical aircraft physical lengths (meters) and friendly names by ICAO type
AIRCRAFT_TYPES = {
    "S76": ("Sikorsky S-76 Helicopter", 13.4, True),
    "B06": ("Bell 206 JetRanger", 12.1, True),
    "B407": ("Bell 407 Helicopter", 12.7, True),
    "B429": ("Bell 429 GlobalRanger", 13.1, True),
    "B430": ("Bell 430 Helicopter", 13.4, True),
    "EC30": ("Airbus H130 Helicopter", 12.6, True),
    "EC35": ("Airbus H135 Helicopter", 12.2, True),
    "EC45": ("Airbus H145 Helicopter", 13.6, True),
    "AS50": ("Airbus H125 AStar", 12.9, True),
    "AS55": ("Airbus AS355 TwinStar", 12.9, True),
    "A109": ("Leonardo AW109", 13.0, True),
    "A139": ("Leonardo AW139", 16.6, True),
    "R44": ("Robinson R44", 11.7, True),
    "R66": ("Robinson R66", 11.7, True),
    "H60": ("Sikorsky UH-60 Black Hawk", 19.8, True),
    "V22": ("Bell Boeing V-22 Osprey", 17.5, True),
    "B737": ("Boeing 737-700", 33.6, False),
    "B738": ("Boeing 737-800", 39.5, False),
    "B739": ("Boeing 737-900", 42.1, False),
    "B38M": ("Boeing 737 MAX 8", 39.5, False),
    "B39M": ("Boeing 737 MAX 9", 42.2, False),
    "A319": ("Airbus A319", 33.8, False),
    "A320": ("Airbus A320", 37.6, False),
    "A321": ("Airbus A321", 44.5, False),
    "A20N": ("Airbus A320neo", 37.6, False),
    "A21N": ("Airbus A321neo", 44.5, False),
    "A220": ("Airbus A220", 38.7, False),
    "BCS1": ("Airbus A220-100", 35.0, False),
    "BCS3": ("Airbus A220-300", 38.7, False),
    "E75L": ("Embraer E175", 31.7, False),
    "E75S": ("Embraer E175", 31.7, False),
    "E190": ("Embraer E190", 36.2, False),
    "CRJ9": ("Bombardier CRJ-900", 36.2, False),
    "B772": ("Boeing 777-200", 63.7, False),
    "B77W": ("Boeing 777-300ER", 73.9, False),
    "B788": ("Boeing 787-8", 56.7, False),
    "B789": ("Boeing 787-9", 62.8, False),
    "B78X": ("Boeing 787-10", 68.3, False),
    "A333": ("Airbus A330-300", 63.7, False),
    "A359": ("Airbus A350-900", 66.8, False),
    "GLF5": ("Gulfstream V", 29.4, False),
    "GLF6": ("Gulfstream G650", 30.4, False),
    "CL35": ("Challenger 350", 20.9, False),
    "C208": ("Cessna 208 Caravan Seaplane", 12.7, False),
}


def lla_to_ecef(lat_deg: float, lon_deg: float, alt_m: float):
    """Converts geodetic WGS84 (deg, deg, meters above ellipsoid/MSL) to Earth-Centered Earth-Fixed (X, Y, Z) meters."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sl * sl)
    return (
        (n + alt_m) * cl * co,
        (n + alt_m) * cl * so,
        (n * (1.0 - WGS84_E2) + alt_m) * sl,
    )


def lla_to_enu(lat_deg: float, lon_deg: float, alt_m: float, lat0: float, lon0: float, alt0: float):
    """Converts target WGS84 coordinates into local tangent-plane East, North, Up (meters) relative to `(lat0, lon0, alt0)`."""
    x, y, z = lla_to_ecef(lat_deg, lon_deg, alt_m)
    x0, y0, z0 = lla_to_ecef(lat0, lon0, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0
    lat_r, lon_r = math.radians(lat0), math.radians(lon0)
    sl, cl = math.sin(lat_r), math.cos(lat_r)
    so, co = math.sin(lon_r), math.cos(lon_r)
    east = -so * dx + co * dy
    north = -sl * co * dx - sl * so * dy + cl * dz
    up = cl * co * dx + cl * so * dy + sl * dz
    return east, north, up


def step_ctrv(e: float, n: float, u: float, ve: float, vn: float, vu: float, omega: float, dt: float):
    """
    Propagates 3D ENU position & velocity forward by dt seconds under
    Constant Turn Rate & Velocity (CTRV) kinematics (omega > 0 = clockwise/right turn).
    """
    if dt == 0.0:
        return e, n, u, ve, vn, vu
    vh = math.hypot(ve, vn)
    if abs(omega) > 1e-4 and vh > 2.0:
        wt = omega * dt
        c, s = math.cos(wt), math.sin(wt)
        ve2 = ve * c + vn * s
        vn2 = -ve * s + vn * c
        de = (ve * s + vn * (1.0 - c)) / omega
        dn = (vn * s - ve * (1.0 - c)) / omega
        return e + de, n + dn, u + vu * dt, ve2, vn2, vu
    return e + ve * dt, n + vn * dt, u + vu * dt, ve, vn, vu


TRACK_REF_LAT = 40.744823
TRACK_REF_LON = -74.008063
TRACK_REF_ALT = 0.0


def project_target(east: float, north: float, up: float, ve: float, vn: float, vu: float, cal: dict):
    """
    Projects 3D ENU position + ENU velocity vector (m/s) into:
      - normalized image coords (u_norm, v_norm) in [0..1]
      - exact image-space velocities (du_s, dv_s) in normalized units/sec
      - PTZ relative angles (pan_deg, tilt_deg)
      - PTZ feed-forward angular rates (pan_dps, tilt_dps)
      - slant distance (dist_m)
    """
    az = math.radians(cal["az_deg"])
    el = math.radians(cal["tilt_deg"])
    roll = math.radians(cal["roll_deg"])

    fx, fy, fz = math.sin(az) * math.cos(el), math.cos(az) * math.cos(el), math.sin(el)
    rx0, ry0, rz0 = math.cos(az), -math.sin(az), 0.0
    ux0, uy0, uz0 = -math.sin(az) * math.sin(el), -math.cos(az) * math.sin(el), math.cos(el)

    cr, sr = math.cos(roll), math.sin(roll)
    rx, ry, rz = cr * rx0 - sr * ux0, cr * ry0 - sr * uy0, cr * rz0 - sr * uz0
    ux, uy, uz = sr * rx0 + cr * ux0, sr * ry0 + cr * uy0, sr * rz0 + cr * uz0

    xc = east * rx + north * ry + up * rz
    yc = east * ux + north * uy + up * uz
    zc = east * fx + north * fy + up * fz

    if zc <= 20.0:
        return None

    half_tan_x = math.tan(math.radians(cal["hfov_deg"] * 0.5))
    half_tan_y = half_tan_x * (9.0 / 16.0)

    xn = xc / zc
    yn = yc / zc
    # Guard undistorted pinhole frustum BEFORE applying cubic k1 barrel distortion:
    # otherwise rays ~76 deg off-axis (high overhead jets) hit (1 + k1*r^2) <= 0 and fold back onto [0..1]!
    if abs(xn) > half_tan_x * 1.35 or abs(yn) > half_tan_y * 1.35:
        return None
    k1 = float(cal.get("k1", 0.0))
    r2 = xn * xn + yn * yn
    if k1 < 0.0 and (1.0 + 3.0 * k1 * r2) <= 0.15:
        return None
    dist_factor = 1.0 + k1 * r2

    u_norm = 0.5 + 0.5 * (xn * dist_factor) / half_tan_x
    v_norm = 0.5 - 0.5 * (yn * dist_factor) / half_tan_y

    dist = math.sqrt(east * east + north * north + up * up)

    # True spherical azimuth/elevation relative to synoptic optical axis
    pan_rad = math.atan2(xc, zc)
    tilt_rad = math.atan2(yc, math.sqrt(xc * xc + zc * zc))

    # Differentiate pan/tilt and exact distorted image (u, v) for silky feed-forward velocity
    vxc = ve * rx + vn * ry + vu * rz
    vyc = ve * ux + vn * uy + vu * uz
    vzc = ve * fx + vn * fy + vu * fz

    inv_zc2 = 1.0 / (zc * zc)
    dxn_dt = (zc * vxc - xc * vzc) * inv_zc2
    dyn_dt = (zc * vyc - yc * vzc) * inv_zc2
    dr2_dt = 2.0 * (xn * dxn_dt + yn * dyn_dt)
    du_s = 0.5 * (dxn_dt * dist_factor + xn * k1 * dr2_dt) / half_tan_x
    dv_s = -0.5 * (dyn_dt * dist_factor + yn * k1 * dr2_dt) / half_tan_y

    denom_xz = max(1.0, xc * xc + zc * zc)
    pan_dps = math.degrees((zc * vxc - xc * vzc) / denom_xz)
    horiz_r = math.sqrt(denom_xz)
    tilt_dps = math.degrees((horiz_r * vyc - yc * (xc * vxc + zc * vzc) / horiz_r) / max(1.0, dist * dist))

    return {
        "u": u_norm,
        "v": v_norm,
        "du_s": du_s,
        "dv_s": dv_s,
        "pan_deg": math.degrees(pan_rad),
        "tilt_deg": math.degrees(tilt_rad),
        "pan_dps": pan_dps,
        "tilt_dps": tilt_dps,
        "dist_m": dist,
    }


def is_occluded_by_skyline(u_norm: float, dist_m: float, alt_m: float) -> bool:
    """
    Filters out low-altitude distant aircraft hidden behind Manhattan (left)
    or Jersey City / Hoboken skyscrapers (right, e.g., EWR final approach).
    """
    # Behind Lower Manhattan skyline (1WTC / Tribeca): u < 0.30 and beyond 3.6 km
    if u_norm < 0.30 and dist_m > 3600 and alt_m < 240:
        return True
    # Behind Chelsea / Meatpacking foreground buildings on far bottom-left
    if u_norm < 0.31 and dist_m < 1500 and alt_m < 65:
        return True
    # Behind Jersey City / Hoboken skyline: u > 0.55 and beyond 4.2 km (Newark EWR traffic)
    if u_norm > 0.55 and dist_m > 4200 and alt_m < 280:
        return True
    return False


def is_vessel_occluded_by_land(lat: float, lon: float, u_norm: float, dist_m: float) -> bool:
    """
    Excludes East River vessels (behind Manhattan) and Newark Bay / Kill Van Kull vessels
    (behind Jersey City / Bayonne) so only visible Hudson River & Upper Bay water is tracked.
    """
    # East River / Pier 11 / Brooklyn side of Manhattan peninsula
    if lat > 40.700 and lon > -74.0125 and dist_m > 2200:
        return True
    if u_norm < 0.305 and dist_m > 1800:
        return True
    # Behind Jersey City / Bayonne peninsula (Newark Bay / Kill Van Kull)
    if lon < -74.036 and lat > 40.695:
        return True
    if lon < -74.065:
        return True
    return False


class SimulatedMaritimeFleet:
    """
    Realistic Hudson River vessel tracks along real channel waypoints so you can test
    boat + aircraft multi-target director switching before plugging in an AIS SDR.
    """

    ROUTES = [
        {
            "mmsi": "367591240",
            "name": "NY WATERWAY HOBOKEN",
            "vessel_type": "High-Speed Passenger Ferry",
            "length_m": 29.0,
            "speed_kts": 24.0,
            "rarity": 1.0,
            "waypoints": [
                (40.7365, -74.0265),
                (40.7405, -74.0165),
                (40.7315, -74.0195),
                (40.7155, -74.0195),
            ],
        },
        {
            "mmsi": "366998110",
            "name": "FDNY MARINE 1 BRAVURA",
            "vessel_type": "Fireboat (140 ft)",
            "length_m": 42.7,
            "speed_kts": 16.5,
            "rarity": 2.4,
            "waypoints": [
                (40.7400, -74.0145),
                (40.7340, -74.0185),
                (40.7220, -74.0220),
                (40.7375, -74.0170),
            ],
        },
        {
            "mmsi": "367144880",
            "name": "MORAN TUG + BARGE",
            "vessel_type": "Articulated Tug & Fuel Barge",
            "length_m": 96.0,
            "speed_kts": 8.5,
            "rarity": 1.8,
            "waypoints": [
                (40.7080, -74.0245),
                (40.7250, -74.0225),
                (40.7390, -74.0195),
                (40.7240, -74.0230),
            ],
        },
        {
            "mmsi": "368012900",
            "name": "SCHOONER AMERICA 2.0",
            "vessel_type": "105 ft Classic Tall Schooner",
            "length_m": 32.0,
            "speed_kts": 9.5,
            "rarity": 2.6,
            "waypoints": [
                (40.7435, -74.0155),
                (40.7360, -74.0165),
                (40.7280, -74.0205),
                (40.7395, -74.0158),
            ],
        },
        {
            "mmsi": "303914000",
            "name": "USCGC STURGEON BAY",
            "vessel_type": "US Coast Guard Cutter",
            "length_m": 42.7,
            "speed_kts": 12.5,
            "rarity": 2.5,
            "waypoints": [
                (40.7160, -74.0250),
                (40.7320, -74.0215),
                (40.7420, -74.0185),
                (40.7260, -74.0240),
            ],
        },
    ]

    def get_vessels(self, now: float) -> list:
        out = []
        for idx, r in enumerate(self.ROUTES):
            wps = r["waypoints"]
            speed_mps = r["speed_kts"] * 0.514444
            # Build closed loop segments
            segs = []
            total_m = 0.0
            for i in range(len(wps)):
                p1 = wps[i]
                p2 = wps[(i + 1) % len(wps)]
                dlat = (p2[0] - p1[0]) * 111132.0
                dlon = (p2[1] - p1[1]) * 111132.0 * math.cos(math.radians(40.73))
                seg_len = max(1.0, math.hypot(dlat, dlon))
                hdg = (math.degrees(math.atan2(dlon, dlat)) + 360.0) % 360.0
                segs.append((p1, p2, seg_len, hdg))
                total_m += seg_len

            cycle_s = total_m / speed_mps
            pos_m = ((now + idx * 73.0) % cycle_s) * speed_mps
            acc = 0.0
            lat, lon, cog = wps[0][0], wps[0][1], 180.0
            for p1, p2, seg_len, hdg in segs:
                if acc + seg_len >= pos_m:
                    frac = (pos_m - acc) / seg_len
                    lat = p1[0] + (p2[0] - p1[0]) * frac
                    lon = p1[1] + (p2[1] - p1[1]) * frac
                    cog = hdg
                    break
                acc += seg_len

            out.append({
                "id": f"ais:{r['mmsi']}",
                "kind": "vessel",
                "source": "sim_ais",
                "callsign": r["name"],
                "subtitle": r["vessel_type"],
                "lat": lat,
                "lon": lon,
                "alt_m": 2.5,
                "gs_kts": r["speed_kts"],
                "track_deg": cog,
                "vrate_mps": 0.0,
                "length_m": r["length_m"],
                "rarity": r["rarity"],
            })
        return out


SCENIC_PRESETS = [
    {
        "id": "colgate_clock",
        "name": "Colgate Clock",
        "label": "COLGATE CLOCK",
        "subtitle": "Jersey City Waterfront • 50-ft Historic Octagonal Dial",
        "lat": 40.712028,
        "lon": -74.033889,
        "alt_m": 16.0,
        "u_override": 0.5504,
        "v_override": 0.5888,
        "zoom_x": 30.0,
        "length_m": 15.2,
    },
    {
        "id": "waters_soul",
        "name": "Water's Soul",
        "label": "WATER'S SOUL",
        "subtitle": "Newport Pier, Jersey City • Jaume Plensa 80-ft Sculpture",
        "lat": 40.730830,
        "lon": -74.028330,
        "alt_m": 22.0,
        "u_override": 0.7782,
        "v_override": 0.6093,
        "zoom_x": 22.0,
        "length_m": 24.4,
    },
    {
        "id": "soccer_pitch",
        "name": "Soccer Pitch",
        "label": "GANSEVOORT SOCCER PITCH",
        "subtitle": "Hudson River Park Athletic Field • South of Little Island",
        "lat": 40.739250,
        "lon": -74.011350,
        "alt_m": 4.0,
        "u_override": 0.46054,
        "v_override": 0.75140,
        "zoom_x": 8.0,
        "length_m": 95.0,
    },
    {
        "id": "statue_of_liberty",
        "name": "Statue of Liberty",
        "label": "STATUE OF LIBERTY",
        "subtitle": "Liberty Island • Upper New York Bay",
        "lat": 40.689249,
        "lon": -74.044500,
        "alt_m": 62.0,
        "u_override": 0.4952,
        "v_override": 0.5768,
        "zoom_x": 30.0,
        "length_m": 46.0,
    },
    {
        "id": "ellis_island",
        "name": "Ellis Island",
        "label": "ELLIS ISLAND",
        "subtitle": "Historic Great Hall • Upper New York Bay",
        "lat": 40.699475,
        "lon": -74.039559,
        "alt_m": 25.0,
        "zoom_x": 24.0,
        "length_m": 118.0,
    },
    {
        "id": "little_island",
        "name": "Little Island",
        "label": "LITTLE ISLAND",
        "subtitle": "Pier 55 Tulip Park & Amphitheatre • Hudson River Park",
        "lat": 40.741940,
        "lon": -74.010280,
        "alt_m": 12.0,
        "u_override": 0.58489,
        "v_override": 0.84172,
        "zoom_x": 3.0,
        "length_m": 85.0,
    },
    {
        "id": "one_wtc",
        "name": "1 World Trade",
        "label": "ONE WORLD TRADE CENTER",
        "subtitle": "Observatory & Spire • Lower Manhattan",
        "lat": 40.712743,
        "lon": -74.013379,
        "alt_m": 417.0,
        "zoom_x": 15.0,
        "length_m": 61.0,
    },
    {
        "id": "verrazzano",
        "name": "Verrazzano Bridge",
        "label": "VERRAZZANO-NARROWS BRIDGE",
        "subtitle": "The Narrows Suspension Span & West Tower • 15.7 km",
        "lat": 40.606950,
        "lon": -74.048200,
        "alt_m": 65.0,
        "u_override": 0.32950,
        "v_override": 0.57920,
        "zoom_x": 28.0,
        "length_m": 211.0,
    },
    {
        "id": "whitney_museum",
        "name": "Whitney Museum",
        "label": "WHITNEY MUSEUM",
        "subtitle": "Renzo Piano Cantilevered Terraces • Gansevoort St",
        "lat": 40.739600,
        "lon": -74.008900,
        "alt_m": 38.0,
        "u_override": 0.2600,
        "v_override": 0.6480,
        "zoom_x": 10.0,
        "length_m": 55.0,
    },
]


def compute_celestial_ephemeris(epoch_s: float, cal: dict) -> list:
    """
    Computes live topocentric Azimuth & Altitude (Meeus Ch. 47 lunar theory + Schlyter
    planetary orbital elements) and calibrated spherical Alt-Azimuth PTZ gimbal angles
    for the Moon, Saturn, Jupiter, and Venus from the Chelsea apartment window.
    """
    jd = epoch_s / 86400.0 + 2440587.5
    T = (jd - 2451545.0) / 36525.0
    d = jd - 2451545.0

    obs_lat = float(cal.get("lat", 40.744823))
    obs_lon = float(cal.get("lon", -74.008063))
    win_az = float(cal.get("az_deg", 207.186))
    syn_tilt = float(cal.get("tilt_deg", 3.171))
    hfov = float(cal.get("hfov_deg", 67.957))
    ptz_pan_off = float(cal.get("ptz_pan_offset_deg", 22.77))
    ptz_tilt_off = float(cal.get("ptz_tilt_offset_deg", 4.596))

    gmst = (280.46061837 + 360.98564736629 * d) % 360.0
    lst = (gmst + obs_lon) % 360.0
    phi = math.radians(obs_lat)
    ecl_r = math.radians(23.439291 - 0.0130042 * T)

    def radec_to_azalt(ra_deg: float, dec_deg: float, parallax_deg: float = 0.0):
        ha = math.radians((lst - ra_deg) % 360.0)
        dec = math.radians(dec_deg)
        sin_alt = math.sin(phi) * math.sin(dec) + math.cos(phi) * math.cos(dec) * math.cos(ha)
        alt = math.degrees(math.asin(max(-1.0, min(1.0, sin_alt))))
        az = (
            math.degrees(
                math.atan2(math.sin(ha), math.cos(ha) * math.sin(phi) - math.tan(dec) * math.cos(phi))
            )
            + 180.0
        ) % 360.0
        alt -= parallax_deg * math.cos(math.radians(max(0.0, alt)))
        return az, alt

    # Sun geocentric ecliptic rectangular coords (for planetary geocentric vector & Moon phase)
    ws = math.radians((282.9404 + 4.70935e-5 * d) % 360.0)
    es = 0.016709 - 1.151e-9 * d
    Ms = math.radians((356.0470 + 0.9856002585 * d) % 360.0)
    Es = Ms + es * math.sin(Ms) * (1.0 + es * math.cos(Ms))
    xvs, yvs = math.cos(Es) - es, math.sqrt(1.0 - es * es) * math.sin(Es)
    lonsun = ws + math.atan2(yvs, xvs)
    rs = math.hypot(xvs, yvs)
    xs_s, ys_s = rs * math.cos(lonsun), rs * math.sin(lonsun)

    # 1. Moon (Meeus Ch. 47 top periodic terms, accurate to ~0.03°)
    Lp = (218.3164477 + 481267.88123421 * T) % 360.0
    D_m = (297.8501921 + 445267.1114034 * T) % 360.0
    M_s = (357.5291092 + 35999.0502909 * T) % 360.0
    Mp = (134.9633964 + 477198.8675055 * T) % 360.0
    F_m = (93.2720950 + 483202.0175233 * T) % 360.0

    lon_m = Lp + (
        6.288774 * math.sin(math.radians(Mp))
        + 1.274027 * math.sin(math.radians(2 * D_m - Mp))
        + 0.658314 * math.sin(math.radians(2 * D_m))
        + 0.213618 * math.sin(math.radians(2 * Mp))
        - 0.185116 * math.sin(math.radians(M_s))
        - 0.114332 * math.sin(math.radians(2 * F_m))
    )
    lat_m = (
        5.128122 * math.sin(math.radians(F_m))
        + 0.280602 * math.sin(math.radians(Mp + F_m))
        + 0.277693 * math.sin(math.radians(Mp - F_m))
        + 0.173237 * math.sin(math.radians(2 * D_m - F_m))
    )
    moon_dist_km = (
        385000.56
        - 20905.355 * math.cos(math.radians(Mp))
        - 3699.111 * math.cos(math.radians(2 * D_m - Mp))
    )
    lon_mr, lat_mr = math.radians(lon_m), math.radians(lat_m)
    x_m = math.cos(lon_mr) * math.cos(lat_mr)
    y_m = math.sin(lon_mr) * math.cos(lat_mr)
    z_m = math.sin(lat_mr)
    xe_m = x_m
    ye_m = y_m * math.cos(ecl_r) - z_m * math.sin(ecl_r)
    ze_m = y_m * math.sin(ecl_r) + z_m * math.cos(ecl_r)
    ra_m = math.degrees(math.atan2(ye_m, xe_m)) % 360.0
    dec_m = math.degrees(math.atan2(ze_m, math.hypot(xe_m, ye_m)))
    par_m = math.degrees(math.asin(6378.14 / moon_dist_km))
    az_moon, alt_moon = radec_to_azalt(ra_m, dec_m, parallax_deg=par_m)

    elong_deg = (lon_m - math.degrees(lonsun)) % 360.0
    illum_pct = int(round(50.0 * (1.0 - math.cos(math.radians(elong_deg)))))

    def solve_planet(N, i, w, a, e, M_p):
        N_r, i_r, w_r, M_r = map(math.radians, (N % 360, i, w % 360, M_p % 360))
        E = M_r + e * math.sin(M_r) * (1.0 + e * math.cos(M_r))
        for _ in range(6):
            E -= (E - e * math.sin(E) - M_r) / (1.0 - e * math.cos(E))
        xv, yv = a * (math.cos(E) - e), a * math.sqrt(1.0 - e * e) * math.sin(E)
        vw, r_h = math.atan2(yv, xv) + w_r, math.hypot(xv, yv)
        xh = r_h * (math.cos(N_r) * math.cos(vw) - math.sin(N_r) * math.sin(vw) * math.cos(i_r))
        yh = r_h * (math.sin(N_r) * math.cos(vw) + math.cos(N_r) * math.sin(vw) * math.cos(i_r))
        zh = r_h * math.sin(vw) * math.sin(i_r)
        xg, yg, zg = xh + xs_s, yh + ys_s, zh
        dist_au = math.sqrt(xg * xg + yg * yg + zg * zg)
        xe_p = xg
        ye_p = yg * math.cos(ecl_r) - zg * math.sin(ecl_r)
        ze_p = yg * math.sin(ecl_r) + zg * math.cos(ecl_r)
        ra_p = math.degrees(math.atan2(ye_p, xe_p)) % 360.0
        dec_p = math.degrees(math.atan2(ze_p, math.hypot(xe_p, ye_p)))
        az_p, alt_p = radec_to_azalt(ra_p, dec_p)
        return az_p, alt_p, dist_au * 149597870.7

    az_sat, alt_sat, dist_sat = solve_planet(
        113.6634 + 2.38980e-5 * d, 2.4886 - 1.081e-7 * d, 339.3939 + 2.97661e-5 * d,
        9.55475, 0.055546 - 9.499e-9 * d, 316.9670 + 0.0334442282 * d
    )
    az_jup, alt_jup, dist_jup = solve_planet(
        100.4542 + 2.76854e-5 * d, 1.3030 - 1.557e-7 * d, 273.8777 + 1.64505e-5 * d,
        5.20256, 0.048498 + 4.469e-9 * d, 19.8950 + 0.0830853001 * d
    )
    az_ven, alt_ven, dist_ven = solve_planet(
        76.6799 + 2.46590e-5 * d, 3.3946 + 2.75e-8 * d, 54.8910 + 1.38374e-5 * d,
        0.723330, 0.006773 - 1.302e-9 * d, 48.0052 + 1.6021302244 * d
    )

    raw_bodies = [
        {
            "id": "moon",
            "name": "Moon",
            "icon": "🌙",
            "label": f"THE MOON ({illum_pct}% ILLUMINATED)",
            "desc": f"{illum_pct}% Lunar Disk • {int(round(moon_dist_km)):,} km",
            "az_deg": az_moon,
            "alt_deg": alt_moon,
            "dist_km": round(moon_dist_km, 0),
            "zoom_x": 24.0,
        },
        {
            "id": "saturn",
            "name": "Saturn",
            "icon": "🪐",
            "label": "PLANET SATURN",
            "desc": f"Ringed Gas Giant • {dist_sat / 1e9:.2f}B km",
            "az_deg": az_sat,
            "alt_deg": alt_sat,
            "dist_km": round(dist_sat, 0),
            "zoom_x": 30.0,
        },
        {
            "id": "jupiter",
            "name": "Jupiter",
            "icon": "🪐",
            "label": "PLANET JUPITER",
            "desc": f"Gas Giant & Galilean Moons • {dist_jup / 1e6:.0f}M km",
            "az_deg": az_jup,
            "alt_deg": alt_jup,
            "dist_km": round(dist_jup, 0),
            "zoom_x": 30.0,
        },
        {
            "id": "venus",
            "name": "Venus",
            "icon": "✨",
            "label": "PLANET VENUS",
            "desc": f"Inner Cloud Planet • {dist_ven / 1e6:.0f}M km",
            "az_deg": az_ven,
            "alt_deg": alt_ven,
            "dist_km": round(dist_ven, 0),
            "zoom_x": 30.0,
        },
    ]

    out = []
    vfov = hfov * 0.5625
    for b in raw_bodies:
        az_b, alt_b = b["az_deg"], b["alt_deg"]
        d_az = ((az_b - win_az + 180.0) % 360.0) - 180.0
        pan_hw = d_az * 0.98 + ptz_pan_off
        tilt_hw = alt_b + (ptz_tilt_off - syn_tilt) + 0.48 * math.sin(math.radians(max(0.0, alt_b)))
        in_window = (-45.0 <= d_az <= 45.0) and (1.5 <= alt_b <= 58.0)

        if in_window:
            status_txt = f"In window now — Az {az_b:.1f}°, Alt +{alt_b:.1f}°"
        elif alt_b <= 0.0:
            status_txt = f"Below horizon (Az {az_b:.0f}°, Alt {alt_b:+.0f}°)"
        elif alt_b > 58.0:
            status_txt = f"Above window header (Az {az_b:.0f}°, Alt +{alt_b:.0f}°)"
        else:
            status_txt = f"Outside SSW window aperture (Az {az_b:.0f}°, Alt +{alt_b:.0f}°)"

        # Map into clamped [0.03..0.97] synoptic reticle coords for visual direction indicator
        u_syn = max(0.03, min(0.97, 0.5 + d_az / max(hfov, 1.0)))
        v_syn = max(0.03, min(0.97, 0.5 - (alt_b - syn_tilt) / max(vfov, 1.0)))

        out.append({
            "id": b["id"],
            "name": b["name"],
            "icon": b["icon"],
            "label": b["label"],
            "subtitle": f"{b['desc']}  •  Az {az_b:.1f}°  Alt {alt_b:+.1f}°",
            "az_deg": round(az_b, 2),
            "alt_deg": round(alt_b, 2),
            "d_az_deg": round(d_az, 2),
            "pan_hw_deg": round(pan_hw, 3),
            "tilt_hw_deg": round(tilt_hw, 3),
            "u": round(u_syn, 4),
            "v": round(v_syn, 4),
            "zoom_x": b["zoom_x"],
            "dist_km": b["dist_km"],
            "visible": in_window,
            "status_note": status_txt,
        })
    return out


class EnhanceLibrary:
    """
    Persistent library of planetary lucky-stacked & turbulence-de-warped PTZ captures.
    Stores `manifest.json` (newest first) alongside `<id>_{thumb.jpg,after.jpg,before.jpg,after.png}`
    inside `enhance_library/` so both Mission Control and `hudsonriver.live/zoom` share a curated archive.
    """

    MAX_ITEMS = 180
    VALID_FILE_RE = None

    def __init__(self, lib_dir: Optional[Path] = None):
        import re

        self.lock = threading.Lock()
        self.lib_dir = lib_dir or (BASE_DIR / "enhance_library")
        self.lib_dir.mkdir(parents=True, exist_ok=True)
        self.manifest_fp = self.lib_dir / "manifest.json"
        if EnhanceLibrary.VALID_FILE_RE is None:
            EnhanceLibrary.VALID_FILE_RE = re.compile(
                r"^[a-z0-9][a-z0-9_-]{2,80}_(thumb\.jpg|after\.jpg|before\.jpg|after\.png)$"
            )
        self.items = self._load_manifest()

    def _load_manifest(self) -> list:
        if not self.manifest_fp.is_file():
            return []
        try:
            data = json.loads(self.manifest_fp.read_text())
            items = data.get("items", []) if isinstance(data, dict) else data
            return [it for it in items if isinstance(it, dict) and it.get("id")]
        except Exception:
            return []

    def _save_manifest_locked(self):
        tmp_fp = self.lib_dir / "manifest.json.tmp"
        tmp_fp.write_text(json.dumps({"items": self.items}, indent=2))
        tmp_fp.replace(self.manifest_fp)

    def count(self) -> int:
        with self.lock:
            return len(self.items)

    def list_items(self, limit: int = 120) -> list:
        with self.lock:
            return [dict(it) for it in self.items[: max(1, min(self.MAX_ITEMS, int(limit)))]]

    def resolve_file(self, filename: str) -> Optional[Path]:
        if not filename or not self.VALID_FILE_RE.match(filename):
            return None
        fp = (self.lib_dir / filename).resolve()
        if fp.parent != self.lib_dir.resolve() or not fp.is_file():
            return None
        return fp

    def record_from_stacks(self, stacks_dir: Path, summary: dict, meta: dict) -> Optional[dict]:
        import re
        import shutil

        after_png = stacks_dir / "latest_after.png"
        before_png = stacks_dir / "latest_before.png"
        if not after_png.is_file():
            return None

        ts = float(summary.get("timestamp") or time.time())
        dt_str = datetime.fromtimestamp(ts).strftime("%Y%m%d-%H%M%S")
        title = str(meta.get("title") or summary.get("label") or "Hudson River View").strip()
        slug = re.sub(r"[^a-z0-9]+", "-", title.lower()).strip("-")[:36] or "hudson-ptz"
        entry_id = f"{dt_str}-{slug}"

        dst_png = self.lib_dir / f"{entry_id}_after.png"
        dst_after_jpg = self.lib_dir / f"{entry_id}_after.jpg"
        dst_before_jpg = self.lib_dir / f"{entry_id}_before.jpg"
        dst_thumb_jpg = self.lib_dir / f"{entry_id}_thumb.jpg"

        shutil.copy2(after_png, dst_png)
        src_after_jpg = stacks_dir / "latest_after.jpg"
        src_before_jpg = stacks_dir / "latest_before.jpg"
        src_thumb_jpg = stacks_dir / "latest_thumb.jpg"

        if src_after_jpg.is_file() and src_before_jpg.is_file() and src_thumb_jpg.is_file():
            shutil.copy2(src_after_jpg, dst_after_jpg)
            shutil.copy2(src_before_jpg, dst_before_jpg)
            shutil.copy2(src_thumb_jpg, dst_thumb_jpg)
        else:
            shutil.copy2(after_png, dst_after_jpg)
            if before_png.is_file():
                shutil.copy2(before_png, dst_before_jpg)
            else:
                shutil.copy2(after_png, dst_before_jpg)
            shutil.copy2(after_png, dst_thumb_jpg)

        entry = {
            "id": entry_id,
            "timestamp": round(ts, 2),
            "created_iso": datetime.fromtimestamp(ts, tz=timezone.utc).isoformat(),
            "title": title,
            "subtitle": str(meta.get("subtitle") or ""),
            "kind": str(meta.get("kind") or "scenic"),
            "dist_km": round(float(meta["dist_km"]), 2) if meta.get("dist_km") is not None else None,
            "zoom_x": round(float(summary.get("zoom_x") or meta.get("zoom_x") or 30.0), 1),
            "width": int(summary.get("width") or 3840),
            "height": int(summary.get("height") or 2160),
            "frames_captured": int(summary.get("frames_captured") or 0),
            "frames_stacked": int(summary.get("frames_stacked") or 0),
            "snr_improvement_x": float(summary.get("snr_improvement_x") or 1.0),
            "peak_drift_arcsec": float(summary.get("peak_drift_arcsec") or 0.0),
            "thumb_file": f"{entry_id}_thumb.jpg",
            "after_jpg": f"{entry_id}_after.jpg",
            "before_jpg": f"{entry_id}_before.jpg",
            "after_png": f"{entry_id}_after.png",
        }

        with self.lock:
            self.items = [it for it in self.items if it.get("id") != entry_id]
            self.items.insert(0, entry)
            overflow = self.items[self.MAX_ITEMS :]
            self.items = self.items[: self.MAX_ITEMS]
            for old in overflow:
                oid = old.get("id", "")
                if oid:
                    for suf in ("_thumb.jpg", "_after.jpg", "_before.jpg", "_after.png"):
                        (self.lib_dir / f"{oid}{suf}").unlink(missing_ok=True)
            self._save_manifest_locked()
        return entry


class LuckyStackManager:
    """
    Asynchronous planetary lucky-imaging + Multi-Alignment-Point turbulence de-warping
    coordinator for Mission Control's 1-click "✨ Lucky Stack" button.
    """

    def __init__(self, director: "WorldDirector"):
        self.director = director
        self.lock = threading.Lock()
        self.stacks_dir = BASE_DIR / "stacks"
        self.stacks_dir.mkdir(parents=True, exist_ok=True)
        self.status = {
            "running": False,
            "phase": "idle",
            "progress_pct": 0,
            "message": "Ready",
            "last_ok": None,
            "last_run_ts": 0.0,
            "summary": None,
        }
        meta_fp = self.stacks_dir / "latest_meta.json"
        if meta_fp.is_file():
            try:
                s = json.loads(meta_fp.read_text())
                self.status["last_ok"] = True
                self.status["last_run_ts"] = float(s.get("timestamp", meta_fp.stat().st_mtime))
                self.status["summary"] = s
                self.status["message"] = f"Last stack: {s.get('label', 'Ready')} ({s.get('snr_improvement_x', '')}× SNR)"
            except Exception:
                pass

    def get_status(self) -> dict:
        with self.lock:
            return dict(self.status)

    def _update(self, **kw):
        with self.lock:
            self.status.update(kw)

    @staticmethod
    def _find_stacker_python() -> str:
        candidates = [
            BASE_DIR / ".venv" / "bin" / "python",
            Path.home() / "Library" / "Application Support" / "HudsonRiverPTZ" / ".venv" / "bin" / "python",
            Path("/opt/homebrew/bin/python3"),
            Path("/usr/local/bin/python3"),
        ]
        for p in candidates:
            if p.is_file() and os.access(str(p), os.X_OK):
                return str(p)
        return sys.executable

    @staticmethod
    def _bmp_center_tenengrad_and_mean(bmp_bytes: bytes) -> tuple:
        import struct

        off = struct.unpack_from("<I", bmp_bytes, 10)[0]
        w, h_sig = struct.unpack_from("<ii", bmp_bytes, 18)
        bpp = struct.unpack_from("<H", bmp_bytes, 28)[0]
        topdown = h_sig < 0
        h = abs(h_sig)
        bytes_pp = bpp // 8
        stride = ((w * bytes_pp + 3) // 4) * 4

        y0, y1 = int(h * 0.25), int(h * 0.75)
        x0, x1 = int(w * 0.25), int(w * 0.75)
        ten = 0
        l_sum = 0
        cnt = 0
        for yy in range(y0, y1, 6):
            ry = yy if topdown else (h - 1 - yy)
            ry_p = (yy + 2) if topdown else (h - 1 - (yy + 2))
            ry_m = (yy - 2) if topdown else (h - 1 - (yy - 2))
            b_cur = off + ry * stride
            b_yp = off + ry_p * stride
            b_ym = off + ry_m * stride
            for xx in range(x0, x1, 6):
                idx = b_cur + xx * bytes_pp + 1  # Green channel
                g_c = bmp_bytes[idx]
                l_sum += g_c
                cnt += 1
                gx = bmp_bytes[idx + 2 * bytes_pp] - bmp_bytes[idx - 2 * bytes_pp]
                gy = bmp_bytes[b_yp + xx * bytes_pp + 1] - bmp_bytes[b_ym + xx * bytes_pp + 1]
                g2 = gx * gx + gy * gy
                if g2 > 144:
                    ten += g2
        return ten, (l_sum / max(cnt, 1))

    def start_async(self, frames: int = 36) -> dict:
        with self.lock:
            if self.status["running"]:
                return {"ok": False, "error": "Lucky Stack is already running"}
            if self.director.calibrating:
                return {"ok": False, "error": "PTZ calibration is currently running"}
            self.status.update(
                running=True,
                phase="starting",
                progress_pct=2,
                message="Preparing 4K lucky-imaging capture…",
            )
        n_frames = max(12, min(80, int(frames)))
        threading.Thread(target=self._worker, args=(n_frames,), daemon=True).start()
        return {"ok": True}

    def _worker(self, n_frames: int):
        import base64
        import re
        import shutil
        import subprocess
        import tempfile

        work_dir = Path(tempfile.mkdtemp(prefix="hudson_lucky_stack_"))
        drv = self.director.hw_driver
        is_sony = isinstance(drv, SonySRGA40Driver)
        cap_w, cap_h = (3840, 2160) if is_sony else (1920, 1080)

        with self.director.lock:
            self.director.stacking = True
            cur_zoom = float(self.director.ptz.zoom_x)
            stack_meta = {"zoom_x": cur_zoom, "kind": "scenic", "subtitle": "", "dist_km": None}
            if not self.director.auto_mode and self.director.manual_point:
                mp = self.director.manual_point
                label = str(mp.get("label") or "HUDSON RIVER LANDMARK")
                stack_meta["title"] = label
                stack_meta["subtitle"] = str(mp.get("subtitle") or "")
                stack_meta["kind"] = str(mp.get("kind") or "landmark")
                stack_meta["dist_km"] = mp.get("dist_km")
            elif self.director.active_target_id:
                t_obj = self.director.last_targets_by_id.get(self.director.active_target_id)
                if t_obj:
                    label = str(t_obj.get("callsign") or self.director.active_target_id)
                    stack_meta["title"] = label
                    stack_meta["subtitle"] = str(t_obj.get("subtitle") or "")
                    stack_meta["kind"] = str(t_obj.get("kind") or "vessel")
                    stack_meta["dist_km"] = t_obj.get("dist_km")
                else:
                    label = str(self.director.active_target_id).upper()
                    stack_meta["title"] = label
            else:
                label = "HUDSON RIVER CORRIDOR"
                stack_meta["title"] = "Hudson River Corridor"

        try:
            try:
                with self.director.obs_cam.open_calibration_session(timeout=8.0) as cal:
                    def grab_bmp(dst_fp: Path) -> bytes:
                        resp = cal.rpc(
                            "GetSourceScreenshot",
                            {
                                "sourceName": cal.ptz_source,
                                "imageFormat": "bmp",
                                "imageWidth": cap_w,
                                "imageHeight": cap_h,
                            },
                        )
                        raw = base64.b64decode((resp.get("imageData") or "").split(",", 1)[-1])
                        dst_fp.write_bytes(raw)
                        return raw

                    self._update(phase="reference", progress_pct=6, message="Capturing single raw reference frame…")
                    time.sleep(0.35)
                    before_bytes = grab_bmp(work_dir / "before.bmp")
                    _, ref_mean = self._bmp_center_tenengrad_and_mean(before_bytes)

                    # Sony SRG-A40 low-noise exposure + 5-step optical focus micro-trim
                    if is_sony and drv.reachable and not drv.needs_password_init:
                        lat = float(self.director.calibration.get("lat", 40.744823))
                        lon = float(self.director.calibration.get("lon", -74.008063))
                        elev = solar_elevation_deg_utc(time.time(), lat, lon)
                        st_im, b_im = drv.cgi.get("/command/inquiry.cgi?inq=imaging", auth=True, timeout=1.6)
                        kv_im = drv._parse_kv_body(b_im) if st_im == 200 else {}
                        cur_gain = int(kv_im.get("ExposureGain", "0") or "0")

                        if not getattr(drv, "celestial_mode", None) and elev < -2.0 and cur_gain >= 5:
                            self._update(phase="exposure", progress_pct=10, message="Switching to low-noise astrophotography gain…")
                            if ref_mean > 42.0:
                                # Bright floodlit night scene (e.g. Gansevoort Soccer Pitch)
                                drv._send_cgi(
                                    "imaging.cgi",
                                    "ExposureMode=manual&ExposureIris=17&ExposureGain=2&ExposureExposureTime=9"
                                    "&VisibilityEnhancer=off&HighSensitivityMode=off",
                                )
                            else:
                                # Dark harbor night scene (Statue of Liberty, Colgate Clock, Water's Soul, Verrazzano)
                                drv._send_cgi(
                                    "imaging.cgi",
                                    "ExposureMode=manual&ExposureIris=17&ExposureGain=3&ExposureExposureTime=8"
                                    "&VisibilityEnhancer=off&HighSensitivityMode=off",
                                )
                            time.sleep(0.65)

                        kv_ptzf = drv.query_ptzf() or {}
                        aptzf = kv_ptzf.get("AbsolutePTZF", "").split(",")
                        if len(aptzf) >= 4:
                            try:
                                f0 = int(aptzf[3], 16)
                                self._update(phase="focus", progress_pct=14, message="Locking peak diffraction focus…")
                                drv._send_cgi("ptzf.cgi", "FocusMode=manual")
                                time.sleep(0.25)
                                best_f, best_sc = f0, -1
                                probe_tmp = work_dir / "focus_probe.bmp"
                                for df in (-20, -10, 0, 10, 20):
                                    fc = max(0x1000, min(0x9fff, f0 + df))
                                    drv._send_cgi("ptzf.cgi", f"AbsoluteFocus={fc:04x}")
                                    time.sleep(0.26)
                                    sc, _ = self._bmp_center_tenengrad_and_mean(grab_bmp(probe_tmp))
                                    if sc > best_sc:
                                        best_sc, best_f = sc, fc
                                drv._send_cgi("ptzf.cgi", f"AbsoluteFocus={best_f:04x}")
                                probe_tmp.unlink(missing_ok=True)
                                time.sleep(0.30)
                            except Exception:
                                pass

                    # Capture lossless burst frames
                    for i in range(n_frames):
                        pct = 18 + int(round(40.0 * (i + 1) / n_frames))
                        self._update(
                            phase="capturing",
                            progress_pct=pct,
                            message=f"Capturing lossless {cap_w}×{cap_h} burst ({i + 1}/{n_frames})…",
                        )
                        grab_bmp(work_dir / f"frame_{i:03d}.bmp")
                        time.sleep(0.03)

                    (work_dir / "meta.json").write_text(
                        json.dumps({"width": cap_w, "height": cap_h, "zoom_x": cur_zoom, "label": label})
                    )
            finally:
                # Immediately restore live camera AF + solar AE (or active celestial exposure) and release gimbal lock!
                if is_sony and drv and drv.reachable:
                    if getattr(drv, "celestial_mode", None):
                        drv.set_celestial_exposure(drv.celestial_mode, force=True)
                    else:
                        drv._send_cgi("ptzf.cgi", f"FocusNearLimit={drv.focus_near_limit_hex}&AFSensitivity=low&FocusMode=auto")
                        drv.focus_mode = "sony_af_20m_limit"
                        drv._apply_solar_exposure_mode(force=True)
                with self.director.lock:
                    self.director.stacking = False

            # Run Python/SciPy MAP de-warping + kappa-sigma stacking + RL wavelet deconvolution
            py_bin = self._find_stacker_python()
            script_fp = BASE_DIR / "lucky_stack_ptz.py"
            proc = subprocess.Popen(
                [py_bin, str(script_fp), "--universal", str(work_dir), str(self.stacks_dir), "--label", label],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            prog_fp = work_dir / "progress.json"
            while proc.poll() is None:
                if prog_fp.is_file():
                    try:
                        pr = json.loads(prog_fp.read_text())
                        self._update(
                            phase=pr.get("phase", "stacking"),
                            progress_pct=int(pr.get("progress_pct", 70)),
                            message=pr.get("message", "Stacking & deconvolving…"),
                        )
                    except Exception:
                        pass
                time.sleep(0.22)

            out_s, err_s = proc.communicate()
            if proc.returncode != 0:
                raise RuntimeError((err_s or out_s or "Stacker subprocess failed").strip()[-240:])

            summary = json.loads((self.stacks_dir / "latest_meta.json").read_text())
            lib_entry = self.director.enhance_library.record_from_stacks(self.stacks_dir, summary, stack_meta)
            if lib_entry:
                summary["library_entry"] = lib_entry
                (self.stacks_dir / "latest_meta.json").write_text(json.dumps(summary, indent=2))
            slug = re.sub(r"[^a-z0-9]+", "_", label.lower()).strip("_") or "hudson_ptz"
            ts_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
            shutil.copy2(self.stacks_dir / "latest_after.png", self.stacks_dir / f"{slug}_{ts_tag}_stacked.png")
            shutil.copy2(self.stacks_dir / "latest_comparison.png", self.stacks_dir / f"{slug}_{ts_tag}_comparison.png")

            # Prune old archive files beyond the 24 most recent so disk usage stays bounded
            archives = sorted(self.stacks_dir.glob("*_20*_*.png"), key=lambda p: p.stat().st_mtime, reverse=True)
            for old_fp in archives[48:]:
                old_fp.unlink(missing_ok=True)

            self._update(
                running=False,
                phase="idle",
                progress_pct=100,
                last_ok=True,
                last_run_ts=float(summary["timestamp"]),
                summary=summary,
                message=f"Stacked {summary['frames_stacked']}/{summary['frames_captured']} frames ({summary['snr_improvement_x']}× SNR)",
            )
        except Exception as exc:
            self._update(
                running=False,
                phase="error",
                progress_pct=0,
                last_ok=False,
                message=f"Lucky Stack error: {exc}",
            )
        finally:
            with self.director.lock:
                now_end = time.time()
                self.director.dwell_started_at = now_end
                if self.director.public_lease_until_ts > 0.0:
                    self.director.public_lease_until_ts = max(
                        self.director.public_lease_until_ts, now_end + 90.0
                    )
            shutil.rmtree(work_dir, ignore_errors=True)


class WorldDirector:
    """
    Central thread-safe world model fusing ADS-B aircraft, VHF/cloud AIS vessels,
    and optional CV detections into 3D CTRV-filtered tracks projected onto the camera frame.
    """

    PTZ_PROFILE_KEYS = (
        "ptz_offset_east_m",
        "ptz_offset_north_m",
        "ptz_offset_up_m",
        "ptz_pan_offset_deg",
        "ptz_tilt_offset_deg",
        "ptz_roll_offset_deg",
        "ptz_pan_scale",
        "ptz_tilt_scale",
        "ptz_collimation_pan_10x_deg",
        "ptz_collimation_tilt_10x_deg",
        "ptz_steps_per_deg",
        "ptz_max_zoom_x",
        "ptz_zoom_hfov_table",
        "ptz_focus_model",
        "ptz_day_exposure",
        "ptz_last_calibration",
    )

    def __init__(self):
        self.lock = threading.Lock()
        self.calibration = json.loads(CALIB_PATH.read_text())
        self._ensure_ptz_profiles()
        self.ptz = VirtualPTZDriver()
        self.hw_driver = None

        initial_obs_url = (
            os.environ.get("OBS_URL", "")
            or os.environ.get("MAC_MINI_CAM_URL", "")
            or self.calibration.get("obs_url", "")
            or self.calibration.get("remote_cam_url", "")
        )
        active_model = str(self.calibration.get("ptz_camera_model", "sony_a40"))
        self.obs_cam = OBSCameraBridge(initial_obs_url, preferred_ptz=active_model)
        self.obs_cam.set_ptz_youtube_config(
            stream_key=os.environ.get("PTZ_YOUTUBE_STREAM_KEY", "").strip(),
            enabled=bool(self.calibration.get("ptz_youtube_enabled", False)),
        )
        self._configure_optional_hw_ptz()

        self.raw_adsb = []
        self.raw_adsb_ts = 0.0
        self.adsb_source = "initializing"
        # Persistent 3D CTRV / anisotropic Kalman tracks (id -> state dict in TRACK_REF ENU)
        self.kinematic_tracks = {}

        self.raw_ais = []
        self.raw_ais_ts = 0.0
        self.local_ais_pkt_state = {}  # mmsi -> (messages_count, rx_epoch, pos_epoch, lat, lon)
        self.ais_source = "initializing"
        ais_key = os.environ.get("AISSTREAM_API_KEY", "").strip()
        self.aisstream = AISStreamTracker(ais_key) if ais_key else None
        self.enable_sim_vessels = not bool(ais_key)
        self.sim_fleet = SimulatedMaritimeFleet()

        # Recent CV detections posted from the Link 2C browser spotter or YOLO script
        self.cv_tracks = {}  # id -> {u, v, w, h, kind, score, ts, du_s, dv_s}
        self.cv_mode = "assist"

        # Director state
        self.auto_mode = True
        self.calibrating = False
        self.stacking = False
        self.auto_recalibrate_daily = bool(self.calibration.get("ptz_auto_recalibrate_daily", True))
        self.active_target_id = None
        self.last_targets_by_id = {}
        self.dwell_started_at = 0.0
        self.active_dwell_limit_sec = 30.0
        self.last_target_switch_ts = 0.0
        self.switch_slew_until_ts = 0.0
        self.last_non_synoptic_kind = "landmark"
        self._latest_snapshot = None
        self._hw_io_lock = threading.Lock()
        self.min_dwell_sec = 28.0
        self.max_dwell_sec = 30.0
        self.landmark_dwell_sec = 30.0
        self.mover_cooldown_sec = 130.0  # Prefer fresh boats before revisiting within ~2 minutes
        self.landmark_max_gap_sec = 64.0
        self.last_landmark_visit_ts = 0.0
        self.synoptic_interval_sec = 224.0  # Zoom out to 1x wide synoptic harbor overview every ~4 minutes
        self.synoptic_dwell_sec = 28.0
        self.last_synoptic_visit_ts = time.time()
        self.revisit_gap_visits = 2  # Also visit >= 2 other targets before revisiting same boat/plane
        self.visit_seq = 0
        self.last_left_seq = {}  # target_id -> visit_seq when Auto-Director stepped away
        self.cooldowns = {}  # target_id -> last_featured_ts
        self.manual_zoom_override = None
        self.manual_point = None  # optional {"u", "v", "pan_deg", "tilt_deg", "zoom_x", "label"}
        self.active_celestial_id = None
        self.public_lease_until_ts = 0.0
        self.public_last_select_ts = 0.0
        self.public_last_enhance_ts = 0.0
        self.enhance_library = EnhanceLibrary()
        self.ptz_calibrator = PTZAutoCalibrator(self)
        self.lucky_stacker = LuckyStackManager(self)
        self.visual_tracker = PTZVisualTracker(self, BASE_DIR)
        self.monthly_yt = MonthlyYouTubeUploader(
            self.obs_cam, BASE_DIR, calibration_getter=lambda: self.calibration
        )
        self.monthly_yt.set_enabled(bool(self.calibration.get("monthly_youtube_enabled", True)))

    def _ensure_ptz_profiles(self):
        """
        Migrates legacy single-camera `calibration.json` so FoMaKo's calibrated 3D bore-sight &
        19-stop focus curve are 100% preserved inside `ptz_profiles['fomako']` alongside
        `ptz_profiles['sony_a40']`, allowing 1-click lossless switching between cameras.
        """
        profiles = self.calibration.get("ptz_profiles")
        changed = False
        if not isinstance(profiles, dict):
            fomako_prof = {
                k: json.loads(json.dumps(self.calibration[k]))
                for k in self.PTZ_PROFILE_KEYS
                if k in self.calibration
            }
            sony_prof = {
                "ptz_offset_east_m": 0.25,
                "ptz_offset_north_m": 0.0,
                "ptz_offset_up_m": 0.0,
                "ptz_pan_offset_deg": 0.0,
                "ptz_tilt_offset_deg": 0.0,
                "ptz_roll_offset_deg": 0.0,
                "ptz_pan_scale": 1.0,
                "ptz_tilt_scale": 1.0,
                "ptz_collimation_pan_10x_deg": 0.0,
                "ptz_collimation_tilt_10x_deg": 0.0,
                "ptz_steps_per_deg": 51.2,
                "ptz_max_zoom_x": 30.0,
                "ptz_zoom_hfov_table": [list(p) for p in DEFAULT_SONY_A40_ZOOM_HFOV_TABLE],
                "ptz_focus_model": {"enabled": False, "zoom_focus_table": [], "inv_km_slope_30x": 0.0},
            }
            profiles = {"fomako": fomako_prof, "sony_a40": sony_prof}
            self.calibration["ptz_profiles"] = profiles
            if "ptz_camera_model" not in self.calibration:
                self.calibration["ptz_camera_model"] = "sony_a40"
            if "ptz_sony_host" not in self.calibration:
                self.calibration["ptz_sony_host"] = os.environ.get("SONY_PTZ_HOST", "192.168.1.136")
            if "ptz_sony_user" not in self.calibration:
                self.calibration["ptz_sony_user"] = os.environ.get("SONY_PTZ_USER", "admin")
            active = self.calibration["ptz_camera_model"]
            for k, v in profiles.get(active, {}).items():
                self.calibration[k] = json.loads(json.dumps(v))
            changed = True
        else:
            if "ptz_sony_host" not in self.calibration:
                self.calibration["ptz_sony_host"] = os.environ.get("SONY_PTZ_HOST", "192.168.1.136")
                changed = True
            if "ptz_sony_user" not in self.calibration:
                self.calibration["ptz_sony_user"] = os.environ.get("SONY_PTZ_USER", "admin")
                changed = True
        if changed:
            CALIB_PATH.write_text(json.dumps(self.calibration, indent=2) + "\n")

    def switch_ptz_camera(self, target_model: str):
        """Hot-switches between `'sony_a40'` and `'fomako'` while preserving each camera's calibration."""
        norm = "fomako" if str(target_model).strip().lower() == "fomako" else "sony_a40"
        with self.lock:
            cur = str(self.calibration.get("ptz_camera_model", "sony_a40"))
            profiles = self.calibration.setdefault("ptz_profiles", {})
            # Snapshot outgoing camera's active calibration
            cur_snap = profiles.setdefault(cur, {})
            for k in self.PTZ_PROFILE_KEYS:
                if k in self.calibration:
                    cur_snap[k] = json.loads(json.dumps(self.calibration[k]))

            self.calibration["ptz_camera_model"] = norm
            next_prof = profiles.setdefault(norm, {})
            for k in self.PTZ_PROFILE_KEYS:
                if k in next_prof:
                    self.calibration[k] = json.loads(json.dumps(next_prof[k]))

            if self.hw_driver and hasattr(self.hw_driver, "_close_usb_pipe"):
                self.hw_driver._close_usb_pipe()
            self.hw_driver = None
            CALIB_PATH.write_text(json.dumps(self.calibration, indent=2) + "\n")

        self.obs_cam.set_preferred_ptz(norm)
        self._configure_optional_hw_ptz()
        last_rep = self.calibration.get("ptz_last_calibration")
        if isinstance(last_rep, dict):
            self.ptz_calibrator._update_status(
                last_run_ts=float(last_rep.get("timestamp", 0.0)),
                last_ok=True,
                pre_rms_deg=last_rep.get("pre_rms_deg"),
                post_rms_deg=last_rep.get("post_rms_deg"),
                inlier_count=last_rep.get("inlier_count", 0),
                message=last_rep.get("summary", "Calibrated"),
                landmarks=last_rep.get("landmarks", []),
            )
        else:
            self.ptz_calibrator._update_status(
                last_run_ts=0.0,
                last_ok=None,
                pre_rms_deg=None,
                post_rms_deg=None,
                inlier_count=0,
                message=f"Ready to calibrate ({'Sony SRG-A40' if norm == 'sony_a40' else 'FoMaKo 30x'})",
                landmarks=[],
            )

    def _configure_optional_hw_ptz(self):
        cam_model = (
            os.environ.get("PTZ_CAMERA_MODEL", "").strip().lower()
            or str(self.calibration.get("ptz_camera_model") or "sony_a40").strip().lower()
        )
        pan_off = float(
            os.environ.get("PTZ_PAN_OFFSET_DEG")
            if os.environ.get("PTZ_PAN_OFFSET_DEG") is not None
            else self.calibration.get("ptz_pan_offset_deg", 0.0)
        )
        tilt_off = float(
            os.environ.get("PTZ_TILT_OFFSET_DEG")
            if os.environ.get("PTZ_TILT_OFFSET_DEG") is not None
            else self.calibration.get("ptz_tilt_offset_deg", 0.0)
        )
        steps_deg = float(self.calibration.get("ptz_steps_per_deg", 14.4))
        max_z = float(self.calibration.get("ptz_max_zoom_x", 30.0))

        if cam_model == "sony_a40":
            sony_host = (
                os.environ.get("SONY_PTZ_HOST", "").strip()
                or str(self.calibration.get("ptz_sony_host") or "192.168.1.136").strip()
            )
            sony_user = (
                os.environ.get("SONY_PTZ_USER", "").strip()
                or str(self.calibration.get("ptz_sony_user") or "admin").strip()
            )
            sony_pass = os.environ.get("SONY_PTZ_PASS", "")
            self.hw_driver = SonySRGA40Driver(
                host=sony_host,
                username=sony_user,
                password=sony_pass,
                pan_offset_deg=pan_off,
                tilt_offset_deg=tilt_off,
                max_zoom_x=max_z,
                calibration=self.calibration,
            )
            self.hw_driver.configure_outdoor_window_defaults()
            return

        fomako_ip = (
            os.environ.get("PTZ_FOMAKO_IP", "").strip()
            or str(self.calibration.get("ptz_fomako_ip") or "").strip()
        )
        visca_ip = os.environ.get("PTZ_VISCA_IP", "").strip()
        axis_ip = os.environ.get("PTZ_AXIS_IP", "").strip()
        usb_enabled = bool(self.calibration.get("ptz_usb_enabled", True)) and (
            (BASE_DIR / "uvc-util").is_file()
            or (Path.home() / "Library/Application Support/HudsonRiverUVC/uvc-util").is_file()
        )

        if fomako_ip or usb_enabled:
            port = int(
                os.environ.get("PTZ_FOMAKO_PORT")
                if os.environ.get("PTZ_FOMAKO_PORT")
                else self.calibration.get("ptz_fomako_port", 5678)
            )
            drv = ViscaOverIPDriver(
                fomako_ip or "127.0.0.1",
                port=port,
                raw_udp=True,
                steps_per_deg=steps_deg,
                pan_offset_deg=pan_off,
                tilt_offset_deg=tilt_off,
                max_zoom_x=max_z,
                calibration=self.calibration,
            )
            if fomako_ip or drv.usb_ready:
                self.hw_driver = drv
                self.hw_driver.configure_outdoor_window_defaults()
        elif visca_ip:
            port = int(os.environ.get("PTZ_VISCA_PORT", "52381"))
            self.hw_driver = ViscaOverIPDriver(
                visca_ip,
                port=port,
                raw_udp=(port in (1259, 5678)),
                steps_per_deg=steps_deg if port in (1259, 5678) else 1440.0,
                pan_offset_deg=pan_off,
                tilt_offset_deg=tilt_off,
                max_zoom_x=max_z,
                calibration=self.calibration,
            )
            self.hw_driver.configure_outdoor_window_defaults()
        elif axis_ip:
            user = os.environ.get("PTZ_AXIS_USER", "root")
            pw = os.environ.get("PTZ_AXIS_PASS", "")
            self.hw_driver = AxisVapixDriver(axis_ip, user, pw)
            self.hw_driver.lock_focus_infinity()

    def pixel_uv_to_pan_tilt(self, u_norm: float, v_norm: float):
        cal = self.calibration
        half_tan_x = math.tan(math.radians(cal["hfov_deg"] * 0.5))
        half_tan_y = half_tan_x * (9.0 / 16.0)
        xd = (u_norm - 0.5) * 2.0 * half_tan_x
        yd = (0.5 - v_norm) * 2.0 * half_tan_y
        k1 = float(cal.get("k1", 0.0))
        xn, yn = xd, yd
        for _ in range(3):
            f = max(0.5, 1.0 + k1 * (xn * xn + yn * yn))
            xn, yn = xd / f, yd / f
        pan_deg = math.degrees(math.atan(xn))
        tilt_deg = math.degrees(math.atan2(yn, math.sqrt(1.0 + xn * xn)))
        return pan_deg, tilt_deg

    def estimate_ray_distance_km(self, pan_deg: float, tilt_deg: float) -> float:
        """
        Computes exact 3D slant distance (km) along a Synoptic ray `(pan_deg, tilt_deg)`:
          - Below the horizon (`up_dir < -0.012`): intersects with the Hudson River surface (`z = 0m` MSL)
            from camera altitude `alt_m` (e.g. `0.40 km` on Little Island, `0.85 km` mid-channel).
          - Near/above the horizon: returns `4.0 km` optical skyline infinity.
        """
        cal = self.calibration
        el = math.radians(float(cal.get("tilt_deg", 3.171)))
        roll = math.radians(float(cal.get("roll_deg", -0.945)))
        alt_m = max(10.0, float(cal.get("alt_m", 86.0)))

        # Upward (+U) component of Synoptic camera basis vectors in ENU
        fz = math.sin(el)
        uz0 = math.cos(el)
        cr, sr = math.cos(roll), math.sin(roll)
        rz = -sr * uz0
        uz = cr * uz0

        pr = math.radians(pan_deg)
        tr = math.radians(tilt_deg)
        xc = math.sin(pr) * math.cos(tr)
        yc = math.sin(tr)
        zc = math.cos(pr) * math.cos(tr)

        up_dir = xc * rz + yc * uz + zc * fz
        if up_dir < -0.012:
            dist_km = (alt_m / (-up_dir)) / 1000.0
            return max(0.22, min(5.0, dist_km))
        return 4.0

    def run_constrained_focus_trim_async(self):
        """
        Runs a fast, strictly distance-constrained micro-focus trim (`[-45..+45]` ticks around
        the 3D distance-predicted `focus-abs`, NEVER unconstrained `auto-focus=1`) so window glass
        can never capture focus while still correcting any thermal shift to sub-step precision.
        """
        drv = self.hw_driver
        if not drv or not isinstance(drv, ViscaOverIPDriver) or self.calibrating:
            return
        if getattr(self, "_focus_trimming", False):
            return
        self._focus_trimming = True

        def _worker():
            import subprocess
            import tempfile

            vis_bin = BASE_DIR / "ptz-vision"
            try:
                drv._send_usb_uvc("auto-focus=0")
                drv.focus_mode = "calibrated_curve"
                if drv.last_zoom_pos is not None:
                    z_cur = drv.wait_zoom_settled(drv.last_zoom_pos, timeout=4.5)
                else:
                    q = drv.query_usb_uvc("zoom-abs")
                    z_cur = int(q) if (q and q.lstrip("-").isdigit()) else 0
                drv.actual_zoom_pos = z_cur
                dist_km = drv.last_dist_km or 3.0
                f_base = drv.compute_calibrated_focus_abs(z_cur, dist_km, include_trim=False)
                if not vis_bin.is_file() or not self.obs_cam.get_status().get("ptz_available"):
                    drv.set_focus_trim_offset(0)
                    drv.wait_focus_settled(f_base)
                    return

                scores = []
                with tempfile.TemporaryDirectory(prefix="ptz_ftrim_") as td:
                    with self.obs_cam.open_calibration_session(timeout=4.0) as sess:
                        for df in (-44, -22, 0, 22, 44):
                            fv = max(120, min(3350, f_base + df))
                            fhw = drv.wait_focus_settled(fv, timeout=1.4)
                            p = f"{td}/f_{df}.jpg"
                            sess.save_ptz_jpeg(p, width=960, height=540, quality=86)
                            st = json.loads(
                                subprocess.check_output(
                                    [str(vis_bin), "stats", p, "0.5", "0.5", "0.32"],
                                    text=True,
                                    timeout=4.0,
                                )
                            )
                            scores.append((fhw, float(st.get("tenengrad", 0.0))))

                scores.sort(key=lambda x: x[0])
                bi = max(range(len(scores)), key=lambda i: scores[i][1])
                best_f, best_ten = scores[bi]
                # Only apply contrast micro-trim if genuine optical texture exists (not featureless sky)
                if best_ten >= 0.10:
                    if 0 < bi < len(scores) - 1:
                        fl, tl = scores[bi - 1]
                        fc, tc = scores[bi]
                        fr, tr = scores[bi + 1]
                        denom = 2.0 * (tl - 2.0 * tc + tr)
                        if (fc - fl) > 0 and (fr - fc) > 0 and abs(denom) > 1e-5:
                            frac = max(-0.5, min(0.5, (tl - tr) / denom))
                            best_f = int(round(fc + frac * 0.5 * ((fc - fl) + (fr - fc))))
                    trim = max(-55, min(55, best_f - f_base))
                    drv.set_focus_trim_offset(trim)
                    drv.wait_focus_settled(f_base + trim, timeout=1.4)
                else:
                    drv.wait_focus_settled(f_base + drv.focus_trim_offset, timeout=1.4)
            except Exception:
                pass
            finally:
                self._focus_trimming = False

        threading.Thread(target=_worker, daemon=True).start()

    def save_calibration(self, updates: dict):
        """Persists updated camera pose / PTZ 3D bore-sight / focus curve / OBS URL to `calibration.json`."""
        with self.lock:
            for k in (
                "lat",
                "lon",
                "alt_m",
                "az_deg",
                "tilt_deg",
                "roll_deg",
                "hfov_deg",
                "k1",
                "adsb_lead_s",
                "ptz_offset_east_m",
                "ptz_offset_north_m",
                "ptz_offset_up_m",
                "ptz_pan_offset_deg",
                "ptz_tilt_offset_deg",
                "ptz_roll_offset_deg",
                "ptz_pan_scale",
                "ptz_tilt_scale",
                "ptz_collimation_pan_10x_deg",
                "ptz_collimation_tilt_10x_deg",
                "ptz_steps_per_deg",
                "ptz_max_zoom_x",
            ):
                if k in updates and updates[k] is not None:
                    self.calibration[k] = float(updates[k])
            for obj_k in (
                "ptz_zoom_hfov_table",
                "ptz_focus_model",
                "ptz_day_exposure",
                "ptz_last_calibration",
            ):
                if obj_k in updates and updates[obj_k] is not None:
                    self.calibration[obj_k] = updates[obj_k]
            if "ptz_auto_recalibrate_daily" in updates and updates["ptz_auto_recalibrate_daily"] is not None:
                self.auto_recalibrate_daily = bool(updates["ptz_auto_recalibrate_daily"])
                self.calibration["ptz_auto_recalibrate_daily"] = self.auto_recalibrate_daily
            if "ptz_fomako_ip" in updates:
                self.calibration["ptz_fomako_ip"] = str(updates["ptz_fomako_ip"]).strip()
            if "ptz_fomako_port" in updates and updates["ptz_fomako_port"]:
                self.calibration["ptz_fomako_port"] = int(updates["ptz_fomako_port"])
            if "ptz_sony_host" in updates and updates["ptz_sony_host"] is not None:
                self.calibration["ptz_sony_host"] = str(updates["ptz_sony_host"]).strip()
            if "ptz_sony_user" in updates and updates["ptz_sony_user"] is not None:
                self.calibration["ptz_sony_user"] = str(updates["ptz_sony_user"]).strip()
            if "ptz_youtube_enabled" in updates and updates["ptz_youtube_enabled"] is not None:
                self.calibration["ptz_youtube_enabled"] = bool(updates["ptz_youtube_enabled"])
            if "monthly_youtube_enabled" in updates and updates["monthly_youtube_enabled"] is not None:
                self.calibration["monthly_youtube_enabled"] = bool(updates["monthly_youtube_enabled"])

            # Keep active camera profile inside `ptz_profiles` synchronized with top-level keys
            active_model = str(self.calibration.get("ptz_camera_model", "sony_a40"))
            prof = self.calibration.setdefault("ptz_profiles", {}).setdefault(active_model, {})
            for k in self.PTZ_PROFILE_KEYS:
                if k in self.calibration:
                    prof[k] = json.loads(json.dumps(self.calibration[k]))

            if self.hw_driver and isinstance(self.hw_driver, (ViscaOverIPDriver, SonySRGA40Driver)):
                self.hw_driver.apply_calibration_dict(self.calibration)
            new_obs_url = updates.get("obs_url") if "obs_url" in updates else updates.get("remote_cam_url")
            if new_obs_url is not None:
                clean_url = str(new_obs_url).strip()
                self.calibration["obs_url"] = clean_url
                self.calibration.pop("remote_cam_url", None)
                self.obs_cam.set_url(clean_url)
            CALIB_PATH.write_text(json.dumps(self.calibration, indent=2) + "\n")

    def ingest_cv_detections(self, detections: list):
        now = time.time()
        with self.lock:
            seen_ids = set()
            for d in detections:
                cid = f"cv:{d.get('id', '0')}"
                seen_ids.add(cid)
                self.cv_tracks[cid] = {
                    "id": cid,
                    "u": float(d["u"]),
                    "v": float(d["v"]),
                    "w": float(d.get("w", 0.02)),
                    "h": float(d.get("h", 0.02)),
                    "du_s": float(d.get("du_s", 0.0)),
                    "dv_s": float(d.get("dv_s", 0.0)),
                    "region": d.get("region", "water"),
                    "label": d.get("label", ""),
                    "ts": now,
                }
            # Expire stale CV tracks older than 1.8s
            self.cv_tracks = {
                k: v for k, v in self.cv_tracks.items() if now - v["ts"] < 1.8
            }

    def poll_adsb_loop(self):
        local_json_file = Path.home() / "Library" / "Application Support" / "HudsonRiverADSB" / "json" / "aircraft.json"
        local_ep = ("local_readsb", "http://127.0.0.1:8080/data/aircraft.json")
        cloud_eps = [
            ("adsb.fi", "https://opendata.adsb.fi/api/v2/lat/40.71/lon/-74.02/dist/15"),
            ("adsb.lol", "https://api.adsb.lol/v2/point/40.71/-74.02/15"),
            ("airplanes.live", "https://api.airplanes.live/v2/point/40.71/-74.02/15"),
        ]
        cloud_idx = 0
        while True:
            fetched = None
            src_used = "offline"
            # 1) Prefer direct local 1090 MHz USB SDR (`live.hudsonriver.readsb` 5 Hz JSON file)
            try:
                if local_json_file.is_file() and (time.time() - local_json_file.stat().st_mtime) < 4.0:
                    data = json.loads(local_json_file.read_text(encoding="utf-8"))
                    ac_list = data.get("aircraft")
                    if ac_list is not None:
                        fetched = ac_list
                        src_used = "local_readsb"
            except Exception:
                pass

            # 2) Otherwise check local HTTP 8080 or cloud ADS-B aggregators
            if fetched is None:
                ordered = [local_ep] + [cloud_eps[(cloud_idx + i) % len(cloud_eps)] for i in range(len(cloud_eps))]
                for name, url in ordered:
                    try:
                        req = urllib.request.Request(
                            url,
                            headers={"User-Agent": "HudsonPTZ-PoC/1.0", "Accept": "application/json"},
                        )
                        timeout = 0.25 if name == "local_readsb" else 3.5
                        raw = urllib.request.urlopen(req, timeout=timeout).read().decode("utf-8")
                        data = json.loads(raw)
                        ac_list = data.get("ac")
                        if ac_list is None:
                            ac_list = data.get("aircraft") or []
                        fetched = ac_list
                        src_used = name
                        if name != "local_readsb":
                            for i, (cn, _) in enumerate(cloud_eps):
                                if cn == name:
                                    cloud_idx = i
                                    break
                        break
                    except Exception:
                        continue

            if fetched is not None:
                now = time.time()
                parsed = []
                for ac in fetched:
                    lat = ac.get("lat")
                    lon = ac.get("lon")
                    alt_geom = ac.get("alt_geom")
                    alt_baro = ac.get("alt_baro")
                    # Reject aircraft explicitly reporting on-ground status (even if alt_geom + geoid > 30 ft)
                    if isinstance(alt_baro, str) and alt_baro.strip().lower() == "ground":
                        continue
                    src_type = (ac.get("type") or "").strip().lower()
                    if src_type in ("tisb_other", "tisb_trackfile", "adsr_other", "mlat"):
                        continue
                    if src_type == "mode_s" and src_used != "local_readsb":
                        continue
                    hex_id = (ac.get("hex") or "unk").strip().lower()
                    if hex_id.startswith("~"):
                        continue
                    seen_pos = float(ac.get("seen_pos") or 0.0)
                    if seen_pos > 15.0:
                        continue
                    nic = ac.get("nic")
                    nac_p = ac.get("nac_p")
                    if (isinstance(nic, int) and 0 < nic < 5) or (isinstance(nac_p, int) and 0 < nac_p < 6):
                        continue
                    if isinstance(alt_geom, (int, float)):
                        # Convert WGS84 ellipsoid height (HAE) to Mean Sea Level (MSL) in NYC (+108.3 ft)
                        alt_msl_ft = float(alt_geom) + NYC_GEOID_OFFSET_FT
                    elif isinstance(alt_baro, (int, float)):
                        alt_msl_ft = float(alt_baro)
                    else:
                        continue
                    if lat is None or lon is None:
                        continue
                    if alt_msl_ft < 30 or alt_msl_ft > 18000:
                        continue

                    display_alt_ft = int(round(alt_baro)) if isinstance(alt_baro, (int, float)) else int(round(alt_msl_ft))

                    tcode = (ac.get("t") or "").strip().upper()
                    cat = (ac.get("category") or "").strip().upper()
                    desc = (ac.get("desc") or "").strip()
                    fallback_name = desc or (f"Aircraft ({tcode})" if tcode else "Aircraft")
                    type_name, length_m, is_heli = AIRCRAFT_TYPES.get(
                        tcode, (fallback_name, 28.0, cat == "A7")
                    )
                    if cat == "A7":
                        is_heli = True
                        if length_m > 20.0:
                            length_m = 13.5

                    flight = (ac.get("flight") or "").strip()
                    reg = (ac.get("r") or "").strip()
                    callsign = flight or reg or hex_id.upper()
                    rssi_val = ac.get("rssi")
                    rf_suffix = (
                        f" ({rssi_val:.0f} dBFS)"
                        if (src_used == "local_readsb" and isinstance(rssi_val, (int, float)))
                        else ""
                    )
                    subtitle = f"{type_name}" + (f" • {reg}" if reg and reg != callsign else "") + rf_suffix

                    vrate_fpm = ac.get("geom_rate") or ac.get("baro_rate") or 0.0
                    parsed.append({
                        "id": f"adsb:{hex_id}",
                        "kind": "helicopter" if is_heli else "aircraft",
                        "source": src_used,
                        "callsign": callsign,
                        "subtitle": subtitle,
                        "tcode": tcode,
                        "lat": float(lat),
                        "lon": float(lon),
                        "alt_m": alt_msl_ft * 0.3048,
                        "alt_ft": display_alt_ft,
                        "gs_kts": float(ac.get("gs") or 0.0),
                        "track_deg": float(ac.get("track") or ac.get("true_heading") or 0.0),
                        "vrate_mps": float(vrate_fpm) * 0.00508 if isinstance(vrate_fpm, (int, float)) else 0.0,
                        "length_m": length_m,
                        "rarity": 2.8 if is_heli else 1.2,
                        "seen_pos": seen_pos,
                        "pos_epoch": now - min(15.0, max(0.0, seen_pos)),
                    })
                with self.lock:
                    self.raw_adsb = parsed
                    self.raw_adsb_ts = now
                    self.adsb_source = src_used
                    for obs in parsed:
                        self._ingest_kinematic_obs(obs, now, is_vessel=False)

            time.sleep(0.25 if src_used == "local_readsb" else 1.5)

    def _ingest_kinematic_obs(self, obs: dict, now: float, is_vessel: bool = False):
        """
        Anisotropic 3D Constant-Turn-Rate & Velocity (CTRV) Kalman / alpha-beta filter.
        Heavily damps along-track ADS-B seen_pos / aggregator timing jitter while keeping
        cross-track lateral turns responsive and coasting smoothly across RF dropouts.
        """
        tid = obs["id"]
        spd_mps = 0.0 if (is_vessel and (obs.get("moored") or obs["gs_kts"] < 0.6)) else obs["gs_kts"] * 0.514444
        trk_rad = math.radians(obs["track_deg"])
        ve_m = spd_mps * math.sin(trk_rad)
        vn_m = spd_mps * math.cos(trk_rad)
        vu_m = obs.get("vrate_mps", 0.0)

        obs_rx = float(obs.get("rx_epoch", now))
        seen_pos = float(obs.get("seen_pos", max(0.0, now - obs.get("pos_epoch", now))))
        e_raw, n_raw, u_raw = lla_to_enu(
            obs["lat"], obs["lon"], obs["alt_m"],
            TRACK_REF_LAT, TRACK_REF_LON, TRACK_REF_ALT,
        )

        tr = self.kinematic_tracks.get(tid)
        if tr is None:
            # For newly-heard slow/nearshore vessels, avoid big unverified single-packet SOG leaps
            max_init_age = 5.0 if not is_vessel else (4.0 if spd_mps < 3.5 else 15.0)
            obs_age = min(max_init_age, max(0.0, seen_pos))
            self.kinematic_tracks[tid] = {
                "meta": dict(obs),
                "is_vessel": is_vessel,
                "e": e_raw + ve_m * obs_age,
                "n": n_raw + vn_m * obs_age,
                "u": u_raw + vu_m * obs_age,
                "ve": ve_m,
                "vn": vn_m,
                "vu": vu_m,
                "omega": 0.0,
                "t_state": now,
                "raw_lat": obs["lat"],
                "raw_lon": obs["lon"],
                "raw_e": e_raw,
                "raw_n": n_raw,
                "last_obs_epoch": obs.get("pos_epoch", now),
                "last_rx_epoch": obs_rx,
                "last_new_pos_ts": now - obs_age,
                "last_hdg": trk_rad,
                "last_hdg_ts": now,
                "pos_hist": deque([(obs_rx, e_raw, n_raw)], maxlen=36),
                "static_moored": bool(is_vessel and spd_mps < 0.4),
                "has_disp_window": False,
            }
            return

        # Always refresh display metadata (callsign, alt_ft, vessel destination, etc.)
        tr["meta"] = dict(obs)
        pos_changed = (
            abs(obs["lat"] - tr["raw_lat"]) > 1e-6
            or abs(obs["lon"] - tr["raw_lon"]) > 1e-6
        )
        if is_vessel:
            pos_changed = pos_changed or (obs_rx > tr.get("last_rx_epoch", 0.0) + 0.2)

        if not pos_changed or (not is_vessel and seen_pos > 12.0):
            # Duplicate cached ADS-B squitter: only advance state if refreshing live Doppler velocity
            if not is_vessel and spd_mps > 1.5 and seen_pos < 6.0:
                dt_step = max(0.0, min(10.0, now - tr["t_state"]))
                pe, pn, pu, pve, pvn, pvu = step_ctrv(
                    tr["e"], tr["n"], tr["u"], tr["ve"], tr["vn"], tr["vu"], tr["omega"], dt_step
                )
                tr["omega"] *= math.exp(-dt_step / 9.0)
                tr["e"], tr["n"], tr["u"] = pe, pn, pu
                tr["ve"] = 0.75 * pve + 0.25 * ve_m
                tr["vn"] = 0.75 * pvn + 0.25 * vn_m
                tr["vu"] = 0.75 * pvu + 0.25 * vu_m
                tr["t_state"] = now
            return

        dt_step = max(0.0, min(35.0, now - tr["t_state"]))
        pe, pn, pu, pve, pvn, pvu = step_ctrv(
            tr["e"], tr["n"], tr["u"], tr["ve"], tr["vn"], tr["vu"], tr["omega"], dt_step
        )
        tr["omega"] *= math.exp(-dt_step / 9.0)

        # Pier GPS multipath & static-lat/lon guard for AIS vessels:
        # Docked ferries/tugs next to tall steel piers often transmit 1.5–6 kt Doppler-multipath SOG
        # even while their (lat, lon) stays inside a ~20 m mooring circle across 14–60 seconds!
        if is_vessel:
            phist: deque = tr.setdefault("pos_hist", deque(maxlen=36))
            if not phist or (obs_rx - phist[-1][0]) >= 0.8 or math.hypot(e_raw - phist[-1][1], n_raw - phist[-1][2]) >= 1.0:
                phist.append((obs_rx, e_raw, n_raw))

            win_anchor = None
            for h_ts, h_e, h_n in phist:
                dt_h = obs_rx - h_ts
                if 14.0 <= dt_h <= 75.0:
                    win_anchor = (dt_h, h_e, h_n)
                    break
            if win_anchor is None and len(phist) >= 2 and (obs_rx - phist[0][0]) >= 14.0:
                win_anchor = (obs_rx - phist[0][0], phist[0][1], phist[0][2])

            dt_pkt = max(1.0, obs_rx - tr.get("last_rx_epoch", now - 10.0))
            raw_disp = math.hypot(e_raw - tr.get("raw_e", e_raw), n_raw - tr.get("raw_n", n_raw))
            prev_vh = math.hypot(tr["ve"], tr["vn"])

            is_static_pos = False
            if win_anchor is not None:
                dt_win, old_e, old_n = win_anchor
                win_disp_m = math.hypot(e_raw - old_e, n_raw - old_n)
                win_mps = win_disp_m / max(1.0, dt_win)
                tr["has_disp_window"] = True
                # Static if net movement over >=14s is < 20 m (or < 0.38 m/s = 0.74 kt)
                if win_disp_m < 20.0 or win_mps < 0.38:
                    is_static_pos = True
                elif spd_mps > win_mps * 1.65 + 0.75:
                    # Clamp inflated AIS Doppler SOG to actual ground displacement vector
                    ve_m = (e_raw - old_e) / dt_win
                    vn_m = (n_raw - old_n) / dt_win
                    spd_mps = math.hypot(ve_m, vn_m)

            if spd_mps == 0.0 or is_static_pos or (prev_vh < 0.45 and raw_disp < 22.0 and spd_mps < 2.2):
                # Stationary / moored vessel: suppress pier multipath SOG completely
                tr["static_moored"] = True
                tr["e"] = 0.75 * tr["e"] + 0.25 * e_raw if raw_disp < 55.0 else e_raw
                tr["n"] = 0.75 * tr["n"] + 0.25 * n_raw if raw_disp < 55.0 else n_raw
                tr["u"] = u_raw
                tr["ve"], tr["vn"], tr["vu"] = 0.0, 0.0, 0.0
                tr["omega"] = 0.0
                tr["t_state"] = now
                tr["raw_lat"], tr["raw_lon"] = obs["lat"], obs["lon"]
                tr["raw_e"], tr["raw_n"] = e_raw, n_raw
                tr["last_obs_epoch"] = obs.get("pos_epoch", now)
                tr["last_rx_epoch"] = obs_rx
                tr["last_new_pos_ts"] = now
                return

            tr["static_moored"] = False
            implied_mps = raw_disp / dt_pkt
            if dt_pkt >= 12.0 and spd_mps > implied_mps * 1.8 + 1.2:
                scale = (implied_mps * 1.25 + 0.4) / spd_mps
                ve_m *= scale
                vn_m *= scale
                spd_mps *= scale

        # Genuinely new underway position squitter received!
        obs_age = min(4.5 if not is_vessel else 15.0, max(0.0, seen_pos))
        obs_e = e_raw + ve_m * obs_age
        obs_n = n_raw + vn_m * obs_age
        obs_u = u_raw + vu_m * obs_age

        re = obs_e - pe
        rn = obs_n - pn
        ru = obs_u - pu
        err_h = math.hypot(re, rn)

        if err_h > (1600.0 if not is_vessel else 450.0):
            # Large discontinuity / teleport: reset cleanly
            tr.update({
                "e": obs_e, "n": obs_n, "u": obs_u,
                "ve": ve_m, "vn": vn_m, "vu": vu_m,
                "omega": 0.0, "t_state": now,
                "raw_lat": obs["lat"], "raw_lon": obs["lon"],
                "raw_e": e_raw, "raw_n": n_raw,
                "last_obs_epoch": obs.get("pos_epoch", now),
                "last_rx_epoch": obs_rx,
                "last_new_pos_ts": now,
                "last_hdg": trk_rad, "last_hdg_ts": now,
            })
            return

        # Estimate smoothed turn rate for aircraft/helicopters
        if not is_vessel and spd_mps > 8.0:
            dt_hdg = now - tr["last_hdg_ts"]
            if 0.5 <= dt_hdg <= 14.0:
                dhdg = (trk_rad - tr["last_hdg"] + math.pi) % (2.0 * math.pi) - math.pi
                om_meas = max(-0.075, min(0.075, dhdg / dt_hdg))
                tr["omega"] = 0.70 * tr["omega"] + 0.30 * om_meas
            tr["last_hdg"] = trk_rad
            tr["last_hdg_ts"] = now
        else:
            tr["omega"] = 0.0

        vh_ref = math.hypot(ve_m, vn_m)
        if vh_ref > 2.5:
            he, hn = ve_m / vh_ref, vn_m / vh_ref
            # Decompose residual into along-track (timing jitter) & cross-track (lateral turn)
            r_par = re * he + rn * hn
            r_perp = -re * hn + rn * he
            r_par_v = max(-250.0, min(250.0, r_par))
            r_perp_v = max(-250.0, min(250.0, r_perp))
            a_par = 0.16 if not is_vessel else 0.28
            a_perp = 0.46 if not is_vessel else 0.48
            b_par = 0.020 if not is_vessel else 0.008
            b_perp = 0.045 if not is_vessel else 0.015

            tr["e"] = pe + a_par * r_par * he - a_perp * r_perp * hn
            tr["n"] = pn + a_par * r_par * hn + a_perp * r_perp * he
            tr["ve"] = 0.58 * pve + 0.42 * ve_m + b_par * r_par_v * he - b_perp * r_perp_v * hn
            tr["vn"] = 0.58 * pvn + 0.42 * vn_m + b_par * r_par_v * hn + b_perp * r_perp_v * he
        else:
            # Hovering helicopter or moored/slow vessel
            tr["e"] = pe + 0.25 * re
            tr["n"] = pn + 0.25 * rn
            tr["ve"] = 0.65 * pve + 0.35 * ve_m
            tr["vn"] = 0.65 * pvn + 0.35 * vn_m

        tr["u"] = pu + (0.28 * ru if not is_vessel else 0.5 * ru)
        tr["vu"] = 0.60 * pvu + 0.40 * vu_m + (0.020 * ru if not is_vessel else 0.0)
        tr["t_state"] = now
        tr["raw_lat"] = obs["lat"]
        tr["raw_lon"] = obs["lon"]
        tr["raw_e"] = e_raw
        tr["raw_n"] = n_raw
        tr["last_obs_epoch"] = obs.get("pos_epoch", now)
        tr["last_rx_epoch"] = obs_rx
        tr["last_new_pos_ts"] = now

    def poll_local_ais_loop(self):
        """Polls AIS-catcher REST v1 API on localhost (when running on Mac mini) or LAN Mac mini."""
        env_url = os.environ.get("AIS_CATCHER_URL", "").strip()
        candidates = (
            [env_url]
            if env_url
            else [
                "http://127.0.0.1:8100/api/v1/vessels?limit=250&max_age_s=300",
                "http://craigs-mac-mini.local:8100/api/v1/vessels?limit=250&max_age_s=300",
            ]
        )
        active_idx = 0
        while True:
            fetched = False
            for offset in range(len(candidates)):
                idx = (active_idx + offset) % len(candidates)
                url = candidates[idx]
                try:
                    req = urllib.request.Request(url)
                    data = json.loads(urllib.request.urlopen(req, timeout=0.9).read().decode("utf-8"))
                    ships = data.get("data") if isinstance(data.get("data"), list) else (data.get("ships") or [])
                    active_idx = idx
                    fetched = True
                    now = time.time()
                    static_lookup = {}
                    if self.aisstream:
                        with self.aisstream.lock:
                            static_lookup = dict(self.aisstream.static_cache)

                    parsed = []
                    for s in ships:
                        lat, lon = s.get("lat"), s.get("lon")
                        if lat is None or lon is None or abs(lat) > 90 or abs(lon) > 180:
                            continue
                        mmsi = str(s.get("mmsi", "")).strip()
                        if not mmsi:
                            continue

                        st = dict(static_lookup.get(mmsi) or {"name": "", "type_code": 0, "length_m": 0.0, "dest": ""})
                        raw_name = (s.get("name") or s.get("shipname") or "").strip()
                        raw_dest = (s.get("destination") or "").strip()
                        raw_type = s.get("ship_type") or s.get("shiptype") or 0
                        bow = s.get("to_bow_m") if s.get("to_bow_m") is not None else s.get("to_bow")
                        stern = s.get("to_stern_m") if s.get("to_stern_m") is not None else s.get("to_stern")
                        raw_len = float((bow or 0) + (stern or 0))

                        updated_static = False
                        if raw_name and raw_name != st.get("name"):
                            st["name"] = raw_name
                            updated_static = True
                        if raw_type and int(raw_type) != st.get("type_code", 0):
                            st["type_code"] = int(raw_type)
                            updated_static = True
                        if raw_len >= 6.0 and raw_len != st.get("length_m", 0.0):
                            st["length_m"] = raw_len
                            updated_static = True
                        if raw_dest and raw_dest != st.get("dest", ""):
                            st["dest"] = raw_dest
                            updated_static = True
                        if updated_static and self.aisstream:
                            with self.aisstream.lock:
                                self.aisstream.static_cache[mmsi] = st

                        nav_status = s.get("nav_status")
                        is_moored = nav_status in ("at_anchor", "moored", "aground", 1, 5, 6)

                        spd = s.get("speed_kn") if "speed_kn" in s else s.get("sog")
                        sog = float(spd) if spd is not None else 0.0
                        if sog > 100.0 or is_moored:
                            sog = 0.0

                        crs = s.get("course_deg") if "course_deg" in s else s.get("cog")
                        if crs is None or float(crs) >= 360.0:
                            hdg = s.get("heading_deg")
                            cog = float(hdg) if (hdg is not None and float(hdg) < 360.0) else 0.0
                        else:
                            cog = float(crs)

                        # Track genuine packet arrivals so CTRV smoother coasts smoothly between VHF squitters
                        msg_cnt = int(s.get("messages") or 0)
                        age_s = float(s.get("age_s") or 0.0)
                        last_seen_iso = s.get("last_seen")
                        pkt_pos_epoch = max(0.0, now - age_s)
                        if isinstance(last_seen_iso, str) and last_seen_iso.endswith("Z"):
                            try:
                                pkt_pos_epoch = datetime.strptime(
                                    last_seen_iso, "%Y-%m-%dT%H:%M:%SZ"
                                ).replace(tzinfo=timezone.utc).timestamp()
                            except Exception:
                                pass

                        prev_pkt = self.local_ais_pkt_state.get(mmsi)
                        if (
                            prev_pkt is None
                            or msg_cnt != prev_pkt[0]
                            or abs(float(lat) - prev_pkt[3]) > 1e-6
                            or abs(float(lon) - prev_pkt[4]) > 1e-6
                        ):
                            rx_epoch = now
                            pos_epoch = min(now, pkt_pos_epoch)
                            self.local_ais_pkt_state[mmsi] = (msg_cnt, rx_epoch, pos_epoch, float(lat), float(lon))
                        else:
                            _, rx_epoch, pos_epoch, _, _ = prev_pkt

                        callsign = st["name"] or f"VESSEL {mmsi}"
                        if is_blacklisted_vessel(mmsi, callsign):
                            continue
                        si_match = identify_staten_island_ferry(mmsi, callsign)
                        if si_match is not None:
                            callsign = si_match[0]
                        type_label, est_len, rarity = classify_ais_ship_type(
                            int(st.get("type_code") or 0),
                            float(st.get("length_m") or 0.0),
                            callsign,
                            mmsi=mmsi,
                        )
                        if sog < 0.5:
                            rarity *= 0.22
                        length_m = float(st["length_m"]) if float(st.get("length_m") or 0.0) >= 6.0 else est_len
                        dest_suffix = f" -> {st['dest']}" if st.get("dest") else ""
                        lvl = s.get("level_db")
                        rf_tag = f" ({lvl:.0f} dB)" if isinstance(lvl, (int, float)) else ""

                        parsed.append({
                            "id": f"ais:{mmsi}",
                            "mmsi": mmsi,
                            "kind": "vessel",
                            "source": "local_ais",
                            "callsign": callsign,
                            "subtitle": f"{type_label}{dest_suffix}{rf_tag}",
                            "lat": float(lat),
                            "lon": float(lon),
                            "alt_m": 2.5,
                            "gs_kts": round(sog, 1),
                            "track_deg": round(cog, 1),
                            "vrate_mps": 0.0,
                            "length_m": length_m,
                            "rarity": rarity,
                            "moored": is_moored,
                            "pos_epoch": pos_epoch,
                            "rx_epoch": rx_epoch,
                        })

                    with self.lock:
                        self.raw_ais = parsed
                        self.raw_ais_ts = now
                        self.ais_source = "local_ais"
                    break
                except Exception:
                    continue

            if not fetched:
                with self.lock:
                    if time.time() - self.raw_ais_ts > 12.0:
                        self.raw_ais = []
            time.sleep(1.0)

    def compute_snapshot(self, drive_hw: bool = True) -> dict:
        now = time.time()
        with self.lock:
            cal = dict(self.calibration)
            local_ais_active = (now - self.raw_ais_ts < 12.0) and len(self.raw_ais) > 0
            local_ais_items = list(self.raw_ais) if local_ais_active else []
            local_ais_count = len(local_ais_items)
            sim_items = []

            if local_ais_active:
                # Fuse zero-latency local VHF SDR vessels (strict priority!) with aisstream for shadowed outer harbor
                merged_by_id = {v["id"]: v for v in local_ais_items}
                if self.aisstream and self.aisstream.connected:
                    for sv in self.aisstream.snapshot_vessels():
                        if sv["id"] not in merged_by_id:
                            merged_by_id[sv["id"]] = sv
                ais_live_items = list(merged_by_id.values())
                ais_harbor_total = len(ais_live_items)
                ais_src = "local_ais"
            elif self.aisstream and self.aisstream.connected:
                ais_live_items = self.aisstream.snapshot_vessels()
                ais_harbor_total = len(ais_live_items)
                ais_src = "aisstream_live"
                if self.enable_sim_vessels:
                    sim_items = self.sim_fleet.get_vessels(now)
            elif self.enable_sim_vessels:
                ais_live_items = []
                sim_items = self.sim_fleet.get_vessels(now)
                ais_harbor_total = len(sim_items)
                ais_src = "simulated_hudson"
            else:
                ais_live_items = []
                ais_harbor_total = 0
                ais_src = "connecting" if self.aisstream else "off"
            cv_items = dict(self.cv_tracks)

            # Ingest live AIS observations ONLY when a genuinely new VHF packet arrived
            for v_obs in ais_live_items:
                rx_ep = v_obs.get("rx_epoch", v_obs.get("pos_epoch", 0.0))
                prev_tr = self.kinematic_tracks.get(v_obs["id"])
                if prev_tr is None or rx_ep > prev_tr.get("last_rx_epoch", 0.0) + 0.05:
                    self._ingest_kinematic_obs(v_obs, now, is_vessel=True)
                else:
                    prev_tr["meta"] = dict(v_obs)

            # Camera position in TRACK_REF ENU frame
            cam_e, cam_n, cam_u = lla_to_enu(
                cal["lat"], cal["lon"], cal["alt_m"],
                TRACK_REF_LAT, TRACK_REF_LON, TRACK_REF_ALT,
            )
            api_lead = 0.0 if self.adsb_source == "local_readsb" else float(cal.get("adsb_lead_s", 0.9))

            # Evaluate smoothed CTRV kinematic tracks + optional deterministic sim boats
            eval_candidates = []
            stale_ids = []
            for tid, tr in self.kinematic_tracks.items():
                item = tr["meta"]
                if tr["is_vessel"] and is_blacklisted_vessel(item.get("mmsi", ""), item.get("callsign", "")):
                    stale_ids.append(tid)
                    continue
                age_since_pos = now - tr["last_new_pos_ts"]
                if tr["is_vessel"]:
                    vh_tr = math.hypot(tr["ve"], tr["vn"])
                    # Confirmed underway channel ferries (>= 5.8 kt / 3.0 m/s) coast up to 180s across
                    # Class-B VHF gaps; slower/docking boats taper at 35-50s; moored boats stay at 0
                    if vh_tr >= 3.0:
                        max_age = 220.0
                        dt_raw = max(0.0, now - tr["t_state"])
                        dt_eval = min(180.0, dt_raw)
                    elif vh_tr >= 0.5:
                        max_age = 150.0
                        dt_raw = max(0.0, now - tr["t_state"])
                        dt_eval = dt_raw if dt_raw <= 35.0 else 35.0 + 15.0 * (1.0 - math.exp(-(dt_raw - 35.0) / 15.0))
                    else:
                        max_age = 480.0
                        dt_eval = 0.0
                    if age_since_pos > max_age:
                        stale_ids.append(tid)
                        continue
                    om_eff = 0.0
                else:
                    if age_since_pos > 24.0:
                        stale_ids.append(tid)
                        continue
                    dt_raw = max(0.0, now - tr["t_state"])
                    dt_eval = min(25.0, dt_raw + api_lead)
                    om_eff = tr["omega"] * math.exp(-dt_raw / 9.0)

                ke, kn, ku, kve, kvn, kvu = step_ctrv(
                    tr["e"], tr["n"], tr["u"],
                    tr["ve"], tr["vn"], tr["vu"],
                    om_eff, dt_eval,
                )
                if tr["is_vessel"] and tr.get("static_moored"):
                    continue
                eval_candidates.append((item, ke - cam_e, kn - cam_n, ku - cam_u, kve, kvn, kvu, bool(tr.get("has_disp_window"))))

            for tid in stale_ids:
                self.kinematic_tracks.pop(tid, None)

            for s_item in sim_items:
                spd_mps = s_item["gs_kts"] * 0.514444
                trk_rad = math.radians(s_item["track_deg"])
                ve = spd_mps * math.sin(trk_rad)
                vn = spd_mps * math.cos(trk_rad)
                e0, n0, u0 = lla_to_enu(
                    s_item["lat"], s_item["lon"], s_item["alt_m"],
                    cal["lat"], cal["lon"], cal["alt_m"],
                )
                eval_candidates.append((s_item, e0, n0, u0, ve, vn, 0.0, True))

            targets = []
            for item, east, north, up, ve, vn, vu, has_disp_win in eval_candidates:
                proj = project_target(east, north, up, ve, vn, vu, cal)
                if not proj:
                    continue

                u_n, v_n = proj["u"], proj["v"]
                if not (-0.06 <= u_n <= 1.06 and -0.06 <= v_n <= 1.06):
                    continue

                dist_m = proj["dist_m"]
                if dist_m > 16000:
                    continue

                if item["kind"] == "vessel":
                    is_si_ferry = "STATEN ISLAND FERRY" in str(item.get("callsign") or "")
                    max_v_dist = 13500.0 if (is_si_ferry or float(item.get("length_m") or 0.0) >= 55.0) else 8000.0
                    if dist_m > max_v_dist or item.get("moored"):
                        continue
                    est_lat = cal["lat"] + north / 111105.0
                    est_lon = cal["lon"] + east / 84300.0
                    if is_vessel_occluded_by_land(est_lat, est_lon, u_n, dist_m):
                        continue
                elif is_occluded_by_skyline(u_n, dist_m, cal["alt_m"] + up):
                    continue

                # Recommended zoom:
                # - Below SRG-A40's 0.50°/s hardware minimum motor speed (`0.02 <= v_dps < 0.48`), size FOV so
                #   the boat/plane glides across a single locked 15s cinema-tripod shot with ZERO move-stop-move!
                # - Above 0.48°/s (continuous velocity pursuit), frame generously so constant-speed pan stays silky.
                obj_angle_deg = max(0.08, math.degrees(2.0 * math.atan2(item["length_m"] * 0.5, dist_m)))
                v_dps = math.hypot(proj["pan_dps"], proj["tilt_dps"])
                if 0.02 <= v_dps < 0.48:
                    desired_hfov = max(obj_angle_deg / 0.34, v_dps * 18.5)
                elif v_dps >= 0.48:
                    desired_hfov = max(obj_angle_deg / 0.24, v_dps * 2.8)
                else:
                    desired_hfov = obj_angle_deg / 0.34
                rec_zoom = max(3.5, min(30.0, cal["hfov_deg"] / max(1.8, desired_hfov)))

                # Smoothed speed & track from filtered velocity (never fallback to raw AIS SOG once displacement window is established)
                filt_vh = math.hypot(ve, vn)
                filt_gs_kts = (
                    filt_vh / 0.514444
                    if (filt_vh > 0.4 or (item["kind"] == "vessel" and has_disp_win))
                    else item["gs_kts"]
                )
                if item["kind"] == "vessel" and filt_gs_kts < 0.8:
                    continue
                filt_trk_deg = (math.degrees(math.atan2(ve, vn)) + 360.0) % 360.0 if filt_vh > 1.0 else item["track_deg"]

                # Interest score for Auto-Director
                prox_factor = max(0.15, min(2.5, 2500.0 / max(400.0, dist_m)))
                motion_bonus = 1.25 if filt_gs_kts > 5.0 else 0.6
                corridor_bonus = 1.6 if (item["kind"] == "helicopter" and dist_m < 4500) else 1.0
                raw_interest = item["rarity"] * prox_factor * motion_bonus * corridor_bonus * 25.0
                interest = self._apply_revisit_cooldown(item["id"], raw_interest, now)

                targets.append({
                    "id": item["id"],
                    "kind": item["kind"],
                    "source": item["source"],
                    "callsign": item["callsign"],
                    "subtitle": item["subtitle"],
                    "u": round(u_n, 5),
                    "v": round(v_n, 5),
                    "du_s": round(proj["du_s"], 6),
                    "dv_s": round(proj["dv_s"], 6),
                    "pan_deg": round(proj["pan_deg"], 3),
                    "tilt_deg": round(proj["tilt_deg"], 3),
                    "pan_dps": round(proj["pan_dps"], 4),
                    "tilt_dps": round(proj["tilt_dps"], 4),
                    "dist_km": round(dist_m / 1000.0, 2),
                    "alt_ft": item.get("alt_ft", 0),
                    "gs_kts": round(filt_gs_kts, 1),
                    "track_deg": round(filt_trk_deg, 0),
                    "length_m": item["length_m"],
                    "rec_zoom_x": round(rec_zoom, 1),
                    "raw_interest": round(raw_interest, 1),
                    "interest": interest,
                    "cv_locked": False,
                })

            # Fuse CV detections: only nudge transponder position for neural YOLO or slow AIS vessels
            # (never yank smooth Kalman-filtered ADS-B aircraft with coarse 320x180 pixel blobs!)
            half_tan_x = math.tan(math.radians(cal["hfov_deg"] * 0.5))
            half_tan_y = half_tan_x * (9.0 / 16.0)

            if self.cv_mode != "off":
                for cid, cv in cv_items.items():
                    is_yolo = bool(cv.get("label"))
                    best_t = None
                    best_d = 0.032  # ~2 degrees association gate
                    for t in targets:
                        d = math.hypot(t["u"] - cv["u"], t["v"] - cv["v"])
                        if d < best_d:
                            best_d = d
                            best_t = t
                    if best_t is not None:
                        if is_yolo:
                            best_t["u"] = round(0.30 * cv["u"] + 0.70 * best_t["u"], 5)
                            best_t["v"] = round(0.30 * cv["v"] + 0.70 * best_t["v"], 5)
                        elif best_t["kind"] == "vessel":
                            best_t["u"] = round(0.15 * cv["u"] + 0.85 * best_t["u"], 5)
                            best_t["v"] = round(0.15 * cv["v"] + 0.85 * best_t["v"], 5)
                        best_t["cv_locked"] = True
                        if best_t["interest"] > 3.0:
                            best_t["interest"] = round(best_t["interest"] * 1.15, 1)
                    elif self.cv_mode == "strict" or is_yolo:
                        # Standalone visual target (only allowed in Strict mode or from neural YOLO)
                        pan_deg = math.degrees(math.atan((cv["u"] - 0.5) * 2.0 * half_tan_x))
                        tilt_deg = math.degrees(math.atan((0.5 - cv["v"]) * 2.0 * half_tan_y))
                        is_sky = cv["region"] == "sky"
                        max_dps = 4.5 if is_sky else 1.2
                        pan_dps = max(-max_dps, min(max_dps, cv["du_s"] * cal["hfov_deg"]))
                        tilt_dps = max(-max_dps, min(max_dps, -cv["dv_s"] * cal["hfov_deg"] * 0.5625))
                        custom_lbl = (cv.get("label") or "").upper()
                        targets.append({
                            "id": cid,
                            "kind": "helicopter" if is_sky else "vessel",
                            "source": "cv_yolo" if custom_lbl else "cv_spotter",
                            "callsign": f"VISUAL {custom_lbl}" if custom_lbl else ("VISUAL AIRCRAFT" if is_sky else "VISUAL WATERCRAFT"),
                            "subtitle": "YOLO Neural Detection (No Transponder)" if custom_lbl else ("Coherent Visual Mover (No ADS-B)" if is_sky else "Coherent Visual Mover (No AIS)"),
                            "u": round(cv["u"], 5),
                            "v": round(cv["v"], 5),
                            "du_s": round(cv["du_s"], 6),
                            "dv_s": round(cv["dv_s"], 6),
                            "pan_deg": round(pan_deg, 2),
                            "tilt_deg": round(tilt_deg, 2),
                            "pan_dps": round(pan_dps, 3),
                            "tilt_dps": round(tilt_dps, 3),
                            "dist_km": 1.8 if is_sky else round(self.estimate_ray_distance_km(pan_deg, tilt_deg), 2),
                            "alt_ft": 800 if is_sky else 0,
                            "gs_kts": 0.0,
                            "track_deg": 0,
                            "length_m": 18.0,
                            "rec_zoom_x": 16.0 if is_sky else 14.0,
                            "interest": self._apply_revisit_cooldown(cid, 34.0, now),
                            "cv_locked": True,
                        })

            presets_out = self._resolve_presets_locked(cal)
            headliner_order = {
                "statue_of_liberty": 0,
                "little_island": 1,
                "one_wtc": 2,
                "colgate_clock": 3,
                "whitney_museum": 4,
                "waters_soul": 5,
                "ellis_island": 6,
                "soccer_pitch": 7,
                "verrazzano": 8,
            }
            presets_sorted = sorted(presets_out, key=lambda item_p: headliner_order.get(item_p["id"], 99))
            preset_candidates = []
            for idx_p, pr in enumerate(presets_sorted):
                pid_full = f"preset:{pr['id']}"
                last_p_seen = self.cooldowns.get(pid_full, 0.0)
                since_p = now - last_p_seen
                if pid_full == self.active_target_id:
                    p_interest = 9.2
                elif last_p_seen == 0.0:
                    # Stagger initial priority slightly below fresh boats/planes (11+), led by Statue of Liberty & Little Island
                    p_interest = round(10.6 - idx_p * 0.14, 2)
                elif since_p < 300.0:
                    # Recently featured landmarks cool down for 5 minutes (always above cooling-down movers = 0.5)
                    p_interest = round(3.8 + (since_p / 300.0) * 5.2, 2)
                else:
                    p_interest = round(9.4 + min(1.4, (since_p - 300.0) / 180.0), 2)
                preset_candidates.append({
                    "id": pid_full,
                    "preset_id": pr["id"],
                    "kind": "landmark",
                    "source": "preset",
                    "callsign": pr["label"],
                    "subtitle": pr["subtitle"],
                    "u": pr["u"],
                    "v": pr["v"],
                    "du_s": 0.0,
                    "dv_s": 0.0,
                    "pan_deg": pr["pan_deg"],
                    "tilt_deg": pr["tilt_deg"],
                    "pan_dps": 0.0,
                    "tilt_dps": 0.0,
                    "dist_km": pr["dist_km"],
                    "alt_ft": 0,
                    "gs_kts": 0.0,
                    "track_deg": 0,
                    "length_m": pr["length_m"],
                    "rec_zoom_x": pr["zoom_x"],
                    "interest": p_interest,
                    "cv_locked": False,
                })

            synoptic_cand = {
                "id": "preset:__synoptic_1x__",
                "preset_id": "__synoptic_1x__",
                "kind": "synoptic",
                "source": "preset",
                "callsign": "HUDSON RIVER HARBOR OVERVIEW",
                "subtitle": "Wide 1× Optical Panorama • Lower Manhattan, New York Harbor & Jersey City",
                "u": 0.5,
                "v": 0.5,
                "du_s": 0.0,
                "dv_s": 0.0,
                "pan_deg": 0.0,
                "tilt_deg": 0.0,
                "pan_dps": 0.0,
                "tilt_dps": 0.0,
                "dist_km": 4.2,
                "alt_ft": 0,
                "gs_kts": 0.0,
                "track_deg": 0,
                "length_m": 1000.0,
                "rec_zoom_x": 1.0,
                "interest": 9.2 if self.active_target_id == "preset:__synoptic_1x__" else 2.0,
                "cv_locked": False,
            }

            targets.sort(key=lambda x: x["interest"], reverse=True)
            sun_elev_deg = solar_elevation_deg_utc(now, float(cal.get("lat", 40.744823)), float(cal.get("lon", -74.008063)))
            is_night_sky = sun_elev_deg < -2.5
            # Skip aircraft & helicopters at night (blinking strobes against dark sky)
            director_movers = [
                t
                for t in targets
                if not (is_night_sky and t.get("kind") in ("aircraft", "helicopter"))
            ]
            director_pool = sorted(
                director_movers + preset_candidates + [synoptic_cand],
                key=lambda x: x["interest"],
                reverse=True,
            )
            director_by_id = {c["id"]: c for c in director_pool}
            self.last_targets_by_id = director_by_id

            # Auto-expire public `/zoom` manual pointing lease back to Auto-Director after 90s idle
            if (
                not self.auto_mode
                and self.public_lease_until_ts > 0.0
                and now >= self.public_lease_until_ts
                and not self.stacking
            ):
                self.auto_mode = True
                self.active_celestial_id = None
                self.manual_point = None
                self.manual_zoom_override = None
                self.public_lease_until_ts = 0.0
                self.dwell_started_at = 0.0

            # Auto-Director selection logic:
            #   - Strict 50% Boats (plus daytime aircraft) / 50% Fixed Landmarks 1:1 alternation
            #   - No aircraft/helicopters at night (`is_night_sky`)
            #   - Periodic 1x Wide Synoptic Harbor Overview every ~4 minutes
            if self.active_target_id not in director_by_id:
                self.active_target_id = None

            def _choose_next_candidate(eval_ts: float, cur_cand: Optional[dict]) -> Optional[dict]:
                if not director_pool:
                    return None
                cur_id = cur_cand["id"] if cur_cand else None
                due_syn = (
                    (cur_cand is None or cur_cand.get("kind") != "synoptic")
                    and (eval_ts - self.last_synoptic_visit_ts) >= self.synoptic_interval_sec
                )
                if due_syn:
                    return synoptic_cand

                if cur_cand and cur_cand.get("kind") == "landmark":
                    prev_kind = "landmark"
                elif cur_cand and cur_cand.get("kind") in ("vessel", "aircraft", "helicopter"):
                    prev_kind = "mover"
                else:
                    prev_kind = self.last_non_synoptic_kind

                lm_sorted = [
                    c
                    for c in sorted(preset_candidates, key=lambda x: x["interest"], reverse=True)
                    if c["id"] != cur_id
                ]
                if not lm_sorted and preset_candidates:
                    lm_sorted = list(preset_candidates)

                fresh_movers = [
                    c
                    for c in sorted(director_movers, key=lambda x: x["interest"], reverse=True)
                    if c["id"] != cur_id
                    and (eval_ts - self.cooldowns.get(c["id"], 0.0)) >= self.mover_cooldown_sec
                    and (self.visit_seq - self.last_left_seq.get(c["id"], -99)) >= self.revisit_gap_visits
                ]
                relaxed_movers = [
                    c
                    for c in sorted(
                        director_movers,
                        key=lambda x: float(x.get("raw_interest") or x["interest"]),
                        reverse=True,
                    )
                    if c["id"] != cur_id and (eval_ts - self.cooldowns.get(c["id"], 0.0)) >= 20.0
                ]

                if prev_kind == "mover":
                    # After any boat/mover, pick a fixed landmark to maintain 50% boats / 50% landmarks
                    if lm_sorted:
                        return lm_sorted[0]
                    if fresh_movers:
                        return fresh_movers[0]
                    if relaxed_movers:
                        return relaxed_movers[0]
                else:
                    # After any fixed landmark, pick a boat (or daytime aircraft) to maintain 50/50 split
                    if fresh_movers:
                        return fresh_movers[0]
                    if relaxed_movers:
                        return relaxed_movers[0]
                    if lm_sorted:
                        return lm_sorted[0]

                return next((c for c in director_pool if c["id"] != cur_id), director_pool[0])

            def _initial_dwell_for(c_obj: Optional[dict]) -> float:
                """
                Computes the planned dwell duration ONCE when acquiring `c_obj` so instantaneous AIS/CV
                velocity fluctuations (`du_s`, `dv_s`) never cause the live countdown timer to jump up and down!
                """
                if not c_obj:
                    return self.max_dwell_sec
                kind_c = c_obj.get("kind")
                if kind_c == "synoptic":
                    return self.synoptic_dwell_sec
                if kind_c == "landmark":
                    return self.landmark_dwell_sec
                base_d = self.max_dwell_sec
                du = float(c_obj.get("du_s") or 0.0)
                dv = float(c_obj.get("dv_s") or 0.0)
                u0 = float(c_obj.get("u") or 0.5)
                v0 = float(c_obj.get("v") or 0.5)
                exit_s = 999.0
                if abs(du) > 1e-4:
                    t_u = ((1.04 - u0) / du) if du > 0.0 else ((-0.04 - u0) / du)
                    exit_s = min(exit_s, max(0.0, t_u))
                if abs(dv) > 1e-4:
                    t_v = ((1.04 - v0) / dv) if dv > 0.0 else ((-0.04 - v0) / dv)
                    exit_s = min(exit_s, max(0.0, t_v))
                if exit_s < 900.0:
                    return max(8.0, min(base_d, exit_s))
                return base_d

            cur_max_dwell = self.active_dwell_limit_sec
            next_target_preview = None
            switched_target_now = False

            if self.auto_mode:
                dwell_elapsed = now - self.dwell_started_at
                vt_st = self.visual_tracker.get_status()
                cur_act_cand = director_by_id.get(self.active_target_id) if self.active_target_id else None
                # Only rotate early for `reacquiring_rf` (lost an established lock behind an obstruction),
                # NEVER for `acquiring` (normal RF-tracked night vessels without a neural bbox lock!)
                long_occluded = (
                    cur_act_cand is not None
                    and cur_act_cand.get("kind") in ("vessel", "aircraft", "helicopter")
                    and vt_st.get("active_target_id") == self.active_target_id
                    and not vt_st.get("locked")
                    and dwell_elapsed >= 20.0
                    and vt_st.get("phase") == "reacquiring_rf"
                )
                if cur_act_cand and cur_act_cand.get("kind") == "landmark":
                    self.last_landmark_visit_ts = now
                if cur_act_cand and cur_act_cand.get("kind") == "synoptic":
                    self.last_synoptic_visit_ts = now
                if cur_act_cand and self.active_dwell_limit_sec <= 0.0:
                    self.active_dwell_limit_sec = _initial_dwell_for(cur_act_cand)
                cur_max_dwell = self.active_dwell_limit_sec if cur_act_cand else self.max_dwell_sec
                force_rotate = self.active_target_id is not None and (
                    dwell_elapsed >= cur_max_dwell or long_occluded
                )
                if self.active_target_id is None or force_rotate:
                    cand = _choose_next_candidate(now, cur_act_cand)
                    if cand:
                        if cand["id"] != self.active_target_id:
                            prev_pan = float(self.ptz.get_state().get("pan_deg", 0.0))
                            prev_tilt = float(self.ptz.get_state().get("tilt_deg", 0.0))
                            prev_zoom = float(
                                (self.hw_driver.get_state().get("zoom_x") if self.hw_driver else None)
                                or self.ptz.get_state().get("zoom_x")
                                or 1.0
                            )
                            ang_jump = math.hypot(float(cand["pan_deg"]) - prev_pan, float(cand["tilt_deg"]) - prev_tilt)
                            new_z = float(cand.get("rec_zoom_x") or 15.0)
                            zoom_jump_s = abs(new_z - prev_zoom) / 9.5 + 0.35 if abs(new_z - prev_zoom) > 1.5 else 0.0
                            est_slew_s = max(1.3, min(4.5, max(ang_jump / 8.0 + 0.40, zoom_jump_s)))

                            if self.active_target_id:
                                self.cooldowns[self.active_target_id] = now
                                self.last_left_seq[self.active_target_id] = self.visit_seq
                            self.visit_seq += 1
                            self.active_target_id = cand["id"]
                            self.dwell_started_at = now
                            self.active_dwell_limit_sec = _initial_dwell_for(cand)
                            self.last_target_switch_ts = now
                            self.switch_slew_until_ts = now + est_slew_s
                            self.manual_zoom_override = None
                            switched_target_now = True
                            cur_act_cand = cand
                            cur_max_dwell = self.active_dwell_limit_sec
                            if cand.get("kind") == "landmark":
                                self.last_landmark_visit_ts = now
                                self.last_non_synoptic_kind = "landmark"
                            elif cand.get("kind") == "synoptic":
                                self.last_synoptic_visit_ts = now
                            else:
                                self.last_non_synoptic_kind = "mover"
                        elif force_rotate:
                            self.dwell_started_at = now
                            self.active_dwell_limit_sec = _initial_dwell_for(cand)
                            cur_max_dwell = self.active_dwell_limit_sec

                if cur_act_cand:
                    rem_s = max(0.0, cur_max_dwell - (now - self.dwell_started_at))
                    # Evaluate `upcoming` at the fixed planned switch timestamp (`dwell_started_at + cur_max_dwell`)
                    # so `next_target` never flickers mid-dwell!
                    switch_eta_ts = self.dwell_started_at + cur_max_dwell
                    upcoming = _choose_next_candidate(switch_eta_ts, cur_act_cand)
                    if upcoming and upcoming["id"] != cur_act_cand["id"]:
                        next_target_preview = {
                            "id": upcoming["id"],
                            "callsign": upcoming["callsign"],
                            "kind": upcoming.get("kind", "vessel"),
                            "in_seconds": max(1, int(math.ceil(rem_s))),
                            "switch_in_s": round(rem_s, 2),
                        }

            celestial_out = compute_celestial_ephemeris(now, cal)
            active_cel = (
                next((c for c in celestial_out if c["id"] == self.active_celestial_id), None)
                if (not self.auto_mode and self.active_celestial_id)
                else None
            )

            active = director_by_id.get(self.active_target_id)
            hw_action = None
            if self.calibrating or self.stacking or getattr(self, "_focus_trimming", False):
                self.visual_tracker.notify_active_target(None, None)
                hw_action = ("stop",)
            elif active_cel:
                self.visual_tracker.notify_active_target(None, None)
                target_zoom = self.manual_zoom_override or active_cel["zoom_x"]
                self.manual_point = {
                    "celestial_id": active_cel["id"],
                    "kind": "celestial",
                    "u": active_cel["u"],
                    "v": active_cel["v"],
                    "pan_deg": active_cel["d_az_deg"],
                    "tilt_deg": round(active_cel["alt_deg"] - float(cal.get("tilt_deg", 3.171)), 3),
                    "az_deg": active_cel["az_deg"],
                    "alt_deg": active_cel["alt_deg"],
                    "zoom_x": target_zoom,
                    "default_zoom_x": active_cel["zoom_x"],
                    "dist_km": active_cel["dist_km"],
                    "length_m": 0.0,
                    "label": active_cel["label"],
                    "subtitle": active_cel["subtitle"],
                }
                self.ptz.move_absolute(self.manual_point["pan_deg"], self.manual_point["tilt_deg"], target_zoom)
                hw_action = ("celestial", dict(active_cel), target_zoom)
            elif self.manual_point and not self.auto_mode:
                self.visual_tracker.notify_active_target(None, None)
                mp = self.manual_point
                target_zoom = self.manual_zoom_override or mp.get("zoom_x", 15.0)
                p_cmd, t_cmd = float(mp["pan_deg"]), float(mp["tilt_deg"])
                mp_dist = mp.get("dist_km") or self.estimate_ray_distance_km(p_cmd, t_cmd)
                self.ptz.move_absolute(p_cmd, t_cmd, target_zoom)
                hw_action = ("manual", p_cmd, t_cmd, target_zoom, mp_dist, str(mp.get("kind", "")))
            elif active:
                self.cooldowns[active["id"]] = now
                target_zoom, vis_guidance = self.visual_tracker.notify_active_target(
                    active, self.manual_zoom_override
                )
                if vis_guidance and vis_guidance.get("locked"):
                    active["cv_locked"] = True
                self.ptz.move_absolute(active["pan_deg"], active["tilt_deg"], target_zoom)
                hw_action = ("active", dict(active), target_zoom, vis_guidance)
            else:
                self.visual_tracker.notify_active_target(None, None)
                z = self.manual_zoom_override or 1.0
                self.ptz.move_absolute(0.0, 0.0, z)
                hw_action = ("idle", z)

            # Toggle OBS Top-Right Overview Inset PiP off automatically at 1x synoptic wide zoom
            cur_ptz_zoom = float(
                (self.hw_driver.get_state().get("zoom_x") if self.hw_driver else None)
                or self.ptz.get_state().get("zoom_x")
                or 1.0
            )
            is_synoptic_1x = (cur_ptz_zoom <= 1.35) or (
                bool(active) and active.get("kind") == "synoptic" and cur_ptz_zoom <= 1.8
            )
            self.obs_cam.set_ptz_inset_visible(not is_synoptic_1x)

            # Project skyline reference landmarks for optional alignment overlay
            landmarks_out = []
            for lm in cal.get("landmarks", []):
                e, n, u_m = lla_to_enu(lm["lat"], lm["lon"], lm["alt_m"], cal["lat"], cal["lon"], cal["alt_m"])
                p = project_target(e, n, u_m, 0, 0, 0, cal)
                if p and 0.0 <= p["u"] <= 1.0 and 0.0 <= p["v"] <= 1.0:
                    landmarks_out.append({
                        "name": lm["name"],
                        "u": round(p["u"], 4),
                        "v": round(p["v"], 4),
                        "dist_km": round(p["dist_m"] / 1000.0, 2),
                    })

            is_slewing_now = bool(
                now < self.switch_slew_until_ts or (now - self.last_target_switch_ts) < 1.1
            )
            snap_out = {
                "timestamp": now,
                "adsb_source": self.adsb_source,
                "ais_source": ais_src,
                "ais_local_count": local_ais_count,
                "ais_harbor_total": ais_harbor_total,
                "enable_sim_vessels": self.enable_sim_vessels,
                "cv_mode": self.cv_mode,
                "obs_cam": self.obs_cam.get_status(),
                "ptz_youtube": self.obs_cam.get_ptz_youtube_status(),
                "monthly_youtube": self.monthly_yt.get_status(),
                "auto_mode": self.auto_mode,
                "active_target_id": self.active_target_id,
                "active_target": active,
                "next_target": next_target_preview,
                "ptz_slewing": is_slewing_now,
                "dwell_remaining_s": round(max(0.0, cur_max_dwell - (now - self.dwell_started_at)), 1)
                if self.active_target_id and self.auto_mode
                else 0.0,
                "ptz": self.ptz.get_state(),
                "ptz_camera_model": str(cal.get("ptz_camera_model", "sony_a40")),
                "ptz_sony_host": str(cal.get("ptz_sony_host", "192.168.1.136")),
                "ptz_sony_user": str(cal.get("ptz_sony_user", "admin")),
                "ptz_sony_has_pass": bool(os.environ.get("SONY_PTZ_PASS", "")),
                "hw_ptz": self.hw_driver.get_state() if self.hw_driver else {"enabled": False},
                "ptz_calibration": self.ptz_calibrator.get_status(),
                "visual_tracker": self.visual_tracker.get_status(),
                "lucky_stack": self.lucky_stacker.get_status(),
                "library_count": self.enhance_library.count(),
                "public_lease_remaining_s": round(max(0.0, self.public_lease_until_ts - now), 1)
                if (not self.auto_mode and self.public_lease_until_ts > 0.0)
                else 0.0,
                "ptz_auto_recalibrate_daily": self.auto_recalibrate_daily,
                "manual_point": self.manual_point if not self.auto_mode else None,
                "calibration": {
                    k: cal.get(k, 0.0)
                    for k in (
                        "lat",
                        "lon",
                        "alt_m",
                        "az_deg",
                        "tilt_deg",
                        "roll_deg",
                        "hfov_deg",
                        "k1",
                        "adsb_lead_s",
                        "ptz_pan_offset_deg",
                        "ptz_tilt_offset_deg",
                        "ptz_roll_offset_deg",
                        "ptz_steps_per_deg",
                        "ptz_max_zoom_x",
                    )
                },
                "targets": targets,
                "landmarks": landmarks_out,
                "presets": presets_out,
                "celestial": celestial_out,
            }
            self._latest_snapshot = snap_out

        # Publish snapshot BEFORE executing synchronous HTTP CGI motor commands (and outside `self.lock`!)
        # so `/api/state` immediately flips to "Slewing to: <Target>" at t=0ms with zero lock/CGI lag!
        if drive_hw and hw_action and self.hw_driver and self._hw_io_lock.acquire(blocking=False):
            try:
                if switched_target_now:
                    # Give the 120ms HUD poller one frame to display "Slewing to: <Name>" right as motors start
                    time.sleep(0.11)
                kind_act = hw_action[0]
                if kind_act == "stop":
                    if hasattr(self.hw_driver, "stop_motion"):
                        self.hw_driver.stop_motion()
                elif kind_act == "celestial":
                    _, ac, tz = hw_action
                    if isinstance(self.hw_driver, SonySRGA40Driver):
                        self.hw_driver.set_celestial_exposure(ac["id"])
                    if hasattr(self.hw_driver, "move_celestial_hw"):
                        self.hw_driver.move_celestial_hw(ac["pan_hw_deg"], ac["tilt_hw_deg"], tz)
                elif kind_act == "manual":
                    _, p_c, t_c, tz, m_d, m_k = hw_action
                    if isinstance(self.hw_driver, SonySRGA40Driver):
                        self.hw_driver.clear_celestial_exposure()
                        self.hw_driver.move_absolute(p_c, t_c, tz, dist_km=m_d, target_kind=m_k)
                    elif isinstance(self.hw_driver, ViscaOverIPDriver):
                        self.hw_driver.move_absolute(p_c, t_c, tz, dist_km=m_d)
                    else:
                        self.hw_driver.move_absolute(p_c, t_c, tz)
                elif kind_act == "active":
                    _, act_d, tz, vg = hw_action
                    if isinstance(self.hw_driver, SonySRGA40Driver):
                        self.hw_driver.clear_celestial_exposure()
                        self.hw_driver.move_absolute(
                            act_d["pan_deg"],
                            act_d["tilt_deg"],
                            tz,
                            pan_dps=act_d.get("pan_dps", 0.0),
                            tilt_dps=act_d.get("tilt_dps", 0.0),
                            dist_km=act_d.get("dist_km"),
                            target_kind=str(act_d.get("kind", "")),
                            visual_guidance=vg,
                        )
                    elif isinstance(self.hw_driver, ViscaOverIPDriver):
                        self.hw_driver.move_absolute(
                            act_d["pan_deg"],
                            act_d["tilt_deg"],
                            tz,
                            pan_dps=act_d.get("pan_dps", 0.0),
                            tilt_dps=act_d.get("tilt_dps", 0.0),
                            dist_km=act_d.get("dist_km"),
                        )
                    else:
                        self.hw_driver.move_absolute(act_d["pan_deg"], act_d["tilt_deg"], tz)
                elif kind_act == "idle":
                    _, tz = hw_action
                    if isinstance(self.hw_driver, SonySRGA40Driver):
                        self.hw_driver.clear_celestial_exposure()
                    if isinstance(self.hw_driver, ViscaOverIPDriver):
                        self.hw_driver.move_absolute(0.0, 0.0, tz, dist_km=4.0)
                    else:
                        self.hw_driver.move_absolute(0.0, 0.0, tz)

                if switched_target_now:
                    hw_done = max(
                        float(getattr(self.hw_driver, "slew_until_ts", 0.0)),
                        float(getattr(self.hw_driver, "zoom_settling_until_ts", 0.0)),
                    )
                    if hw_done > 0.0:
                        with self.lock:
                            self.switch_slew_until_ts = min(
                                time.time() + 5.0, max(self.last_target_switch_ts + 1.1, hw_done)
                            )
            finally:
                self._hw_io_lock.release()

        return snap_out

    def get_fast_snapshot(self) -> dict:
        """Lock-free instant read (<0.2ms) for `/api/state` & HUD pollers so CGI I/O never delays chyrons."""
        snap = self._latest_snapshot
        if snap is not None and (time.time() - float(snap.get("timestamp", 0.0))) < 2.5:
            return snap
        return self.compute_snapshot(drive_hw=False)

    def _apply_revisit_cooldown(self, tid: str, raw_interest: float, now: float) -> float:
        """
        Enforces a strict 65-second revisit lockout (and >= `revisit_gap_visits` intervening shots)
        after following any boat or aircraft so Auto-Director never ping-pongs onto the same vessel,
        depressing recently visited movers (`0.5`) well below scenic Harbor Landmarks (`3.8–10.8`).
        """
        if tid == self.active_target_id:
            return round(raw_interest, 1)
        left_seq = self.last_left_seq.get(tid)
        last_seen = self.cooldowns.get(tid, 0.0)
        recent_time = last_seen > 0.0 and (now - last_seen) < self.mover_cooldown_sec
        recent_seq = left_seq is not None and (self.visit_seq - left_seq) < self.revisit_gap_visits
        if recent_time or recent_seq:
            return 0.5
        return round(raw_interest, 1)

    def _resolve_presets_locked(self, cal: dict) -> list:
        presets_out = []
        for pr in SCENIC_PRESETS:
            e, n, u_m = lla_to_enu(pr["lat"], pr["lon"], pr["alt_m"], cal["lat"], cal["lon"], cal["alt_m"])
            p = project_target(e, n, u_m, 0, 0, 0, cal)
            if not p:
                continue
            dist_km = round(p["dist_m"] / 1000.0, 2)
            if "u_override" in pr and "v_override" in pr:
                u_val = float(pr["u_override"])
                v_val = float(pr["v_override"])
                pan_d, tilt_d = self.pixel_uv_to_pan_tilt(u_val, v_val)
            else:
                u_val = float(p["u"])
                v_val = float(p["v"])
                pan_d = float(p["pan_deg"])
                tilt_d = float(p["tilt_deg"])
            if not (0.0 <= u_val <= 1.0 and 0.0 <= v_val <= 1.0):
                continue
            presets_out.append({
                "id": pr["id"],
                "name": pr["name"],
                "label": pr["label"],
                "subtitle": pr["subtitle"],
                "u": round(u_val, 5),
                "v": round(v_val, 5),
                "pan_deg": round(pan_d, 3),
                "tilt_deg": round(tilt_d, 3),
                "zoom_x": float(pr["zoom_x"]),
                "dist_km": dist_km,
                "length_m": float(pr.get("length_m", 25.0)),
            })
        return presets_out

    def pan_tilt_to_pixel_uv(self, pan_deg: float, tilt_deg: float) -> tuple:
        hfov = math.radians(float(self.calibration.get("hfov_deg", 67.957)))
        vfov = 2.0 * math.atan(math.tan(hfov * 0.5) * (9.0 / 16.0))
        u = 0.5 + math.tan(math.radians(pan_deg)) / (2.0 * max(1e-4, math.tan(hfov * 0.5)))
        v = 0.5 - math.tan(math.radians(tilt_deg)) / (2.0 * max(1e-4, math.tan(vfov * 0.5)))
        return max(0.0, min(1.0, u)), max(0.0, min(1.0, v))

    @staticmethod
    def _ptz_fov_for_zoom(zoom_x: float) -> tuple:
        z = max(1.0, min(30.0, float(zoom_x)))
        hfov_deg = max(2.67, 68.0 / z)
        vfov_deg = math.degrees(2.0 * math.atan(math.tan(math.radians(hfov_deg * 0.5)) * (9.0 / 16.0)))
        return hfov_deg, vfov_deg

    def hold_for_enhance(self, lease_sec: float = 90.0):
        """Pauses Auto-Director on the current target/frame for `lease_sec` when Enhance! is clicked."""
        now = time.time()
        with self.lock:
            if self.hw_driver and hasattr(self.hw_driver, "stop_motion"):
                self.hw_driver.stop_motion()
            self.dwell_started_at = now
            if self.auto_mode:
                self.auto_mode = False
                self.public_lease_until_ts = now + lease_sec + 30.0
                if not self.active_target_id and not self.manual_point:
                    ptz_st = self.ptz.get_state()
                    p_d = float(ptz_st.get("pan_deg", 0.0))
                    t_d = float(ptz_st.get("tilt_deg", 0.0))
                    z_x = float(self.manual_zoom_override or ptz_st.get("zoom_x", 1.0))
                    u_est, v_est = self.pan_tilt_to_pixel_uv(p_d, t_d)
                    self.manual_point = {
                        "preset_id": None,
                        "kind": "scenic",
                        "u": round(u_est, 5),
                        "v": round(v_est, 5),
                        "pan_deg": round(p_d, 3),
                        "tilt_deg": round(t_d, 3),
                        "zoom_x": round(z_x, 2),
                        "default_zoom_x": round(z_x, 2),
                        "dist_km": round(self.estimate_ray_distance_km(p_d, t_d), 2),
                        "length_m": 25.0,
                        "label": "HUDSON RIVER CORRIDOR",
                        "subtitle": f"Enhance Hold · {z_x:.1f}× Optical",
                    }
            elif self.public_lease_until_ts > 0.0:
                self.public_lease_until_ts = max(self.public_lease_until_ts, now + lease_sec + 30.0)

    def apply_select_payload(self, payload: dict, is_public: bool = False) -> dict:
        PUBLIC_LEASE_SEC = 90.0
        now = time.time()
        with self.lock:
            if is_public and self.calibrating:
                return {"ok": False, "error": "Camera calibration in progress"}
            if is_public and self.stacking:
                return {"ok": False, "error": "Enhance capture in progress — hold tight!"}

            tid = payload.get("target_id")
            preset_id = payload.get("preset_id")
            celestial_id = payload.get("celestial_id")
            has_mp = bool(payload.get("manual_point"))
            has_ptz_click = bool(payload.get("ptz_click"))
            has_ptz_nudge = bool(payload.get("ptz_nudge"))
            zoom_only = (
                "zoom_x" in payload
                and tid is None
                and not preset_id
                and not celestial_id
                and not has_mp
                and not has_ptz_click
                and not has_ptz_nudge
                and "auto_mode" not in payload
            )

            # Gentle rate limit on public target switches (zoom/nudge allowed smoothly)
            if is_public and not zoom_only and not has_ptz_nudge and tid != "__auto__":
                if (now - self.public_last_select_ts) < 1.2:
                    return {"ok": False, "error": "Slewing — wait a moment"}
                self.public_last_select_ts = now

            if zoom_only:
                z_raw = payload.get("zoom_x")
                new_z = max(1.0, min(30.0, float(z_raw))) if z_raw else None
                self.manual_zoom_override = new_z
                if self.manual_point:
                    self.manual_point["zoom_x"] = (
                        self.manual_zoom_override
                        or self.manual_point.get("default_zoom_x")
                        or self.manual_point["zoom_x"]
                    )
                if is_public and not self.auto_mode:
                    self.public_lease_until_ts = now + PUBLIC_LEASE_SEC
            elif tid == "__auto__":
                self.auto_mode = True
                self.active_celestial_id = None
                self.manual_point = None
                self.manual_zoom_override = None
                self.public_lease_until_ts = 0.0
                self.dwell_started_at = 0.0
            elif tid == "__next__":
                self.auto_mode = True
                self.active_celestial_id = None
                self.manual_point = None
                self.public_lease_until_ts = 0.0
                if self.active_target_id:
                    self.cooldowns[self.active_target_id] = now
                    self.last_left_seq[self.active_target_id] = self.visit_seq
                self.active_target_id = None
                self.dwell_started_at = 0.0
                self.last_target_switch_ts = now
            elif celestial_id:
                c_list = compute_celestial_ephemeris(now, self.calibration)
                c_match = next((c for c in c_list if c["id"] == celestial_id), None)
                if not c_match or not c_match.get("visible"):
                    return {
                        "ok": False,
                        "error": c_match.get("status_note", "Not currently visible") if c_match else "Unknown object",
                    }
                self.auto_mode = False
                self.active_target_id = None
                self.last_target_switch_ts = now
                self.manual_zoom_override = None
                self.active_celestial_id = c_match["id"]
                self.public_lease_until_ts = (now + PUBLIC_LEASE_SEC) if is_public else 0.0
                self.manual_point = {
                    "celestial_id": c_match["id"],
                    "kind": "celestial",
                    "u": c_match["u"],
                    "v": c_match["v"],
                    "pan_deg": c_match["d_az_deg"],
                    "tilt_deg": round(
                        c_match["alt_deg"] - float(self.calibration.get("tilt_deg", 3.171)), 3
                    ),
                    "az_deg": c_match["az_deg"],
                    "alt_deg": c_match["alt_deg"],
                    "zoom_x": c_match["zoom_x"],
                    "default_zoom_x": c_match["zoom_x"],
                    "dist_km": c_match["dist_km"],
                    "length_m": 0.0,
                    "label": c_match["label"],
                    "subtitle": c_match["subtitle"],
                }
            elif preset_id:
                self.active_celestial_id = None
                presets = self._resolve_presets_locked(self.calibration)
                matched = next((p for p in presets if p["id"] == preset_id), None)
                if not matched:
                    return {"ok": False, "error": "Unknown preset"}
                self.auto_mode = False
                self.active_target_id = None
                self.last_target_switch_ts = now
                self.manual_zoom_override = None
                self.public_lease_until_ts = (now + PUBLIC_LEASE_SEC) if is_public else 0.0
                self.manual_point = {
                    "preset_id": matched["id"],
                    "kind": "landmark",
                    "u": matched["u"],
                    "v": matched["v"],
                    "pan_deg": matched["pan_deg"],
                    "tilt_deg": matched["tilt_deg"],
                    "zoom_x": matched["zoom_x"],
                    "default_zoom_x": matched["zoom_x"],
                    "dist_km": matched["dist_km"],
                    "length_m": matched["length_m"],
                    "label": matched["label"],
                    "subtitle": matched["subtitle"],
                }
            elif has_ptz_click or has_ptz_nudge:
                ptz_st = self.ptz.get_state()
                cur_pan = float(ptz_st.get("pan_deg", 0.0))
                cur_tilt = float(ptz_st.get("tilt_deg", 0.0))
                cur_zoom = float(self.manual_zoom_override or ptz_st.get("zoom_x", 12.0))
                hfov_d, vfov_d = self._ptz_fov_for_zoom(cur_zoom)
                if has_ptz_click:
                    pc = payload["ptz_click"]
                    rx = max(0.0, min(1.0, float(pc.get("rx", 0.5))))
                    ry = max(0.0, min(1.0, float(pc.get("ry", 0.5))))
                    new_pan = max(-72.0, min(72.0, cur_pan + (rx - 0.5) * hfov_d))
                    new_tilt = max(-24.0, min(54.0, cur_tilt - (ry - 0.5) * vfov_d))
                    new_zoom = max(1.0, min(30.0, float(pc.get("zoom_x") or cur_zoom)))
                else:
                    pn = payload["ptz_nudge"]
                    sx = max(-1.0, min(1.0, float(pn.get("step_x", 0.0))))
                    sy = max(-1.0, min(1.0, float(pn.get("step_y", 0.0))))
                    new_pan = max(-72.0, min(72.0, cur_pan + sx * hfov_d))
                    new_tilt = max(-24.0, min(54.0, cur_tilt + sy * vfov_d))
                    new_zoom = cur_zoom
                u_est, v_est = self.pan_tilt_to_pixel_uv(new_pan, new_tilt)
                d_km = self.estimate_ray_distance_km(new_pan, new_tilt)
                self.active_celestial_id = None
                self.auto_mode = False
                self.active_target_id = None
                self.public_lease_until_ts = (now + PUBLIC_LEASE_SEC) if is_public else 0.0
                az_abs = (float(self.calibration.get("az_deg", 207.186)) + new_pan) % 360.0
                el_abs = float(self.calibration.get("tilt_deg", 3.171)) + new_tilt
                self.manual_point = {
                    "preset_id": None,
                    "kind": "manual",
                    "u": round(u_est, 5),
                    "v": round(v_est, 5),
                    "pan_deg": round(new_pan, 3),
                    "tilt_deg": round(new_tilt, 3),
                    "zoom_x": round(new_zoom, 2),
                    "default_zoom_x": round(new_zoom, 2),
                    "dist_km": round(d_km, 2),
                    "length_m": 25.0,
                    "label": "CUSTOM HARBOR POINT",
                    "subtitle": f"Az {az_abs:.1f}° · El {el_abs:+.1f}° · {new_zoom:.1f}× Optical",
                }
            elif has_mp:
                self.active_celestial_id = None
                mp_in = payload["manual_point"]
                u_m = max(0.0, min(1.0, float(mp_in["u"])))
                v_m = max(0.0, min(1.0, float(mp_in["v"])))
                pan_d, tilt_d = self.pixel_uv_to_pan_tilt(u_m, v_m)
                z_m = max(1.0, min(30.0, float(mp_in.get("zoom_x") or self.manual_zoom_override or 15.0)))
                d_km = float(mp_in.get("dist_km") or self.estimate_ray_distance_km(pan_d, tilt_d))
                self.auto_mode = False
                self.active_target_id = None
                self.public_lease_until_ts = (now + PUBLIC_LEASE_SEC) if is_public else 0.0
                self.manual_point = {
                    "preset_id": mp_in.get("preset_id"),
                    "kind": str(mp_in.get("kind") or "manual"),
                    "u": round(u_m, 5),
                    "v": round(v_m, 5),
                    "pan_deg": round(pan_d, 3),
                    "tilt_deg": round(tilt_d, 3),
                    "zoom_x": round(z_m, 2),
                    "default_zoom_x": round(z_m, 2),
                    "dist_km": round(d_km, 2),
                    "length_m": float(mp_in.get("length_m") or 25.0),
                    "label": str(mp_in.get("label") or "CUSTOM HARBOR POINT"),
                    "subtitle": str(mp_in.get("subtitle") or "Interactive Viewport Point"),
                }
            else:
                self.active_celestial_id = None
                self.auto_mode = bool(payload.get("auto_mode", False))
                self.manual_point = None
                self.manual_zoom_override = None
                self.active_target_id = tid
                self.dwell_started_at = now
                self.last_target_switch_ts = now
                self.public_lease_until_ts = (
                    (now + PUBLIC_LEASE_SEC) if (is_public and not self.auto_mode) else 0.0
                )

            if not zoom_only and "zoom_x" in payload:
                z_raw = payload.get("zoom_x")
                self.manual_zoom_override = max(1.0, min(30.0, float(z_raw))) if z_raw else None
                if self.manual_point:
                    self.manual_point["zoom_x"] = (
                        self.manual_zoom_override
                        or self.manual_point.get("default_zoom_x")
                        or self.manual_point["zoom_x"]
                    )

            if not zoom_only and not has_ptz_nudge and tid != "__auto__":
                self.last_target_switch_ts = now
                self.switch_slew_until_ts = now + 2.4

        self.compute_snapshot(drive_hw=False)
        return {"ok": True}

    def compute_public_snapshot(self) -> dict:
        """
        Sanitized public snapshot for `hudsonriver.live/zoom` via Cloudflare Tunnel.
        Strips all internal LAN IPs, credentials, camera settings, and calibration controls.
        """
        snap = self.get_fast_snapshot()
        ptz_raw = snap.get("ptz") or {}
        zoom_x = float(ptz_raw.get("zoom_x", 1.0))
        hfov_d, vfov_d = self._ptz_fov_for_zoom(zoom_x)
        ls_raw = snap.get("lucky_stack") or {}
        ls_summary = ls_raw.get("summary") or {}
        lib_entry = ls_summary.get("library_entry")

        return {
            "timestamp": snap["timestamp"],
            "auto_mode": snap["auto_mode"],
            "lease_remaining_s": snap.get("public_lease_remaining_s", 0.0),
            "active_target_id": snap["active_target_id"],
            "active_target": snap.get("active_target"),
            "dwell_remaining_s": snap["dwell_remaining_s"],
            "ptz": {
                "pan_deg": round(float(ptz_raw.get("pan_deg", 0.0)), 2),
                "tilt_deg": round(float(ptz_raw.get("tilt_deg", 0.0)), 2),
                "zoom_x": round(zoom_x, 2),
                "hfov_deg": round(hfov_d, 2),
                "vfov_deg": round(vfov_d, 2),
            },
            "manual_point": snap["manual_point"],
            "targets": snap["targets"],
            "landmarks": snap["landmarks"],
            "presets": snap["presets"],
            "celestial": snap["celestial"],
            "lucky_stack": {
                "running": bool(ls_raw.get("running")),
                "phase": ls_raw.get("phase", "idle"),
                "progress_pct": int(ls_raw.get("progress_pct", 0)),
                "message": ls_raw.get("message", ""),
                "last_ok": ls_raw.get("last_ok"),
                "last_run_ts": ls_raw.get("last_run_ts", 0.0),
                "latest_entry": lib_entry,
            },
            "library_count": snap.get("library_count", 0),
        }


DIRECTOR = WorldDirector()


class RequestHandler(SimpleHTTPRequestHandler):
    """
    Threaded HTTP handler serving static frontend assets (`/` Mission Control, `/?obs=1`
    Virtual PTZ, `/?hud=1` OBS transparent AR overlay) plus JSON & JPEG REST endpoints.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(STATIC_DIR), **kwargs)

    def log_message(self, fmt, *args):
        pass  # Keep launchd / terminal stdout quiet during 24/7 polling

    def _send_json(self, obj: dict, status: int = 200, cache_ctrl: str = "no-store"):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", cache_ctrl)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _is_tunnel_request(self) -> bool:
        return bool(self.headers.get("CF-Connecting-IP") or self.headers.get("CF-Ray"))

    def _verify_tunnel_secret(self) -> bool:
        expected = os.environ.get("PTZ_TUNNEL_SECRET", "").strip()
        provided = (self.headers.get("X-PTZ-Tunnel-Secret") or "").strip()
        if not expected or not provided:
            return False
        return hmac.compare_digest(expected.encode("utf-8"), provided.encode("utf-8"))

    def _serve_library_file(self, filename: str):
        fp = DIRECTOR.enhance_library.resolve_file(filename)
        if not fp:
            self.send_response(404)
            self.end_headers()
            return
        blob = fp.read_bytes()
        ctype = "image/png" if filename.endswith(".png") else "image/jpeg"
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "public, max-age=31536000, immutable")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(blob)))
        self.end_headers()
        self.wfile.write(blob)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        if self._is_tunnel_request():
            if not parsed.path.startswith("/api/public/") or not self._verify_tunnel_secret():
                self._send_json({"error": "forbidden"}, 403)
                return

        if parsed.path == "/api/public/state":
            self._send_json(DIRECTOR.compute_public_snapshot())
            return
        if parsed.path in ("/api/public/library", "/api/enhance_library"):
            self._send_json({"items": DIRECTOR.enhance_library.list_items()})
            return
        if parsed.path.startswith("/api/public/library/"):
            self._serve_library_file(parsed.path.split("/api/public/library/", 1)[1])
            return
        if parsed.path.startswith("/api/enhance_library/"):
            self._serve_library_file(parsed.path.split("/api/enhance_library/", 1)[1])
            return
        if parsed.path == "/api/public/frame.jpg":
            jpeg, _ = DIRECTOR.obs_cam.get_ptz_frame(demand=True, wait_fresh=1.0)
            if not jpeg:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)
            return
        if parsed.path == "/api/public/wide.jpg":
            jpeg, _ = DIRECTOR.obs_cam.get_frame(demand=True, wait_fresh=1.0)
            if not jpeg:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)
            return
        if parsed.path == "/api/state":
            self._send_json(DIRECTOR.get_fast_snapshot())
            return
        if parsed.path in (
            "/api/lucky_stack/before.png",
            "/api/lucky_stack/after.png",
            "/api/lucky_stack/comparison.png",
        ):
            kind = parsed.path.rsplit("/", 1)[-1]
            fp = DIRECTOR.lucky_stacker.stacks_dir / f"latest_{kind}"
            if not fp.is_file():
                self.send_response(404)
                self.end_headers()
                return
            blob = fp.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(blob)))
            self.end_headers()
            self.wfile.write(blob)
            return
        if parsed.path == "/api/cam_frame.jpg":
            jpeg, _ = DIRECTOR.obs_cam.get_frame(demand=True, wait_fresh=1.2)
            if not jpeg:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)
            return
        if parsed.path == "/api/ptz_frame.jpg":
            jpeg, _ = DIRECTOR.obs_cam.get_ptz_frame(demand=True, wait_fresh=1.2)
            if not jpeg:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)
            return
        if parsed.path == "/api/cam_stream.mjpg":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.end_headers()
            last_ts = 0.0
            try:
                while True:
                    wait_s = 1.0 if last_ts == 0.0 else 0.25
                    jpeg, ts = DIRECTOR.obs_cam.get_frame(demand=True, wait_fresh=wait_s)
                    if jpeg and ts != last_ts:
                        last_ts = ts
                        hdr = f"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n".encode("ascii")
                        self.wfile.write(hdr)
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                    time.sleep(0.04)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
        if parsed.path == "/install-exposure.sh":
            host = self.headers.get("Host") or "192.168.1.245:8765"
            script = f"""#!/bin/zsh
set -euo pipefail
BASE_URL="http://{host}"
APP_SUPPORT="$HOME/Library/Application Support/HudsonRiverUVC"
PLIST_LABEL="live.hudsonriver.uvc-exposure"
PLIST_PATH="$HOME/Library/LaunchAgents/${{PLIST_LABEL}}.plist"
LOG_PATH="$HOME/Library/Logs/hudson-uvc-exposure.log"

echo "===================================================================="
echo "  Hudson River Cam — Insta360 Link 2C Pro Day/Night Exposure Setup"
echo "===================================================================="
mkdir -p "$APP_SUPPORT" "$HOME/Library/LaunchAgents" "$HOME/Library/Logs"

echo "1) Downloading uvc-util & solar exposure scheduler from $BASE_URL..."
curl -fsSL "$BASE_URL/dl/uvc-util" -o "$APP_SUPPORT/uvc-util"
curl -fsSL "$BASE_URL/dl/auto_exposure_day_night.py" -o "$APP_SUPPORT/auto_exposure_day_night.py"
curl -fsSL "$BASE_URL/dl/calibration.json" -o "$APP_SUPPORT/calibration.json"
xattr -cr "$APP_SUPPORT" 2>/dev/null || true
chmod +x "$APP_SUPPORT/uvc-util" "$APP_SUPPORT/auto_exposure_day_night.py"

echo ""
echo "2) Probing connected Insta360 UVC camera (zero OBS interruption)..."
echo "--------------------------------------------------------------------"
"$APP_SUPPORT/uvc-util" --probe

echo ""
echo "3) Today's Hudson River Solar Exposure Schedule:"
echo "--------------------------------------------------------------------"
if [[ -x "/opt/homebrew/bin/python3" ]]; then
  PY_BIN="/opt/homebrew/bin/python3"
else
  PY_BIN="/usr/bin/python3"
fi
"$PY_BIN" "$APP_SUPPORT/auto_exposure_day_night.py" --uvc-bin "$APP_SUPPORT/uvc-util" --schedule-today

cat > "$PLIST_PATH" <<'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>Label</key>
    <string>live.hudsonriver.uvc-exposure</string>
    <key>ProgramArguments</key>
    <array>
        <string>__PY_BIN__</string>
        <string>__APP_SUPPORT__/auto_exposure_day_night.py</string>
        <string>--uvc-bin</string>
        <string>__APP_SUPPORT__/uvc-util</string>
        <string>--day-ev</string>
        <string>0.0</string>
        <string>--night-ev</string>
        <string>-1.8</string>
        <string>--quiet-unchanged</string>
    </array>
    <key>RunAtLoad</key>
    <true/>
    <key>StartInterval</key>
    <integer>180</integer>
    <key>StandardOutPath</key>
    <string>__LOG_PATH__</string>
    <key>StandardErrorPath</key>
    <string>__LOG_PATH__</string>
</dict>
</plist>
EOF
"$PY_BIN" -c '
import sys
from pathlib import Path
p = Path(sys.argv[1])
t = (
    p.read_text()
    .replace("__PY_BIN__", sys.argv[2])
    .replace("__APP_SUPPORT__", sys.argv[3])
    .replace("__LOG_PATH__", sys.argv[4])
)
p.write_text(t)
' "$PLIST_PATH" "$PY_BIN" "$APP_SUPPORT" "$LOG_PATH"

GUI_DOMAIN="gui/$(id -u)"
launchctl bootout "$GUI_DOMAIN/$PLIST_LABEL" 2>/dev/null || true
if ! launchctl bootstrap "$GUI_DOMAIN" "$PLIST_PATH" 2>/dev/null; then
  launchctl unload "$PLIST_PATH" 2>/dev/null || true
  launchctl load "$PLIST_PATH"
fi

echo ""
echo "4) Applying current exposure target right now..."
"$PY_BIN" "$APP_SUPPORT/auto_exposure_day_night.py" --uvc-bin "$APP_SUPPORT/uvc-util"

echo ""
echo "===================================================================="
echo "  Installed & Running Standalone on Mac mini (no Dropbox needed)!"
echo "  - Files:       $APP_SUPPORT"
echo "  - LaunchAgent: $PLIST_PATH"
echo "  - Log file:    $LOG_PATH"
echo "===================================================================="
""".encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/x-shellscript; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(script)))
            self.end_headers()
            self.wfile.write(script)
            return
        if parsed.path in ("/dl/uvc-util", "/dl/auto_exposure_day_night.py", "/dl/calibration.json"):
            fname = parsed.path.split("/dl/", 1)[1]
            fpath = BASE_DIR / fname
            if not fpath.is_file():
                self.send_response(404)
                self.end_headers()
                return
            blob = fpath.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(blob)))
            self.end_headers()
            self.wfile.write(blob)
            return
        return super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        if self._is_tunnel_request():
            if not parsed.path.startswith("/api/public/") or not self._verify_tunnel_secret():
                self._send_json({"error": "forbidden"}, 403)
                return

        length = min(65536, max(0, int(self.headers.get("Content-Length", "0"))))
        raw = self.rfile.read(length).decode("utf-8") if length > 0 else "{}"
        try:
            payload = json.loads(raw) if raw else {}
        except Exception:
            self._send_json({"ok": False, "error": "Invalid JSON"}, 400)
            return

        if parsed.path == "/api/public/select":
            res = DIRECTOR.apply_select_payload(payload, is_public=True)
            self._send_json(res, 200 if res.get("ok") else 409)
            return

        if parsed.path == "/api/public/enhance":
            now = time.time()
            st_cur = DIRECTOR.lucky_stacker.get_status()
            if st_cur.get("running"):
                self._send_json({"ok": True, "already_running": True, "status": st_cur})
                return
            wait_s = 20.0 - (now - max(DIRECTOR.public_last_enhance_ts, float(st_cur.get("last_run_ts") or 0.0)))
            if wait_s > 0.0:
                self._send_json(
                    {"ok": False, "error": f"Enhance cooldown — ready in {int(math.ceil(wait_s))}s"},
                    429,
                )
                return
            with DIRECTOR.lock:
                DIRECTOR.public_last_enhance_ts = now
            DIRECTOR.hold_for_enhance(lease_sec=90.0)
            res = DIRECTOR.lucky_stacker.start_async(frames=32)
            self._send_json(res, 200 if res.get("ok") else 409)
            return

        if parsed.path == "/api/select":
            res = DIRECTOR.apply_select_payload(payload, is_public=False)
            self._send_json(res, 200 if res.get("ok") else 409)
            return

        if parsed.path == "/api/settings":
            with DIRECTOR.lock:
                if "enable_sim_vessels" in payload:
                    DIRECTOR.enable_sim_vessels = bool(payload["enable_sim_vessels"])
                if "cv_mode" in payload and payload["cv_mode"] in ("assist", "strict", "off"):
                    DIRECTOR.cv_mode = payload["cv_mode"]
                    if DIRECTOR.cv_mode == "off":
                        DIRECTOR.cv_tracks.clear()
                if "auto_mode" in payload:
                    DIRECTOR.auto_mode = bool(payload["auto_mode"])
                if "zoom_x" in payload:
                    DIRECTOR.manual_zoom_override = float(payload["zoom_x"]) if payload["zoom_x"] else None
                    if DIRECTOR.manual_point:
                        DIRECTOR.manual_point["zoom_x"] = (
                            DIRECTOR.manual_zoom_override
                            or DIRECTOR.manual_point.get("default_zoom_x")
                            or DIRECTOR.manual_point["zoom_x"]
                        )
            if "ptz_camera_model" in payload and payload["ptz_camera_model"] in ("sony_a40", "fomako"):
                DIRECTOR.switch_ptz_camera(payload["ptz_camera_model"])
            if any(k in payload for k in ("ptz_sony_host", "ptz_sony_user", "ptz_sony_pass")):
                s_updates = {}
                if "ptz_sony_host" in payload and payload["ptz_sony_host"] is not None:
                    s_updates["ptz_sony_host"] = str(payload["ptz_sony_host"]).strip()
                    upsert_dotenv_var("SONY_PTZ_HOST", s_updates["ptz_sony_host"])
                if "ptz_sony_user" in payload and payload["ptz_sony_user"] is not None:
                    s_updates["ptz_sony_user"] = str(payload["ptz_sony_user"]).strip() or "admin"
                    upsert_dotenv_var("SONY_PTZ_USER", s_updates["ptz_sony_user"])
                if "ptz_sony_pass" in payload and payload["ptz_sony_pass"] is not None:
                    upsert_dotenv_var("SONY_PTZ_PASS", str(payload["ptz_sony_pass"]))
                if s_updates:
                    DIRECTOR.save_calibration(s_updates)
                if DIRECTOR.hw_driver and isinstance(DIRECTOR.hw_driver, SonySRGA40Driver):
                    DIRECTOR.hw_driver.set_credentials(
                        host=os.environ.get("SONY_PTZ_HOST"),
                        username=os.environ.get("SONY_PTZ_USER"),
                        password=os.environ.get("SONY_PTZ_PASS"),
                    )
            if "obs_url" in payload or "remote_cam_url" in payload:
                DIRECTOR.save_calibration(
                    {"obs_url": payload.get("obs_url") if "obs_url" in payload else payload.get("remote_cam_url")}
                )
            if "calibration" in payload:
                DIRECTOR.save_calibration(payload["calibration"])
            if "ptz_auto_recalibrate_daily" in payload:
                DIRECTOR.save_calibration(
                    {"ptz_auto_recalibrate_daily": bool(payload["ptz_auto_recalibrate_daily"])}
                )
            if "ptz_youtube_key" in payload or "ptz_youtube_enabled" in payload:
                new_yt_key = None
                if "ptz_youtube_key" in payload and payload["ptz_youtube_key"] is not None:
                    k_str = str(payload["ptz_youtube_key"]).strip()
                    if k_str or payload.get("clear_ptz_youtube_key"):
                        upsert_dotenv_var("PTZ_YOUTUBE_STREAM_KEY", k_str)
                        new_yt_key = k_str
                new_yt_en = None
                if "ptz_youtube_enabled" in payload and payload["ptz_youtube_enabled"] is not None:
                    new_yt_en = bool(payload["ptz_youtube_enabled"])
                    DIRECTOR.save_calibration({"ptz_youtube_enabled": new_yt_en})
                DIRECTOR.obs_cam.set_ptz_youtube_config(stream_key=new_yt_key, enabled=new_yt_en)
            if any(
                k in payload
                for k in (
                    "yt_oauth_client_id",
                    "yt_oauth_client_secret",
                    "yt_oauth_refresh_token",
                    "yt_hudson_playlist_id",
                    "monthly_youtube_enabled",
                )
            ):
                if payload.get("yt_oauth_client_id"):
                    upsert_dotenv_var("YT_OAUTH_CLIENT_ID", str(payload["yt_oauth_client_id"]).strip())
                if payload.get("yt_oauth_client_secret"):
                    upsert_dotenv_var("YT_OAUTH_CLIENT_SECRET", str(payload["yt_oauth_client_secret"]).strip())
                if payload.get("yt_oauth_refresh_token"):
                    upsert_dotenv_var("YT_OAUTH_REFRESH_TOKEN", str(payload["yt_oauth_refresh_token"]).strip())
                if payload.get("yt_hudson_playlist_id"):
                    upsert_dotenv_var("YT_HUDSON_PLAYLIST_ID", str(payload["yt_hudson_playlist_id"]).strip())
                if "monthly_youtube_enabled" in payload and payload["monthly_youtube_enabled"] is not None:
                    m_en = bool(payload["monthly_youtube_enabled"])
                    DIRECTOR.save_calibration({"monthly_youtube_enabled": m_en})
                    DIRECTOR.monthly_yt.set_enabled(m_en)
                DIRECTOR.monthly_yt.notify_oauth_updated()
            self._send_json({"ok": True})
            return

        if parsed.path == "/api/monthly_youtube_record":
            res = DIRECTOR.monthly_yt.trigger_manual_slot_now(
                slot=str(payload.get("slot") or "sunset"),
                duration_sec=int(payload.get("duration_sec") or 10800),
                privacy=str(payload.get("privacy") or "public"),
                camera_source=str(payload.get("camera_source") or "synoptic"),
                custom_title=str(payload.get("title") or ""),
                custom_description=str(payload.get("description") or ""),
            )
            self._send_json(res, 200 if res.get("ok") else 409)
            return

        if parsed.path == "/api/calibrate_ptz":
            res = DIRECTOR.ptz_calibrator.start_async(reason=str(payload.get("reason") or "manual"))
            self._send_json(res, 200 if res.get("ok") else 409)
            return

        if parsed.path == "/api/cv_detections":
            DIRECTOR.ingest_cv_detections(payload.get("detections", []))
            self._send_json({"ok": True})
            return

        if parsed.path == "/api/lucky_stack":
            DIRECTOR.hold_for_enhance(lease_sec=90.0)
            res = DIRECTOR.lucky_stacker.start_async(frames=int(payload.get("frames") or 36))
            self._send_json(res, 200 if res.get("ok") else 409)
            return

        if parsed.path == "/api/reset_camera":
            res = DIRECTOR.obs_cam.request_camera_reset()
            self._send_json(res, 200 if res.get("ok") else 503)
            return

        if parsed.path == "/api/ptz_focus":
            mode = str(payload.get("mode") or "one_push")
            drv = DIRECTOR.hw_driver
            if drv and isinstance(drv, (ViscaOverIPDriver, SonySRGA40Driver)):
                if mode == "one_push":
                    if isinstance(drv, SonySRGA40Driver):
                        drv.trigger_one_push_focus()
                    else:
                        DIRECTOR.run_constrained_focus_trim_async()
                elif mode == "manual":
                    drv.set_auto_focus(False)
                else:
                    drv.set_auto_focus(True)
                self._send_json({"ok": True, "focus_mode": drv.focus_mode})
            else:
                self._send_json({"ok": False, "error": "No PTZ driver configured"}, 503)
            return

        self._send_json({"error": "not_found"}, 404)


def director_control_loop():
    """Autonomous ~7 Hz hardware PTZ tracking loop so state & hardware stay crisp 24/7."""
    while True:
        try:
            DIRECTOR.compute_snapshot()
        except Exception:
            pass
        time.sleep(0.14)


def ptz_exposure_meter_loop():
    """
    Closed-loop highlight-protected outdoor exposure meter for FoMaKo PTZ:
      - Piggybacks at zero OBS cost on Mission Control's cached PTZ preview frames when open.
      - When running headless (0 browsers open), takes a lightweight 320x180 snapshot (`publish_preview=False`)
        1s after slews settle and once every 45s to track passing clouds & specular river glare.
      - Strictly respects zoom/focus motor interlocks (`not drv.pending_focus_sync`).
    """
    import subprocess
    import tempfile

    vis_bin = BASE_DIR / "ptz-vision"
    last_metered_ts = 0.0
    last_probe_ts = 0.0
    follow_up_needed = False
    probe_dir = Path(tempfile.mkdtemp(prefix="hudson_ptz_meter_"))
    probe_jpg = probe_dir / "meter.jpg"

    time.sleep(3.0)
    while True:
        try:
            drv = DIRECTOR.hw_driver
            if (
                drv
                and isinstance(drv, (ViscaOverIPDriver, SonySRGA40Driver))
                and not DIRECTOR.calibrating
                and not DIRECTOR.stacking
                and not getattr(DIRECTOR, "_focus_trimming", False)
                and drv.usb_ready
            ):
                now = time.time()
                lat = float(DIRECTOR.calibration.get("lat", 40.744823))
                lon = float(DIRECTOR.calibration.get("lon", -74.008063))
                elev = solar_elevation_deg_utc(now, lat, lon)

                if elev < -3.5:
                    follow_up_needed = False
                    if (now - drv.last_exposure_sync_ts) >= 60.0:
                        drv.sync_outdoor_exposure(force=False)
                    time.sleep(5.0)
                    continue

                if not drv.pending_focus_sync and (now - drv.last_exposure_sync_ts) >= 45.0:
                    drv.sync_outdoor_exposure(force=False)
                    now = time.time()

                settled = (
                    not drv.pending_focus_sync
                    and (now - drv.last_large_slew_ts) >= 0.85
                    and (now - drv.last_zoom_target_change_ts) >= 1.20
                    and (now - drv.last_exposure_sync_ts) >= 0.85
                )
                if settled and vis_bin.is_file() and DIRECTOR.obs_cam.get_status().get("ptz_available"):
                    jpeg, f_ts = DIRECTOR.obs_cam.get_ptz_frame(demand=False)
                    min_valid_ts = max(drv.last_exposure_sync_ts + 0.35, drv.last_large_slew_ts + 0.55)
                    if jpeg and f_ts > min_valid_ts and (now - f_ts) < 2.0 and f_ts > last_metered_ts:
                        probe_jpg.write_bytes(jpeg)
                        st = json.loads(
                            subprocess.check_output(
                                [str(vis_bin), "stats", str(probe_jpg)], text=True, timeout=3.0
                            )
                        )
                        adjusted = drv.apply_luma_feedback(
                            float(st.get("mean", 122.0)), float(st.get("clipped_frac", 0.0))
                        )
                        last_metered_ts = f_ts
                        follow_up_needed = adjusted
                    else:
                        need_post_slew = (drv.last_large_slew_ts > last_probe_ts) and ((now - last_probe_ts) >= 2.8)
                        probe_interval = 6.0 if (DIRECTOR.active_target_id or DIRECTOR.manual_point) else 45.0
                        need_ambient = (now - max(last_metered_ts, last_probe_ts)) >= probe_interval
                        if follow_up_needed or need_post_slew or need_ambient:
                            fresh_jpg, fresh_ts = DIRECTOR.obs_cam.get_ptz_frame(demand=True, wait_fresh=1.2)
                            if fresh_jpg and fresh_ts > min_valid_ts:
                                probe_jpg.write_bytes(fresh_jpg)
                                st = json.loads(
                                    subprocess.check_output(
                                        [str(vis_bin), "stats", str(probe_jpg)], text=True, timeout=3.0
                                    )
                                )
                                adjusted = drv.apply_luma_feedback(
                                    float(st.get("mean", 122.0)), float(st.get("clipped_frac", 0.0))
                                )
                                last_probe_ts = fresh_ts
                                last_metered_ts = fresh_ts
                                follow_up_needed = adjusted
        except Exception:
            pass
        time.sleep(1.20)


def ptz_maintenance_loop():
    """
    Periodic autonomous FoMaKo maintenance loop (every 180s):
      Automatic once-daily 30x multi-landmark self-calibration in clear midday daylight
      (`sun_elev >= 16 deg`, >= 22h since last calibration) so windowsill drift is corrected forever.
    """
    time.sleep(20.0)
    while True:
        try:
            drv = DIRECTOR.hw_driver
            if drv and isinstance(drv, SonySRGA40Driver) and drv.reachable and not drv.needs_password_init:
                st_s, body_s = drv.cgi.get("/command/inquiry.cgi?inq=stream", auth=True, timeout=2.0)
                if st_s == 200 and body_s:
                    kv_s = drv._parse_kv_body(body_s)
                    if kv_s.get("StreamMode") == "rtmp" and kv_s.get("StreamStatus") not in (
                        "streaming",
                        "streaming-ssl",
                    ):
                        st_r, body_r = drv.cgi.get("/command/inquiry.cgi?inq=rtmp", auth=True, timeout=2.0)
                        kv_r = drv._parse_kv_body(body_r) if st_r == 200 else {}
                        if kv_r.get("RtmpStreamKey1Used") == "1" and kv_r.get("RtmpServerUrl1"):
                            drv.cgi.get("/command/rtmpexe.cgi?RtmpReady=on", auth=True, timeout=2.5)
                            time.sleep(0.8)
                            drv.cgi.get("/command/rtmpexe.cgi?RtmpStreaming=on", auth=True, timeout=2.5)

            if drv and isinstance(drv, (ViscaOverIPDriver, SonySRGA40Driver)) and not DIRECTOR.calibrating:
                now = time.time()
                lat = float(DIRECTOR.calibration.get("lat", 40.744823))
                lon = float(DIRECTOR.calibration.get("lon", -74.008063))
                cam_az = float(DIRECTOR.calibration.get("az_deg", 207.186))
                elev, sun_az = solar_position_deg_utc(now, lat, lon)
                az_diff = abs(((sun_az - cam_az + 180.0) % 360.0) - 180.0)
                low_direct_glare = az_diff < 32.0 and elev < 38.0

                cal_st = DIRECTOR.ptz_calibrator.get_status()
                last_ts = float(cal_st.get("last_run_ts") or 0.0)
                if (
                    DIRECTOR.auto_recalibrate_daily
                    and DIRECTOR.auto_mode
                    and elev >= 14.0
                    and not low_direct_glare
                    and (now - last_ts) >= 22.0 * 3600.0
                    and DIRECTOR.obs_cam.get_status().get("ptz_available")
                ):
                    DIRECTOR.ptz_calibrator.start_async(reason="daily_auto")
        except Exception:
            pass
        time.sleep(60.0)


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


def main():
    def _handle_sigterm(*_):
        if DIRECTOR.hw_driver and hasattr(DIRECTOR.hw_driver, "_close_usb_pipe"):
            DIRECTOR.hw_driver._close_usb_pipe()
        os._exit(0)

    signal.signal(signal.SIGTERM, _handle_sigterm)
    port = int(os.environ.get("PORT", "8765"))
    threading.Thread(target=DIRECTOR.poll_adsb_loop, daemon=True).start()
    threading.Thread(target=DIRECTOR.poll_local_ais_loop, daemon=True).start()
    threading.Thread(target=director_control_loop, daemon=True).start()
    threading.Thread(target=ptz_exposure_meter_loop, daemon=True).start()
    threading.Thread(target=ptz_maintenance_loop, daemon=True).start()
    if DIRECTOR.aisstream:
        DIRECTOR.aisstream.start()

    server = ReusableThreadingHTTPServer(("0.0.0.0", port), RequestHandler)
    print(f"\nHudson River PTZ Director Server running!")
    print(f"  -> Mission Control UI:        http://localhost:{port}/")
    print(f"  -> Clean Simulated PTZ (16:9): http://localhost:{port}/?obs=1")
    print(f"  -> Zero-Copy Transparent HUD: http://localhost:{port}/?hud=1")
    print("Press Ctrl+C to stop.\n")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.")


if __name__ == "__main__":
    main()
