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

import json
import math
import os
import threading
import time
import urllib.request
from datetime import datetime, timezone
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

from aisstream_client import AISStreamTracker, classify_ais_ship_type
from obs_camera_bridge import OBSCameraBridge
from ptz_drivers import AxisVapixDriver, VirtualPTZDriver, ViscaOverIPDriver

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
    k1 = float(cal.get("k1", 0.0))
    r2 = xn * xn + yn * yn
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


class WorldDirector:
    """
    Central thread-safe world model fusing ADS-B aircraft, VHF/cloud AIS vessels,
    and optional CV detections into 3D CTRV-filtered tracks projected onto the camera frame.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.calibration = json.loads(CALIB_PATH.read_text())
        self.ptz = VirtualPTZDriver()
        self.hw_driver = None
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

        initial_obs_url = (
            os.environ.get("OBS_URL", "")
            or os.environ.get("MAC_MINI_CAM_URL", "")
            or self.calibration.get("obs_url", "")
            or self.calibration.get("remote_cam_url", "")
        )
        self.obs_cam = OBSCameraBridge(initial_obs_url)

        # Recent CV detections posted from the Link 2C browser spotter or YOLO script
        self.cv_tracks = {}  # id -> {u, v, w, h, kind, score, ts, du_s, dv_s}
        # "assist" = refine ADS-B/AIS positions only (no purple reflection false positives)
        # "strict" = also promote coherent linear visual movers (or YOLO neural boxes)
        # "off"    = ignore CV motion spotter completely
        self.cv_mode = "assist"

        # Director state
        self.auto_mode = True
        self.active_target_id = None
        self.dwell_started_at = 0.0
        self.min_dwell_sec = 14.0
        self.cooldowns = {}  # target_id -> last_featured_ts
        self.manual_zoom_override = None

    def _configure_optional_hw_ptz(self):
        fomako_ip = os.environ.get("PTZ_FOMAKO_IP")
        visca_ip = os.environ.get("PTZ_VISCA_IP")
        axis_ip = os.environ.get("PTZ_AXIS_IP")
        pan_off = float(os.environ.get("PTZ_PAN_OFFSET_DEG", "0.0"))
        tilt_off = float(os.environ.get("PTZ_TILT_OFFSET_DEG", "0.0"))
        if fomako_ip:
            port = int(os.environ.get("PTZ_FOMAKO_PORT", "1259"))
            self.hw_driver = ViscaOverIPDriver(
                fomako_ip, port=port, raw_udp=True, steps_per_deg=14.4,
                pan_offset_deg=pan_off, tilt_offset_deg=tilt_off,
            )
            self.hw_driver.lock_focus_infinity()
        elif visca_ip:
            port = int(os.environ.get("PTZ_VISCA_PORT", "52381"))
            self.hw_driver = ViscaOverIPDriver(
                visca_ip, port=port, raw_udp=(port == 1259),
                steps_per_deg=14.4 if port == 1259 else 1440.0,
                pan_offset_deg=pan_off, tilt_offset_deg=tilt_off,
            )
            self.hw_driver.lock_focus_infinity()
        elif axis_ip:
            user = os.environ.get("PTZ_AXIS_USER", "root")
            pw = os.environ.get("PTZ_AXIS_PASS", "")
            self.hw_driver = AxisVapixDriver(axis_ip, user, pw)
            self.hw_driver.lock_focus_infinity()

    def save_calibration(self, updates: dict):
        """Persists updated camera pose / lens distortion / OBS WebSocket URL to `calibration.json`."""
        with self.lock:
            for k in ("lat", "lon", "alt_m", "az_deg", "tilt_deg", "roll_deg", "hfov_deg", "k1", "adsb_lead_s"):
                if k in updates:
                    self.calibration[k] = float(updates[k])
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
            # 1) Always check local 1090 MHz SDR (readsb) first via fast loopback
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
                    hex_id = (ac.get("hex") or "unk").strip().lower()
                    callsign = flight or reg or hex_id.upper()
                    subtitle = f"{type_name}" + (f" • {reg}" if reg and reg != callsign else "")

                    seen_pos = float(ac.get("seen_pos") or 0.0)
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

            time.sleep(1.5)

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

        # Pier GPS multipath guard for AIS vessels (e.g., moored yachts next to Pier 40 steel shed)
        if is_vessel:
            dt_pkt = max(1.0, obs_rx - tr.get("last_rx_epoch", now - 10.0))
            raw_disp = math.hypot(e_raw - tr.get("raw_e", e_raw), n_raw - tr.get("raw_n", n_raw))
            implied_mps = raw_disp / dt_pkt
            prev_vh = math.hypot(tr["ve"], tr["vn"])
            if spd_mps == 0.0 or (prev_vh < 0.5 and raw_disp < 55.0):
                # Stationary / moored vessel: suppress pier multipath SOG spikes completely
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
            a_par = 0.16 if not is_vessel else 0.28
            a_perp = 0.46 if not is_vessel else 0.48
            b_par = 0.020 if not is_vessel else 0.008
            b_perp = 0.045 if not is_vessel else 0.015

            tr["e"] = pe + a_par * r_par * he - a_perp * r_perp * hn
            tr["n"] = pn + a_par * r_par * hn + a_perp * r_perp * he
            tr["ve"] = 0.58 * pve + 0.42 * ve_m + b_par * r_par * he - b_perp * r_perp * hn
            tr["vn"] = 0.58 * pvn + 0.42 * vn_m + b_par * r_par * hn + b_perp * r_perp * he
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
                "http://192.168.1.175:8100/api/v1/vessels?limit=250&max_age_s=300",
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
                        type_label, est_len, rarity = classify_ais_ship_type(
                            int(st.get("type_code") or 0), float(st.get("length_m") or 0.0), callsign
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

    def compute_snapshot(self) -> dict:
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
                eval_candidates.append((item, ke - cam_e, kn - cam_n, ku - cam_u, kve, kvn, kvu))

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
                eval_candidates.append((s_item, e0, n0, u0, ve, vn, 0.0))

            targets = []
            for item, east, north, up, ve, vn, vu in eval_candidates:
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
                    est_lat = cal["lat"] + north / 111105.0
                    est_lon = cal["lon"] + east / 84300.0
                    if is_vessel_occluded_by_land(est_lat, est_lon, u_n, dist_m):
                        continue
                elif is_occluded_by_skyline(u_n, dist_m, cal["alt_m"] + up):
                    continue

                # Recommended zoom so target fills ~36% of horizontal FOV
                obj_angle_deg = max(0.08, math.degrees(2.0 * math.atan2(item["length_m"] * 0.5, dist_m)))
                desired_hfov = obj_angle_deg / 0.34
                rec_zoom = max(3.5, min(30.0, cal["hfov_deg"] / max(1.8, desired_hfov)))

                # Smoothed speed & track from filtered velocity
                filt_vh = math.hypot(ve, vn)
                filt_gs_kts = filt_vh / 0.514444 if filt_vh > 0.4 else item["gs_kts"]
                filt_trk_deg = (math.degrees(math.atan2(ve, vn)) + 360.0) % 360.0 if filt_vh > 1.0 else item["track_deg"]

                # Interest score for Auto-Director
                prox_factor = max(0.15, min(2.5, 2500.0 / max(400.0, dist_m)))
                motion_bonus = 1.25 if filt_gs_kts > 5.0 else 0.6
                corridor_bonus = 1.6 if (item["kind"] == "helicopter" and dist_m < 4500) else 1.0
                last_seen = self.cooldowns.get(item["id"], 0.0)
                cooldown_mult = 0.28 if (now - last_seen < 150.0 and item["id"] != self.active_target_id) else 1.0

                interest = round(item["rarity"] * prox_factor * motion_bonus * corridor_bonus * cooldown_mult * 25.0, 1)

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
                            "dist_km": 1.4 if is_sky else 1.2,
                            "alt_ft": 800 if is_sky else 0,
                            "gs_kts": 0.0,
                            "track_deg": 0,
                            "length_m": 18.0,
                            "rec_zoom_x": 16.0 if is_sky else 14.0,
                            "interest": 34.0,
                            "cv_locked": True,
                        })

            targets.sort(key=lambda x: x["interest"], reverse=True)
            by_id = {t["id"]: t for t in targets}

            # Auto-Director selection logic
            if self.active_target_id not in by_id:
                self.active_target_id = None

            if self.auto_mode:
                dwell_elapsed = now - self.dwell_started_at
                if self.active_target_id is None or dwell_elapsed >= self.min_dwell_sec:
                    if targets:
                        top = targets[0]
                        if top["id"] != self.active_target_id:
                            if self.active_target_id:
                                self.cooldowns[self.active_target_id] = now
                            self.active_target_id = top["id"]
                            self.dwell_started_at = now
                            self.manual_zoom_override = None

            active = by_id.get(self.active_target_id)
            if active:
                self.cooldowns[active["id"]] = now
                target_zoom = self.manual_zoom_override or active["rec_zoom_x"]
                self.ptz.move_absolute(active["pan_deg"], active["tilt_deg"], target_zoom)
                if self.hw_driver:
                    # Closed-loop proportional nudge + velocity feed-forward
                    self.hw_driver.move_absolute(active["pan_deg"], active["tilt_deg"], target_zoom)
            else:
                z = self.manual_zoom_override or 1.0
                self.ptz.move_absolute(0.0, 0.0, z)

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

            return {
                "timestamp": now,
                "adsb_source": self.adsb_source,
                "ais_source": ais_src,
                "ais_local_count": local_ais_count,
                "ais_harbor_total": ais_harbor_total,
                "enable_sim_vessels": self.enable_sim_vessels,
                "cv_mode": self.cv_mode,
                "obs_cam": self.obs_cam.get_status(),
                "auto_mode": self.auto_mode,
                "active_target_id": self.active_target_id,
                "dwell_remaining_s": round(max(0.0, self.min_dwell_sec - (now - self.dwell_started_at)), 1)
                if self.active_target_id and self.auto_mode
                else 0.0,
                "ptz": self.ptz.get_state(),
                "calibration": {
                    k: cal.get(k, 0.0)
                    for k in ("lat", "lon", "alt_m", "az_deg", "tilt_deg", "roll_deg", "hfov_deg", "k1", "adsb_lead_s")
                },
                "targets": targets,
                "landmarks": landmarks_out,
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

    def _send_json(self, obj: dict, status: int = 200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/api/state":
            self._send_json(DIRECTOR.compute_snapshot())
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
/usr/bin/python3 "$APP_SUPPORT/auto_exposure_day_night.py" --uvc-bin "$APP_SUPPORT/uvc-util" --schedule-today

cat > "$PLIST_PATH" <<'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>Label</key>
    <string>live.hudsonriver.uvc-exposure</string>
    <key>ProgramArguments</key>
    <array>
        <string>/usr/bin/python3</string>
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
/usr/bin/python3 -c '
import sys
from pathlib import Path
p = Path(sys.argv[1])
t = p.read_text().replace("__APP_SUPPORT__", sys.argv[2]).replace("__LOG_PATH__", sys.argv[3])
p.write_text(t)
' "$PLIST_PATH" "$APP_SUPPORT" "$LOG_PATH"

GUI_DOMAIN="gui/$(id -u)"
launchctl bootout "$GUI_DOMAIN/$PLIST_LABEL" 2>/dev/null || true
if ! launchctl bootstrap "$GUI_DOMAIN" "$PLIST_PATH" 2>/dev/null; then
  launchctl unload "$PLIST_PATH" 2>/dev/null || true
  launchctl load "$PLIST_PATH"
fi

echo ""
echo "4) Applying current exposure target right now..."
/usr/bin/python3 "$APP_SUPPORT/auto_exposure_day_night.py" --uvc-bin "$APP_SUPPORT/uvc-util"

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
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length).decode("utf-8") if length > 0 else "{}"
        payload = json.loads(raw) if raw else {}

        if parsed.path == "/api/select":
            with DIRECTOR.lock:
                tid = payload.get("target_id")
                if tid == "__auto__":
                    DIRECTOR.auto_mode = True
                    DIRECTOR.manual_zoom_override = None
                    DIRECTOR.dwell_started_at = 0.0
                elif tid == "__next__":
                    DIRECTOR.auto_mode = True
                    if DIRECTOR.active_target_id:
                        DIRECTOR.cooldowns[DIRECTOR.active_target_id] = time.time()
                    DIRECTOR.active_target_id = None
                    DIRECTOR.dwell_started_at = 0.0
                else:
                    DIRECTOR.auto_mode = bool(payload.get("auto_mode", False))
                    DIRECTOR.active_target_id = tid
                    DIRECTOR.dwell_started_at = time.time()
                if "zoom_x" in payload:
                    DIRECTOR.manual_zoom_override = float(payload["zoom_x"]) if payload["zoom_x"] else None
            self._send_json({"ok": True})
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
            if "obs_url" in payload or "remote_cam_url" in payload:
                DIRECTOR.save_calibration(
                    {"obs_url": payload.get("obs_url") if "obs_url" in payload else payload.get("remote_cam_url")}
                )
            if "calibration" in payload:
                DIRECTOR.save_calibration(payload["calibration"])
            self._send_json({"ok": True})
            return

        if parsed.path == "/api/cv_detections":
            DIRECTOR.ingest_cv_detections(payload.get("detections", []))
            self._send_json({"ok": True})
            return

        if parsed.path == "/api/reset_camera":
            res = DIRECTOR.obs_cam.request_camera_reset()
            self._send_json(res, 200 if res.get("ok") else 503)
            return

        self._send_json({"error": "not_found"}, 404)


def main():
    port = int(os.environ.get("PORT", "8765"))
    threading.Thread(target=DIRECTOR.poll_adsb_loop, daemon=True).start()
    threading.Thread(target=DIRECTOR.poll_local_ais_loop, daemon=True).start()
    if DIRECTOR.aisstream:
        DIRECTOR.aisstream.start()

    server = ThreadingHTTPServer(("0.0.0.0", port), RequestHandler)
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
