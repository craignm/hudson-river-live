"""
Autonomous Monthly 3-Hour 4K Synoptic Recorder & YouTube Uploader (`monthly_youtube_uploader.py`)
Pure Python 3.9+ standard library — zero external pip dependencies.

Records five 3-hour clean 4K30 videos per month from the wide synoptic camera (`Video Capture Device 2`
in OBS via `source-record`, with zero AR HUD pills and zero interruption to the 24/7 live stream) ONLY
when the 3-hour recording window has clear, sunny / crisp skies, and uploads them as standard YouTube
VODs (`videos.insert` resumable upload) to satisfy and exceed the YouTube Partner Program's upload and
watch-hour eligibility requirements:

  1. Dawn      (from 1st,  sunrise - 30m):                    "Hudson River at dawn, <Month> <Year>"
  2. Morning   (from 7th,  8:15 AM ET, Mon-Fri):              "Hudson River morning, <Month> <Year>"
  3. Afternoon (from 13th, 1:00 PM ET):                       "Hudson River afternoon, <Month> <Year>"
  4. Sunset    (from 19th, sunset - 60m):                     "Hudson River sunset, <Month> <Year>"
  5. Night     (from 24th, evening nightfall sun == -10.0°):  "Hudson River at night, <Month> <Year>"

Also adopts any in-progress OBS recording across server restarts and queues completed `.mov` files if
OAuth credentials are added or refreshed after a recording finishes.
"""

import calendar
from datetime import datetime, timedelta
import json
import math
import os
from pathlib import Path
import re
import threading
import time
from typing import Dict, Optional, Tuple
import urllib.error
import urllib.parse
import urllib.request
from zoneinfo import ZoneInfo

NY_TZ = ZoneInfo("America/New_York")
DEFAULT_LAT = 40.744823
DEFAULT_LON = -74.008063
DEFAULT_SLOT_DURATION_SEC = 3 * 3600  # 3 hours (10,800s)

SLOT_DEFINITIONS = (
    {
        "slot": "dawn",
        "min_day": 1,
        "weekday_only": False,
        "title_fmt": "Hudson River at dawn, {month_year}",
        "lead_desc": "Three hours of clear dawn light and early morning harbor traffic over the Hudson River, New York Harbor, and Jersey City ({month_year}).",
    },
    {
        "slot": "morning",
        "min_day": 7,
        "weekday_only": True,
        "title_fmt": "Hudson River morning, {month_year}",
        "lead_desc": "Three hours of crisp morning sunlight, commuter ferries, and harbor traffic on the Hudson River in New York ({month_year}).",
    },
    {
        "slot": "afternoon",
        "min_day": 13,
        "weekday_only": False,
        "title_fmt": "Hudson River afternoon, {month_year}",
        "lead_desc": "Three hours of sunny afternoon harbor traffic, sightseeing vessels, and skyline views over the Hudson River ({month_year}).",
    },
    {
        "slot": "sunset",
        "min_day": 19,
        "weekday_only": False,
        "title_fmt": "Hudson River sunset, {month_year}",
        "lead_desc": "Three hours of golden hour, clear sunset, twilight, and rising skyline lights over the Hudson River, Lower Manhattan, and Jersey City ({month_year}).",
    },
    {
        "slot": "night",
        "min_day": 24,
        "weekday_only": False,
        "title_fmt": "Hudson River at night, {month_year}",
        "lead_desc": "Three hours of bustling evening skyline lights, illuminated ferries, Little Island, and riverfront traffic on the Hudson River ({month_year}).",
    },
)

ALL_SLOT_NAMES = tuple(s["slot"] for s in SLOT_DEFINITIONS)

BASE_DESCRIPTION_BODY = (
    "Watch live 24/7 on https://hudsonriver.live/, including interactive landmark labels, "
    "live ship & aircraft tracking, weather, and tide conditions.\n\n"
    "Looking south over the Hudson River from 17th Street on the west side of Manhattan. "
    "On the left, in the distance, you can see downtown Manhattan and One World Trade Center. "
    "To the right (west) of the river is Jersey City, New Jersey. In the foreground is Little Island "
    "and in the bottom-right corner is Pier 57. The Statue of Liberty stands in Upper New York Bay "
    "near the center of the frame, with Staten Island and the Verrazzano-Narrows Bridge beyond."
)

CHUNK_SIZE_BYTES = 16 * 1024 * 1024  # 16 MB resumable upload chunks
RECORDING_FILENAME_RE = re.compile(
    r"^hudson_(\d{4}-\d{2})_((?:ptz_)?(?:dawn|morning|afternoon|sunset|night))_.*\.(mov|mp4)$"
)


def _solar_elevation_deg(epoch_s: float, lat_deg: float, lon_deg: float) -> float:
    """Computes apparent solar elevation (degrees) at `epoch_s` UTC."""
    jd = epoch_s / 86400.0 + 2440587.5
    n = jd - 2451545.0
    mean_long = (280.460 + 0.9856474 * n) % 360.0
    g_rad = math.radians((357.528 + 0.9856003 * n) % 360.0)
    ecl_long = math.radians(mean_long + 1.915 * math.sin(g_rad) + 0.020 * math.sin(2.0 * g_rad))
    obliq = math.radians(23.439 - 0.0000004 * n)
    sin_dec = math.sin(obliq) * math.sin(ecl_long)
    dec = math.asin(max(-1.0, min(1.0, sin_dec)))
    ra = math.atan2(math.cos(obliq) * math.sin(ecl_long), math.cos(ecl_long))
    gmst_deg = (280.46061837 + 360.98564736629 * (jd - 2451545.0)) % 360.0
    lst_rad = math.radians((gmst_deg + lon_deg) % 360.0)
    ha = lst_rad - ra
    lat_r = math.radians(lat_deg)
    sin_el = math.sin(lat_r) * math.sin(dec) + math.cos(lat_r) * math.cos(dec) * math.cos(ha)
    return math.degrees(math.asin(max(-1.0, min(1.0, sin_el))))


def compute_nyc_sunrise_sunset_ts(
    local_date: datetime, lat_deg: float = DEFAULT_LAT, lon_deg: float = DEFAULT_LON
) -> Tuple[float, float]:
    """Finds astronomical sunrise & sunset Unix timestamps (`elev == -0.833°`) on `local_date`."""
    midnight_ny = datetime(local_date.year, local_date.month, local_date.day, 0, 0, 0, tzinfo=NY_TZ)
    t0 = midnight_ny.timestamp()
    sunrise_ts = t0 + 6.75 * 3600.0
    sunset_ts = t0 + 17.0 * 3600.0

    prev_t = t0 + 3.5 * 3600.0
    prev_el = _solar_elevation_deg(prev_t, lat_deg, lon_deg)
    for _ in range(1, int((18.5 * 3600) // 300) + 1):
        cur_t = prev_t + 300.0
        cur_el = _solar_elevation_deg(cur_t, lat_deg, lon_deg)
        if prev_el < -0.833 <= cur_el:
            lo, hi = prev_t, cur_t
            for _ in range(12):
                mid = 0.5 * (lo + hi)
                if _solar_elevation_deg(mid, lat_deg, lon_deg) < -0.833:
                    lo = mid
                else:
                    hi = mid
            sunrise_ts = 0.5 * (lo + hi)
        elif prev_el >= -0.833 > cur_el:
            lo, hi = prev_t, cur_t
            for _ in range(12):
                mid = 0.5 * (lo + hi)
                if _solar_elevation_deg(mid, lat_deg, lon_deg) >= -0.833:
                    lo = mid
                else:
                    hi = mid
            sunset_ts = 0.5 * (lo + hi)
        prev_t, prev_el = cur_t, cur_el

    return sunrise_ts, sunset_ts


def compute_nyc_evening_dark_ts(
    sunset_ts: float,
    lat_deg: float = DEFAULT_LAT,
    lon_deg: float = DEFAULT_LON,
    dark_elev_deg: float = -10.0,
) -> float:
    """
    Finds the exact evening Unix timestamp after `sunset_ts` when solar elevation drops below
    `dark_elev_deg` (`-10.0°` = past civil twilight once the western horizon glow has faded into
    proper dark night sky, e.g. ~5:26 PM ET in Nov/Dec and ~9:35 PM EDT in June).
    """
    prev_t = sunset_ts
    prev_el = _solar_elevation_deg(prev_t, lat_deg, lon_deg)
    for _ in range(1, int((3.5 * 3600) // 180) + 1):
        cur_t = prev_t + 180.0
        cur_el = _solar_elevation_deg(cur_t, lat_deg, lon_deg)
        if prev_el >= dark_elev_deg > cur_el:
            lo, hi = prev_t, cur_t
            for _ in range(12):
                mid = 0.5 * (lo + hi)
                if _solar_elevation_deg(mid, lat_deg, lon_deg) >= dark_elev_deg:
                    lo = mid
                else:
                    hi = mid
            return 0.5 * (lo + hi)
        prev_t, prev_el = cur_t, cur_el
    return sunset_ts + 55.0 * 60.0


def compute_day_slot_windows(
    local_date: datetime, lat_deg: float = DEFAULT_LAT, lon_deg: float = DEFAULT_LON
) -> Dict[str, float]:
    sr_ts, ss_ts = compute_nyc_sunrise_sunset_ts(local_date, lat_deg, lon_deg)
    night_dark_ts = compute_nyc_evening_dark_ts(ss_ts, lat_deg, lon_deg, dark_elev_deg=-10.0)
    morning_dt = datetime(local_date.year, local_date.month, local_date.day, 8, 15, 0, tzinfo=NY_TZ)
    # 1:00 PM ET: guarantees 3 full hours of sunny afternoon daylight year-round (even before 4:30 PM winter sunsets)
    afternoon_dt = datetime(local_date.year, local_date.month, local_date.day, 13, 0, 0, tzinfo=NY_TZ)
    return {
        "dawn": sr_ts - 30.0 * 60.0,
        "morning": morning_dt.timestamp(),
        "afternoon": afternoon_dt.timestamp(),
        "sunset": ss_ts - 60.0 * 60.0,
        "night": night_dark_ts,
    }


def check_clear_sunny_weather(
    lat_deg: float = DEFAULT_LAT, lon_deg: float = DEFAULT_LON, late_month_fallback: bool = False
) -> Tuple[bool, str]:
    """
    Verifies pleasant, dry, clear / partly-sunny sky conditions over the Hudson River across the
    active recording window via Open-Meteo (zero API key):
      - Uses layer-weighted effective cloud cover (`max(low, 0.85 * mid) + 0.20 * high`) so thin
        35,000-ft ice-crystal cirrus (`cloud_cover_high = 100%` over bright blue sky) never gets
        mistaken for gray low overcast!
      - Normal rule: dry (`precipitation == 0` now & across the 3-hour window), non-precipitating
        WMO weather_code in `(0, 1, 2, 3)`, effective cloud cover <= 55% now and <= 60% 3h-avg
        (<= 75% peak), visibility >= 10 km.
      - Late-month safety fallback (last 2 days of month only): allows up to 75% effective cloud cover
        as long as there is no precipitation or fog.
    """
    url = (
        "https://api.open-meteo.com/v1/forecast?"
        + urllib.parse.urlencode(
            {
                "latitude": round(lat_deg, 4),
                "longitude": round(lon_deg, 4),
                "current": "cloud_cover,cloud_cover_low,cloud_cover_mid,cloud_cover_high,precipitation,weather_code,visibility",
                "hourly": "cloud_cover,cloud_cover_low,cloud_cover_mid,cloud_cover_high,precipitation,weather_code",
                "timezone": "America/New_York",
                "forecast_days": 2,
            }
        )
    )

    def _eff_cloud(tot: float, low: Optional[float], mid: Optional[float], high: Optional[float]) -> float:
        if low is None and mid is None and high is None:
            return float(tot)
        lo_v = float(low or 0.0)
        mi_v = float(mid or 0.0)
        hi_v = float(high or 0.0)
        return min(100.0, max(lo_v, 0.85 * mi_v) + 0.20 * hi_v)

    try:
        req = urllib.request.Request(url, headers={"User-Agent": "HudsonRiverPTZ/1.0"})
        with urllib.request.urlopen(req, timeout=10.0) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        cur = data.get("current") or {}
        hourly = data.get("hourly") or {}
        raw_clouds = float(cur.get("cloud_cover", 100.0))
        eff_cur = _eff_cloud(
            raw_clouds,
            cur.get("cloud_cover_low"),
            cur.get("cloud_cover_mid"),
            cur.get("cloud_cover_high"),
        )
        precip = float(cur.get("precipitation", 0.0))
        wcode = int(cur.get("weather_code", 3))
        vis_m = float(cur.get("visibility", 16000.0))

        h_times = hourly.get("time") or []
        h_clouds = hourly.get("cloud_cover") or []
        h_low = hourly.get("cloud_cover_low") or []
        h_mid = hourly.get("cloud_cover_mid") or []
        h_high = hourly.get("cloud_cover_high") or []
        h_precip = hourly.get("precipitation") or []
        h_codes = hourly.get("weather_code") or []
        win_eff = []
        win_precip = []
        win_codes = []
        now_ny = datetime.now(tz=NY_TZ)
        for hr_ahead in (1, 2, 3):
            key_hr = (now_ny + timedelta(hours=hr_ahead)).strftime("%Y-%m-%dT%H:00")
            if key_hr in h_times:
                idx = h_times.index(key_hr)
                c_tot = float(h_clouds[idx]) if idx < len(h_clouds) else raw_clouds
                c_lo = h_low[idx] if idx < len(h_low) else None
                c_mi = h_mid[idx] if idx < len(h_mid) else None
                c_hi = h_high[idx] if idx < len(h_high) else None
                win_eff.append(_eff_cloud(c_tot, c_lo, c_mi, c_hi))
                if idx < len(h_precip):
                    win_precip.append(float(h_precip[idx]))
                if idx < len(h_codes):
                    win_codes.append(int(h_codes[idx]))

        avg_win_eff = (sum(win_eff) / len(win_eff)) if win_eff else eff_cur
        peak_win_eff = max(win_eff) if win_eff else eff_cur
        max_win_precip = max(win_precip) if win_precip else 0.0

        max_cur_eff = 75.0 if late_month_fallback else 55.0
        max_avg_eff = 80.0 if late_month_fallback else 60.0
        max_pk_eff = 90.0 if late_month_fallback else 75.0
        allowed_codes = (0, 1, 2, 3)

        ok = (
            precip <= 0.0
            and max_win_precip <= 0.0
            and wcode in allowed_codes
            and all(c in allowed_codes for c in win_codes)
            and eff_cur <= max_cur_eff
            and avg_win_eff <= max_avg_eff
            and peak_win_eff <= max_pk_eff
            and vis_m >= 10000.0
        )
        summary = (
            f"eff-clouds {eff_cur:.0f}% now / {avg_win_eff:.0f}% 3h-avg (pk {peak_win_eff:.0f}%), "
            f"code {wcode}, vis {vis_m / 1000.0:.0f}km"
        )
        return ok, summary
    except Exception as exc:
        return False, f"weather check unavailable ({exc})"


class MonthlyYouTubeUploader:
    """
    Background scheduler that records 3-hour clean 4K synoptic videos on clear, sunny days via OBS
    `source-record` and uploads them to YouTube via Data API v3 (`videos.insert`).
    """

    def __init__(self, obs_bridge, base_dir: Path, calibration_getter=None):
        self.obs_bridge = obs_bridge
        self.base_dir = Path(base_dir)
        self.calibration_getter = calibration_getter
        self.ledger_path = self.base_dir / "monthly_uploads.json"
        self.cache_dir = Path.home() / "Library" / "Caches" / "HudsonRiverMonthly"
        self.lock = threading.Lock()

        self.enabled: bool = True
        self.running_job: Optional[dict] = None
        self.last_error: str = ""
        self.last_weather_note: str = ""
        self.last_uploaded: Optional[dict] = None
        self._skipped_windows: Dict[str, float] = {}
        self._ledger: dict = self._load_ledger()
        self._stop_event = threading.Event()
        self._wake_event = threading.Event()

        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self._cleanup_temp_test_files()
        threading.Thread(target=self._scheduler_loop, daemon=True).start()

    def _load_ledger(self) -> dict:
        if self.ledger_path.is_file():
            try:
                data = json.loads(self.ledger_path.read_text())
                if isinstance(data, dict):
                    return data
            except Exception:
                pass
        return {}

    def _save_ledger(self):
        try:
            self.ledger_path.write_text(json.dumps(self._ledger, indent=2) + "\n")
        except Exception as exc:
            print(f"[Monthly YouTube] Failed to write ledger: {exc}", flush=True)

    def _cleanup_temp_test_files(self):
        """Deletes stale ad-hoc test clips while preserving any unuploaded monthly `hudson_YYYY-MM_*` recordings."""
        try:
            cutoff = time.time() - 48.0 * 3600.0
            for fp in self.cache_dir.iterdir():
                if not fp.is_file():
                    continue
                if RECORDING_FILENAME_RE.match(fp.name):
                    continue
                if fp.stat().st_mtime < cutoff:
                    fp.unlink(missing_ok=True)
        except Exception:
            pass

    @staticmethod
    def _oauth_credentials() -> Tuple[str, str, str]:
        cid = os.environ.get("YT_OAUTH_CLIENT_ID", "").strip()
        csec = os.environ.get("YT_OAUTH_CLIENT_SECRET", "").strip()
        rtok = os.environ.get("YT_OAUTH_REFRESH_TOKEN", "").strip()
        return cid, csec, rtok

    def has_oauth(self) -> bool:
        cid, csec, rtok = self._oauth_credentials()
        return bool(cid and csec and rtok)

    def notify_oauth_updated(self):
        self._wake_event.set()

    def set_enabled(self, enabled: bool):
        with self.lock:
            self.enabled = bool(enabled)
        self._wake_event.set()

    def _lat_lon(self) -> Tuple[float, float]:
        if self.calibration_getter:
            try:
                cal = self.calibration_getter() or {}
                return float(cal.get("lat", DEFAULT_LAT)), float(cal.get("lon", DEFAULT_LON))
            except Exception:
                pass
        return DEFAULT_LAT, DEFAULT_LON

    @staticmethod
    def _slot_metadata(ym: str, slot: str) -> Tuple[str, str]:
        dt = datetime.strptime(ym + "-01", "%Y-%m-%d")
        month_year = dt.strftime("%B %Y")
        if slot.startswith("ptz_"):
            base_slot = slot[4:]
            title = f"Hudson River {base_slot} — 30x pan/tilt/zoom, {month_year}"
            desc = (
                f"Three hours of live 4K 30x optical pan/tilt/zoom tracking of ferries, vessels, aircraft, "
                f"and harbor landmarks on the Hudson River in New York ({month_year}).\n\n"
                + BASE_DESCRIPTION_BODY
            )
            return title, desc
        sdef = next(s for s in SLOT_DEFINITIONS if s["slot"] == slot)
        title = sdef["title_fmt"].format(month_year=month_year)
        desc = sdef["lead_desc"].format(month_year=month_year) + "\n\n" + BASE_DESCRIPTION_BODY
        return title, desc

    def _pending_local_files(self) -> list:
        """Returns completed local monthly `.mov`/`.mp4` recordings waiting for OAuth/upload."""
        out = []
        now = time.time()
        try:
            for fp in sorted(self.cache_dir.iterdir()):
                if not fp.is_file():
                    continue
                m = RECORDING_FILENAME_RE.match(fp.name)
                if not m:
                    continue
                ym, slot, _ = m.groups()
                if (self._ledger.get(ym) or {}).get(slot, {}).get("video_id"):
                    continue
                st = fp.stat()
                if (now - st.st_mtime) < 25.0:
                    continue
                if st.st_size >= 50_000_000:
                    out.append((ym, slot, fp))
        except Exception:
            pass
        return out

    def _find_active_obs_recording(self) -> Optional[Tuple[str, str, Path, float]]:
        """Detects an already-running OBS `Monthly Synoptic 4K Recorder` file across server restarts."""
        now = time.time()
        try:
            for fp in self.cache_dir.iterdir():
                if not fp.is_file():
                    continue
                m = RECORDING_FILENAME_RE.match(fp.name)
                if not m:
                    continue
                st = fp.stat()
                if (now - st.st_mtime) <= 25.0:
                    ym, slot, _ = m.groups()
                    birth = getattr(st, "st_birthtime", st.st_ctime)
                    return ym, slot, fp, float(birth)
        except Exception:
            pass
        return None

    def compute_next_scheduled_slot(self, now_ts: Optional[float] = None) -> Optional[dict]:
        """Determines the soonest upcoming unuploaded (month, slot) window."""
        now = time.time() if now_ts is None else float(now_ts)
        now_ny = datetime.fromtimestamp(now, tz=NY_TZ)
        lat, lon = self._lat_lon()

        with self.lock:
            ledger_copy = json.loads(json.dumps(self._ledger))
            skipped = dict(self._skipped_windows)

        pending_keys = {(ym, slot) for ym, slot, _ in self._pending_local_files()}
        active_rec = self._find_active_obs_recording()
        if active_rec:
            pending_keys.add((active_rec[0], active_rec[1]))

        best_candidate: Optional[dict] = None
        for day_offset in range(0, 40):
            d_ny = (now_ny + timedelta(days=day_offset)).replace(hour=12, minute=0, second=0, microsecond=0)
            ym = f"{d_ny.year:04d}-{d_ny.month:02d}"
            month_year = d_ny.strftime("%B %Y")
            done_for_month = ledger_copy.get(ym) or {}
            windows = compute_day_slot_windows(d_ny, lat, lon)
            days_in_month = calendar.monthrange(d_ny.year, d_ny.month)[1]
            is_late_month = d_ny.day >= (days_in_month - 2)
            is_weekday = d_ny.weekday() < 5

            for sdef in SLOT_DEFINITIONS:
                slot = sdef["slot"]
                if slot in done_for_month and done_for_month[slot].get("video_id"):
                    continue
                if (ym, slot) in pending_keys:
                    continue
                if d_ny.day < sdef["min_day"]:
                    continue
                if sdef.get("weekday_only") and not is_weekday and not is_late_month:
                    continue
                win_key = f"{d_ny.strftime('%Y-%m-%d')}:{slot}"
                if win_key in skipped:
                    continue
                start_ts = windows[slot]
                if start_ts + 4500.0 < now:
                    continue
                if best_candidate is None or start_ts < best_candidate["start_ts"]:
                    best_candidate = {
                        "ym": ym,
                        "slot": slot,
                        "win_key": win_key,
                        "late_month": is_late_month,
                        "title": sdef["title_fmt"].format(month_year=month_year),
                        "description": sdef["lead_desc"].format(month_year=month_year) + "\n\n" + BASE_DESCRIPTION_BODY,
                        "start_ts": start_ts,
                        "start_et": datetime.fromtimestamp(start_ts, tz=NY_TZ).strftime("%b %-d, %-I:%M %p ET"),
                        "duration_sec": DEFAULT_SLOT_DURATION_SEC,
                        "privacy": "public",
                    }
            if best_candidate and (best_candidate["start_ts"] - now) < 36.0 * 3600.0:
                break
        return best_candidate

    def get_status(self) -> dict:
        now = time.time()
        now_ny = datetime.fromtimestamp(now, tz=NY_TZ)
        cur_ym = f"{now_ny.year:04d}-{now_ny.month:02d}"
        nxt = self.compute_next_scheduled_slot(now)
        pending = self._pending_local_files()
        total_slots = len(ALL_SLOT_NAMES)
        with self.lock:
            has_auth = self.has_oauth()
            month_done = dict(self._ledger.get(cur_ym) or {})
            completed_slots = [k for k in ALL_SLOT_NAMES if month_done.get(k, {}).get("video_id")]
            job = dict(self.running_job) if self.running_job else None
            err = self.last_error
            wnote = self.last_weather_note
            en = self.enabled

        if job:
            st_txt = job.get("status_text", "Recording / uploading monthly 4K video…")
        elif pending and not has_auth:
            titles = ", ".join(f'"{self._slot_metadata(ym, sl)[0]}"' for ym, sl, _ in pending)
            st_txt = f"Recorded {titles} — run 'Authorize Monthly YouTube Uploads.command' to upload!"
        elif not has_auth:
            st_txt = "Clear-day recorder armed · Run 'Authorize Monthly YouTube Uploads.command' to enable YouTube upload"
        elif not en:
            st_txt = f"Paused ({len(completed_slots)}/{total_slots} uploaded for {now_ny.strftime('%b %Y')})"
        elif nxt:
            w_sfx = f" ({wnote})" if wnote else ""
            st_txt = (
                f"{len(completed_slots)}/{total_slots} in {now_ny.strftime('%b %Y')} · "
                f"Next clear-day check: \"{nxt['title']}\" @ {nxt['start_et']}{w_sfx}"
            )
        else:
            st_txt = f"All {len(completed_slots)}/{total_slots} clear-day videos uploaded for {now_ny.strftime('%B %Y')}"

        return {
            "enabled": en,
            "oauth_ready": has_auth,
            "current_month": cur_ym,
            "completed_this_month": completed_slots,
            "pending_uploads": [
                {"ym": ym, "slot": sl, "file": fp.name, "size_gb": round(fp.stat().st_size / (1024**3), 2)}
                for ym, sl, fp in pending
            ],
            "month_records": month_done,
            "next_slot": nxt,
            "running_job": job,
            "last_weather_note": wnote,
            "last_error": err,
            "status_text": st_txt,
        }

    def trigger_manual_slot_now(
        self,
        slot: str = "sunset",
        duration_sec: int = DEFAULT_SLOT_DURATION_SEC,
        privacy: str = "public",
        camera_source: str = "synoptic",
        custom_title: str = "",
        custom_description: str = "",
    ) -> dict:
        """Starts an immediate clean 4K recording from `synoptic` or `ptz` and uploads to YouTube."""
        if slot not in ALL_SLOT_NAMES:
            return {"ok": False, "error": f"Invalid slot (choose from {', '.join(ALL_SLOT_NAMES)})"}
        cam_src = "ptz" if str(camera_source).lower() == "ptz" else "synoptic"
        with self.lock:
            if self.running_job is not None:
                return {"ok": False, "error": "A recording or upload job is already running"}
            now_ny = datetime.now(tz=NY_TZ)
            ym = f"{now_ny.year:04d}-{now_ny.month:02d}"
            default_title, default_desc = self._slot_metadata(ym, slot)
            if cam_src == "ptz" and not custom_title:
                month_year = now_ny.strftime("%B %Y")
                title = f"Hudson River {slot} — 30x pan/tilt/zoom, {month_year}"
                desc = (
                    f"Three hours of live 4K 30x optical pan/tilt/zoom tracking of ferries, vessels, aircraft, "
                    f"and harbor landmarks on the Hudson River in New York ({month_year}).\n\n"
                    + BASE_DESCRIPTION_BODY
                )
            else:
                title = custom_title.strip() or default_title
                desc = custom_description.strip() or default_desc
            job_spec = {
                "ym": ym,
                "slot": f"ptz_{slot}" if cam_src == "ptz" else slot,
                "camera_source": cam_src,
                "title": title,
                "description": desc,
                "duration_sec": max(15, min(14400, int(duration_sec))),
                "privacy": privacy if privacy in ("public", "unlisted", "private") else "public",
                "record_in_ledger": (privacy == "public" and (cam_src == "ptz" or duration_sec >= 1800)),
            }
            self.running_job = {
                "phase": "starting",
                "title": title,
                "progress_pct": 0,
                "status_text": f"Starting 4K {'PTZ ' if cam_src == 'ptz' else ''}capture: \"{title}\"…",
            }
        threading.Thread(target=self._execute_job, args=(job_spec,), daemon=True).start()
        return {"ok": True, "title": title, "camera_source": cam_src}

    def _scheduler_loop(self):
        time.sleep(3.5)
        while not self._stop_event.is_set():
            try:
                with self.lock:
                    idle = self.running_job is None
                if idle:
                    active_rec = self._find_active_obs_recording()
                    if active_rec:
                        ym, slot, fp, birth_ts = active_rec
                        is_ptz = slot.startswith("ptz_")
                        adopt_dur = DEFAULT_SLOT_DURATION_SEC
                        title, desc = self._slot_metadata(ym, slot)
                        job_spec = {
                            "ym": ym,
                            "slot": slot,
                            "camera_source": "ptz" if is_ptz else "synoptic",
                            "title": title,
                            "description": desc,
                            "duration_sec": adopt_dur,
                            "privacy": "public",
                            "record_in_ledger": True,
                            "adopted_file": fp,
                            "adopted_start_ts": birth_ts,
                        }
                        with self.lock:
                            if self.running_job is None:
                                self.running_job = {
                                    "phase": "recording",
                                    "title": title,
                                    "progress_pct": int(
                                        min(99, max(0, (time.time() - birth_ts) * 100.0 / adopt_dur))
                                    ),
                                    "status_text": f"Recording 4K \"{title}\"…",
                                }
                                threading.Thread(target=self._execute_job, args=(job_spec,), daemon=True).start()
                        continue

                with self.lock:
                    can_upload_pending = self.enabled and self.has_oauth() and (self.running_job is None)
                if can_upload_pending:
                    pending = self._pending_local_files()
                    if pending:
                        ym, slot, fp = pending[0]
                        title, desc = self._slot_metadata(ym, slot)
                        job_spec = {
                            "ym": ym,
                            "slot": slot,
                            "title": title,
                            "description": desc,
                            "duration_sec": DEFAULT_SLOT_DURATION_SEC,
                            "privacy": "public",
                            "record_in_ledger": True,
                            "existing_completed_file": fp,
                        }
                        with self.lock:
                            if self.running_job is None:
                                self.running_job = {
                                    "phase": "uploading",
                                    "title": title,
                                    "progress_pct": 0,
                                    "status_text": f"Uploading queued 4K video \"{title}\"…",
                                }
                                threading.Thread(target=self._execute_job, args=(job_spec,), daemon=True).start()
                        continue

                with self.lock:
                    can_schedule = self.enabled and (self.running_job is None)
                if can_schedule:
                    now = time.time()
                    nxt = self.compute_next_scheduled_slot(now)
                    if nxt and nxt["start_ts"] <= now <= (nxt["start_ts"] + 4500.0):
                        lat, lon = self._lat_lon()
                        is_clear, w_summary = check_clear_sunny_weather(
                            lat, lon, late_month_fallback=bool(nxt.get("late_month"))
                        )
                        with self.lock:
                            self.last_weather_note = w_summary
                        if not is_clear:
                            print(
                                f"[Monthly YouTube] Skipping {nxt['win_key']} ('{nxt['title']}') — "
                                f"waiting for a clear, sunny window ({w_summary}).",
                                flush=True,
                            )
                            with self.lock:
                                self._skipped_windows[nxt["win_key"]] = now
                        else:
                            print(
                                f"[Monthly YouTube] Clear, sunny window confirmed ({w_summary})! "
                                f"Starting '{nxt['title']}'.",
                                flush=True,
                            )
                            job_spec = {
                                "ym": nxt["ym"],
                                "slot": nxt["slot"],
                                "title": nxt["title"],
                                "description": nxt["description"],
                                "duration_sec": DEFAULT_SLOT_DURATION_SEC,
                                "privacy": "public",
                                "record_in_ledger": True,
                            }
                            with self.lock:
                                if self.running_job is None:
                                    self.running_job = {
                                        "phase": "starting",
                                        "title": job_spec["title"],
                                        "progress_pct": 0,
                                        "status_text": f"Starting 3-hour 4K capture: \"{job_spec['title']}\"…",
                                    }
                                    threading.Thread(target=self._execute_job, args=(job_spec,), daemon=True).start()
            except Exception as exc:
                print(f"[Monthly YouTube] Scheduler check error: {exc}", flush=True)

            self._wake_event.wait(timeout=15.0)
            self._wake_event.clear()

    def _fetch_access_token(self) -> str:
        cid, csec, rtok = self._oauth_credentials()
        if not (cid and csec and rtok):
            raise RuntimeError("Missing YT_OAUTH_CLIENT_ID / SECRET / REFRESH_TOKEN in .env")
        body = urllib.parse.urlencode(
            {
                "client_id": cid,
                "client_secret": csec,
                "refresh_token": rtok,
                "grant_type": "refresh_token",
            }
        ).encode("utf-8")
        req = urllib.request.Request(
            "https://oauth2.googleapis.com/token",
            data=body,
            headers={"Content-Type": "application/x-www-form-urlencoded"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=20.0) as resp:
            tok_data = json.loads(resp.read().decode("utf-8"))
        access_token = tok_data.get("access_token", "")
        if not access_token:
            raise RuntimeError(f"OAuth refresh returned no access_token: {tok_data}")
        return access_token

    def _upload_file_resumable(self, file_path: Path, title: str, description: str, privacy: str) -> str:
        """Uploads `file_path` to YouTube Data API v3 (`videos.insert`) in 16 MB resumable chunks."""
        access_token = self._fetch_access_token()
        total_bytes = file_path.stat().st_size
        meta = {
            "snippet": {
                "title": title,
                "description": description,
                "categoryId": "19",
                "tags": ["Hudson River", "New York City", "Manhattan", "New York Harbor", "4K"],
                "defaultLanguage": "en",
            },
            "status": {
                "privacyStatus": privacy,
                "selfDeclaredMadeForKids": False,
                "embeddable": True,
            },
        }
        meta_bytes = json.dumps(meta).encode("utf-8")
        mime_type = "video/quicktime" if file_path.suffix.lower() == ".mov" else "video/mp4"

        init_req = urllib.request.Request(
            "https://www.googleapis.com/upload/youtube/v3/videos?uploadType=resumable&part=snippet,status",
            data=meta_bytes,
            headers={
                "Authorization": f"Bearer {access_token}",
                "Content-Type": "application/json; charset=UTF-8",
                "X-Upload-Content-Length": str(total_bytes),
                "X-Upload-Content-Type": mime_type,
            },
            method="POST",
        )
        with urllib.request.urlopen(init_req, timeout=30.0) as init_resp:
            upload_url = init_resp.headers.get("Location")
        if not upload_url:
            raise RuntimeError("YouTube resumable upload did not return a Location session URL")

        sent = 0
        with file_path.open("rb") as fp:
            while sent < total_bytes:
                fp.seek(sent)
                chunk = fp.read(min(CHUNK_SIZE_BYTES, total_bytes - sent))
                if not chunk:
                    break
                end_byte = sent + len(chunk) - 1
                headers = {
                    "Content-Length": str(len(chunk)),
                    "Content-Type": mime_type,
                    "Content-Range": f"bytes {sent}-{end_byte}/{total_bytes}",
                }
                for attempt in range(5):
                    try:
                        put_req = urllib.request.Request(upload_url, data=chunk, headers=headers, method="PUT")
                        with urllib.request.urlopen(put_req, timeout=180.0) as put_resp:
                            resp_body = put_resp.read().decode("utf-8", errors="replace")
                            res_json = json.loads(resp_body) if resp_body.strip() else {}
                            vid = res_json.get("id", "")
                            if vid:
                                return vid
                            sent = end_byte + 1
                            break
                    except urllib.error.HTTPError as he:
                        if he.code == 308:
                            rng = he.headers.get("Range") or ""
                            if "-" in rng:
                                sent = int(rng.split("-", 1)[1]) + 1
                            else:
                                sent = end_byte + 1
                            break
                        if he.code in (500, 502, 503, 504) and attempt < 4:
                            time.sleep(2.5 * (attempt + 1))
                            continue
                        err_txt = he.read().decode("utf-8", errors="replace")[:300]
                        raise RuntimeError(f"YouTube upload HTTP {he.code}: {err_txt}") from he
                    except Exception:
                        if attempt < 4:
                            time.sleep(3.0 * (attempt + 1))
                            continue
                        raise

                pct = int(round(100.0 * sent / max(1, total_bytes)))
                with self.lock:
                    if self.running_job:
                        self.running_job["phase"] = "uploading"
                        self.running_job["progress_pct"] = pct
                        self.running_job["status_text"] = (
                            f"Uploading \"{title}\" to YouTube ({pct}% · "
                            f"{sent / (1024**3):.2f}/{total_bytes / (1024**3):.2f} GB)…"
                        )

        raise RuntimeError("Upload stream finished without YouTube returning a video ID")

    def _add_video_to_playlist(self, video_id: str) -> bool:
        """Automatically appends newly uploaded monthly Hudson River VODs to `YT_HUDSON_PLAYLIST_ID`."""
        playlist_id = os.environ.get("YT_HUDSON_PLAYLIST_ID", "").strip()
        if not playlist_id or not video_id:
            return False
        try:
            access_token = self._fetch_access_token()
            body = json.dumps(
                {
                    "snippet": {
                        "playlistId": playlist_id,
                        "resourceId": {
                            "kind": "youtube#video",
                            "videoId": video_id,
                        },
                    }
                }
            ).encode("utf-8")
            req = urllib.request.Request(
                "https://www.googleapis.com/youtube/v3/playlistItems?part=snippet",
                data=body,
                headers={
                    "Authorization": f"Bearer {access_token}",
                    "Content-Type": "application/json; charset=UTF-8",
                },
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=20.0) as resp:
                resp.read()
            print(f"[Monthly YouTube] Added {video_id} to playlist {playlist_id}", flush=True)
            return True
        except Exception as exc:
            print(f"[Monthly YouTube] Playlist add warning for {video_id}: {exc}", flush=True)
            return False

    def _execute_job(self, job_spec: dict):
        title = job_spec["title"]
        dur_s = int(job_spec["duration_sec"])
        cam_src = str(job_spec.get("camera_source") or "synoptic")
        recorded_file: Optional[Path] = job_spec.get("existing_completed_file")
        uploaded_ok = False

        try:
            self.cache_dir.mkdir(parents=True, exist_ok=True)
            if recorded_file is None:
                adopted_fp: Optional[Path] = job_spec.get("adopted_file")
                if adopted_fp is not None:
                    t_start = float(job_spec.get("adopted_start_ts") or time.time())
                    stem = adopted_fp.stem
                    before_files = set()
                    print(
                        f"[Monthly YouTube] Adopted live 4K recording '{adopted_fp.name}' "
                        f"({int(time.time() - t_start)}s elapsed / {dur_s}s).",
                        flush=True,
                    )
                else:
                    stem = f"hudson_{job_spec['ym']}_{job_spec['slot']}_{datetime.now(tz=NY_TZ).strftime('%Y%m%d_%H%M')}"
                    before_files = {p.name for p in self.cache_dir.iterdir() if p.is_file()}
                    if cam_src == "ptz":
                        if not self.obs_bridge.start_ptz_recording(str(self.cache_dir), stem):
                            raise RuntimeError("OBS bridge could not start 4K PTZ broadcast recording")
                    else:
                        if not self.obs_bridge.start_synoptic_recording(str(self.cache_dir), stem):
                            raise RuntimeError("OBS bridge could not start clean 4K synoptic recording")
                    t_start = time.time()

                try:
                    while True:
                        elapsed = time.time() - t_start
                        if elapsed >= dur_s:
                            break
                        pct = int(round(100.0 * elapsed / max(1, dur_s)))
                        rem_min = max(1, int(math.ceil((dur_s - elapsed) / 60.0)))
                        size_str = ""
                        if adopted_fp and adopted_fp.is_file():
                            size_str = f" · {adopted_fp.stat().st_size / (1024**3):.2f} GB"
                        else:
                            matches = [p for p in self.cache_dir.iterdir() if p.is_file() and p.name.startswith(stem)]
                            if matches:
                                size_str = f" · {matches[0].stat().st_size / (1024**3):.2f} GB"
                        with self.lock:
                            if self.running_job:
                                self.running_job["phase"] = "recording"
                                self.running_job["progress_pct"] = pct
                                self.running_job["status_text"] = (
                                    f"Recording 4K \"{title}\" ({pct}% · {rem_min}m left{size_str})…"
                                )
                        time.sleep(2.0)
                finally:
                    if cam_src == "ptz":
                        self.obs_bridge.stop_ptz_recording()
                    else:
                        self.obs_bridge.stop_synoptic_recording()

                time.sleep(6.0)
                if adopted_fp and adopted_fp.is_file():
                    recorded_file = adopted_fp
                else:
                    candidates = [
                        p
                        for p in self.cache_dir.iterdir()
                        if p.is_file() and (p.name.startswith(stem) or p.name not in before_files)
                    ]
                    if not candidates:
                        raise RuntimeError(f"Recorded file '{stem}' not found in {self.cache_dir}")
                    recorded_file = max(candidates, key=lambda p: p.stat().st_mtime)

                # Wait until OBS finishes flushing the `hybrid_mov` trailer (~21 GB for 3h; require 3 consecutive
                # quiescent size+mtime samples 3s apart so total_bytes is 100% final before opening upload)
                prev_sig = (-1, -1.0)
                stable_streak = 0
                for _ in range(30):
                    st_f = recorded_file.stat()
                    cur_sig = (st_f.st_size, st_f.st_mtime)
                    if cur_sig == prev_sig and st_f.st_size > 0:
                        stable_streak += 1
                        if stable_streak >= 3:
                            break
                    else:
                        stable_streak = 0
                        prev_sig = cur_sig
                    time.sleep(3.0)

            min_bytes = 500_000 if dur_s < 120 else 50_000_000
            if recorded_file.stat().st_size < min_bytes:
                raise RuntimeError(
                    f"Recorded file '{recorded_file.name}' is unexpectedly small ({recorded_file.stat().st_size} B)"
                )

            if not self.has_oauth():
                print(
                    f"[Monthly YouTube] Finished recording '{recorded_file.name}' "
                    f"({recorded_file.stat().st_size / (1024**3):.2f} GB). Queued on disk awaiting YouTube OAuth.",
                    flush=True,
                )
                with self.lock:
                    self.last_error = ""
                return

            with self.lock:
                if self.running_job:
                    self.running_job["phase"] = "uploading"
                    self.running_job["progress_pct"] = 0
                    self.running_job["status_text"] = f"Uploading \"{title}\" to YouTube (0%)…"

            video_id = self._upload_file_resumable(
                recorded_file,
                title=title,
                description=job_spec["description"],
                privacy=job_spec["privacy"],
            )
            uploaded_ok = True
            print(f"[Monthly YouTube] Uploaded '{title}' -> https://youtu.be/{video_id}", flush=True)
            self._add_video_to_playlist(video_id)

            entry = {
                "video_id": video_id,
                "url": f"https://www.youtube.com/watch?v={video_id}",
                "title": title,
                "privacy": job_spec["privacy"],
                "uploaded_at": time.time(),
            }
            with self.lock:
                self.last_error = ""
                self.last_uploaded = entry
                if job_spec.get("record_in_ledger"):
                    ym_map = self._ledger.setdefault(job_spec["ym"], {})
                    ym_map[job_spec["slot"]] = entry
                    self._save_ledger()
        except Exception as exc:
            err_msg = str(exc)
            print(f"[Monthly YouTube] Job failed for '{title}': {err_msg}", flush=True)
            with self.lock:
                self.last_error = err_msg
        finally:
            if uploaded_ok and recorded_file and recorded_file.is_file():
                try:
                    recorded_file.unlink(missing_ok=True)
                except Exception:
                    pass
            with self.lock:
                self.running_job = None
