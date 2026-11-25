"""
Zero-Dependency Maritime AIS WebSocket Client (`aisstream_client.py`)
Pure Python 3.9+ standard library (`ssl`, `socket`, `base64`, `struct`, `json`).

Streams live Class A & Class B AIS vessel reports from `wss://stream.aisstream.io/v0/stream`
for the Lower Hudson River & New York Harbor bounding box, and classifies vessels by
ITU-R M.1371 ship-type code + operator naming heuristics (NY Waterway ferries, FDNY
fireboats, USCG cutters, Moran/McAllister/Reinauer tugs, Sandy Hook pilots, etc.).

In `server.py`, local zero-latency VHF SDR packets from `AIS-catcher` (`:8100`) take strict
priority for any vessel heard directly over RF, while `AISStreamTracker` fills in static
vessel dimensions/names/destinations and shadowed outer-harbor vessels.
"""

import atexit
import base64
import json
import os
import socket
import ssl
import struct
import sys
import tempfile
import threading
import time
from datetime import datetime, timezone
from pathlib import Path


def _default_ais_cache_path() -> Path:
    env_override = os.environ.get("AIS_CACHE_PATH", "").strip()
    if env_override:
        return Path(env_override).expanduser()
    if sys.platform == "darwin":
        return Path.home() / "Library" / "Caches" / "HudsonRiverPTZ" / "ais_cache.json"
    return Path(tempfile.gettempdir()) / "hudson_river_ptz_ais_cache.json"


# Stored in local OS cache directory (outside Dropbox!) so frequent AIS updates never trigger cloud sync churn
AIS_CACHE_PATH = _default_ais_cache_path()
LEGACY_DROPBOX_CACHE_PATH = Path(__file__).resolve().parent / ".ais_cache.json"


def _parse_aisstream_utc(raw_utc, fallback_now: float) -> float:
    """Parses aisstream MetaData.time_utc ('YYYY-MM-DD HH:MM:SS.nnnnnnnnn +0000 UTC') to epoch."""
    if not raw_utc or not isinstance(raw_utc, str):
        return fallback_now
    try:
        parts = raw_utc.strip().split()
        if len(parts) < 2:
            return fallback_now
        dt_part, tm_part = parts[0], parts[1]
        if "." in tm_part:
            sec_s, frac_s = tm_part.split(".", 1)
            frac_us = (frac_s + "000000")[:6]
            dt = datetime.strptime(f"{dt_part} {sec_s}.{frac_us}", "%Y-%m-%d %H:%M:%S.%f").replace(tzinfo=timezone.utc)
        else:
            dt = datetime.strptime(f"{dt_part} {tm_part}", "%Y-%m-%d %H:%M:%S").replace(tzinfo=timezone.utc)
        epoch = dt.timestamp()
        return max(fallback_now - 30.0, min(fallback_now, epoch))
    except Exception:
        return fallback_now


# Vessels with faulty/offset AIS GPS transponders that should be completely ignored
BLACKLISTED_VESSEL_MMSIS = frozenset(
    {
        "367415390",  # EMPIRE STATE (NY Waterway ferry — AIS GPS position consistently wrong)
    }
)
BLACKLISTED_VESSEL_NAMES = (
    "EMPIRE STATE",
)


def is_blacklisted_vessel(mmsi: str, name: str = "") -> bool:
    """Returns True if `(mmsi, name)` is on the faulty-AIS vessel ignore list."""
    mmsi_s = str(mmsi or "").strip()
    if mmsi_s in BLACKLISTED_VESSEL_MMSIS:
        return True
    uname = " ".join((name or "").upper().split())
    if not uname:
        return False
    return any(uname == b or uname.startswith(b + " ") for b in BLACKLISTED_VESSEL_NAMES)


STATEN_ISLAND_FERRIES_BY_MMSI = {
    "368089980": ("SSG MICHAEL H. OLLIS", 97.5),
    "368143060": ("SANDY GROUND", 97.5),
    "368217570": ("DOROTHY DAY", 97.5),
    "366952790": ("GUY V. MOLINARI", 94.5),
    "366952870": ("SEN. JOHN J. MARCHI", 94.5),
    "366952890": ("SPIRIT OF AMERICA", 94.5),
    "367000140": ("SAMUEL I. NEWHOUSE", 94.5),
    "367000130": ("ANDREW J. BARBERI", 94.5),
    "367000120": ("ALICE AUSTEN", 63.1),
    "367000110": ("JOHN A. NOBLE", 63.1),
}

STATEN_ISLAND_FERRY_NAME_PATTERNS = (
    ("MICHAEL H. OLLIS", "SSG MICHAEL H. OLLIS", 97.5),
    ("MICHAEL H OLLIS", "SSG MICHAEL H. OLLIS", 97.5),
    ("SSG OLLIS", "SSG MICHAEL H. OLLIS", 97.5),
    ("OLLIS", "SSG MICHAEL H. OLLIS", 97.5),
    ("SANDY GROUND", "SANDY GROUND", 97.5),
    ("DOROTHY DAY", "DOROTHY DAY", 97.5),
    ("MOLINARI", "GUY V. MOLINARI", 94.5),
    ("MARCHI", "SEN. JOHN J. MARCHI", 94.5),
    ("SPIRIT OF AMERICA", "SPIRIT OF AMERICA", 94.5),
    ("SAMUEL I. NEWHOUSE", "SAMUEL I. NEWHOUSE", 94.5),
    ("SAMUEL I NEWHOUSE", "SAMUEL I. NEWHOUSE", 94.5),
    ("NEWHOUSE", "SAMUEL I. NEWHOUSE", 94.5),
    ("ANDREW J. BARBERI", "ANDREW J. BARBERI", 94.5),
    ("ANDREW J BARBERI", "ANDREW J. BARBERI", 94.5),
    ("ALICE AUSTEN", "ALICE AUSTEN", 63.1),
    ("JOHN A. NOBLE", "JOHN A. NOBLE", 63.1),
    ("JOHN A NOBLE", "JOHN A. NOBLE", 63.1),
)


def identify_staten_island_ferry(mmsi: str, name: str):
    """
    If `(mmsi, name)` belongs to the NYC DOT Staten Island Ferry fleet, returns
    `(display_callsign, canonical_vessel_name, length_m)`; otherwise returns `None`.
    """
    mmsi_s = str(mmsi or "").strip()
    if mmsi_s in STATEN_ISLAND_FERRIES_BY_MMSI:
        canon, hull_m = STATEN_ISLAND_FERRIES_BY_MMSI[mmsi_s]
        return f"STATEN ISLAND FERRY · {canon}", canon, hull_m
    uname = " ".join((name or "").upper().replace(".", ". ").split())
    uname_compact = (name or "").upper()
    if "CIRCLE LINE" in uname_compact:
        return None
    for pat, canon, hull_m in STATEN_ISLAND_FERRY_NAME_PATTERNS:
        if pat in uname_compact or pat in uname:
            return f"STATEN ISLAND FERRY · {canon}", canon, hull_m
    return None


def classify_ais_ship_type(type_code: int, length_m: float, name: str, mmsi: str = ""):
    si_match = identify_staten_island_ferry(mmsi, name)
    if si_match is not None:
        _, _, si_len = si_match
        return "NYC DOT Staten Island Ferry", max(length_m, si_len), 2.85
    uname = (name or "").upper()
    if "FDNY" in uname or "FIRE" in uname:
        return "FDNY Fireboat", max(length_m, 32.0), 2.9
    if "CG " in uname or "COAST GUARD" in uname or "USCGC" in uname or type_code == 51:
        return "US Coast Guard / SAR", max(length_m, 28.0), 2.8
    if "NYPD" in uname or "POLICE" in uname or type_code == 55:
        return "Harbor Law Enforcement", max(length_m, 18.0), 2.7
    if type_code == 35:
        return "Military / Naval Vessel", max(length_m, 45.0), 3.0
    if type_code in (31, 32, 52) or "TUG" in uname or "MORAN" in uname or "MCALLISTER" in uname or "REINAUER" in uname:
        return "Harbor Tug / Tow", max(length_m, 32.0), 2.0
    if type_code == 36:
        return "Sailing Vessel", max(length_m, 16.0), 2.5
    if type_code == 37:
        return "Motor Yacht / Pleasure Craft", max(length_m, 20.0), 2.2
    if type_code == 50:
        return "Sandy Hook Pilot Boat", max(length_m, 22.0), 2.3
    if 40 <= type_code <= 49:
        return "High-Speed Craft", max(length_m, 28.0), 1.8
    if 60 <= type_code <= 69:
        if length_m >= 140:
            return "Cruise Ship", length_m, 3.3
        return "Passenger Ferry", max(length_m, 28.0), 1.35
    if 70 <= type_code <= 79:
        return "Cargo / Container Ship", max(length_m, 120.0), 2.6
    if 80 <= type_code <= 89:
        return "Tanker", max(length_m, 95.0), 2.4
    return "Maritime Vessel", max(length_m, 26.0), 1.5


class MinimalWSSClient:
    """Minimal RFC 6455 WebSocket-over-TLS client using Python stdlib only."""

    def __init__(self, host: str, path: str, port: int = 443, timeout: float = 35.0):
        raw_sock = socket.create_connection((host, port), timeout=timeout)
        ctx = ssl.create_default_context()
        self.sock = ctx.wrap_socket(raw_sock, server_hostname=host)
        self._handshake(host, path)

    def _handshake(self, host: str, path: str):
        ws_key = base64.b64encode(os.urandom(16)).decode("ascii")
        req = (
            f"GET {path} HTTP/1.1\r\n"
            f"Host: {host}\r\n"
            f"Upgrade: websocket\r\n"
            f"Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {ws_key}\r\n"
            f"Sec-WebSocket-Version: 13\r\n\r\n"
        )
        self.sock.sendall(req.encode("ascii"))
        resp = b""
        while b"\r\n\r\n" not in resp:
            chunk = self.sock.recv(1024)
            if not chunk:
                raise ConnectionError("EOF during WebSocket handshake")
            resp += chunk
        header_line = resp.split(b"\r\n", 1)[0].decode("latin1", errors="replace")
        if "101" not in header_line:
            raise ConnectionError(f"WebSocket handshake failed: {header_line}")
        self._leftover = resp.split(b"\r\n\r\n", 1)[1]

    def _recv_exact(self, n: int) -> bytes:
        buf = bytearray(self._leftover[:n])
        self._leftover = self._leftover[n:]
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("WebSocket closed")
            buf.extend(chunk)
        return bytes(buf)

    def send_Frame(self, opcode: int, payload: bytes):
        mask_key = os.urandom(4)
        header = bytearray([0x80 | (opcode & 0x0F)])
        ln = len(payload)
        if ln < 126:
            header.append(0x80 | ln)
        elif ln < 65536:
            header.append(0x80 | 126)
            header.extend(struct.pack(">H", ln))
        else:
            header.append(0x80 | 127)
            header.extend(struct.pack(">Q", ln))
        header.extend(mask_key)
        masked = bytes(b ^ mask_key[i & 3] for i, b in enumerate(payload))
        self.sock.sendall(bytes(header) + masked)

    def send_text(self, text: str):
        self.send_Frame(0x01, text.encode("utf-8"))

    def recv_message(self):
        fragments = bytearray()
        while True:
            b1, b2 = self._recv_exact(2)
            fin = (b1 & 0x80) != 0
            opcode = b1 & 0x0F
            masked = (b2 & 0x80) != 0
            ln = b2 & 0x7F
            if ln == 126:
                ln = struct.unpack(">H", self._recv_exact(2))[0]
            elif ln == 127:
                ln = struct.unpack(">Q", self._recv_exact(8))[0]
            mask_key = self._recv_exact(4) if masked else None
            data = self._recv_exact(ln)
            if mask_key:
                data = bytes(b ^ mask_key[i & 3] for i, b in enumerate(data))

            if opcode == 0x08:  # Close
                raise ConnectionError("Server closed WebSocket")
            if opcode == 0x09:  # Ping -> send Pong
                self.send_Frame(0x0A, data)
                continue
            if opcode == 0x0A:  # Pong
                continue

            fragments.extend(data)
            if fin:
                return fragments.decode("utf-8", errors="replace")

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


class AISStreamTracker:
    """Maintains live dictionary of NY Harbor & Hudson River AIS vessels."""

    # Bounding box covering Hudson River (Midtown to Battery), Upper NY Bay & Narrows
    NY_HARBOR_BBOX = [[[40.58, -74.10], [40.84, -73.95]]]

    def __init__(self, api_key: str):
        self.api_key = api_key.strip()
        self.lock = threading.RLock()
        self.vessels = {}  # mmsi -> dict
        self.static_cache = {}  # mmsi -> {name, type_code, length_m, dest}
        self._last_cache_save = 0.0
        self._load_disk_cache()
        atexit.register(self._flush_disk_cache)
        self.connected = False
        self.messages_rx = 0
        self.last_error = ""

    def _load_disk_cache(self):
        try:
            src_path = AIS_CACHE_PATH if AIS_CACHE_PATH.exists() else (
                LEGACY_DROPBOX_CACHE_PATH if LEGACY_DROPBOX_CACHE_PATH.exists() else None
            )
            if src_path is not None:
                data = json.loads(src_path.read_text(encoding="utf-8"))
                now = time.time()
                self.static_cache = data.get("static_cache") or {}
                for mmsi, v in (data.get("vessels") or {}).items():
                    if now - v.get("pos_epoch", 0.0) < 480.0:
                        st_init = self.static_cache.get(mmsi) or {
                            "name": v.get("callsign", ""),
                            "type_code": 0,
                            "length_m": float(v.get("length_m") or 0.0),
                            "dest": "",
                        }
                        if is_blacklisted_vessel(mmsi, st_init.get("name") or v.get("callsign", "")):
                            continue
                        self._refresh_vessel_metadata(v, st_init)
                        self.vessels[mmsi] = v
                if src_path == LEGACY_DROPBOX_CACHE_PATH:
                    self._flush_disk_cache()
        except Exception:
            pass
        # Always clean up any legacy .ais_cache.json left inside the Dropbox folder
        for legacy in (LEGACY_DROPBOX_CACHE_PATH, LEGACY_DROPBOX_CACHE_PATH.with_suffix(".tmp")):
            try:
                if legacy.exists():
                    legacy.unlink()
            except Exception:
                pass

    def _flush_disk_cache(self):
        try:
            AIS_CACHE_PATH.parent.mkdir(parents=True, exist_ok=True)
            tmp = AIS_CACHE_PATH.with_suffix(".tmp")
            with self.lock:
                payload = json.dumps({"static_cache": self.static_cache, "vessels": self.vessels})
            tmp.write_text(payload, encoding="utf-8")
            tmp.replace(AIS_CACHE_PATH)
        except Exception:
            pass

    def _maybe_save_disk_cache(self, now: float):
        if now - self._last_cache_save < 30.0:
            return
        self._last_cache_save = now
        self._flush_disk_cache()

    def start(self):
        if not self.api_key:
            return
        threading.Thread(target=self._run_forever, daemon=True).start()

    def _run_forever(self):
        backoff = 2.0
        while True:
            ws = None
            try:
                ws = MinimalWSSClient("stream.aisstream.io", "/v0/stream", timeout=45.0)
                sub = {
                    "APIKey": self.api_key,
                    "BoundingBoxes": self.NY_HARBOR_BBOX,
                    "FilterMessageTypes": [
                        "PositionReport",
                        "StandardClassBPositionReport",
                        "ExtendedClassBPositionReport",
                        "ShipStaticData",
                        "StaticDataReport",
                    ],
                }
                ws.send_text(json.dumps(sub))
                with self.lock:
                    self.connected = True
                    self.last_error = ""
                backoff = 2.0

                while True:
                    raw = ws.recv_message()
                    self._handle_packet(raw)
            except Exception as exc:
                with self.lock:
                    self.connected = False
                    self.last_error = str(exc)
                time.sleep(backoff)
                backoff = min(30.0, backoff * 1.7)
            finally:
                if ws:
                    ws.close()

    def _handle_packet(self, raw: str):
        msg = json.loads(raw)
        if "error" in msg:
            raise RuntimeError(msg["error"])

        mtype = msg.get("MessageType", "")
        meta = msg.get("MetaData") or {}
        body = msg.get("Message") or {}
        mmsi = str(meta.get("MMSI") or meta.get("MMSI_String") or "").strip()
        if not mmsi:
            return

        now = time.time()
        meta_name = (meta.get("ShipName") or "").strip()

        with self.lock:
            self.messages_rx += 1
            st = self.static_cache.setdefault(mmsi, {"name": meta_name, "type_code": 0, "length_m": 0.0, "dest": ""})
            if meta_name and not st["name"]:
                st["name"] = meta_name

            if mtype == "ShipStaticData":
                sd = body.get("ShipStaticData") or {}
                nm = (sd.get("Name") or "").strip()
                if nm:
                    st["name"] = nm
                st["type_code"] = int(sd.get("Type") or st["type_code"] or 0)
                dim = sd.get("Dimension") or {}
                length = float((dim.get("A") or 0) + (dim.get("B") or 0))
                if length >= 5.0:
                    st["length_m"] = length
                dest = (sd.get("Destination") or "").strip()
                if dest:
                    st["dest"] = dest
                if mmsi in self.vessels:
                    self._refresh_vessel_metadata(self.vessels[mmsi], st)
                return

            if mtype == "StaticDataReport":
                sdr = body.get("StaticDataReport") or {}
                rep_a = sdr.get("ReportA") or {}
                rep_b = sdr.get("ReportB") or {}
                nm = (rep_a.get("Name") or "").strip()
                if nm:
                    st["name"] = nm
                if rep_b.get("ShipType"):
                    st["type_code"] = int(rep_b["ShipType"])
                dim = rep_b.get("Dimension") or {}
                length = float((dim.get("A") or 0) + (dim.get("B") or 0))
                if length >= 5.0:
                    st["length_m"] = length
                if mmsi in self.vessels:
                    self._refresh_vessel_metadata(self.vessels[mmsi], st)
                return

            pr = (
                body.get("PositionReport")
                or body.get("StandardClassBPositionReport")
                or body.get("ExtendedClassBPositionReport")
            )
            if not pr:
                return

            lat = pr.get("Latitude", meta.get("latitude"))
            lon = pr.get("Longitude", meta.get("longitude"))
            if lat is None or lon is None or abs(lat) > 90 or abs(lon) > 180:
                return

            nav_status = pr.get("NavigationalStatus")
            raw_sog = float(pr.get("Sog") or 0.0)
            sog = 0.0 if raw_sog > 100.0 else raw_sog  # 102.3 = AIS "not available" sentinel
            is_moored_status = (nav_status in (1, 5, 6)) and (sog < 1.5)
            if is_moored_status:
                sog = 0.0
            cog = float(pr.get("Cog") or 0.0)
            if cog >= 360.0:
                hdg = float(pr.get("TrueHeading") or 0.0)
                cog = hdg if hdg < 360.0 else 0.0

            if pr.get("Type"):
                st["type_code"] = int(pr["Type"])

            tx_epoch = _parse_aisstream_utc(meta.get("time_utc"), now)
            name = st["name"] or meta_name or f"VESSEL {mmsi}"
            if is_blacklisted_vessel(mmsi, name):
                self.vessels.pop(mmsi, None)
                return
            vessel = {
                "id": f"ais:{mmsi}",
                "mmsi": mmsi,
                "kind": "vessel",
                "source": "aisstream",
                "callsign": name,
                "subtitle": "Maritime Vessel",
                "lat": float(lat),
                "lon": float(lon),
                "alt_m": 2.5,
                "gs_kts": round(sog, 1),
                "track_deg": round(cog, 1),
                "vrate_mps": 0.0,
                "length_m": 28.0,
                "rarity": 1.5,
                "moored": is_moored_status,
                "pos_epoch": tx_epoch,
                "rx_epoch": now,
            }
            self._refresh_vessel_metadata(vessel, st)
            self.vessels[mmsi] = vessel

            # Prune stale vessels (> 8 minutes without position update) and persist warm cache
            if len(self.vessels) > 25:
                self.vessels = {
                    k: v for k, v in self.vessels.items() if now - v["pos_epoch"] < 480.0
                }
            self._maybe_save_disk_cache(now)

    @staticmethod
    def _refresh_vessel_metadata(vessel: dict, st: dict):
        mmsi_s = str(vessel.get("mmsi") or "")
        if st["name"]:
            vessel["callsign"] = st["name"]
        si_match = identify_staten_island_ferry(mmsi_s, vessel.get("callsign", ""))
        if si_match is not None:
            vessel["callsign"] = si_match[0]
        type_label, est_len, rarity = classify_ais_ship_type(
            st["type_code"], st["length_m"], vessel["callsign"], mmsi=mmsi_s
        )
        # Moored / docked boats (0.0-0.4 kts) get much lower priority than underway river traffic
        if vessel["gs_kts"] < 0.5:
            rarity *= 0.22
        vessel["length_m"] = st["length_m"] if st["length_m"] >= 6.0 else est_len
        vessel["rarity"] = rarity
        dest = f" -> {st['dest']}" if st.get("dest") else ""
        vessel["subtitle"] = f"{type_label}{dest}"

    def snapshot_vessels(self) -> list:
        now = time.time()
        with self.lock:
            return [
                dict(v)
                for mmsi, v in self.vessels.items()
                if now - v["pos_epoch"] < 480.0 and not is_blacklisted_vessel(mmsi, v.get("callsign", ""))
            ]
