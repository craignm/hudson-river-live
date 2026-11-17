"""
PTZ Hardware & Virtual Control Drivers (`ptz_drivers.py`)
Pure Python 3.9+ standard library — zero external pip dependencies.

Provides a unified `BasePTZDriver` interface for:
  1. `VirtualPTZDriver`  — Smooth virtual pan/tilt/zoom state used by `/?obs=1` digital 4K zoom
                           and Mission Control telemetry when operating with a fixed 4K camera.
  2. `ViscaOverIPDriver` — Non-blocking UDP VISCA-over-IP driver supporting both:
                           • FoMaKo / PTZOptics raw UDP VISCA (`port=1259`, 14.4 steps/deg)
                           • Sony SRG / BRC 8-byte sequenced header VISCA (`port=52381`, 1440 steps/deg)
  3. `AxisVapixDriver`   — HTTP Digest-authenticated `/axis-cgi/com/ptz.cgi` driver for Axis PTZs.
"""

import math
import socket
import struct
import time
import urllib.parse
import urllib.request


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
        dt = min(0.25, max(0.0, now - self.last_update))
        self.pan_rate = pan_deg_per_s
        self.tilt_rate = tilt_deg_per_s
        self.pan_deg = max(-170.0, min(170.0, self.pan_deg + pan_deg_per_s * dt))
        self.tilt_deg = max(-30.0, min(90.0, self.tilt_deg + tilt_deg_per_s * dt))
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


class ViscaOverIPDriver(BasePTZDriver):
    """
    VISCA-over-IP UDP driver supporting BOTH:
      - FoMaKo K30NS / PTZOptics (`raw_udp=True`, port 1259, 14.4 steps/deg)
      - Sony SRG-A40 / BRC-X400 (`raw_udp=False`, port 52381 w/ 8-byte Sony header, 1440 steps/deg)
    """

    def __init__(
        self,
        host: str,
        port: int = 1259,
        raw_udp: bool = True,
        steps_per_deg: float = 14.4,
        pan_offset_deg: float = 0.0,
        tilt_offset_deg: float = 0.0,
    ):
        self.host = host
        self.port = port
        self.raw_udp = raw_udp
        self.steps_per_deg = steps_per_deg
        self.pan_offset_deg = pan_offset_deg
        self.tilt_offset_deg = tilt_offset_deg
        self.seq = 1
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.last_zoom_x = 1.0

    def _send_visca(self, payload: bytes):
        if self.raw_udp:
            packet = payload
        else:
            header = struct.pack(">HH", 0x0100, len(payload)) + struct.pack(">I", self.seq)
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            packet = header + payload
        try:
            self.sock.sendto(packet, (self.host, self.port))
        except OSError:
            pass

    @staticmethod
    def _encode_nibble_16(val: int) -> bytes:
        val = val & 0xFFFF
        return bytes([
            (val >> 12) & 0x0F,
            (val >> 8) & 0x0F,
            (val >> 4) & 0x0F,
            val & 0x0F,
        ])

    def home(self):
        self._send_visca(bytes([0x81, 0x01, 0x06, 0x04, 0xFF]))

    def lock_focus_infinity(self):
        # 81 01 04 38 03 FF = Manual Focus; 81 01 04 48 00 00 00 00 FF = Optical Infinity
        self._send_visca(bytes([0x81, 0x01, 0x04, 0x38, 0x03, 0xFF]))
        self._send_visca(bytes([0x81, 0x01, 0x04, 0x48, 0x00, 0x00, 0x00, 0x00, 0xFF]))

    def set_zoom(self, zoom_x: float):
        # Standard VISCA 30x optical zoom curve: 0x0000 (1x, 58.1° HFOV) .. 0x4000 (30x, 2.14° HFOV)
        norm = min(1.0, max(0.0, math.log(max(1.0, zoom_x)) / math.log(30.0)))
        z_pos = int(round(norm * 0x4000))
        cmd = bytes([0x81, 0x01, 0x04, 0x47]) + self._encode_nibble_16(z_pos) + bytes([0xFF])
        self._send_visca(cmd)
        self.last_zoom_x = zoom_x

    def move_absolute(self, pan_deg: float, tilt_deg: float, zoom_x: float):
        adj_pan = max(-168.0, min(168.0, pan_deg + self.pan_offset_deg))
        adj_tilt = max(-28.0, min(88.0, tilt_deg + self.tilt_offset_deg))
        pan_steps = int(round(adj_pan * self.steps_per_deg))
        tilt_steps = int(round(adj_tilt * self.steps_per_deg))
        cmd = (
            bytes([0x81, 0x01, 0x06, 0x02, 0x12, 0x0E])
            + self._encode_nibble_16(pan_steps)
            + self._encode_nibble_16(tilt_steps)
            + bytes([0xFF])
        )
        self._send_visca(cmd)
        if abs(zoom_x - self.last_zoom_x) > 0.25:
            self.set_zoom(zoom_x)

    def move_velocity(self, pan_deg_per_s: float, tilt_deg_per_s: float, zoom_x: float):
        # Map deg/s to VISCA speed table (0x01..0x18 pan, 0x01..0x14 tilt)
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
