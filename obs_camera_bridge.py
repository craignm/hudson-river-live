"""
OBS WebSocket v5 Camera Bridge (`obs_camera_bridge.py`)
Pure Python 3.9+ standard library — zero external pip dependencies.

Connects `server.py` to OBS Studio over WebSocket (`obs://127.0.0.1:4455` when running
alongside OBS on the Mac mini, or `obs://<host>:4455` over LAN during local development).

Note on architecture:
  - OBS's broadcast AR overlay (`http://127.0.0.1:8765/?hud=1`) is transparent HTML/Canvas
    and does NOT pull video frames through this bridge.
  - Instead, this bridge serves the live video backdrop inside the Mission Control web UI
    (`/` and `/?obs=1`) when an operator opens a browser tab to inspect targets or calibrate:
      1. Zero-Load Standby (default): Keeps an idle WebSocket open to OBS (`0` screenshots/sec)
         whenever no Mission Control browser tab has requested frames in the last 4 seconds,
         leaving OBS's Metal/OpenGL render loop completely untouched during 24/7 streaming.
      2. Instant On-Demand Wakeup (~50 ms): As soon as `/api/cam_frame.jpg` is requested,
         wakes the worker thread to pull lightweight 720p (~6 fps) JPEG scene snapshots,
         then automatically returns to standby 4 seconds after the browser tab closes.
      3. Manual Camera Reset (`POST /api/reset_camera`): Cycles OBS's `macos-avcapture`
         input (`device: ""` -> restore) on demand from Mission Control.
"""

import base64
import hashlib
import json
import os
from pathlib import Path
import select
import socket
import struct
import subprocess
import threading
import time
from typing import Optional, Tuple
from urllib.parse import urlparse


# On-demand Mission Control preview settings (kept gentle to protect 4K30 broadcast headroom)
OBS_SCREENSHOT_WIDTH = int(os.environ.get("OBS_SCREENSHOT_WIDTH", "1280"))
OBS_SCREENSHOT_HEIGHT = int(os.environ.get("OBS_SCREENSHOT_HEIGHT", "720"))
OBS_SCREENSHOT_QUALITY = int(os.environ.get("OBS_SCREENSHOT_QUALITY", "78"))
OBS_SCREENSHOT_FPS = max(1.0, min(15.0, float(os.environ.get("OBS_SCREENSHOT_FPS", "6.0"))))
OBS_IDLE_TIMEOUT_SEC = 4.0

# Optional built-in freeze watchdog. Disabled by default ("0") because the dedicated
# `obs_freeze_monitor.py` LaunchAgent handles pixel-diff monitoring, ntfy.sh push alerts,
# and 2-stage recovery (OBS soft reset + IOKit USB bus re-enumeration).
OBS_WATCHDOG_ENABLED = os.environ.get("OBS_WATCHDOG_ENABLED", "0").strip() in ("1", "true", "True", "yes")
OBS_WATCHDOG_INTERVAL_SEC = max(10.0, float(os.environ.get("OBS_WATCHDOG_INTERVAL_SEC", "20.0")))
OBS_WATCHDOG_FROZEN_COUNT = max(2, int(os.environ.get("OBS_WATCHDOG_FROZEN_COUNT", "3")))
OBS_WATCHDOG_MIN_COOLDOWN_SEC = 180.0
OBS_WATCHDOG_MAX_COOLDOWN_SEC = 900.0

CAMLINK_4K_NV12_SETTINGS = {
    "device_name": "Cam Link 4K",
    "enable_audio": False,
    "use_preset": False,
    "supported_format": "Mzg0MHgyMTYwIDI5Ljk3MC0yOS45NzAgMiA4NzU3MDQ0Mzg=",
    "resolution": '{"width":3840,"height":2160}',
    "input_format": 875704438,
    "color_space": 2,
    "video_range": 1,
}

OBS_PTZ_YT_SCENE = "PTZ YouTube Broadcast"
OBS_PTZ_YT_HUD_INPUT = "PTZ Broadcast HUD"
OBS_PTZ_YT_FILTER = "PTZ YouTube Stream"
OBS_MONTHLY_REC_FILTER = "Monthly Synoptic 4K Recorder"
OBS_YOUTUBE_RTMPS_SERVER = "rtmps://a.rtmps.youtube.com:443/live2"
# Exact integer 16:9 top-right inset geometry on a 3840x2160 4K canvas (matches static/ptz_yt_hud.html)
OBS_INSET_X = 2848.0
OBS_INSET_Y = 81.0
OBS_INSET_W = 896.0
OBS_INSET_H = 504.0


class OBSRPCError(RuntimeError):
    """Raised when an obs-websocket v5 request returns `requestStatus.result == False`."""


class PlainWSClient:
    """
    Minimal RFC 6455 `ws://` client using Python's built-in `socket` module.
    Handles client-to-server frame masking, fragmented server messages, and Ping/Pong keepalives.
    """

    def __init__(self, host: str, port: int = 4455, path: str = "/", timeout: float = 5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except OSError:
            pass

        # RFC 6455 opening HTTP Upgrade handshake
        ws_key = base64.b64encode(os.urandom(16)).decode("ascii")
        req = (
            f"GET {path} HTTP/1.1\r\n"
            f"Host: {host}:{port}\r\n"
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
                raise ConnectionError("EOF in OBS WebSocket handshake")
            resp += chunk
        if b"101" not in resp.split(b"\r\n", 1)[0]:
            raise ConnectionError("OBS WebSocket handshake rejected")
        # Preserve any WebSocket frame bytes that arrived coalesced with the HTTP 101 headers
        self._leftover = resp.split(b"\r\n\r\n", 1)[1]

    def _recv_exact(self, n: int) -> bytes:
        buf = bytearray(self._leftover[:n])
        self._leftover = self._leftover[n:]
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("OBS WebSocket closed")
            buf.extend(chunk)
        return bytes(buf)

    def has_pending_data(self) -> bool:
        """Non-blocking check for unsolicited Ping or Event frames while parked in standby."""
        if self._leftover:
            return True
        try:
            readable, _, _ = select.select([self.sock], [], [], 0.0)
            return bool(readable)
        except (OSError, ValueError):
            return False

    def send_frame(self, opcode: int, payload: bytes):
        """Sends a masked RFC 6455 client frame (FIN=1)."""
        mask = os.urandom(4)
        hdr = bytearray([0x80 | (opcode & 0x0F)])
        ln = len(payload)
        if ln < 126:
            hdr.append(0x80 | ln)
        elif ln < 65536:
            hdr.append(0x80 | 126)
            hdr.extend(struct.pack(">H", ln))
        else:
            hdr.append(0x80 | 127)
            hdr.extend(struct.pack(">Q", ln))
        hdr.extend(mask)
        masked = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
        self.sock.sendall(bytes(hdr) + masked)

    def send_json(self, obj: dict):
        self.send_frame(0x01, json.dumps(obj).encode("utf-8"))

    def recv_json(self) -> dict:
        """Reads and reassembles a complete JSON text message, auto-replying to Ping frames."""
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
            mask = self._recv_exact(4) if masked else None
            data = self._recv_exact(ln)
            if mask:
                data = bytes(b ^ mask[i & 3] for i, b in enumerate(data))
            if opcode == 0x08:  # Connection Close
                raise ConnectionError("OBS WebSocket closed")
            if opcode == 0x09:  # Ping -> Pong
                self.send_frame(0x0A, data)
                continue
            if opcode == 0x0A:  # Pong
                continue
            fragments.extend(data)
            if fin:
                return json.loads(fragments.decode("utf-8", errors="replace"))

    def drain_incoming(self):
        """Drains any queued Ping/Event frames while the bridge is idle."""
        while self.has_pending_data():
            self.recv_json()

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


class OBSCameraBridge:
    """
    On-demand OBS WebSocket v5 frame bridge for Mission Control.
    Sleeps at 0% GPU/CPU load when no browser viewer is active, and wakes immediately
    when `get_frame(demand=True)` is called by `/api/cam_frame.jpg` or `/api/cam_stream.mjpg`.
    """

    def __init__(self, initial_url: str = "", preferred_ptz: str = "sony_a40"):
        self.lock = threading.Lock()
        self._frame_cond = threading.Condition(self.lock)
        self._wake_event = threading.Event()
        self._manual_reset_event = threading.Event()

        self.url = ""
        self.preferred_ptz: str = preferred_ptz if preferred_ptz in ("sony_a40", "fomako") else "sony_a40"
        self.latest_jpeg: bytes = b""
        self.latest_ts: float = 0.0
        self.last_demand_ts: float = 0.0
        self.ptz_latest_jpeg: bytes = b""
        self.ptz_latest_ts: float = 0.0
        self.ptz_last_demand_ts: float = 0.0
        self._last_alive_ts: float = 0.0
        self.connected: bool = False
        self.status_text: str = "Not configured"
        self.camera_input_name: str = ""
        self.ptz_input_name: str = ""
        self.watchdog_resets: int = 0
        self.last_watchdog_reset_ts: float = 0.0
        self._pending_device_restore: Optional[Tuple[str, str]] = None
        self._version = 0

        # Dedicated PTZ YouTube Broadcast scene + Source Record RTMPS output state
        self._want_ptz_inset_visible: bool = True
        self._applied_ptz_inset_visible: Optional[bool] = None
        self.ptz_youtube_enabled: bool = False
        self.ptz_youtube_key: str = ""
        self._ptz_yt_config_seq: int = 0
        self.ptz_yt_scene_ready: bool = False
        self.ptz_yt_streaming: bool = False
        self.ptz_yt_status_text: str = "Provisioning OBS scene…"

        if initial_url:
            self.set_url(initial_url)
        threading.Thread(target=self._worker_loop, daemon=True).start()

    def set_ptz_inset_visible(self, visible: bool):
        """Shows or hides the wide overview PiP camera inside `PTZ YouTube Broadcast` (auto-hidden at 1x)."""
        vis = bool(visible)
        with self.lock:
            if self._want_ptz_inset_visible == vis:
                return
            self._want_ptz_inset_visible = vis
        self._wake_event.set()

    def set_ptz_youtube_config(self, stream_key: Optional[str] = None, enabled: Optional[bool] = None):
        """Updates the separate YouTube RTMPS stream key & live toggle for `PTZ YouTube Broadcast`."""
        changed = False
        with self.lock:
            if stream_key is not None:
                clean_k = str(stream_key).strip()
                if clean_k != self.ptz_youtube_key:
                    self.ptz_youtube_key = clean_k
                    changed = True
            if enabled is not None:
                en = bool(enabled)
                if en != self.ptz_youtube_enabled:
                    self.ptz_youtube_enabled = en
                    changed = True
            if changed:
                self._ptz_yt_config_seq += 1
        if changed:
            self._wake_event.set()

    def get_ptz_youtube_status(self) -> dict:
        """Returns sanitized status for `/api/state` (never exposes the raw stream key)."""
        with self.lock:
            has_key = bool(self.ptz_youtube_key)
            masked = f"••••-{self.ptz_youtube_key[-4:]}" if len(self.ptz_youtube_key) >= 4 else ("••••" if has_key else "")
            return {
                "enabled": self.ptz_youtube_enabled,
                "has_key": has_key,
                "masked_key": masked,
                "scene_ready": self.ptz_yt_scene_ready,
                "streaming": self.ptz_yt_streaming,
                "inset_visible": self._want_ptz_inset_visible,
                "status_text": self.ptz_yt_status_text,
            }

    def set_preferred_ptz(self, preferred_ptz: str):
        """Hot-switches the active PTZ video capture input in OBS ('sony_a40' Cam Link 4K vs 'fomako' USB)."""
        norm = "fomako" if str(preferred_ptz).strip().lower() == "fomako" else "sony_a40"
        with self.lock:
            if self.preferred_ptz == norm:
                return
            self.preferred_ptz = norm
            self.ptz_input_name = ""
            self.ptz_latest_jpeg = b""
            self.ptz_latest_ts = 0.0
            self._version += 1
        self._wake_event.set()

    def set_url(self, new_url: str):
        """Updates target OBS WebSocket URL (e.g. `obs://127.0.0.1:4455` or `obs://:pass@host:4455`)."""
        u = (new_url or "").strip()
        if u and "://" not in u:
            u = f"obs://{u}" if ":" in u else f"obs://{u}:4455"
        with self.lock:
            self.url = u
            self._version += 1
            self.connected = False
            self.camera_input_name = ""
            self.status_text = f"Connecting to {u}…" if u else "Not configured"
        self._wake_event.set()

    def get_frame(self, demand: bool = True, wait_fresh: float = 1.0) -> Tuple[bytes, float]:
        """
        Returns `(jpeg_bytes, timestamp)` for Mission Control.
        When `demand=True`, wakes the worker out of standby and optionally waits up to
        `wait_fresh` seconds if the cached frame is older than 1.5s.
        """
        now = time.time()
        with self._frame_cond:
            if not demand:
                return self.latest_jpeg, self.latest_ts
            was_idle = (now - self.last_demand_ts) > OBS_IDLE_TIMEOUT_SEC
            self.last_demand_ts = now
            need_wait = wait_fresh > 0.0 and self.connected and (
                not self.latest_jpeg or (now - self.latest_ts) > 1.5
            )
            if was_idle or need_wait:
                self._wake_event.set()
            if need_wait:
                prev_ts = self.latest_ts
                deadline = time.time() + wait_fresh
                while self.connected and self.latest_ts == prev_ts:
                    rem = deadline - time.time()
                    if rem <= 0:
                        break
                    self._frame_cond.wait(timeout=rem)
            return self.latest_jpeg, self.latest_ts

    def get_ptz_frame(self, demand: bool = True, wait_fresh: float = 1.0) -> Tuple[bytes, float]:
        """
        Returns `(jpeg_bytes, timestamp)` from the hardware FoMaKo PTZ OBS input
        (`Video Capture Device 3 (FoMaKo)` / USB vendor `0x33f1`) on demand.
        """
        now = time.time()
        with self._frame_cond:
            if not demand:
                return self.ptz_latest_jpeg, self.ptz_latest_ts
            was_idle = (now - self.ptz_last_demand_ts) > OBS_IDLE_TIMEOUT_SEC
            self.ptz_last_demand_ts = now
            need_wait = wait_fresh > 0.0 and self.connected and bool(self.ptz_input_name) and (
                not self.ptz_latest_jpeg or (now - self.ptz_latest_ts) > 1.5
            )
            if was_idle or need_wait:
                self._wake_event.set()
            if need_wait:
                prev_ts = self.ptz_latest_ts
                deadline = time.time() + wait_fresh
                while self.connected and self.ptz_latest_ts == prev_ts:
                    rem = deadline - time.time()
                    if rem <= 0:
                        break
                    self._frame_cond.wait(timeout=rem)
            return self.ptz_latest_jpeg, self.ptz_latest_ts

    def request_camera_reset(self) -> dict:
        """Schedules an immediate soft cycle of OBS's `macos-avcapture` camera input."""
        with self.lock:
            if not self.connected:
                return {"ok": False, "error": "OBS WebSocket is not connected"}
            cam_name = self.camera_input_name or "macos-avcapture"
        self._manual_reset_event.set()
        self._wake_event.set()
        return {"ok": True, "scheduled": True, "camera_input": cam_name}

    def publish_ptz_preview(self, jpeg: bytes):
        """Publishes a freshly captured FoMaKo calibration frame into Mission Control's PTZ preview cache."""
        if not jpeg:
            return
        now = time.time()
        with self._frame_cond:
            self.ptz_latest_jpeg = jpeg
            self.ptz_latest_ts = now
            self._frame_cond.notify_all()

    def open_calibration_session(self, timeout: float = 8.0) -> "OBSCalibrationSession":
        """
        Opens a dedicated synchronous OBS WebSocket v5 session for automated PTZ calibration.
        Captures clean, uncached raw sensor frames directly from the 4K Insta360 input and
        1080p/4K FoMaKo PTZ input without touching OBS's active broadcast Program scene.
        """
        with self.lock:
            url = self.url or "obs://127.0.0.1:4455"
            syn_hint = self.camera_input_name
            ptz_hint = self.ptz_input_name
        return OBSCalibrationSession(self, url, syn_hint=syn_hint, ptz_hint=ptz_hint, timeout=timeout)

    def _one_shot_obs_rpc(self, requests: list, timeout: float = 6.0) -> bool:
        """Executes a short list of `(requestType, requestData)` calls over a dedicated OBS WebSocket."""
        with self.lock:
            url = self.url or "obs://127.0.0.1:4455"
        parsed = urlparse(url if "://" in url else f"obs://{url}")
        host = parsed.hostname or "127.0.0.1"
        port = parsed.port or 4455
        password = parsed.password or ""
        ws = PlainWSClient(host, port, timeout=timeout)
        try:
            hello = ws.recv_json()
            d = hello.get("d") or {}
            auth_info = d.get("authentication")
            ident = {"rpcVersion": 1, "eventSubscriptions": 0}
            if auth_info and password:
                secret = base64.b64encode(
                    hashlib.sha256((password + auth_info["salt"]).encode("utf-8")).digest()
                ).decode("ascii")
                ident["authentication"] = base64.b64encode(
                    hashlib.sha256((secret + auth_info["challenge"]).encode("utf-8")).digest()
                ).decode("ascii")
            ws.send_json({"op": 1, "d": ident})
            if ws.recv_json().get("op") != 2:
                return False
            for idx, (rtype, rdata) in enumerate(requests, 1):
                rid = f"os_{idx}"
                ws.send_json({"op": 6, "d": {"requestType": rtype, "requestId": rid, "requestData": rdata}})
                while True:
                    msg = ws.recv_json()
                    if msg.get("op") == 7 and (msg.get("d") or {}).get("requestId") == rid:
                        break
            return True
        except Exception as exc:
            print(f"[OBS Bridge] One-shot RPC failed: {exc}", flush=True)
            return False
        finally:
            ws.close()

    def start_synoptic_recording(self, output_dir: str, filename_stem: str) -> bool:
        """
        Starts zero-cost 4K30 recording via OBS's native `StartRecord` (`RecEncoder=none` = "Use stream encoder"),
        muxing the already-encoded 4K30 HEVC packets from the live Synoptic stream directly to disk with
        zero extra VideoToolbox encoder sessions and zero OpenGL `source-record` GPU readbacks.
        """
        with self.lock:
            syn_src = self.camera_input_name or "Video Capture Device 2"
        return self._one_shot_obs_rpc(
            [
                (
                    "RemoveSourceFilter",
                    {
                        "sourceName": syn_src,
                        "filterName": OBS_MONTHLY_REC_FILTER,
                    },
                ),
                (
                    "SetCurrentProgramScene",
                    {"sceneName": "Scene"},
                ),
                (
                    "StartStream",
                    {},
                ),
                (
                    "SetRecordDirectory",
                    {"recordDirectory": output_dir},
                ),
                (
                    "SetProfileParameter",
                    {
                        "parameterCategory": "Output",
                        "parameterName": "FilenameFormatting",
                        "parameterValue": filename_stem,
                    },
                ),
                (
                    "StartRecord",
                    {},
                ),
            ]
        )

    def stop_synoptic_recording(self) -> bool:
        """Cleanly stops OBS's native `StartRecord` output so OBS finalizes the `hybrid_mov` trailer."""
        with self.lock:
            syn_src = self.camera_input_name or "Video Capture Device 2"
        return self._one_shot_obs_rpc(
            [
                (
                    "StopRecord",
                    {},
                ),
                (
                    "RemoveSourceFilter",
                    {
                        "sourceName": syn_src,
                        "filterName": OBS_MONTHLY_REC_FILTER,
                    },
                ),
            ]
        )

    def start_ptz_recording(self, output_dir: str, filename_stem: str) -> bool:
        """
        Starts zero-cost 4K30 piggyback recording on the active `PTZ YouTube Stream` filter
        (`PTZ YouTube Broadcast` scene: 30x Sony A40 + chyrons + top-right wide overview inset),
        attaching `mov output` directly to the live hardware encoder with zero extra GPU/encoder load.
        """
        return self._one_shot_obs_rpc(
            [
                (
                    "SetSourceFilterSettings",
                    {
                        "sourceName": OBS_PTZ_YT_SCENE,
                        "filterName": OBS_PTZ_YT_FILTER,
                        "filterSettings": {
                            "record_mode": 1,
                            "path": output_dir,
                            "filename_formatting": filename_stem,
                            "rec_format": "hybrid_mov",
                        },
                        "overlay": True,
                    },
                ),
            ]
        )

    def stop_ptz_recording(self) -> bool:
        """Stops piggyback `.mov` file recording on `PTZ YouTube Stream` without interrupting its live RTMPS stream."""
        return self._one_shot_obs_rpc(
            [
                (
                    "SetSourceFilterSettings",
                    {
                        "sourceName": OBS_PTZ_YT_SCENE,
                        "filterName": OBS_PTZ_YT_FILTER,
                        "filterSettings": {"record_mode": 0},
                        "overlay": True,
                    },
                ),
            ]
        )

    def get_status(self) -> dict:
        """Returns connection & standby telemetry for `/api/state`."""
        with self.lock:
            now = time.time()
            idle = (now - self.last_demand_ts) > OBS_IDLE_TIMEOUT_SEC and (
                now - self.ptz_last_demand_ts
            ) > OBS_IDLE_TIMEOUT_SEC
            alive = self.connected and ((now - self._last_alive_ts) < 45.0)
            suffix_parts = []
            if alive:
                suffix_parts.append("standby" if idle else f"live ~{OBS_SCREENSHOT_FPS:.0f}fps")
                if self.ptz_input_name:
                    if "sony" in self.ptz_input_name.lower() or "cam link" in self.ptz_input_name.lower():
                        suffix_parts.append("Sony A40 Cam Link ready")
                    else:
                        suffix_parts.append("FoMaKo USB ready")
                if OBS_WATCHDOG_ENABLED and self.camera_input_name:
                    if self.watchdog_resets > 0:
                        suffix_parts.append(f"watchdog: {self.watchdog_resets} reset(s)")
                    else:
                        suffix_parts.append("watchdog armed")
            status_display = (
                f"{self.status_text} ({', '.join(suffix_parts)})"
                if suffix_parts
                else self.status_text
            )
            return {
                "url": self.url,
                "connected": alive,
                "standby": bool(alive and idle),
                "status_text": status_display,
                "frame_bytes": len(self.latest_jpeg),
                "camera_input": self.camera_input_name,
                "ptz_available": bool(alive and self.ptz_input_name),
                "ptz_input": self.ptz_input_name,
                "watchdog_resets": self.watchdog_resets,
                "last_watchdog_reset_ts": self.last_watchdog_reset_ts,
            }

    def _worker_loop(self):
        """Supervisor loop that maintains and reconnects the OBS WebSocket session."""
        while True:
            with self.lock:
                url = self.url
                ver = self._version
            if not url:
                self._wake_event.wait(timeout=0.5)
                self._wake_event.clear()
                continue

            try:
                self._run_obs_ws(url, ver)
            except Exception as exc:
                with self._frame_cond:
                    if self._version == ver:
                        self.connected = False
                        self.status_text = f"Error: {exc}"
                    self._frame_cond.notify_all()
                time.sleep(2.0)

    def _run_obs_ws(self, url: str, ver: int):
        """Runs a single authenticated OBS WebSocket v5 session."""
        parsed = urlparse(url if "://" in url else f"obs://{url}")
        host = parsed.hostname or "127.0.0.1"
        port = parsed.port or 4455
        password = parsed.password or ""

        ws = PlainWSClient(host, port, timeout=5.0)
        try:
            # 1. OBS WebSocket v5 Hello (op=0) -> Identify (op=1) -> Identified (op=2)
            hello = ws.recv_json()
            d = hello.get("d") or {}
            auth_info = d.get("authentication")
            ident = {"rpcVersion": 1, "eventSubscriptions": 0}
            if auth_info and password:
                secret = base64.b64encode(
                    hashlib.sha256((password + auth_info["salt"]).encode("utf-8")).digest()
                ).decode("ascii")
                auth_resp = base64.b64encode(
                    hashlib.sha256((secret + auth_info["challenge"]).encode("utf-8")).digest()
                ).decode("ascii")
                ident["authentication"] = auth_resp

            ws.send_json({"op": 1, "d": ident})
            identified = ws.recv_json()
            if identified.get("op") != 2:
                raise PermissionError("OBS WebSocket auth failed (check password)")

            req_seq = 0

            def obs_rpc(req_type: str, req_data: Optional[dict] = None, check: bool = True) -> dict:
                nonlocal req_seq
                req_seq += 1
                req_id = f"r_{req_seq}"
                payload = {"requestType": req_type, "requestId": req_id}
                if req_data is not None:
                    payload["requestData"] = req_data
                ws.send_json({"op": 6, "d": payload})
                while True:
                    msg = ws.recv_json()
                    if msg.get("op") == 7:
                        md = msg.get("d") or {}
                        if md.get("requestId") == req_id:
                            st = md.get("requestStatus") or {}
                            if check and not st.get("result", False):
                                raise OBSRPCError(
                                    f"{req_type} failed (code={st.get('code')}): {st.get('comment', '')}"
                                )
                            return md.get("responseData") or {}

            def discover_cameras() -> Tuple[str, str]:
                """Returns `(synoptic_insta360_input, active_ptz_input)` matching `self.preferred_ptz`."""
                with self.lock:
                    pref = self.preferred_ptz
                resp = obs_rpc("GetInputList", check=False)
                av_names = []
                for inp in resp.get("inputs") or []:
                    kind = inp.get("unversionedInputKind") or inp.get("inputKind") or ""
                    if kind.startswith("macos-avcapture") or kind.startswith("av_capture_input"):
                        nm = inp.get("inputName") or ""
                        if nm:
                            av_names.append(nm)
                syn_in = ""
                fomako_in = ""
                sony_in = ""
                for nm in av_names:
                    cur = obs_rpc("GetInputSettings", {"inputName": nm}, check=False).get("inputSettings") or {}
                    dev = str(cur.get("device") or "").lower()
                    dname = str(cur.get("device_name") or "").lower()
                    low_nm = nm.lower()
                    if "0fd9" in dev or "cam link" in dname or "sony" in low_nm or "cam link" in low_nm:
                        if not sony_in:
                            sony_in = nm
                    elif "33f1" in dev or "uhd camera" in dname or "fomako" in low_nm:
                        if not fomako_in:
                            fomako_in = nm
                    elif "2e1a" in dev or "insta360" in dname:
                        if not syn_in:
                            syn_in = nm

                # Auto-provision non-broadcast 'Sony A40 PTZ' scene + Cam Link 4K input if attached via USB
                if pref == "sony_a40" and not sony_in and av_names:
                    props = obs_rpc(
                        "GetInputPropertiesListPropertyItems",
                        {"inputName": av_names[0], "propertyName": "device"},
                        check=False,
                    )
                    camlink_id = ""
                    camlink_name = "Cam Link 4K"
                    for it in props.get("propertyItems") or []:
                        ival = str(it.get("itemValue") or "")
                        iname = str(it.get("itemName") or "")
                        if "0fd9" in ival.lower() or "cam link" in iname.lower():
                            camlink_id = ival
                            camlink_name = iname or "Cam Link 4K"
                            break
                    if camlink_id:
                        slist = obs_rpc("GetSceneList", check=False)
                        snames = [s.get("sceneName") for s in (slist.get("scenes") or [])]
                        if "Sony A40 PTZ" not in snames:
                            obs_rpc("CreateScene", {"sceneName": "Sony A40 PTZ"}, check=False)
                        new_in = "Video Capture Device 4 (Sony A40)"
                        obs_rpc(
                            "CreateInput",
                            {
                                "sceneName": "Sony A40 PTZ",
                                "inputName": new_in,
                                "inputKind": "macos-avcapture-fast",
                                "inputSettings": {
                                    "device": camlink_id,
                                    **CAMLINK_4K_NV12_SETTINGS,
                                },
                                "sceneItemEnabled": True,
                            },
                            check=False,
                        )
                        sony_in = new_in

                ptz_in = (sony_in or fomako_in) if pref == "sony_a40" else (fomako_in or sony_in)
                if not syn_in:
                    for nm in av_names:
                        if nm not in (sony_in, fomako_in):
                            syn_in = nm
                            break
                return syn_in, ptz_in

            def discover_camera_input() -> str:
                return discover_cameras()[0]

            def cycle_camera_input(cam_name: str, reason: str) -> bool:
                s_resp = obs_rpc("GetInputSettings", {"inputName": cam_name}, check=False)
                settings = s_resp.get("inputSettings") or {}
                orig_dev = settings.get("device") or ""
                if not orig_dev and self._pending_device_restore and self._pending_device_restore[0] == cam_name:
                    orig_dev = self._pending_device_restore[1]
                if not orig_dev:
                    print(f"[OBS Bridge] Cannot cycle '{cam_name}': device ID is empty.")
                    return False

                ts_str = time.strftime("%Y-%m-%d %H:%M:%S")
                is_camlink = "0fd9" in orig_dev.lower() or "cam link" in str(settings.get("device_name") or "").lower()
                print(f"[OBS Bridge {ts_str}] Cycling '{cam_name}' ({orig_dev}) — {reason}", flush=True)
                self._pending_device_restore = (cam_name, orig_dev)
                try:
                    obs_rpc(
                        "SetInputSettings",
                        {"inputName": cam_name, "inputSettings": {"device": ""}, "overlay": True},
                    )
                    if is_camlink:
                        uvc_bin = Path(__file__).resolve().parent / "uvc-util"
                        if uvc_bin.is_file():
                            subprocess.run(
                                [str(uvc_bin), "-V", "0x0fd9:0x0066", "--usb-reset"],
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL,
                                check=False,
                            )
                        time.sleep(2.2)
                    else:
                        time.sleep(1.2)
                finally:
                    restore_cfg = {"device": orig_dev, **CAMLINK_4K_NV12_SETTINGS} if is_camlink else {"device": orig_dev}
                    obs_rpc(
                        "SetInputSettings",
                        {"inputName": cam_name, "inputSettings": restore_cfg, "overlay": not is_camlink},
                    )
                    self._pending_device_restore = None

                now_reset = time.time()
                with self.lock:
                    self.watchdog_resets += 1
                    self.last_watchdog_reset_ts = now_reset
                    reset_num = self.watchdog_resets
                print(
                    f"[OBS Bridge {ts_str}] Restored '{cam_name}' successfully (reset #{reset_num}).",
                    flush=True,
                )
                return True

            # Crash-safe restore if a previous connection dropped mid-reset
            if self._pending_device_restore:
                pend_name, pend_dev = self._pending_device_restore
                cur = (obs_rpc("GetInputSettings", {"inputName": pend_name}, check=False).get("inputSettings") or {})
                if not cur.get("device"):
                    obs_rpc(
                        "SetInputSettings",
                        {"inputName": pend_name, "inputSettings": {"device": pend_dev}, "overlay": True},
                        check=False,
                    )
                self._pending_device_restore = None

            scene_data = obs_rpc("GetCurrentProgramScene", check=False)
            scene_name = scene_data.get("currentProgramSceneName") or "Scene"
            if scene_name != "Scene":
                s_all = [s.get("sceneName") for s in (obs_rpc("GetSceneList", check=False).get("scenes") or [])]
                if "Scene" in s_all:
                    obs_rpc("SetCurrentProgramScene", {"sceneName": "Scene"}, check=False)
                    scene_name = "Scene"
            # Ensure main 24/7 Synoptic stream is running (e.g. if OBS was launched manually without --startstreaming)
            st_init = obs_rpc("GetStreamStatus", check=False)
            if not st_init.get("outputActive", False) and not st_init.get("outputReconnecting", False):
                obs_rpc("StartStream", check=False)
            cam_input_name, ptz_input_name = discover_cameras()
            if cam_input_name:
                # Ensure any legacy duplicate 4K source-record filter is removed from Video Capture Device 2
                obs_rpc(
                    "RemoveSourceFilter",
                    {"sourceName": cam_input_name, "filterName": OBS_MONTHLY_REC_FILTER},
                    check=False,
                )
            ptz_yt_inset_item_id: Optional[int] = None
            applied_yt_seq = -1

            def ensure_ptz_youtube_broadcast_scene(syn_in: str, ptz_in: str) -> Optional[int]:
                """
                Creates/maintains the 4K `PTZ YouTube Broadcast` OBS scene without ever altering
                the live `Scene` Program output:
                  Layer 0 (bottom): Raw PTZ 4K camera (`Video Capture Device 4 (Sony A40)`)
                  Layer 1 (middle): Wide Synoptic PiP (`Video Capture Device 2` @ top-right 896x504)
                  Layer 2 (top):    Transparent 4K Browser HUD (`http://127.0.0.1:8765/ptz_yt_hud.html`)
                """
                if not ptz_in:
                    return None
                slist = obs_rpc("GetSceneList", check=False)
                snames = [s.get("sceneName") for s in (slist.get("scenes") or [])]
                if OBS_PTZ_YT_SCENE not in snames:
                    obs_rpc("CreateScene", {"sceneName": OBS_PTZ_YT_SCENE}, check=False)

                # Ensure Browser Source HUD input exists with up-to-date 4K30 URL
                hud_settings = {
                    "url": "http://127.0.0.1:8765/ptz_yt_hud.html",
                    "width": 3840,
                    "height": 2160,
                    "fps": 30,
                    "fps_custom": True,
                    "reroute_audio": False,
                    "shutdown": False,
                    "restart_when_active": False,
                }
                in_list = obs_rpc("GetInputList", check=False).get("inputs") or []
                has_hud_input = any((i.get("inputName") == OBS_PTZ_YT_HUD_INPUT) for i in in_list)
                if not has_hud_input:
                    obs_rpc(
                        "CreateInput",
                        {
                            "sceneName": OBS_PTZ_YT_SCENE,
                            "inputName": OBS_PTZ_YT_HUD_INPUT,
                            "inputKind": "browser_source",
                            "inputSettings": hud_settings,
                            "sceneItemEnabled": True,
                        },
                        check=False,
                    )
                else:
                    obs_rpc(
                        "SetInputSettings",
                        {"inputName": OBS_PTZ_YT_HUD_INPUT, "inputSettings": hud_settings, "overlay": True},
                        check=False,
                    )
                    obs_rpc(
                        "PressInputPropertiesButton",
                        {"inputName": OBS_PTZ_YT_HUD_INPUT, "propertyName": "refreshnocache"},
                        check=False,
                    )

                items = obs_rpc("GetSceneItemList", {"sceneName": OBS_PTZ_YT_SCENE}, check=False).get("sceneItems") or []
                by_name = {it.get("sourceName"): it.get("sceneItemId") for it in items if it.get("sourceName")}

                def ensure_item(src_name: str, enabled: bool = True) -> Optional[int]:
                    if not src_name:
                        return None
                    iid = by_name.get(src_name)
                    if iid is None:
                        cr = obs_rpc(
                            "CreateSceneItem",
                            {"sceneName": OBS_PTZ_YT_SCENE, "sourceName": src_name, "sceneItemEnabled": enabled},
                            check=False,
                        )
                        iid = cr.get("sceneItemId")
                        if iid is not None:
                            by_name[src_name] = iid
                    return iid

                with self.lock:
                    want_vis = self._want_ptz_inset_visible

                ptz_iid = ensure_item(ptz_in, True)
                syn_iid = ensure_item(syn_in, want_vis) if syn_in else None
                hud_iid = ensure_item(OBS_PTZ_YT_HUD_INPUT, True)

                # Remove any stale PTZ input if camera model switched
                for src_nm, iid in list(by_name.items()):
                    if src_nm not in (ptz_in, syn_in, OBS_PTZ_YT_HUD_INPUT) and iid is not None:
                        obs_rpc(
                            "RemoveSceneItem",
                            {"sceneName": OBS_PTZ_YT_SCENE, "sceneItemId": iid},
                            check=False,
                        )

                full_4k_xf = {
                    "positionX": 0.0,
                    "positionY": 0.0,
                    "alignment": 5,
                    "boundsType": "OBS_BOUNDS_SCALE_INNER",
                    "boundsAlignment": 0,
                    "boundsWidth": 3840.0,
                    "boundsHeight": 2160.0,
                    "cropLeft": 0,
                    "cropRight": 0,
                    "cropTop": 0,
                    "cropBottom": 0,
                }
                if ptz_iid is not None:
                    obs_rpc(
                        "SetSceneItemTransform",
                        {"sceneName": OBS_PTZ_YT_SCENE, "sceneItemId": ptz_iid, "sceneItemTransform": full_4k_xf},
                        check=False,
                    )
                    obs_rpc(
                        "SetSceneItemIndex",
                        {"sceneName": OBS_PTZ_YT_SCENE, "sceneItemId": ptz_iid, "sceneItemIndex": 0},
                        check=False,
                    )
                if syn_iid is not None:
                    inset_xf = {
                        "positionX": OBS_INSET_X,
                        "positionY": OBS_INSET_Y,
                        "alignment": 5,
                        "boundsType": "OBS_BOUNDS_SCALE_INNER",
                        "boundsAlignment": 0,
                        "boundsWidth": OBS_INSET_W,
                        "boundsHeight": OBS_INSET_H,
                        "cropLeft": 0,
                        "cropRight": 0,
                        "cropTop": 0,
                        "cropBottom": 0,
                    }
                    obs_rpc(
                        "SetSceneItemTransform",
                        {"sceneName": OBS_PTZ_YT_SCENE, "sceneItemId": syn_iid, "sceneItemTransform": inset_xf},
                        check=False,
                    )
                    obs_rpc(
                        "SetSceneItemIndex",
                        {"sceneName": OBS_PTZ_YT_SCENE, "sceneItemId": syn_iid, "sceneItemIndex": 1},
                        check=False,
                    )
                    obs_rpc(
                        "SetSceneItemEnabled",
                        {"sceneName": OBS_PTZ_YT_SCENE, "sceneItemId": syn_iid, "sceneItemEnabled": want_vis},
                        check=False,
                    )
                    with self.lock:
                        self._applied_ptz_inset_visible = want_vis
                if hud_iid is not None:
                    obs_rpc(
                        "SetSceneItemTransform",
                        {
                            "sceneName": OBS_PTZ_YT_SCENE,
                            "sceneItemId": hud_iid,
                            "sceneItemTransform": full_4k_xf,
                        },
                        check=False,
                    )
                    obs_rpc(
                        "SetSceneItemIndex",
                        {
                            "sceneName": OBS_PTZ_YT_SCENE,
                            "sceneItemId": hud_iid,
                            "sceneItemIndex": 2 if syn_iid is not None else 1,
                        },
                        check=False,
                    )

                # CRITICAL FOR HEADLESS 24/7 MAC MINI OPERATION:
                # Anchor `PTZ YouTube Broadcast` at index 0 inside the live Program scene (`Scene`),
                # completely underneath the opaque 3840x2160 `Video Capture Device 2` layer.
                # This keeps `videoActive=True` on both `PTZ YouTube Broadcast` (`source_record_filter`)
                # and `Cam Link 4K` inside `obs_graphics_thread` at 30.00 fps even when macOS sleeps the
                # display or occludes the Qt preview window, while changing 0 pixels on the main stream!
                prog_scene = (obs_rpc("GetCurrentProgramScene", check=False).get("currentProgramSceneName")) or "Scene"
                if prog_scene != OBS_PTZ_YT_SCENE:
                    p_items = obs_rpc("GetSceneItemList", {"sceneName": prog_scene}, check=False).get("sceneItems") or []
                    anchor_it = next((it for it in p_items if it.get("sourceName") == OBS_PTZ_YT_SCENE), None)
                    if anchor_it is None:
                        cr_a = obs_rpc(
                            "CreateSceneItem",
                            {"sceneName": prog_scene, "sourceName": OBS_PTZ_YT_SCENE, "sceneItemEnabled": True},
                            check=False,
                        )
                        anchor_iid = cr_a.get("sceneItemId")
                    else:
                        anchor_iid = anchor_it.get("sceneItemId")
                        obs_rpc(
                            "SetSceneItemEnabled",
                            {"sceneName": prog_scene, "sceneItemId": anchor_iid, "sceneItemEnabled": True},
                            check=False,
                        )
                    if anchor_iid is not None:
                        obs_rpc(
                            "SetSceneItemTransform",
                            {
                                "sceneName": prog_scene,
                                "sceneItemId": anchor_iid,
                                "sceneItemTransform": {
                                    "positionX": 16.0,
                                    "positionY": 16.0,
                                    "alignment": 5,
                                    "boundsType": "OBS_BOUNDS_SCALE_INNER",
                                    "boundsAlignment": 0,
                                    "boundsWidth": 32.0,
                                    "boundsHeight": 18.0,
                                },
                            },
                            check=False,
                        )
                        obs_rpc(
                            "SetSceneItemIndex",
                            {"sceneName": prog_scene, "sceneItemId": anchor_iid, "sceneItemIndex": 0},
                            check=False,
                        )

                with self.lock:
                    self.ptz_yt_scene_ready = True
                return syn_iid

            def sync_ptz_youtube_filter():
                """Configures `source_record_filter` on `PTZ YouTube Broadcast` for independent 4K RTMPS streaming."""
                nonlocal applied_yt_seq
                with self.lock:
                    want_en = self.ptz_youtube_enabled
                    yt_key = self.ptz_youtube_key
                    cur_seq = self._ptz_yt_config_seq
                if cur_seq == applied_yt_seq:
                    return

                flist = obs_rpc("GetSourceFilterList", {"sourceName": OBS_PTZ_YT_SCENE}, check=False).get("filters") or []
                has_filter = any((f.get("filterName") == OBS_PTZ_YT_FILTER) for f in flist)
                should_stream = bool(want_en and yt_key)
                f_settings = {
                    "stream_mode": 1 if should_stream else 0,
                    "server": OBS_YOUTUBE_RTMPS_SERVER,
                    "key": yt_key,
                    "encoder": "com.apple.videotoolbox.videoencoder.ave.avc",
                    "rate_control": "CBR",
                    "bitrate": 14000,
                    "keyint_sec": 2,
                    "profile": "high",
                    "audio_encoder": "CoreAudio_AAC",
                    "audio_bitrate": 128,
                    "different_audio": False,
                    "scale": False,
                }
                if not has_filter:
                    f_settings["record_mode"] = 0
                    if not should_stream:
                        applied_yt_seq = cur_seq
                        with self.lock:
                            self.ptz_yt_streaming = False
                            self.ptz_yt_status_text = (
                                "YouTube PTZ Broadcast: Paste Stream Key to go live"
                                if want_en and not yt_key
                                else (
                                    "YouTube PTZ Broadcast: Scene ready (stream paused)"
                                    if yt_key
                                    else "YouTube PTZ Broadcast: Scene ready — paste Stream Key"
                                )
                            )
                        return
                    cr = obs_rpc(
                        "CreateSourceFilter",
                        {
                            "sourceName": OBS_PTZ_YT_SCENE,
                            "filterName": OBS_PTZ_YT_FILTER,
                            "filterKind": "source_record_filter",
                            "filterSettings": f_settings,
                        },
                        check=False,
                    )
                    if not cr and not any(
                        (f.get("filterName") == OBS_PTZ_YT_FILTER)
                        for f in (obs_rpc("GetSourceFilterList", {"sourceName": OBS_PTZ_YT_SCENE}, check=False).get("filters") or [])
                    ):
                        with self.lock:
                            self.ptz_yt_streaming = False
                            self.ptz_yt_status_text = "OBS source-record plugin missing"
                        applied_yt_seq = cur_seq
                        return
                    obs_rpc(
                        "SetSourceFilterEnabled",
                        {
                            "sourceName": OBS_PTZ_YT_SCENE,
                            "filterName": OBS_PTZ_YT_FILTER,
                            "filterEnabled": True,
                        },
                        check=False,
                    )
                else:
                    if not should_stream:
                        obs_rpc(
                            "SetSourceFilterSettings",
                            {
                                "sourceName": OBS_PTZ_YT_SCENE,
                                "filterName": OBS_PTZ_YT_FILTER,
                                "filterSettings": {"stream_mode": 0, "record_mode": 0},
                                "overlay": True,
                            },
                            check=False,
                        )
                        obs_rpc(
                            "SetSourceFilterEnabled",
                            {
                                "sourceName": OBS_PTZ_YT_SCENE,
                                "filterName": OBS_PTZ_YT_FILTER,
                                "filterEnabled": False,
                            },
                            check=False,
                        )
                    else:
                        obs_rpc(
                            "SetSourceFilterSettings",
                            {
                                "sourceName": OBS_PTZ_YT_SCENE,
                                "filterName": OBS_PTZ_YT_FILTER,
                                "filterSettings": f_settings,
                                "overlay": True,
                            },
                            check=False,
                        )
                        obs_rpc(
                            "SetSourceFilterEnabled",
                            {
                                "sourceName": OBS_PTZ_YT_SCENE,
                                "filterName": OBS_PTZ_YT_FILTER,
                                "filterEnabled": True,
                            },
                            check=False,
                        )

                applied_yt_seq = cur_seq
                with self.lock:
                    self.ptz_yt_streaming = should_stream
                    if should_stream:
                        self.ptz_yt_status_text = "YouTube PTZ Broadcast: LIVE 4K30 (Chyron + Overview Inset)"
                    elif want_en and not yt_key:
                        self.ptz_yt_status_text = "YouTube PTZ Broadcast: Paste Stream Key to go live"
                    elif yt_key:
                        self.ptz_yt_status_text = "YouTube PTZ Broadcast: Scene ready (stream paused)"
                    else:
                        self.ptz_yt_status_text = "YouTube PTZ Broadcast: Scene ready — paste Stream Key"

            def ensure_ptz_preview_active(target_ptz_input: str):
                """
                Verifies `target_ptz_input` is actively ticking (`videoActive` or `videoShowing`).
                When anchored inside Program `Scene` at index 0, `videoActive` is True 24/7 without Studio Mode.
                """
                if not target_ptz_input:
                    return
                act = obs_rpc("GetSourceActive", {"sourceName": target_ptz_input}, check=False)
                if act.get("videoActive", False) or act.get("videoShowing", False):
                    return
                slist = obs_rpc("GetSceneList", check=False)
                prog_name = slist.get("currentProgramSceneName") or scene_name
                ptz_scene = ""
                for sc in slist.get("scenes") or []:
                    sn = sc.get("sceneName") or ""
                    if not sn or sn == prog_name:
                        continue
                    items = obs_rpc("GetSceneItemList", {"sceneName": sn}, check=False).get("sceneItems") or []
                    if any((it.get("sourceName") == target_ptz_input) for it in items):
                        ptz_scene = sn
                        break
                if ptz_scene:
                    obs_rpc("SetStudioModeEnabled", {"studioModeEnabled": True}, check=False)
                    obs_rpc("SetCurrentPreviewScene", {"sceneName": ptz_scene}, check=False)

            if ptz_input_name:
                ptz_yt_inset_item_id = ensure_ptz_youtube_broadcast_scene(cam_input_name, ptz_input_name)
                ensure_ptz_preview_active(ptz_input_name)
                sync_ptz_youtube_filter()
                # Immediate startup liveness check: if Cam Link 4K booted into a stalled/black state
                # after an OBS restart, recover it immediately instead of waiting 2 minutes.
                time.sleep(0.35)
                s1 = (
                    obs_rpc(
                        "GetSourceScreenshot",
                        {"sourceName": ptz_input_name, "imageFormat": "png", "imageWidth": 320, "imageHeight": 180},
                        check=False,
                    ).get("imageData")
                    or ""
                ).split(",", 1)[-1]
                time.sleep(0.25)
                s2 = (
                    obs_rpc(
                        "GetSourceScreenshot",
                        {"sourceName": ptz_input_name, "imageFormat": "png", "imageWidth": 320, "imageHeight": 180},
                        check=False,
                    ).get("imageData")
                    or ""
                ).split(",", 1)[-1]
                if s1 and s2 and s1 == s2:
                    cycle_camera_input(ptz_input_name, "PTZ capture frozen on OBS connect")

            now = time.time()
            with self.lock:
                if self._version == ver:
                    self.connected = True
                    self._last_alive_ts = now
                    self.camera_input_name = cam_input_name
                    self.ptz_input_name = ptz_input_name
                    self.status_text = f"OBS ({scene_name} @ {host})"

            frame_interval = 1.0 / OBS_SCREENSHOT_FPS
            last_keepalive = now
            last_scene_refresh = now
            last_watchdog_check = 0.0
            prev_watchdog_hash = ""
            frozen_streak = 0
            last_ptz_watchdog_check = 0.0
            prev_ptz_watchdog_hash = ""
            ptz_frozen_streak = 0
            last_ptz_reset_ts = 0.0
            reset_cooldown = OBS_WATCHDOG_MIN_COOLDOWN_SEC

            while True:
                now = time.time()
                with self.lock:
                    if self._version != ver:
                        return
                    need_syn = (now - self.last_demand_ts) <= OBS_IDLE_TIMEOUT_SEC
                    need_ptz = (now - self.ptz_last_demand_ts) <= OBS_IDLE_TIMEOUT_SEC
                    is_idle = not (need_syn or need_ptz)

                ws.drain_incoming()

                # Sync dynamic Top-Right Overview Inset visibility (auto-hidden at 1x synoptic zoom)
                if ptz_yt_inset_item_id is not None:
                    with self.lock:
                        want_vis = self._want_ptz_inset_visible
                        app_vis = self._applied_ptz_inset_visible
                    if want_vis != app_vis:
                        obs_rpc(
                            "SetSceneItemEnabled",
                            {
                                "sceneName": OBS_PTZ_YT_SCENE,
                                "sceneItemId": ptz_yt_inset_item_id,
                                "sceneItemEnabled": want_vis,
                            },
                            check=False,
                        )
                        with self.lock:
                            self._applied_ptz_inset_visible = want_vis

                # Sync PTZ YouTube RTMPS stream filter settings if changed via Mission Control
                if ptz_input_name:
                    sync_ptz_youtube_filter()

                # Handle manual reset request (`POST /api/reset_camera`)
                if self._manual_reset_event.is_set():
                    self._manual_reset_event.clear()
                    cam_input_name, ptz_input_name = discover_cameras()
                    if cam_input_name:
                        cycle_camera_input(cam_input_name, "manual reset requested")
                    if ptz_input_name:
                        ensure_ptz_preview_active(ptz_input_name)
                        cycle_camera_input(ptz_input_name, "manual PTZ reset requested")
                    prev_watchdog_hash = ""
                    frozen_streak = 0
                    prev_ptz_watchdog_hash = ""
                    ptz_frozen_streak = 0
                    last_watchdog_check = time.time()
                    last_ptz_watchdog_check = last_watchdog_check
                    with self.lock:
                        self.camera_input_name = cam_input_name
                        self.ptz_input_name = ptz_input_name
                        self._last_alive_ts = last_watchdog_check

                # Always-on lightweight PTZ capture watchdog (every 15s, even if a viewer tab is open,
                # using 640x360 PNG so 24x box-downsampling on clear sky never quantizes away shot noise)
                now = time.time()
                if ptz_input_name and (now - last_ptz_watchdog_check) >= 15.0:
                    last_ptz_watchdog_check = now
                    last_keepalive = now
                    ensure_ptz_preview_active(ptz_input_name)
                    pw_shot = obs_rpc(
                        "GetSourceScreenshot",
                        {
                            "sourceName": ptz_input_name,
                            "imageFormat": "png",
                            "imageWidth": 640,
                            "imageHeight": 360,
                        },
                        check=False,
                    )
                    pw_b64 = pw_shot.get("imageData", "")
                    if "," in pw_b64:
                        pw_b64 = pw_b64.split(",", 1)[1]
                    if pw_b64:
                        cur_phash = hashlib.sha256(base64.b64decode(pw_b64)).hexdigest()
                        if prev_ptz_watchdog_hash and cur_phash == prev_ptz_watchdog_hash:
                            ptz_frozen_streak += 1
                            if ptz_frozen_streak >= 2 and (now - last_ptz_reset_ts) >= 90.0:
                                if cycle_camera_input(
                                    ptz_input_name,
                                    f"PTZ capture frozen ({ptz_frozen_streak} identical 15s checks)",
                                ):
                                    last_ptz_reset_ts = time.time()
                                    ptz_frozen_streak = 0
                                    prev_ptz_watchdog_hash = ""
                        else:
                            ptz_frozen_streak = 0
                            prev_ptz_watchdog_hash = cur_phash
                    with self.lock:
                        self._last_alive_ts = time.time()
                        self.connected = True

                # Optional standalone watchdog (off by default when `obs_freeze_monitor.py` is running)
                now = time.time()
                if OBS_WATCHDOG_ENABLED and (now - last_watchdog_check) >= OBS_WATCHDOG_INTERVAL_SEC:
                    last_watchdog_check = now
                    last_keepalive = now
                    if not cam_input_name:
                        cam_input_name = discover_camera_input()
                        with self.lock:
                            self.camera_input_name = cam_input_name

                    if cam_input_name:
                        act = obs_rpc("GetSourceActive", {"sourceName": cam_input_name}, check=False)
                        if act.get("videoShowing", False):
                            w_shot = obs_rpc(
                                "GetSourceScreenshot",
                                {
                                    "sourceName": cam_input_name,
                                    "imageFormat": "png",
                                    "imageWidth": 160,
                                    "imageHeight": 90,
                                },
                                check=False,
                            )
                            w_b64 = w_shot.get("imageData", "")
                            if "," in w_b64:
                                w_b64 = w_b64.split(",", 1)[1]
                            if w_b64:
                                raw_png = base64.b64decode(w_b64)
                                cur_hash = hashlib.sha256(raw_png).hexdigest()
                                if prev_watchdog_hash and cur_hash == prev_watchdog_hash:
                                    frozen_streak += 1
                                    if (
                                        frozen_streak >= OBS_WATCHDOG_FROZEN_COUNT
                                        and (now - self.last_watchdog_reset_ts) >= reset_cooldown
                                    ):
                                        elapsed_s = int(frozen_streak * OBS_WATCHDOG_INTERVAL_SEC)
                                        if cycle_camera_input(
                                            cam_input_name,
                                            f"frozen frame detected ({frozen_streak} identical checks / ~{elapsed_s}s)",
                                        ):
                                            frozen_streak = 0
                                            prev_watchdog_hash = ""
                                            reset_cooldown = min(
                                                OBS_WATCHDOG_MAX_COOLDOWN_SEC, reset_cooldown * 2.0
                                            )
                                else:
                                    if prev_watchdog_hash and cur_hash != prev_watchdog_hash:
                                        reset_cooldown = OBS_WATCHDOG_MIN_COOLDOWN_SEC
                                    frozen_streak = 0
                                    prev_watchdog_hash = cur_hash
                        else:
                            frozen_streak = 0
                            prev_watchdog_hash = ""

                    with self.lock:
                        self._last_alive_ts = time.time()
                        self.connected = True

                # Standby path: 0 screenshots/sec, lightweight GetVersion heartbeat every 20s
                if is_idle:
                    if (now - last_keepalive) >= 20.0:
                        obs_rpc("GetVersion")
                        last_keepalive = time.time()
                        with self.lock:
                            self._last_alive_ts = last_keepalive
                            self.connected = True
                    self._wake_event.wait(timeout=1.0)
                    self._wake_event.clear()
                    continue

                # Active viewer path: refresh program scene & PTZ input names periodically
                if (now - last_scene_refresh) > 15.0:
                    s_info = obs_rpc("GetCurrentProgramScene", check=False)
                    if s_info.get("currentProgramSceneName"):
                        scene_name = s_info["currentProgramSceneName"]
                    cam_input_name, ptz_input_name = discover_cameras()
                    with self.lock:
                        self.camera_input_name = cam_input_name
                        self.ptz_input_name = ptz_input_name
                    last_scene_refresh = now

                t_req = time.time()
                last_keepalive = t_req

                if need_syn:
                    shot_data = obs_rpc(
                        "GetSourceScreenshot",
                        {
                            "sourceName": scene_name,
                            "imageFormat": "jpg",
                            "imageWidth": OBS_SCREENSHOT_WIDTH,
                            "imageHeight": OBS_SCREENSHOT_HEIGHT,
                            "imageCompressionQuality": OBS_SCREENSHOT_QUALITY,
                        },
                        check=False,
                    )
                    img_b64 = shot_data.get("imageData", "")
                    if "," in img_b64:
                        img_b64 = img_b64.split(",", 1)[1]
                    if img_b64:
                        jpeg = base64.b64decode(img_b64)
                        ts_done = time.time()
                        with self._frame_cond:
                            self.latest_jpeg = jpeg
                            self.latest_ts = ts_done
                            self._last_alive_ts = ts_done
                            self.connected = True
                            self._frame_cond.notify_all()

                # Re-check `need_ptz` right before PTZ screenshot so simultaneous `/api/cam_frame.jpg` +
                # `/api/ptz_frame.jpg` wakeups serve BOTH frames on the very first loop iteration.
                with self.lock:
                    need_ptz = need_ptz or ((time.time() - self.ptz_last_demand_ts) <= OBS_IDLE_TIMEOUT_SEC)

                if need_ptz and ptz_input_name:
                    ptz_shot = obs_rpc(
                        "GetSourceScreenshot",
                        {
                            "sourceName": ptz_input_name,
                            "imageFormat": "jpg",
                            "imageWidth": OBS_SCREENSHOT_WIDTH,
                            "imageHeight": OBS_SCREENSHOT_HEIGHT,
                            "imageCompressionQuality": OBS_SCREENSHOT_QUALITY,
                        },
                        check=False,
                    )
                    ptz_b64 = ptz_shot.get("imageData", "")
                    if "," in ptz_b64:
                        ptz_b64 = ptz_b64.split(",", 1)[1]
                    if ptz_b64:
                        p_jpeg = base64.b64decode(ptz_b64)
                        ts_done = time.time()
                        with self._frame_cond:
                            self.ptz_latest_jpeg = p_jpeg
                            self.ptz_latest_ts = ts_done
                            self._last_alive_ts = ts_done
                            self.connected = True
                            self._frame_cond.notify_all()

                sleep_for = max(0.02, frame_interval - (time.time() - t_req))
                time.sleep(sleep_for)
        finally:
            ws.close()


class OBSCalibrationSession:
    """
    Context-managed OBS WebSocket v5 client for uncached high-resolution calibration frames.
    Reads raw camera inputs directly (`Video Capture Device 2` & `Video Capture Device 3 (FoMaKo)`)
    so AR HUD graphics on the broadcast scene never contaminate vision template matching.
    """

    def __init__(
        self,
        bridge: OBSCameraBridge,
        url: str,
        syn_hint: str = "",
        ptz_hint: str = "",
        timeout: float = 8.0,
    ):
        self.bridge = bridge
        self.url = url
        self.syn_source = syn_hint
        self.ptz_source = ptz_hint
        self.timeout = timeout
        self.ws: Optional[PlainWSClient] = None
        self._seq = 0

    def __enter__(self) -> "OBSCalibrationSession":
        parsed = urlparse(self.url if "://" in self.url else f"obs://{self.url}")
        host = parsed.hostname or "127.0.0.1"
        port = parsed.port or 4455
        password = parsed.password or ""

        self.ws = PlainWSClient(host, port, timeout=self.timeout)
        hello = self.ws.recv_json()
        d = hello.get("d") or {}
        auth_info = d.get("authentication")
        ident = {"rpcVersion": 1, "eventSubscriptions": 0}
        if auth_info and password:
            secret = base64.b64encode(
                hashlib.sha256((password + auth_info["salt"]).encode("utf-8")).digest()
            ).decode("ascii")
            auth_resp = base64.b64encode(
                hashlib.sha256((secret + auth_info["challenge"]).encode("utf-8")).digest()
            ).decode("ascii")
            ident["authentication"] = auth_resp
        self.ws.send_json({"op": 1, "d": ident})
        identified = self.ws.recv_json()
        if identified.get("op") != 2:
            self.close()
            raise PermissionError("OBS WebSocket authentication failed during calibration")

        if not self.syn_source or not self.ptz_source:
            self._discover_inputs()
        if not self.syn_source:
            self.close()
            raise RuntimeError("Could not find Synoptic camera input in OBS")
        if not self.ptz_source:
            self.close()
            raise RuntimeError("Could not find PTZ camera input in OBS")
        self._ensure_ptz_source_showing()
        self._ensure_ptz_live_frames()
        return self

    def _grab_ptz_probe_hash(self) -> str:
        resp = self.rpc(
            "GetSourceScreenshot",
            {
                "sourceName": self.ptz_source,
                "imageFormat": "png",
                "imageWidth": 640,
                "imageHeight": 360,
            },
            check=False,
        )
        b64 = (resp.get("imageData") or "").split(",", 1)[-1]
        return hashlib.sha256(base64.b64decode(b64)).hexdigest() if b64 else ""

    def _ensure_ptz_live_frames(self):
        """
        Verifies the PTZ capture device (e.g. Cam Link 4K) is delivering fresh frames before
        starting calibration; restores with full `CAMLINK_4K_NV12_SETTINGS` + USB reset if frozen.
        """
        h1 = self._grab_ptz_probe_hash()
        for _ in range(3):
            time.sleep(0.25)
            hn = self._grab_ptz_probe_hash()
            if h1 and hn and hn != h1:
                return
        cur = (self.rpc("GetInputSettings", {"inputName": self.ptz_source}, check=False).get("inputSettings") or {})
        orig_dev = cur.get("device") or ""
        is_camlink = "0fd9" in orig_dev.lower() or "cam link" in str(cur.get("device_name") or "").lower()
        if orig_dev:
            print(f"[OBS Calibration] PTZ input '{self.ptz_source}' appears frozen; cycling device {orig_dev}…", flush=True)
            try:
                self.rpc("SetInputSettings", {"inputName": self.ptz_source, "inputSettings": {"device": ""}, "overlay": True})
                if is_camlink:
                    uvc_bin = Path(__file__).resolve().parent / "uvc-util"
                    if uvc_bin.is_file():
                        subprocess.run(
                            [str(uvc_bin), "-V", "0x0fd9:0x0066", "--usb-reset"],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                            check=False,
                        )
                    time.sleep(2.2)
                else:
                    time.sleep(1.4)
            finally:
                restore_cfg = {"device": orig_dev, **CAMLINK_4K_NV12_SETTINGS} if is_camlink else {"device": orig_dev}
                self.rpc(
                    "SetInputSettings",
                    {"inputName": self.ptz_source, "inputSettings": restore_cfg, "overlay": not is_camlink},
                )
            time.sleep(2.0)
            h3 = self._grab_ptz_probe_hash()
            time.sleep(0.30)
            h4 = self._grab_ptz_probe_hash()
            if h3 and h3 != h4:
                print(f"[OBS Calibration] PTZ input '{self.ptz_source}' recovered live frames!", flush=True)
                return
        self.close()
        raise RuntimeError(f"PTZ video input '{self.ptz_source}' is frozen or HDMI signal is disconnected")

    def _ensure_ptz_source_showing(self):
        """
        Ensures the FoMaKo PTZ scene is active in OBS Studio Mode Preview so `GetSourceScreenshot`
        reads live VSync frames instead of a frozen inactive-scene texture, without ever changing
        the live YouTube Program scene (`Scene`).
        """
        act = self.rpc("GetSourceActive", {"sourceName": self.ptz_source}, check=False)
        if act.get("videoShowing", False):
            return
        slist = self.rpc("GetSceneList", check=False)
        prog_name = slist.get("currentProgramSceneName") or "Scene"
        for sc in slist.get("scenes") or []:
            sn = sc.get("sceneName") or ""
            if not sn or sn == prog_name:
                continue
            items = self.rpc("GetSceneItemList", {"sceneName": sn}, check=False).get("sceneItems") or []
            if any((it.get("sourceName") == self.ptz_source) for it in items):
                self.rpc("SetStudioModeEnabled", {"studioModeEnabled": True}, check=False)
                self.rpc("SetCurrentPreviewScene", {"sceneName": sn}, check=False)
                time.sleep(0.25)
                break

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def close(self):
        if self.ws is not None:
            self.ws.close()
            self.ws = None

    def rpc(self, req_type: str, req_data: Optional[dict] = None, check: bool = True) -> dict:
        if self.ws is None:
            raise ConnectionError("OBSCalibrationSession is not open")
        self._seq += 1
        req_id = f"cal_{self._seq}"
        payload = {"requestType": req_type, "requestId": req_id}
        if req_data is not None:
            payload["requestData"] = req_data
        self.ws.send_json({"op": 6, "d": payload})
        while True:
            msg = self.ws.recv_json()
            if msg.get("op") == 7:
                md = msg.get("d") or {}
                if md.get("requestId") == req_id:
                    st = md.get("requestStatus") or {}
                    if check and not st.get("result", False):
                        raise OBSRPCError(
                            f"{req_type} failed (code={st.get('code')}): {st.get('comment', '')}"
                        )
                    return md.get("responseData") or {}

    def _discover_inputs(self):
        pref = getattr(self.bridge, "preferred_ptz", "sony_a40")
        resp = self.rpc("GetInputList", check=False)
        av_names = []
        for inp in resp.get("inputs") or []:
            kind = inp.get("unversionedInputKind") or inp.get("inputKind") or ""
            if kind.startswith("macos-avcapture") or kind.startswith("av_capture_input"):
                nm = inp.get("inputName") or ""
                if nm:
                    av_names.append(nm)
        fomako_in = ""
        sony_in = ""
        for nm in av_names:
            cur = self.rpc("GetInputSettings", {"inputName": nm}, check=False).get("inputSettings") or {}
            dev = str(cur.get("device") or "").lower()
            dname = str(cur.get("device_name") or "").lower()
            low_nm = nm.lower()
            if "0fd9" in dev or "cam link" in dname or "sony" in low_nm or "cam link" in low_nm:
                if not sony_in:
                    sony_in = nm
            elif "33f1" in dev or "uhd camera" in dname or "fomako" in low_nm:
                if not fomako_in:
                    fomako_in = nm
            elif "2e1a" in dev or "insta360" in dname:
                if not self.syn_source:
                    self.syn_source = nm
        if not self.ptz_source:
            self.ptz_source = (sony_in or fomako_in) if pref == "sony_a40" else (fomako_in or sony_in)
        if not self.syn_source:
            for nm in av_names:
                if nm not in (sony_in, fomako_in):
                    self.syn_source = nm
                    break

    def save_synoptic_jpeg(self, dst_path: str, width: int = 3840, height: int = 2160, quality: int = 93):
        resp = self.rpc(
            "GetSourceScreenshot",
            {
                "sourceName": self.syn_source,
                "imageFormat": "jpg",
                "imageWidth": width,
                "imageHeight": height,
                "imageCompressionQuality": quality,
            },
        )
        img_b64 = (resp.get("imageData") or "").split(",", 1)[-1]
        raw = base64.b64decode(img_b64)
        with open(dst_path, "wb") as f:
            f.write(raw)

    def save_ptz_jpeg(
        self,
        dst_path: str,
        width: int = 1920,
        height: int = 1080,
        quality: int = 93,
        publish_preview: bool = True,
    ):
        resp = self.rpc(
            "GetSourceScreenshot",
            {
                "sourceName": self.ptz_source,
                "imageFormat": "jpg",
                "imageWidth": width,
                "imageHeight": height,
                "imageCompressionQuality": quality,
            },
        )
        img_b64 = (resp.get("imageData") or "").split(",", 1)[-1]
        raw = base64.b64decode(img_b64)
        with open(dst_path, "wb") as f:
            f.write(raw)
        if publish_preview:
            self.bridge.publish_ptz_preview(raw)
