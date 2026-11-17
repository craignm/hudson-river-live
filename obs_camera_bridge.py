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
import select
import socket
import struct
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

    def __init__(self, initial_url: str = ""):
        self.lock = threading.Lock()
        self._frame_cond = threading.Condition(self.lock)
        self._wake_event = threading.Event()
        self._manual_reset_event = threading.Event()

        self.url = ""
        self.latest_jpeg: bytes = b""
        self.latest_ts: float = 0.0
        self.last_demand_ts: float = 0.0
        self._last_alive_ts: float = 0.0
        self.connected: bool = False
        self.status_text: str = "Not configured"
        self.camera_input_name: str = ""
        self.watchdog_resets: int = 0
        self.last_watchdog_reset_ts: float = 0.0
        self._pending_device_restore: Optional[Tuple[str, str]] = None
        self._version = 0

        if initial_url:
            self.set_url(initial_url)
        threading.Thread(target=self._worker_loop, daemon=True).start()

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

    def request_camera_reset(self) -> dict:
        """Schedules an immediate soft cycle of OBS's `macos-avcapture` camera input."""
        with self.lock:
            if not self.connected:
                return {"ok": False, "error": "OBS WebSocket is not connected"}
            cam_name = self.camera_input_name or "macos-avcapture"
        self._manual_reset_event.set()
        self._wake_event.set()
        return {"ok": True, "scheduled": True, "camera_input": cam_name}

    def get_status(self) -> dict:
        """Returns connection & standby telemetry for `/api/state`."""
        with self.lock:
            now = time.time()
            idle = (now - self.last_demand_ts) > OBS_IDLE_TIMEOUT_SEC
            alive = self.connected and ((now - self._last_alive_ts) < 45.0)
            suffix_parts = []
            if alive:
                suffix_parts.append("standby" if idle else f"live ~{OBS_SCREENSHOT_FPS:.0f}fps")
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

            def discover_camera_input() -> str:
                resp = obs_rpc("GetInputList", check=False)
                for inp in resp.get("inputs") or []:
                    kind = inp.get("unversionedInputKind") or inp.get("inputKind") or ""
                    if kind.startswith("macos-avcapture") or kind.startswith("av_capture_input"):
                        return inp.get("inputName") or ""
                return ""

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
                print(f"[OBS Bridge {ts_str}] Cycling '{cam_name}' ({orig_dev}) — {reason}", flush=True)
                self._pending_device_restore = (cam_name, orig_dev)
                try:
                    obs_rpc(
                        "SetInputSettings",
                        {"inputName": cam_name, "inputSettings": {"device": ""}, "overlay": True},
                    )
                    time.sleep(1.2)
                finally:
                    obs_rpc(
                        "SetInputSettings",
                        {"inputName": cam_name, "inputSettings": {"device": orig_dev}, "overlay": True},
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
            cam_input_name = discover_camera_input()

            now = time.time()
            with self.lock:
                if self._version == ver:
                    self.connected = True
                    self._last_alive_ts = now
                    self.camera_input_name = cam_input_name
                    self.status_text = f"OBS ({scene_name} @ {host})"

            frame_interval = 1.0 / OBS_SCREENSHOT_FPS
            last_keepalive = now
            last_scene_refresh = now
            last_watchdog_check = 0.0
            prev_watchdog_hash = ""
            frozen_streak = 0
            reset_cooldown = OBS_WATCHDOG_MIN_COOLDOWN_SEC

            while True:
                now = time.time()
                with self.lock:
                    if self._version != ver:
                        return
                    is_idle = (now - self.last_demand_ts) > OBS_IDLE_TIMEOUT_SEC

                ws.drain_incoming()

                # Handle manual reset request (`POST /api/reset_camera`)
                if self._manual_reset_event.is_set():
                    self._manual_reset_event.clear()
                    if not cam_input_name:
                        cam_input_name = discover_camera_input()
                    if cam_input_name:
                        cycle_camera_input(cam_input_name, "manual reset requested")
                        prev_watchdog_hash = ""
                        frozen_streak = 0
                        last_watchdog_check = time.time()
                        with self.lock:
                            self._last_alive_ts = last_watchdog_check

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

                # Active viewer path: refresh program scene name periodically and pull 720p JPEG
                if (now - last_scene_refresh) > 15.0:
                    s_info = obs_rpc("GetCurrentProgramScene", check=False)
                    if s_info.get("currentProgramSceneName"):
                        scene_name = s_info["currentProgramSceneName"]
                    last_scene_refresh = now

                t_req = time.time()
                last_keepalive = t_req
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

                sleep_for = max(0.02, frame_interval - (time.time() - t_req))
                time.sleep(sleep_for)
        finally:
            ws.close()
