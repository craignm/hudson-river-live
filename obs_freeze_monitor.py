#!/usr/bin/env python3
"""
24/7 OBS Webcam Freeze & Stream Health Monitor (pure Python 3.9+ stdlib, zero pip installs).

Designed to run as a macOS LaunchAgent on the streaming M6 Mac (127.0.0.1:4455):
  - Periodically captures a tiny 160x90 lossless PNG directly from OBS's `macos-avcapture`
    input (e.g. 'Video Capture Device 2', bypassing scene clock/browser overlays).
  - Decodes raw 8-bit RGB pixels in pure Python and computes frame-to-frame Mean Absolute
    Difference (MAD), % changed channels, exact SHA-256 match, and blackout detection.
  - If the camera freezes (MAD < 0.05 across 3 checks / ~60s), immediately sends a push
    notification (with inline JPEG snapshot + OBS stats) via ntfy.sh, auto-cycles OBS's
    AVCaptureSession (`device: ""` -> 1.5s -> restore), verifies live pixel motion, and
    sends a follow-up recovery push notification with the recovered snapshot.
  - Also alerts if OBS crashes/closes, if the YouTube stream stops unexpectedly, or if
    headless rendering lag (>10%) ever returns.
"""

import argparse
import base64
import hashlib
import json
import os
import socket
import struct
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import zlib
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from urllib.parse import urlparse


def _load_dotenv_if_present() -> None:
    """Loads optional `.env` key=value pairs from the script directory if present."""
    env_path = Path(__file__).resolve().parent / ".env"
    if not env_path.is_file():
        return
    try:
        for line in env_path.read_text(encoding="utf-8").splitlines():
            s = line.strip()
            if not s or s.startswith("#") or "=" not in s:
                continue
            k, v = s.split("=", 1)
            os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))
    except Exception:
        pass


_load_dotenv_if_present()

DEFAULT_OBS_URL = os.environ.get("OBS_URL", "obs://127.0.0.1:4455")
DEFAULT_NTFY_TOPIC = os.environ.get("NTFY_TOPIC", "")
DEFAULT_NTFY_SERVER = os.environ.get("NTFY_SERVER", "https://ntfy.sh")
DEFAULT_DATA_DIR = Path(
    os.environ.get("OBS_MONITOR_DIR", str(Path.home() / ".local" / "share" / "hudson-obs-monitor"))
)


class PlainWSClient:
    """Minimal RFC 6455 ws:// client for local/LAN OBS WebSocket v5."""

    def __init__(self, host: str, port: int = 4455, path: str = "/", timeout: float = 5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except OSError:
            pass
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
                raise ConnectionError("EOF in WS handshake")
            resp += chunk
        if b"101" not in resp.split(b"\r\n", 1)[0]:
            raise ConnectionError("OBS WebSocket handshake rejected")
        self._leftover = resp.split(b"\r\n\r\n", 1)[1]

    def _recv_exact(self, n: int) -> bytes:
        buf = bytearray(self._leftover[:n])
        self._leftover = self._leftover[n:]
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("WS closed")
            buf.extend(chunk)
        return bytes(buf)

    def send_frame(self, opcode: int, payload: bytes):
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
            if opcode == 0x08:
                raise ConnectionError("OBS WS closed")
            if opcode == 0x09:
                self.send_frame(0x0A, data)
                continue
            if opcode == 0x0A:
                continue
            fragments.extend(data)
            if fin:
                return json.loads(fragments.decode("utf-8", errors="replace"))

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


class OBSSession:
    """Authenticated OBS WebSocket v5 RPC client."""

    def __init__(self, obs_url: str):
        parsed = urlparse(obs_url if "://" in obs_url else f"obs://{obs_url}")
        self.host = parsed.hostname or "127.0.0.1"
        self.port = parsed.port or 4455
        self.password = parsed.password or ""
        self.ws = PlainWSClient(self.host, self.port, timeout=5.0)
        self._seq = 0
        self._authenticate()

    def _authenticate(self):
        hello = self.ws.recv_json()
        d = hello.get("d") or {}
        auth_info = d.get("authentication")
        ident = {"rpcVersion": 1, "eventSubscriptions": 0}
        if auth_info and self.password:
            secret = base64.b64encode(
                hashlib.sha256((self.password + auth_info["salt"]).encode("utf-8")).digest()
            ).decode("ascii")
            auth_resp = base64.b64encode(
                hashlib.sha256((secret + auth_info["challenge"]).encode("utf-8")).digest()
            ).decode("ascii")
            ident["authentication"] = auth_resp
        self.ws.send_json({"op": 1, "d": ident})
        identified = self.ws.recv_json()
        if identified.get("op") != 2:
            raise PermissionError("OBS WebSocket authentication failed")

    def rpc(self, req_type: str, req_data: Optional[dict] = None, check: bool = True) -> dict:
        self._seq += 1
        req_id = f"m_{self._seq}"
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
                        raise RuntimeError(
                            f"{req_type} failed (code={st.get('code')}): {st.get('comment', '')}"
                        )
                    return md.get("responseData") or {}

    def close(self):
        self.ws.close()


def decode_png_rgb_bytes(png: bytes) -> bytearray:
    """
    Pure-stdlib 8-bit non-interlaced RGB/RGBA PNG decoder.
    Returns tightly packed R,G,B bytes (3 bytes/pixel) for accurate pixel diffing.
    """
    if png[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("Invalid PNG signature")
    w, h, depth, ctype, comp, filt, inter = struct.unpack(">IIBBBBB", png[16:29])
    if depth != 8 or inter != 0 or ctype not in (2, 6):
        raise ValueError(f"Unsupported PNG format: depth={depth}, ctype={ctype}, interlace={inter}")

    bpp = 4 if ctype == 6 else 3
    idat = bytearray()
    pos = 8
    while pos + 8 <= len(png):
        ln, chunk_type = struct.unpack(">I4s", png[pos : pos + 8])
        if chunk_type == b"IDAT":
            idat.extend(png[pos + 8 : pos + 8 + ln])
        elif chunk_type == b"IEND":
            break
        pos += 12 + ln

    raw = zlib.decompress(bytes(idat))
    stride = w * bpp
    recon = bytearray(h * stride)

    for y in range(h):
        row_start = y * (stride + 1)
        ft = raw[row_start]
        row_in = raw[row_start + 1 : row_start + 1 + stride]
        dst_off = y * stride
        prev_off = (y - 1) * stride

        if ft == 0:
            recon[dst_off : dst_off + stride] = row_in
        elif ft == 1:
            for i in range(stride):
                a = recon[dst_off + i - bpp] if i >= bpp else 0
                recon[dst_off + i] = (row_in[i] + a) & 0xFF
        elif ft == 2:
            if y == 0:
                recon[dst_off : dst_off + stride] = row_in
            else:
                for i in range(stride):
                    recon[dst_off + i] = (row_in[i] + recon[prev_off + i]) & 0xFF
        elif ft == 3:
            for i in range(stride):
                a = recon[dst_off + i - bpp] if i >= bpp else 0
                b = recon[prev_off + i] if y > 0 else 0
                recon[dst_off + i] = (row_in[i] + ((a + b) >> 1)) & 0xFF
        elif ft == 4:
            for i in range(stride):
                a = recon[dst_off + i - bpp] if i >= bpp else 0
                b = recon[prev_off + i] if y > 0 else 0
                c = recon[prev_off + i - bpp] if (y > 0 and i >= bpp) else 0
                p = a + b - c
                pa = abs(p - a)
                pb = abs(p - b)
                pc = abs(p - c)
                pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                recon[dst_off + i] = (row_in[i] + pr) & 0xFF
        else:
            raise ValueError(f"Unknown PNG filter {ft}")

    if bpp == 3:
        return recon

    rgb = bytearray(w * h * 3)
    out_i = 0
    for src_i in range(0, len(recon), 4):
        rgb[out_i] = recon[src_i]
        rgb[out_i + 1] = recon[src_i + 1]
        rgb[out_i + 2] = recon[src_i + 2]
        out_i += 3
    return rgb


def compute_rgb_diff(rgb_a: bytearray, rgb_b: bytearray) -> Tuple[float, float]:
    """Returns (mean_abs_diff 0..255, pct_changed_channels 0..100)."""
    n = len(rgb_a)
    if n == 0 or n != len(rgb_b):
        return 255.0, 100.0
    total_diff = 0
    nonzero = 0
    for a, b in zip(rgb_a, rgb_b):
        d = a - b if a >= b else b - a
        if d:
            total_diff += d
            nonzero += 1
    return total_diff / n, (100.0 * nonzero) / n


def compute_rgb_levels(rgb: bytearray) -> Tuple[float, int]:
    """Returns (mean_rgb 0..255, max_rgb 0..255) to detect dead all-black camera feeds."""
    if not rgb:
        return 0.0, 0
    return sum(rgb) / len(rgb), max(rgb)


class MonitorLogger:
    def __init__(self, data_dir: Path):
        self.data_dir = data_dir
        self.incidents_dir = data_dir / "incidents"
        self.log_path = data_dir / "monitor.log"
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.incidents_dir.mkdir(parents=True, exist_ok=True)

    def info(self, msg: str):
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        line = f"[{ts}] {msg}"
        print(line, flush=True)
        try:
            if self.log_path.exists() and self.log_path.stat().st_size > 2_000_000:
                bak = self.data_dir / "monitor.log.1"
                self.log_path.replace(bak)
            with self.log_path.open("a", encoding="utf-8") as f:
                f.write(line + "\n")
        except Exception:
            pass

    def save_incident_image(self, label: str, jpeg_bytes: bytes) -> Optional[Path]:
        if not jpeg_bytes:
            return None
        stamp = time.strftime("%Y%m%d-%H%M%S")
        out = self.incidents_dir / f"{stamp}_{label}.jpg"
        try:
            out.write_bytes(jpeg_bytes)
            # Prune old incident images beyond most recent 60 files
            files = sorted(self.incidents_dir.glob("*.jpg"))
            for old in files[:-60]:
                old.unlink(missing_ok=True)
            return out
        except Exception:
            return None


def send_ntfy(
    server: str,
    topic: str,
    title: str,
    message: str,
    priority: int = 4,
    tags: str = "movie_camera",
    jpeg_bytes: Optional[bytes] = None,
    filename: str = "snapshot.jpg",
    logger: Optional[MonitorLogger] = None,
) -> bool:
    if not topic:
        return False
    base = f"{server.rstrip('/')}/{urllib.parse.quote(topic.strip('/'))}"
    try:
        if jpeg_bytes:
            qs = urllib.parse.urlencode(
                {
                    "title": title,
                    "message": message,
                    "priority": str(priority),
                    "tags": tags,
                    "filename": filename,
                }
            )
            req = urllib.request.Request(
                f"{base}?{qs}",
                data=jpeg_bytes,
                headers={"Content-Type": "image/jpeg", "User-Agent": "HudsonOBSMonitor/1.0"},
                method="PUT",
            )
        else:
            qs = urllib.parse.urlencode(
                {
                    "title": title,
                    "priority": str(priority),
                    "tags": tags,
                }
            )
            req = urllib.request.Request(
                f"{base}?{qs}",
                data=message.encode("utf-8"),
                headers={"Content-Type": "text/plain; charset=utf-8", "User-Agent": "HudsonOBSMonitor/1.0"},
                method="POST",
            )
        with urllib.request.urlopen(req, timeout=10.0) as resp:
            resp.read(1024)
        if logger:
            logger.info(f"Sent ntfy alert -> {topic}: {title}")
        return True
    except Exception as exc:
        if logger:
            logger.info(f"WARNING: Failed to send ntfy alert ({title}): {exc}")
        return False


def find_uvc_util_bin(data_dir: Path) -> Optional[Path]:
    candidates = [
        data_dir / "uvc-util",
        Path(__file__).resolve().parent / "uvc-util",
        Path.home() / "Library" / "Application Support" / "HudsonRiverUVC" / "uvc-util",
    ]
    for p in candidates:
        if p.is_file() and os.access(p, os.X_OK):
            return p
    return None


def resync_uvc_exposure() -> None:
    """Re-apply solar day/night EV after camera reset or USB re-enumeration."""
    uvc_dir = Path.home() / "Library" / "Application Support" / "HudsonRiverUVC"
    exp_script = uvc_dir / "auto_exposure_day_night.py"
    uvc_bin = uvc_dir / "uvc-util"
    if exp_script.is_file() and uvc_bin.is_file():
        try:
            subprocess.run(
                [sys.executable, str(exp_script), "--uvc-bin", str(uvc_bin), "--force-sync", "--quiet-unchanged"],
                timeout=8.0,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return
        except Exception:
            pass
    try:
        subprocess.run(
            ["/bin/launchctl", "kickstart", f"gui/{os.getuid()}/live.hudsonriver.uvc-exposure"],
            timeout=3.0,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass


def discover_insta360_device_id(obs: OBSSession, cam_name: str, fallback_dev: str) -> str:
    """Finds the current macos-avcapture device uniqueID for Insta360 after USB re-enumeration."""
    resp = obs.rpc(
        "GetInputPropertiesListPropertyItems",
        {"inputName": cam_name, "propertyName": "device"},
        check=False,
    )
    items = resp.get("propertyItems") or []
    for it in items:
        val = str(it.get("itemValue") or "")
        if val and val == fallback_dev:
            return val
    for it in items:
        val = str(it.get("itemValue") or "")
        name = str(it.get("itemName") or "")
        if "2e1a" in val.lower() or ("insta360" in name.lower() and "virtual" not in name.lower()):
            if val:
                return val
    return fallback_dev


INSTA360_4K30_MJPEG_SETTINGS = {
    "use_preset": False,
    "supported_format": "Mzg0MHgyMTYwIDI0LjAwMC0yNC4wMDAyNS4wMDAtMjUuMDAwMzAuMDAwLTMwLjAwMCAyIDg3NTcwNDQzOA==",
    "resolution": '{"width":3840,"height":2160}',
    "input_format": 875704438,
    "color_space": 2,
    "video_range": 1,
    "frame_rate": {"numerator": 30000030, "denominator": 1000000},
}

INSTA360_1080P30_MJPEG_SETTINGS = {
    "use_preset": False,
    "supported_format": "MTkyMHgxMDgwIDI0LjAwMC0yNC4wMDAyNS4wMDAtMjUuMDAwMzAuMDAwLTMwLjAwMDUwLjAwMC01MC4wMDA2MC4wMDAtNjAuMDAwIDIgODc1NzA0NDM4",
    "resolution": '{"width":1920,"height":1080}',
    "input_format": 875704438,
    "color_space": 2,
    "video_range": 1,
    "frame_rate": {"numerator": 30000030, "denominator": 1000000},
}


def ensure_scene_item_4k_bounds(obs: OBSSession, cam_name: str) -> None:
    """
    Ensures the camera scene item uses OBS_BOUNDS_SCALE_INNER (3840x2160) so temporary
    1080p30 thermal step-downs fill the full 4K broadcast canvas with zero layout shift.
    """
    scene = obs.rpc("GetCurrentProgramScene", check=False).get("currentProgramSceneName") or "Scene"
    items = obs.rpc("GetSceneItemList", {"sceneName": scene}, check=False).get("sceneItems") or []
    for it in items:
        if it.get("sourceName") == cam_name:
            item_id = it.get("sceneItemId")
            tf = (
                obs.rpc("GetSceneItemTransform", {"sceneName": scene, "sceneItemId": item_id}, check=False).get(
                    "sceneItemTransform"
                )
                or {}
            )
            if tf.get("boundsType") != "OBS_BOUNDS_SCALE_INNER" or float(tf.get("boundsWidth") or 0) < 3840.0:
                obs.rpc(
                    "SetSceneItemTransform",
                    {
                        "sceneName": scene,
                        "sceneItemId": item_id,
                        "sceneItemTransform": {
                            "positionX": 0.0,
                            "positionY": 0.0,
                            "boundsType": "OBS_BOUNDS_SCALE_INNER",
                            "boundsWidth": 3840.0,
                            "boundsHeight": 2160.0,
                        },
                    },
                    check=False,
                )


def ensure_insta360_mjpeg_locked(
    obs: OBSSession, cam_name: str, logger: MonitorLogger, target_mode: str = "4k30"
) -> None:
    """
    Ensures Insta360 Link 2C Pro in OBS is locked to explicit MJPEG (NV12/420v, formatIndex 2)
    at either '4k30' (normal) or '1080p30' (temporary afternoon solar thermal cooldown),
    avoiding Use Preset (which silently falls into buggy H264 formatIndex 1).
    """
    if not cam_name:
        return
    cur = obs.rpc("GetInputSettings", {"inputName": cam_name}, check=False).get("inputSettings") or {}
    dev = str(cur.get("device") or "")
    dev_name = str(cur.get("device_name") or "")
    if "2e1a" not in dev.lower() and "insta360 link 2" not in dev_name.lower():
        return
    target_cfg = INSTA360_1080P30_MJPEG_SETTINGS if target_mode == "1080p30" else INSTA360_4K30_MJPEG_SETTINGS
    label = "1080p30 MJPEG (thermal cooldown)" if target_mode == "1080p30" else "4K30 MJPEG"
    if (
        cur.get("use_preset") is False
        and cur.get("input_format") == 875704438
        and cur.get("supported_format") == target_cfg["supported_format"]
    ):
        return
    ensure_scene_item_4k_bounds(obs, cam_name)
    merged = dict(cur)
    merged.update(target_cfg)
    obs.rpc("SetInputSettings", {"inputName": cam_name, "inputSettings": merged, "overlay": False}, check=False)
    logger.info(f"Locked '{cam_name}' ({dev_name or dev}) to explicit {label} (NV12/420v, 30fps)")


def discover_camera_input(obs: OBSSession, explicit_name: str = "") -> str:
    if explicit_name:
        return explicit_name
    resp = obs.rpc("GetInputList", check=False)
    for inp in resp.get("inputs") or []:
        kind = inp.get("unversionedInputKind") or inp.get("inputKind") or ""
        if kind.startswith("macos-avcapture") or kind.startswith("av_capture_input"):
            return inp.get("inputName") or ""
    return ""


def grab_source_image(
    obs: OBSSession, source_name: str, fmt: str = "png", width: int = 160, height: int = 90, quality: int = 80
) -> bytes:
    params: Dict[str, object] = {
        "sourceName": source_name,
        "imageFormat": fmt,
        "imageWidth": width,
        "imageHeight": height,
    }
    if fmt in ("jpg", "jpeg"):
        params["imageCompressionQuality"] = quality
    shot = obs.rpc("GetSourceScreenshot", params, check=False)
    b64 = shot.get("imageData") or ""
    if "," in b64:
        b64 = b64.split(",", 1)[1]
    return base64.b64decode(b64) if b64 else b""


def run_test_alert(args, logger: MonitorLogger) -> int:
    logger.info(f"Connecting to {args.obs_url} for live test alert -> {args.ntfy_server}/{args.ntfy_topic}")
    obs = OBSSession(args.obs_url)
    try:
        cam_name = discover_camera_input(obs, args.camera_input)
        stats = obs.rpc("GetStats", check=False)
        stream = obs.rpc("GetStreamStatus", check=False)

        mad_str = "n/a"
        jpeg = b""
        if cam_name:
            p1 = decode_png_rgb_bytes(grab_source_image(obs, cam_name, "png", 160, 90))
            time.sleep(0.8)
            p2 = decode_png_rgb_bytes(grab_source_image(obs, cam_name, "png", 160, 90))
            mad, chg = compute_rgb_diff(p1, p2)
            mad_str = f"MAD={mad:.2f}/255 ({chg:.1f}% pixels changing)"
            jpeg = grab_source_image(obs, cam_name, "jpg", 640, 360, 82)

        fps = stats.get("activeFps", 0.0)
        r_skip = stats.get("renderSkippedFrames", 0)
        r_tot = max(1, stats.get("renderTotalFrames", 1))
        live = stream.get("outputActive", False)

        msg = (
            f"24/7 freeze monitor is active on {socket.gethostname()}.\n"
            f"Camera: '{cam_name or 'none'}' ({mad_str})\n"
            f"OBS: {fps:.2f} FPS | Render lag: {r_skip}/{r_tot} ({100.0 * r_skip / r_tot:.2f}%) | "
            f"YouTube: {'LIVE' if live else 'INACTIVE'}"
        )
        ok = send_ntfy(
            args.ntfy_server,
            args.ntfy_topic,
            "Hudson OBS Freeze Monitor Armed",
            msg,
            priority=3,
            tags="white_check_mark,movie_camera",
            jpeg_bytes=jpeg or None,
            filename="hudson_live_check.jpg",
            logger=logger,
        )
        return 0 if ok else 1
    finally:
        obs.close()


def run_monitor_loop(args, logger: MonitorLogger):
    logger.info(
        f"Starting 24/7 OBS Freeze Monitor -> obs={args.obs_url}, ntfy={args.ntfy_server}/{args.ntfy_topic}, "
        f"interval={args.interval}s, threshold=MAD<{args.mad_threshold} x{args.frozen_checks} checks, "
        f"auto_reset={not args.alert_only}, thermal_stepdown={not args.no_thermal_stepdown}"
    )

    obs: Optional[OBSSession] = None
    pending_restore: Optional[Tuple[str, str]] = None

    prev_rgb: Optional[bytearray] = None
    prev_sha256: str = ""
    frozen_streak = 0
    blackout_streak = 0
    reset_cooldown = 20.0
    last_reset_ts = 0.0

    thermal_cooldown_until: float = 0.0
    recent_stall_ts: List[float] = []
    last_auto_heal_notify_ts: float = 0.0

    obs_down_streak = 0
    obs_down_alerted = False

    saw_stream_live = False
    stream_down_streak = 0
    stream_down_alerted = False

    prev_render_skipped: Optional[int] = None
    prev_render_total: Optional[int] = None
    prev_lag_window_ts = 0.0
    last_lag_alert_ts = 0.0
    last_heartbeat_log_ts = 0.0

    while True:
        loop_started = time.time()
        try:
            if obs is None:
                obs = OBSSession(args.obs_url)
                logger.info(f"Connected to OBS WebSocket at {args.obs_url}")
                if obs_down_alerted:
                    send_ntfy(
                        args.ntfy_server,
                        args.ntfy_topic,
                        "OBS Studio Back Online",
                        f"Reconnected to OBS WebSocket on {socket.gethostname()}.",
                        priority=3,
                        tags="white_check_mark",
                        logger=logger,
                    )
                    obs_down_alerted = False
                obs_down_streak = 0
                prev_rgb = None
                prev_sha256 = ""
                frozen_streak = 0
                blackout_streak = 0

                # Crash-safe restore if interrupted mid-reset
                if pending_restore:
                    p_cam, p_dev = pending_restore
                    cur = (obs.rpc("GetInputSettings", {"inputName": p_cam}, check=False).get("inputSettings") or {})
                    if not cur.get("device"):
                        obs.rpc(
                            "SetInputSettings",
                            {"inputName": p_cam, "inputSettings": {"device": p_dev}, "overlay": True},
                            check=False,
                        )
                        logger.info(f"Restored pending device '{p_dev}' on '{p_cam}'")
                    pending_restore = None

                init_mode = "1080p30" if time.time() < thermal_cooldown_until else "4k30"
                ensure_insta360_mjpeg_locked(
                    obs, discover_camera_input(obs, args.camera_input), logger, target_mode=init_mode
                )
                resync_uvc_exposure()

            stats = obs.rpc("GetStats", check=False)
            stream = obs.rpc("GetStreamStatus", check=False)
            cam_name = discover_camera_input(obs, args.camera_input)

            fps = float(stats.get("activeFps") or 0.0)
            r_skip = int(stats.get("renderSkippedFrames") or 0)
            r_tot = int(stats.get("renderTotalFrames") or 0)
            live = bool(stream.get("outputActive", False))
            reconnecting = bool(stream.get("outputReconnecting", False))

            # 0. Restore native 4K30 MJPEG once an afternoon solar thermal cooldown expires
            now = time.time()
            if thermal_cooldown_until > 0.0 and now >= thermal_cooldown_until and cam_name:
                thermal_cooldown_until = 0.0
                recent_stall_ts.clear()
                ensure_insta360_mjpeg_locked(obs, cam_name, logger, target_mode="4k30")
                resync_uvc_exposure()
                time.sleep(2.5)
                restored_jpg = grab_source_image(obs, cam_name, "jpg", 640, 360, 82)
                prev_rgb = None
                prev_sha256 = ""
                frozen_streak = 0
                blackout_streak = 0
                logger.info(f"Thermal cooldown complete on '{cam_name}'; restored native 4K30 MJPEG.")
                send_ntfy(
                    args.ntfy_server,
                    args.ntfy_topic,
                    "Hudson Camera Restored to 4K30",
                    f"Thermal cooldown finished; '{cam_name}' is back at native 3840x2160 @ 30 fps MJPEG.",
                    priority=2,
                    tags="sunny,movie_camera",
                    jpeg_bytes=restored_jpg or None,
                    filename="restored_4k30.jpg",
                    logger=logger,
                )

            # 1. Monitor YouTube Live stream continuity
            if live:
                if stream_down_alerted:
                    send_ntfy(
                        args.ntfy_server,
                        args.ntfy_topic,
                        "Hudson YouTube Stream Resumed",
                        f"OBS output is LIVE again ({fps:.1f} FPS).",
                        priority=3,
                        tags="white_check_mark,satellite",
                        logger=logger,
                    )
                    stream_down_alerted = False
                saw_stream_live = True
                stream_down_streak = 0
            elif saw_stream_live and not reconnecting:
                stream_down_streak += 1
                if stream_down_streak >= 4 and not stream_down_alerted:
                    down_s = int(stream_down_streak * args.interval)
                    send_ntfy(
                        args.ntfy_server,
                        args.ntfy_topic,
                        "Hudson YouTube Stream Stopped",
                        f"OBS stream output has been inactive for ~{down_s}s (reconnecting=False).",
                        priority=5,
                        tags="warning,satellite",
                        logger=logger,
                    )
                    stream_down_alerted = True

            # 2. Monitor rolling 60s OBS rendering lag spikes (>10%)
            now = time.time()
            if prev_render_total is None:
                prev_render_skipped = r_skip
                prev_render_total = r_tot
                prev_lag_window_ts = now
            elif (now - prev_lag_window_ts) >= 60.0:
                d_tot = r_tot - prev_render_total
                d_skip = r_skip - (prev_render_skipped or 0)
                if d_tot >= 300:
                    lag_pct = (100.0 * max(0, d_skip)) / d_tot
                    if lag_pct >= 10.0 and (now - last_lag_alert_ts) >= 1800.0:
                        send_ntfy(
                            args.ntfy_server,
                            args.ntfy_topic,
                            f"OBS Rendering Lag Spike ({lag_pct:.1f}%)",
                            (
                                f"OBS missed {d_skip}/{d_tot} render frames ({lag_pct:.1f}%) over the last "
                                f"{int(now - prev_lag_window_ts)}s (FPS={fps:.1f})."
                            ),
                            priority=4,
                            tags="warning,hourglass_flowing_sand",
                            logger=logger,
                        )
                        last_lag_alert_ts = now
                prev_render_skipped = r_skip
                prev_render_total = r_tot
                prev_lag_window_ts = now

            # 3. Monitor raw camera pixel diff ('Video Capture Device 2')
            if cam_name:
                act = obs.rpc("GetSourceActive", {"sourceName": cam_name}, check=False)
                if act.get("videoShowing", False):
                    png = grab_source_image(obs, cam_name, "png", 160, 90)
                    if png:
                        cur_sha = hashlib.sha256(png).hexdigest()
                        cur_rgb = decode_png_rgb_bytes(png)
                        mean_lvl, max_lvl = compute_rgb_levels(cur_rgb)

                        is_blackout = mean_lvl < 0.8 and max_lvl <= 2
                        blackout_streak = (blackout_streak + 1) if is_blackout else 0

                        mad = 255.0
                        chg_pct = 100.0
                        exact_same = False
                        if prev_rgb is not None and len(prev_rgb) == len(cur_rgb):
                            exact_same = cur_sha == prev_sha256
                            mad, chg_pct = (0.0, 0.0) if exact_same else compute_rgb_diff(prev_rgb, cur_rgb)
                            if mad < args.mad_threshold:
                                frozen_streak += 1
                                logger.info(
                                    f"Freeze candidate #{frozen_streak}/{args.frozen_checks} on '{cam_name}': "
                                    f"MAD={mad:.4f}/255, changed={chg_pct:.2f}%, exact_sha_match={exact_same}"
                                )
                            else:
                                if frozen_streak > 0:
                                    logger.info(
                                        f"Motion resumed on '{cam_name}': MAD={mad:.3f}/255 ({chg_pct:.1f}% changed)"
                                    )
                                frozen_streak = 0
                                if not is_blackout:
                                    reset_cooldown = 20.0

                        prev_rgb = cur_rgb
                        prev_sha256 = cur_sha

                        # Periodic quiet hourly heartbeat in local monitor.log
                        if (now - last_heartbeat_log_ts) >= 3600.0:
                            last_heartbeat_log_ts = now
                            cur_mode = "1080p30-cooldown" if now < thermal_cooldown_until else "4k30"
                            logger.info(
                                f"Heartbeat OK: cam='{cam_name}' ({cur_mode}) MAD={mad:.2f} mean_rgb={mean_lvl:.1f} "
                                f"fps={fps:.2f} render_skip={r_skip}/{r_tot} live={live}"
                            )

                        # Trigger freeze OR complete-blackout incident!
                        trigger_reason = ""
                        if frozen_streak >= args.frozen_checks:
                            sec = int(frozen_streak * args.interval)
                            exact_str = ", byte-identical SHA256" if exact_same else ""
                            trigger_reason = (
                                f"Frozen for ~{sec}s (MAD={mad:.4f}/255, {chg_pct:.2f}% changed{exact_str})"
                            )
                        elif blackout_streak >= args.frozen_checks:
                            sec = int(blackout_streak * args.interval)
                            trigger_reason = f"Pure-black signal dropout for ~{sec}s (mean_rgb={mean_lvl:.2f})"

                        if trigger_reason and (now - last_reset_ts) >= reset_cooldown:
                            last_reset_ts = now
                            snap_jpg = grab_source_image(obs, cam_name, "jpg", 640, 360, 82)
                            saved_path = logger.save_incident_image("frozen", snap_jpg)
                            logger.info(f"INCIDENT on '{cam_name}': {trigger_reason} (saved={saved_path})")
                            lag_total_pct = 100.0 * r_skip / max(1, r_tot)

                            if args.alert_only:
                                send_ntfy(
                                    args.ntfy_server,
                                    args.ntfy_topic,
                                    "Hudson Camera FROZEN",
                                    (
                                        f"'{cam_name}': {trigger_reason}.\n"
                                        f"OBS: {fps:.1f} FPS | Cumulative lag: {lag_total_pct:.2f}% | "
                                        f"YouTube: {'LIVE' if live else 'OFF'}\n"
                                        f"Alert-only mode enabled (leaving camera untouched)."
                                    ),
                                    priority=5,
                                    tags="rotating_light,movie_camera",
                                    jpeg_bytes=snap_jpg or None,
                                    filename="frozen_frame.jpg",
                                    logger=logger,
                                )
                                reset_cooldown = (
                                    120.0 if reset_cooldown < 120.0 else min(1800.0, reset_cooldown * 2.0)
                                )
                                frozen_streak = 0
                                blackout_streak = 0
                            else:
                                s_resp = obs.rpc("GetInputSettings", {"inputName": cam_name}, check=False)
                                orig_dev = (s_resp.get("inputSettings") or {}).get("device") or ""
                                if orig_dev:
                                    # Stage 1: Soft OBS AVCaptureSession cycle (~5.5s)
                                    pending_restore = (cam_name, orig_dev)
                                    try:
                                        obs.rpc(
                                            "SetInputSettings",
                                            {
                                                "inputName": cam_name,
                                                "inputSettings": {"device": ""},
                                                "overlay": True,
                                            },
                                        )
                                        time.sleep(1.5)
                                    finally:
                                        obs.rpc(
                                            "SetInputSettings",
                                            {
                                                "inputName": cam_name,
                                                "inputSettings": {"device": orig_dev},
                                                "overlay": True,
                                            },
                                        )
                                        pending_restore = None
                                    resync_uvc_exposure()
                                    logger.info(f"Cycled AVCaptureSession on '{cam_name}' ({orig_dev}); verifying...")
                                    time.sleep(4.0)

                                    v1 = decode_png_rgb_bytes(grab_source_image(obs, cam_name, "png", 160, 90))
                                    time.sleep(1.0)
                                    v2_png = grab_source_image(obs, cam_name, "png", 160, 90)
                                    v2 = decode_png_rgb_bytes(v2_png)
                                    post_mad, post_chg = compute_rgb_diff(v1, v2)
                                    post_mean, post_max = compute_rgb_levels(v2)
                                    after_jpg = grab_source_image(obs, cam_name, "jpg", 640, 360, 82)
                                    logger.save_incident_image("after_reset", after_jpg)

                                    recovered = post_mad >= args.mad_threshold and not (
                                        post_mean < 0.8 and post_max <= 2
                                    )
                                    recovery_method = "OBS AVCaptureSession cycle"

                                    # Stage 2 Escalation: If soft OBS cycle failed (USB bulk endpoint hardware STALL),
                                    # perform programmatic IOKit USBDeviceReEnumerate (software unplug/replug).
                                    # Insta360 Link 2C Pro firmware takes ~16-18s to reboot out of black frames,
                                    # so poll up to ~26s total before declaring failure.
                                    if not recovered:
                                        uvc_bin = find_uvc_util_bin(args.data_dir)
                                        if uvc_bin is not None:
                                            logger.info(
                                                f"Soft reset did not clear stall (MAD={post_mad:.4f}/255, "
                                                f"mean_rgb={post_mean:.2f}); escalating to IOKit USB re-enumeration "
                                                f"via {uvc_bin}..."
                                            )
                                            pending_restore = (cam_name, orig_dev)
                                            rebound_dev = orig_dev
                                            try:
                                                obs.rpc(
                                                    "SetInputSettings",
                                                    {
                                                        "inputName": cam_name,
                                                        "inputSettings": {"device": ""},
                                                        "overlay": True,
                                                    },
                                                    check=False,
                                                )
                                                time.sleep(1.0)
                                                proc = subprocess.run(
                                                    [str(uvc_bin), "-V", "0x2e1a", "--usb-reset"],
                                                    capture_output=True,
                                                    text=True,
                                                    timeout=10.0,
                                                )
                                                out_msg = (proc.stdout or proc.stderr or "").strip()
                                                logger.info(
                                                    f"uvc-util --usb-reset rc={proc.returncode}: {out_msg}"
                                                )
                                                time.sleep(8.0)
                                                rebound_dev = discover_insta360_device_id(obs, cam_name, orig_dev)
                                            except Exception as usb_exc:
                                                logger.info(f"WARNING: USB re-enumeration error: {usb_exc}")
                                            finally:
                                                obs.rpc(
                                                    "SetInputSettings",
                                                    {
                                                        "inputName": cam_name,
                                                        "inputSettings": {"device": rebound_dev},
                                                        "overlay": True,
                                                    },
                                                    check=False,
                                                )
                                                pending_restore = None

                                            recovery_method = "IOKit USB bus re-enumeration (software replug)"
                                            for poll_idx in range(6):
                                                time.sleep(2.5)
                                                fresh_dev = discover_insta360_device_id(obs, cam_name, rebound_dev)
                                                if fresh_dev and fresh_dev != rebound_dev:
                                                    rebound_dev = fresh_dev
                                                    obs.rpc(
                                                        "SetInputSettings",
                                                        {
                                                            "inputName": cam_name,
                                                            "inputSettings": {"device": rebound_dev},
                                                            "overlay": True,
                                                        },
                                                        check=False,
                                                    )
                                                    time.sleep(1.5)
                                                resync_uvc_exposure()
                                                v1 = decode_png_rgb_bytes(
                                                    grab_source_image(obs, cam_name, "png", 160, 90)
                                                )
                                                time.sleep(1.0)
                                                v2_png = grab_source_image(obs, cam_name, "png", 160, 90)
                                                v2 = decode_png_rgb_bytes(v2_png)
                                                post_mad, post_chg = compute_rgb_diff(v1, v2)
                                                post_mean, post_max = compute_rgb_levels(v2)
                                                recovered = post_mad >= args.mad_threshold and not (
                                                    post_mean < 0.8 and post_max <= 2
                                                )
                                                logger.info(
                                                    f"Post-USB-reset check #{poll_idx + 1}/6: "
                                                    f"MAD={post_mad:.3f}/255, mean_rgb={post_mean:.2f}, "
                                                    f"recovered={recovered}"
                                                )
                                                if recovered:
                                                    break

                                            after_jpg = grab_source_image(obs, cam_name, "jpg", 640, 360, 82)
                                            logger.save_incident_image("after_usb_reset", after_jpg)

                                    heal_finished = time.time()
                                    if recovered:
                                        reset_cooldown = 20.0
                                        logger.info(
                                            f"RECOVERY VERIFIED on '{cam_name}' ({recovery_method}): "
                                            f"MAD={post_mad:.3f}/255 ({post_chg:.1f}% changed)"
                                        )
                                        recent_stall_ts = [
                                            t for t in recent_stall_ts if (heal_finished - t) <= args.thermal_window_s
                                        ]
                                        recent_stall_ts.append(heal_finished)

                                        # Automatic sunny-afternoon thermal step-down (4K30 -> 1080p30 MJPEG)
                                        if (
                                            not args.no_thermal_stepdown
                                            and thermal_cooldown_until == 0.0
                                            and len(recent_stall_ts) >= args.thermal_stall_count
                                        ):
                                            thermal_cooldown_until = heal_finished + args.thermal_cooldown_s
                                            cooldown_min = int(round(args.thermal_cooldown_s / 60.0))
                                            window_min = int(round(args.thermal_window_s / 60.0))
                                            ensure_insta360_mjpeg_locked(
                                                obs, cam_name, logger, target_mode="1080p30"
                                            )
                                            resync_uvc_exposure()
                                            time.sleep(2.5)
                                            v2_png = grab_source_image(obs, cam_name, "png", 160, 90)
                                            v2 = decode_png_rgb_bytes(v2_png)
                                            after_jpg = grab_source_image(obs, cam_name, "jpg", 640, 360, 82)
                                            logger.info(
                                                f"THERMAL STEP-DOWN activated on '{cam_name}': "
                                                f"{len(recent_stall_ts)} stalls in <= {window_min}m -> "
                                                f"running 1080p30 MJPEG (scaled to 4K) for {cooldown_min}m."
                                            )
                                            send_ntfy(
                                                args.ntfy_server,
                                                args.ntfy_topic,
                                                "Hudson Camera Thermal Step-Down (1080p30)",
                                                (
                                                    f"'{cam_name}' stalled {len(recent_stall_ts)} times within "
                                                    f"{window_min} min (afternoon window sun/heat) and auto-recovered "
                                                    f"via {recovery_method}.\n"
                                                    f"Stepped camera down to 1080p30 MJPEG (scaled seamlessly to 4K) "
                                                    f"for {cooldown_min} min to cool the sensor; will restore 4K30 "
                                                    f"automatically."
                                                ),
                                                priority=3,
                                                tags="thermometer,sunny,movie_camera",
                                                jpeg_bytes=after_jpg or None,
                                                filename="thermal_stepdown_1080p.jpg",
                                                logger=logger,
                                            )
                                            last_auto_heal_notify_ts = heal_finished
                                        elif (
                                            heal_finished - last_auto_heal_notify_ts
                                        ) >= args.auto_heal_notify_cooldown:
                                            last_auto_heal_notify_ts = heal_finished
                                            quiet_min = int(round(args.auto_heal_notify_cooldown / 60.0))
                                            send_ntfy(
                                                args.ntfy_server,
                                                args.ntfy_topic,
                                                "Hudson Camera Auto-Healed",
                                                (
                                                    f"'{cam_name}' ({trigger_reason}) auto-recovered via "
                                                    f"{recovery_method} (MAD={post_mad:.2f}/255).\n"
                                                    f"Routine auto-heal notices are rate-limited to 1 per {quiet_min}m."
                                                ),
                                                priority=2,
                                                tags="wrench,white_check_mark",
                                                jpeg_bytes=after_jpg or None,
                                                filename="recovered_frame.jpg",
                                                logger=logger,
                                            )
                                    else:
                                        reset_cooldown = (
                                            120.0
                                            if reset_cooldown < 120.0
                                            else min(1800.0, reset_cooldown * 2.0)
                                        )
                                        logger.info(
                                            f"WARNING: Camera '{cam_name}' still appears stuck after {recovery_method} "
                                            f"(MAD={post_mad:.4f}/255, next retry in {int(reset_cooldown)}s)"
                                        )
                                        send_ntfy(
                                            args.ntfy_server,
                                            args.ntfy_topic,
                                            "Hudson Camera STILL FROZEN — Needs Check",
                                            (
                                                f"'{cam_name}': {trigger_reason}.\n"
                                                f"Tried soft OBS reset + IOKit USB re-enumeration, "
                                                f"but diff is still MAD={post_mad:.4f}/255 (mean_rgb={post_mean:.1f}).\n"
                                                f"Next auto-retry in {max(1, int(round(reset_cooldown / 60.0)))} min."
                                            ),
                                            priority=5,
                                            tags="sos,rotating_light",
                                            jpeg_bytes=after_jpg or None,
                                            filename="still_frozen.jpg",
                                            logger=logger,
                                        )

                                    prev_rgb = v2 if v2 else None
                                    prev_sha256 = hashlib.sha256(v2_png).hexdigest() if v2_png else ""
                                    frozen_streak = 0
                                    blackout_streak = 0
                else:
                    prev_rgb = None
                    prev_sha256 = ""
                    frozen_streak = 0
                    blackout_streak = 0

        except KeyboardInterrupt:
            logger.info("Monitor stopped by user.")
            if obs:
                obs.close()
            return
        except Exception as exc:
            if obs:
                obs.close()
                obs = None
            obs_down_streak += 1
            logger.info(f"OBS connection/check error (#{obs_down_streak}): {exc}")
            if obs_down_streak >= 3 and not obs_down_alerted:
                down_s = int(obs_down_streak * args.interval)
                send_ntfy(
                    args.ntfy_server,
                    args.ntfy_topic,
                    "OBS Studio Unreachable",
                    (
                        f"Cannot connect to OBS WebSocket ({args.obs_url}) on {socket.gethostname()} "
                        f"for ~{down_s}s: {exc}"
                    ),
                    priority=5,
                    tags="rotating_light,computer",
                    logger=logger,
                )
                obs_down_alerted = True

        elapsed = time.time() - loop_started
        time.sleep(max(1.0, args.interval - elapsed))


def main():
    parser = argparse.ArgumentParser(description="24/7 OBS Webcam Freeze & Stream Health Monitor")
    parser.add_argument("--obs-url", default=DEFAULT_OBS_URL, help="OBS WebSocket v5 URL (default: obs://127.0.0.1:4455)")
    parser.add_argument("--ntfy-topic", default=DEFAULT_NTFY_TOPIC, help="Private ntfy.sh topic name")
    parser.add_argument("--ntfy-server", default=DEFAULT_NTFY_SERVER, help="ntfy server URL (default: https://ntfy.sh)")
    parser.add_argument("--camera-input", default="", help="Explicit OBS camera input name (auto-discovers macos-avcapture if empty)")
    parser.add_argument("--interval", type=float, default=10.0, help="Seconds between frame diff checks (default: 10)")
    parser.add_argument("--frozen-checks", type=int, default=3, help="Consecutive static checks before recovery (default: 3 = ~30s)")
    parser.add_argument("--mad-threshold", type=float, default=0.05, help="Mean absolute RGB diff (0..255) below which frames count as frozen (default: 0.05)")
    parser.add_argument("--auto-heal-notify-cooldown", type=float, default=1800.0, help="Min seconds between routine low-priority auto-heal ntfy notices (default: 1800 = 30m)")
    parser.add_argument("--thermal-stall-count", type=int, default=3, help="Stalls within thermal window that trigger temporary 1080p30 cooldown (default: 3)")
    parser.add_argument("--thermal-window-s", type=float, default=1200.0, help="Rolling window in seconds for counting repeated solar/thermal stalls (default: 1200 = 20m)")
    parser.add_argument("--thermal-cooldown-s", type=float, default=3600.0, help="Seconds to hold 1080p30 MJPEG before restoring native 4K30 (default: 3600 = 60m)")
    parser.add_argument("--no-thermal-stepdown", action="store_true", help="Disable automatic 4K30 -> 1080p30 thermal step-down on repeated stalls")
    parser.add_argument("--alert-only", action="store_true", help="Only send ntfy alerts; do not auto-reset OBS camera input")
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR, help="Directory for logs and incident snapshots")
    parser.add_argument("--test-alert", action="store_true", help="Send a live test push notification with snapshot and exit")
    args = parser.parse_args()

    logger = MonitorLogger(args.data_dir)
    if args.test_alert:
        sys.exit(run_test_alert(args, logger))
    run_monitor_loop(args, logger)


if __name__ == "__main__":
    main()
