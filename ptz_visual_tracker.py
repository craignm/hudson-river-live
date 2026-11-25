"""
Real-Time Closed-Loop Visual Framing & Tracking Controller (`ptz_visual_tracker.py`)
Pure Python 3.9+ standard library + persistent Apple Neural Engine helper (`./ptz-tracker`).

Architecture (RF Acquisition -> Visual Lock & Center -> Smooth Zoom & Inertial LOS Tracking):
  1. Wide Acquisition Basket (`ACQUIRE`):
     When switching to an AIS vessel or ADS-B aircraft/helicopter, starts at a wider acquisition
     zoom (~7x-12x instead of jumping straight to 25x-30x) so RF latency/antenna-offset errors
     still place the boat or aircraft comfortably inside the PTZ frame.
  2. Apple Neural Engine Recognition (`ptz-tracker`):
     Streams ~8 Hz live frames from the PTZ camera (`Video Capture Device 4 (Sony A40)` via
     `OBSCameraBridge.get_ptz_frame`), running CoreML YOLOv11s + Apple Vision saliency/classifier
     + stateful `VNTrackObjectRequest` optical-flow tracking in ~6 ms/frame.
  3. Latency-Invariant Inertial Line-of-Sight (LOS) Estimator:
     Reconstructs the target's true gimbal-space angle at frame capture time:
       `LOS_hw(t_frame) = CamPose_hw(t_frame) + PixelOffset_deg(u, v, hfov)`
     and filters `Bias(t) = LOS_hw(t_frame) - RF_Pred_hw(t_frame)` with an alpha-beta tracker.
     Because `CamPose + PixelOffset` is invariant to camera motion, video capture latency never
     causes feedback oscillation or overshoot.
  4. Zoom-Invariant Auto-Framing:
     Computes the object's true angular span (`det_w * cam_hfov_deg`) to smoothly ramp zoom from
     acquisition zoom up to framing zoom while preventing large barges/ferries/close aircraft
     from clipping the edges of the frame.
"""

import base64
import json
import math
import shutil
import subprocess
import tempfile
import threading
import time
import urllib.request
import zipfile
from pathlib import Path
from typing import Dict, Optional, Tuple


YOLO11S_COREML_ZIP_URL = (
    "https://huggingface.co/SharpAI/yolo11s-coreml-fp16/resolve/main/yolo11s_mlpackage_fp16_auto.zip"
)


class PTZVisualTracker:
    """Background 8 Hz visual detector/tracker driving closed-loop PTZ centering & framing."""

    def __init__(self, director, base_dir: Path):
        self.director = director
        self.base_dir = Path(base_dir)
        self.models_dir = self.base_dir / "models"
        self.mlpackage_path = self.models_dir / "yolo11s.mlpackage"
        self.mlmodelc_path = self.models_dir / "yolo11s.mlmodelc"

        self.lock = threading.Lock()
        self._wake_event = threading.Event()
        self._proc: Optional[subprocess.Popen] = None
        self._proc_lock = threading.Lock()
        self._model_ready = False
        self._downloading_model = False

        # Active target visual tracking state (read by `WorldDirector` & `SonySRGA40Driver`)
        self._target_id: Optional[str] = None
        self._target_kind: str = ""
        self._target_started_ts: float = 0.0
        self._phase: str = "idle"  # idle | acquiring | locking | tracking | coasting
        self._locked: bool = False
        self._hit_streak: int = 0
        self._miss_streak: int = 0
        self._last_lock_ts: float = 0.0
        self._last_opt_ts: float = 0.0

        # Filtered RF -> Visual offset in hardware gimbal degrees & deg/s
        self._bias_pan_hw: float = 0.0
        self._bias_tilt_hw: float = 0.0
        self._bias_pan_dps: float = 0.0
        self._bias_tilt_dps: float = 0.0
        self._prev_raw_bias: Optional[Tuple[float, float, float, float]] = None

        # Latest visual bounding box & zoom-invariant angular size
        self._det_u: float = 0.5
        self._det_v: float = 0.5
        self._det_w: float = 0.0
        self._det_h: float = 0.0
        self._det_conf: float = 0.0
        self._det_label: str = ""
        self._det_source: str = ""
        self._infer_ms: float = 0.0
        self._obj_span_w_deg: float = 0.0
        self._obj_span_h_deg: float = 0.0

        # Smooth zoom ramp & discrete re-center trigger sequence
        self._cur_zoom_cap: float = 10.0
        self._recenter_seq: int = 0
        self._last_recenter_ts: float = 0.0
        self._last_ptz_query_ts: float = 0.0

        threading.Thread(target=self._ensure_yolo_model_bg, daemon=True).start()
        threading.Thread(target=self._worker_loop, daemon=True).start()

    def _find_tracker_bin(self) -> Optional[Path]:
        candidates = [
            self.base_dir / "ptz-tracker",
            Path(__file__).resolve().parent / "ptz-tracker",
        ]
        for p in candidates:
            if p.is_file():
                return p
        return None

    def _ensure_yolo_model_bg(self):
        """Downloads `yolo11s.mlpackage` (~17 MB) once into `models/` if not already cached."""
        if (self.mlpackage_path / "Manifest.json").is_file() or self.mlmodelc_path.exists():
            self._model_ready = True
            return
        if self._downloading_model:
            return
        self._downloading_model = True
        tmp_dir = Path(tempfile.mkdtemp(prefix="yolo11s_dl_"))
        try:
            self.models_dir.mkdir(parents=True, exist_ok=True)
            zip_path = tmp_dir / "yolo11s.zip"
            print("[Visual Tracker] Downloading YOLOv11s CoreML FP16 model (~17 MB) for Neural Engine…", flush=True)
            req = urllib.request.Request(YOLO11S_COREML_ZIP_URL, headers={"User-Agent": "HudsonRiverPTZ/1.0"})
            with urllib.request.urlopen(req, timeout=30.0) as resp:
                zip_path.write_bytes(resp.read())
            unpack_dir = tmp_dir / "unpacked"
            unpack_dir.mkdir(parents=True, exist_ok=True)
            with zipfile.ZipFile(zip_path, "r") as zf:
                zf.extractall(unpack_dir)
            pkg_dst = self.mlpackage_path
            if (unpack_dir / "Manifest.json").is_file():
                if pkg_dst.exists():
                    shutil.rmtree(pkg_dst, ignore_errors=True)
                shutil.move(str(unpack_dir), str(pkg_dst))
            else:
                subs = list(unpack_dir.glob("*.mlpackage"))
                if subs:
                    if pkg_dst.exists():
                        shutil.rmtree(pkg_dst, ignore_errors=True)
                    shutil.move(str(subs[0]), str(pkg_dst))
            if (pkg_dst / "Manifest.json").is_file():
                self._model_ready = True
                print(f"[Visual Tracker] Installed {pkg_dst} — restarting ptz-tracker with YOLOv11s.", flush=True)
                self._restart_proc()
        except Exception as exc:
            print(f"[Visual Tracker] Model download warning (using Apple Vision saliency fallback): {exc}", flush=True)
        finally:
            self._downloading_model = False
            shutil.rmtree(tmp_dir, ignore_errors=True)

    def _restart_proc(self):
        with self._proc_lock:
            if self._proc is not None:
                try:
                    self._proc.kill()
                except Exception:
                    pass
                self._proc = None

    def _rpc(self, payload: dict) -> Optional[dict]:
        bin_path = self._find_tracker_bin()
        if bin_path is None:
            return None
        with self._proc_lock:
            if self._proc is None or self._proc.poll() is not None:
                args = [str(bin_path), str(self.mlpackage_path), str(self.mlmodelc_path)]
                try:
                    self._proc = subprocess.Popen(
                        args,
                        stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.DEVNULL,
                        text=True,
                        bufsize=1,
                    )
                except Exception:
                    self._proc = None
                    return None
            try:
                assert self._proc.stdin is not None and self._proc.stdout is not None
                self._proc.stdin.write(json.dumps(payload) + "\n")
                self._proc.stdin.flush()
                line = self._proc.stdout.readline()
                if not line:
                    self._proc = None
                    return None
                return json.loads(line)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass
                self._proc = None
                return None

    @staticmethod
    def compute_acquisition_zoom(rec_zoom_x: float, kind: str) -> float:
        """
        Starts slightly zoomed out from final framing zoom so boats/planes near the edge of the
        RF prediction basket are reliably captured inside the initial PTZ frame.
        """
        rz = max(1.0, min(30.0, float(rec_zoom_x)))
        if rz <= 6.5:
            return rz
        if kind in ("aircraft", "helicopter"):
            return round(max(6.5, min(11.0, rz * 0.46)), 2)
        return round(max(7.5, min(13.5, rz * 0.54)), 2)

    def notify_active_target(self, target: Optional[dict], manual_zoom_override: Optional[float]) -> Tuple[float, Optional[dict]]:
        """
        Called by `WorldDirector.update_world()` (~4 Hz) when driving a target.
        Returns `(effective_zoom_x, visual_guidance_dict_or_None)`.
        """
        now = time.time()
        if not target or target.get("kind") not in ("vessel", "aircraft", "helicopter"):
            with self.lock:
                if self._target_id is not None:
                    self._reset_locked(None, "", now)
            rec_z = float(manual_zoom_override or (target.get("rec_zoom_x", 1.0) if target else 1.0))
            return rec_z, None

        tid = str(target["id"])
        kind = str(target.get("kind") or "vessel")
        rec_z = float(target.get("rec_zoom_x", 15.0))

        with self.lock:
            if tid != self._target_id:
                self._reset_locked(tid, kind, now)
                self._cur_zoom_cap = self.compute_acquisition_zoom(rec_z, kind)
                self._wake_event.set()

            age_since_start = now - self._target_started_ts
            age_since_lock = (now - self._last_lock_ts) if self._last_lock_ts > 0.0 else 999.0
            active_lock = self._locked and age_since_lock <= 2.3
            occluded_sec = age_since_lock if self._last_lock_ts > 0.0 else age_since_start

            # Determine zoom progression:
            #   - Hold wider acquisition zoom while acquiring OR when re-finding via AIS/ADS-B after occlusion!
            #   - Once visually locked & centered, glide smoothly up to visual object-size cap.
            acq_z = self.compute_acquisition_zoom(rec_z, kind)
            if manual_zoom_override is not None:
                eff_zoom = float(manual_zoom_override)
            elif active_lock:
                # Cap zoom so the real visual hull/fuselage occupies <= 38% width & <= 42% height
                vis_max_z = 30.0
                if self._obj_span_w_deg > 0.05 or self._obj_span_h_deg > 0.05:
                    need_hfov = max(
                        2.67,
                        self._obj_span_w_deg / 0.38,
                        (self._obj_span_h_deg / 0.42) * (16.0 / 9.0),
                    )
                    vis_max_z = max(3.5, min(30.0, 70.0 / need_hfov))
                goal_z = round(min(rec_z, vis_max_z), 2)
                off_ctr = math.hypot(self._det_u - 0.5, self._det_v - 0.5)
                if off_ctr <= 0.34:
                    # Single smooth Sony hardware ZoomSpeedScale=45 glide from acq_z -> goal_z
                    # with an 18% deadband thereafter so zoom never chatters or blocks focus sync!
                    rel_diff = abs(goal_z - self._cur_zoom_cap) / max(1.0, self._cur_zoom_cap)
                    if self._cur_zoom_cap <= acq_z + 0.25 or rel_diff >= 0.18:
                        self._cur_zoom_cap = goal_z
                elif off_ctr > 0.42 and self._cur_zoom_cap > acq_z * 1.25:
                    self._cur_zoom_cap = round(max(acq_z, self._cur_zoom_cap * 0.72), 2)
                eff_zoom = round(max(1.5, min(30.0, self._cur_zoom_cap)), 2)
            elif self._phase == "reacquiring_rf" or age_since_start < 4.6:
                # Occluded behind a building or still acquiring: widen to acq_z to re-find via AIS/ADS-B!
                self._cur_zoom_cap = acq_z
                eff_zoom = acq_z
            else:
                # Graceful RF fallback if visual recognition never locked (e.g. heavy fog / unlit night target)
                self._cur_zoom_cap = rec_z
                eff_zoom = round(rec_z, 2)

            # Extrapolate bias slightly from `_last_opt_ts` to `now`
            dt_ext = max(0.0, min(0.8, now - self._last_opt_ts)) if self._last_opt_ts > 0.0 else 0.0
            bp = (self._bias_pan_hw + self._bias_pan_dps * dt_ext) if active_lock else 0.0
            bt = (self._bias_tilt_hw + self._bias_tilt_dps * dt_ext) if active_lock else 0.0

            guidance = {
                "target_id": tid,
                "locked": active_lock,
                "phase": self._phase,
                "occluded_sec": round(occluded_sec if not active_lock else 0.0, 2),
                "bias_pan_hw": round(bp, 4),
                "bias_tilt_hw": round(bt, 4),
                "bias_pan_dps": round(self._bias_pan_dps if active_lock else 0.0, 4),
                "bias_tilt_dps": round(self._bias_tilt_dps if active_lock else 0.0, 4),
                "det_u": round(self._det_u, 4),
                "det_v": round(self._det_v, 4),
                "det_w": round(self._det_w, 4),
                "det_h": round(self._det_h, 4),
                "det_conf": round(self._det_conf, 3),
                "det_label": self._det_label,
                "recenter_seq": self._recenter_seq,
            }
            return eff_zoom, guidance

    def _reset_locked(self, new_tid: Optional[str], kind: str, now: float):
        self._target_id = new_tid
        self._target_kind = kind
        self._target_started_ts = now
        self._phase = "acquiring" if new_tid else "idle"
        self._locked = False
        self._hit_streak = 0
        self._miss_streak = 0
        self._last_lock_ts = 0.0
        self._last_opt_ts = 0.0
        self._bias_pan_hw = 0.0
        self._bias_tilt_hw = 0.0
        self._bias_pan_dps = 0.0
        self._bias_tilt_dps = 0.0
        self._prev_raw_bias = None
        self._det_u = 0.5
        self._det_v = 0.5
        self._det_w = 0.0
        self._det_h = 0.0
        self._det_conf = 0.0
        self._det_label = ""
        self._det_source = ""
        self._obj_span_w_deg = 0.0
        self._obj_span_h_deg = 0.0
        self._last_recenter_ts = 0.0

    def get_status(self) -> dict:
        with self.lock:
            return {
                "active_target_id": self._target_id,
                "phase": self._phase,
                "locked": self._locked,
                "model_ready": self._model_ready,
                "u": round(self._det_u, 4),
                "v": round(self._det_v, 4),
                "w": round(self._det_w, 4),
                "h": round(self._det_h, 4),
                "conf": round(self._det_conf, 3),
                "label": self._det_label,
                "source": self._det_source,
                "infer_ms": round(self._infer_ms, 1),
                "bias_pan_deg": round(self._bias_pan_hw, 3),
                "bias_tilt_deg": round(self._bias_tilt_hw, 3),
                "bias_pan_dps": round(self._bias_pan_dps, 3),
                "bias_tilt_dps": round(self._bias_tilt_dps, 3),
                "zoom_cap_x": round(self._cur_zoom_cap, 2),
            }

    def _worker_loop(self):
        last_seen_frame_ts = 0.0
        last_rpc_tid: Optional[str] = None
        prev_cam_pose: Optional[Tuple[float, float, float]] = None

        while True:
            with self.lock:
                tid = self._target_id
                kind = self._target_kind

            if (
                not tid
                or self.director.calibrating
                or self.director.stacking
                or getattr(self.director, "_focus_trimming", False)
            ):
                if last_rpc_tid is not None:
                    self._rpc({"cmd": "reset", "target_id": ""})
                    last_rpc_tid = None
                self._wake_event.wait(timeout=0.25)
                self._wake_event.clear()
                continue

            drv = self.director.hw_driver
            # Only run on physical PTZ drivers with OBS PTZ capture available
            if drv is None or not hasattr(drv, "get_estimated_hw_pose"):
                time.sleep(0.25)
                continue

            if tid != last_rpc_tid:
                self._rpc({"cmd": "reset", "target_id": tid, "kind": kind})
                last_rpc_tid = tid
                last_seen_frame_ts = 0.0
                prev_cam_pose = None

            t_loop = time.time()

            # Wait only for fast pan/tilt acquisition slews to finish (optical zoom does not motion-blur)
            slew_rem = getattr(drv, "slew_until_ts", 0.0) - t_loop
            if slew_rem > 0.0:
                with self.lock:
                    if self._target_id == tid and not self._locked:
                        self._phase = "acquiring"
                time.sleep(min(0.12, max(0.03, slew_rem)))
                continue

            # Grab a fresh 720p PTZ frame from OBSCameraBridge (~8 Hz)
            jpeg, frame_ts = self.director.obs_cam.get_ptz_frame(demand=True, wait_fresh=0.22)
            if not jpeg or frame_ts <= last_seen_frame_ts or (time.time() - frame_ts) > 1.2:
                time.sleep(0.06)
                continue
            last_seen_frame_ts = frame_ts

            # Optical exposure timestamp (~85 ms upstream of OBS WebSocket screenshot capture)
            t_opt = frame_ts - 0.085
            cam_p_hw, cam_t_hw, cam_hfov, was_slewing = drv.get_estimated_hw_pose(t_opt)
            if was_slewing:
                time.sleep(0.06)
                continue

            # Look up current RF target state from WorldDirector
            with self.director.lock:
                t_obj = (getattr(self.director, "last_targets_by_id", {}) or {}).get(tid)
                t_snap_ts = time.time()
            if not t_obj:
                time.sleep(0.08)
                continue

            # Rewind RF target angle cleanly to `t_opt`
            dt_rf = max(-0.5, min(1.5, t_snap_ts - t_opt))
            rf_p_syn = float(t_obj["pan_deg"]) - float(t_obj.get("pan_dps", 0.0)) * dt_rf
            rf_t_syn = float(t_obj["tilt_deg"]) - float(t_obj.get("tilt_dps", 0.0)) * dt_rf
            dist_km = t_obj.get("dist_km")
            rf_p_hw, rf_t_hw = drv.compute_hw_angles(rf_p_syn, rf_t_syn, zoom_x=15.0, dist_km=dist_km)

            cam_vfov = math.degrees(2.0 * math.atan(math.tan(math.radians(cam_hfov * 0.5)) * (9.0 / 16.0)))
            half_tan_x = max(1e-4, math.tan(math.radians(cam_hfov * 0.5)))
            half_tan_y = max(1e-4, math.tan(math.radians(cam_vfov * 0.5)))

            with self.lock:
                cur_bp = self._bias_pan_hw
                cur_bt = self._bias_tilt_hw
                was_locked = self._locked

            exp_p_hw = rf_p_hw + (cur_bp if was_locked else 0.0)
            exp_t_hw = rf_t_hw + (cur_bt if was_locked else 0.0)
            expect_u = max(0.10, min(0.90, 0.5 + math.tan(math.radians(exp_p_hw - cam_p_hw)) / (2.0 * half_tan_x)))
            expect_v = max(0.10, min(0.90, 0.5 - math.tan(math.radians(exp_t_hw - cam_t_hw)) / (2.0 * half_tan_y)))

            # Reset pixel-space optical-flow tracker if the camera itself slewed or zoomed significantly
            reseed_pixel_tracker = False
            if prev_cam_pose is not None:
                pp_c, pt_c, phf_c = prev_cam_pose
                if (
                    math.hypot(cam_p_hw - pp_c, cam_t_hw - pt_c) > 0.18 * cam_hfov
                    or abs(cam_hfov - phf_c) / max(1.0, cam_hfov) > 0.14
                ):
                    reseed_pixel_tracker = True
            prev_cam_pose = (cam_p_hw, cam_t_hw, cam_hfov)

            resp = self._rpc({
                "cmd": "detect_track",
                "target_id": tid,
                "kind": kind,
                "reseed": reseed_pixel_tracker,
                "expect_u": round(expect_u, 4),
                "expect_v": round(expect_v, 4),
                "jpeg_b64": base64.b64encode(jpeg).decode("ascii"),
            })
            if not resp or not resp.get("ok"):
                time.sleep(0.08)
                continue

            now_done = time.time()
            need_tracker_reset = False
            with self.lock:
                if self._target_id != tid:
                    continue
                self._infer_ms = float(resp.get("ms", 0.0))
                dsrc = str(resp.get("source") or "")
                is_neural_hit = bool(resp.get("found")) and dsrc != "vn_track_coast"

                if is_neural_hit:
                    du = float(resp["u"])
                    dv = float(resp["v"])
                    dw = float(resp["w"])
                    dh = float(resp["h"])
                    dconf = float(resp.get("conf", 0.0))
                    dlbl = str(resp.get("label") or "")

                    # True Inertial Line-of-Sight (LOS) angle of the target in hardware gimbal degrees
                    off_p_deg = math.degrees(math.atan((du - 0.5) * 2.0 * half_tan_x))
                    off_t_deg = math.degrees(math.atan((0.5 - dv) * 2.0 * half_tan_y))
                    meas_los_p = cam_p_hw + off_p_deg
                    meas_los_t = cam_t_hw + off_t_deg

                    # Clamp max RF-to-Visual excursion so a background boat across the harbor never hijacks far away
                    max_bp = 8.5 if kind in ("aircraft", "helicopter") else 6.5
                    max_bt = 6.0 if kind in ("aircraft", "helicopter") else 4.5
                    raw_bp = max(-max_bp, min(max_bp, meas_los_p - rf_p_hw))
                    raw_bt = max(-max_bt, min(max_bt, meas_los_t - rf_t_hw))

                    # Update zoom-invariant physical angular span of the target
                    inst_w_deg = dw * cam_hfov
                    inst_h_deg = dh * cam_vfov
                    if self._obj_span_w_deg <= 0.01:
                        self._obj_span_w_deg = inst_w_deg
                        self._obj_span_h_deg = inst_h_deg
                    else:
                        self._obj_span_w_deg = 0.78 * self._obj_span_w_deg + 0.22 * inst_w_deg
                        self._obj_span_h_deg = 0.78 * self._obj_span_h_deg + 0.22 * inst_h_deg

                    self._det_u = du
                    self._det_v = dv
                    self._det_w = dw
                    self._det_h = dh
                    self._det_conf = dconf
                    self._det_label = dlbl
                    self._det_source = dsrc
                    self._miss_streak = 0

                    if not self._locked:
                        if self._prev_raw_bias is not None:
                            pp_los, pt_los, pp_bp, pp_bt = self._prev_raw_bias
                            if math.hypot(meas_los_p - pp_los, meas_los_t - pt_los) <= 2.4:
                                self._hit_streak += 1
                            else:
                                self._hit_streak = 1
                        else:
                            self._hit_streak = 1

                        if self._hit_streak >= 2:
                            _, _, pp_bp, pp_bt = self._prev_raw_bias or (0.0, 0.0, raw_bp, raw_bt)
                            self._bias_pan_hw = 0.5 * (pp_bp + raw_bp)
                            self._bias_tilt_hw = 0.5 * (pp_bt + raw_bt)
                            self._bias_pan_dps = 0.0
                            self._bias_tilt_dps = 0.0
                            self._locked = True
                            self._phase = "tracking"
                            self._last_lock_ts = now_done
                            self._last_opt_ts = t_opt
                            # Trigger immediate centering refinement if target was off-center at initial/re-acquired lock
                            if math.hypot(du - 0.5, dv - 0.5) >= 0.09:
                                self._recenter_seq += 1
                                self._last_recenter_ts = now_done
                        else:
                            self._phase = "locking"
                        self._prev_raw_bias = (meas_los_p, meas_los_t, raw_bp, raw_bt)
                    else:
                        self._hit_streak += 1
                        dt_f = max(0.04, min(1.2, t_opt - self._last_opt_ts)) if self._last_opt_ts > 0.0 else 0.12
                        pred_bp = self._bias_pan_hw + self._bias_pan_dps * dt_f
                        pred_bt = self._bias_tilt_hw + self._bias_tilt_dps * dt_f
                        res_p = raw_bp - pred_bp
                        res_t = raw_bt - pred_bt

                        # Reject sudden >2.5 deg jump once stably locked
                        if math.hypot(res_p, res_t) <= 2.6:
                            alpha = 0.38 if self._hit_streak <= 5 else 0.28
                            beta = 0.085
                            self._bias_pan_hw = max(-max_bp, min(max_bp, pred_bp + alpha * res_p))
                            self._bias_tilt_hw = max(-max_bt, min(max_bt, pred_bt + alpha * res_t))
                            max_rate = 2.4 if kind in ("aircraft", "helicopter") else 1.1
                            self._bias_pan_dps = max(
                                -max_rate,
                                min(max_rate, 0.90 * self._bias_pan_dps + (beta / dt_f) * res_p),
                            )
                            self._bias_tilt_dps = max(
                                -max_rate,
                                min(max_rate, 0.90 * self._bias_tilt_dps + (beta / dt_f) * res_t),
                            )
                            self._last_lock_ts = now_done
                            self._last_opt_ts = t_opt
                            self._phase = "tracking"

                            # For slow sub-motor-floor vessels (or sudden maneuvers), smoothly re-center
                            # BEFORE the object ever reaches the edge of the frame (`|u-0.5| > 0.28`)!
                            if (
                                (abs(du - 0.5) > 0.28 or abs(dv - 0.5) > 0.30)
                                and (now_done - self._last_recenter_ts) >= 2.2
                            ):
                                self._recenter_seq += 1
                                self._last_recenter_ts = now_done
                else:
                    # Occluded behind a foreground building/pier or momentarily lost:
                    #   - 0..2.2s: Coast smoothly across the obstacle at AIS/ADS-B speed (zeroing bias_dps so
                    #              we never latch onto the stationary building edge!).
                    #   - >2.2s:   Drop visual bias to 0, widen zoom back to `acq_z`, and actively re-seek the
                    #              live AIS/ADS-B position every 3.0s until the boat/plane emerges!
                    self._miss_streak += 1
                    self._bias_pan_dps = 0.0
                    self._bias_tilt_dps = 0.0
                    if self._locked:
                        if (now_done - self._last_lock_ts) <= 2.2:
                            self._phase = "coasting"
                        else:
                            self._locked = False
                            self._hit_streak = 0
                            self._prev_raw_bias = None
                            self._bias_pan_hw = 0.0
                            self._bias_tilt_hw = 0.0
                            self._phase = "reacquiring_rf"
                            self._recenter_seq += 1
                            self._last_recenter_ts = now_done
                            need_tracker_reset = True
                    else:
                        self._hit_streak = 0
                        self._prev_raw_bias = None
                        if self._last_lock_ts > 0.0:
                            self._phase = "reacquiring_rf"
                        if (
                            self._phase == "reacquiring_rf"
                            and (now_done - self._last_recenter_ts) >= 3.0
                        ):
                            self._recenter_seq += 1
                            self._last_recenter_ts = now_done

            if need_tracker_reset:
                self._rpc({"cmd": "reset", "target_id": tid, "kind": kind})

            elapsed = time.time() - t_loop
            time.sleep(max(0.03, 0.12 - elapsed))
