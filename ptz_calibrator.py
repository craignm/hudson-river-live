"""
Automated Multi-Landmark 30x Optical PTZ & Focus Calibrator (`ptz_calibrator.py`)
Pure Python 3.9+ standard library + compiled macOS Accelerate helper (`./ptz-vision`).

Pipeline:
  1. Captures two raw 4K (`3840x2160`) frames 0.55s apart from the Synoptic camera (`Video Capture Device 2`)
     and runs `ptz-vision pick-anchor` across 8 skyline/waterfront zones spanning `-20 deg .. +22 deg` pan
     and `0.38 km .. 4.21 km` range to select guaranteed-stationary 2D architectural corners.
  2. Auto-trims FoMaKo outdoor daylight UVC exposure (`iris-abs`, `exposure-time-abs`, `gain=0`) so sunlit
     building facades never clip to white.
  3. Coarse-to-fine optical lock per landmark:
       • ~4.3x (`zoom-abs=10240`, `HFOV=14.19 deg`) -> `match-syn` against rectified 4K Synoptic patch
       • ~10x  (`zoom-abs=13824`, `HFOV=6.21 deg`)  -> `match-syn` + fine re-centering slew
       • 30x   (`zoom-abs=16384`, `HFOV~2.30 deg`)  -> bracketed Tenengrad optical `focus-abs` sweep +
                                                       sub-pixel `match-zoom` from 10x reference frame
  4. Samples the lens's rear-focus zoom tracking curve across `zoom-abs = 0 .. 16384` on the sharpest
     far landmark so manual focus lock stays razor-sharp at ALL zooms (`1x .. 30x`) without glass hunting.
  5. Fits a 5-DOF 3D SO(3) gimbal alignment model (`yaw`, `pitch`, windowsill `roll`, `pan_scale`, `tilt_scale`)
     via Huber-IRLS Gauss-Newton + lens collimation offset + hyperfocal `1/dist_km` focus slope, and hot-applies
     the result to `WorldDirector` & `calibration.json`.
"""

import itertools
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time
from typing import Dict, List, Optional, Tuple

from ptz_drivers import (
    DEFAULT_FOMAKO_ZOOM_HFOV_TABLE,
    DEFAULT_SONY_A40_ZOOM_HFOV_TABLE,
    SonySRGA40Driver,
    ViscaOverIPDriver,
    project_synoptic_to_ptz_gimbal,
    solar_elevation_deg_utc,
)


# 8 architectural search zones distributed across the Chelsea -> Hudson River 4K Synoptic FOV
LANDMARK_ZONES: List[Dict] = [
    {
        "name": "1 WTC Upper Facade",
        "u0": 0.234,
        "v0": 0.455,
        "du": 0.030,
        "dv": 0.038,
        "dist_km": 3.61,
        "far_ref": True,
    },
    {
        "name": "Whitney Museum",
        "u0": 0.226,
        "v0": 0.670,
        "du": 0.034,
        "dv": 0.034,
        "dist_km": 0.59,
        "far_ref": False,
    },
    {
        "name": "Little Island S Amphitheater",
        "u0": 0.402,
        "v0": 0.816,
        "du": 0.038,
        "dv": 0.030,
        "dist_km": 0.43,
        "far_ref": False,
    },
    {
        "name": "Little Island N Overlook",
        "u0": 0.538,
        "v0": 0.824,
        "du": 0.040,
        "dv": 0.030,
        "dist_km": 0.38,
        "far_ref": False,
    },
    {
        "name": "Statue of Liberty",
        "u0": 0.491,
        "v0": 0.589,
        "du": 0.022,
        "dv": 0.026,
        "dist_km": 6.90,
        "far_ref": True,
    },
    {
        "name": "Colgate Clock",
        "u0": 0.548,
        "v0": 0.597,
        "du": 0.024,
        "dv": 0.024,
        "dist_km": 4.25,
        "far_ref": True,
    },
    {
        "name": "30 Hudson (GS)",
        "u0": 0.565,
        "v0": 0.536,
        "du": 0.028,
        "dv": 0.036,
        "dist_km": 4.21,
        "far_ref": True,
    },
    {
        "name": "99 Hudson JC",
        "u0": 0.610,
        "v0": 0.526,
        "du": 0.026,
        "dv": 0.036,
        "dist_km": 4.13,
        "far_ref": True,
    },
    {
        "name": "Urby Tower JC",
        "u0": 0.665,
        "v0": 0.536,
        "du": 0.028,
        "dv": 0.036,
        "dist_km": 3.54,
        "far_ref": True,
    },
    {
        "name": "Newport / Hoboken Waterfront",
        "u0": 0.802,
        "v0": 0.585,
        "du": 0.052,
        "dv": 0.038,
        "dist_km": 2.40,
        "far_ref": True,
    },
]


def _median(vals: List[float]) -> float:
    if not vals:
        return 0.0
    s = sorted(vals)
    n = len(s)
    mid = n // 2
    if n % 2 == 1:
        return s[mid]
    return 0.5 * (s[mid - 1] + s[mid])


def _solve_linear_5x5(a_mat: List[List[float]], b_vec: List[float]) -> List[float]:
    """Gaussian elimination with partial pivoting for 5x5 normal equations."""
    n = len(b_vec)
    aug = [list(a_mat[i]) + [b_vec[i]] for i in range(n)]
    for col in range(n):
        piv = col
        best = abs(aug[col][col])
        for r in range(col + 1, n):
            if abs(aug[r][col]) > best:
                best = abs(aug[r][col])
                piv = r
        if best < 1e-12:
            continue
        if piv != col:
            aug[col], aug[piv] = aug[piv], aug[col]
        diag = aug[col][col]
        for c in range(col, n + 1):
            aug[col][c] /= diag
        for r in range(n):
            if r == col:
                continue
            factor = aug[r][col]
            if abs(factor) > 0.0:
                for c in range(col, n + 1):
                    aug[r][c] -= factor * aug[col][c]
    return [aug[i][n] for i in range(n)]


def fit_3d_gimbal_alignment(
    observations: List[Dict],
    init_params: Tuple[float, float, float, float, float],
    baseline_cam_m: Tuple[float, float, float],
) -> Dict:
    """
    Fits 3D SO(3) gimbal + stereo windowsill baseline model
    `(yaw_deg, pitch_deg, roll_deg, pan_scale, tilt_scale, bx_m, by_m)`
    to measured 30x/10x `(pan_syn, tilt_syn, dist_km) -> (pan_hw, tilt_hw)` correspondences
    using Huber-IRLS damped Gauss-Newton with automatic outlier rejection.
    """
    yaw, pitch, roll, p_scale, t_scale = init_params
    bx0, by0, bz0 = baseline_cam_m

    def eval_pred(params: List[float], obs: Dict) -> Tuple[float, float]:
        return project_synoptic_to_ptz_gimbal(
            pan_syn_deg=obs["pan_syn_deg"],
            tilt_syn_deg=obs["tilt_syn_deg"],
            yaw_offset_deg=params[0],
            pitch_offset_deg=params[1],
            roll_offset_deg=params[2],
            pan_scale=params[3],
            tilt_scale=params[4],
            zoom_x=30.0,
            dist_km=obs.get("dist_km"),
            baseline_cam_m=(params[5], params[6], bz0),
        )

    def run_irls(active_obs: List[Dict], start_p: List[float], allow_scale_roll: bool = True) -> List[float]:
        params = list(start_p)
        eps_step = [1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-3, 1e-3]
        pan_vals = [float(o["pan_syn_deg"]) for o in active_obs]
        pan_span = (max(pan_vals) - min(pan_vals)) if pan_vals else 0.0
        weak_geom = (not allow_scale_roll) or len(active_obs) < 3 or pan_span < 12.0
        has_near = any(float(o.get("dist_km", 99.0)) < 1.0 for o in active_obs)
        has_far = any(float(o.get("dist_km", 0.0)) > 2.0 for o in active_obs)
        b_sigma = 2.50 if (has_near and has_far and len(active_obs) >= 4) else 0.03
        prior_center = [start_p[0], start_p[1], 0.0, 1.0, 1.0, bx0, by0]
        prior_sigma = [
            50.0,
            50.0,
            3.0 if weak_geom else 12.0,
            0.05 if weak_geom else 0.15,
            0.05 if weak_geom else 0.15,
            b_sigma,
            b_sigma,
        ]
        huber_delta = 0.16  # degrees
        n_dof = 7

        for _ in range(32):
            jt_w_j = [[0.0] * n_dof for _ in range(n_dof)]
            jt_w_r = [0.0] * n_dof
            for i in range(n_dof):
                w_prior = 1.0 / (prior_sigma[i] * prior_sigma[i])
                jt_w_j[i][i] += w_prior
                jt_w_r[i] += w_prior * (prior_center[i] - params[i])

            for obs in active_obs:
                pred_p, pred_t = eval_pred(params, obs)
                rp = obs["pan_hw_deg"] - pred_p
                rt = obs["tilt_hw_deg"] - pred_t
                err_mag = math.hypot(rp, rt)
                w_huber = 1.0 if err_mag <= huber_delta else (huber_delta / err_mag)
                # Weight 30x sub-pixel locks 3.0x higher than 10x-only locks
                w_obs = w_huber * (3.0 if obs.get("lock_stage") == "30x" else 1.0)

                jp = [0.0] * n_dof
                jt = [0.0] * n_dof
                for k in range(n_dof):
                    p_plus = list(params)
                    p_plus[k] += eps_step[k]
                    pp, pt = eval_pred(p_plus, obs)
                    jp[k] = (pp - pred_p) / eps_step[k]
                    jt[k] = (pt - pred_t) / eps_step[k]

                for r_idx in range(n_dof):
                    jt_w_r[r_idx] += w_obs * (jp[r_idx] * rp + jt[r_idx] * rt)
                    for c_idx in range(n_dof):
                        jt_w_j[r_idx][c_idx] += w_obs * (jp[r_idx] * jp[c_idx] + jt[r_idx] * jt[c_idx])

            # Levenberg-Marquardt diagonal damping
            for i in range(n_dof):
                jt_w_j[i][i] += 1e-6
            delta = _solve_linear_5x5(jt_w_j, jt_w_r)
            for i in range(n_dof):
                params[i] += delta[i]
            params[2] = max(-6.0, min(6.0, params[2]))
            params[3] = max(0.92, min(1.12, params[3]))
            params[4] = max(0.92, min(1.12, params[4]))
            params[5] = max(-4.50, min(4.50, params[5]))
            params[6] = max(-2.00, min(2.00, params[6]))
            if max(abs(d) for d in delta) < 1e-6:
                break
        return params

    # Pre-fit RMS under initial parameters
    init_list = [yaw, pitch, roll, p_scale, t_scale, bx0, by0]
    pre_sq = []
    for o in observations:
        pp, pt = eval_pred(init_list, o)
        pre_sq.append((o["pan_hw_deg"] - pp) ** 2 + (o["tilt_hw_deg"] - pt) ** 2)
    pre_rms_deg = math.sqrt(sum(pre_sq) / max(1, len(pre_sq)))

    # Robust median yaw/pitch seed so large camera relocations start near the true offset
    med_yaw = _median([o["pan_hw_deg"] - o["pan_syn_deg"] for o in observations])
    med_pitch = _median([o["tilt_hw_deg"] - o["tilt_syn_deg"] for o in observations])
    seed_list = [med_yaw, med_pitch, 0.0, 1.0, 1.0, bx0, by0]

    # Exact combinatorial consensus (RANSAC over all 3-landmark subsets) so 1-3 nighttime
    # window-grid/water outliers or noisy 10x fallbacks never pull Pass 1 away from true 30x locks.
    candidate_models = [run_irls(observations, seed_list)]
    if len(observations) >= 4:
        for k_sub in (3, 4, 5):
            if len(observations) < k_sub:
                continue
            for sub in itertools.combinations(observations, k_sub):
                ps_sub = [float(x["pan_syn_deg"]) for x in sub]
                if max(ps_sub) - min(ps_sub) < 10.0:
                    continue
                candidate_models.append(run_irls(list(sub), seed_list, allow_scale_roll=True))

    best_inliers: List[Dict] = []
    best_score_tuple = (-1, -1, -999.0)
    best_model = candidate_models[0]
    for cand_p in candidate_models:
        cand_in = []
        cand_30x = 0
        sq_sum = 0.0
        for o in observations:
            pp, pt = eval_pred(cand_p, o)
            err = math.hypot(o["pan_hw_deg"] - pp, o["tilt_hw_deg"] - pt)
            if err <= 0.24:
                cand_in.append(o)
                sq_sum += err * err
                if o.get("lock_stage") == "30x":
                    cand_30x += 1
        rms_neg = -math.sqrt(sq_sum / max(1, len(cand_in))) if cand_in else -999.0
        sc = (len(cand_in), cand_30x, rms_neg)
        if sc > best_score_tuple:
            best_score_tuple = sc
            best_inliers = cand_in
            best_model = cand_p

    inliers = best_inliers
    for o in observations:
        pp, pt = eval_pred(best_model, o)
        o["pass1_err_deg"] = math.hypot(o["pan_hw_deg"] - pp, o["tilt_hw_deg"] - pt)
    if len(inliers) < 3:
        inliers = sorted(observations, key=lambda x: x["pass1_err_deg"])[: max(3, len(observations))]

    p_final = run_irls(inliers, best_model, allow_scale_roll=True)

    inlier_ids = {id(o) for o in inliers}
    post_sq = []
    per_landmark = []
    for o in observations:
        pp, pt = eval_pred(p_final, o)
        ep = o["pan_hw_deg"] - pp
        et = o["tilt_hw_deg"] - pt
        emag = math.hypot(ep, et)
        is_in = id(o) in inlier_ids and emag <= 0.26
        if is_in:
            post_sq.append(ep * ep + et * et)
        per_landmark.append({
            "name": o["name"],
            "lock_stage": o.get("lock_stage", "30x"),
            "dist_km": round(float(o.get("dist_km", 0.0)), 2),
            "pan_syn_deg": round(o["pan_syn_deg"], 3),
            "tilt_syn_deg": round(o["tilt_syn_deg"], 3),
            "pan_hw_deg": round(o["pan_hw_deg"], 3),
            "tilt_hw_deg": round(o["tilt_hw_deg"], 3),
            "residual_pan_deg": round(ep, 4),
            "residual_tilt_deg": round(et, 4),
            "residual_deg": round(emag, 4),
            "focus_30x": o.get("focus_30x"),
            "tenengrad_30x": o.get("tenengrad_30x"),
            "psr": round(float(o.get("psr", 0.0)), 2),
            "score": round(float(o.get("score", 0.0)), 3),
            "inlier": is_in,
        })

    post_rms_deg = math.sqrt(sum(post_sq) / max(1, len(post_sq))) if post_sq else 999.0
    return {
        "yaw_offset_deg": p_final[0],
        "pitch_offset_deg": p_final[1],
        "roll_offset_deg": p_final[2],
        "pan_scale": p_final[3],
        "tilt_scale": p_final[4],
        "baseline_cam_m": (p_final[5], p_final[6], bz0),
        "inlier_count": len(post_sq),
        "total_matched": len(observations),
        "pre_rms_deg": pre_rms_deg,
        "post_rms_deg": post_rms_deg,
        "landmarks": per_landmark,
    }


class PTZAutoCalibrator:
    """Thread-safe background controller for multi-landmark 30x PTZ & optical focus calibration."""

    def __init__(self, director):
        self.director = director
        self.lock = threading.Lock()
        self.running = False
        self.status: Dict = {
            "running": False,
            "phase": "idle",
            "progress_pct": 0,
            "message": "Ready",
            "last_run_ts": 0.0,
            "last_ok": None,
            "pre_rms_deg": None,
            "post_rms_deg": None,
            "inlier_count": 0,
            "landmarks": [],
        }
        # Seed initial status from persisted `calibration.json` report if present
        last_rep = self.director.calibration.get("ptz_last_calibration")
        if isinstance(last_rep, dict):
            self.status.update({
                "last_run_ts": float(last_rep.get("timestamp", 0.0)),
                "last_ok": True,
                "pre_rms_deg": last_rep.get("pre_rms_deg"),
                "post_rms_deg": last_rep.get("post_rms_deg"),
                "inlier_count": last_rep.get("inlier_count", 0),
                "message": last_rep.get("summary", "Calibrated"),
                "landmarks": last_rep.get("landmarks", []),
            })

    def get_status(self) -> Dict:
        with self.lock:
            return dict(self.status)

    def _update_status(self, **kwargs):
        with self.lock:
            self.status.update(kwargs)

    def start_async(self, reason: str = "manual") -> Dict:
        with self.lock:
            if self.running:
                return {"ok": False, "error": "Calibration is already running", "status": dict(self.status)}
            drv = self.director.hw_driver
            if not drv or not isinstance(drv, (ViscaOverIPDriver, SonySRGA40Driver)):
                return {"ok": False, "error": "PTZ hardware driver is not connected"}
            if isinstance(drv, SonySRGA40Driver):
                if not drv.probe_connection(force=True):
                    return {
                        "ok": False,
                        "error": drv.last_error or f"Sony SRG-A40 at {drv.host} is not ready",
                    }
            self.running = True
            cam_label = "Sony SRG-A40" if isinstance(drv, SonySRGA40Driver) else "FoMaKo 30x"
            self.status.update({
                "running": True,
                "phase": "starting",
                "progress_pct": 2,
                "message": f"Starting {cam_label} 30x multi-landmark calibration ({reason})…",
                "landmarks": [],
            })
        t = threading.Thread(target=self._run_worker, args=(reason,), daemon=True)
        t.start()
        return {"ok": True, "started": True, "status": self.get_status()}

    def _find_vision_bin(self) -> Path:
        p = Path(__file__).resolve().parent / "ptz-vision"
        if p.is_file() and os.access(p, os.X_OK):
            return p
        raise FileNotFoundError(f"ptz-vision binary not found at {p}")

    def _run_vision(self, vis_bin: Path, args: List[str]) -> Dict:
        out = subprocess.check_output([str(vis_bin)] + args, text=True, timeout=8.0).strip()
        return json.loads(out)

    def _slew_and_settle(
        self,
        drv,
        pan_hw_deg: float,
        tilt_hw_deg: float,
        prev_pan_deg: Optional[float],
        prev_tilt_deg: Optional[float],
    ) -> Tuple[float, float]:
        cmd_p, cmd_t = drv.move_raw_hardware_deg(pan_hw_deg, tilt_hw_deg)
        if hasattr(drv, "wait_pantilt_settled"):
            drv.wait_pantilt_settled(cmd_p, cmd_t, timeout=4.5)
            return cmd_p, cmd_t
        if drv._has_usb_uvc():
            want_pa = int(round(cmd_p * 3600.0))
            want_ta = int(round(cmd_t * 3600.0))
            t0 = time.time()
            while (time.time() - t0) < 4.5:
                time.sleep(0.15)
                rb = drv.query_usb_uvc("pan-tilt-arcsec")
                if rb and "," in rb:
                    try:
                        cp, ct = [int(x) for x in rb.split(",", 1)]
                        if abs(cp - want_pa) <= 550 and abs(ct - want_ta) <= 550:
                            time.sleep(0.28)
                            return cmd_p, cmd_t
                    except ValueError:
                        pass
            time.sleep(0.25)
            return cmd_p, cmd_t

        if prev_pan_deg is None or prev_tilt_deg is None:
            wait_s = 1.65
        else:
            jump = math.hypot(cmd_p - prev_pan_deg, cmd_t - prev_tilt_deg)
            wait_s = max(0.46, min(2.10, 0.44 + jump / 20.0))
        time.sleep(wait_s)
        return cmd_p, cmd_t

    def _auto_trim_daylight_exposure(
        self,
        drv: ViscaOverIPDriver,
        obs_sess,
        vis_bin: Path,
        tmp_dir: Path,
        cal: dict,
        pan_syn_deg: float = 0.0,
        tilt_syn_deg: float = -2.0,
    ) -> Optional[Dict]:
        """
        Closed-loop highlight-protected outdoor exposure trim using the 3D solar/horizon EV model:
          - Applies feed-forward `AE=1` (`iris-abs=7` f/5.6 sweet spot in sun, `AE=8` at night)
          - Refines `ambient_ev_bias` across up to 4 frames so reference landmarks have zero sky clipping.
        """
        if isinstance(drv, SonySRGA40Driver):
            drv.sync_outdoor_exposure(
                force=True, pan_syn_deg=pan_syn_deg, tilt_syn_deg=tilt_syn_deg, zoom_x=4.33
            )
            return None

        drv.local_ev_trim = 0.0
        drv.sync_outdoor_exposure(
            force=True, pan_syn_deg=pan_syn_deg, tilt_syn_deg=tilt_syn_deg, zoom_x=4.33
        )
        exp_img = str(tmp_dir / "exp_trim.jpg")
        for _ in range(4):
            time.sleep(0.45)
            obs_sess.save_ptz_jpeg(exp_img)
            st = self._run_vision(vis_bin, ["stats", exp_img])
            mean_l = float(st.get("mean", 122.0))
            clipped = float(st.get("clipped_frac", 0.0))
            if not drv.apply_luma_feedback(mean_l, clipped):
                break
        # Transfer half of initial landmark trim into global ambient bias for the calibration run
        drv.ambient_ev_bias = max(-3.5, min(4.5, drv.ambient_ev_bias + 0.5 * drv.local_ev_trim))
        drv.local_ev_trim *= 0.5
        return {
            "iris_abs": drv.day_iris_abs,
            "shutter_abs": drv.day_shutter_abs,
            "ambient_ev_bias": round(drv.ambient_ev_bias, 3),
            "manual_glare": True,
        }

    def _find_sharp_focus_at_zoom(
        self,
        drv: ViscaOverIPDriver,
        obs_sess,
        vis_bin: Path,
        tmp_dir: Path,
        tag: str,
        seed_focus: Optional[int] = None,
        use_af_seed: bool = True,
        half_span: int = 240,
        step_size: int = 80,
        roi_u: float = 0.50,
        roi_v: float = 0.50,
    ) -> Tuple[int, float, str]:
        """
        Finds the diffraction-sharp optical `focus-abs` maximizing center-ROI Tenengrad gradient energy:
          1. Optionally lets FoMaKo's internal zoom-focus table converge for 0.90s (`auto-focus=1` -> `0`)
             and reads `?focus-abs` as the starting neighborhood.
          2. Sweeps `[f_center - half_span .. f_center + half_span]` in manual focus mode, fits a
             parabolic peak to Tenengrad sharpness, and captures the final frame at `best_focus_abs`.
        """
        f_center = seed_focus
        if use_af_seed or f_center is None:
            drv._send_usb_uvc("auto-focus=1")
            time.sleep(0.92)
            drv._send_usb_uvc("auto-focus=0")
            drv.focus_mode = "calibrated_curve"
            q = drv.query_usb_uvc("focus-abs")
            if q is not None:
                try:
                    f_center = int(q)
                except ValueError:
                    pass
        if f_center is None:
            f_center = 2850

        drv._send_usb_uvc("auto-focus=0")
        drv.focus_mode = "calibrated_curve"

        candidates = []
        n_steps = max(1, half_span // max(10, step_size))
        for k in range(-n_steps, n_steps + 1):
            fv = max(120, min(3360, int(round(f_center + k * step_size))))
            if not candidates or candidates[-1] != fv:
                candidates.append(fv)

        scores: List[Tuple[int, float]] = []
        best_idx = 0
        best_ten = -1.0
        roi_args = [f"{roi_u:.3f}", f"{roi_v:.3f}", "0.24"]

        for idx, fv in enumerate(candidates):
            drv.wait_focus_settled(fv)
            time.sleep(0.18)
            img_p = str(tmp_dir / f"foc_{tag}_{fv}.jpg")
            obs_sess.save_ptz_jpeg(img_p)
            st = self._run_vision(vis_bin, ["stats", img_p] + roi_args)
            ten = float(st.get("tenengrad", 0.0))
            scores.append((fv, ten))
            if ten > best_ten:
                best_ten = ten
                best_idx = idx

        # Wide fallback scan tailored to zoom regime (30x tele ~430 vs wide/mid ~2500..3100)
        if best_ten < 0.24 and half_span < 500:
            wide_fvs = (
                (260, 360, 440, 520, 640, 800)
                if f_center < 1100
                else (2480, 2640, 2780, 2920, 3060, 3220)
            )
            for fv in wide_fvs:
                drv.wait_focus_settled(fv)
                time.sleep(0.18)
                img_p = str(tmp_dir / f"foc_wide_{tag}_{fv}.jpg")
                obs_sess.save_ptz_jpeg(img_p)
                st = self._run_vision(vis_bin, ["stats", img_p] + roi_args)
                ten = float(st.get("tenengrad", 0.0))
                scores.append((fv, ten))
                if ten > best_ten:
                    best_ten = ten
                    best_idx = len(scores) - 1

        scores.sort(key=lambda x: x[0])
        best_idx = max(range(len(scores)), key=lambda i: scores[i][1])
        best_f, best_ten = scores[best_idx]

        # Parabolic sub-step focus interpolation when peak is interior
        if 0 < best_idx < len(scores) - 1:
            f_l, t_l = scores[best_idx - 1]
            f_c, t_c = scores[best_idx]
            f_r, t_r = scores[best_idx + 1]
            if (f_c - f_l) > 0 and (f_r - f_c) > 0 and t_c >= t_l and t_c >= t_r:
                # Symmetric 3-point parabola offset
                denom = 2.0 * (t_l - 2.0 * t_c + t_r)
                if abs(denom) > 1e-5:
                    frac = max(-0.5, min(0.5, (t_l - t_r) / denom))
                    step_avg = 0.5 * ((f_c - f_l) + (f_r - f_c))
                    best_f = max(120, min(3360, int(round(f_c + frac * step_avg))))

        drv.wait_focus_settled(best_f)
        time.sleep(0.20)
        final_img = str(tmp_dir / f"sharp_{tag}.jpg")
        obs_sess.save_ptz_jpeg(final_img)
        st_final = self._run_vision(vis_bin, ["stats", final_img] + roi_args)
        final_ten = max(best_ten, float(st_final.get("tenengrad", best_ten)))
        return best_f, final_ten, final_img

    def _run_worker(self, reason: str):
        t0 = time.time()
        tmp_dir = Path(tempfile.mkdtemp(prefix="hudson_ptz_cal_"))
        drv: ViscaOverIPDriver = self.director.hw_driver
        self.director.calibrating = True
        try:
            vis_bin = self._find_vision_bin()
            with self.director.lock:
                cal = json.loads(json.dumps(self.director.calibration))

            syn_hfov = float(cal.get("hfov_deg", 67.957))
            syn_k1 = float(cal.get("k1", -0.06))

            self._update_status(
                phase="synoptic_anchors",
                progress_pct=6,
                message="Capturing 4K Synoptic frames & picking stationary 2D skyline corners…",
            )

            with self.director.obs_cam.open_calibration_session() as obs_sess:
                syn1_path = str(tmp_dir / "syn1.jpg")
                syn2_path = str(tmp_dir / "syn2.jpg")
                obs_sess.save_synoptic_jpeg(syn1_path, width=3840, height=2160, quality=94)
                time.sleep(0.55)
                obs_sess.save_synoptic_jpeg(syn2_path, width=3840, height=2160, quality=94)

                anchors: List[Dict] = []
                for z in LANDMARK_ZONES:
                    res = self._run_vision(
                        vis_bin,
                        [
                            "pick-anchor",
                            syn1_path,
                            syn2_path,
                            f"{z['u0']:.4f}",
                            f"{z['v0']:.4f}",
                            f"{z['du']:.4f}",
                            f"{z['dv']:.4f}",
                            "1.65",
                            f"{syn_hfov:.4f}",
                            f"{syn_k1:.5f}",
                        ],
                    )
                    if res.get("ok"):
                        anchors.append({
                            "name": z["name"],
                            "u": float(res["u"]),
                            "v": float(res["v"]),
                            "pan_syn_deg": float(res["pan_deg"]),
                            "tilt_syn_deg": float(res["tilt_deg"]),
                            "corner_lambda": float(res.get("corner_lambda", 0.0)),
                            "dist_km": float(z["dist_km"]),
                            "far_ref": bool(z.get("far_ref", False)),
                        })

                if len(anchors) < 3:
                    raise RuntimeError(
                        f"Only {len(anchors)} stationary landmarks visible in 4K Synoptic camera (fog/night/obstruction); aborting safely."
                    )

                # Put center/west-of-Hudson far skyline references first (30 Hudson / 99 Hudson / Urby)
                # so morning SE sunrise glare behind Lower Manhattan never washes out the initial seed!
                anchors.sort(key=lambda a: (0 if a["far_ref"] else 1, abs(a["u"] - 0.62)))

                # Live running estimate of bore-sight (updated online after Landmark #1 so even large bumps converge fast!)
                cur_yaw = float(cal.get("ptz_pan_offset_deg", drv.pan_offset_deg))
                cur_pitch = float(cal.get("ptz_tilt_offset_deg", drv.tilt_offset_deg))
                cur_roll = float(cal.get("ptz_roll_offset_deg", drv.roll_offset_deg))
                cur_pscale = float(cal.get("ptz_pan_scale", drv.pan_scale))
                cur_tscale = float(cal.get("ptz_tilt_scale", drv.tilt_scale))
                baseline_m = drv.baseline_cam_m

                # Step 2: Slew to first far skyline landmark at ~4.3x and trim daylight exposure
                first_a = anchors[0]
                self._update_status(
                    phase="exposure_trim",
                    progress_pct=12,
                    message=f"Auto-trimming FoMaKo outdoor exposure on {first_a['name']}…",
                )
                is_sony = isinstance(drv, SonySRGA40Driver)
                z_reg_4x = 10672 if is_sony else 10240
                z_reg_10x = 14688 if is_sony else 13824
                z_reg_30x = drv.max_zoom_reg if is_sony else 16384
                hfov_4x = drv.register_to_hfov_deg(z_reg_4x)
                hfov_10x = drv.register_to_hfov_deg(z_reg_10x)

                # Stage 0 (Sony Pan-Tilt Home Re-Index + 1x Wide Coarse Pose Acquisition):
                # Re-homes optical limit switches (`PanTiltReset=on`) in case the gimbal head was bumped
                # by hand while moving the camera, then locks coarse `(yaw, pitch)` across ±35° at 1x wide!
                if is_sony:
                    self._update_status(
                        phase="coarse_1x",
                        progress_pct=9,
                        message="Re-homing Sony optical limit switches & acquiring 1x coarse pose…",
                    )
                    drv.stop_motion()
                    drv._send_cgi("ptzf.cgi", "PanTiltReset=on", timeout=3.0)
                    time.sleep(1.4)
                    for _ in range(45):
                        time.sleep(0.35)
                        kv_rst = drv.query_ptzf() or {}
                        pst_rst = kv_rst.get("PtzfStatus", "")
                        if pst_rst.startswith("idle,idle"):
                            break
                    time.sleep(0.4)
                    drv._send_cgi("ptzf.cgi", "FocusNearLimit=9000&FocusMode=manual")
                    drv.focus_mode = "manual_lock"
                    seed_p = max(-60.0, min(60.0, cur_yaw))
                    prev_cmd_p, prev_cmd_t = self._slew_and_settle(drv, seed_p, 0.0, None, None)
                    drv.wait_zoom_settled(0)
                    drv.wait_focus_settled(drv.compute_sony_focus_reg_for_zoom_and_dist(0, first_a["dist_km"]))
                    time.sleep(0.45)
                    img_1x = str(tmp_dir / "stage0_1x.jpg")
                    obs_sess.save_ptz_jpeg(img_1x)
                    hfov_1x = drv.register_to_hfov_deg(0)
                    s0_dyaws: List[float] = []
                    s0_dpitch: List[float] = []
                    for probe_a in anchors:
                        if not probe_a.get("far_ref"):
                            continue
                        m1 = self._run_vision(
                            vis_bin,
                            [
                                "match-syn",
                                syn1_path,
                                img_1x,
                                f"{probe_a['pan_syn_deg']:.4f}",
                                f"{probe_a['tilt_syn_deg']:.4f}",
                                f"{syn_hfov:.4f}",
                                f"{syn_k1:.5f}",
                                f"{hfov_1x:.2f}",
                                "14.0",
                            ],
                        )
                        if m1.get("ok") and float(m1.get("psr", 0.0)) >= 5.5 and float(m1.get("score", 0.0)) >= 0.15:
                            meas_hw_p = prev_cmd_p + float(m1["delta_pan_deg"])
                            meas_hw_t = prev_cmd_t + float(m1["delta_tilt_deg"])
                            pred_1p, pred_1t = project_synoptic_to_ptz_gimbal(
                                probe_a["pan_syn_deg"],
                                probe_a["tilt_syn_deg"],
                                cur_yaw,
                                cur_pitch,
                                cur_roll,
                                cur_pscale,
                                cur_tscale,
                                zoom_x=1.0,
                                dist_km=probe_a["dist_km"],
                                baseline_cam_m=baseline_m,
                            )
                            s0_dyaws.append(meas_hw_p - pred_1p)
                            s0_dpitch.append(meas_hw_t - pred_1t)
                    if s0_dyaws:
                        cur_yaw += _median(s0_dyaws)
                        cur_pitch += _median(s0_dpitch)
                        print(
                            f"[PTZ Calibrator] Stage 0 1x consensus ({len(s0_dyaws)} anchors): "
                            f"coarse yaw={cur_yaw:+.2f}°, pitch={cur_pitch:+.2f}°",
                            flush=True,
                        )

                p0, t0_ang = project_synoptic_to_ptz_gimbal(
                    first_a["pan_syn_deg"],
                    first_a["tilt_syn_deg"],
                    cur_yaw,
                    cur_pitch,
                    cur_roll,
                    cur_pscale,
                    cur_tscale,
                    zoom_x=4.33,
                    dist_km=first_a["dist_km"],
                    baseline_cam_m=baseline_m,
                )
                prev_cmd_p, prev_cmd_t = self._slew_and_settle(drv, p0, t0_ang, None, None)
                drv.wait_zoom_settled(z_reg_4x)
                if is_sony:
                    drv.wait_focus_settled(drv.compute_sony_focus_reg_for_zoom_and_dist(z_reg_4x, first_a["dist_km"]))
                    time.sleep(0.35)
                else:
                    drv.wait_focus_settled(2850)
                day_exp_result = self._auto_trim_daylight_exposure(
                    drv,
                    obs_sess,
                    vis_bin,
                    tmp_dir,
                    cal,
                    pan_syn_deg=first_a["pan_syn_deg"],
                    tilt_syn_deg=first_a["tilt_syn_deg"],
                )

                # Seed initial focus registers around through-window infinity
                if is_sony:
                    f_10240 = 0x2000
                else:
                    f_10240, _, _ = self._find_sharp_focus_at_zoom(
                        drv, obs_sess, vis_bin, tmp_dir, "seed4x", seed_focus=2850, use_af_seed=False, half_span=160, step_size=80
                    )
                f_13824: Optional[int] = 0x2000 if is_sony else None
                f_16384_far_seed: Optional[int] = 0x1800 if is_sony else None
                cur_zoom_reg = z_reg_4x

                matched_obs: List[Dict] = []
                collimation_dp: List[float] = []
                collimation_dt: List[float] = []
                measured_30x_hfovs: List[float] = []
                best_far_landmark: Optional[Dict] = None

                # Step 3: Coarse (~4x) -> Medium (~10x) -> Fine (30x + Focus Sweep) across all anchors
                n_anc = len(anchors)
                for idx, anc in enumerate(anchors):
                    pct_base = 16 + int(round((idx / float(n_anc)) * 66))
                    nm = anc["name"]
                    self._update_status(
                        phase="matching_landmarks",
                        progress_pct=pct_base,
                        message=f"[{idx + 1}/{n_anc}] {nm}: ~4x coarse lock…",
                    )

                    # --- Stage 1: Coarse ~4x ---
                    pred_p, pred_t = project_synoptic_to_ptz_gimbal(
                        anc["pan_syn_deg"],
                        anc["tilt_syn_deg"],
                        cur_yaw,
                        cur_pitch,
                        cur_roll,
                        cur_pscale,
                        cur_tscale,
                        zoom_x=4.33,
                        dist_km=anc["dist_km"],
                        baseline_cam_m=baseline_m,
                    )
                    drv.sync_outdoor_exposure(
                        force=False,
                        pan_syn_deg=anc["pan_syn_deg"],
                        tilt_syn_deg=anc["tilt_syn_deg"],
                        zoom_x=4.33,
                        allow_before_zoom=True,
                    )
                    prev_cmd_p, prev_cmd_t = self._slew_and_settle(drv, pred_p, pred_t, prev_cmd_p, prev_cmd_t)
                    if cur_zoom_reg != z_reg_4x:
                        drv.wait_zoom_settled(z_reg_4x)
                        cur_zoom_reg = z_reg_4x
                    if is_sony:
                        drv.wait_focus_settled(drv.compute_sony_focus_reg_for_zoom_and_dist(z_reg_4x, anc["dist_km"]))
                    else:
                        drv.wait_focus_settled(f_10240)
                    time.sleep(0.25)

                    img_4x = str(tmp_dir / f"lm{idx}_4x.jpg")
                    obs_sess.save_ptz_jpeg(img_4x)
                    m4 = self._run_vision(
                        vis_bin,
                        [
                            "match-syn",
                            syn1_path,
                            img_4x,
                            f"{anc['pan_syn_deg']:.4f}",
                            f"{anc['tilt_syn_deg']:.4f}",
                            f"{syn_hfov:.4f}",
                            f"{syn_k1:.5f}",
                            f"{hfov_4x:.2f}",
                            "4.6",
                        ],
                    )
                    if not m4.get("ok") or float(m4.get("psr", 0.0)) < 5.5 or float(m4.get("score", 0.0)) < 0.14:
                        print(f"[PTZ Calibrator] Skipping {nm} at ~4x: {m4}", flush=True)
                        continue

                    pan_4x = prev_cmd_p + float(m4["delta_pan_deg"])
                    tilt_4x = prev_cmd_t + float(m4["delta_tilt_deg"])

                    # --- Stage 2: Medium ~10x ---
                    self._update_status(
                        progress_pct=pct_base + 2,
                        message=f"[{idx + 1}/{n_anc}] {nm}: 10x medium lock & centering…",
                    )
                    prev_cmd_p, prev_cmd_t = self._slew_and_settle(drv, pan_4x, tilt_4x, prev_cmd_p, prev_cmd_t)
                    drv.wait_zoom_settled(z_reg_10x)
                    cur_zoom_reg = z_reg_10x
                    if f_13824 is None:
                        f_13824, _, img_10x = self._find_sharp_focus_at_zoom(
                            drv, obs_sess, vis_bin, tmp_dir, f"lm{idx}_10x", seed_focus=2885, use_af_seed=False, half_span=180, step_size=90
                        )
                    else:
                        if is_sony:
                            drv.wait_focus_settled(
                                drv.compute_sony_focus_reg_for_zoom_and_dist(z_reg_10x, anc["dist_km"])
                            )
                        else:
                            drv.wait_focus_settled(f_13824)
                        time.sleep(0.28 if is_sony else 0.20)
                        img_10x = str(tmp_dir / f"lm{idx}_10x.jpg")
                        obs_sess.save_ptz_jpeg(img_10x)

                    m10 = self._run_vision(
                        vis_bin,
                        [
                            "match-syn",
                            syn1_path,
                            img_10x,
                            f"{anc['pan_syn_deg']:.4f}",
                            f"{anc['tilt_syn_deg']:.4f}",
                            f"{syn_hfov:.4f}",
                            f"{syn_k1:.5f}",
                            f"{hfov_10x:.2f}",
                            "2.40",
                        ],
                    )
                    if not m10.get("ok") or float(m10.get("psr", 0.0)) < 6.0 or float(m10.get("score", 0.0)) < 0.16:
                        print(f"[PTZ Calibrator] Skipping {nm} at 10x: {m10}", flush=True)
                        continue

                    pan_10x_exact = prev_cmd_p + float(m10["delta_pan_deg"])
                    tilt_10x_exact = prev_cmd_t + float(m10["delta_tilt_deg"])

                    # Re-center at 10x if > 0.16 deg off-axis so the landmark sits cleanly inside the 30x FOV!
                    if math.hypot(float(m10["delta_pan_deg"]), float(m10["delta_tilt_deg"])) > 0.16:
                        prev_cmd_p, prev_cmd_t = self._slew_and_settle(
                            drv, pan_10x_exact, tilt_10x_exact, prev_cmd_p, prev_cmd_t
                        )
                        img_10x = str(tmp_dir / f"lm{idx}_10x_ctr.jpg")
                        obs_sess.save_ptz_jpeg(img_10x)
                        m10_ctr = self._run_vision(
                            vis_bin,
                            [
                                "match-syn",
                                syn1_path,
                                img_10x,
                                f"{anc['pan_syn_deg']:.4f}",
                                f"{anc['tilt_syn_deg']:.4f}",
                                f"{syn_hfov:.4f}",
                                f"{syn_k1:.5f}",
                                f"{hfov_10x:.2f}",
                                "2.40",
                            ],
                        )
                        if m10_ctr.get("ok") and float(m10_ctr.get("psr", 0.0)) >= 6.0:
                            m10 = m10_ctr
                            pan_10x_exact = prev_cmd_p + float(m10["delta_pan_deg"])
                            tilt_10x_exact = prev_cmd_t + float(m10["delta_tilt_deg"])

                    # Online median bore-sight refinement (clamped to ±2.2° per step so a single outlier never poisons pose)
                    if len(matched_obs) == 0:
                        pred0_p, pred0_t = project_synoptic_to_ptz_gimbal(
                            anc["pan_syn_deg"],
                            anc["tilt_syn_deg"],
                            cur_yaw,
                            cur_pitch,
                            cur_roll,
                            cur_pscale,
                            cur_tscale,
                            zoom_x=10.0,
                            dist_km=anc["dist_km"],
                            baseline_cam_m=baseline_m,
                        )
                        dp0 = pan_10x_exact - pred0_p
                        dt0 = tilt_10x_exact - pred0_t
                        if math.hypot(dp0, dt0) <= 2.4:
                            cur_yaw += dp0
                            cur_pitch += dt0

                    # --- Stage 3: Fine 30x + Focus Lock ---
                    self._update_status(
                        progress_pct=pct_base + 5,
                        message=f"[{idx + 1}/{n_anc}] {nm}: 30x optical lock…",
                    )
                    drv.wait_zoom_settled(z_reg_30x)
                    cur_zoom_reg = z_reg_30x

                    if is_sony:
                        best_f30 = drv.compute_sony_focus_reg_for_zoom_and_dist(z_reg_30x, anc["dist_km"])
                        drv.wait_focus_settled(best_f30)
                        time.sleep(0.35)
                        img_30x = str(tmp_dir / f"lm{idx}_30x.jpg")
                        obs_sess.save_ptz_jpeg(img_30x)
                        st30 = self._run_vision(vis_bin, ["stats", img_30x, "0.500", "0.500", "0.24"])
                        best_ten30 = float(st30.get("tenengrad", 0.0))
                    else:
                        seed_30x = f_16384_far_seed if f_16384_far_seed is not None else 430
                        best_f30, best_ten30, img_30x = self._find_sharp_focus_at_zoom(
                            drv,
                            obs_sess,
                            vis_bin,
                            tmp_dir,
                            f"lm{idx}_30x",
                            seed_focus=seed_30x,
                            use_af_seed=False,
                            half_span=180 if f_16384_far_seed is None else 150,
                            step_size=60 if f_16384_far_seed is None else 50,
                        )
                        if anc["far_ref"] and f_16384_far_seed is None and best_ten30 >= 0.16:
                            f_16384_far_seed = best_f30

                    eff_10x_hfov = float(m10.get("eff_hfov_deg", hfov_10x))
                    exp_30x_hfov = drv.register_to_hfov_deg(z_reg_30x)
                    ratio_est = max(2.0, min(3.4, eff_10x_hfov / max(1.5, exp_30x_hfov)))
                    mz = self._run_vision(
                        vis_bin,
                        [
                            "match-zoom",
                            img_10x,
                            img_30x,
                            f"{float(m10['ptz_u']):.5f}",
                            f"{float(m10['ptz_v']):.5f}",
                            f"{ratio_est:.3f}",
                            f"{eff_10x_hfov / ratio_est:.3f}",
                        ],
                    )

                    if mz.get("ok") and float(mz.get("psr", 0.0)) >= 5.8 and float(mz.get("score", 0.0)) >= 0.24:
                        pan_30x_exact = prev_cmd_p + float(mz["delta_pan_deg"])
                        tilt_30x_exact = prev_cmd_t + float(mz["delta_tilt_deg"])
                        collimation_dp.append(pan_10x_exact - pan_30x_exact)
                        collimation_dt.append(tilt_10x_exact - tilt_30x_exact)
                        eff_30x = float(mz.get("eff_hfov_deg", 2.30))
                        if 1.85 <= eff_30x <= 2.85:
                            measured_30x_hfovs.append(eff_30x)

                        # Visual confirmation nudge if >= 1 stepper increment off-center
                        if math.hypot(float(mz["delta_pan_deg"]), float(mz["delta_tilt_deg"])) >= 0.14:
                            prev_cmd_p, prev_cmd_t = self._slew_and_settle(
                                drv, pan_30x_exact, tilt_30x_exact, prev_cmd_p, prev_cmd_t
                            )
                            obs_sess.save_ptz_jpeg(str(tmp_dir / f"lm{idx}_30x_centered.jpg"))

                        obs_entry = {
                            "name": nm,
                            "lock_stage": "30x",
                            "dist_km": anc["dist_km"],
                            "far_ref": anc["far_ref"],
                            "pan_syn_deg": anc["pan_syn_deg"],
                            "tilt_syn_deg": anc["tilt_syn_deg"],
                            "pan_hw_deg": pan_30x_exact,
                            "tilt_hw_deg": tilt_30x_exact,
                            "focus_30x": best_f30,
                            "tenengrad_30x": round(best_ten30, 4),
                            "psr": float(mz["psr"]),
                            "score": float(mz["score"]),
                        }
                        matched_obs.append(obs_entry)
                        if anc["far_ref"] and (
                            best_far_landmark is None or float(mz["psr"]) > best_far_landmark["psr"]
                        ):
                            best_far_landmark = obs_entry
                        print(
                            f"[PTZ Calibrator] LOCKED 30x {nm}: syn=({anc['pan_syn_deg']:+.3f},{anc['tilt_syn_deg']:+.3f}) "
                            f"-> hw=({pan_30x_exact:+.3f},{tilt_30x_exact:+.3f}) focus={best_f30} "
                            f"ten={best_ten30:.3f} score={mz['score']:.3f} PSR={mz['psr']:.1f}",
                            flush=True,
                        )
                    else:
                        # High-confidence 10x fallback if 30x window patch had low local texture
                        matched_obs.append({
                            "name": nm,
                            "lock_stage": "10x",
                            "dist_km": anc["dist_km"],
                            "far_ref": anc["far_ref"],
                            "pan_syn_deg": anc["pan_syn_deg"],
                            "tilt_syn_deg": anc["tilt_syn_deg"],
                            "pan_hw_deg": pan_10x_exact,
                            "tilt_hw_deg": tilt_10x_exact,
                            "focus_30x": best_f30 if best_ten30 >= 0.25 else None,
                            "tenengrad_30x": round(best_ten30, 4),
                            "psr": float(m10["psr"]),
                            "score": float(m10["score"]),
                        })
                        print(
                            f"[PTZ Calibrator] LOCKED 10x {nm}: syn=({anc['pan_syn_deg']:+.3f},{anc['tilt_syn_deg']:+.3f}) "
                            f"-> hw=({pan_10x_exact:+.3f},{tilt_10x_exact:+.3f}) PSR={m10['psr']:.1f}",
                            flush=True,
                        )

                if len(matched_obs) < 3:
                    raise RuntimeError(
                        f"Only {len(matched_obs)} landmarks achieved high-confidence optical lock (< 3 required)."
                    )

                # Step 4: Sample full rear-focus zoom tracking table on FoMaKo (Sony SRG-A40 has hardware parfocal zoom-focus)
                ref_lm = best_far_landmark or matched_obs[0]
                far_f30_vals = [
                    int(o["focus_30x"])
                    for o in matched_obs
                    if o.get("far_ref") and o.get("focus_30x") is not None
                ]
                f30_inf = int(round(_median(far_f30_vals))) if far_f30_vals else int(ref_lm.get("focus_30x") or (0x2000 if is_sony else 495))

                if is_sony:
                    zoom_focus_samples = [(0, f30_inf), (16384, f30_inf), (z_reg_30x, f30_inf)]
                else:
                    self._update_status(
                        phase="focus_curve",
                        progress_pct=85,
                        message=f"Characterizing rear-focus zoom curve (1x..30x) on {ref_lm['name']}…",
                    )
                    prev_cmd_p, prev_cmd_t = self._slew_and_settle(
                        drv, ref_lm["pan_hw_deg"], ref_lm["tilt_hw_deg"], prev_cmd_p, prev_cmd_t
                    )
                    z_seed_pairs = [
                        (16324, 710),
                        (16185, 1060),
                        (16038, 1330),
                        (15881, 1580),
                        (15712, 1780),
                        (15536, 1970),
                        (15340, 2140),
                        (15128, 2310),
                        (14886, 2450),
                        (14422, 2690),
                        (13824, 2860),
                        (12294, 2980),
                        (10240, 2780),
                        (8208, 2530),
                        (5600, 2270),
                        (3000, 2100),
                        (0, 2000),
                    ]
                    zoom_focus_samples = [(16384, int(f30_inf))]
                    for z_stop, f_seed in z_seed_pairs:
                        if z_stop == 10240 and f_10240 is not None:
                            zoom_focus_samples.append((10240, int(f_10240)))
                            continue
                        if z_stop == 13824 and f_13824 is not None:
                            zoom_focus_samples.append((13824, int(f_10240 if f_13824 is None else f_13824)))
                            continue
                        z_actual = drv.wait_zoom_settled(z_stop)
                        fz, ften, _ = self._find_sharp_focus_at_zoom(
                            drv,
                            obs_sess,
                            vis_bin,
                            tmp_dir,
                            f"zcurve_{z_stop}",
                            seed_focus=f_seed,
                            use_af_seed=False,
                            half_span=100,
                            step_size=50,
                        )
                        zoom_focus_samples.append((int(z_actual), int(fz if ften >= 0.08 else f_seed)))
                    zoom_focus_samples.sort(key=lambda p: p[0])

            # Step 5: Fit 3D SO(3) gimbal model + collimation + near-focus distance slope
            self._update_status(
                phase="solving",
                progress_pct=94,
                message="Solving 3D SO(3) gimbal alignment…",
            )
            coll_pan_10x = _median(collimation_dp) if collimation_dp else 0.0
            coll_tilt_10x = _median(collimation_dt) if collimation_dt else 0.0

            # Convert any 10x-only fallback observations into the 30x collimation frame before 3D fit
            for o in matched_obs:
                if o["lock_stage"] == "10x":
                    o["pan_hw_deg"] -= coll_pan_10x
                    o["tilt_hw_deg"] -= coll_tilt_10x

            init_5 = (
                float(drv.pan_offset_deg),
                float(drv.tilt_offset_deg),
                float(drv.roll_offset_deg),
                float(drv.pan_scale),
                float(drv.tilt_scale),
            )
            fit = fit_3d_gimbal_alignment(matched_obs, init_5, baseline_m)
            if fit["inlier_count"] < 3 or fit["post_rms_deg"] > 0.25:
                raise RuntimeError(
                    f"3D fit rejected by safety gate (inliers={fit['inlier_count']}, RMS={fit['post_rms_deg']:.3f} deg)."
                )

            # Fit 30x hyperfocal near-distance slope: focus_30x = f30_inf + slope * (1/dist_km - 1/4.0)
            foc_pts = [
                (1.0 / max(0.25, float(o["dist_km"])) - 0.25, float(o["focus_30x"]) - float(f30_inf))
                for o in matched_obs
                if o.get("focus_30x") is not None and float(o.get("tenengrad_30x", 0.0)) >= 0.18
            ]
            inv_km_slope = 0.0
            if not is_sony and len(foc_pts) >= 3:
                s_xx = sum(x * x for x, _ in foc_pts)
                s_xy = sum(x * y for x, y in foc_pts)
                if s_xx > 0.25:
                    inv_km_slope = max(-320.0, min(320.0, s_xy / s_xx))

            base_zoom_tbl = DEFAULT_SONY_A40_ZOOM_HFOV_TABLE if is_sony else DEFAULT_FOMAKO_ZOOM_HFOV_TABLE
            zoom_tbl = [list(p) for p in base_zoom_tbl]
            if len(measured_30x_hfovs) >= 2:
                zoom_tbl[-1][1] = round(_median(measured_30x_hfovs), 3)

            elapsed_s = round(time.time() - t0, 1)
            cam_tag = "Sony SRG-A40" if is_sony else "FoMaKo"
            summary_msg = (
                f"[{cam_tag}] Locked {fit['inlier_count']}/{len(anchors)} landmarks at 30x in {elapsed_s}s — "
                f"RMS {fit['post_rms_deg']:.3f}° (was {fit['pre_rms_deg']:.3f}°), "
                f"pan {fit['yaw_offset_deg']:+.2f}°, tilt {fit['pitch_offset_deg']:+.2f}°, "
                f"roll {fit['roll_offset_deg']:+.2f}°"
            )

            az_rad = math.radians(float(cal.get("az_deg", 207.186)))
            el_rad = math.radians(float(cal.get("tilt_deg", 3.171)))
            bx_fit, by_fit, bz_fit = fit["baseline_cam_m"]
            be_fit = (
                bx_fit * math.cos(az_rad)
                - by_fit * math.sin(az_rad) * math.sin(el_rad)
                + bz_fit * math.sin(az_rad) * math.cos(el_rad)
            )
            bn_fit = (
                -bx_fit * math.sin(az_rad)
                - by_fit * math.cos(az_rad) * math.sin(el_rad)
                + bz_fit * math.cos(az_rad) * math.cos(el_rad)
            )
            bu_fit = by_fit * math.cos(el_rad) + bz_fit * math.sin(el_rad)

            cal_updates = {
                "ptz_offset_east_m": round(be_fit, 4),
                "ptz_offset_north_m": round(bn_fit, 4),
                "ptz_offset_up_m": round(bu_fit, 4),
                "ptz_pan_offset_deg": round(fit["yaw_offset_deg"], 4),
                "ptz_tilt_offset_deg": round(fit["pitch_offset_deg"], 4),
                "ptz_roll_offset_deg": round(fit["roll_offset_deg"], 4),
                "ptz_pan_scale": round(fit["pan_scale"], 5),
                "ptz_tilt_scale": round(fit["tilt_scale"], 5),
                "ptz_collimation_pan_10x_deg": round(coll_pan_10x, 4),
                "ptz_collimation_tilt_10x_deg": round(coll_tilt_10x, 4),
                "ptz_zoom_hfov_table": zoom_tbl,
                "ptz_focus_model": {
                    "enabled": not is_sony,
                    "zoom_focus_table": [[z, f] for z, f in zoom_focus_samples],
                    "inv_km_slope_30x": round(inv_km_slope, 2),
                },
                "ptz_last_calibration": {
                    "timestamp": round(time.time(), 2),
                    "camera_model": "sony_a40" if is_sony else "fomako",
                    "reason": reason,
                    "elapsed_s": elapsed_s,
                    "inlier_count": fit["inlier_count"],
                    "total_zones": len(anchors),
                    "pre_rms_deg": round(fit["pre_rms_deg"], 4),
                    "post_rms_deg": round(fit["post_rms_deg"], 4),
                    "summary": summary_msg,
                    "landmarks": fit["landmarks"],
                },
            }
            if day_exp_result is not None:
                cal_updates["ptz_day_exposure"] = day_exp_result

            self.director.save_calibration(cal_updates)
            drv.configure_outdoor_window_defaults()

            self._update_status(
                running=False,
                phase="complete",
                progress_pct=100,
                last_run_ts=time.time(),
                last_ok=True,
                pre_rms_deg=round(fit["pre_rms_deg"], 4),
                post_rms_deg=round(fit["post_rms_deg"], 4),
                inlier_count=fit["inlier_count"],
                message=summary_msg,
                landmarks=fit["landmarks"],
            )
            print(f"[PTZ Calibrator] SUCCESS: {summary_msg}", flush=True)

        except Exception as exc:
            err_msg = f"Calibration aborted safely: {exc}"
            print(f"[PTZ Calibrator] ERROR: {err_msg}", flush=True)
            try:
                drv.configure_outdoor_window_defaults()
            except Exception:
                pass
            self._update_status(
                running=False,
                phase="error",
                progress_pct=0,
                last_run_ts=time.time(),
                last_ok=False,
                message=err_msg,
            )
        finally:
            self.director.calibrating = False
            with self.lock:
                self.running = False
            shutil.rmtree(tmp_dir, ignore_errors=True)
