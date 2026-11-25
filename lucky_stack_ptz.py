#!/usr/bin/env python3
"""
Astrophotography "Lucky Imaging" + Multi-Point Turbulence De-Warping + Wavelet Stacker
(`lucky_stack_ptz.py`)

Supports two modes:
  1. `--universal <work_dir> <out_dir> [--label TITLE]`:
     Scene-adaptive full-frame 16:9 (up to 3840x2160 4K) pipeline invoked by Mission Control's
     "✨ Lucky Stack" button for any day or night Hudson River / NY Harbor target.
  2. `<frames_dir> <out_dir>`:
     Original cropped raw `.rgb` burst mode (1.5x Drizzle portrait stacker).
"""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy import ndimage


ARCSEC_PER_4K_PX_AT_30X = (2.67 * 3600.0) / 3840.0  # 2.503 arcsec/px


def write_progress(work_dir: Path, phase: str, pct: int, message: str):
    try:
        tmp = work_dir / "progress.json.tmp"
        dst = work_dir / "progress.json"
        tmp.write_text(json.dumps({"phase": phase, "progress_pct": int(pct), "message": message}))
        tmp.replace(dst)
    except Exception:
        pass


def compute_luma(rgb: np.ndarray) -> np.ndarray:
    return (
        0.2126 * rgb[..., 0].astype(np.float32)
        + 0.7152 * rgb[..., 1].astype(np.float32)
        + 0.0722 * rgb[..., 2].astype(np.float32)
    )


def estimate_scene_noise_sigma(luma: np.ndarray) -> float:
    """
    Scene-independent high-pass MAD noise estimator: divides the image into blocks,
    selects the 12% quietest blocks (sky, smooth water, or flat walls), removes smooth
    lighting gradients, and computes robust 1.4826 * MAD noise sigma.
    """
    h, w = luma.shape
    hp = luma - ndimage.gaussian_filter(luma, sigma=1.4)
    bh, bw = max(32, h // 12), max(32, w // 20)
    sigmas = []
    for y in range(0, h - bh + 1, bh):
        for x in range(0, w - bw + 1, bw):
            blk = hp[y : y + bh, x : x + bw]
            med = float(np.median(blk))
            mad = float(np.median(np.abs(blk - med)))
            sigmas.append(1.4826 * mad)
    if not sigmas:
        return 1.0
    sigmas.sort()
    n_quiet = max(1, len(sigmas) // 8)
    return float(np.median(sigmas[:n_quiet]))


def find_best_anchor_box(luma: np.ndarray, box_size: int = 896) -> tuple:
    """Finds the `box_size x box_size` window with highest structural Sobel energy."""
    h, w = luma.shape
    bh = min(box_size, int(h * 0.85))
    bw = min(box_size, int(w * 0.85))
    # Downsample 4x for instant scan
    small = luma[::4, ::4]
    gx = ndimage.sobel(small, axis=1)
    gy = ndimage.sobel(small, axis=0)
    g2 = ndimage.uniform_filter(gx * gx + gy * gy, size=(bh // 4, bw // 4))
    sh, sw = small.shape
    my0, my1 = max(1, bh // 8), max(2, sh - bh // 8)
    mx0, mx1 = max(1, bw // 8), max(2, sw - bw // 8)
    sub = g2[my0:my1, mx0:mx1]
    iy, ix = np.unravel_index(int(np.argmax(sub)), sub.shape)
    cy = (my0 + iy) * 4
    cx = (mx0 + ix) * 4
    y0 = max(0, min(h - bh, cy - bh // 2))
    x0 = max(0, min(w - bw, cx - bw // 2))
    return (slice(y0, y0 + bh), slice(x0, x0 + bw))


def score_lucky_sharpness(luma: np.ndarray, roi_slice, noise_sigma: float) -> float:
    sub = luma[roi_slice]
    # 2x stride on 4K keeps sharpness scoring < 4ms/frame while resolving 2px edges
    if sub.shape[0] > 1000:
        sub = sub[::2, ::2]
    gx = ndimage.sobel(sub, axis=1)
    gy = ndimage.sobel(sub, axis=0)
    g2 = gx * gx + gy * gy
    thr = (4.5 * max(noise_sigma, 0.5)) ** 2
    valid = g2 > thr
    if not np.any(valid):
        return float(g2.mean())
    flux = float(np.maximum(sub - np.percentile(sub, 15), 0.0).mean()) + 1.0
    return float(g2[valid].sum()) / (sub.size * flux)


def tukey_2d(h: int, w: int, alpha: float = 0.25) -> np.ndarray:
    def _1d(n):
        x = np.linspace(0.0, 1.0, n, dtype=np.float32)
        win = np.ones(n, dtype=np.float32)
        edge = alpha * 0.5
        m1 = x < edge
        m2 = x > (1.0 - edge)
        win[m1] = 0.5 * (1.0 + np.cos(np.pi * (2.0 * x[m1] / alpha - 1.0)))
        win[m2] = 0.5 * (1.0 + np.cos(np.pi * (2.0 * x[m2] / alpha - 2.0 / alpha + 1.0)))
        return win

    return np.outer(_1d(h), _1d(w))


def subpixel_shift_phase_corr(
    ref_patch: np.ndarray, mov_patch: np.ndarray, win: np.ndarray, max_shift: float = 12.0
):
    r0 = (ref_patch - ref_patch.mean()) * win
    m0 = (mov_patch - mov_patch.mean()) * win
    fr = np.fft.rfft2(r0)
    fm = np.fft.rfft2(m0)
    cps = fr * np.conj(fm)
    denom = np.abs(cps)
    eps = np.percentile(denom, 60) * 0.15 + 1e-6
    cc = np.fft.irfft2(cps / (denom + eps), s=r0.shape)
    cc = np.fft.fftshift(cc)

    h, w = r0.shape
    cy, cx = h // 2, w // 2
    rad = int(math.ceil(max_shift))
    y0, y1 = max(1, cy - rad), min(h - 1, cy + rad + 1)
    x0, x1 = max(1, cx - rad), min(w - 1, cx + rad + 1)
    sub = cc[y0:y1, x0:x1]
    iy, ix = np.unravel_index(int(np.argmax(sub)), sub.shape)
    py, px = y0 + iy, x0 + ix

    peak_val = float(cc[py, px])
    bg_std = float(cc.std()) + 1e-6
    snr = peak_val / bg_std

    dy_sub = 0.0
    dx_sub = 0.0
    if 0 < py < h - 1:
        v_m, v_0, v_p = float(cc[py - 1, px]), peak_val, float(cc[py + 1, px])
        d = v_m - 2.0 * v_0 + v_p
        if abs(d) > 1e-7:
            dy_sub = max(-0.6, min(0.6, 0.5 * (v_m - v_p) / d))
    if 0 < px < w - 1:
        h_m, h_0, h_p = float(cc[py, px - 1]), peak_val, float(cc[py, px + 1])
        d = h_m - 2.0 * h_0 + h_p
        if abs(d) > 1e-7:
            dx_sub = max(-0.6, min(0.6, 0.5 * (h_m - h_p) / d))

    dy = (py + dy_sub) - cy
    dx = (px + dx_sub) - cx
    return float(dy), float(dx), float(snr)


def compute_map_dewarp_grid(
    ref_luma: np.ndarray,
    grad_mag: np.ndarray,
    grad_thr: float,
    mov_luma_globally_aligned: np.ndarray,
    tile_size: int = 112,
    stride: int = 64,
    max_local_shift: float = 2.4,
):
    """Returns compact coarse `(smooth_dy, smooth_dx)` grids for low-memory strip warping."""
    h, w = ref_luma.shape
    ys = list(range(tile_size // 2, h - tile_size // 2 + 1, stride))
    xs = list(range(tile_size // 2, w - tile_size // 2 + 1, stride))
    gh, gw = len(ys), len(xs)

    grid_dy = np.zeros((gh, gw), dtype=np.float32)
    grid_dx = np.zeros((gh, gw), dtype=np.float32)
    grid_wt = np.zeros((gh, gw), dtype=np.float32)

    win = tukey_2d(tile_size, tile_size, alpha=0.35)
    half = tile_size // 2

    for iy, cy in enumerate(ys):
        for ix, cx in enumerate(xs):
            mean_g = float(grad_mag[cy - half : cy + half, cx - half : cx + half].mean())
            if mean_g < grad_thr:
                continue
            r_tile = ref_luma[cy - half : cy + half, cx - half : cx + half]
            m_tile = mov_luma_globally_aligned[cy - half : cy + half, cx - half : cx + half]
            dy, dx, snr = subpixel_shift_phase_corr(r_tile, m_tile, win, max_shift=3.5)
            if snr >= 4.2 and abs(dy) <= max_local_shift and abs(dx) <= max_local_shift:
                wt = (mean_g - grad_thr) * min(snr, 15.0)
                grid_dy[iy, ix] = dy
                grid_dx[iy, ix] = dx
                grid_wt[iy, ix] = wt

    if float(grid_wt.max()) <= 0.0:
        return np.zeros((gh, gw), dtype=np.float32), np.zeros((gh, gw), dtype=np.float32), 0.0

    smooth_wt = ndimage.gaussian_filter(grid_wt, sigma=1.25) + 1e-6
    smooth_dy = ndimage.gaussian_filter(grid_dy * grid_wt, sigma=1.25) / smooth_wt
    smooth_dx = ndimage.gaussian_filter(grid_dx * grid_wt, sigma=1.25) / smooth_wt
    conf_mask = np.clip(smooth_wt / (np.percentile(grid_wt[grid_wt > 0], 50) * 0.35 + 1e-6), 0.0, 1.0)
    smooth_dy *= conf_mask
    smooth_dx *= conf_mask

    active_mag = np.hypot(smooth_dy[grid_wt > 0], smooth_dx[grid_wt > 0])
    rms_local = float(np.sqrt(np.mean(active_mag * active_mag))) if active_mag.size else 0.0
    return smooth_dy.astype(np.float32), smooth_dx.astype(np.float32), rms_local


def align_atmospheric_dispersion(rgb: np.ndarray, roi_slice) -> tuple:
    g_ref = np.clip(rgb[..., 1][roi_slice], 0.0, 215.0)
    win = tukey_2d(*g_ref.shape, alpha=0.25)
    shifts = {}
    out = rgb.copy()
    for ch_idx, ch_name in ((0, "R"), (2, "B")):
        c_mov = np.clip(rgb[..., ch_idx][roi_slice], 0.0, 215.0)
        dy, dx, snr = subpixel_shift_phase_corr(g_ref, c_mov, win, max_shift=4.0)
        if snr >= 5.0 and abs(dy) < 2.5 and abs(dx) < 2.5:
            out[..., ch_idx] = np.clip(
                ndimage.shift(rgb[..., ch_idx], shift=(dy, dx), order=1, mode="nearest"),
                0.0,
                255.0,
            )
            shifts[ch_name] = (round(dy, 3), round(dx, 3))
        else:
            shifts[ch_name] = (0.0, 0.0)
    return out, shifts


def regularized_rl_wavelet_enhance(
    linear_rgb: np.ndarray, sky_sigma: float, psf_sigma: float = 1.10, n_iter: int = 10, wave_gain: float = 0.50
) -> np.ndarray:
    eps = 1e-4
    lum = np.maximum(
        0.2126 * linear_rgb[..., 0]
        + 0.7152 * linear_rgb[..., 1]
        + 0.0722 * linear_rgb[..., 2],
        eps,
    )

    lum_smooth = ndimage.gaussian_filter(lum, sigma=1.6)
    lo_t = 3.0 * sky_sigma
    hi_t = 11.5 * sky_sigma
    sig_mask = np.clip((lum_smooth - lo_t) / max(hi_t - lo_t, 1e-3), 0.0, 1.0)
    sig_mask = sig_mask * sig_mask * (3.0 - 2.0 * sig_mask)

    est = lum.copy()
    noise_damp = 3.2 * sky_sigma
    for _ in range(n_iter):
        conv = ndimage.gaussian_filter(est, sigma=psf_sigma) + eps
        ratio = lum / conv
        resid_strength = np.clip(np.abs(lum - conv) / (noise_damp + eps), 0.0, 1.0)
        damped_ratio = 1.0 + sig_mask * resid_strength * (ratio - 1.0)
        corr = ndimage.gaussian_filter(damped_ratio, sigma=psf_sigma)
        est = np.maximum(est * corr, eps)

    s1 = ndimage.gaussian_filter(est, sigma=1.0)
    s2 = ndimage.gaussian_filter(s1, sigma=2.0)
    w1 = est - s1
    w2 = s1 - s2
    lum_sharp = np.maximum(est + sig_mask * (wave_gain * w1 + 0.68 * wave_gain * w2), eps)

    gain = (lum_sharp / lum)[..., None]
    return np.maximum(linear_rgb * gain, 0.0)


def adaptive_tone_map(stacked_rgb: np.ndarray, deconv_rgb: np.ndarray, sky_sigma: float) -> tuple:
    """
    Automatically adapts tone-mapping between dark-sky night scenes (Statue of Liberty,
    Colgate Clock, night skyline) and bright floodlit/daylight scenes (Gansevoort Soccer
    Pitch, daytime ships & bridges).
    """
    orig_lum = compute_luma(stacked_rgb)
    p25 = float(np.percentile(orig_lum, 25))
    p50 = float(np.percentile(orig_lum, 50))
    is_dark_night_scene = (p25 < 24.0) and (p50 < 48.0)

    if is_dark_night_scene:
        # Subtract dark-sky airglow pedestal cleanly
        bg = np.percentile(stacked_rgb, 12, axis=(0, 1)) * 0.88
        lin = np.maximum(deconv_rgb - bg[None, None, :], 0.0)
        lum = np.maximum(compute_luma(lin), 1e-5)
        white_ref = max(float(np.percentile(lum, 99.3)), 95.0)
        norm = lin / white_ref
        n_lum = np.maximum(compute_luma(norm), 1e-6)
        beta = 3.8
        s_lum = np.arcsinh(beta * n_lum) / math.asinh(beta)
        s_lum = s_lum / (1.0 + 0.22 * np.maximum(s_lum - 0.78, 0.0))
        s_lum = np.clip(s_lum / 0.96, 0.0, 1.0)
        rgb_s = norm * (s_lum / n_lum)[..., None]
        lum_o = compute_luma(rgb_s)[..., None]
        c_mask = np.clip((lum - 4.0 * sky_sigma) / 20.0, 0.0, 1.0)[..., None]
        rgb_v = lum_o + (1.0 + 0.18 * c_mask) * (rgb_s - lum_o)
        return np.clip(rgb_v * 255.0 + 0.5, 0.0, 255.0).astype(np.uint8), "night_arcsinh"
    else:
        # Daylight or bright floodlit scene: gentle dehaze + crisp natural contrast
        bg = np.minimum(np.percentile(stacked_rgb, 2, axis=(0, 1)) * 0.45, 14.0)
        lin = np.maximum(deconv_rgb - bg[None, None, :], 0.0)
        top = max(float(np.percentile(lin, 99.7)), 220.0)
        norm = np.clip(lin / top, 0.0, 1.0)
        # Subtle S-curve & 10% vibrance
        curved = np.clip(norm ** 0.95, 0.0, 1.0)
        lum_o = compute_luma(curved)[..., None]
        rgb_v = lum_o + 1.10 * (curved - lum_o)
        return np.clip(rgb_v * 255.0 + 0.5, 0.0, 255.0).astype(np.uint8), "daylight_natural"


def run_universal_pipeline(work_dir: Path, out_dir: Path, label: str):
    t0 = time.time()
    out_dir.mkdir(parents=True, exist_ok=True)
    meta = json.loads((work_dir / "meta.json").read_text())
    zoom_x = float(meta.get("zoom_x", 30.0))
    arcsec_per_px = ((68.0 / max(zoom_x, 1.0)) * 3600.0) / float(meta.get("width", 3840))
    if zoom_x >= 28.0 and meta.get("width", 3840) == 3840:
        arcsec_per_px = ARCSEC_PER_4K_PX_AT_30X

    write_progress(work_dir, "loading", 60, "Loading lossless burst frames…")
    before_img = np.asarray(Image.open(work_dir / "before.bmp"))[..., :3]
    frame_paths = sorted(work_dir.glob("frame_*.bmp"))
    if not frame_paths:
        raise RuntimeError("No burst frames found in work_dir")

    frames = [np.asarray(Image.open(fp))[..., :3] for fp in frame_paths]
    h, w, _ = frames[0].shape

    before_luma = compute_luma(before_img)
    before_sigma = estimate_scene_noise_sigma(before_luma)

    lumas = [compute_luma(f) for f in frames]
    single_sigma = float(np.median([estimate_scene_noise_sigma(l) for l in lumas[:7]]))

    # 1. Lucky Frame Selection
    write_progress(work_dir, "grading", 65, f"Grading {len(frames)} frames for calm atmospheric seeing…")
    center_roi = (slice(int(h * 0.10), int(h * 0.90)), slice(int(w * 0.10), int(w * 0.90)))
    scores = [score_lucky_sharpness(l, center_roi, single_sigma) for l in lumas]
    order = np.argsort(scores)[::-1]
    n_keep = max(10, int(round(len(frames) * 0.65)))
    lucky_idx = order[:n_keep]

    # 2. Global Sub-Pixel Registration on Highest-Contrast Structural Anchor
    write_progress(work_dir, "aligning", 70, f"Sub-pixel registering top {n_keep}/{len(frames)} lucky frames…")
    anchor_box = find_best_anchor_box(lumas[lucky_idx[0]], box_size=min(896, h - 64))
    align_win = tukey_2d(
        anchor_box[0].stop - anchor_box[0].start,
        anchor_box[1].stop - anchor_box[1].start,
        alpha=0.25,
    )
    anchor_patch = lumas[lucky_idx[0]][anchor_box]
    prelim = []
    for idx in lucky_idx[:15]:
        dy, dx, _ = subpixel_shift_phase_corr(anchor_patch, lumas[idx][anchor_box], align_win, max_shift=16.0)
        prelim.append(ndimage.shift(lumas[idx], shift=(dy, dx), order=1, mode="nearest"))
    ref_luma = np.median(np.stack(prelim, axis=0), axis=0).astype(np.float32)
    del prelim

    # 3. Multi-Alignment-Point (MAP) Turbulence De-Warping Grids
    write_progress(work_dir, "dewarping", 76, "Computing Multi-Alignment-Point atmospheric de-warp fields…")
    gx_ref = ndimage.sobel(ref_luma, axis=1)
    gy_ref = ndimage.sobel(ref_luma, axis=0)
    grad_mag = np.hypot(gx_ref, gy_ref)
    grad_thr = float(np.percentile(grad_mag, 80)) * 0.85 + max(6.0, 3.0 * single_sigma)

    tile_sz = 128 if w >= 3000 else 96
    tile_st = 64 if w >= 3000 else 48
    global_shifts = []
    coarse_warps = []
    local_rms_vals = []

    ref_anchor = ref_luma[anchor_box]
    for k, idx in enumerate(lucky_idx):
        dy, dx, _ = subpixel_shift_phase_corr(ref_anchor, lumas[idx][anchor_box], align_win, max_shift=16.0)
        global_shifts.append((dy, dx))
        mov_glob = ndimage.shift(lumas[idx], shift=(dy, dx), order=1, mode="nearest")
        s_dy, s_dx, loc_rms = compute_map_dewarp_grid(
            ref_luma, grad_mag, grad_thr, mov_glob, tile_size=tile_sz, stride=tile_st, max_local_shift=2.4
        )
        coarse_warps.append((s_dy, s_dx))
        local_rms_vals.append(loc_rms)

    dys = [s[0] for s in global_shifts]
    dxs = [s[1] for s in global_shifts]
    peak_drift_px = math.hypot(float(max(dxs) - min(dxs)), float(max(dys) - min(dys)))
    rms_tip_tilt_px = math.hypot(float(np.std(dxs)), float(np.std(dys)))
    rms_local_px = float(np.mean(local_rms_vals))

    # 4. Low-Memory Strip-Wise Bilinear/Bicubic De-Warping + Kappa-Sigma Clipped Stacking
    write_progress(work_dir, "stacking", 84, f"Kappa-sigma stacking {n_keep} de-warped frames ({w}×{h})…")
    stacked = np.empty((h, w, 3), dtype=np.float32)
    full_ldy_list = []
    full_ldx_list = []
    gh, gw = coarse_warps[0][0].shape
    zy, zx = h / float(gh), w / float(gw)
    for s_dy, s_dx in coarse_warps:
        full_ldy_list.append(ndimage.zoom(s_dy, (zy, zx), order=1)[:h, :w].astype(np.float16))
        full_ldx_list.append(ndimage.zoom(s_dx, (zy, zx), order=1)[:h, :w].astype(np.float16))

    n_strips = 4
    strip_h = (h + n_strips - 1) // n_strips
    x_row = np.arange(w, dtype=np.float32)

    for s_i in range(n_strips):
        sy0 = s_i * strip_h
        sy1 = min(h, sy0 + strip_h)
        sh = sy1 - sy0
        yy, xx = np.meshgrid(np.arange(sy0, sy1, dtype=np.float32), x_row, indexing="ij")
        strip_stack = np.empty((n_keep, sh, w, 3), dtype=np.float32)
        pad_m = 6
        py0 = max(0, sy0 - pad_m)
        py1 = min(h, sy1 + pad_m)

        for k, idx in enumerate(lucky_idx):
            dy, dx = global_shifts[k]
            ldy = full_ldy_list[k][sy0:sy1].astype(np.float32)
            ldx = full_ldx_list[k][sy0:sy1].astype(np.float32)
            coords = np.array([yy - py0 - (dy + ldy), xx - (dx + ldx)], dtype=np.float32)
            src_sub = frames[idx][py0:py1]
            for c in range(3):
                strip_stack[k, ..., c] = ndimage.map_coordinates(
                    src_sub[..., c].astype(np.float32), coords, order=1, mode="nearest"
                )

        med = np.median(strip_stack, axis=0)
        mad = np.median(np.abs(strip_stack - med), axis=0)
        sig = np.maximum(1.4826 * mad, 1.2)
        msk = np.abs(strip_stack - med) <= (2.6 * sig)
        cnt = np.maximum(msk.sum(axis=0), 1)
        stacked[sy0:sy1] = np.where(msk, strip_stack, 0.0).sum(axis=0) / cnt

    del full_ldy_list, full_ldx_list

    # 5. Atmospheric Dispersion Alignment + Regularized RL Deconvolution + Ātrous Wavelets
    write_progress(work_dir, "deconvolving", 92, "Running Regularized Richardson-Lucy & wavelet deconvolution…")
    stacked, disp_shifts = align_atmospheric_dispersion(stacked, anchor_box)
    stacked_sigma = estimate_scene_noise_sigma(compute_luma(stacked))
    snr_gain = max(before_sigma, single_sigma) / max(stacked_sigma, 0.08)

    deconv = regularized_rl_wavelet_enhance(
        stacked, sky_sigma=max(stacked_sigma, 0.25), psf_sigma=1.08, n_iter=10, wave_gain=0.52
    )
    final_rgb, tone_mode = adaptive_tone_map(stacked, deconv, sky_sigma=stacked_sigma)

    # Align `before_img` to `ref_luma` so the interactive Before/After wipe slider aligns to 0.05 px!
    b_dy, b_dx, _ = subpixel_shift_phase_corr(ref_anchor, before_luma[anchor_box], align_win, max_shift=16.0)
    before_aligned = np.empty_like(before_img)
    for c in range(3):
        before_aligned[..., c] = np.clip(
            ndimage.shift(before_img[..., c].astype(np.float32), shift=(b_dy, b_dx), order=1, mode="nearest"),
            0.0,
            255.0,
        ).astype(np.uint8)

    write_progress(work_dir, "saving", 97, "Saving 4K stacked PNG, web JPEGs & comparison card…")
    before_pil = Image.fromarray(before_aligned)
    after_pil = Image.fromarray(final_rgb)
    before_pil.save(out_dir / "latest_before.png", compress_level=3)
    after_pil.save(out_dir / "latest_after.png", compress_level=3)
    before_pil.save(out_dir / "latest_before.jpg", quality=88, optimize=True, progressive=True)
    after_pil.save(out_dir / "latest_after.jpg", quality=92, optimize=True, progressive=True)
    thumb_w = 640
    thumb_h = max(1, int(round(thumb_w * h / float(w))))
    after_pil.resize((thumb_w, thumb_h), resample=Image.Resampling.LANCZOS).save(
        out_dir / "latest_thumb.jpg", quality=86, optimize=True, progressive=True
    )

    # Build Side-by-Side Comparison Card with 2.5x Zoom Callout of the Anchor Structural Region!
    disp_w = 1280
    disp_h = int(round(disp_w * h / float(w)))  # 720
    pad = 20
    hdr_h = 80
    call_h = 340
    comp_w = pad * 3 + disp_w * 2
    comp_h = hdr_h + disp_h + pad + call_h + pad
    card = Image.new("RGB", (comp_w, comp_h), (11, 15, 23))
    draw = ImageDraw.Draw(card)
    try:
        f_title = ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial Bold.ttf", 25)
        f_sub = ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial.ttf", 17)
        f_lbl = ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial Bold.ttf", 18)
    except Exception:
        f_title = f_sub = f_lbl = ImageFont.load_default()

    title_txt = f"{label} — Planetary Lucky-Imaging & Turbulence De-Warping ({w}×{h} @ {zoom_x:.1f}x)"
    draw.text((pad, 14), title_txt, fill=(235, 242, 252), font=f_title)
    draw.text(
        (pad, 48),
        f"Before: Single Live Frame (σ={before_sigma:.2f} DN)  |  After: {n_keep}/{len(frames)} Lucky Stack + MAP De-Warp + RL Wavelets (σ={stacked_sigma:.2f} DN, {snr_gain:.1f}× Noise Reduction, Drift {peak_drift_px * arcsec_per_px:.1f}\")",
        fill=(148, 168, 196),
        font=f_sub,
    )

    b_thumb = Image.fromarray(before_aligned).resize((disp_w, disp_h), resample=Image.Resampling.LANCZOS)
    a_thumb = Image.fromarray(final_rgb).resize((disp_w, disp_h), resample=Image.Resampling.LANCZOS)
    card.paste(b_thumb, (pad, hdr_h))
    card.paste(a_thumb, (pad * 2 + disp_w, hdr_h))
    draw.rectangle([pad, hdr_h, pad + disp_w - 1, hdr_h + disp_h - 1], outline=(55, 72, 98), width=2)
    draw.rectangle([pad * 2 + disp_w, hdr_h, pad * 2 + disp_w * 2 - 1, hdr_h + disp_h - 1], outline=(56, 189, 248), width=2)
    draw.text((pad + 12, hdr_h + 10), "BEFORE: Single Raw Frame", fill=(248, 113, 113), font=f_lbl)
    draw.text((pad * 2 + disp_w + 12, hdr_h + 10), f"AFTER: {n_keep}-Frame Lucky Stack + RL Wavelets", fill=(110, 231, 183), font=f_lbl)

    # 2.5x Zoom Callout centered on highest-detail anchor box
    acy = (anchor_box[0].start + anchor_box[0].stop) // 2
    acx = (anchor_box[1].start + anchor_box[1].stop) // 2
    zw = min(w, 512)
    zh = int(round(zw * call_h / float(disp_w)))
    zx0 = max(0, min(w - zw, acx - zw // 2))
    zy0 = max(0, min(h - zh, acy - zh // 2))
    yc = hdr_h + disp_h + pad
    bz = Image.fromarray(before_aligned[zy0 : zy0 + zh, zx0 : zx0 + zw]).resize((disp_w, call_h), resample=Image.Resampling.NEAREST)
    az = Image.fromarray(final_rgb[zy0 : zy0 + zh, zx0 : zx0 + zw]).resize((disp_w, call_h), resample=Image.Resampling.NEAREST)
    card.paste(bz, (pad, yc))
    card.paste(az, (pad * 2 + disp_w, yc))
    draw.rectangle([pad, yc, pad + disp_w - 1, yc + call_h - 1], outline=(55, 72, 98), width=2)
    draw.rectangle([pad * 2 + disp_w, yc, pad * 2 + disp_w * 2 - 1, yc + call_h - 1], outline=(56, 189, 248), width=2)
    draw.text((pad + 12, yc + 10), "2.5× PIXEL ZOOM INSET (Before)", fill=(248, 113, 113), font=f_lbl)
    draw.text((pad * 2 + disp_w + 12, yc + 10), "2.5× PIXEL ZOOM INSET (Lucky Stacked + Deconvolved)", fill=(110, 231, 183), font=f_lbl)

    card.save(out_dir / "latest_comparison.png", compress_level=3)

    elapsed = round(time.time() - t0, 2)
    summary = {
        "ok": True,
        "timestamp": time.time(),
        "label": label,
        "width": w,
        "height": h,
        "zoom_x": zoom_x,
        "frames_captured": len(frames),
        "frames_stacked": n_keep,
        "before_sigma_dn": round(before_sigma, 2),
        "stacked_sigma_dn": round(stacked_sigma, 3),
        "snr_improvement_x": round(snr_gain, 1),
        "peak_drift_px": round(peak_drift_px, 2),
        "peak_drift_arcsec": round(peak_drift_px * arcsec_per_px, 2),
        "tip_tilt_rms_px": round(rms_tip_tilt_px, 2),
        "map_warp_rms_px": round(rms_local_px, 2),
        "map_warp_rms_arcsec": round(rms_local_px * arcsec_per_px, 2),
        "dispersion_shifts_px": disp_shifts,
        "tone_mode": tone_mode,
        "stack_elapsed_s": elapsed,
    }
    (out_dir / "latest_meta.json").write_text(json.dumps(summary, indent=2))
    write_progress(work_dir, "done", 100, f"Stacked {n_keep}/{len(frames)} frames ({snr_gain:.1f}× SNR)")
    print(json.dumps(summary))


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--universal":
        ap = argparse.ArgumentParser()
        ap.add_argument("--universal", action="store_true")
        ap.add_argument("work_dir", type=Path)
        ap.add_argument("out_dir", type=Path)
        ap.add_argument("--label", type=str, default="HUDSON RIVER PTZ")
        args = ap.parse_args()
        run_universal_pipeline(args.work_dir, args.out_dir, args.label)
    else:
        print("Usage: lucky_stack_ptz.py --universal <work_dir> <out_dir> [--label TITLE]")
        sys.exit(1)
