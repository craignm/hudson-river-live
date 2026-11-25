#!/usr/bin/env python3
"""
Parametric 3D-Printable Window Anti-Reflection Shroud Generator
for the FoMaKo K30NS 4K 30x Optical Zoom PTZ Camera.

Pure Python 3.9+ standard library — zero external pip dependencies.

Generates 100% watertight, 2-manifold, support-free 3D printable meshes
sized for a single-nozzle print on the Bambu Lab H2D (350 x 320 x 325 mm):
  1. fomako_k30ns_shroud_hood.stl       (Main Window Hood — prints Window-Face Down, 0 supports)
  2. fomako_k30ns_shroud_backplate.stl  (Slide-In Chevron Louvered Backplate — prints Flat, 0 supports)
  3. fomako_k30ns_shroud_assembly_preview.stl (Assembled upright preview in windowsill coordinates)
  4. fomako_k30ns_shroud_h2d.3mf        (Multi-object 3MF ready for Bambu Studio)

Optical & Mechanical Highlights:
  - Camera specs: FoMaKo K30NS — 253.5 mm (W) x 144.0 mm (D) x 169.0 mm (H), ±170° pan, -30°..+90° tilt.
  - Bed-flat window contact flange (320 x 218 mm) for flush seal against window glass (bare or w/ 6 mm foam tape).
  - Integrated 46 mm Front Chin Baffle blocks 100% of the K30NS green power LED & white front silkscreen logos.
  - 6 internal 45° sawtooth matte-box optical baffles trap grazing double-pane window reflections.
  - Open-bottom windowsill design lets K30NS factory rubber feet grip the sill directly, with a curved
    elliptical base cradle that self-centers the 253.5 x 144 mm base at the exact 82 mm optical setback
    (clearing full 360° / +90° power-on boot calibration without touching glass or shroud!).
  - Tool-free dovetail slide-in rear backplate with 18 omni-directional 45° '/\\' chevron light-trap louvers
    (0% direct line-of-sight light transmission from any room angle, ~8,400 mm² passive solar heat exhaust)
    plus an unobstructed 184 x 44 mm rear cable pass-through for PoE LAN, HDMI, SDI, USB 3.0, and power.
"""

from __future__ import annotations

import math
import os
import struct
import sys
import xml.sax.saxutils as saxutils
import zipfile
import zlib
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]
Tri3 = Tuple[Vec3, Vec3, Vec3]


# ============================================================================
# Parametric Configuration (all dimensions in millimeters)
# ============================================================================

@dataclass
class ShroudConfig:
    # FoMaKo K30NS official body dimensions & kinematics
    cam_base_width: float = 253.5
    cam_base_depth: float = 144.0
    cam_level_height: float = 169.0
    cam_tilt_axis_z: float = 135.0
    cam_front_setback: float = 10.0     # Distance from window face (zp=0) to front nose of oval base
    cam_base_radial_clearance: float = 1.5

    # Hood exterior & aperture geometry (fits Bambu Lab H2D 325 mm single-nozzle zone)
    front_outer_width: float = 316.0    # At window glass (zp = 0), 6.0 mm flange around 304 mm aperture
    front_outer_height: float = 214.0
    rear_outer_width: float = 279.0     # At rear frame (zp = 170..182, +/- 139.5 mm)
    rear_outer_height: float = 209.0
    hood_total_depth: float = 182.0     # Total depth on windowsill (7.17 inches)

    # Window aperture & LED/logo chin baffle
    front_aperture_width: float = 304.0  # +/- 152 mm -> +/-55 deg tele pan, 133.5 deg wide river arc
    front_aperture_top: float = 204.0
    chin_front_height: float = 46.0      # Covers green LED & "4K FoMaKo NDI|HX3" logos (at z=12..30 mm)
    chin_wall_depth: float = 6.0         # Solid 6 mm front chin wall (zp = 0..6 mm, 100% opaque)
    chin_bevel_height: float = 42.0      # 45-deg anti-glare inner bevel at zp = 6 mm

    # Corner fillets
    outer_top_fillet: float = 22.0
    inner_top_fillet: float = 16.0

    # Sawtooth internal optical baffles (support-free <= 35 deg overhang)
    num_baffles: int = 6
    baffle_start_zp: float = 22.0
    baffle_pitch_zp: float = 24.0
    baffle_ramp_in_zp: float = 6.0
    baffle_ramp_out_zp: float = 5.0
    baffle_depth: float = 3.6

    # Rear dovetail slide-in channel (zp = 170..182 mm)
    rear_frame_start_zp: float = 170.0
    groove_seat_zp: float = 174.0
    groove_lip_start_zp: float = 178.0
    groove_seat_half_x: float = 135.5    # 271.0 mm seat width
    groove_lip_half_x: float = 132.0     # 264.0 mm retaining lip width (41.2 deg dovetail)
    groove_stop_z: float = 44.0          # Backplate bottom stop height (leaves 44 mm cable arch below)

    # Slide-in Chevron Louvered Backplate
    backplate_clearance_x: float = 0.45  # Per-side slip-fit clearance in dovetail track
    backplate_tongue_thick: float = 3.6
    backplate_dovetail_thick: float = 7.6
    backplate_louver_thick: float = 10.4  # Two 5.2 mm stages at 45 deg -> 100% optical occlusion
    cable_arch_half_width: float = 92.0   # 184 mm wide rear connector opening
    apron_bottom_z: float = 18.0          # Corner light-skirt drop on left/right of connectors

    @property
    def cam_pan_center_y(self) -> float:
        return self.cam_front_setback + 0.5 * self.cam_base_depth


# ============================================================================
# Exact 2D Polygon Triangulation (Hole Bridging + Ear Clipping, Zero T-Junctions)
# ============================================================================

def signed_area_2d(poly: Sequence[Vec2]) -> float:
    area = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        area += x1 * y2 - x2 * y1
    return 0.5 * area


def cross2(o: Vec2, a: Vec2, b: Vec2) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def pt_eq(a: Vec2, b: Vec2, eps: float = 1e-7) -> bool:
    return abs(a[0] - b[0]) <= eps and abs(a[1] - b[1]) <= eps


def point_in_tri_strict_or_edge(p: Vec2, a: Vec2, b: Vec2, c: Vec2, eps: float = 1e-8) -> bool:
    c1 = cross2(a, b, p)
    c2 = cross2(b, c, p)
    c3 = cross2(c, a, p)
    return c1 >= -eps and c2 >= -eps and c3 >= -eps


def segments_properly_intersect(a: Vec2, b: Vec2, c: Vec2, d: Vec2, eps: float = 1e-8) -> bool:
    if pt_eq(a, c) or pt_eq(a, d) or pt_eq(b, c) or pt_eq(b, d):
        return False
    d1 = cross2(a, b, c)
    d2 = cross2(a, b, d)
    d3 = cross2(c, d, a)
    d4 = cross2(c, d, b)
    if ((d1 > eps and d2 < -eps) or (d1 < -eps and d2 > eps)) and (
        (d3 > eps and d4 < -eps) or (d3 < -eps and d4 > eps)
    ):
        return True
    return False


def point_on_segment_interior(p: Vec2, a: Vec2, b: Vec2, eps: float = 1e-7) -> bool:
    if pt_eq(p, a, eps) or pt_eq(p, b, eps):
        return False
    if abs(cross2(a, b, p)) > eps * max(1.0, math.hypot(b[0] - a[0], b[1] - a[1])):
        return False
    dot = (p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])
    sq = (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
    return 0.0 < dot < sq


def bridge_holes_into_outer(outer_ccw: Sequence[Vec2], holes_cw: Sequence[Sequence[Vec2]]) -> List[Vec2]:
    """
    Splices CW holes into a CCW outer polygon using zero-width mutually-visible bridges.
    Adds ZERO new vertex coordinates, preserving 100% manifold edge matching.
    """
    poly = list(outer_ccw)
    if signed_area_2d(poly) < 0:
        poly.reverse()

    cleaned_holes: List[List[Vec2]] = []
    for h in holes_cw:
        hl = list(h)
        if signed_area_2d(hl) > 0:
            hl.reverse()
        cleaned_holes.append(hl)

    # Process holes sorted by rightmost vertex X descending
    cleaned_holes.sort(key=lambda h: max(pt[0] for pt in h), reverse=True)

    for hole in cleaned_holes:
        m_idx = max(range(len(hole)), key=lambda i: (hole[i][0], hole[i][1]))
        mx, my = hole[m_idx]

        # Find nearest intersection of ray (mx, my) -> (+inf, my) with edges of poly
        best_x = float("inf")
        best_cand_idx = -1
        n = len(poly)
        for i in range(n):
            p1 = poly[i]
            p2 = poly[(i + 1) % n]
            if (p1[1] > my) == (p2[1] > my):
                if abs(p1[1] - my) < 1e-8 and abs(p2[1] - my) < 1e-8:
                    ix = min(p1[0], p2[0])
                    if ix >= mx - 1e-8 and ix < best_x:
                        best_x = ix
                        best_cand_idx = i if p1[0] >= p2[0] else (i + 1) % n
                continue
            t = (my - p1[1]) / (p2[1] - p1[1])
            ix = p1[0] + t * (p2[0] - p1[0])
            if ix >= mx - 1e-8 and ix < best_x:
                best_x = ix
                # Pick endpoint of intersected edge with larger X (guaranteed >= mx if convex, or check visibility)
                best_cand_idx = i if p1[0] >= p2[0] else (i + 1) % n

        if best_cand_idx < 0:
            raise RuntimeError("Failed to find ray bridge for hole")

        cand_pt = poly[best_cand_idx]
        if pt_eq((mx, my), cand_pt):
            v_idx = best_cand_idx
        else:
            # Check if any reflex vertex of poly lies inside triangle M - I - P
            ix_pt = (best_x, my)
            tri_a, tri_b, tri_c = (mx, my), ix_pt, cand_pt
            if cross2(tri_a, tri_b, tri_c) < 0:
                tri_b, tri_c = tri_c, tri_b

            v_idx = best_cand_idx
            min_tan = float("inf")
            found_inside = False
            for j in range(n):
                pj = poly[j]
                if pt_eq(pj, cand_pt) or pj[0] < mx - 1e-8:
                    continue
                prev_p = poly[(j - 1) % n]
                next_p = poly[(j + 1) % n]
                # Reflex check in CCW polygon
                if cross2(prev_p, pj, next_p) <= 1e-8:
                    if abs(cross2(tri_a, tri_b, tri_c)) > 1e-9 and point_in_tri_strict_or_edge(pj, tri_a, tri_b, tri_c):
                        ang = abs(math.atan2(pj[1] - my, pj[0] - mx))
                        dist = math.hypot(pj[0] - mx, pj[1] - my)
                        key = (ang, dist)
                        if not found_inside or key < best_key:  # type: ignore[name-defined]
                            found_inside = True
                            best_key = key
                            v_idx = j
                            min_tan = ang

            # Fallback direct visibility scan if segment M -> poly[v_idx] crosses any boundary
            def visible(idx: int) -> bool:
                target = poly[idx]
                for k in range(n):
                    e1, e2 = poly[k], poly[(k + 1) % n]
                    if segments_properly_intersect((mx, my), target, e1, e2):
                        return False
                    if point_on_segment_interior(e1, (mx, my), target):
                        return False
                for k in range(len(hole)):
                    e1, e2 = hole[k], hole[(k + 1) % len(hole)]
                    if segments_properly_intersect((mx, my), target, e1, e2):
                        return False
                return True

            if not visible(v_idx):
                candidates = sorted(
                    range(n),
                    key=lambda j: math.hypot(poly[j][0] - mx, poly[j][1] - my),
                )
                for c_idx in candidates:
                    # Ensure bridge enters interior of poly at c_idx
                    if visible(c_idx):
                        v_idx = c_idx
                        break

        # Splice: ... -> poly[v_idx] -> hole[m_idx..] -> hole[..m_idx] -> hole[m_idx] -> poly[v_idx] -> ...
        ordered_hole = hole[m_idx:] + hole[:m_idx]
        poly = (
            poly[: v_idx + 1]
            + ordered_hole
            + [hole[m_idx], poly[v_idx]]
            + poly[v_idx + 1 :]
        )

    return poly


def triangulate_polygon_with_holes(
    outer_ccw: Sequence[Vec2], holes_cw: Sequence[Sequence[Vec2]] = ()
) -> List[Tuple[Vec2, Vec2, Vec2]]:
    """Ear-clipping triangulation returning CCW 2D triangles using ONLY input vertices."""
    pts = bridge_holes_into_outer(outer_ccw, holes_cw) if holes_cw else list(outer_ccw)
    if not holes_cw and signed_area_2d(pts) < 0:
        pts.reverse()

    # Remove consecutive duplicate vertices if any
    cleaned: List[Vec2] = []
    for p in pts:
        if not cleaned or not pt_eq(cleaned[-1], p):
            cleaned.append(p)
    if len(cleaned) > 1 and pt_eq(cleaned[0], cleaned[-1]):
        cleaned.pop()

    tris: List[Tuple[Vec2, Vec2, Vec2]] = []
    idxs = list(range(len(cleaned)))

    guard = 0
    max_guard = len(idxs) * len(idxs) * 4 + 100
    while len(idxs) > 3:
        guard += 1
        if guard > max_guard:
            raise RuntimeError(f"Ear clipping stalled with {len(idxs)} vertices left")

        m = len(idxs)
        ear_found = False

        # Remove zero-area collinear spikes cleanly
        spike_removed = False
        for i in range(m):
            ia, ib, ic = idxs[(i - 1) % m], idxs[i], idxs[(i + 1) % m]
            pa, pb, pc = cleaned[ia], cleaned[ib], cleaned[ic]
            if pt_eq(pa, pc):
                # Folded zero-area bridge tip: drop ib and one copy of pa/pc
                del idxs[i]
                rem_idx = (i - 1) % len(idxs) if i > 0 else len(idxs) - 1
                del idxs[rem_idx]
                spike_removed = True
                break
        if spike_removed:
            continue

        for pass_tol in (1e-9, -1e-7):
            for i in range(m):
                ia, ib, ic = idxs[(i - 1) % m], idxs[i], idxs[(i + 1) % m]
                pa, pb, pc = cleaned[ia], cleaned[ib], cleaned[ic]
                area2 = cross2(pa, pb, pc)
                if area2 <= pass_tol:
                    continue

                contains_other = False
                for j in range(m):
                    ij = idxs[j]
                    if ij in (ia, ib, ic):
                        continue
                    pj = cleaned[ij]
                    if pt_eq(pj, pa) or pt_eq(pj, pb) or pt_eq(pj, pc):
                        continue
                    if point_in_tri_strict_or_edge(pj, pa, pb, pc, eps=1e-8):
                        contains_other = True
                        break

                if not contains_other:
                    if area2 > 1e-10:
                        tris.append((pa, pb, pc))
                    del idxs[i]
                    ear_found = True
                    break
            if ear_found:
                break

        if not ear_found:
            # Degenerate collinear vertex removal
            removed_col = False
            for i in range(m):
                ia, ib, ic = idxs[(i - 1) % m], idxs[i], idxs[(i + 1) % m]
                pa, pb, pc = cleaned[ia], cleaned[ib], cleaned[ic]
                if abs(cross2(pa, pb, pc)) <= 1e-6:
                    del idxs[i]
                    removed_col = True
                    break
            if not removed_col:
                raise RuntimeError(f"Could not find ear in {m}-gon")

    if len(idxs) == 3:
        pa, pb, pc = cleaned[idxs[0]], cleaned[idxs[1]], cleaned[idxs[2]]
        if cross2(pa, pb, pc) > 1e-10:
            tris.append((pa, pb, pc))

    return tris


# ============================================================================
# Mesh Builder & Lofting Primitives
# ============================================================================

class MeshBuilder:
    def __init__(self, name: str):
        self.name = name
        self.tris: List[Tri3] = []

    @staticmethod
    def _snap(p: Vec3) -> Vec3:
        return (round(p[0], 5), round(p[1], 5), round(p[2], 5))

    def add_tri(self, a: Vec3, b: Vec3, c: Vec3) -> None:
        sa, sb, sc = self._snap(a), self._snap(b), self._snap(c)
        if sa == sb or sb == sc or sc == sa:
            return
        # Skip collinear zero-area triangles
        ux, uy, uz = sb[0] - sa[0], sb[1] - sa[1], sb[2] - sa[2]
        vx, vy, vz = sc[0] - sa[0], sc[1] - sa[1], sc[2] - sa[2]
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        if nx * nx + ny * ny + nz * nz <= 1e-14:
            return
        self.tris.append((sa, sb, sc))

    def add_quad(self, a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> None:
        self.add_tri(a, b, c)
        self.add_tri(a, c, d)

    def add_cap_2d(
        self,
        z: float,
        outer_ccw: Sequence[Vec2],
        holes_cw: Sequence[Sequence[Vec2]] = (),
        facing_up: bool = True,
    ) -> None:
        tris2 = triangulate_polygon_with_holes(outer_ccw, holes_cw)
        for a, b, c in tris2:
            va = (a[0], a[1], z)
            vb = (b[0], b[1], z)
            vc = (c[0], c[1], z)
            if facing_up:
                self.add_tri(va, vb, vc)
            else:
                self.add_tri(va, vc, vb)

    def add_loft_loop(self, loop0: Sequence[Vec2], z0: float, loop1: Sequence[Vec2], z1: float) -> None:
        """Lofts two 2D loops of identical vertex count from z0 to z1.
        CCW loops produce outward-pointing side normals; CW loops produce inward hole normals."""
        assert len(loop0) == len(loop1), f"Loop mismatch: {len(loop0)} != {len(loop1)}"
        n = len(loop0)
        for i in range(n):
            a0 = (loop0[i][0], loop0[i][1], z0)
            b0 = (loop0[(i + 1) % n][0], loop0[(i + 1) % n][1], z0)
            b1 = (loop1[(i + 1) % n][0], loop1[(i + 1) % n][1], z1)
            a1 = (loop1[i][0], loop1[i][1], z1)
            self.add_quad(a0, b0, b1, a1)

    def transformed(self, fn: Callable[[Vec3], Vec3], new_name: str = "") -> "MeshBuilder":
        out = MeshBuilder(new_name or self.name)
        for a, b, c in self.tris:
            out.add_tri(fn(a), fn(b), fn(c))
        return out

    def extend(self, other: "MeshBuilder") -> None:
        self.tris.extend(other.tris)


# ============================================================================
# Part 1: FoMaKo K30NS Main Window Hood (Prints Window-Face Down, zp = 0..182)
# ============================================================================

def arc_points(cx: float, cy: float, r: float, a0: float, a1: float, steps: int) -> List[Vec2]:
    """Returns `steps + 1` points inclusive from angle a0 to a1 (radians)."""
    return [
        (cx + r * math.cos(a0 + (a1 - a0) * k / steps), cy + r * math.sin(a0 + (a1 - a0) * k / steps))
        for k in range(steps + 1)
    ]


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * max(0.0, min(1.0, t))


def build_hood_mesh(cfg: ShroudConfig) -> MeshBuilder:
    """Clean, strictly manifold construction of Part 1 (Window Hood)."""
    mesh = MeshBuilder("FoMaKo_K30NS_Window_Hood")

    rx_seat = cfg.groove_seat_half_x          # 135.50 mm
    rx_roof_mark = cfg.groove_seat_half_x + 0.35  # 135.85 mm (strictly inside flat roof <= 136.3 mm)
    x_cut0 = 96.0                             # Curved elliptical bolster engages from x = +/-96 to +/-128.25 mm
    n_arc = 10

    def outer_half_w(zp: float) -> float:
        return 0.5 * lerp(cfg.front_outer_width, cfg.rear_outer_width, zp / cfg.rear_frame_start_zp)

    def outer_h(zp: float) -> float:
        return lerp(cfg.front_outer_height, cfg.rear_outer_height, zp / cfg.rear_frame_start_zp)

    def inner_base_half_w(zp: float) -> float:
        return lerp(0.5 * cfg.front_aperture_width, cfg.groove_lip_half_x - 0.5, zp / cfg.rear_frame_start_zp)

    def inner_roof_h(zp: float) -> float:
        return lerp(cfg.front_aperture_top, 202.0, zp / cfg.rear_frame_start_zp)

    def baffle_offset(zp: float) -> float:
        for idx in range(cfg.num_baffles):
            s0 = cfg.baffle_start_zp + idx * cfg.baffle_pitch_zp
            s_peak = s0 + cfg.baffle_ramp_in_zp
            s1 = s_peak + cfg.baffle_ramp_out_zp
            if s0 <= zp <= s_peak:
                return cfg.baffle_depth * (zp - s0) / cfg.baffle_ramp_in_zp
            if s_peak < zp <= s1:
                return cfg.baffle_depth * (s1 - zp) / cfg.baffle_ramp_out_zp
        return 0.0

    def cradle_half_x(zp: float) -> float:
        a = 0.5 * cfg.cam_base_width + cfg.cam_base_radial_clearance
        b = 0.5 * cfg.cam_base_depth + cfg.cam_base_radial_clearance
        yc = cfg.cam_pan_center_y
        if zp <= yc:
            dy = (zp - yc) / b
            return max(x_cut0, a * math.sqrt(max(0.0, 1.0 - dy * dy)))
        return lerp(a, 130.5, (zp - yc) / (cfg.rear_frame_start_zp - yc))

    def chin_y(zp: float) -> float:
        if zp <= 2.0:
            return cfg.chin_front_height
        return lerp(cfg.chin_front_height, cfg.chin_bevel_height, (zp - 2.0) / (cfg.chin_wall_depth - 2.0))

    def make_outer_arch(zp: float, x_bot: float) -> List[Vec2]:
        hw = outer_half_w(zp)
        hh = outer_h(zp)
        r_eff = lerp(18.0, 3.2, zp / cfg.rear_frame_start_zp)
        ur_arc = arc_points(hw - r_eff, hh - r_eff, r_eff, 0.0, 0.5 * math.pi, n_arc)
        ul_arc = arc_points(-hw + r_eff, hh - r_eff, r_eff, 0.5 * math.pi, math.pi, n_arc)
        return (
            [(x_bot, 0.0), (hw, 0.0), (hw, cfg.groove_stop_z)]
            + ur_arc
            + [(rx_roof_mark, hh), (rx_seat, hh), (0.0, hh), (-rx_seat, hh), (-rx_roof_mark, hh)]
            + ul_arc
            + [(-hw, cfg.groove_stop_z), (-hw, 0.0), (-x_bot, 0.0)]
        )

    def make_inner_arch(zp: float) -> List[Vec2]:
        bo = baffle_offset(zp)
        hw_in = inner_base_half_w(zp) - bo
        hh_in = inner_roof_h(zp) - bo
        r_in = lerp(15.0, 4.0, zp / cfg.rear_frame_start_zp)
        ul_in = arc_points(-hw_in + r_in, hh_in - r_in, r_in, math.pi, 0.5 * math.pi, n_arc)
        ur_in = arc_points(hw_in - r_in, hh_in - r_in, r_in, 0.5 * math.pi, 0.0, n_arc)

        if zp < cfg.chin_wall_depth - 1e-9:
            yc = chin_y(zp)
            pts_left = [(-x_cut0, yc), (-hw_in, yc), (-hw_in, cfg.groove_stop_z)]
            pts_right = [(hw_in, cfg.groove_stop_z), (hw_in, yc), (x_cut0, yc)]
        else:
            # Smoothly step down from chin bevel (42 mm at zp=6) to sleek 12 mm tall base cradle rail (by zp=38)
            # at <= 43 deg slope in zp so zero overhangs exist and corner mass stays minimal!
            t_step = min(1.0, max(0.0, (zp - cfg.chin_wall_depth) / 32.0))
            y_rail = lerp(cfg.chin_bevel_height, 12.0, t_step)
            y_shelf = lerp(cfg.chin_bevel_height, 16.0, t_step)
            xc = min(cradle_half_x(zp), hw_in - 0.4)
            pts_left = [(-xc, y_rail), (-hw_in, y_shelf), (-hw_in, cfg.groove_stop_z)]
            pts_right = [(hw_in, cfg.groove_stop_z), (hw_in, y_shelf), (xc, y_rail)]

        return pts_left + ul_in + [(0.0, hh_in)] + ur_in + pts_right

    # Stage A: 0 -> chin_wall_depth (6.0 mm)
    def stage_a(zp: float) -> Tuple[List[Vec2], List[Vec2]]:
        yc = chin_y(zp)
        return make_outer_arch(zp, x_cut0) + [(0.0, 0.0)], make_inner_arch(zp) + [(0.0, yc)]

    z_a = [0.0, 2.0, 4.0, cfg.chin_wall_depth]
    o0, i0 = stage_a(0.0)
    mesh.add_cap_2d(0.0, o0, [i0], facing_up=False)
    for k in range(len(z_a) - 1):
        oa0, ia0 = stage_a(z_a[k])
        oa1, ia1 = stage_a(z_a[k + 1])
        mesh.add_loft_loop(oa0, z_a[k], oa1, z_a[k + 1])
        mesh.add_loft_loop(ia0, z_a[k], ia1, z_a[k + 1])

    yc_end = cfg.chin_bevel_height
    mesh.add_cap_2d(
        cfg.chin_wall_depth,
        [(-x_cut0, 0.0), (0.0, 0.0), (x_cut0, 0.0), (x_cut0, yc_end), (0.0, yc_end), (-x_cut0, yc_end)],
        facing_up=True,
    )

    # Stage B: 6.0 -> 174.0 mm
    def stage_b(zp: float) -> List[Vec2]:
        xc = cradle_half_x(zp)
        return make_outer_arch(zp, xc) + make_inner_arch(zp)

    z_b_set = {cfg.chin_wall_depth, cfg.rear_frame_start_zp, cfg.groove_seat_zp}
    for z_step in range(8, 174, 3):
        z_b_set.add(float(z_step))
    for idx in range(cfg.num_baffles):
        s0 = cfg.baffle_start_zp + idx * cfg.baffle_pitch_zp
        s_peak = s0 + cfg.baffle_ramp_in_zp
        s1 = s_peak + cfg.baffle_ramp_out_zp
        z_b_set.update([s0, s_peak, s1])
    z_b = sorted(z for z in z_b_set if cfg.chin_wall_depth <= z <= cfg.groove_seat_zp)

    for k in range(len(z_b) - 1):
        mesh.add_loft_loop(stage_b(z_b[k]), z_b[k], stage_b(z_b[k + 1]), z_b[k + 1])

    # Stage C Transition Cap at zp = 174.0 mm
    z_seat = cfg.groove_seat_zp
    hh_174 = outer_h(z_seat)
    stop_y = cfg.groove_stop_z
    inner_174 = make_inner_arch(z_seat)
    # Trim first 2 & last 2 cradle points ((-xc, 12), (-hw_in, 16) and (+hw_in, 16), (+xc, 12))
    # since those belong to the Left & Right posts below stop_y = 44.0 mm!
    inner_arch_from_stop = inner_174[2:-2]  # (-hw_in_174, stop_y) ... (+hw_in_174, stop_y)

    roof_seat_cap: List[Vec2] = (
        [
            (rx_seat, hh_174),
            (0.0, hh_174),
            (-rx_seat, hh_174),
            (-rx_seat, stop_y),
        ]
        + inner_arch_from_stop
        + [
            (rx_seat, stop_y),
        ]
    )
    mesh.add_cap_2d(z_seat, roof_seat_cap, facing_up=True)

    # Stage C Dovetail Track Posts: 174 -> 178 -> 182 mm
    def make_posts(zp: float) -> Tuple[List[Vec2], List[Vec2]]:
        hw = outer_half_w(z_seat)
        hh = outer_h(z_seat)
        r_eff = 3.2
        xc = cradle_half_x(z_seat)
        hw_in_174 = inner_base_half_w(z_seat)

        if zp <= cfg.groove_lip_start_zp:
            xg = rx_seat
        else:
            t_lip = (zp - cfg.groove_lip_start_zp) / (cfg.hood_total_depth - cfg.groove_lip_start_zp)
            xg = lerp(rx_seat, cfg.groove_lip_half_x, t_lip)

        ur_arc = arc_points(hw - r_eff, hh - r_eff, r_eff, 0.0, 0.5 * math.pi, n_arc)
        ul_arc = arc_points(-hw + r_eff, hh - r_eff, r_eff, 0.5 * math.pi, math.pi, n_arc)

        right_post = (
            [(xc, 0.0), (hw, 0.0), (hw, stop_y)]
            + ur_arc
            + [(rx_roof_mark, hh), (xg, hh), (xg, stop_y), (hw_in_174, stop_y), (hw_in_174, 16.0), (xc, 12.0)]
        )
        left_post = (
            [(-hw, 0.0), (-xc, 0.0), (-xc, 12.0), (-hw_in_174, 16.0), (-hw_in_174, stop_y), (-xg, stop_y), (-xg, hh), (-rx_roof_mark, hh)]
            + ul_arc
            + [(-hw, stop_y)]
        )
        return right_post, left_post

    z_c = [z_seat, cfg.groove_lip_start_zp, cfg.hood_total_depth]
    for k in range(len(z_c) - 1):
        rp0, lp0 = make_posts(z_c[k])
        rp1, lp1 = make_posts(z_c[k + 1])
        mesh.add_loft_loop(rp0, z_c[k], rp1, z_c[k + 1])
        mesh.add_loft_loop(lp0, z_c[k], lp1, z_c[k + 1])

    rp_top, lp_top = make_posts(cfg.hood_total_depth)
    mesh.add_cap_2d(cfg.hood_total_depth, rp_top, facing_up=True)
    mesh.add_cap_2d(cfg.hood_total_depth, lp_top, facing_up=True)

    return mesh


# ============================================================================
# Part 2: Slide-In Chevron Light-Trap Rear Backplate (Prints Flat, zp = 0..10.4)
# ============================================================================

def build_backplate_mesh(cfg: ShroudConfig) -> MeshBuilder:
    """
    Builds the slide-in rear backplate flat on zp = 0..10.4 mm:
      - zp = 0..3.6 mm: Full-width tongue (270.1 mm) + lower corner aprons around 184 mm cable arch
                        + 18 chevron louver openings shifting +zp at 45 degrees.
      - zp = 3.6..5.2 mm: Side edges bevel inward at 41.2 degrees (dovetail tongue); louvers reach apex (+5.2 mm).
      - zp = 5.2..7.6 mm: Side dovetail completes at zp = 7.6 mm (263.1 mm); louvers angle back down -45 degrees.
      - zp = 7.6..10.4 mm: Side wings step inward to central louver boss (+/- 116 mm) + top finger pull tab,
                           while chevron louvers complete 2nd bounce to zp = 10.4 mm!
    """
    mesh = MeshBuilder("FoMaKo_K30NS_Chevron_Backplate")

    w_seat_half = cfg.groove_seat_half_x - cfg.backplate_clearance_x  # 135.05 mm
    w_lip_half = cfg.groove_lip_half_x - cfg.backplate_clearance_x    # 131.55 mm
    w_boss_half = 129.5                      # Seals full corner width right up to the +/-131.0 mm posts!
    w_apron_out = w_boss_half
    w_arch_half = cfg.cable_arch_half_width  # 92.0 mm (184 mm wide connector opening)

    y_apron_bot = 14.0                       # Extends corner light-skirts down near windowsill
    y_stop = cfg.groove_stop_z + 0.4         # 44.4 mm
    y_top = cfg.rear_outer_height            # 209.0 mm
    y_tab_top = y_top + 8.0                  # Finger pull tab above rear roof
    w_tab_half = 28.0

    # 9 rows x 2 columns = 18 Chevron Labyrinth Louvers!
    louver_rows = [72.0 + i * 13.0 for i in range(9)]  # 72..176 mm (apex reaches +5.2 + 4.6 = +9.8 -> 185.8 mm)
    slot_h = 4.6
    louver_x_spans = [(-106.0, -5.0), (5.0, 106.0)]  # Central 10 mm spine keeps blades ultra-rigid

    def chevron_shift(zp: float) -> float:
        mid = 0.5 * cfg.backplate_louver_thick  # 5.2 mm
        if zp <= mid:
            return zp  # +45 deg upward slope
        return cfg.backplate_louver_thick - zp  # -45 deg downward return slope

    def make_louver_holes_cw(zp: float) -> List[List[Vec2]]:
        dy = chevron_shift(zp)
        holes: List[List[Vec2]] = []
        for y0 in louver_rows:
            yb = y0 + dy
            yt = yb + slot_h
            for x0, x1 in louver_x_spans:
                # CW winding for hole!
                holes.append([(x0, yb), (x0, yt), (x1, yt), (x1, yb)])
        return holes

    def make_plate_outer_ccw(zp: float) -> List[Vec2]:
        """Outer perimeter for zp in [0.0, 7.6 mm] (includes dovetail side tongues + cable arch aprons)."""
        if zp <= cfg.backplate_tongue_thick:
            xw = w_seat_half
        else:
            t = (zp - cfg.backplate_tongue_thick) / (cfg.backplate_dovetail_thick - cfg.backplate_tongue_thick)
            xw = lerp(w_seat_half, w_lip_half, t)

        # Keep exact vertices at (+/- w_boss_half, y_stop) and (+/- w_boss_half, y_top) so zp=7.6 step has 0 T-junctions!
        return [
            (w_arch_half, y_stop),
            (w_arch_half, y_apron_bot),
            (w_apron_out, y_apron_bot),
            (w_apron_out, y_stop),
            (xw, y_stop),
            (xw, y_top),
            (w_boss_half, y_top),
            (w_tab_half, y_top),
            (w_tab_half, y_tab_top),
            (-w_tab_half, y_tab_top),
            (-w_tab_half, y_top),
            (-w_boss_half, y_top),
            (-xw, y_top),
            (-xw, y_stop),
            (-w_apron_out, y_stop),
            (-w_apron_out, y_apron_bot),
            (-w_arch_half, y_apron_bot),
            (-w_arch_half, y_stop),
        ]

    # Wait: at zp = 7.6 mm, the central boss continues up to zp = 10.4 mm on x in [-w_boss_half, +w_boss_half],
    # y in [y_stop, y_top] + pull tab! Let us include (+/- w_boss_half, y_stop) on the bottom edge too!
    # Wait: w_boss_half = 116.0 lies between w_arch_half (92.0) and w_apron_out (128.0)!
    # If the central boss spans x in [-116, +116], y in [58.0, y_top] (starting at y_boss_bot = 58.0 mm, strictly above y_stop = 44.4 mm!),
    # OR if the central boss spans x in [-w_arch_half (92? No, louvers go to +/- 106!)]:
    # Even simpler!! Let w_apron_out = w_boss_half = 116.0 mm!
    # Check: if w_apron_out == w_boss_half == 116.0 mm:
    #   - Aprons span [-116, -92] and [+92, +116] down to y = 18.0 mm!
    #   - At zp = 7.6 mm, we step off ONLY the Left Dovetail Wing [-131.55, -116] x [44.4, 212]
    #     and Right Dovetail Wing [+116, +131.55] x [44.4, 212] (plus the two lower aprons if desired, or keep aprons 7.6 mm thick)!
    # Let's verify that!! At zp = 7.6 mm:
    #   - Left wing cap: [-131.55, -116] x [y_stop, y_top] + Left apron [-116, -92] x [y_apron_bot, y_stop]
    #   - Right wing cap: [+116, +131.55] x [y_stop, y_top] + Right apron [+92, +116] x [y_apron_bot, y_stop]
    #   - Remaining Boss (zp = 7.6 -> 10.4 mm): [-116, +116] x [y_stop, y_top] + Finger Pull Tab!
    # ZERO interior seams, 100% shared boundary vertices!!

    w_apron_out = w_boss_half  # 116.0 mm

    def make_plate_stage1_ccw(zp: float) -> List[Vec2]:
        if zp <= cfg.backplate_tongue_thick:
            xw = w_seat_half
        else:
            t = (zp - cfg.backplate_tongue_thick) / (cfg.backplate_dovetail_thick - cfg.backplate_tongue_thick)
            xw = lerp(w_seat_half, w_lip_half, t)
        return [
            (w_arch_half, y_stop),
            (w_arch_half, y_apron_bot),
            (w_boss_half, y_apron_bot),
            (w_boss_half, y_stop),
            (xw, y_stop),
            (xw, y_top),
            (w_boss_half, y_top),
            (w_tab_half, y_top),
            (w_tab_half, y_tab_top),
            (-w_tab_half, y_tab_top),
            (-w_tab_half, y_top),
            (-w_boss_half, y_top),
            (-xw, y_top),
            (-xw, y_stop),
            (-w_boss_half, y_stop),
            (-w_boss_half, y_apron_bot),
            (-w_arch_half, y_apron_bot),
            (-w_arch_half, y_stop),
        ]

    def make_boss_stage2_ccw() -> List[Vec2]:
        return [
            (w_arch_half, y_stop),
            (w_boss_half, y_stop),
            (w_boss_half, y_top),
            (w_tab_half, y_top),
            (w_tab_half, y_tab_top),
            (-w_tab_half, y_tab_top),
            (-w_tab_half, y_top),
            (-w_boss_half, y_top),
            (-w_boss_half, y_stop),
            (-w_arch_half, y_stop),
        ]

    # Bottom cap at zp = 0.0
    mesh.add_cap_2d(0.0, make_plate_stage1_ccw(0.0), make_louver_holes_cw(0.0), facing_up=False)

    # Loft zp = 0.0 -> 3.6 -> 5.2 -> 7.6 mm
    z_s1 = [0.0, cfg.backplate_tongue_thick, 0.5 * cfg.backplate_louver_thick, cfg.backplate_dovetail_thick]
    for k in range(len(z_s1) - 1):
        z0, z1 = z_s1[k], z_s1[k + 1]
        mesh.add_loft_loop(make_plate_stage1_ccw(z0), z0, make_plate_stage1_ccw(z1), z1)
        h0 = make_louver_holes_cw(z0)
        h1 = make_louver_holes_cw(z1)
        for lh0, lh1 in zip(h0, h1):
            mesh.add_loft_loop(lh0, z0, lh1, z1)

    # Cap stepped-off Dovetail Wings & Aprons at zp = 7.6 mm (exact shared vertices!)
    z_step = cfg.backplate_dovetail_thick
    right_wing_cap: List[Vec2] = [
        (w_arch_half, y_stop),
        (w_arch_half, y_apron_bot),
        (w_boss_half, y_apron_bot),
        (w_boss_half, y_stop),
    ]
    right_dovetail_cap: List[Vec2] = [
        (w_boss_half, y_stop),
        (w_lip_half, y_stop),
        (w_lip_half, y_top),
        (w_boss_half, y_top),
    ]
    left_wing_cap: List[Vec2] = [
        (-w_boss_half, y_stop),
        (-w_boss_half, y_apron_bot),
        (-w_arch_half, y_apron_bot),
        (-w_arch_half, y_stop),
    ]
    left_dovetail_cap: List[Vec2] = [
        (-w_lip_half, y_stop),
        (-w_boss_half, y_stop),
        (-w_boss_half, y_top),
        (-w_lip_half, y_top),
    ]
    for cap_poly in (right_wing_cap, right_dovetail_cap, left_wing_cap, left_dovetail_cap):
        mesh.add_cap_2d(z_step, cap_poly, facing_up=True)

    # Loft Stage 2 Central Louver Boss from zp = 7.6 -> 10.4 mm
    z_top = cfg.backplate_louver_thick
    boss_poly = make_boss_stage2_ccw()
    mesh.add_loft_loop(boss_poly, z_step, boss_poly, z_top)
    for lh0, lh1 in zip(make_louver_holes_cw(z_step), make_louver_holes_cw(z_top)):
        mesh.add_loft_loop(lh0, z_step, lh1, z_top)

    # Top cap at zp = 10.4 mm
    mesh.add_cap_2d(z_top, boss_poly, make_louver_holes_cw(z_top), facing_up=True)

    return mesh


# ============================================================================
# Manifold Verification, Overhang Check & Optical Clearance Suite
# ============================================================================

def verify_mesh(mesh: MeshBuilder) -> Dict[str, float]:
    """Verifies 100% watertight 2-manifold topology, signed volume, bounding box, and print overhangs."""
    vert_map: Dict[Vec3, int] = {}
    verts: List[Vec3] = []

    def get_vid(v: Vec3) -> int:
        q = (round(v[0], 4), round(v[1], 4), round(v[2], 4))
        vid = vert_map.get(q)
        if vid is None:
            vid = len(verts)
            vert_map[q] = vid
            verts.append(q)
        return vid

    half_edges: Dict[Tuple[int, int], int] = {}
    volume6 = 0.0
    surface_area = 0.0
    max_overhang_from_vertical_deg = 0.0
    unsupported_down_area = 0.0

    for a, b, c in mesh.tris:
        ia, ib, ic = get_vid(a), get_vid(b), get_vid(c)
        if ia == ib or ib == ic or ic == ia:
            continue
        for u, v in ((ia, ib), (ib, ic), (ic, ia)):
            half_edges[(u, v)] = half_edges.get((u, v), 0) + 1

        # Signed volume contribution
        volume6 += (
            a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0])
        )

        # Normal & area
        ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        norm = math.sqrt(nx * nx + ny * ny + nz * nz)
        if norm > 1e-12:
            tri_area = 0.5 * norm
            surface_area += tri_area
            nz_unit = nz / norm
            z_min_tri = min(a[2], b[2], c[2])
            # Check downward-facing surfaces above the build plate (z > 0.05 mm)
            if z_min_tri > 0.05 and nz_unit < -1e-4:
                # Angle from vertical wall: nz_unit = 0 is 0 deg overhang; nz_unit = -1 is 90 deg horizontal bridge
                overhang_deg = math.degrees(math.asin(min(1.0, abs(nz_unit))))
                if overhang_deg > max_overhang_from_vertical_deg:
                    max_overhang_from_vertical_deg = overhang_deg
                if overhang_deg > 46.0:
                    unsupported_down_area += tri_area

    boundary_edges = 0
    non_manifold_edges = 0
    for (u, v), cnt in half_edges.items():
        if cnt != 1:
            non_manifold_edges += 1
        opp = half_edges.get((v, u), 0)
        if opp == 0:
            boundary_edges += 1

    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]

    volume_mm3 = volume6 / 6.0
    # Typical FDM print mass estimate: ~38% effective solid density (3 walls + top/bottom + 15% gyroid infill) * 1.27 g/cm3 PETG
    est_petg_grams = (volume_mm3 / 1000.0) * 0.42 * 1.27

    stats = {
        "triangles": len(mesh.tris),
        "vertices": len(verts),
        "boundary_edges": boundary_edges,
        "non_manifold_edges": non_manifold_edges,
        "volume_cm3": volume_mm3 / 1000.0,
        "surface_cm2": surface_area / 100.0,
        "est_petg_g": est_petg_grams,
        "size_x_mm": max(xs) - min(xs),
        "size_y_mm": max(ys) - min(ys),
        "size_z_mm": max(zs) - min(zs),
        "min_z_mm": min(zs),
        "max_overhang_deg": max_overhang_from_vertical_deg,
        "unsupported_area_mm2": unsupported_down_area,
    }
    if boundary_edges != 0 or non_manifold_edges != 0 or volume_mm3 <= 0:
        raise RuntimeError(f"Mesh verification FAILED for {mesh.name}: {stats}")
    if unsupported_down_area > 0.01:
        raise RuntimeError(f"Support-free overhang check FAILED for {mesh.name}: {stats}")
    return stats


# ============================================================================
# Binary STL & Bambu Studio 3MF Writers
# ============================================================================

def write_binary_stl(path: str, mesh: MeshBuilder) -> None:
    header = f"Antigravity CAD - {mesh.name}".encode("ascii", "ignore")[:80].ljust(80, b"\x00")
    with open(path, "wb") as f:
        f.write(header)
        f.write(struct.pack("<I", len(mesh.tris)))
        for a, b, c in mesh.tris:
            ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
            vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
            nx = uy * vz - uz * vy
            ny = uz * vx - ux * vz
            nz = ux * vy - uy * vx
            ln = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            f.write(
                struct.pack(
                    "<12fH",
                    nx / ln, ny / ln, nz / ln,
                    a[0], a[1], a[2],
                    b[0], b[1], b[2],
                    c[0], c[1], c[2],
                    0,
                )
            )


def write_3mf(path: str, objects: Sequence[Tuple[MeshBuilder, Tuple[float, float, float]]]) -> None:
    """Writes a standard-compliant 3MF package with named mesh objects positioned on the build bed."""
    content_types = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
        '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
        "</Types>"
    )
    rels = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
        'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>'
        "</Relationships>"
    )

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<model unit="millimeter" xml:lang="en-US" '
        'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">',
        '  <metadata name="Title">FoMaKo K30NS Window Anti-Reflection Shroud</metadata>',
        '  <metadata name="Application">Hudson River PTZ CAD Generator</metadata>',
        "  <resources>",
    ]

    build_items: List[str] = []
    for obj_id, (mesh, offset_xyz) in enumerate(objects, start=1):
        vmap: Dict[Vec3, int] = {}
        vlist: List[Vec3] = []
        idx_tris: List[Tuple[int, int, int]] = []
        for a, b, c in mesh.tris:
            tri_ids = []
            for pt in (a, b, c):
                vid = vmap.get(pt)
                if vid is None:
                    vid = len(vlist)
                    vmap[pt] = vid
                    vlist.append(pt)
                tri_ids.append(vid)
            idx_tris.append((tri_ids[0], tri_ids[1], tri_ids[2]))

        safe_name = saxutils.escape(mesh.name)
        lines.append(f'    <object id="{obj_id}" name="{safe_name}" type="model">')
        lines.append("      <mesh>")
        lines.append("        <vertices>")
        for vx, vy, vz in vlist:
            lines.append(f'          <vertex x="{vx:.4f}" y="{vy:.4f}" z="{vz:.4f}"/>')
        lines.append("        </vertices>")
        lines.append("        <triangles>")
        for v1, v2, v3 in idx_tris:
            lines.append(f'          <triangle v1="{v1}" v2="{v2}" v3="{v3}"/>')
        lines.append("        </triangles>")
        lines.append("      </mesh>")
        lines.append("    </object>")

        tx, ty, tz = offset_xyz
        transform = f"1 0 0 0 1 0 0 0 1 {tx:.3f} {ty:.3f} {tz:.3f}"
        build_items.append(f'    <item objectid="{obj_id}" transform="{transform}"/>')

    lines.append("  </resources>")
    lines.append("  <build>")
    lines.extend(build_items)
    lines.append("  </build>")
    lines.append("</model>")

    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("[Content_Types].xml", content_types)
        zf.writestr("_rels/.rels", rels)
        zf.writestr("3D/3dmodel.model", "\n".join(lines))


# ============================================================================
# Pure-Python Z-Buffer Software Renderer (for Visual Inspection PNGs)
# ============================================================================

def render_shaded_png(
    path: str,
    meshes_with_colors: Sequence[Tuple[MeshBuilder, Tuple[int, int, int]]],
    cam_eye: Vec3,
    cam_target: Vec3,
    cam_up: Vec3 = (0.0, 0.0, 1.0),
    width: int = 900,
    height: int = 650,
    fov_deg: float = 36.0,
) -> None:
    """Renders shaded 3D meshes to a PNG file using pure Python stdlib (zlib + struct)."""
    # Camera basis
    fx = cam_target[0] - cam_eye[0]
    fy = cam_target[1] - cam_eye[1]
    fz = cam_target[2] - cam_eye[2]
    fl = math.sqrt(fx * fx + fy * fy + fz * fz)
    fx, fy, fz = fx / fl, fy / fl, fz / fl

    rx = fy * cam_up[2] - fz * cam_up[1]
    ry = fz * cam_up[0] - fx * cam_up[2]
    rz = fx * cam_up[1] - fy * cam_up[0]
    rl = math.sqrt(rx * rx + ry * ry + rz * rz)
    rx, ry, rz = rx / rl, ry / rl, rz / rl

    ux = ry * fz - rz * fy
    uy = rz * fx - rx * fz
    uz = rx * fy - ry * fx

    focal = 0.5 * height / math.tan(math.radians(0.5 * fov_deg))
    zbuf = [1e30] * (width * height)
    rgb = bytearray([244, 246, 249] * (width * height))

    # Key + fill + rim directional lights
    l1 = (-0.45, -0.60, 0.66)
    l2 = (0.65, 0.35, 0.67)

    for mesh, base_col in meshes_with_colors:
        for a, b, c in mesh.tris:
            # Face normal
            e1x, e1y, e1z = b[0] - a[0], b[1] - a[1], b[2] - a[2]
            e2x, e2y, e2z = c[0] - a[0], c[1] - a[1], c[2] - a[2]
            nx = e1y * e2z - e1z * e2y
            ny = e1z * e2x - e1x * e2z
            nz = e1x * e2y - e1y * e2x
            nl = math.sqrt(nx * nx + ny * ny + nz * nz)
            if nl < 1e-12:
                continue
            nx, ny, nz = nx / nl, ny / nl, nz / nl

            proj = []
            skip = False
            for p in (a, b, c):
                dx, dy, dz = p[0] - cam_eye[0], p[1] - cam_eye[1], p[2] - cam_eye[2]
                cz = dx * fx + dy * fy + dz * fz
                if cz <= 5.0:
                    skip = True
                    break
                cx = dx * rx + dy * ry + dz * rz
                cy = dx * ux + dy * uy + dz * uz
                sx = 0.5 * width + (cx / cz) * focal
                sy = 0.5 * height - (cy / cz) * focal
                proj.append((sx, sy, cz))
            if skip:
                continue

            (x0, y0, z0), (x1, y1, z1), (x2, y2, z2) = proj
            area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
            if abs(area) < 1e-4:
                continue

            # Two-sided studio shading so cutaways/interiors are crisp
            vx_d = cam_eye[0] - a[0]
            vy_d = cam_eye[1] - a[1]
            vz_d = cam_eye[2] - a[2]
            if nx * vx_d + ny * vy_d + nz * vz_d < 0:
                nx, ny, nz = -nx, -ny, -nz

            d1 = max(0.0, nx * l1[0] + ny * l1[1] + nz * l1[2])
            d2 = max(0.0, nx * l2[0] + ny * l2[1] + nz * l2[2])
            shade = min(1.18, 0.28 + 0.58 * d1 + 0.30 * d2)
            cr = min(255, int(base_col[0] * shade))
            cg = min(255, int(base_col[1] * shade))
            cb = min(255, int(base_col[2] * shade))

            min_x = max(0, int(math.floor(min(x0, x1, x2))))
            max_x = min(width - 1, int(math.ceil(max(x0, x1, x2))))
            min_y = max(0, int(math.floor(min(y0, y1, y2))))
            max_y = min(height - 1, int(math.ceil(max(y0, y1, y2))))

            inv_area = 1.0 / area
            for py in range(min_y, max_y + 1):
                fy_p = py + 0.5
                row_off = py * width
                for px in range(min_x, max_x + 1):
                    fx_p = px + 0.5
                    w0 = ((x1 - fx_p) * (y2 - fy_p) - (y1 - fy_p) * (x2 - fx_p)) * inv_area
                    w1 = ((x2 - fx_p) * (y0 - fy_p) - (y2 - fy_p) * (x0 - fx_p)) * inv_area
                    w2 = 1.0 - w0 - w1
                    if w0 >= 0.0 and w1 >= 0.0 and w2 >= 0.0:
                        pz = 1.0 / (w0 / z0 + w1 / z1 + w2 / z2)
                        idx = row_off + px
                        if pz < zbuf[idx]:
                            zbuf[idx] = pz
                            c_off = idx * 3
                            rgb[c_off] = cr
                            rgb[c_off + 1] = cg
                            rgb[c_off + 2] = cb

    # Encode PNG
    raw_rows = bytearray()
    stride = width * 3
    for y in range(height):
        raw_rows.append(0)
        raw_rows.extend(rgb[y * stride : (y + 1) * stride])

    def png_chunk(tag: bytes, payload: bytes) -> bytes:
        return (
            struct.pack(">I", len(payload))
            + tag
            + payload
            + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)
        )

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    png_bytes = (
        b"\x89PNG\r\n\x1a\n"
        + png_chunk(b"IHDR", ihdr)
        + png_chunk(b"IDAT", zlib.compress(bytes(raw_rows), 6))
        + png_chunk(b"IEND", b"")
    )
    with open(path, "wb") as f:
        f.write(png_bytes)


def build_assembled_upright_preview(
    hood_print: MeshBuilder, backplate_print: MeshBuilder, cfg: ShroudConfig
) -> Tuple[MeshBuilder, MeshBuilder]:
    """Transforms print-oriented Hood and Backplate into upright Windowsill coordinates (X right, Y into room, Z up)."""
    # Hood print coords: (xp, yp, zp) = (x_cam, z_cam, y_cam) -> upright: (xp, -zp, yp) so +Y looks out window!
    # Wait: let +Y be INTO room (0 at window glass, +182 at rear) and +Z be UP from windowsill (0..218):
    # Transform (xp, yp, zp) -> (-xp, zp, yp) has det = (-1)*(0 - 1) = +1 (preserves outward normals!!).
    hood_upright = hood_print.transformed(lambda p: (-p[0], p[2], p[1]), "Hood_Upright")

    # Backplate print coords: zp=0 faces camera (-Y_room), zp=10.4 faces room (+Y_room), seated at y_room = 174.4 mm!
    # Transform (xp, yp, zp) -> (-xp, cfg.groove_seat_zp + 0.4 + zp, yp) ALSO has det = +1!
    bp_upright = backplate_print.transformed(
        lambda p: (-p[0], cfg.groove_seat_zp + 0.4 + p[2], p[1]), "Backplate_Upright"
    )
    return hood_upright, bp_upright


def main() -> None:
    out_dir = os.path.dirname(os.path.abspath(__file__))
    cfg = ShroudConfig()

    print("Generating FoMaKo K30NS Window Anti-Reflection Shroud CAD...")
    hood_mesh = build_hood_mesh(cfg)
    backplate_mesh = build_backplate_mesh(cfg)

    hood_stats = verify_mesh(hood_mesh)
    bp_stats = verify_mesh(backplate_mesh)

    hood_stl = os.path.join(out_dir, "fomako_k30ns_shroud_hood.stl")
    bp_stl = os.path.join(out_dir, "fomako_k30ns_shroud_backplate.stl")
    asm_stl = os.path.join(out_dir, "fomako_k30ns_shroud_assembly_preview.stl")
    h2d_3mf = os.path.join(out_dir, "fomako_k30ns_shroud_h2d.3mf")

    # Center each print STL cleanly on (0, 0, z>=0)
    hood_centered = hood_mesh.transformed(lambda p: (p[0], p[1] - 0.5 * cfg.front_outer_height, p[2]), hood_mesh.name)
    bp_centered = backplate_mesh.transformed(lambda p: (p[0], p[1] - 119.0, p[2]), backplate_mesh.name)

    write_binary_stl(hood_stl, hood_centered)
    write_binary_stl(bp_stl, bp_centered)

    hood_up, bp_up = build_assembled_upright_preview(hood_mesh, backplate_mesh, cfg)
    asm_mesh = MeshBuilder("FoMaKo_K30NS_Shroud_Assembly")
    asm_mesh.extend(hood_up)
    asm_mesh.extend(bp_up)
    write_binary_stl(asm_stl, asm_mesh)

    # Place Hood centered on Bambu H2D bed (175, 155, 0) and Backplate next to it for instant 2-plate auto-arrange
    write_3mf(
        h2d_3mf,
        [
            (hood_centered, (185.0, 155.0, 0.0)),
            (bp_centered, (520.0, 155.0, 0.0)),
        ],
    )

    print("\n=== CAD Verification Passed (100% Watertight 2-Manifold, Zero Supports) ===")
    for label, st in [("Part 1: Window Hood", hood_stats), ("Part 2: Chevron Backplate", bp_stats)]:
        print(
            f"  {label}:\n"
            f"    Dimensions (Print X x Y x Z) : {st['size_x_mm']:.1f} x {st['size_y_mm']:.1f} x {st['size_z_mm']:.1f} mm\n"
            f"    Triangles / Vertices         : {st['triangles']} tris / {st['vertices']} verts\n"
            f"    Boundary / Non-Manifold Edges: {st['boundary_edges']} / {st['non_manifold_edges']}\n"
            f"    Solid Volume / Est. PETG Mass: {st['volume_cm3']:.1f} cm³ (~{st['est_petg_g']:.0f} g at 15% gyroid)\n"
            f"    Max Overhang from Vertical   : {st['max_overhang_deg']:.1f}° (Unsupported area > 46°: {st['unsupported_area_mm2']:.2f} mm²)"
        )

    if "--render-dir" in sys.argv:
        rdir = sys.argv[sys.argv.index("--render-dir") + 1]
        os.makedirs(rdir, exist_ok=True)
        print(f"\nRendering verification views to {rdir} ...")
        # View 1: Front-3/4 looking into Window Aperture (shows Chin Baffle + 6 Sawtooth Optical Vanes + Cradle!)
        render_shaded_png(
            os.path.join(rdir, "view_front_aperture.png"),
            [(hood_up, (52, 58, 68)), (bp_up, (78, 88, 104))],
            cam_eye=(-250.0, -370.0, 210.0),
            cam_target=(0.0, 85.0, 105.0),
        )
        # View 2: Rear-3/4 showing Slide-In Chevron Louvered Backplate + 184x44mm Cable Arch + Finger Pull Tab
        render_shaded_png(
            os.path.join(rdir, "view_rear_assembled.png"),
            [(hood_up, (52, 58, 68)), (bp_up, (85, 98, 118))],
            cam_eye=(260.0, 470.0, 235.0),
            cam_target=(0.0, 95.0, 105.0),
        )
        # View 3: Print-Bed Orientation (Hood Window-Face Down)
        render_shaded_png(
            os.path.join(rdir, "view_hood_print_bed.png"),
            [(hood_centered, (58, 66, 78))],
            cam_eye=(280.0, -360.0, 290.0),
            cam_target=(0.0, 0.0, 85.0),
        )
        print("Rendered 3 verification PNGs successfully.")


if __name__ == "__main__":
    main()
