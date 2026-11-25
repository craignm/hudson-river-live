# Sony SRG-A40 4K PTZ — 3D-Printable Flared Window Anti-Reflection Shroud

Custom two-part, **100% support-free** sculpted cinema matte-box window shroud engineered for the **Sony SRG-A40** (`156.7 × 184.0 × 200.0 mm`) and ray-traced to stay completely out of view across the **Insta360 Link 2C** field of view (`67.96° × 41.52°`) plus generous pan/tilt/zoom margin.

---

## Files in This Folder

Folder path:
`/Users/craignm/Library/CloudStorage/Dropbox-Dropbox/Craig Nevill-Manning/AI/PTZ camera over the Hudson River/shroud/sony_a40/`

| File | Print Orientation | Size (`X × Y × Z`) | Supports | Est. Mass |
| :--- | :--- | :--- | :--- | :--- |
| `sony_srg_a40_shroud_h2d.3mf` | Pre-oriented (Auto-Arrange onto 2 plates) | Plate 1 + Plate 2 | **None** | ~`866 g` total |
| `sony_srg_a40_shroud_hood.stl` | **Window-Face Down** (already oriented) | `292.0 × 210.0 × 216.0 mm` | **None** | ~`763 g` |
| `sony_srg_a40_shroud_backplate.stl` | **Flat on Bed** (already oriented) | `172.1 × 200.0 × 10.4 mm` | **None** | ~`103 g` |
| `sony_srg_a40_shroud_assembly_preview.stl` | Upright preview only (do not print as-is) | `292.0 × 218.8 × 214.0 mm` | — | — |
| `generate_sony_a40_shroud.py` | Pure Python 3 stdlib CAD script | — | — | Parametric generator |

---

## Optical & Mechanical Engineering Summary

1. **Cleared for Full Insta360 Link 2C FOV (`68.0° × 41.5°`) + Margin:**
   - **Sculpted Flared Matte-Box Profile:** Compact **`184.0 mm` (`7.24"`)** sill footprint and rear width hugging the `156.7 mm` Sony body, flaring out to a **`280.0 × 138.0 mm` inner window aperture** (`292.0 mm` outer) on the glass side:
     - **At `1×` Wide (`70.0°` Sony HFOV / `68.0°` Link HFOV):** `+105 mm` lateral clearance per side when centered; remains 100% unobstructed even with up to **`±25°` off-axis yaw** at full `1×` wide zoom.
     - **Tracking Across Link FOV (`±34°`) + Margin:** Unobstructed out to **`±42°` pan at `2.0×` zoom**, **`±45°` pan at `3.5×` zoom**, and **`±50°` pan (`100°` arc) at `10×–30×` telephoto**.
2. **`62.0 mm` Front Chin Baffle (`POWER / NETWORK` LEDs + `SONY` Logo Mask):**
   - Unlike shorter PTZ bases, the Sony SRG-A40 has `POWER` & `NETWORK` LEDs at `z = 12–18 mm` **plus** a bright white `SONY` emblem at `z = 46–52 mm`. The `62 mm` front chin wall blocks 100% of both while still clearing the camera's full **`-20°` downward mechanical tilt limit** even at `1×` wide zoom.
3. **Self-Centering Semicircular + Parallel Base Cradle (`216 mm` Total Sill Depth):**
   - Open bottom lets the Sony SRG-A40's factory rubber feet grip your windowsill directly.
   - Precision semicircular front bolster (`R = 80.0 mm` for the `R = 78.35 mm` Sony front base) + parallel rear rails (`160.0 mm` span) center the camera at `y_cam = 84.35 mm` behind the glass, clearing full `±170°` pan and `+90°` tilt boot calibration.
4. **Exposed Rear I/O Block + 16 Blind `/\` Chevron Thermal Louvers:**
   - The dovetail rear backplate seats right above Sony's `73.7 mm` tall rear connector block (`136.0 × 74.4 mm` lower opening + side corner light-skirts), leaving `HDMI`, `3G-SDI`, `PoE++ LAN`, `VISCA`, `Audio`, and `19.5V DC` ports accessible while blocking 100% of direct room light and venting afternoon window solar heat.

---

## Recommended Bambu Lab H2D Print & Camera Tips

- **Filament:** **Matte Black PETG**, **Black ASA/ABS**, or **`BIQU PLA-HR`** (resists afternoon window solar heating).
- **Build Plate & Slicer:** **Textured PEI Plate**, **Supports = `None`**, `0.20–0.24 mm` layer height, `3` walls, `15%` gyroid infill, optional **Fuzzy Skin** (`0.2 mm / 0.15 mm`).
- **Sony Tally Light Tip:** The SRG-A40 has an upper dome tally lamp above the lens (`Red` / `Green`). Make sure **Tally Lamp** is disabled in the Sony web menu (`System -> Installation -> Tally Lamp: Off`) or cover the tiny oval lens-dome tally window with a `6 mm` dot of black electrical tape so it never reflects in the window at night.
