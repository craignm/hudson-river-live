# FoMaKo K30NS 4K 30x PTZ — 3D-Printable Window Anti-Reflection Shroud

Custom two-part, **100% support-free** window reflection shroud engineered specifically for the **FoMaKo K30NS** (`253.5 × 144.0 × 169.0 mm`) sitting upright on a windowsill flush against the glass, sized to print cleanly on a **Bambu Lab H2D** (`350 × 320 × 325 mm` bed, single-nozzle `325 × 320 mm` compatible).

---

## Files Included

| File | Print Orientation | Size (`X × Y × Z`) | Supports | Purpose |
| :--- | :--- | :--- | :--- | :--- |
| `fomako_k30ns_shroud_h2d.3mf` | Pre-oriented (arrange onto 2 plates) | Plate 1 + Plate 2 | **None** | Ready-to-open Bambu Studio project with both named parts |
| `fomako_k30ns_shroud_hood.stl` | **Window-Face Down** (already oriented) | `316.0 × 214.0 × 182.0 mm` | **None** | Main wrap-around matte box hood, LED chin baffle, 6 sawtooth light traps & elliptical base cradle |
| `fomako_k30ns_shroud_backplate.stl` | **Flat on Bed** (already oriented) | `270.1 × 203.0 × 10.4 mm` | **None** | Slide-in dovetail rear light shield with 18 blind `45°` `/\` chevron thermal louvers & cable arch |
| `fomako_k30ns_shroud_assembly_preview.stl` | Upright preview only (do not print as-is) | `316.0 × 185.2 × 217.0 mm` | — | Assembled windowsill preview mesh |
| `generate_k30ns_shroud.py` | Pure Python 3 stdlib CAD script | — | — | Parametric generator + manifold/overhang verifier (`python3 generate_k30ns_shroud.py`) |

---

## Key Optical & Thermal Features

1. **Bed-Flat Window Seal Rim (`316 × 214 mm` outer, `304 × 158 mm` optical aperture):**
   - Prints window-face down on your **Textured PEI Plate**, guaranteeing a dead-flat, micro-matte front rim that sits flush against the glass bare or with optional `1/4" (6 mm)` black adhesive foam weatherstripping.
   - **Ray-traced river sweep:** Unobstructed **`±55°` telephoto pan** and **`±37.7°` wide-angle pan** (`133.5°` total wide river panorama), plus **`-28°` downward tilt** (easily clearing Little Island at `-11.3°`) to **`+32°` upward tilt** for aircraft.
2. **Integrated `46 mm` Front Chin Baffle:**
   - Completely hides the K30NS's bright **green front status LED** and white silkscreened **`4K / FoMaKo / NDI|HX3`** base logos from reflecting in double-pane window glass at twilight and night.
3. **6 Concentric `45°` Sawtooth Optical Light-Trap Vanes:**
   - Breaks up grazing reflections along both side walls and the ceiling so semi-gloss dark filament sheen cannot bounce stray light back onto the window.
4. **Open-Bottom Sill Grip + Self-Centering Elliptical Base Cradle (`y_cam = 82.0 mm`):**
   - Open center bottom lets the K30NS's factory rubber feet grip your windowsill directly without adding height.
   - Curved elliptical inner bolsters (`256.5 mm` span = `253.5 mm + 1.5 mm/side`) automatically center the camera and stop its front nose `10 mm` back from the glass — **clearing a full `±170°` pan and `-30°..+90°` vertical power-on boot calibration spin inside the closed shroud without touching the glass or roof**.
5. **Slide-In Dovetail Rear Backplate with 18 Omni-Directional `/\` Chevron Louvers:**
   - Drops straight down into rear dovetail tracks for instant tool-free access to the K30NS `ON/OFF` switch, `LAN (PoE)`, `HDMI`, `3G-SDI`, and `USB 3.0` ports (`184 × 44 mm` lower cable arch).
   - Two-stage `45°` `/\` chevron air channels have **0.0% direct optical line-of-sight** from any angle in the room while exhausting solar heat during sunny afternoons.

---

## Recommended Bambu Studio Settings (H2D)

- **Filament (Important for Afternoon Window Sun!):**
  - **Best:** **Matte Black PETG**, **Black ASA/ABS**, or **Heat-Resistant Matte Black PLA (`BIQU PLA-HR`)**. *(Avoid standard low-temp glossy PLA if your window gets direct afternoon sunlight, as enclosed dark window bays can exceed `55°C`.)*
- **Build Plate:** **Textured PEI Plate** (gives the window-facing front flange & chin baffle a non-reflective matte texture).
- **Supports:** **Disabled (`None`)** on both plates — every overhang is `≤ 45°` with zero horizontal bridges.
- **Layer Height:** `0.20 mm` or `0.24 mm` (`0.4 mm` nozzle)
- **Wall Loops:** `3` walls (`15%` Gyroid or Cross-Hatch infill)
- **Optional Pro Anti-Glare Tweak:** Enable Bambu Studio **Fuzzy Skin -> Contour (`0.2 mm` point distance, `0.15 mm` thickness)** on the Hood, or line the interior walls with adhesive black flock velvet if shooting in a brightly lit room at night.

---

## Windowsill Setup Tips

1. **Optional Yoke Logo Mask:** Put a small piece of matte-black gaffer or electrical tape over the vertical white `FoMaKo` logo on the camera's side yoke arm (the `46 mm` chin baffle already blocks the green base LED and front base logos).
2. **Placement:** Set the **Main Hood** flush against the window glass (add `6 mm × 3 mm` black open-cell foam tape around the front rim if your window frame prevents hard plastic from sitting flush).
3. **Drop Camera In:** Slide the **FoMaKo K30NS** forward through the open back until its oval base nests gently against the curved front alignment bolsters, plug in your cables, flip the power switch ON, and slide the **Chevron Rear Backplate** down into the rear dovetail tracks.
