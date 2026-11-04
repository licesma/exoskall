# Exoskeleton project — working notes

How to use this folder (for Esteban):
- `screenshots/` — drop Fusion screenshots here (PNG/JPG). Claude can view them.
- `exports/`    — export STEP (preferred) or STL from Fusion here. Claude can measure/verify them.
- This file     — write questions, decisions, or status below. Claude reads it when you ask.

Tip: name exports with a version, e.g. `elbow_v1.step`, `elbow_v2.step`.

---

## Key reference numbers (from Gento Luna URDF, per arm)

- Shoulder cluster (J1 pitch ±173°, J2 roll ±118%) — axes meet at shoulder point
- J3 upper-arm rotation ±173° (ring joint — hard, do last)
- Upper arm length: 264 mm, elbow offset: 18 mm
- J4 elbow: −145° / +45°  ← STARTING JOINT
- Forearm length: 264 mm
- J5 forearm rotation ±173° (ring joint — hard, do last)
- Wrist: J6 ±60°, J7 ±90° — axes meet at wrist point
- Shoulders 429 mm apart (±214.6 mm from centerline)
- Link masses: 1.54 / 1.54 / 1.36 / 1.03 / 1.00 / 0.39 / 0.38 kg (shoulder→wrist)

## Status / decisions

- 2026-08-08: STEP files requested from Gento, waiting.
- 2026-08-08: NEW PLAN — adapt existing G1 exoskeleton (GELLO-style leader arm, XL330 servos + 5972K159 bearings, STEP at gento_luna/exo/exports/exo_skeleton_2026-08-08.step) instead of designing from scratch. Joint structure matches Gento Luna 1:1.
- Requirement: match ALL joint positions as precisely as physically possible (joint limits deferred for now).

## Change list — exo → Gento Luna (per arm, mirror for R)

| # | Difference | Exo | Gento | Fix |
|---|---|---|---|---|
| 1 | Shoulder pitch tilt | 16.3° | 0° | re-angle Module 0 mount on chest |
| 2 | Pitch↔roll axes | skew ~14 mm | intersect | reposition Module 1 bracket |
| 3 | Upper-arm rot. axis | 6.2 mm off shoulder plane | through shoulder pt | shift Module 2 by 6.2 mm |
| 4 | Upper arm length | 183.7 | 264 | stretch link M2→M3 +80.3 mm |
| 5 | Elbow side-offset | 15.8 | 18 | +2.2 mm elbow bracket |
| 6 | Forearm axis offset | 10 fwd | 18 back | move forearm axis 28 mm (Module 4 bracket) |
| 7 | Forearm length | 138 | 264 | stretch link M4→M5 +126 mm |
| 8 | Wrist pitch↔yaw | 46 mm apart | intersect | DECIDED 2026-08-08: sacrifice wrist pitch (J6), keep yaw (J7). Remove Module 5, Module 6 axis takes the wrist-point position |
| 9 | Shoulder spacing | 200.4 | 429.2 | widen chest mount +114.4 mm/side |

Exact displacement directions for #1,2,3,6: Claude computes from STEP+URDF before each edit.

## Questions for Claude

(write here)
