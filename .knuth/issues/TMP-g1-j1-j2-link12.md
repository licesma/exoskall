---
id: TMP-g1-j1-j2-link12
title: Generator round 1 — place J1, J2, J3 and build links 12, 23 from the G1 URDF
description: Fusion script places J1, J2, J3 at the G1 right-shoulder frames and generates links 12 and 23; compare with the existing arm.
status: in-progress
mode: subagent
parent: TMP-exoskeleton-from-urdf
---
## Context

First validation of the generator (see parent). Input: `g1/g1_29dof.urdf`, right arm.

| exo | URDF joint | parent→child | xyz (m) | rpy | axis |
|---|---|---|---|---|---|
| J1 | `right_shoulder_pitch_joint` | torso_link → right_shoulder_pitch_link | 0.0039563 −0.10021 0.23778 | −0.27931 5.5e−5 1.9e−4 | 0 1 0 |
| J2 | `right_shoulder_roll_joint` | right_shoulder_pitch_link → right_shoulder_roll_link | 0 −0.038 −0.013831 | 0.27925 0 0 | 1 0 0 |
| J3 | `right_shoulder_yaw_joint` | right_shoulder_roll_link → right_shoulder_yaw_link | (see URDF) | | 0 0 1 |

Scope added 2026-09-13: also place J3 (exo `Module 2 R`) and generate link 23 (J2 rotor pad → J3
stator pad) from `Link 2.step` / `Link 2 R`, which Esteban considers the representative link pattern;
the body generator is shaped by it.

Ground truth to compare against: `gento_luna/exo/exports/exo_skeleton_2026-08-08.step`
(hand-built for this URDF) and `gento_luna/Gento Luna_URDF 20260428/urdf/Link 2.step`
(= link 12, see `.knuth/docs/exo-link.md`).

## Task

1. **Place joints.** Extend `fusion_scripts/PlaceModules/PlaceModules.py` (or a new
   `fusion_scripts/BuildArm/`) so it places `sensor module 2` at J1 and J2: module
   origin on the joint origin, module Z on the joint axis. Joint frames are computed
   by the existing FK code; make joint selection and URDF path parameters, not constants.
2. **Generate link 12** in the same script: rotor pad on J1 (4× M3 holes, stop block,
   index feature), stator pad on J2 (actuator cutout, 3× M3 holes), body joining them.
   Measure pad dimensions and hole positions from `Link 2.step`; the body may be the
   simplest solid that connects the pads without intersecting either joint.
3. **Compare visually.** No compare script. Esteban runs the script in a document where
   the existing arm is visible and judges the overlay by eye. The script must therefore
   build everything under one new component (e.g. `generated_R_arm`) so it can be
   toggled/moved as a unit, and must not touch existing components.

## Done when

- Script runs in Fusion (Esteban runs it) and produces J1, J2, link 12 — screenshot in
  `gento_luna/exo/screenshots/`.
- Before hand-off, the agent's own numeric sanity check (J1/J2 module placement in the
  exo STEP vs URDF FK, and Link 2.step pad measurements) is pasted in this file.
- `.knuth/docs/exo-link.md` gets the measured pad dimensions.

## Judgment calls to report, not ask

Z sign of the module vs URDF axis sign; how the index feature is represented on the
pad; rotation of each module about its Z.

## Sanity check (agent, 2026-09-13, before hand-off)

Tooling: `/usr/bin/python3`, `tools/steplib.py` (regex STEP reader), `tools/test_armgen.py`.
Exo STEP = `gento_luna/exo/exports/exo_skeleton_2026-08-08.step` (1.41 M entities, parses in 1.7 s).
Exo frame: root `arm complete` at identity; **left J1 rotor-face centre is the document origin**.
URDF frame: pelvis. The two differ by a pure translation, URDF(mm) = exo + (−0.007, 100.22, 291.78).

### Module placement in the exo STEP vs URDF FK (right arm)

| | exo STEP (mm / unit) | URDF FK (`armgen.joint_world_pose`) | diff |
|---|---|---|---|
| J1 origin | `Module 0 R` (0, −200.440, 0) | (−0.007, −100.210, 291.780) | 0.01 mm after frame shift |
| J1 axis | (0, 0.9613, −0.2756) | (−0.0002, 0.9612, −0.2757) | 0.013° |
| J2 origin | `Module 1 R` (0, −240.772, −2.791) | (0.000, −140.550, 288.961) | — |
| J2 axis | (1, 0, 0) | (1.0000, 0.0002, −0.0001) | 0.013° |
| J1→J2 offset | (0, −40.332, −2.791) | (0.008, −40.340, −2.819) | 0.03 mm |
| J1↔J2 axis skew | 13.80 mm | 13.83 mm | |
| J1 tilt from Y | 16.0° | 16.0° | |
| shoulder spacing | 200.44 | 200.43 | |

Verdict: the exo was built to this URDF; premise holds. Exo `Module k` = URDF joint k+1
(Module 0 = shoulder pitch … Module 6 = wrist yaw; axes match one by one).

### Physical module orientation (right arm is a Fusion *mirror* of the left)

Sub-parts are named `(Mirror)` and the right-arm occurrence transforms are rotations applied
to mirrored geometry, so the STEP `z` column is not the physical rotor-face normal. Measured
from the geometry (bearing sits at −7.5 along the rotor normal; Link 1 R stator pad at
world x ∈ [−32.5, −24.5]):

| joint | physical module Z (out of rotor face) | vs URDF axis | connector side (module +Y) |
|---|---|---|---|
| J1 R | (0, −0.9613, 0.2756) | **−axis** | (0, 0.2756, 0.9613) ≈ up |
| J2 R | (1, 0, 0) | **+axis** | (0, 1, 0) toward chest |

J2 in the J1 module frame: (0, −13.8, 38.0) mm, J2 axis = J1-frame +X (exo) vs
(−0.001, −13.83, 38.00), (1, 0, 0) from the URDF (`tools/test_armgen.py`, all 24 checks ≤ 0.05 mm /
0.001; the sabotaged copy with J1 `axis_sign` flipped fails 4 checks and trips the intersection guard).

### Link 12 = `Link 1 R` in the exo STEP (NOT `Link 2.step`, see findings)

Measured in the J1 module frame (left `Link 1`, identical mirrored on the right):

- rotor pad: disk **r 26**, z ∈ [0, 8]; 4× Ø3.2 at r 11, angles 0/90/180/270
- link stop block: ring sector r 20–26, ±5° about module +X, z ∈ [−3.6, 0] (stator top at −4.0)
- stator pad (J2 side): disk **r 27**, 8 thick, on the stator underside (module z −24.5 … −32.5);
  3× Ø3.2 at r 19.25, angles 0/180/270; actuator cutout x ∈ [−10, 10], y ∈ [−9.5, 24.5] (= XL330 envelope, zero clearance)
- body: L-bracket — the 8 mm pad plate runs to the stator-slab far face; a vertical plate in the
  slab thickness (x ∈ [24.5, 32.5] left / [−32.5, −24.5] right) runs from z ≈ 10 up into the pad.
  J2 stator rim sits 11.0 mm above the pad plane (3 mm gap).
- index feature: `Link 1 R` carries two Ø1.5 rounded features at r ≈ 10–13 near module +X (unidentified;
  the exo rotor `Component155` is an older revision with 8 holes at 45° and no notch). `Link 2.step` has none.

## Judgment calls

- **Z sign**: per-joint `axis_sign` parameter; defaults J1 = −1, J2 = +1 reproduce the existing right arm.
- **Rotation about Z**: per-joint `y_hint` (world direction the connector side leans to): J1 up (0,0,1),
  J2 (0,1,0). Defaults match the exo. A mirrored module cannot be reproduced with the real (unmirrored)
  `sensor module 2`; with these defaults the connector side / cutout matches, the two stop blocks land
  on world +X instead of −X.
- **Index feature**: a tab 3.6 × 2.75 × 1.2 mm at module +Y (r 14.95–17.7) mating with the current rotor's
  notch (4.0 × 3.15 × 1.5 at +Y), 0.2 mm clearance per side. The unidentified Ø1.5 features of `Link 1 R`
  are not reproduced.
- **Stop block**: copied from `Link 1 R` (±5° at module +X, 3.6 tall); its parallel-sided box
  approximation is 4.0 mm wide at mid radius instead of radial faces. Limits vs URDF still deferred.
- **Origin**: `ORIGIN_AT_JOINT = 'left_shoulder_pitch_joint'` so the generated arm overlays the existing
  design (its origin is the left J1 rotor face). Set to `None` for raw pelvis coordinates.
- **Body**: L-bracket from axis-aligned boxes in the J1 frame (mimics the original's shape, minus its
  Ø3/Ø10 counterbored hole and Ø5 hole on the vertical plate). Requires J2 ⊥ J1 and the J2 module clear
  of the pad plate; both are checked and fail loudly.
- **Geometry construction**: `TemporaryBRepManager` primitives + booleans inside a base feature,
  instead of sketches on construction planes (no plane-orientation ambiguity, works for any frame).
- New folder `fusion_scripts/BuildArm/` instead of editing PlaceModules (PlaceModules left untouched;
  its FK is duplicated in `armgen.py`, which is importable without Fusion).

## Hand-off (Esteban)

1. Open the Fusion design the exo STEP was exported from (`arm complete`, existing arm visible).
   `sensor module 2` must be in that design or saved in the same project.
2. Scripts and Add-Ins → add `fusion_scripts/BuildArm` → Run → pick `g1/g1_29dof.urdf`.
3. Expect: new root component `generated_R_arm` containing `J1_right_shoulder_pitch_joint`,
   `J2_right_shoulder_roll_joint` (each wrapping one module) and `link_12` (one body);
   a message box lists the frames. The joints should sit on `Module 0 R` / `Module 1 R`; the
   link should overlay `Link 1 R` except the stop-block side. Toggle `generated_R_arm` to compare.
4. Screenshot → `gento_luna/exo/screenshots/`.

## Round 2 (2026-09-13) — placement bug, iteration loop, J3 + link 23

**Observed:** first run placed J1 and J2 modules at the identical pose although the report listed
the right frames. **Mechanism (API docs, not observed in a debugger):** in a parametric design,
occurrence transforms set through the API (including the matrix passed to `addNewComponent`)
are pending position captures until `design.snapshots.add()`; the link's base feature
(`startEdit`/`finishEdit`) triggered a timeline compute that reset all pending wrapper transforms
to identity. PlaceModules never created a feature after placing, which is why it did not show it.
**Fix:** links are built first (body pre-transformed into the arm frame, component at identity);
wrappers are placed after, with `transform2` set explicitly; `design.snapshots.add()` is called
when `hasPendingSnapshot`; every wrapper transform and the module-under-wrapper are read back and
compared in the report (`OK` / `MISMATCH`). The arm component is at identity (origin shift baked
into the joint matrices) so parent-relative and root-relative transforms coincide. The module is
inserted once at the root and reused with `addExistingComponent`, then the root copy is deleted.

Iteration loop: `URDF_PATH` defaults to `<repo>/g1/g1_29dof.urdf` (dialog only if missing);
report + traceback written to `gento_luna/exo/exports/buildarm_report.txt`; previous
`generated_R_arm*` root components are deleted at the start of each run.

### J3 decisive check (exo `Module 2 R` vs URDF `right_shoulder_yaw_joint`)

| | exo STEP | URDF FK (exo frame) | diff |
|---|---|---|---|
| origin | (6.240, −240.766, −105.991) | (0.003, −247.017, −106.018) | **8.83 mm** (6.24 in x, 6.25 in y, 0.03 in z) |
| axis | (0, 0, 1) STEP; physical Z (0, 0, −1) | (0.0001, 0.0001, 1) | 0.01° |

Axes match, positions do not: the hand-built exo carries the 6.2 mm yaw-axis offset along +X
(NOTES change-list #3), the G1 URDF has it along −Y (outboard). The generator follows the URDF, so
the generated J3 and link 23 will sit 8.8 mm from `Module 2 R` / `Link 2 R` in the overlay — this is
the exo's deviation, not a generator error. Physical J3 module Z = −URDF axis; connector side = world −X.

### Link 23 = `Link 2.step` (= `Link 2 R` mirrored, 83/83 vertices) measured in the J2 / J3 frames

- rotor pad (J2): r 27 × 8; 4× Ø3.2 at r 11; stop block r 20–27, **±10° at 270°** (module −Y), 3.5 below face
- stator pad (J3): r 27 on the underside, holes 0/180/270, cutout 20 × 34; **flange** 25 wide (X_B ±12.5)
  extending the pad to Y_B = 39.24 with r 15 side fillets
- body: **bar** 25 wide (X_B) × 7.8 thick at Y_B ∈ [31.45, 39.24] (6.85 past the cutout), running along
  −X_A from the flange to a **riser** at s ∈ [28.4, 34.9] just outside the J2 module radius, up to a plate
  extension of the rotor pad; bar level z_A ∈ [−33, −25.2]. J3 in the J2 frame: (−103.2, 0, 6.24).
- generator (`armgen.link_bar_geometry`): plate extension s ∈ [0, 35], riser s ∈ [27, 35], bar s ∈ [27, 78.7]
  at z_A = −29.11 ± 4, flange to Y_B = 39.35 — all within 0.15 mm of the measured planes
  (`tools/test_armgen.py`, 42 checks; sabotaged copy with J3 `y_hint` flipped fails 4).
- fillets (r 15 / r 12.5 / r 8.5 / r 5) and the r 7.9 relief are not reproduced.

### Judgment calls (round 2)

- `body` per link: `link_12` keeps the measured L-bracket, `link_23` uses the bar pattern; the bar is the
  default for new links. Bar/riser/extension all use the 25 mm bar width (Link 2's riser is 36 wide).
- J3 `axis_sign = −1`, `y_hint = (−1, 0, 0)` (from the exo XL330 centroid direction).
- The 8.83 mm J3 deviation is reported, not compensated.
- `addByInsert` is done once at the root and the copy deleted after reuse — the report warns if the
  delete fails (stray module at the origin).

### Hand-off (round 2)

Same as before; expect `generated_R_arm` with three wrappers (J1, J2, J3), `link_12`, `link_23`, and
`gento_luna/exo/exports/buildarm_report.txt` ending in per-joint `read back … -> OK` lines. J3 and
link 23 will be 8.8 mm off `Module 2 R` / `Link 2 R` by design (see above).
