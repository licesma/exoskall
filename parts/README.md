# parts/: print files

## The rotor (final, EXO-333, 2026-10-22)

Two printed parts sit inside the bearing of each `sensor module 2` joint. Both are Fusion
High-refinement 3MF exports of the module copy `sensor module 2 rotor` in the saved document
`exo generated`, made by the generator (`fusion_scripts/BuildArm/BuildArm.py`,
`ensure_v3_module` -> `build_v3_copy` -> `rotor_v3` + `module_rotor_final`). Units mm,
each in its own component frame: **module z = file z - 7.5** (the rotor face is file z 7.5).

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) | Through-holes |
|---|---|---|---|---|---|
| `top.3mf` | upper rotor part (`inner_top`), rotor key sockets (EXO-364) | 36.000 x 35.999 x 8.200 | 5772.270 | 5772.377 | 8 |
| `bottom.3mf` | lower rotor part, on the servo horn (`inner_bottom`) | 35.000 x 34.995 x 6.200 | 3865.634 | 3865.869 | 8 |
| `rotor.bambu.3mf` | Bambu Studio plate: `top` rotor face down (sockets on the bed, 881.5 mm2), `bottom` O35 plate down, supports on (tree auto), EXO-294 ids; `tools/bambu_plate_exo343.py --plate rotor` (EXO-364) | - | - | - | - |

Module z, all read back off the exported meshes (EXO-333 record):

- **Joint face z -7.0** between the parts. The bottom's web runs from the horn top / bore
  ceiling -10.5 to it: **web 3.5**. A stock **M2 x 6** horn screw seated on the joint face
  goes **2.5** mm into the horn.
- **Race gap 6.0**: top shoulder (disk underside) **-5.001**, bottom flange **-11.001**, on
  the 7.000 bearing race, so the printed parts clamp it.
- **Journal O30.1** (r 15.05) on both parts: bottom -11.001..-7.0, top -7.0..-5.001.
- **Nut pockets 5.6 AF** (ISO 4032 M3), flats radial, r 11, z -13.2..-10.6, from the
  bottom's underside. Pocket-to-journal wall **1.156**, socket-to-journal wall **1.200**.
- Top: four **O5 head columns** through to the rotor face (r 6), 4 x O3.2 M3 at r 11,
  eight **O3.0 pegs** (r 12.2, 1.2 tall, down to -8.2), and the **rotor key** (EXO-364): three
  **O3.3 x 1.3 sockets** in the rotor face at r 15.0, 45/135/225 deg (315 empty; the old index
  notch is filled). Bottom: horn bore
  r 8.1 (0.3 above the servo top), 4 x O2.2 horn holes, 4 x O3.2, eight **O3.3 sockets**
  1.3 deep (floor -8.3).
- Assembly: horn screws (M2 x 6) through the bottom into the horn first, through the top's
  columns if it is already stacked; then the link's four **M3 x 20** through pad, top and
  bottom into the nuts.

## Link 12 + stator (EXO-341, 2026-10-22)

High-refinement 3MF exports from `exo generated`; read back with `tools/verify_3mf.py`
(watertight, one shell) and vertex rings.

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) |
|---|---|---|---|---|
| `link_12.3mf` | `g_link_12` of `generated_R_arm_general` (J1 rotor -> J2 stator), key pegs (EXO-364) | 58.498 x 74.514 x 63.529 | 40406.203 | 40411.590 |
| `stator.3mf` | 15 deg stop-block stator + assembly mark (component `stator EXO-341 export`) | 53.999 x 54.000 x 24.100 | 25680.834 | 25684.494 |
| `link.bambu.3mf` | Bambu Studio plate (EXO-343): 2x stator underside down; link_12 rotated (baked) onto the J2 pad's outer face, 58.5 tall; settings = `rotor.bambu.3mf`; built by `tools/bambu_plate_exo343.py` | - | - | - |

- **Stator**: own frame, **module z = file z - 7.5** (underside file z -17.0). Bore O42.10
  (r 21.05), 3 x O4.0 inserts r 19.25 at 0/180/270, "0" label: identical to the previous
  file (`archive/2026-10-tests/stator_pre_exo341.3mf`); only the mark is new.
- **Link**: ARM coordinates. J1 rotor face (rotor pad z 0..8 in J1): centre
  (0, -200.430, 0), axis (0.0002, -0.9612, 0.2757). J2 stator-pad face (J2 z -24.5, pad
  -32.5..-24.5): centre (-24.492, -240.775, -2.817), normal +X. J2 origin
  (0.0076, -240.7704, -2.8187), X_J2 (0, -0.2757, -0.9612), Y_J2 (-0.0002, 0.9612, -0.2757).
  Holes 4 x O3.2 r 11 (J1), 3 x O3.2 r 19.25 at 0/180/270 (J2), cutout x +-10, y -9.5..24.5,
  rotor key (EXO-364): three O3.0 x 1.2 pegs at r 15.0, J1 45/135/225 deg (315 empty), into
  the top's sockets (0.15 radial, 0.1 axial); the index tab is gone.
- **Assembly mark** (`armgen.ALIGN_MARK_*`): a 0.6 wide x 0.4 deep radial line at module
  **+Y** (connector side, no insert) on both mating faces: stator underside r 24.5..27 (out
  through the rim), link pad face r 24.5..27.6 (0.6 stays visible outside the stator). Line
  them up, then fit the three M3 screws.

## Link 23 + 4-joint plate (EXO-345, 2026-10-22)

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) |
|---|---|---|---|---|
| `link_23.3mf` | `g_link_23` of `generated_R_arm_general` (J2 rotor -> J3 stator), ARM coordinates, with the assembly mark and key pegs (EXO-364) | 62.504 x 60.246 x 105.699 | 45832.391 | 45842.577 |
| `joints4.bambu.3mf` | Bambu Studio plate: 4x stator (underside down), 4x top (rotor face down), 4x bottom (O35 plate down) as `rotor.bambu.3mf`, + link_23 standing on its J3 flange end face (62.5 tall, J2 pad on supports); settings = `rotor.bambu.3mf`; layout 229.5 x 216.75, margins >= 13.25, gaps >= 10.5; `tools/bambu_plate_exo343.py --plate joints4` | - | - | - |

- **Link 23** (arm coords): J2 rotor face centre (0.008, -240.770, -2.819), normal +X;
  J3 stator-pad face centre (0.004, -247.015, -81.518), normal -Z. Rotor pad 4 x O3.2 r 11,
  key pegs at J2 45/135/225 deg (EXO-364), stop block r 20..27, -55..+69 deg; stator pad 3 x O3.2 r 19.25 at
  0/180/270, XL330 cutout; mark at J3 +Y, r 24.5..27.6 (EXO-346).

## Everything on one plate (EXO-348, 2026-10-22)

- `all.bambu.3mf`: **one plate**, 14 objects = `joints4.bambu.3mf` (4x stator, 4x top, 4x bottom, link_23) + link_12 as in `link.bambu.3mf`, same print orientations (meshes identical), no XY rotation; layout 236.24 x 232.51, margins >= 9.88, gaps >= 10.99; settings = `rotor.bambu.3mf`; `tools/bambu_plate_exo343.py --plate all`.

## Stator bearing-seat test, O41.8..42.6 (EXO-350, EXO-355, EXO-358, 2026-11-03)

`stator.3mf` (seat O42.10) grips the 5972K159 outer race too hard to remove. Copies of
`stator EXO-341 export` with only the seat enlarged (file z -3.5..3.5 = module -11..-4; shoulder
below, stop block, inserts, "0", mark, pocket, r 27 rim unchanged) and "42.x" debossed 0.4 on
the top face at -Y. Components `stator d42.x export` in `exo generated` (bulb off);
`tools/bridge_scripts/build_stator_dia_exo350.py`. Same frame as `stator.3mf`. Below 42.1 the
seat is made smaller by unioning a ring r R..21.10 over z -3.5..3.5 (shoulder z -3.5 and step bore
r 19.25 unchanged; the exposed shoulder is then r 19.25..R). 42.7 / 42.9
(EXO-350) were dropped by EXO-355 and are in `archive/2026-10-tests/`.

| File | Seat (mesh fit) | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) |
|---|---|---|---|---|
| `stator_d41.8.3mf` | O41.8000, z -3.5..3.5 (ring unioned, EXO-358) | 53.999 x 54.000 x 24.100 | 25815.537 | 25819.058 |
| `stator_d41.9.3mf` | O41.9000, z -3.5..3.5 (ring unioned, EXO-358) | 53.999 x 54.000 x 24.100 | 25769.600 | 25773.105 |
| `stator_d42.0.3mf` | O42.0000, z -3.5..3.5 (ring unioned, EXO-358) | 53.999 x 54.000 x 24.100 | 25723.026 | 25726.534 |
| `stator_d42.1.3mf` | O42.1000 = `stator.3mf` + label only (EXO-358) | 53.999 x 54.000 x 24.100 | 25677.170 | 25680.827 |
| `stator_d42.2.3mf` | O42.2000, z -3.5..3.5 | 53.999 x 54.000 x 24.100 | 25630.734 | 25634.005 |
| `stator_d42.3.3mf` | O42.3000, z -3.5..3.5 | 53.999 x 54.000 x 24.100 | 25584.431 | 25587.709 |
| `stator_d42.4.3mf` | O42.4000, z -3.5..3.5 | 53.999 x 54.000 x 24.100 | 25537.812 | 25541.092 |
| `stator_d42.5.3mf` | O42.5000, z -3.5..3.5 | 53.999 x 54.000 x 24.100 | 25491.064 | 25494.326 |
| `stator_d42.6.3mf` | O42.6000, z -3.5..3.5 | 53.999 x 54.000 x 24.100 | 25443.948 | 25447.461 |
| `stator_dia.bambu.3mf` | Bambu plate: the five, underside down, bore + label up, rows of 3 + 2, gaps 10.5; settings = `rotor.bambu.3mf`; `tools/bambu_plate_exo343.py --plate stator_dia` | - | - | - |
| `stator_dia_low.bambu.3mf` | Bambu plate (EXO-358): stator_d41.8 .. 42.4 (7), underside down, rows of 3 + 3 + 1, gaps 10.5, margins 36.5; settings = `rotor.bambu.3mf`; `tools/bambu_plate_exo343.py --plate stator_dia_low` | - | - | - |

## Rotor key (EXO-364, 2026-11-04)

The index notch / tab is replaced by three pegs on every link's rotor pad (O3.0 x 1.2) into three sockets in the rotor top (O3.3 x 1.3), r 15.0 at module 45/135/225 deg, 315 empty: the link goes on one way only (URDF q = 0 <-> empty position at 315 <-> link as drawn). `top.3mf`, `link_12.3mf`, `link_23.3mf` re-exported; `rotor`, `link`, `joints4`, `all` plates rebuilt (same layouts and settings, stator unchanged O42.1). Print a new top with new links: old tops (notch) and old links (tab) do not mate with the new parts.

## Other files

- `stator.f3d`: stator as it was before EXO-341 (no mark).
- `tool_organizer.3mf`: EXO-258 organizer box.
- `archive/2026-10-tests/`: every rotor test print of October 2026 (original and v3 parts,
  pocket, web, race-gap and journal-diameter tests, their plates, and the previous README),
  one line each in its `INDEX.md`.
