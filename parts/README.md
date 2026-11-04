# parts/: print files

## The rotor (final, EXO-333, 2026-10-22)

Two printed parts sit inside the bearing of each `sensor module 2` joint. Both are Fusion
High-refinement 3MF exports of the module copy `sensor module 2 rotor` in the saved document
`exo generated`, made by the generator (`fusion_scripts/BuildArm/BuildArm.py`,
`ensure_v3_module` -> `build_v3_copy` -> `rotor_v3` + `module_rotor_final`). Units mm,
each in its own component frame: **module z = file z - 7.5** (the rotor face is file z 7.5).

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) | Through-holes |
|---|---|---|---|---|---|
| `top.3mf` | upper rotor part (`inner_top`) | 36.000 x 35.996 x 8.200 | 5786.178 | 5786.431 | 8 |
| `bottom.3mf` | lower rotor part, on the servo horn (`inner_bottom`) | 35.000 x 34.995 x 6.200 | 3865.634 | 3865.869 | 8 |
| `rotor.bambu.3mf` | Bambu Studio plate: `top` rotor face down, `bottom` O35 plate down, supports on (tree auto), EXO-294 ids | - | - | - | - |

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
  eight **O3.0 pegs** (r 12.2, 1.2 tall, down to -8.2), the index notch. Bottom: horn bore
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
| `link_12.3mf` | `g_link_12` of `generated_R_arm_general` (J1 rotor -> J2 stator) | 58.498 x 74.514 x 63.529 | 40392.766 | 40398.023 |
| `stator.3mf` | 15 deg stop-block stator + assembly mark, bearing seat **O42.2** (component `stator EXO-341 export`, EXO-363) | 53.999 x 54.000 x 24.100 | 25634.842 | 25638.147 |
| `link.bambu.3mf` | Bambu Studio plate (EXO-343): 2x stator underside down; link_12 rotated (baked) onto the J2 pad's outer face, 58.5 tall; settings = `rotor.bambu.3mf`; built by `tools/bambu_plate_exo343.py` | - | - | - |

- **Stator**: own frame, **module z = file z - 7.5** (underside file z -17.0). Bearing seat
  **O42.2** (r 21.10, file z -3.5..3.5, EXO-363; was O42.10, now
  `archive/2026-10-tests/stator_d42.1_pre_exo363.3mf`), 3 x O4.0 inserts r 19.25 at
  0/180/270, "0" label, assembly mark (EXO-341). Every vertex outside the seat equals the
  previous file.
- **Link**: ARM coordinates. J1 rotor face (rotor pad z 0..8 in J1): centre
  (0, -200.430, 0), axis (0.0002, -0.9612, 0.2757). J2 stator-pad face (J2 z -24.5, pad
  -32.5..-24.5): centre (-24.492, -240.775, -2.817), normal +X. J2 origin
  (0.0076, -240.7704, -2.8187), X_J2 (0, -0.2757, -0.9612), Y_J2 (-0.0002, 0.9612, -0.2757).
  Holes 4 x O3.2 r 11 (J1), 3 x O3.2 r 19.25 at 0/180/270 (J2), cutout x +-10, y -9.5..24.5,
  index tab 3.6 x 2.75 x 1.2 at J1 +Y.
- **Assembly mark** (`armgen.ALIGN_MARK_*`): a 0.6 wide x 0.4 deep radial line at module
  **+Y** (connector side, no insert) on both mating faces: stator underside r 24.5..27 (out
  through the rim), link pad face r 24.5..27.6 (0.6 stays visible outside the stator). Line
  them up, then fit the three M3 screws.

## Link 23 + 4-joint plate (EXO-345, 2026-10-22)

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) |
|---|---|---|---|---|
| `link_23.3mf` | `g_link_23` of `generated_R_arm_general` (J2 rotor -> J3 stator), ARM coordinates, with the assembly mark | 62.504 x 60.246 x 105.699 | 45818.953 | 45829.010 |
| `joints4.bambu.3mf` | Bambu Studio plate: 4x stator (underside down), 4x top (rotor face down), 4x bottom (O35 plate down) as `rotor.bambu.3mf`, + link_23 standing on its J3 flange end face (62.5 tall, J2 pad on supports); settings = `rotor.bambu.3mf`; layout 229.5 x 216.75, margins >= 13.25, gaps >= 10.5; `tools/bambu_plate_exo343.py --plate joints4` | - | - | - |

- **Link 23** (arm coords): J2 rotor face centre (0.008, -240.770, -2.819), normal +X;
  J3 stator-pad face centre (0.004, -247.015, -81.518), normal -Z. Rotor pad 4 x O3.2 r 11,
  tab at J2 +Y, stop block r 20..27, -55..+69 deg; stator pad 3 x O3.2 r 19.25 at
  0/180/270, XL330 cutout; mark at J3 +Y, r 24.5..27.6 (EXO-346).

## Everything on one plate (EXO-348, 2026-10-22)

- `all.bambu.3mf`: **one plate**, 14 objects = `joints4.bambu.3mf` (4x stator, 4x top, 4x bottom, link_23) + link_12 as in `link.bambu.3mf`, same print orientations (meshes identical), no XY rotation; layout 236.24 x 232.51, margins >= 9.88, gaps >= 10.99; settings = `rotor.bambu.3mf`; `tools/bambu_plate_exo343.py --plate all`.

## Stator bearing-seat test (EXO-350/355/358): result O42.2

Note: `link`/`joints4`/`all.bambu.3mf` still carry the O42.1 stator until they are rebuilt (EXO-364).

Seats O41.8..42.6 printed: 42.1 refuses the 5972K159 outer race, **42.2 holds it, tight and removable** -> `stator.3mf` (EXO-363); test stators and their two plates in `archive/2026-10-tests/` (INDEX.md).

## Other files

- `stator.f3d`: stator as it was before EXO-341 (no mark).
- `tool_organizer.3mf`: EXO-258 organizer box.
- `archive/2026-10-tests/`: every rotor test print of October 2026 (original and v3 parts,
  pocket, web, race-gap and journal-diameter tests, their plates, and the previous README),
  one line each in its `INDEX.md`.
