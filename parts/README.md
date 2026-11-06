# parts/: print files

## Fused link + bottom (EXO-370 / EXO-375, 2026-11-06)

The separate rotor top is gone: every link's rotor pad IS the rotor top (disk, journal, register
pegs, O16 centre hole). The joint's printed parts are the **stator**, the **bottom** (on the servo
horn) and the links. The rotor pad is **7** mm (default, M3x20); a **6** mm variant (M3x18) is
exported for a print test, labelled "p6" (the default "p7"). Fusion High-refinement 3MF exports
from `exo generated` (`tools/bridge_scripts/export_exo370.py`, `export_p6_exo375.py`); links in
ARM coordinates, the bottom in its component frame (**module z = file z - 7.5**).

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) | genus |
|---|---|---|---|---|---|
| `bottom.3mf` | `inner_bottom` of `sensor module 2 rotor`: joint face -8.0, boss r 7.8 to -7.0, journal 3.0, sockets floor -9.3 | 35.000 x 34.995 x 6.200 | 3377.371 | 3377.595 | 8 |
| `link_12_p7.3mf` | `g_link_12`, rotor pad 7, "p7" | 58.499 x 79.711 x 63.530 | 42859.827 | 42864.846 | 9 |
| `link_23_p7.3mf` | `g_link_23`, rotor pad 7, "p7" | 62.504 x 60.245 x 105.699 | 47968.263 | 47977.183 | 9 |
| `link_12_p6.3mf` | `g_link_12`, rotor pad 6, "p6" (print test) | 58.499 x 79.711 x 63.530 | 41522.010 | 41526.872 | 9 |
| `link_23_p6.3mf` | `g_link_23`, rotor pad 6, "p6" (print test) | 62.504 x 60.245 x 105.699 | 46311.994 | 46320.559 | 9 |

The stack, module frame of the joint (rotor face z 0):

| module z | part | what |
|---|---|---|
| **+7.0** (p6: +6.0) | link | rotor pad outer face: the four M3 heads seat here (**M3x20**, p6 **M3x18**), inside the r 18 **head flat** (the 8 mm ribbon is cut down to the pad face inside r 18, and keeps its 8 outside); label "p7"/"p6" 0.4 deep at r 13.8, 45 deg |
| 0 | link | rotor-face plane: the pad's underside r 18..pad_r; link stop block below it, r >= 20, to -3.6 |
| 0 .. -5.001 | link | fused top disk r 18; its underside **-5.001** is the race shoulder (1.0 clamp on the 7.000 race) |
| -5.001 .. **-8.0** | link | journal **O30.1**, **3.0** |
| -8.0 .. -9.2 | link | 8 register pegs O3.0 at r 12.2 (22.5 + k*45) |
| through | link | **O16 centre hole** (driver to the M2 heads; 1.4 wall to the M3 holes), 4x O3.2 M3 at r 11 |
| -7.0 .. -8.0 | bottom | **boss r 7.8** inside the link's O16 hole (0.2 radial); the four **M2x6** horn-screw heads seat on its top -7.0; 0.700 wall from each O2.2 horn hole to the boss edge |
| -8.0 .. -11.001 | bottom | joint face -8.0; journal O30.1, **3.0**; flange shoulder -11.001 (race gap 6.000) |
| -8.0 .. -9.3 | bottom | 8 sockets O3.3 (0.15 radial / 0.1 axial on the pegs) |
| -10.5 | bottom | horn bore ceiling = XL330 horn top; web **3.5** under the heads |
| -10.6 .. -13.2 | bottom | 5.6 AF nut pockets; ISO 4032 nut -10.6 .. -13.0 |
| -12.989 / **-13.0** / -12.0 | screws | M2x6 tip (2.489 into the horn) / M3x20 tip at p7 (= nut bottom) / M3x18 tip at p6 (1.4 of thread) |
| -13.5 | XL330 | servo top (0.5 under the M3x20 tip) |

Assembly: bottom on the horn with the four M2x6 (driver through the link's O16 hole, so they
can also be reached with the link on); the link's pegs into the bottom's sockets; four M3 through
pad + bottom into the nuts. Nothing keys the link's angle beyond the M3 pattern (4 positions,
90 deg apart): set q = 0 at assembly. Old tops / bottoms / links (EXO-333/364) do not mate with
these; they are in `archive/2026-10-tests/` (`*_exo364.*`).

## Link 12 + stator (EXO-341, 2026-10-22)

High-refinement 3MF exports from `exo generated`; read back with `tools/verify_3mf.py`
(watertight, one shell) and vertex rings.

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) |
|---|---|---|---|---|
| `stator.3mf` | 15 deg stop-block stator + assembly mark (component `stator EXO-341 export`) | 53.999 x 54.000 x 24.100 | 25680.834 | 25684.494 |
| `link.bambu.3mf` | Bambu Studio plate (EXO-343, rebuilt EXO-375 with `link_12_p7`): 2x stator underside down; link_12 rotated (baked) onto the J2 pad's outer face, 58.5 tall; settings = the archived `rotor_exo364.bambu.3mf`; built by `tools/bambu_plate_exo343.py` | - | - | - |

- **Stator**: own frame, **module z = file z - 7.5** (underside file z -17.0). Bore O42.10
  (r 21.05), 3 x O4.0 inserts r 19.25 at 0/180/270, "0" label: identical to the previous
  file (`archive/2026-10-tests/stator_pre_exo341.3mf`); only the mark is new.
- **Link**: ARM coordinates. J1 rotor face (rotor pad z 0..8 in J1): centre
  (0, -200.430, 0), axis (0.0002, -0.9612, 0.2757). J2 stator-pad face (J2 z -24.5, pad
  -32.5..-24.5): centre (-24.492, -240.775, -2.817), normal +X. J2 origin
  (0.0076, -240.7704, -2.8187), X_J2 (0, -0.2757, -0.9612), Y_J2 (-0.0002, 0.9612, -0.2757).
  Holes 4 x O3.2 r 11 (J1), 3 x O3.2 r 19.25 at 0/180/270 (J2), cutout x +-10, y -9.5..24.5.
  Since EXO-370 the J1 rotor end is the fused rotor top (table above); the EXO-364 key pegs are gone.
- **Assembly mark** (`armgen.ALIGN_MARK_*`): a 0.6 wide x 0.4 deep radial line at module
  **+Y** (connector side, no insert) on both mating faces: stator underside r 24.5..27 (out
  through the rim), link pad face r 24.5..27.6 (0.6 stays visible outside the stator). Line
  them up, then fit the three M3 screws.

## Link 23 + 4-joint plate (EXO-345, 2026-10-22)

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) |
|---|---|---|---|---|
| `joints4.bambu.3mf` | Bambu Studio plate: 4x stator (underside down), 4x bottom (O35 plate down), + link_23_p7 standing on its J3 flange end face (62.5 tall, J2 pad on supports); EXO-375: rebuilt without the four tops (same positions); layout 201.5 x 216.75, margins >= 13.25, gaps >= 10.5; `tools/bambu_plate_exo343.py --plate joints4` | - | - | - |
| `joints2.bambu.3mf` | Bambu Studio plate (EXO-369, rebuilt EXO-375): link_12_p7, link_23_p7, link_12_p6, link_23_p6, 2x stator, 2x bottom; link_12 on the J2 pad outer face, link_23 on the J3 flange end face, stators underside down, bottoms plate down; supports on, settings = the archived rotor plate; layout 228.24 x 234.40, margins >= 10.80, gaps >= 10.50; `tools/bambu_plate_exo343.py --plate joints2` | - | - | - |

- **Link 23** (arm coords): J2 rotor face centre (0.008, -240.770, -2.819), normal +X;
  J3 stator-pad face centre (0.004, -247.015, -81.518), normal -Z. Rotor pad 4 x O3.2 r 11 and
  the fused rotor top (EXO-370), stop block r 20..27, -55..+69 deg; stator pad 3 x O3.2 r 19.25 at
  0/180/270, XL330 cutout; mark at J3 +Y, r 24.5..27.6 (EXO-346).

## Everything on one plate (EXO-348, 2026-10-22)

- `all.bambu.3mf`: **one plate**, 10 objects = `joints4.bambu.3mf` (4x stator, 4x bottom, link_23_p7) + link_12_p7 as in `link.bambu.3mf`, same print orientations (meshes identical), no XY rotation; EXO-375: rebuilt without the four tops, stator_2/3 moved +2.6 in y (the fused link_12 is 79.7 deep); layout 230.00 x 235.10, margins >= 9.16, gaps >= 10.99; `tools/bambu_plate_exo343.py --plate all`.

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

## Rotor key (EXO-364, 2026-11-04) - superseded by EXO-370

The three key pegs / sockets (and before them the index notch) are gone with the separate top;
the EXO-364 parts are archived as `archive/2026-10-tests/*_exo364.*`.

## Other files

- `stator.f3d`: stator as it was before EXO-341 (no mark).
- `tool_organizer.3mf`: EXO-258 organizer box.
- `archive/2026-10-tests/`: every rotor test print of October 2026 (original and v3 parts,
  pocket, web, race-gap and journal-diameter tests, their plates, and the previous README), and
  since EXO-375 the retired top, bottom, links and rotor plate of EXO-364 (`*_exo364.*`); one line
  each in its `INDEX.md`.
- `bottom_h0.5.3mf`, `bottom_h1.0.3mf`, `rotor_head.bambu.3mf` (EXO-352, unused): variants of the
  EXO-333 bottom; they do not mate with the fused links.
