# parts/: print files

## Wrist side rule: link_56 and link_67 regenerated, plate joints67 rebuilt (EXO-444, 2026-11-16)

The operator hugs the exo from outside, so the wrist cuff goes on the OUTER side of the arm. The
generator had put the J6 module on the outer side of the J7 axis (J7 module x -29.9, its XL330 to
-66.4), where the cuff and the forearm go. New generator rule, for this arm and wrist only
(`armgen.WRIST_OUTER_MAX_MM`, `G1_SPEC['wrist_side']`): where the axis-sign combinations tie (all
four do: 6/6 links, 410.016 mm, slide -29.906), keep J6 on the inner side of the J7 axis (J6 -1,
J7 +1); and nothing from J5 to J7 may reach more than 40 mm to the outer side (outer derived from
the wrist's world y: world -Y = J7 module -X). Measured on the solids: link_67 29.59, J5 27.02,
link_56 27.02, J7 27.00, J6 on the inner side (was J6 66.42, link_56 62.42).

`exo generated` v36 (0 FAIL, race clamp 127.822 x 6, bracket J1..J6 PASS, as v35). link_12 to
link_45 are unchanged in Fusion (volume, area, faces, bounding box, centre of mass identical to v35)
and not re-exported; `stator.3mf`, `bottom.3mf` and every other file are byte-identical. link_56 and
link_67 are their mirror images (same lengths, landings and webs; link_67's tongue -54.378 deg
instead of +54.378), numbers re-placed by the EXO-421 rule. **The link_56 and link_67 printed from
the EXO-425 files are superseded: reprint them.**

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) | genus |
|---|---|---|---|---|---|
| `link_56.3mf` | `g_link_56` (J5 rotor to J6 stator), J6 on the inner side; numbers "5" J5 rim 90 deg, "6" J6 outer face 30 deg | 72.993 x 89.419 x 54.005 | 43475.335 | 43480.825 | 9 |
| `link_67.3mf` | `g_link_67` (J6 rotor to J7 stator), mirrored; numbers "6" J6 rim 270 deg, "7" J7 outer face 30 deg | 102.590 x 67.494 x 59.502 | 43433.865 | 43438.326 | 9 |
| `joints67.bambu.3mf` | Bambu plate as EXO-425: link_56 back left (x 43.7..97.7, y 116.3..205.7, 73.00 tall), link_67 back right (x 109.7..212.3, y 131.3..190.8, 67.50 tall), both **rotor side down** (joint face 410.2 mm2 on the bed; link_67's rotor axis is now arm +Y, so its bed axis is +Y), outer-only 6 mm brim each; 2 stators in front; gaps >= 11.99, margins >= 43.7; standard settings; `--plate joints67` | - | - | - | - |

On the meshes: opening 20.200 x 34.200 at 8 depths x 3 lines (37.2 in y only through the EXO-341
mark groove at z -24.6); rotor end as every link (journal r 15.05, disk r 18.00, recess r 8.00,
centre hole r 6.90, joint face -8.000, socket top -5.900 and O3.30, counterbore O6.19 x 2.0); the
four numbers read correctly from outside (depth raster). Orientation re-checked: rotor side up is
still impossible (no flat face at the far end: 0.0 / 10.1 mm2), rotor side down 410.2 mm2, 73.00 /
67.50 tall, supported 2494.7 / 2224.7 mm2 (EXO-425: 2494.5 / 2224.5), CoM 9.9 / 7.9 outside the
r 15.05 foot, so the 6 mm brim stays.

J7 clearance with the manual 01 + clamp 07 + hand base + enclosure 10 as stand-ins on the real
links (EXO-441 method, 2 mm): J7 -55..+80 at J6 0, -50..+80 for |J6| <= 30, -35..+80 over the full
J6 range; the same as the study. Not sliced, not printed.

## Six stators, plate stators6 (EXO-435, 2026-11-16)

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) | genus |
|---|---|---|---|---|---|
| `stators6.bambu.3mf` | Bambu plate: six copies of the current `stator.3mf` (seat 42.1, unchanged), stator_1..6, as exported (underside on the bed, as on joints34 / joints67), rows of 3 + 3 at 64.5 pitch: x 36.5..219.5, y 68.75..187.25, 24.1 tall; gaps 10.50, margins >= 36.5, exclude zone clear; each mesh block md5 = the one generated from `stator.3mf` (25680.834 mm3); standard settings (PLA-CF Matcha Green, normal supports 30 / 0.15 / 3 / 0.2, global brim off, `different_settings_to_system`); `--plate stators6` | - | - | - | - |

## Chest mount with hooks, plate chest_mount (EXO-429, 2026-11-14)

The chest mount to print. It is the NEWER version (EXO-427 version B): component `chest mount`
(Body14) of `arm complete` v35, one body with 4 J-hooks at the ends (purpose not recorded), no
pocket on the robot face. Copied (body copy, identity occurrence, arm frame kept, so the EXO-399 /
EXO-400 seat results hold) into its own Fusion document `/body mount/chest mount hooks 20.2`
(base feature + one Offset Faces feature, saved; `arm complete` untouched). Both J1 actuator
openings widened 20.0 x 34.0 -> **20.2 x 34.2** (0.1 per side, the fit proven on the links,
EXO-417/419), through the full 7.0 pad (`tools/bridge_scripts/chest_mount_exo429.py`).

Measured on the mesh: 20.200 x 34.200 at 9 depths through each pad; against Body14 exported the
same way, 16 of 18120 vertices differ (the 8 opening corners per pad, moved 0.1 + 0.1), triangles
identical, everything else bit-identical (seat holes 3x O4.000 at r 19.250, pads 24.5..31.5,
hooks, the 8 board holes). Volume -151.760 mm3 in Fusion = 2 x (20.2 x 34.2 - 20 x 34) x 7.
Thinnest wall round each opening: 3.40 to the pad rim at mid depth (was 3.53), 1.83 at the pad
faces where the rim is rounded (was 1.95); 7.15 / 7.65 to the O4.0 holes.

The 8 holes on the robot (x -45) face, O3.5 with O6.0 x 3.0 counterbores, carry two Dynamixel
reader boards, 4 holes each (Esteban, 2026-11-14), y -50.47 / -92.47 and -107.97 / -149.97, z
-4.16 / -52.16 (-4.26 / -52.26 on the right board). Unchanged.

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) | genus |
|---|---|---|---|---|---|
| `chest_mount.3mf` | `chest mount` of `/body mount/chest mount hooks 20.2` (Body14 of `arm complete` v35, openings 20.2 x 34.2), arm frame | 75.000 x 200.430 x 114.956 | 212133.928 | 212140.720 | 18 |
| `chest_mount.bambu.3mf` | Bambu plate: as the EXO-401 plate, the x -45 board face down (16929.5 mm2 on the bed, counterbores open on the bed), pads vertical; 114.96 x 200.43, 75 tall, centred, margins 70.52 / 27.78; standard settings (PLA-CF Matcha Green, normal supports, global brim off), 2 walls and 15% infill on the part, and two **modifier boxes** (one per J1 pad and its root) with **4 wall loops and 40% sparse infill**; `--plate chest_mount` | - | - | - | - |

Supports (threshold 30): the two opening roofs (34.2 x 7 each, 478.8 mm2) at 55.1 above the bed,
supports standing on the opening floors at 34.9 (the 20.2 is the vertical size); a 50 x 15 recess
in the bed face (722 mm2 ceiling at 5); the 8 counterbore ceilings at 3 (149 mm2); the tops of the
horizontal O4.0 seat holes (small). The hooks stand on the bed and curl upward: no support beyond
their 0.5 mm bottom edge round.

Pad modifiers (Esteban: dense only on the pads), plate coordinates, full height z 0..75.5:
`pad_left_dense` x 91.05..175.55, y 177.215..216.215; `pad_right_dense` x 91.05..175.55, y
39.785..78.785 (84.5 x 39.0 each). Each = the pad root footprint at the base plate (10 thick)
plus 8 mm into the plate, clamped at the plate end (hooks left out); it also takes in part of the
board hole columns at y -50.47 / -149.97. Stored as Bambu saves them (type "other" mesh +
`modifier_part` with its own wall_loops / sparse_infill_density, part config, not preset values).
Rough estimate (slices, solid shells ignored): standard 60 g, with the modifiers 98 g (+38 g),
whole part at 4 walls / 30% 113 g (+53 g), whole part at 4 / 40% 134 g (+74 g).

## Links 56 and 67, plate joints67 (EXO-425, 2026-11-14)

Superseded by EXO-444 (top of this file): both links regenerated with J6 on the inner side; the
numbers below are the EXO-425 files.

First export of the last two right arm links, from `exo generated` v35 (the EXO-421 build: opening
20.2 x 34.2, joint numbers), High refinement, arm coordinates, export only (document unmodified,
not saved; `tools/bridge_scripts/export_exo425.py`). Same rotor end as the other links (EXO-394:
pad 8, O6.2 x 2.0 counterbores, eight O3.3 x 2.1 sockets, journal O30.1, recess r 8, O13.8 centre
hole; probed in Fusion and on the mesh). Opening 20.200 x 34.200 at 8 depths through the pad
(37.2 in y only through the EXO-341 mark groove on link_56, as on link_12..45). Thinnest wall round
the opening (full depth): link_56 3.81, link_67 3.00 (same as link_34). Numbers read correctly on
the mesh: link_56 "5" J5 rim 125 deg, "6" J6 outer face 150 deg; link_67 "6" J6 rim 270 deg, "7" J7
outer face 30 deg.

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) | genus |
|---|---|---|---|---|---|
| `link_56.3mf` | `g_link_56` (J5 rotor to J6 stator, wrist roll, in-plane landing, 25 wide); numbers 5 / 6 | 73.015 x 89.407 x 54.001 | 43476.084 | 43480.828 | 9 |
| `link_67.3mf` | `g_link_67` (J6 rotor to J7 stator, wrist pitch, angled landing, 21 wide); numbers 6 / 7 | 102.580 x 67.508 x 59.504 | 43433.710 | 43438.693 | 9 |
| `joints67.bambu.3mf` | Bambu plate: link_56 BACK LEFT (x 43.7..97.7, y 116.3..205.7, 73.00 tall), link_67 BACK RIGHT (x 109.7..212.3, y 131.2..190.8, 67.50 tall), both **rotor side down** (joint face 410.2 mm2 on the bed, journal vertical, outer-only 6 mm brim each); stator_1 FRONT LEFT (x 67..121), stator_2 FRONT RIGHT (x 133..187), y 50.3..104.3, as exported (as joints34), current `stator.3mf` (seat 42.1); gaps >= 12.00, margins >= 43.7; standard settings (PLA-CF Matcha Green, normal supports 30 / 0.15 / 3 / 0.2, global brim off, `different_settings_to_system`); `--plate joints67` | - | - | - | - |

Rotor side up is impossible on both: the far end along the journal axis is the round rim of the
stator pad, no flat face to stand on. Rotor side down they stand like link_12 / link_34; the centre
of mass is 24.7 (link_56) / 23.7 (link_67) from the journal axis, 9.6 / 8.6 outside the r 15.05
foot, carried by the supports under the pad. Supported: pad and ribbon underside 8 above the bed
(1591.7 / 918.7 mm2), the stop tabs at 4.4 (341.6 / 459.3), shoulder, sockets and recess (232.5),
the opening roof (link_56 20.2 x 8 at 55.6, link_67 34.2 x 8 at 48.0). Numbers on vertical faces,
not on supports. Not sliced, not printed.

## Joint numbers on every link, plate links345 (EXO-421, EXO-422, 2026-11-13)

Every link carries the number of each joint it meets, one per end: link_12 "1" and "2", link_23
"2" and "3", link_34 "3" and "4", link_45 "4" and "5" (link_56, link_67 in Fusion likewise), so
the two parts meeting at a joint show the same number. Generator feature (`armgen.JOINT_LABEL_*`,
`BuildArmGen._joint_labels`): Arial Black 4.5 high, debossed 0.4. Rotor end: on the round edge
(rim) of the rotor pad, mid height. Stator end: on the stator pad's outer face (the side the M3
heads go in from), r 20 between an insert and the opening. Never on a mating or functional face.
Both faces are vertical on every plate, so no number prints on supports. Digit up = world up at
q = 0 where the face allows it. ~99 % of each groove survives a 0.4 nozzle (mesh raster).

| Link | rotor end number | stator end number |
|---|---|---|
| link_12 | "1" J1 rim 345 deg, faces front | "2" J2 outer face 210 deg, faces back |
| link_23 | "2" J2 rim 285 deg, faces right | "3" J3 outer face 300 deg, faces up |
| link_34 | "3" J3 rim 135 deg, faces front-right | "4" J4 outer face 150 deg, faces the torso |
| link_45 | "4" J4 rim 35 deg, faces front-up | "5" J5 outer face 30 deg, faces back |

`exo generated` v35 (0 FAIL, race clamp 127.822, bracket J1..J6 PASS). Links re-exported
(`export_exo421.py`); each differs from its EXO-419 file only on the two labelled faces (bbox
equal, opening still 20.200 x 34.200). Volumes: link_12 44229.471, link_23 49685.932, link_34
63325.391, link_45 62020.286. Plates with links rebuilt (link, joints4, all, joints2, links2,
links34, joints34): positions, orientations, settings unchanged.

| File | What | Footprint | Settings |
|---|---|---|---|
| `links345.bambu.3mf` | link_23 FRONT centre (x 75.2..180.9, y 55.9..116.2; J3 flange end face down, as links2); link_34 BACK LEFT (x 40.4..101.1, y 128.1..200.2; rotor side down, as links34); link_45 BACK RIGHT (x 113.1..215.6, y 132.1..196.1; rotor side up, as links34); gaps >= 11.90, margins >= 40.4; `--plate links345` | 175.16 x 144.26 | PLA-CF Matcha Green, normal supports 30 / 0.15 / 3 / 0.2, global brim off, link_34 outer brim 6, link_23 its own auto brim (as on links2), `different_settings_to_system` |

## Link actuator opening 20.2 x 34.2 (EXO-419, 2026-11-13)

The EXO-417 fit test settled it: Esteban printed `link12_open` (PLA-CF, normal supports, rotor
side down) and "0.2 works fine", i.e. the link with 0.2 TOTAL clearance, 0.1 per side. That is now
the design default on every link: `armgen.ACT_OPEN_MARGIN` 0.2 -> 0.1 (links only; `stator.3mf` and
`body_mount` stay 20.0 x 34.0, untouched). The landing solve keeps its 3.0 web, so it gives back
half of what EXO-415 moved: link_23 and link_45 land 0.1 less far out (62.6 tall on the plates),
link_34's landing disk r 27.834 -> 27.630 (116.15 tall). link_12 changes only at the opening
walls and is the tested `link_12_open0.2` without its label (every vertex, bbox equal).
`exo generated` rebuilt (0 FAIL, race clamp 127.822, bracket J1..J6 PASS) and saved v34; links
re-exported with `tools/bridge_scripts/export_exo419.py`; every plate with a link rebuilt, layouts,
orientations and settings unchanged. Measured on every mesh: 20.200 x 34.200 at 8 depths through
the 8.0 pad. Thinnest plate round the opening (full pad depth): link_12 3.62, link_23 3.81,
link_34 3.00, link_45 7.55 (M3 hole edge; 6.09 near the pad face). The fit-test files and the
`link12_open` plate are in `archive/2026-10-tests/` (INDEX).

| File | V at EXO-415 (mm3) | V now (mm3) | Fusion V | change |
|---|---|---|---|---|
| `link_12.3mf` | 44150.533 | 44237.869 | 44241.508 | +87.336, opening walls only |
| `link_23.3mf` | 49637.858 | 49694.996 | 49702.823 | opening, bar 0.1 higher (tongue end y 35.6) |
| `link_34.3mf` | 63440.986 | 63334.118 | 63340.638 | opening, landing disk r 27.630 |
| `link_45.3mf` | 62000.328 | 62029.693 | 62041.359 | opening, bar 0.1 higher (tongue end y 35.6) |

## link_12 opening fit test, 0.2 / 0.4 total clearance (EXO-417, 2026-11-13; archived by EXO-419)

Result (Esteban, 2026-11-13): the "0.2" link fits; it became the default (EXO-419). The files below
now live in `archive/2026-10-tests/` (the plate as `link12_open_exo417.bambu.3mf`); the
`--plate link12_open` entry is retired.

Two link_12 variants to find the opening the XL330 passes. Named by the TOTAL clearance over the
XL330's 20.0 x 34.0: "0.2" = `ACT_OPEN_MARGIN` 0.1 per side, "0.4" = 0.2 per side (the EXO-415
default, = `link_12.3mf` at the time). Built in `exo generated` through the bridge without saving
(`tools/bridge_scripts/run_open0.2_exo417.py` / `run_open0.4_exo417.py`, export
`export_link12_open_exo417.py`), then the document was closed unsaved and reopened at v33.
Each carries its label debossed 0.4 deep, Arial Black 3.0, on the rotor pad's outer face
(z +8, between the M3 counterbores at 0 and 90 deg), which is UP on the plate; 98.5 % of the
label area survives a 0.4 mm opening. `link_12_open0.4` = `link_12.3mf` + the label (every
`link_12.3mf` vertex is in it); `link_12_open0.2` differs from it only at the 12 opening-wall
vertices and the label. Nothing else moves at margin 0.1 for link_12.

| File | Part | Opening (mesh, 8 depths) | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) | genus |
|---|---|---|---|---|---|---|
| `link_12_open0.2.3mf` | `g_link_12`, margin 0.1, label "0.2" | 20.200 x 34.200 | 58.500 x 79.184 x 63.530 | 44233.633 | 44237.266 | 9 |
| `link_12_open0.4.3mf` | `g_link_12`, margin 0.2, label "0.4" | 20.400 x 34.400 | 58.500 x 79.184 x 63.530 | 44146.411 | 44150.046 | 9 |
| `link12_open.bambu.3mf` | Bambu plate: both, rotor side down exactly as link_12 on links2 (joint face 410.2 mm2 on the bed, 73 tall, labels up); **"0.2" on the LEFT** (x 63.5..122.0), **"0.4" on the RIGHT** (x 134.0..192.5), y 94.58..161.42, gap 12; object names `link_12_open0.2` / `link_12_open0.4`; settings byte-identical to links2 (PLA-CF Matcha Green, normal supports, global auto_brim kept); `tools/bambu_plate_exo343.py --plate link12_open` | - | - | - | - | - |

## Link actuator opening 20.4 x 34.4 (EXO-415, 2026-11-12)

The actuator would not pass the printed links (EXO-413: every opening was the XL330's own
20.0 x 34.0, zero clearance). Esteban's call: 0.2 per side on the links. `armgen.ACT_OPEN_MARGIN`
0.0 -> 0.2 (only the generated links read it; `stator.3mf` and `body_mount` stay 20.0 x 34.0 and
were not touched). The landing solve keeps its 3.0 web, so besides the opening: link_23 and link_45
land 0.2 further out (their bar is 0.2 longer, 62.7 tall on the plates), link_34's landing disk
grows r 27.424 -> 27.834 (116.4 tall). link_12 changes only at the opening walls.
`exo generated` rebuilt (0 FAIL, bracket J1..J6 PASS) and saved v33; links re-exported with
`tools/bridge_scripts/export_exo415.py`; every plate with a link rebuilt, layouts, orientations
and settings unchanged. Measured on every mesh: 20.400 x 34.400 through the whole 8.0 pad.
Thinnest plate round the opening (full pad depth): link_12 3.49, link_23 3.68, link_34 3.00,
link_45 6.0; M3 hole to opening 7.45 (0/180 deg) and 7.95 (270 deg).

| File | V before (mm3) | V now (mm3) | Fusion V | change |
|---|---|---|---|---|
| `link_12.3mf` | 44324.563 | 44150.533 | 44154.172 | -174.030 = the 0.2 frame x 8.0 deep, less 0.048 of mark groove |
| `link_23.3mf` | 49751.082 | 49637.858 | 49645.770 | opening, bar 0.2 lower |
| `link_34.3mf` | 63225.829 | 63440.986 | 63447.633 | opening, landing disk r 27.834 |
| `link_45.3mf` | 62060.991 | 62000.328 | 62009.438 | opening, bar 0.2 lower |

## Chest mount, older version (EXO-401, 2026-11-11; archived by EXO-429)

Superseded by `chest_mount` above. Both files are now in `archive/2026-10-tests/` as
`body_mount_exo401.3mf` / `body_mount_exo401.bambu.3mf`; the `--plate body_mount` entry is retired.

The hand-made chest mount that holds both arms' J1 stators (Fusion `arm complete_chest mount`,
folder `/body mount`, doc v1, one body `Body1`; EXO-399, EXO-400), for a fit test with the
generated J1 stator. High-refinement 3MF export from the Fusion doc, opened read-only and never
saved (`tools/bridge_scripts/export_body_mount_exo401.py`), in the doc frame = arm frame (origin
on the left J1 rotor face). Not the exo STEP copy: its outline differs (EXO-399).

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) | genus |
|---|---|---|---|---|---|
| `body_mount.3mf` | `Body1` of `arm complete_chest mount`: two J1 seats, each a 7.0 pad with 3x O4.0 plain holes at r 19.25 (0/180/270, none on +Y); 8x O3.5 / O6.0 counterbores on the x -45 robot face | 75.000 x 176.430 x 114.148 | 200380.178 | 200387.392 | 18 |
| `body_mount.bambu.3mf` | Bambu plate: the mount on its **x -45 robot bolt face** (15655 mm2 on the bed, counterbores open on the bed); J1 seat pads vertical, their O4.0 holes horizontal; 114.15 x 176.43, 75 tall, centred, margins 70.9 (x) / 39.8 (y); supports on (ceilings 3 / 5 / 7 / 55 above the bed); EXO-411: standard settings (PLA-CF Matcha Green, normal supports, global brim off, see Material below), geometry unchanged | - | - | - | - |

Fit test: M3 from the mount side into the stator's three heat-set inserts (stator underside on
the pad). Screw length is modelled nowhere (EXO-399).

## Links 34 and 45 (EXO-402, 2026-11-12)

The next two right arm links, same generation as link_12 / link_23 (EXO-394: pad 8, O6.2 x 2.0
counterbores, eight O3.3 x 2.1 sockets, chamfered shoulder; probed in Fusion, identical numbers).
High-refinement 3MF exports of `g_link_34` / `g_link_45` from `exo generated` v32 (opened
read-only, closed unsaved; `tools/bridge_scripts/export_exo402.py`), in ARM coordinates. They mate
with `stator.3mf` and `bottom.3mf` (EXO-394), not with EXO-383 parts.

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) | genus |
|---|---|---|---|---|---|
| `link_34.3mf` | `g_link_34` (J3 rotor to J4 stator); EXO-419: opening 20.2 x 34.2, landing disk r 27.630; EXO-421: numbers 3 / 4 | 69.414 x 58.505 x 116.152 | 63325.391 | 63331.790 | 9 |
| `link_45.3mf` | `g_link_45` (J4 rotor to J5 stator); EXO-419: opening 20.2 x 34.2, bar 0.1 lower than EXO-394; EXO-421: numbers 4 / 5 | 102.506 x 62.605 x 64.004 | 62020.286 | 62031.881 | 9 |
| `links34.bambu.3mf` | Bambu plate (EXO-405/406): link_34 **rotor side down** (J3 joint face, 410.2 mm2 on the bed, 116.15 tall since EXO-419; pad underside 8 above the bed on supports), **outer-only brim 6** on this object only; link_45 **rotor side up** (J4 journal vertical, joint face up at 45.4, sockets and journal up, 62.6 tall since EXO-419) on its J5-end face (435.8 mm2; 431.3 at EXO-415, 440.3 before); its pad outer face cannot reach the bed (the J5 end sticks out 29.4 past it) and faces down at 29.4 on supports; 175.16 x 72.10, gap 12; **Bambu PLA-CF @BBL X1C, Matcha Green #5C9748**; supports normal(auto), threshold 30, top Z 0.15, 3 top interface layers at 0.2 spacing, global brim off; EXO-409: these process values are listed in `different_settings_to_system`, so Bambu Studio keeps them on open (before, it reset them to the system preset: tree(auto), support off); `tools/bambu_plate_exo343.py --plate links34` | - | - | - | - |
| `joints34.bambu.3mf` | Bambu plate: the two links as above, 2x stator, 2x bottom (pegs up); 214.00 x 137.99, gaps >= 11.89, margins >= 21 (EXO-419); same material and settings; `--plate joints34` | - | - | - | - |

link_45 cannot go rotor side down: its J5 stator pad reaches 17.1 mm past the J4 joint face.
Rotor side up (EXO-405) it stands on the J5-end face, centre of mass 23 mm off that foot; the
supports under the pad face (2304.3 mm2 at 29.4, the M3 counterbores open into them) carry it.
link_34 rotor side down: centre of mass 18.9 from the J3 axis (outside the r 15.05 foot, inside the
r 27 pad that rests on supports). To lay it flat instead (J4 pad outer face, 4669.7 mm2, 58.5
tall, J3 journal horizontal), see the comment at `LINKS34_LAYOUT`.

Material (EXO-406): plates carry Bambu PLA-CF, Matcha Green (preset `Bambu PLA-CF @BBL X1C`,
values from the installed Bambu Studio profiles; the printer preset stays `Bambu Lab P1S 0.4
nozzle`, listed as compatible by that preset). PLA-CF wants a hardened nozzle
(required_nozzle_HRC 40); the plates still say stainless_steel.

Standard settings on every plate in `parts/` (EXO-411 rebuilt the nine older ones from PLA Basic
and tree supports; meshes and positions unchanged): PLA-CF as above, supports normal(auto),
threshold 30, not build-plate-only, top Z 0.15, 3 top interface layers at 0.2 spacing, all listed
in `different_settings_to_system`. Brim: global off on links34, joints34, chest_mount, stator_dia,
stator_dia_low, rotor_head (link_34 outer-only 6). The plates with link_12 or link_23 (link,
joints4, all, joints2, links2) keep their previous global auto_brim until Esteban decides.

## Fused link + bottom (EXO-370 / EXO-394, 2026-11-06)

The separate rotor top is gone: every link's rotor pad IS the rotor top. The joint's printed
parts are the **stator**, the **bottom** (on the servo horn, now carrying the eight register
**pegs**) and the links (pad 8, **O6.2 x 2.0** M3 head counterbores, eight **O3.3 x 2.1 sockets**,
disk underside: flat shoulder r 15.05..16.2 + 45 deg chamfer). One variant, plain names (EXO-394;
the EXO-383 c1.0 / c1.5 / c2.0 test links are archived). Fusion High-refinement 3MF exports from
`exo generated` (`tools/bridge_scripts/export_exo370.py`); links in ARM coordinates, the bottom in
its component frame (**module z = file z - 7.5**).

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) | genus |
|---|---|---|---|---|---|
| `bottom.3mf` | `inner_bottom` of `sensor module 2 rotor`: joint face -8.0, 8 pegs O3.0 x 2.0 to -6.0, boss r 7.8 to -7.0, journal 3.0 | 35.000 x 34.995 x 7.200 | 3578.998 | 3579.643 | 8 |
| `link_12.3mf` | `g_link_12`; EXO-419: opening 20.2 x 34.2; EXO-421: numbers 1 / 2 | 58.500 x 79.184 x 63.530 | 44229.471 | 44233.048 | 9 |
| `link_23.3mf` | `g_link_23`; EXO-419: opening 20.2 x 34.2, bar 0.1 lower than EXO-394; EXO-421: numbers 2 / 3 | 62.604 x 60.246 x 105.702 | 49685.932 | 49693.659 | 9 |
| `joints2.bambu.3mf` | Bambu plate: link_12 **rotor side down** (joint face, 410.2 mm2 on the bed, 73 tall), link_23 on its J3 flange end face (it cannot go rotor side down: its bar hangs 19 mm past the joint face), 2x stator, 2x bottom (pegs up); 214.00 x 132.84, gaps >= 12; EXO-419: rebuilt with the 20.2 x 34.2 links | - | - | - | - |
| `links2.bambu.3mf` | Bambu plate: the two links, same orientations; 176.20 x 66.84, gap 12; EXO-419: rebuilt with the 20.2 x 34.2 links (link_23 62.6 tall on 329.9 mm2), standard settings, global auto_brim kept; EXO-421: rebuilt with the numbered links; **print this one first** | - | - | - | - |

The stack, module frame of the joint (rotor face z 0):

| module z | part | what |
|---|---|---|
| **+8.0** | link | rotor pad outer face (pad 8, one plane, no label) |
| **+6.0** | link | floor of the four **O6.2 x 2.0** M3 head counterbores (r 11); the M3 head (O5.5 x 3.0) seats here |
| 0 | link | rotor-face plane: the pad's underside r 18..pad_r; link stop block below it, r >= 20, to -3.6 |
| -3.201 .. -5.001 | link | disk rim r 18 at -3.201, **45 deg chamfer** down to r 16.2, **flat race shoulder r 15.05..16.2 at -5.001** (1.0 clamp on the 7.000 race; clear of the shield ring r 16.181..18.864, EXO-337) |
| -5.001 .. **-8.0** | link | journal **O30.1**, **3.0** |
| -8.0 .. -5.9 | link | 8 register **sockets O3.3 x 2.1** at r 12.2 (22.5 + k*45) |
| -8.0 .. -5.4 | link | underside **recess r 8.0** over the boss and the M2 heads (heads to -5.647, r 7.63: 0.37 radial, 0.25 axial) |
| through | link | **O13.8 centre hole** (1.0 to each counterbore; partial driver access to the M2 heads), 4x O3.2 M3 at r 11 (1.4 to the recess) |
| -8.0 .. -6.0 | bottom | 8 register **pegs O3.0 x 2.0** (0.15 radial / 0.1 axial in the sockets) |
| -7.0 .. -8.0 | bottom | **boss r 7.8** (0.2 in the recess); the four **M2x6** horn-screw heads seat on its top -7.0; 0.700 wall from each O2.2 horn hole to the boss edge |
| -8.0 .. -11.001 | bottom | joint face -8.0; journal O30.1, **3.0**; flange shoulder -11.001 (race gap 6.000) |
| -10.5 | bottom | horn bore ceiling = XL330 horn top; web **3.5** under the heads |
| -10.6 .. -13.2 | bottom | 5.6 AF nut pockets; ISO 4032 nut -10.6 .. -13.0 |
| -12.0 | M3x18 | tip from the 2.0 counterbore (1.4 into the nut); M3x20 would end at **-14.0, 0.5 into the servo** |
| -12.989 | M2x6 | horn-screw tip (2.489 into the horn) |
| -13.5 | XL330 | servo top |

Assembly: bottom on the horn with the four M2x6 **first** (the O13.8 hole gives only partial
driver access to the heads); the bottom's pegs into the link's sockets; four M3 through pad +
bottom into the nuts, heads in the counterbores. **Screw: M3x18** (tip -12.0); an M3x20 from the
2.0 counterbore ends at -14.0, 0.5 into the servo. Nothing keys the link's angle beyond the M3
pattern (4 positions, 90 deg apart): set q = 0 at assembly. Older tops / bottoms / links
(EXO-333/364/375/383) do not mate with these; they are in `archive/2026-10-tests/`.

## Link 12 + stator (EXO-341, 2026-10-22)

High-refinement 3MF exports from `exo generated`; read back with `tools/verify_3mf.py`
(watertight, one shell) and vertex rings.

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) |
|---|---|---|---|---|
| `stator.3mf` | 15 deg stop-block stator + assembly mark (component `stator EXO-341 export`) | 53.999 x 54.000 x 24.100 | 25680.834 | 25684.494 |
| `link.bambu.3mf` | Bambu Studio plate (EXO-343, rebuilt EXO-394): 2x stator underside down; link_12 rotor side down (joint face on the bed, 73 tall); settings = the archived `rotor_exo364.bambu.3mf` + the standard overrides (EXO-411, global auto_brim kept); built by `tools/bambu_plate_exo343.py` | - | - | - |

- **Stator**: own frame, **module z = file z - 7.5** (underside file z -17.0). Bore O42.10
  (r 21.05), 3 x O4.0 inserts r 19.25 at 0/180/270, "0" label: identical to the previous
  file (`archive/2026-10-tests/stator_pre_exo341.3mf`); only the mark is new.
- **Link**: ARM coordinates. J1 rotor face (rotor pad z 0..8 in J1): centre
  (0, -200.430, 0), axis (0.0002, -0.9612, 0.2757). J2 stator-pad face (J2 z -24.5, pad
  -32.5..-24.5): centre (-24.492, -240.775, -2.817), normal +X. J2 origin
  (0.0076, -240.7704, -2.8187), X_J2 (0, -0.2757, -0.9612), Y_J2 (-0.0002, 0.9612, -0.2757).
  Holes 4 x O3.2 r 11 (J1), 3 x O3.2 r 19.25 at 0/180/270 (J2), cutout x +-10, y -9.5..24.5 (EXO-419: x +-10.1, y -9.6..24.6 on the generated links; EXO-415 had +-10.2, -9.7..24.7).
  Since EXO-370 the J1 rotor end is the fused rotor top (table above); the EXO-364 key pegs are gone.
- **Assembly mark** (`armgen.ALIGN_MARK_*`): a 0.6 wide x 0.4 deep radial line at module
  **+Y** (connector side, no insert) on both mating faces: stator underside r 24.5..27 (out
  through the rim), link pad face r 24.5..27.6 (0.6 stays visible outside the stator). Line
  them up, then fit the three M3 screws.

## Link 23 + 4-joint plate (EXO-345, 2026-10-22)

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) |
|---|---|---|---|---|
| `joints4.bambu.3mf` | Bambu Studio plate: 4x stator (underside down), 4x bottom (O35 plate down), + link_23 standing on its J3 flange end face (62.5 tall, 62.6 since EXO-419, J2 pad on supports); EXO-375/383/394: rebuilt without the four tops (same positions), current link_23; layout 201.5 x 216.75, margins >= 13.25, gaps >= 10.5; `tools/bambu_plate_exo343.py --plate joints4` | - | - | - |

- **Link 23** (arm coords): J2 rotor face centre (0.008, -240.770, -2.819), normal +X;
  J3 stator-pad face centre (0.004, -247.015, -81.518), normal -Z. Rotor pad 4 x O3.2 r 11 and
  the fused rotor top (EXO-370), stop block r 20..27, -55..+69 deg; stator pad 3 x O3.2 r 19.25 at
  0/180/270, XL330 cutout; mark at J3 +Y, r 24.5..27.6 (EXO-346).

## Everything on one plate (EXO-348, 2026-10-22)

- `all.bambu.3mf`: **one plate**, 10 objects = `joints4.bambu.3mf` (4x stator, 4x bottom, link_23) + link_12 rotor side down as in `link.bambu.3mf`, no XY rotation; EXO-375/394: rebuilt without the four tops, stator_2/3 moved +2.6 in y; layout 230.00 x 232.51, margins >= 9.88, gaps >= 10.99; `tools/bambu_plate_exo343.py --plate all`.

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
| `stator_dia.bambu.3mf` | Bambu plate: the five, underside down, bore + label up, rows of 3 + 2, gaps 10.5; standard settings (EXO-411); `tools/bambu_plate_exo343.py --plate stator_dia` | - | - | - |
| `stator_dia_low.bambu.3mf` | Bambu plate (EXO-358): stator_d41.8 .. 42.4 (7), underside down, rows of 3 + 3 + 1, gaps 10.5, margins 36.5; standard settings (EXO-411); `tools/bambu_plate_exo343.py --plate stator_dia_low` | - | - | - |

## Rotor key (EXO-364, 2026-11-04) - superseded by EXO-370

The three key pegs / sockets (and before them the index notch) are gone with the separate top;
the EXO-364 parts are archived as `archive/2026-10-tests/*_exo364.*`.

## Other files

- `stator.f3d`: stator as it was before EXO-341 (no mark).
- `tool_organizer.3mf`: EXO-258 organizer box.
- `archive/2026-10-tests/`: every rotor test print of October 2026 (original and v3 parts,
  pocket, web, race-gap and journal-diameter tests, their plates, and the previous README), and
  since EXO-375/383 the retired top, bottom, links and rotor plate of EXO-364 (`*_exo364.*`) and the
  EXO-375 pad 7 / pad 6 links (`link_*_p7`, `link_*_p6`), the EXO-383 c1.0 / c1.5 / c2.0 links,
  their plates and bottom (`*_exo383.*`); one line
  each in its `INDEX.md`.
- `bottom_h0.5.3mf`, `bottom_h1.0.3mf`, `rotor_head.bambu.3mf` (EXO-352, unused): variants of the
  EXO-333 bottom; they do not mate with the fused links.
