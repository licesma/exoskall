# parts/: print files

## Chest mount (EXO-401, 2026-11-11)

The hand-made chest mount that holds both arms' J1 stators (Fusion `arm complete_chest mount`,
folder `/body mount`, doc v1, one body `Body1`; EXO-399, EXO-400), for a fit test with the
generated J1 stator. High-refinement 3MF export from the Fusion doc, opened read-only and never
saved (`tools/bridge_scripts/export_body_mount_exo401.py`), in the doc frame = arm frame (origin
on the left J1 rotor face). Not the exo STEP copy: its outline differs (EXO-399).

| File | Part | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) | genus |
|---|---|---|---|---|---|
| `body_mount.3mf` | `Body1` of `arm complete_chest mount`: two J1 seats, each a 7.0 pad with 3x O4.0 plain holes at r 19.25 (0/180/270, none on +Y); 8x O3.5 / O6.0 counterbores on the x -45 robot face | 75.000 x 176.430 x 114.148 | 200380.178 | 200387.392 | 18 |
| `body_mount.bambu.3mf` | Bambu plate: the mount on its **x -45 robot bolt face** (15655 mm2 on the bed, counterbores open on the bed); J1 seat pads vertical, their O4.0 holes horizontal; 114.15 x 176.43, 75 tall, centred, margins 70.9 (x) / 39.8 (y); supports on (ceilings 3 / 5 / 7 / 55 above the bed); settings as `link.bambu.3mf` | - | - | - | - |

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
| `link_34.3mf` | `g_link_34` (J3 rotor to J4 stator) | 69.207 x 58.505 x 115.946 | 63225.829 | 63232.554 | 9 |
| `link_45.3mf` | `g_link_45` (J4 rotor to J5 stator) | 102.506 x 62.505 x 64.004 | 62060.991 | 62072.567 | 9 |
| `links34.bambu.3mf` | Bambu plate (EXO-405/406): link_34 **rotor side down** (J3 joint face, 410.2 mm2 on the bed, 115.9 tall; pad underside 8 above the bed on supports), **outer-only brim 6** on this object only; link_45 **rotor side up** (J4 journal vertical, joint face up at 45.4, sockets and journal up, 62.5 tall) on its J5-end face (440.3 mm2); its pad outer face cannot reach the bed (the J5 end sticks out 29.4 past it) and faces down at 29.4 on supports; 175.16 x 71.89, gap 12; **Bambu PLA-CF @BBL X1C, Matcha Green #5C9748**; supports normal(auto), threshold 30, top Z 0.15, 3 top interface layers at 0.2 spacing, global brim off; EXO-409: these process values are listed in `different_settings_to_system`, so Bambu Studio keeps them on open (before, it reset them to the system preset: tree(auto), support off); `tools/bambu_plate_exo343.py --plate links34` | - | - | - | - |
| `joints34.bambu.3mf` | Bambu plate: the two links as above, 2x stator, 2x bottom (pegs up); 214.00 x 137.89, gaps >= 11.99, margins >= 21; same material and settings; `--plate joints34` | - | - | - | - |

link_45 cannot go rotor side down: its J5 stator pad reaches 17.1 mm past the J4 joint face.
Rotor side up (EXO-405) it stands on the J5-end face, centre of mass 23 mm off that foot; the
supports under the pad face (2304.3 mm2 at 29.4, the M3 counterbores open into them) carry it.
link_34 rotor side down: centre of mass 18.9 from the J3 axis (outside the r 15.05 foot, inside the
r 27 pad that rests on supports). To lay it flat instead (J4 pad outer face, 4669.7 mm2, 58.5
tall, J3 journal horizontal), see the comment at `LINKS34_LAYOUT`.

Material (EXO-406): plates built from now on carry Bambu PLA-CF, Matcha Green (preset `Bambu
PLA-CF @BBL X1C`, values from the installed Bambu Studio profiles; the printer preset stays `Bambu
Lab P1S 0.4 nozzle`, listed as compatible by that preset). PLA-CF wants a hardened nozzle
(required_nozzle_HRC 40); the plates still say stainless_steel. Older plates keep PLA Basic.

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
| `link_12.3mf` | `g_link_12` | 58.500 x 79.184 x 63.530 | 44324.563 | 44328.204 | 9 |
| `link_23.3mf` | `g_link_23` | 62.504 x 60.246 x 105.699 | 49751.082 | 49759.191 | 9 |
| `joints2.bambu.3mf` | Bambu plate: link_12 **rotor side down** (joint face, 410.2 mm2 on the bed, 73 tall), link_23 on its J3 flange end face (it cannot go rotor side down: its bar hangs 19 mm past the joint face), 2x stator, 2x bottom (pegs up); 214.00 x 132.84, gaps >= 12 | - | - | - | - |
| `links2.bambu.3mf` | Bambu plate: the two links, same orientations; 176.20 x 66.84, gap 12 | - | - | - | - |

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
| `link.bambu.3mf` | Bambu Studio plate (EXO-343, rebuilt EXO-394): 2x stator underside down; link_12 rotor side down (joint face on the bed, 73 tall); settings = the archived `rotor_exo364.bambu.3mf`; built by `tools/bambu_plate_exo343.py` | - | - | - |

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
| `joints4.bambu.3mf` | Bambu Studio plate: 4x stator (underside down), 4x bottom (O35 plate down), + link_23 standing on its J3 flange end face (62.5 tall, J2 pad on supports); EXO-375/383/394: rebuilt without the four tops (same positions), current link_23; layout 201.5 x 216.75, margins >= 13.25, gaps >= 10.5; `tools/bambu_plate_exo343.py --plate joints4` | - | - | - |

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
  since EXO-375/383 the retired top, bottom, links and rotor plate of EXO-364 (`*_exo364.*`) and the
  EXO-375 pad 7 / pad 6 links (`link_*_p7`, `link_*_p6`), the EXO-383 c1.0 / c1.5 / c2.0 links,
  their plates and bottom (`*_exo383.*`); one line
  each in its `INDEX.md`.
- `bottom_h0.5.3mf`, `bottom_h1.0.3mf`, `rotor_head.bambu.3mf` (EXO-352, unused): variants of the
  EXO-333 bottom; they do not mate with the fused links.
