# parts/: print files

All files are Fusion native 3MF exports of one body (except the `inner_bottom` 5.4/5.6 meshes, EXO-297 mesh booleans, and the `*.bambu.3mf` plates), in millimetres, in the part's own
component frame (module z = file z - 7.5; the rotor face is file z 7.5). They are not laid
out for printing: orient them in the slicer. Numbers below were read back from the 3MF
files with `python3 -I tools/verify_3mf.py parts/*.3mf` (EXO-265). "Fusion V" is the
B-rep volume read in Fusion on 2026-10-09.

## Rotor: two designs, side by side

| File | Design | Fusion body | bbox (mm) | Mesh V (mm3) | Fusion V (mm3) | Through-holes |
|---|---|---|---|---|---|---|
| `manual_inner_top.3mf` | original `sensor module 2` (hand-made, G1 leader) | `Component2` | 36.000 x 35.996 x 9.000 | 6472.861 | 6473.293 | 8 |
| `manual_inner_bottom.3mf` | original `sensor module 2` | `Component3` | 34.998 x 34.999 x 4.200 | 2950.379 | 2950.793 | 4 |
| `inner_top.3mf` | `sensor module 2 v3` (EXO-225/238/240) | `inner_top` | 36.000 x 35.999 x 10.200 | 6657.892 | 6657.964 | 8 |
| `inner_bottom.3mf` | `sensor module 2 v3`, **5.6 AF pocket** (EXO-301; = `inner_bottom_p56`, debossed "56") | mesh boolean (EXO-297) | 34.998 x 34.999 x 4.200 | 2524.065 | - | 8 |
| `inner_bottom_p54.3mf` | v3 test variant, 5.4 AF pocket, debossed "54" (EXO-296/297) | mesh boolean | 34.998 x 34.999 x 4.200 | 2544.407 | - | 8 |
| `inner_bottom_p56.3mf` | v3 test variant, 5.6 AF pocket, debossed "56" (EXO-296/297); copied to `inner_bottom.3mf` | mesh boolean | 34.998 x 34.999 x 4.200 | 2524.065 | - | 8 |
| `inner_bottom_p58.3mf` | v3 previous print file, 5.8 AF pocket (EXO-240), kept for the record | `inner_bottom` | 34.998 x 34.999 x 4.200 | 2507.250 | 2507.386 | 8 |
| `top_3.5mm.3mf` | web-3.5 pair (EXO-304/310): spigot z -7.0..-4 (3.0), pegs to -8.2, debossed "3.5" on the joint face | `top_3.5mm` | 36.000 x 35.996 x 8.200 | 5462.709 | 5462.942 | 8 |
| `bottom_3.5mm.3mf` | web-3.5 pair: boss -11.001..-7.0 (journal 4.0), **web 3.5** (-10.5..-7.0), debossed "3.5" | `bottom_3.5mm` | 35.000 x 34.995 x 6.200 | 3844.186 | 3844.258 | 8 |
| `top_4mm.3mf` | web-4 pair: spigot z -6.5..-4 (2.5), pegs to -7.7, debossed "4" on the joint face | `top_4mm` | 36.000 x 35.996 x 7.700 | 5166.217 | 5166.344 | 8 |
| `bottom_4mm.3mf` | web-4 pair: boss -11.001..-6.5 (journal 4.5), **web 4.0** (-10.5..-6.5), debossed "4" | `bottom_4mm` | 35.000 x 34.995 x 6.700 | 4175.230 | 4175.491 | 8 |
| `top_6.9.3mf` | race-clamp top for `bottom_3.5mm` (EXO-315): shoulder z -4.101, **gap 6.9** (was 7.001); joint face -7.0; debossed "6.9" on the joint face | `top_6.9` | 36.000 x 35.996 x 8.200 | 5493.823 | 5494.057 | 8 |
| `top_6.8.3mf` | race-clamp top for `bottom_3.5mm` (EXO-315): shoulder z -4.201, **gap 6.8** (was 7.001); joint face -7.0; debossed "6.8" on the joint face | `top_6.8` | 36.000 x 35.996 x 8.200 | 5524.854 | 5525.089 | 8 |
| `top_6.7.3mf` | race-clamp top for `bottom_3.5mm` (EXO-315): shoulder z -4.301, **gap 6.7** (was 7.001); joint face -7.0; debossed "6.7" on the joint face | `top_6.7` | 36.000 x 35.996 x 8.200 | 5556.450 | 5556.689 | 8 |
| `top_6.6.3mf` | race-clamp top for `bottom_3.5mm` (EXO-315): shoulder z -4.401, **gap 6.6** (was 7.001); joint face -7.0; debossed "6.6" on the joint face | `top_6.6` | 36.000 x 35.996 x 8.200 | 5587.123 | 5587.362 | 8 |
| `top_6.5.3mf` | race-clamp top for `bottom_3.5mm` (EXO-320): shoulder z -4.501, **gap 6.5**; joint face -7.0; debossed "6.5" on the joint face | `top_6.5` | 36.000 x 35.996 x 8.200 | 5618.299 | 5618.540 | 8 |
| `top_6.4.3mf` | race-clamp top for `bottom_3.5mm` (EXO-320): shoulder z -4.601, **gap 6.4**; joint face -7.0; debossed "6.4" on the joint face | `top_6.4` | 36.000 x 35.996 x 8.200 | 5649.482 | 5649.725 | 8 |
| `top_6.3.3mf` | race-clamp top for `bottom_3.5mm` (EXO-320): shoulder z -4.701, **gap 6.3**; joint face -7.0; debossed "6.3" on the joint face | `top_6.3` | 36.000 x 35.996 x 8.200 | 5680.646 | 5680.889 | 8 |
| `top_6.2.3mf` | race-clamp top for `bottom_3.5mm` (EXO-320): shoulder z -4.801, **gap 6.2**; joint face -7.0; debossed "6.2" on the joint face | `top_6.2` | 36.000 x 35.996 x 8.200 | 5711.603 | 5711.849 | 8 |
| `top_6.1.3mf` | race-clamp top for `bottom_3.5mm` (EXO-323): shoulder z -4.901, **gap 6.1**; joint face -7.0; debossed "6.1" on the joint face | `top_6.1` | 36.000 x 35.996 x 8.200 | 5743.149 | 5743.397 | 8 |
| `top_6.0.3mf` | race-clamp top for `bottom_3.5mm` (EXO-323): shoulder z -5.001, **gap 6.0**; joint face -7.0; debossed "6.0" on the joint face | `top_6.0` | 36.000 x 35.996 x 8.200 | 5773.722 | 5773.970 | 8 |
| `top_5.8.3mf` | race-clamp top for `bottom_3.5mm` (EXO-326): shoulder z -5.201, **gap 5.8**; joint face -7.0; debossed "5.8" on the joint face | `top_5.8` | 36.000 x 35.996 x 8.200 | 5835.932 | 5836.183 | 8 |
| `top_5.6.3mf` | race-clamp top for `bottom_3.5mm` (EXO-326): shoulder z -5.401, **gap 5.6**; joint face -7.0; debossed "5.6" on the joint face | `top_5.6` | 36.000 x 35.996 x 8.200 | 5898.201 | 5898.456 | 8 |
| `top_5.4.3mf` | race-clamp top for `bottom_3.5mm` (EXO-326): shoulder z -5.601, **gap 5.4**; joint face -7.0; debossed "5.4" on the joint face | `top_5.4` | 36.000 x 35.996 x 8.200 | 5960.560 | 5960.819 | 8 |
| `top_5.2.3mf` | race-clamp top for `bottom_3.5mm` (EXO-326): shoulder z -5.801, **gap 5.2**; joint face -7.0; debossed "5.2" on the joint face | `top_5.2` | 36.000 x 35.996 x 8.200 | 6022.681 | 6022.943 | 8 |
| `top_5.0.3mf` | race-clamp top for `bottom_3.5mm` (EXO-326): shoulder z -6.001, **gap 5.0**; joint face -7.0; debossed "5.0" on the joint face | `top_5.0` | 36.000 x 35.996 x 8.200 | 6084.800 | 6085.064 | 8 |
| `bottom_d30.1.3mf` | journal test (EXO-327): `bottom_3.5mm` with the boss **O30.1** (r 15.05, z -11.001..-7.0), pocket wall 1.1556; debossed "30.1" | `bottom_d30.1` | 35.000 x 34.995 x 6.200 | 3861.952 | 3862.184 | 8 |
| `bottom_d30.2.3mf` | journal test (EXO-327): `bottom_3.5mm` with the boss **O30.2** (r 15.10, z -11.001..-7.0), pocket wall 1.2056; debossed "30.2" | `bottom_d30.2` | 35.000 x 34.995 x 6.200 | 3880.451 | 3880.676 | 8 |
| `bottom_d30.3.3mf` | journal test (EXO-327): `bottom_3.5mm` with the boss **O30.3** (r 15.15, z -11.001..-7.0), pocket wall 1.2556; debossed "30.3" | `bottom_d30.3` | 35.000 x 34.995 x 6.200 | 3899.600 | 3899.835 | 8 |
| `bottom_d30.4.3mf` | journal test (EXO-327): `bottom_3.5mm` with the boss **O30.4** (r 15.20, z -11.001..-7.0), pocket wall 1.3056; debossed "30.4" | `bottom_d30.4` | 35.000 x 34.995 x 6.200 | 3918.615 | 3918.851 | 8 |
| `bottom_d30.5.3mf` | journal test (EXO-327): `bottom_3.5mm` with the boss **O30.5** (r 15.25, z -11.001..-7.0), pocket wall 1.3556; debossed "30.5" | `bottom_d30.5` | 35.000 x 34.995 x 6.200 | 3937.665 | 3937.903 | 8 |
| `bottom_d30.6.3mf` | journal test (EXO-327): `bottom_3.5mm` with the boss **O30.6** (r 15.30, z -11.001..-7.0), pocket wall 1.4056; debossed "30.6" | `bottom_d30.6` | 35.000 x 34.995 x 6.200 | 3956.785 | 3957.025 | 8 |
| `top_d30.1.3mf` | journal test (EXO-327): `top_3.5mm` with the spigot **O30.1** (r 15.05, z -7.0..-4.0); debossed "30.1" | `top_d30.1` | 36.000 x 35.996 x 8.200 | 5475.906 | 5476.142 | 8 |
| `top_d30.2.3mf` | journal test (EXO-327): `top_3.5mm` with the spigot **O30.2** (r 15.10, z -7.0..-4.0); debossed "30.2" | `top_d30.2` | 36.000 x 35.996 x 8.200 | 5489.665 | 5489.904 | 8 |
| `top_d30.3.3mf` | journal test (EXO-327): `top_3.5mm` with the spigot **O30.3** (r 15.15, z -7.0..-4.0); debossed "30.3" | `top_d30.3` | 36.000 x 35.996 x 8.200 | 5504.065 | 5504.301 | 8 |
| `top_d30.4.3mf` | journal test (EXO-327): `top_3.5mm` with the spigot **O30.4** (r 15.20, z -7.0..-4.0); debossed "30.4" | `top_d30.4` | 36.000 x 35.996 x 8.200 | 5518.301 | 5518.541 | 8 |
| `top_d30.5.3mf` | journal test (EXO-327): `top_3.5mm` with the spigot **O30.5** (r 15.25, z -7.0..-4.0); debossed "30.5" | `top_d30.5` | 36.000 x 35.996 x 8.200 | 5532.564 | 5532.806 | 8 |
| `top_d30.6.3mf` | journal test (EXO-327): `top_3.5mm` with the spigot **O30.6** (r 15.30, z -7.0..-4.0); debossed "30.6" | `top_d30.6` | 36.000 x 35.996 x 8.200 | 5546.881 | 5547.126 | 8 |

Same outside shape in both designs, so both fit the same bearing and stator. Top part:
face disk r 18 (module z -4..0) over a r 15 spigot (z -9..-4). Bottom part: r 15
(z -11..-9) over a r 17.5 flange (z -13.2..-11), with a r 8.1 bore underneath (z -13.2..-10.5).
The bearing (5972K159) inner race (bore r 15, z -11..-4) slides over both r 15 surfaces.
It is clamped between the underside of the top disk (z -4) and the bottom flange (z -11.001).
The XL330 horn disk (r 8.0) sits in the r 8.1 bore and centres the rotor.

**Original (`manual_*`).** Four horn screws (M1.6 per EXO-11, at r 6, 45/135/225/315 deg)
go in from the rotor face. Each head sits in a O5 counterbore in the top part (z -5..0),
and the shank goes through BOTH parts into the horn. So the horn screws clamp the two
parts together and to the horn, and can only be driven once the stack is built. The top
part has 4x O3.2 at r 11 (0/90/180/270) and hex pockets open at its underside
(z -9..-5). These hold thin 1.8 mm M3 nuts (DIN 439) for the link's M3 bolts. The bottom
part closes the pockets from below. The bottom part has no M3 holes and no pegs (4
through-holes = the horn screws). Only the horn screws and the shared bearing bore locate the two parts against each other.
Measured (EXO-291, `tools/measure_holes.py`): hex pocket **6.0 AF** (AC 6.928, 4.0 deep) - loose,
the 5.5 AF M3 nut falls out; horn screw hole **O2.2** (fits M2, loose for M1.6), O5.0 counterbore.

**v3 (`inner_*`).** The horn screws (M2 x 4, r 6) hold the **bottom part alone**. Their
heads sit on its web (z -9), so you drive them first through the top part's four
**through** O5 columns (8 through-holes in the top = 4 columns + 4 M3 holes). The link's
four M3 x 20 bolts (r 11) then go through both parts into **standard ISO 4032 nuts**
captured in **5.6 AF** hex pockets (EXO-301; was 5.8, EXO-240) on the bottom part's underside (z -13.2..-10.6). The XL330
case keeps the nuts in. Eight O3.0 pegs under the top part (r 12.2, 1.2 tall) drop into
eight O3.3 sockets in the bottom part. The index notch is on both top parts.

**Pocket size (EXO-301).** 6.0 (original) -> 5.8 (EXO-240) -> **5.6** (Esteban, 2026-10-15).
The 5.6 `inner_bottom.3mf` is a mesh boolean of the 5.8 Fusion export (Fusion bridge down), so
it has no Fusion V and carries a debossed "56" (0.4 deep) on the boss face (file z -1.5, centre).
Measured AF 5.600 at all four pockets. The generator (`BuildArm.py` `V3_NUT_AF`) now says 5.6,
but it has not been re-run: the v3 module in `exo generated` still has 5.8 (pending).

**Thicker web (EXO-304/310, `top_*` / `bottom_*`).** Fusion export 2026-10-17 (`BuildArm.build_v4_copy`,
`tools/bridge_scripts/build_v4_exo304.py`, scratch document, not saved). The flat joint face moves up from z -9.0
to **-7.0** (pair 3.5) or **-6.5** (pair 4). Web = horn top -10.5 to the face: **3.5 / 4.0**, so a stock M2 x 6
horn screw reaches **2.5 / 2.0** mm into the horn (v3: 1.5 web, 4.5). Journal on the r 15 race: bottom 4.0 / 4.5,
top 3.0 / 2.5; race clamp -4 / -11.001 unchanged. Pegs/sockets move with the face (sockets 1.3 deep; floor to the
nut pocket 2.3 / 2.8); 5.6 AF pockets, wall to the r 15 boss 1.106 as v3. Tops and bottoms pair by label only:
a 3.5 top on a 4 bottom does not close. Horn screws in the Fusion module not moved.

**Race clamp (EXO-315 `top_6.9` .. `top_6.6`, EXO-320/323 `top_6.5` .. `top_6.0`, EXO-326 `top_5.8` .. `top_5.0`).** `top_3.5mm` + its race shoulder (disk underside, the r 15..18
annulus, 311.018 mm2) extruded down 0.101 .. 1.001 in 0.1 steps and 1.201 .. 2.001 in 0.2 steps, so the shoulder-to-flange gap on `bottom_3.5mm`
(-11.001) is 6.9 .. 6.0 and 5.8 .. 5.0 for the 7.000 race. Rotor face 0 and joint face -7.0 unchanged; the label on
the joint face reads the gap instead of "3.5" (it is the face that is up on the bed). All fit `bottom_3.5mm` only.

**Journal diameter (EXO-327, `*_d30.1` .. `*_d30.6`).** The race bore is a nominal O30 and the O30.000 journal is a
slip fit. A ring r 14.9..R is joined over the journal band of `bottom_3.5mm` (z -11.001..-7.0) and `top_3.5mm`
(-7.0..-4.0), R = 15.05 .. 15.30; nothing inside r 14.9 changes. Circle fit on the exported meshes: D 30.1000 ..
30.6000 exactly at both band ends. Pocket-to-boss wall grows 1.1056 -> 1.1556 .. 1.4056. Label ("30.x") on the
joint face of each part; same-label bottom and top go together. Gap stays 7.001 (no race clamp change).

## Other files

- `rotor_dia.bambu.3mf`: EXO-327 plate, all twelve journal-test parts, 4 x 3 with each bottom next to its top
  (bottoms plate down, tops rotor face down), supports on (tree auto), EXO-294 ids 1..23 odd meshes, 2..24 even wrappers.
- `rotor_gap3.bambu.3mf`: EXO-326 plate, five tops 5.8 / 5.6 / 5.4 / 5.2 / 5.0 (3 + 2), rotor face down,
  supports on (tree auto), EXO-294 ids 1/3/5/7/9 meshes, 2/4/6/8/10 wrappers.
- `rotor_gap2.bambu.3mf`: EXO-320/323 plate, six tops 6.5 / 6.4 / 6.3 / 6.2 / 6.1 / 6.0 (3 x 2), rotor face down,
  supports on (tree auto), EXO-294 ids 1/3/5/7/9/11 meshes, 2/4/6/8/10/12 wrappers.
- `rotor_gap.bambu.3mf`: EXO-315 plate, the four race-clamp tops, rotor face down, supports on (tree auto),
  EXO-294 ids 1/3/5/7 meshes, 2/4/6/8 wrappers.
- `rotor_web.bambu.3mf`: EXO-304/310 plate, both pairs: tops rotor face down, bottoms plate down,
  supports on (tree auto), EXO-294 ids 1/3/5/7 meshes, 2/4/6/8 wrappers.
- `rotors_v3_pockets.bambu.3mf`: pocket-variant plate (EXO-296/297): one `inner_top` + `inner_bottom_p54`
  + `inner_bottom_p56`, oriented as in EXO-267.
- `rotors_v3_x2.bambu.3mf`: two complete v3 rotors with the 5.6 pocket (EXO-300): `inner_top` x2 +
  `inner_bottom_p56` x2, supports on. See EXO-300 for its read-back.
- `rotors_compare.bambu.3mf`: Bambu Studio project (EXO-267), the four rotor files above on one
  plate, oriented for printing (tops rotor face down, bottoms O35 plate down), with the
  printer/filament/process settings of `inner_bottom_v2.3mf`. Built by
  `python3 -I tools/bambu_project_exo267.py`, not by Bambu Studio.
- `inner_bottom_v2.3mf`: Bambu Studio plate, 8 copies of an older v3 bottom part (before
  EXO-238 sockets and EXO-240 nut pocket; mesh V 2576.451 each, no sockets). Superseded.
- `stator.3mf` / `stator.f3d`: stator (not checked in EXO-265).
- `tool_organizer.3mf`: EXO-258 organizer box.
