# Exo joint ("sensor module 2")

The single repeated building block of the exoskeleton. One per arm joint, 7 per arm.
Joints are connected by links — see `exo-link.md`.

**Geometry (measurable):** `gento_luna/Gento Luna_URDF 20260428/urdf/sensor module 2.step`
— STEP AP214 assembly exported from Fusion 2026-09-13. Editable source is the Fusion
design `sensor module 2` (cloud), not the STEP.

**Photos:** `gento_luna/exo/screenshots/` (add as available).

## How it works — terminology (use these names everywhere)

Four functional parts:

| Name | Printed? | Role |
|---|---|---|
| **Actuator** | not printed — bought `XL330`, but MODELLED in the design | Dynamixel XL330 servo. Sits inside the stator, horn facing up. It **is** a body: component `XL330 v2`, solid `Body1` (plus 5 small bodies for the connectors and the horn shaft, all inside its box). Every placed module therefore carries the real actuator and `check_interference` sees it — never build a proxy for it (EXO-201). Measured in the module frame: x −10…10, y −9.5…24.5, z −36.5 (its back) … −10.5 — i.e. it sticks 12 mm below the stator underside −24.5. |
| **Stator** | yes — 1 body | Outer cylinder. Fixed to the servo body; seats the bearing's outer race. Has a **stop block** on its rim, rising to rotor-face height. Underside mounts to the *previous* link via **3× M3 heat-set inserts** (triangle) and an embossed **"0"** — a part-number label (joints are labeled 0,1,2,…). The actuator's back and its two connectors protrude below the stator face — the previous link must clear them. |
| **Rotor** | yes — 2 bodies | Inner cylinder + face disk. Bolted to the servo horn; rides on the bearing's inner race. Mounts to the *next* link. Has an **index notch** on its rim (see below). |
| **Bearing** | no — `5972K159` | One ball bearing between rotor and stator. Carries all side/axial load so the servo horn only sees torque. |

**The rotor is a bridge.** A link is never screwed to the servo horn directly:
horn → rotor → link. Rotor face hole pattern (8 + center):

- **inner 4** → M1.6 screws into the horn (`4x pan_head_philip_mc_screw_m1-6`)
- **outer 4** → M3 screws from the next link into nuts captured in the rotor
  (`4x m3 hex nut`)

**Index notch.** The rotor's 4 M3 holes are 4-fold symmetric, so a link could bolt on in
any of 4 orientations 90° apart. The notch marks the one correct orientation. This
matters because skeleton pose is read from the servo's *absolute* encoder: bolting the
link 90° off after a re-assembly silently invalidates the calibration. Every link must
carry the mating feature; never drop the notch in a redesign. EXO-236 and EXO-238 both
measured that the notch is the **only** feature anywhere on the rotor pair that is not
4-fold symmetric.

**Stator underside** (previous link side): `3x m3_6mm_insert`, plus the exposed actuator back.

**Joint limits are mechanical.** The stator's stop block (the **joint stop**) collides
with a matching block on the link bolted to the rotor (the **link stop**). The angular gap
between them *is* the joint's range; changing a limit means moving/resizing a block, not a
servo setting. Both are derived, not chosen — `armgen.limited_joint` (EXO-204/EXO-208):

    joint stop arc  a = JOINT_STOP_DEG        one hyperparameter, 15.0°
    link stop arc   b = 360° − a − range      whatever is left, and b ≥ a
    link stop angle              the one that puts first contact at exactly `lower`

The joint stop is the **minimum** and the link stop takes the rest, because the joint stop
is the same at every joint of every arm while the link stop is generated per joint. `b ≥ a`
is therefore also the statement that this module can limit a range of at most 360 − 2a =
**330°**; a wider joint is refused by name. Note that `a + b = 360° − range` whatever `a`
is, so `JOINT_STOP_DEG` changes only how the circle is *split* between the two blocks — the
link stop's **angle**, the contact poses and the travel do not depend on it.

**EXO-208: the block is 15° in the arm, 20° in the design.** G1's shoulder pitch has a
330.00077° range, so it needs `a ≤ 14.9996°`; the module as drawn is 19.9993° and would
refuse that joint. The narrowing is made in a **copy**, never in `sensor module 2` itself:
`BuildArm.ensure_stop_module` places one more occurrence of the linked module, `breakLink()`s
it into this document, renames it **`sensor module 2 stop15`** and combine-cuts two ring
sectors (r 20…30, z −4…+1) off the ends of the block, leaving `JOINT_STOP_DEG` centred on
the block's own measured centre 168.00225°. Measured on the result: **160.50223…175.50227°,
arc 15.00005°**, radii and height untouched, stator body 25730.0 → 25685.1 mm³. The
function holds the whole recipe and re-measures what it made, so the part is re-makeable
from nothing by running it. Luna's 346° joints would need 7° — deferred.

**EXO-333: the module rotor, FINAL (Esteban, 2026-10-22).** After the October print tests
(EXO-304/310 web, EXO-315/320/323/326 race gap, EXO-327 journal diameter) the rotor is fixed and
the generator makes only this one. The module copy is **`sensor module 2 rotor`** (it was
`sensor module 2 v3` until EXO-333; that legacy copy is removed from `exo generated`).
`BuildArm.build_v3_copy` = copy + `breakLink` + `_restore_child_positions` (EXO-304: a breakLink
of the migrated module can misplace the copy's children; each takes its linked twin's
placement, worst box difference 0.000000) + rename + 15° stop + `rotor_v3` (the v3 pair below,
checked on `V3_STAGE_PROBES`) + **`module_rotor_final`** + `fasteners_v3`. Constants `ROTOR_*`.
All numbers module z, measured on the solid in `exo generated` and on the exported meshes:

| | value |
|---|---|
| joint face (flat, between the parts) | **−7.000** (v3: −9.0); pegs to −8.200, sockets floor −8.300 |
| bottom web (horn top / bore ceiling −10.500 → joint face) | **3.500** (v3: 1.5) |
| horn screw | stock **M2 × 6**, head on the joint face, tip −12.989 (unchanged): **2.489 mm into the horn** (v3: M2 × 4 through 1.5; M2 × 6 put 4.5 in and hurt the servo) |
| top race shoulder (disk underside) | **−5.001** (v3: −4.000) |
| bottom race shoulder (flange top) | −11.001 (unchanged) |
| race gap on the 7.000 race | **6.000** (v3: 7.001, race loose) |
| journal, both parts | **Ø30.100** (r 15.05001 bisected; mesh fit 30.1000); bottom −11.001…−7.0 (4.0), top −7.0…−5.001 (2.0) |
| nut pocket | **5.6 AF** (mesh rmin 1.600 / rmax 3.233 → 5.600), z −13.2…−10.6 |
| pocket outer corner → journal wall | **1.156** (corner r 13.894), over z −11.001…−10.6 |
| socket → journal wall | **1.200** |
| bodies | top **5786.431**, bottom **3865.869** mm³ (Fusion) |

`V3_PROBES` (43) assert the final rotor; `ensure_v3_module` accepts nothing else. The M3 × 20
bolts and ISO 4032 nuts are unchanged (tip −12.0, nut −13.0…−10.6). In CAD the parts are drawn
with the joint faces CLOSED, so the clamp and the Ø30.1 journal overlap the drawn bearing:
inner race (Body2, r 15…16.181) 111.4 mm³ with the top and 16.5 with the bottom, and the
shield/cage ring Body11 (r 16.181…18.864, z −5.400…−4.124) **134.0 mm³ with the top's disk**
(0.877 deep) — all module-internal. Physically the race stops the top first (the 1.0 of
interference is taken by the print), which is what Esteban's `top_6.0` test print did; if
the disk is ever seen to rub the shield, lower the shoulder only on r 15…16.18. Arms rebuilt
with it in `exo generated`: G1 **6/6 links, 328 bodies, 0 FAIL**, axes 0.00e+00° off the
URDF, stop bracket **J1…J6 ALL PASS**; Luna unchanged (R12 refused EXO-224, 1 FAIL = EXO-244).
Print files `parts/top.3mf`, `parts/bottom.3mf`, `parts/rotor.bambu.3mf`; the test prints are
in `parts/archive/2026-10-tests/`. The paragraphs below are the history that led here.

**EXO-225: `sensor module 2 v3` — the horn screws hold the lower rotor part alone.**
In the design as drawn, the four small horn screws pass through BOTH rotor parts at once
and clamp them to the horn, so they can only be driven after the stack is built. v3 splits
that: the horn screws (now **M2 × 4**, head seated on the lower part's own web top face
z −9.0) hold the **lower** part alone and are driven first; the four **M3** bolts the link
already puts through the rotor pad at r 11 carry on down through the upper part into the
lower one, into **hex nuts captured under it** (pocket **5.6** across flats since EXO-301, 5.8 in EXO-240,
FLATS radial, z −13.200 … **−10.600** — 2.6 deep for a STANDARD ISO 4032 nut, ceiling 1.60,
retained below by the XL330's top face 0.300 away — the nut's inner flat r 8.250 clears the
horn disk r 8.000 by 0.250, and the pocket's own inner flat r 8.200 clears it by 0.200 at 5.6).
The upper part has **no Ø2.2 hole at all**: one plain **Ø5.0 column per screw head, running
clear THROUGH it** from the z −9.000 mating face to the rotor face 0 — a blind pocket
z −9.100 … −7.000 until EXO-240, when the print showed no driver can reach that far down a
closed hole — the hex pockets filled and its Ø3.2 taken down to z −9. The two parts are located by the **four
M3 shanks** — Ø3.0 in a Ø3.2 hole on a r 11 bolt circle, 0.1 mm radial — **and by eight
Ø3.0 register pegs on the upper part** (EXO-238, see below). Rotor face z = 0, the 4× Ø3.2 at
r 11, the index notch, the bearing seat and the stator are untouched; bodies 6473.293 →
**6657.964** and 2950.793 → **2507.386** mm³ (EXO-240 numbers). Made, like the 15° stop
(which it folds in), in a **copy**: `BuildArm.ensure_v3_module` holds the whole recipe and
re-measures the result against 39 point-containment probes. The arm places v3 at every joint.

**EXO-231: the standard nut and the bolt, both drawn.** The module as delivered carries a
1.8 mm **thin** nut (DIN 439); Esteban uses standard **ISO 4032** (5.5 across flats, 2.4
thick), which is why the pocket ceiling moved −11.200 → **−10.600**. The four thin nuts are
deleted and four standard ones are drawn instead, at z **−13.000 … −10.600** — the bolt
pulls them up onto the ceiling, 0.200 inside the pocket mouth. The M3 bolt is drawn too,
four of them, Ø3.0 shank with a Ø5.5 × 3.0 socket head seated on the link's rotor pad outer
face z = `armgen.PAD_T` = 8: **M3 × 20**, tip z **−12.000**, **1.400 mm** of thread in the
nut and **1.520 mm** of air under the tip before the XL330's top face (M3 × 22 would be
0.5 mm inside it). Both sets are one component of four bodies, placed at identity inside the
module, so the module frame is their own. The 1.30 mm wall between the Ø3.2 hole and the
r 8.1 bore now survives only z −10.600 … −10.500 (0.100 mm); above the bore top −10.500 the
hole runs through the solid web, not a wall.

**EXO-240: two fixes off Esteban's print — the head pockets go through, the nut pocket
tightens.** (1) *The M2 screws did not go all the way in.* A blind Ø5.0 pocket z −9.100 …
−7.000 puts the head 7 mm down a closed hole: nothing reaches it, so the screw never seats.
Esteban's call was to open it as deep as needed, through to the rotor face if that is what it
takes, and it is: `V3_HEAD_Z` −9.1…−7.0 → **−9.1…+0.1**, both ends past a face, Ø5.0
unchanged. Measured on the solid: void on the screw axis at **every** height from the mating
face to the rotor face, at all four angles, and the bore is **5.0000 wide at z −8.5, −6.0,
−4.0, −2.0 and −0.5** — one plain cylinder, not a step, because the fill that closes the old
Ø2.2/Ø5.0 pair still runs first. It costs **−549.779 mm³** (the cut goes 706.858 where it
used to take 157.080), upper part 7207.743 → **6657.964**. Nothing else is in that column:
the upper part is solid from the axis out to r **3.5010** and from r **8.5010** out at every
height, so the walls are **3.8786** mm to the Ø3.2 M3 hole and **3.0408** mm to the nearest
register peg, both read off two measured edges. The M2 head tops out at z **−7.6474** with
**1.32…1.49 mm** of radial room in the column — 7.65 mm of open Ø5.0 above it. The rotor face
carries four Ø5.0 mouths at r 6 again, as it did in v2; the link's rotor pad is solid there
(`armgen` drills it only at r 11), so the column is the driver's path while the two rotor
parts are stacked *alone* — which is the assembly order anyway: horn screws first, link last.
(2) *The nuts
fell out.* `V3_NUT_AF` 6.0 → **5.8** across flats: an ISO 4032 M3 nut is 5.5 at max, so 5.8
leaves **0.150 mm per side** (was 0.250). Measured off the solid at y 1.600: inner flat
x **8.1000**, outer flat x **13.9000**, **across flats 5.8000**. Phase, depth and the
flats-radial orientation are unchanged — ceiling **−10.6000**, **2.6000** deep off the part's
own bottom face, ceiling band **1.6000** — and the pocket's inner flat now stands **0.100**
off the XL330 horn disk r 8.000 where at 6.0 it sat exactly on it; it is **tangent** to the
r 8.100 horn bore, leaving 0.1597 mm of wall at y 1.600 tapering to 0 on the centre line.
The 1.30 mm wall band still survives exactly **0.1000 mm** (z −10.6000 … −10.5000), the
EXO-238 floor over the sockets is still **0.3000 mm**, and the pocket corner now pokes
**0.3023** mm into the socket footprint instead of 0.4152. Pockets **−219.342 mm³** (was
−238.839, which overlapped the bore), lower part 2487.889 → **2507.386**. `V3_PROBES` 36 →
**39**, and **four** of the old ones changed sign, which is the point: the probes that
asserted a filled column and its blind ceiling now assert an open one, three more stand on the
solid left around it, and one new probe stands on the pocket corner a 6.0 pocket cut away.

**EXO-301: 5.6 across flats is official (Esteban, 2026-10-15).** 6.0 (EXO-11; nut falls out,
measured EXO-291) → 5.8 (EXO-240) → **5.6**, picked from the EXO-296 5.4/5.6 test variants.
`V3_NUT_AF` 5.8 → **5.6** in `BuildArm.py`: 0.050 per side on a 5.5 nut, inner flat r **8.200**
(0.200 off the horn disk, 0.100 wall to the r 8.100 bore), circumradius 3.2332. The three
floor probes moved from (11.000, 3.185) — only 0.0417 inside a 5.6 hex — to (11.000, 3.130):
0.0893 inside the hex, 0.0875 inside the socket. The print file `parts/inner_bottom.3mf` is
the EXO-297 mesh-boolean p56 (debossed "56" on the boss face); the 5.8 mesh is kept as
`parts/inner_bottom_p58.3mf`. (Resolved by EXO-333: the generator was re-run and the module in
`exo generated` carries 5.6; those files are now in `parts/archive/2026-10-tests/`.)

**EXO-236 → EXO-238: the register pegs changed sides, and there are eight of them.** v3 as
first drawn put two Ø3.0 pegs on the LOWER part (r 12.2, 67.5°/112.5°) into two Ø3.3 sockets
in the upper one. EXO-236 deleted them — the lower part prints on its own z −9 mating face,
so those pins stood on the bed and printed badly. EXO-238 brings them back **the other way
round and eight strong**: the **upper** part prints rotor-face-down, so a pin standing down
off *its* z −9 face points up into the air and needs no support. Measured on the solid:
eight Ø3.0 pegs at **r 12.2, 22.5° + k·45°** (22.5° off both the M3 holes at 0/90/180/270 and
the horn screws at 45/135/225/315), **z −9.000 … −10.200 — 1.200 tall**, into eight Ø3.3
sockets **z −9.000 … −10.300**: **0.150 radial** and **0.100 axial** clearance. Volumes:
upper 7139.884 → **7207.743** (+67.858 = 8 × Ø3.0 × 1.2), lower 2576.840 → **2487.889** mm³
(−88.951 = 8 × Ø3.3 × 1.3). **The walls, each read off two measured edges:** peg to the Ø3.2
M3 hole **1.5766**, socket to the same hole **1.4266**; the rim is r **15.0000** on both parts
over this band, so the face annulus outside a peg root is **1.3000** and the lower part's wall
outside a socket is **1.1500** — the thinnest number in the joint, and the one to watch if the
pegs ever move outward. The hex nut pocket's corner pokes **0.4152** mm into the socket
footprint in plan (**0.3023** since EXO-240 shrank the pocket), leaving a **0.3000 mm floor**
between the socket bottom −10.300 and the pocket ceiling −10.600 (printed as a bridge over
the socket, the part being face-down) — measured again after EXO-240, unchanged.

Consequence, and it is measured, not argued: **the two rotor parts go together in any of
four orientations 90° apart and all four are the same part.** A swept point-containment grid
(dr 0.5, dz 0.25, dθ 2°, r ≤ 17.9 / 17.4 over each part's full z span, the upper one now down
to the peg tips −10.2 — 64800 + 25200 quadruples) found the containment at θ, θ+90, θ+180,
θ+270 identical **everywhere** on the lower part and everywhere on the upper part except the
**index notch** (210 points, r 15.130…17.630, z −1.325…−0.325). EXO-240 re-ran the sweep coarser
(dr 0.5, dz 0.5, dθ 5° — 12960 + 5040 quadruples) over the parts with the through columns and
the 5.8 pocket: again **0** asymmetric quadruples anywhere but the notch. The M3 holes (0/90/180/270),
the horn screws (45/135/225/315), the hex pockets (phase = angle + 30°) and the eight pegs
(22.5° + k·45° — 8-fold, which contains 4-fold) are each their own 4-fold pattern. So the
notch is the *only* feature in the whole rotor that fixes an angle, and what it fixes is the **link** on the
rotor face. One thing that follows is worth writing down: the lower part's own horn-screw
pattern is 4-fold too (measured), so on the **rotor side** the rotor↔horn joint is 4-fold as
well, and whether the XL330's horn itself carries anything that keys it was NOT measured
here. If it does not, the notch fixes the link on the rotor but not the rotor on the absolute
encoder — which was already the case before v3 (the horn screws were always at
45/135/225/315) and which EXO-236 did not change. Open point, not a regression.

Renders: `gento_luna/exo/screenshots/exo-joint-iso.png` (rotor face up, bearing ring
visible), `exo-joint-bottom.png` (stator underside, 3 inserts, actuator back),
`exo-joint-bottom-iso.png`.

## STEP product tree (raw names, 2026-09-13)

Flat under root `sensor module 2`: `XL330`, `5972K159_Ball Bearing`, `Component1`,
`Component2`, `Component3`, `4x pan_head_philip_mc_screw_m1-6`, `3x m3_6mm_insert`,
`4x m3 hex nut`. Mapped 2026-09-13 (mm, module frame):

| Product | Part | Geometry |
|---|---|---|
| `Component1` → `stator` | **Stator** | r 27 outer, z −24.5 … −4; bearing seat r 21.05; 3× Ø4 insert holes at r 19.25, angles **0/180/270** (none on +Y = connector side); actuator pocket x ±10, y −9.5 … 24.5; stop block r 21.05–27, angles **158.00258°–178.00192°** (arc 19.99935, centre 168.00225 — measured on the solid, EXO-208), z −4 … −0.4; the arm places the 15° copy instead |
| `Component2` → `inner_top` | **Rotor** face disk | r 18 (z −4 … 0) over r 15 (z −9 … −4); 4× Ø3.2 at r 11 (0/90/180/270) with hex nut pockets; 4× Ø2.2 at r 6 (45/135/…) for the horn screws; **index notch** at +Y: 4.0 wide (x), r 14.75 … 17.9, 1.5 deep |
| `Component3` → `inner_bottom` | **Rotor** inner cylinder | r 17.5 / r 15, z −13.2 … −9, bore r 8.1 |
| `XL330` | Actuator | x ±10, y −9.5 … 24.5 (connectors on +Y), z −36.5 … −10.5; sticks 12 mm out of the stator underside |
| `5972K159` | Bearing | r 15 … 21, z −11 … −4 |

### Part names: `Component1..3` → `stator` / `inner_top` / `inner_bottom` (EXO-242)

`Component1`, `Component2`, `Component3` are Fusion's defaults, the names the cloud design
was drawn with, and they are useless on a print queue. Every module copy the generator makes
(`sensor module 2 v3`, since EXO-333 `sensor module 2 rotor`) is therefore **renamed** as it is made — `BuildArm.rename_v3_parts`, run on
the fresh local copy right after the `breakLink`, so every re-make carries the names and a
module without them is rejected and re-made (`BuildArm.v3_rename_verdict`):

| original | v3 copy | the printed part |
|---|---|---|
| `Component1` | **`stator`** | the part that does not turn; carries the joint stop |
| `Component2` | **`inner_top`** | rotor face disk, z −7 … 0 (EXO-333; −9 in v3); pegs, head columns — `parts/top.3mf` |
| `Component3` | **`inner_bottom`** | rotor inner cylinder, z −13.2 … −7 (EXO-333; −9 in v3); sockets, nut pockets — `parts/bottom.3mf` |

`sensor module 2` itself keeps `Component1..3`: it is a linked reference and is never
written. **Both names are live at once**, since the original and the copy sit in the same
document, so anything that looks a part up by name goes through
`armgen.module_part_names(which)` / `armgen.is_module_part(name, which)`, which accept
either. Nothing that decides geometry reads a name at all —
`BuildArm.module_stator_occ` and `module_rotor_occs` find the parts on the **solid**, which
is exactly why the copy may be renamed, and `armgen.classify_interference` reads the
bounding box, never the label.

The exo STEP (`exo_skeleton_2026-08-08.step`) uses an older revision of this module: the
rotor there (`Component155` …) has 8× Ø3.2 at r 11 (45° pitch) and no notch; stator,
bearing and XL330 are identical and sit at the same angles.

## Frame convention

**Origin = center of the rotor's top face**, on the rotation axis. Module local **Z**
= the rotation axis, pointing out of the rotor face. Module **+Y** = the actuator's
connector side (the stator side without an insert); module +X = the insert at 0°.
`fusion_scripts/PlaceModules/PlaceModules.py` places this origin on each URDF joint origin
with Z aligned to the joint axis and a free rotation about Z; `fusion_scripts/BuildArm/`
adds a per-joint axis sign and a world "connector side" hint that fixes the rotation.

In the exo STEP (exo frame = left J1 rotor face at the origin) `Module k` = joint k+1.
The right arm is a Fusion **mirror** of the left, so its module Z signs relative to the
URDF axes are: J1 −, J2 + (measured from the geometry, not from the STEP transforms).

## Target: Gento Luna URDF

`gento_luna/Gento Luna_URDF 20260428/urdf/Gento Luna_URDF.urdf` — joints
`Arm_{R,L}{1..7}_Joint` (+ `Head_J2_Joint`, unused). Meshes in `../meshes/*.STL`.
Reference lengths and the change list live in `gento_luna/exo/NOTES.md`.

## Known/open

- The 15° block lives only in the working document as `sensor module 2 stop15`; the
  cloud design and the STEP still carry the 20° one. Making it a saved design of its
  own is Esteban's call (EXO-208).
- Bearing is what gives the joint stiffness; any redesign keeps it.
- EXO-333: in CAD (joint faces closed) the final top's disk overlaps the bearing's shield/cage
  ring (Body11) by 0.877; physically the race stops it first. Watch for rubbing on the first build.
- EXO-350: the printed stator seat O42.10 (r 21.05) grips the bearing's outer race too hard
  to remove. Test prints `parts/stator_d42.2..42.6.3mf`, 0.1 steps (EXO-355; plate `stator_dia.bambu.3mf`) change
  only the seat (module z -11..-4). Not printed yet; when Esteban picks one, carry that radius
  into the stator the generator places.
- EXO-358: the printed 42.1 stators refuse the bearing while 42.2..42.6 are loose; baseline set
  `parts/stator_d41.8..42.1.3mf` (plate `stator_dia_low.bambu.3mf`, + 42.2 .. 42.4). Below 42.1 the seat
  is a ring unioned onto the source (r R..21.10, module z -11..-4). Not printed yet.
- EXO-352: driven home, the stock M2 x 6 horn screws (tip -12.989) stop the servo. Test prints
  `parts/bottom_h0.5.3mf` / `bottom_h1.0.3mf` (plate `rotor_head.bambu.3mf`) = `bottom.3mf` with
  the four head seats counterbored O5.0 0.5 / 1.0 below the joint face (tip -12.489 / -11.989).
  Not printed yet; the chosen depth goes into the generator's bottom, own issue.
