# parts/hand_candidates: manual hand-side parts to choose from (EXO-434, 2026-11-16)

The generator stops at link_67 (J6 rotor to J7 stator). Nothing is generated after J7. These are
all the hand-side parts of the hand-made ("manual") design, so you can pick one.

**Main place to look: Fusion, folder `/hand/candidates`** (hub Estebans, project G1 exeskeleton).
One new document per candidate, named `NN short_name` like the files here. Each holds solid
(B-rep) copies of the bodies in their original frame (assembly frame for parts that live in an
assembly, document frame otherwise), so you can measure and edit them. Document 01 also holds the
manual J7 parts and the hand base as see-through reference bodies named `ref ...`. The original
documents and assemblies were only read: none was modified, moved or saved.

This folder is the secondary overview: a 3MF (High refinement, mesh of the same bodies, same
frame, one object per body) and a picture per candidate, plus `overview.png` (all candidates at
one scale, except 15). Every 3MF was checked with `tools/verify_3mf.py`: closed meshes, and each
body's mesh volume equals the export of the original body.

## Which one the manual arm used

In its latest G1 assembly (`arm complete` v35, 2026-05-28, and the identical copies `arm test`
2026-08-08 and `arm complete BACKUP 2026-08-08`) the right arm ends with:
J7 (`Module 6 R`) -> **01** (component `cuff v12(Mirror)`) -> hand base (`right v9`), with clamp
**07** hanging from 01, enclosure **10** around the hand base and clamp **11** hanging from 10.
The left arm uses the same link body (named `Link7`) with clamp **08**, enclosure **12** and clamp **13**.
`arm complete luna` (2026-09-19, the Luna arm) ends with **06** instead; 01 and its clamps are hidden there.

## How it must attach to the generated J7

Generated J7 has no separate rotor top: since EXO-370/394 the rotor top is part of every link
(the "rotor pad"). The J7 module only has the rotor bottom: joint face at module z -8.0, 8 pegs
O3.0 at r 12.2, 4 captured M3 nuts at r 11, journal O30.1. So whatever goes on J7 must bring its
own rotor pad (8 thick pad, O30.1 x 3.0 journal, 8 sockets O3.3 at r 12.2, 4x O3.2 at r 11 with
O6.2 x 2.0 counterbores, r 8 recess, O13.8 centre hole), exactly like link_12..link_67 do.

All manual last links (01 to 06) instead have a flat 7 mm plate with only **4x O3.2 at r 11.000,
90 deg apart**, made to sit on the manual module's separate printed rotor top. The r 11 circle
matches the generated nut circle, but nothing else of the rotor end is there. **None fits the
generated J7 as is.**

## Candidates (most likely "last link to the hand" first)

Dates and versions are those of the source in Rodrigo's hub (EXO-271 inventory); the Estebans
copies are all v1 from the 2026-10-15 migration. Size = bounding box of all bodies, mm.

| # | File | Picture | Fusion document | Name and source | Date | Size (mm) | What it is | How it attaches | Fits generated J7 as is | Note |
|---|---|---|---|---|---|---|---|---|---|---|
| 01 | `01_last_link_cuff_v12.3mf` | `01_last_link_cuff_v12.png` | `/hand/candidates/01 last_link_cuff_v12` | Component `cuff v12(Mirror)` in `arm complete` v35, right arm (1 body, V 39673 mm3). Same body as left arm `Link7`, in `arm test`, `arm complete BACKUP 2026-08-08`, and hidden in the `cuff` document v17 | 2026-05-28 | 72.5 x 81.8 x 40.5 | The last link the manual arm used: round plate on the J7 rotor, a flange for the hand base, an ear for the wrist clamp | J7 side: 7 mm plate, 4x O3.2 at r 11 on the J7 axis. Hand side: flat face 46.5 mm from the J7 axis with 4x O3.3 on a 10 x 30 rectangle (5 deep). Clamp ear: 2x O4.5 with O8 counterbores, 20.2 apart (M4) | No: no rotor pad (no journal, sockets, counterbores); needs an adapter or a new link | Best starting point. Fusion doc also has `ref` J7 rotor top, rotor bottom, stator, XL330 and the hand base |
| 02 | `02_writst_mount_e.3mf` | `02_writst_mount_e.png` | `/hand/candidates/02 writst_mount_e` | Document `/hand/cuff/writst mount_e` (also root copy `writst mount_e`, and referenced hidden in both `wrist_sketch3_with_straps`) | 2026-04-12 (v8) | 72.5 x 70.0 x 40.5 | Earlier last link: same plate and hand flange as 01, no clamp ear | Same J7 plate (4x O3.2 r 11) and same 10 x 30 hand flange as 01 | No, same reason as 01 | Simpler version of 01 |
| 03 | `03_gen1_last_link_plate.3mf` | `03_gen1_last_link_plate.png` | `/hand/candidates/03 gen1_last_link_plate` | `arm` (gen1, `/arm/old`): hidden `Component36` + visible root `Body28` (2 bodies) | 2026-04-13 (v41) | 81.5 x 81.2 x 50.8 | Gen1 last link with strap slots, plus an 8 mm hand plate | J7 plate 4x O3.2 r 11; hand flange 10 x 30 O3.3; plate Body28 carries the same 10 x 30 pattern | No, same reason as 01 | Gen1 arm has no other hand part |
| 04 | `04_writst_mount.3mf` | `04_writst_mount.png` | `/hand/candidates/04 writst_mount` | Document `/hand/cuff/writst mount` (10 bodies: two link variants side by side, each with a strap arc and spacer bars) | 2026-04-13 (v1) | 286.5 x 148.0 x 80.6 | Two variants of a last link with a fixed strap arc for the wrist | J7 plate 4x O3.2 r 11; hand flange 10 x 30 O3.3; strap parts with O5.5 holes | No, same reason as 01 | Exploration; two variants in one file |
| 05 | `05_writst_mount_c.3mf` | `05_writst_mount_c.png` | `/hand/candidates/05 writst_mount_c` | Document `/hand/cuff/writst_mount_c` (4 bodies: two variants) | 2026-04-11 (v3) | 222.5 x 188.0 x 104.0 | Two variants of a last link with a large curved wrist rest | J7 plate 4x O3.2 r 11; hand flange 10 x 30 O3.3; rest with O5.5 holes | No, same reason as 01 | Exploration |
| 06 | `06_luna_hand_link_plate.3mf` | `06_luna_hand_link_plate.png` | `/hand/candidates/06 luna_hand_link_plate` | `arm complete luna` v16: `Hand link R` + `Hand adapter plate R` (2 bodies; left arm copies identical) | 2026-09-19 | 68.0 x 82.7 x 125.0 | Luna arm's last link: a column from the last module to a square hand flange, plus a 4 mm adapter plate | Module side 4x O3.2 at r 11; hand side 4x O4.3 on a 26.9 mm square (r 19) with a 50 x 50 x 4 plate (O8 centre) | No, same reason as 01; also made for Luna's geometry, not G1 | Newest manual hand part, but for Luna |
| 07 | `07_wrist_clamp_v12.3mf` | `07_wrist_clamp_v12.png` | `/hand/candidates/07 wrist_clamp_v12` | `arm complete` v35 right arm, `cuff v12(Mirror)` > `hook v3 (1)`: side0 (3 bodies), lock, spring, Component107 (6 bodies; a hidden second spring left out) | 2026-05-28 | 40.0 x 96.1 x 72.3 | Hinged wrist clamp with spring lock, the one used on 01 | Hangs on 01's ear with 2x M4 | Not a J7 part (goes on 01) | Used on the right arm |
| 08 | `08_wrist_clamp_soft_inside.3mf` | `08_wrist_clamp_soft_inside.png` | `/hand/candidates/08 wrist_clamp_soft_inside` | Document `/hand/cuff/cuffs with soft inside` (= left arm of `arm complete`, `arm test`, BACKUP, luna): 9 bodies, 3 of them soft inserts; the 3 M4 nuts left out | 2026-06-01 (v10; arm complete pins v9) | 40.0 x 96.1 x 80.2 | Newer wrist clamp with soft inner liners | Same M4 hanging point as 07 (on 01 / Link7) | Not a J7 part | Newest clamp |
| 09 | `09_clamp_cuff_doc_v17.3mf` | `09_clamp_cuff_doc_v17.png` | `/hand/candidates/09 clamp_cuff_doc_v17` | Document `/hand/cuff/cuff`, sub-assembly `hook v3` (6 bodies, mostly hidden in the doc) | 2026-05-28 (v17) | 72.3 x 96.1 x 40.0 | Last edited state of the clamp in the `cuff` doc; small differences from 07 (side0, lock, spring volumes) | Same as 07 | Not a J7 part | Doc frame, not the arm frame |
| 10 | `10_hand_enclosure_right_v9.3mf` | `10_hand_enclosure_right_v9.png` | `/hand/candidates/10 hand_enclosure_right_v9` | `arm complete` v35 right arm, `right v9` Body34/35/36/38 (4 bodies) | 2026-04-19 (`right` v9) | 40.3 x 93.1 x 102.0 | Ring enclosure in 4 pieces around the hand base | Around the hand base (`right v9` Body1), not on J7; carries clamp 11 | Not a J7 part | Used on the right arm |
| 11 | `11_hand_side_clamp_right_v9.3mf` | `11_hand_side_clamp_right_v9.png` | `/hand/candidates/11 hand_side_clamp_right_v9` | `arm complete` v35, `right v9` > `cuff v12(Mirror) (1)` > `hook v3(Mirror)` (6 bodies) | 2026-04-19 | 30.0 x 96.1 x 72.3 | Second, narrower clamp further toward the hand | Hangs from enclosure 10 | Not a J7 part | Used on the right arm |
| 12 | `12_hand_enclosure_left.3mf` | `12_hand_enclosure_left.png` | `/hand/candidates/12 hand_enclosure_left` | `arm complete` v35 left arm, `Hand enclosure` Component204/206/215/216 (4 bodies; also arm test, BACKUP, hidden in luna) | 2026-05-28 | 40.3 x 93.0 x 102.0 | Left arm version of 10 (different pieces) | Around the hand base | Not a J7 part | Mirror it for a right arm |
| 13 | `13_hand_side_clamp_left.3mf` | `13_hand_side_clamp_left.png` | `/hand/candidates/13 hand_side_clamp_left` | `arm complete` v35 left arm, `cuff v12(Mirror) (1) (1)` > `hook v3(Mirror) (1)` (9 bodies, 3 soft inserts) | 2026-05-28 | 30.0 x 96.1 x 80.2 | Left arm version of 11, with soft inserts | Hangs from enclosure 12 | Not a J7 part | |
| 14 | `14_wuji_enclosure.3mf` | `14_wuji_enclosure.png` | `/hand/candidates/14 wuji_enclosure` | Document `/hand/robot/wuji/wuji enclosure` (1 body) | 2026-05-27 (v3) | 40.3 x 52.0 x 102.0 | One enclosure piece, same place as Component215 of 12 but a newer shape (V 26085 vs 24912) | Around the hand base | Not a J7 part | |
| 15 | `15_clamp_parts_april_10x.3mf` | `15_clamp_parts_april_10x.png` | `/hand/candidates/15 clamp_parts_april_10x` | Documents `/hand/cuff/hook`, `lock`, `spring`, `side0` (5 bodies, each in its own doc frame, so they overlap) | 2026-04-11 (v3) | 856 x 930 x 300 | Early clamp parts, **drawn 10x too big** (spring = exactly 1000x the volume of the assembly spring) | As 07 once scaled 0.1 | Not a J7 part | Left at drawn size; do not print as is |
| 16 | `16_wrist_sketch3_strap_mesh.3mf` | `16_wrist_sketch3_strap_mesh.png` | `/hand/candidates/16 wrist_sketch3_strap_mesh` | Document `/hand/cuff/wrist_sketch3_with_straps`, body `MeshBody1 (1)` (a solid made from a mesh, 9692 faces; the `(copy v1)` doc is identical) | 2026-03-07 (v4) | 189.0 x 95.6 x 188.1 | Honeycomb forearm and hand brace concept with a mount plate | Not measured (mesh-made faces) | Unknown, mesh only | Oldest idea |

Left out: `wristband_2.0` (only a mesh body, about 2016 x 1644 x 35 mm as drawn, so probably 10x;
its 3MF export wrote no file and a mesh cannot be copied as a solid), `inspire_link` (empty
document: no bodies, sketches or features), `/other/Direct-Adapter-Mount` (60 x 52 x 30, nothing
ties it to the hand), the hand base itself (`right v9` Body1, 56 x 56 x 71: the thing the link
carries, included as `ref` in Fusion doc 01), and a hidden stray copy of a soft insert under the
left arm `Hand` component.

## What is missing either way

- A J7 end for the generated arm. Two ways: (a) a new generated part "link_7h": the generated
  rotor pad (as on every link) plus 01's hand flange (4x O3.3, 10 x 30, 46.5 from the axis) and
  clamp ear; or (b) a printed adapter disc = the generated rotor pad alone, with 01 bolted on top
  through the same 4 holes at r 11 (M3 about 7 mm longer than the M3x18 of the links; not checked).
  The archived separate rotor top (EXO-333) predates the EXO-394 bottom with pegs and should not
  be expected to fit.
- J7 itself still needs its stator and rotor bottom printed (the joints67 plate has stators; a
  rotor bottom for J7 is needed too).
- Clearance of 01's ears and clamp against the generated J7 stator and its 15 deg stop block, and
  the J7 range, were not checked. 01 has no link stop block, so J7 would be limited only by contact.
- Screws: M3 for J7, M3 for the hand flange, M4 for the clamp hinge (M4 x 25/35 and nuts are in
  the assemblies), springs for the clamp locks.
- Left arm: only the left arm assemblies carry 08, 12, 13; a right arm needs mirrored copies.
