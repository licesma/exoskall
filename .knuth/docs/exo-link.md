# Exo link

A link connects two exo joints (see `exo-joint.md`): it bolts to the **rotor** of one
joint and to the **stator** of the next. Every link is a different body — the
joint-to-joint relationship is what it encodes — but its two ends are always the
same two **pads**.

**Geometry (measurable):**
- `gento_luna/exo/exports/exo_skeleton_2026-08-08.step` — the whole arm; each link is a
  product `Link N` / `Link N R` (left / right). This is the reference for every link.
- `gento_luna/Gento Luna_URDF 20260428/urdf/Link 2.step`, `Link 3.step` — single-body
  exports of two of them, 2026-09-13. Editable source is the Fusion design, not the STEP.

**Renders:** `gento_luna/exo/screenshots/link-2-iso.png`, `link-3-iso.png`.

## Naming

Links are named by the two joints they connect, using URDF joint numbers 1–7:
**`12`** = joint 1 rotor → joint 2 stator, **`23`**, … **`67`**. Suffix `R`/`L` when
the mirrored bodies differ. The chest link is special (joins the two arms' first joints)
and is out of scope for now.

Legacy names: the exo STEP's `Module k` is joint k+1 and `Link N` runs from `Module N-1`
rotor to `Module N` stator, i.e. **`Link N` = link `N,N+1`**: `Link 1 R` = **`12`**,
`Link 2.step` = **`23`**, `Link 3.step` = **`34`** (verified 2026-09-13 by matching the
pads to the module frames: Link 2's rotor pad is on Module 1 (0, 40.33, −2.79)/axis X and
its stator pad on Module 2 (6.24, 40.33, −105.99)/axis Z).

## Anatomy

| Part | Repeats? | What it is |
|---|---|---|
| **Rotor pad** | yes | Round disk. **4× M3 holes** → screws into the nuts captured in the *next* joint's rotor. Carries a small **stop block** that collides with that joint's stator stop block — this pair sets the joint's travel. |
| **Stator pad** | yes | Round disk on the stator underside. **Rectangular cutout** for the *previous* joint's actuator back + connectors to pass through, **3× M3 holes** → that stator's heat-set inserts. |
| **Body** | no | Link-specific geometry between the pads. Sets the relative pose (offset + orientation) of the two joint axes. This is what changes to match the URDF. |

Rotor pad orientation is fixed by the rotor's index notch (see `exo-joint.md`) — the
pad must carry the mating feature so the link can only go on one way.

## Measured pads (mm, in the module frame of the joint they bolt to)

Module frame: origin = rotor-face centre, Z = axis out of the rotor face, +Y = actuator
connector side (the side with no insert). Same numbers left and right.

| Feature | Link 12 (`Link 1 R`) | Link 23 (`Link 2.step`) |
|---|---|---|
| Rotor pad disk | r **26**, z 0 … 8 | r **27**, z 0 … 8 |
| Rotor pad holes | 4× Ø3.2 at r 11, angles 0/90/180/270 | same |
| Link stop block | ring r 20–26, **±5°** about module +X, z −3.6 … 0 | ring r 20–27, ±10°, z −3.5 … 0 |
| Index feature | two Ø1.5 rounded features at r ≈ 10–13 near +X, 1.5 proud (unidentified; exo rotor is an older revision, see joint doc) | none |
| Stator pad disk | r **27**, z −32.5 … −24.5 (on the stator underside) | same |
| Stator pad holes | 3× Ø3.2 at r 19.25, angles 0/180/270 | same |
| Actuator cutout | x −10 … 10, y −9.5 … 24.5 (= XL330 envelope, no clearance) | same |
| Body | L-bracket: 8 mm pad plate extended to the stator slab, vertical plate in the slab thickness; Ø3 hole with Ø10 counterbore and a Ø5 hole on the vertical plate; big r 400 / r 500 arcs | 25 mm wide bar with r 12.5 / r 15 rounds |

Link 12 layout in the J1 frame: J2 origin at (0, −13.8, 38.0), J2 axis ⊥ J1 axis, stator
slab at J1-x ∈ [24.5, 32.5] (left) / [−32.5, −24.5] (right); the J2 stator rim is 11 mm
above the rotor-pad plane (3 mm clearance over the 8 mm plate).

Link 23 layout in the J2 frame (left): J3 origin at (−103.2, 0, 6.24), J3 axis = −X_J2,
J3 connector side (+Y_J3) = −Z_J2. **Representative body pattern ("bar")**: the rotor pad plate
is extended along the bar direction to s = 35; a riser at s ∈ [28.4, 34.9] (just outside the
module radius) drops to the bar level z = −33 … −25.2; a 25 × 7.8 bar runs to a 25-wide
flange that extends the J3 stator pad to Y_J3 = 39.24, i.e. the bar lands on the connector
side 6.85 mm past the actuator cutout. The J2 stop block is ±10° at 270° (−Y). Note the G1
URDF puts J3 6.25 mm outboard (−Y) of J2 while the exo puts it 6.24 mm forward (+X): 8.83 mm
apart; the exo deviates from the URDF here.

## Measured extents (bbox, mm)

| Link | x × y × z |
|---|---|
| `12` (`Link 1 R`) | 55.2 × 68.3 × 53.3 |
| `23` (Link 2.step) | 58.5 × 67.8 × 89.7 |
| `34` (Link 3.step) | 82.0 × 51.3 × 212.5 |

## Generator

`fusion_scripts/BuildArm/` places the joints and builds links 12 (L-bracket) and 23 (bar)
from the numbers above (`armgen.py` holds the constants and the bar rule
`link_bar_geometry`; `tools/test_armgen.py` checks frames and bar geometry against the exo STEP).

## Known/open

- **Assembly mark (EXO-341)**: every generated link's stator-pad face carries a 0.6 x 0.4 deep radial line at the J(b) module +Y (r 23.5..27.6, `armgen.ALIGN_MARK_*`, cut in `BuildArmGen.build_link`); the matching line on the stator underside is cut only on the export copy (`stator EXO-341 export` in `exo generated`, `tools/bridge_scripts/export_link_stator_exo341.py`), not on the module the arms place.
- The Ø1.5 features on `Link 1 R`'s rotor pad are not identified; the current rotor's notch
  (`exo-joint.md`) has no mating feature on any exported link.
- Stop-block angular placement per link vs URDF limits — not yet derived.
- Which link (if any) is the chest link in the exo STEP (`Link 1` vs `chest mount`) — not mapped.
