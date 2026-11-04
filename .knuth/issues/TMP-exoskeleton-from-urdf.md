---
id: TMP-exoskeleton-from-urdf
title: Build upper-body exoskeleton from Gento Luna URDF
description: Adapt the existing G1 leader exoskeleton so every joint axis matches the Gento Luna URDF.
status: backlog
mode: interactive
---
## Context

We have a hand-built upper-body leader exoskeleton for the Unitree G1 (GELLO-style).
Each of the 7 arm joints is one copy of the same **exo joint** assembly. The Gento Luna
arm has the same 7-joint structure, so the plan is to *adapt* the existing exo, not
design from scratch.

Ground truth for all geometry: `.knuth/docs/exo-joint.md` (the joint) and
`gento_luna/exo/NOTES.md` (the 9-item change list and the reference numbers).

## Task

Build the arm from the URDF with a generator: URDF in, arm out.

- **Joints are reused, never generated.** The exo joint is a fixed Fusion design
  (`sensor module 2`, see `.knuth/docs/exo-joint.md`). The generator only *places* one
  copy per URDF joint, origin on the joint frame, Z on the joint axis
  (`fusion_scripts/PlaceModules/PlaceModules.py` already does this).
- **Links are the only generated geometry.** Link `N,N+1` = rotor pad at joint N +
  stator pad at joint N+1 + a body derived from the N→N+1 transform
  (`.knuth/docs/exo-link.md`).

Validate the generator on the arm we already have before pointing it at Gento Luna:
run it on the G1 URDF and compare against `gento_luna/exo/exports/exo_skeleton_2026-08-08.step`.
Ground truth for link `N,N+1` is the `Link N` product *inside that STEP* (exo `Module k`
= URDF joint k+1; the standalone `Link 2.step`/`Link 3.step` are links 23/34).
Joint limits / stop blocks are deferred.

## Done when

- Generator reproduces the existing G1 arm (joint axes + links) within tolerance.
- Generator run on the Gento Luna URDF yields the adapted arm; every NOTES change-list
  item is then either covered by it or explicitly dropped with a reason here.
- A STEP export of the adapted exo exists in `gento_luna/exo/exports/` and a measurement
  script confirms each joint axis is within tolerance of its URDF frame.

## Findings from subissues

- 2026-09-13 (round 1): existing right J1/J2 match the G1 URDF to 0.03 mm / 0.02°.
- The existing right arm is a Fusion *mirror* of the left. `sensor module 2` cannot be
  mirrored by placement, so a generated right arm has its stop blocks on the opposite side
  from the CAD. Physical hardware already does (servos aren't mirrorable) — accepted.
- The existing exo uses an older rotor (8 holes, no index notch); the current
  `sensor module 2` has the notch. Generated links carry the index tab.

## Decisions

- 2026-08-08: wrist pitch (J6) sacrificed, yaw (J7) kept. Module 5 removed.
- Fusion is driven by Esteban; agents parse STEP/URDF, compute displacements, and write
  Fusion scripts — they do not run Fusion themselves.
