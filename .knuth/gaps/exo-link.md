# Gaps — exo-link

- open: BuildArm links 12/23 not yet seen in Fusion (agent cannot run Fusion); overlay vs `Link 1 R` / `Link 2 R` pending Esteban's screenshot.
- open: the identical-pose bug is explained from the API docs (pending position capture reset by the base-feature compute), not observed in a debugger; the round-2 read-back in buildarm_report.txt is what confirms or refutes it.
- open: link bodies are now pre-transformed into the arm frame with their components at identity, so `bRepBodies.add` coordinate space no longer matters as long as the arm component stays at identity (read back and reported).
- open: the two Ø1.5 features on `Link 1 R`'s rotor pad are unidentified; the generated tab matches the current rotor notch instead and has not been fit-checked.
- open: stop block reproduced as a parallel-sided 4 mm box (original has radial ±5° faces); angle vs URDF limits not derived.
- open: link 12's small holes on the vertical plate (Ø3/Ø10 counterbore, Ø5) not reproduced — purpose unknown.
- open: the Gento URDF has zero J1/J2 skew; the L-bracket generator has only been checked on the G1 layout (perpendicular axes, J2 above the pad).
- open: link 23 fillets (r 15/12.5/8.5/5) and the r 7.9 relief are not reproduced; riser is 25 wide (Link 2: 36).
- open: `root.occurrences.addByInsert` copy is deleted after reuse; if `deleteMe` fails a stray module sits at the origin (reported as WARNING).
- open: J3 generated 8.83 mm from exo `Module 2 R` — a URDF-vs-exo deviation, unverified which of the two Esteban wants the arm to follow.
