# exoskall

Scripts that build a 7-joint exoskeleton arm in Fusion 360.

- Generate the arm, its links and the `sensor module 2` joint parts (stator and rotor bottom; since EXO-370 the rotor top is fused into every link's rotor pad) from scripts (`fusion_scripts/BuildArm/BuildArm.py`, `BuildArmGen.py`).
- Export the printable parts as 3MF (`parts/`) and lay them out as Bambu Studio plates (`tools/bambu_plate_exo343.py`).
- Measure and verify the exported parts: watertight, one shell, volumes checked against Fusion (`tools/verify_3mf.py`).
