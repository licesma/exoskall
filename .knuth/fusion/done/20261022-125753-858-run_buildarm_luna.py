"""EXO-218: build the Gento Luna right arm into `generated_R_arm_luna`, beside the G1 one.

Same builder, same rules, a different ARM SPEC: the Luna URDF, Arm_R1..R7_Joint, one WIDTH
per link defaulted to the G1 arm's link of the same index, and the axis signs and slides
precomputed by `tools/luna_solve.py` and VERIFIED here (`sweepgen.verify_solved`).

EXO-242: the document is SAVED here too - before and after the build - and the report's
last line names it. An unsaved document is SHOUTED about, never just mentioned.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/run_buildarm_luna.py --timeout 2700
"""
import importlib
import sys

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
import sweepgen
import BuildArm
import BuildArmGen
importlib.reload(armgen)
importlib.reload(sweepgen)
importlib.reload(BuildArm)
importlib.reload(BuildArmGen)

log('DOCUMENT before the build:')
log(BuildArm.save_active_document(app))
log(BuildArmGen.build_luna(app, ui, design, log))
log('DOCUMENT after the build:')
log(BuildArm.save_active_document(app))
