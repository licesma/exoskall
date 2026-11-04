"""Run BuildArmGen.build_general through the bridge (EXO-144 method, EXO-150 spin rule).

EXO-242: the document is SAVED by this script - once before the build, so the file exists
even if the build dies, and once after, so the arm is on disk the moment the report is
written. The last line of the report names the document; a document that is still unsaved
when the run ends is SHOUTED about (`BuildArm.SAVE_SHOUT`), because that is exactly how
every build since EXO-193 was lost.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/run_buildarm_general.py --timeout 2700
"""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, sweepgen, BuildArm, BuildArmGen
importlib.reload(armgen); importlib.reload(sweepgen); importlib.reload(BuildArm)
importlib.reload(BuildArmGen)

log('DOCUMENT before the build:')
log(BuildArm.save_active_document(app))
log(BuildArmGen.build_general(app, ui, design, log))
log('DOCUMENT after the build:')
log(BuildArm.save_active_document(app))
