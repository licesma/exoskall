"""Run BuildArmGen.build_general through the bridge (EXO-144 method, EXO-150 spin rule)."""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, sweepgen, BuildArm, BuildArmGen
importlib.reload(armgen); importlib.reload(sweepgen); importlib.reload(BuildArm)
importlib.reload(BuildArmGen)
log(BuildArmGen.build_general(app, ui, design, log))
