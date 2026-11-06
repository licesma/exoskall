"""SUPERSEDED by EXO-383 (pad 8 + counterbore variants: run_cbore_*_exo383.py).
EXO-375: build the G1 arm with the 6 mm rotor-pad VARIANT (M3x18) for export only - same
builder, `armgen.ROTOR_PAD_T` set to 6.0 after the reload. NOT saved here: the document is
rebuilt at the default 7 and saved right after (`run_buildarm_general.py`).
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/run_buildarm_general_p6_exo375.py --timeout 900"""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, sweepgen, BuildArm, BuildArmGen
importlib.reload(armgen); importlib.reload(sweepgen); importlib.reload(BuildArm)
importlib.reload(BuildArmGen)
armgen.ROTOR_PAD_T = 6.0
log('VARIANT: armgen.ROTOR_PAD_T = %.1f (default %.1f)' % (armgen.ROTOR_PAD_T, armgen.ROTOR_PAD_T_DEFAULT))
log(BuildArmGen.build_general(app, ui, design, log))
