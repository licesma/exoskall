"""EXO-383: build the G1 arm with a counterbore-depth VARIANT for export only - same builder,
`armgen.ROTOR_CBORE_D` set to CBORE_D (default 1.5) after the reload. NOT saved here: the
document is rebuilt at the default and saved afterwards (`run_buildarm_general.py`).
Run: CBORE_D is read from the wrapper `run_cbore_1.5_exo383.py` / `run_cbore_2.0_exo383.py`."""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, sweepgen, BuildArm, BuildArmGen
importlib.reload(armgen); importlib.reload(sweepgen); importlib.reload(BuildArm)
importlib.reload(BuildArmGen)
armgen.ROTOR_CBORE_D = float(globals().get('CBORE_D', 1.5))
log('VARIANT: armgen.ROTOR_CBORE_D = %.1f (default %.1f), label %s' % (armgen.ROTOR_CBORE_D, armgen.ROTOR_CBORE_D_DEFAULT, armgen.rotor_label_text()))
log(BuildArmGen.build_general(app, ui, design, log))
