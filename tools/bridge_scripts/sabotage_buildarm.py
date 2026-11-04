# KNUTH-2 negative test: build with the riser INSIDE the module/slab radius and the pure-Python
# clearance guards disabled, so only the Fusion interference check can catch it. Expect
# build_arm to raise 'interference FAIL' and buildarm_report.txt to hold FAIL lines.
# Run run_buildarm.py afterwards to restore a clean generated_R_arm.
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
armgen.RISER_CLEAR = -3.0
armgen.MIN_CLEARANCE = -10.0
try:
    BuildArm.build_arm(app, ui, design, log=log, interactive=False)
except RuntimeError as e:
    log('EXPECTED RAISE: %s' % e)
    raise
log('UNEXPECTED: build_arm returned without raising')
raise RuntimeError('sabotage not detected')
