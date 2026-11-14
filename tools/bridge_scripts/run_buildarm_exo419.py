"""EXO-419: rebuild `generated_R_arm_general` in `exo generated` with ACT_OPEN_MARGIN 0.1
(link actuator opening 20.2 x 34.2), as run_buildarm_exo415.py. Same build as run_buildarm_general.py (EXO-394/395) but it
does NOT save: the bracket (bracket_exo218.py) runs first, and save_exo419.py saves only after
both pass. Refuses to run on any other document.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/run_buildarm_exo419.py --timeout 2700
"""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, sweepgen, BuildArm, BuildArmGen
importlib.reload(armgen); importlib.reload(sweepgen); importlib.reload(BuildArm)
importlib.reload(BuildArmGen)
doc = app.activeDocument
if doc.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r, not %r' % (doc.name, BuildArm.GEN_DOC_NAME))
log('DOCUMENT %r v%s isModified %s; ACT_OPEN_MARGIN %.3f -> opening %s'
    % (doc.name, doc.dataFile.versionNumber, doc.isModified, armgen.ACT_OPEN_MARGIN,
       armgen.actuator_rect()))
log(BuildArmGen.build_general(app, ui, design, log))
log('NOT SAVED (EXO-419: bracket first, then save_exo419.py); isModified %s' % doc.isModified)
