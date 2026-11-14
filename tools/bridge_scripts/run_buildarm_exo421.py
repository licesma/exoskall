"""EXO-421: rebuild `generated_R_arm_general` in `exo generated` with the joint numbers
(`armgen.JOINT_LABEL_ON`: joint a's number on the rotor pad's rim, joint b's on the stator pad's
outer face, every link 12..67), as run_buildarm_exo419.py. It does NOT save: the bracket
(bracket_exo218.py) runs first, and save_exo421.py saves only after both pass. Refuses to run on
any other document.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/run_buildarm_exo421.py --timeout 2700
"""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, sweepgen, BuildArm, BuildArmGen
importlib.reload(armgen); importlib.reload(sweepgen); importlib.reload(BuildArm)
importlib.reload(BuildArmGen)
doc = app.activeDocument
if doc.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r, not %r' % (doc.name, BuildArm.GEN_DOC_NAME))
log('DOCUMENT %r v%s isModified %s; ACT_OPEN_MARGIN %.3f -> opening %s; JOINT_LABEL_ON %s (%s %.1f, %.1f deep)'
    % (doc.name, doc.dataFile.versionNumber, doc.isModified, armgen.ACT_OPEN_MARGIN,
       armgen.actuator_rect(), armgen.JOINT_LABEL_ON, armgen.JOINT_LABEL_FONT, armgen.JOINT_LABEL_H,
       armgen.JOINT_LABEL_D))
log(BuildArmGen.build_general(app, ui, design, log))
log('NOT SAVED (EXO-421: bracket first, then save_exo421.py); isModified %s' % doc.isModified)
