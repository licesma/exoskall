"""EXO-444: rebuild `generated_R_arm_general` in `exo generated` with the wrist side rule
(G1_SPEC['wrist_side']: among tied axis-sign combinations J6 sits on the inner side of the J7 axis,
expected J6 -1, J7 +1, slide -29.906; and nothing from J5 to J7 reaches more than 40 mm to the
outer side of the J7 axis, checked on the routes before the build and on the solids after it).
As run_buildarm_exo421.py: it does NOT save; the bracket (bracket_exo218.py) runs next and
save_exo444.py saves only after both pass. Refuses to run on any other document.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/run_buildarm_exo444.py --timeout 2700
"""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, sweepgen, BuildArm, BuildArmGen
importlib.reload(armgen); importlib.reload(sweepgen); importlib.reload(BuildArm)
importlib.reload(BuildArmGen)
doc = app.activeDocument
if doc.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r, not %r' % (doc.name, BuildArm.GEN_DOC_NAME))
log('DOCUMENT %r v%s isModified %s; opening %s; JOINT_LABEL_ON %s; wrist_side %s'
    % (doc.name, doc.dataFile.versionNumber, doc.isModified, armgen.actuator_rect(),
       armgen.JOINT_LABEL_ON, BuildArmGen.G1_SPEC.get('wrist_side')))
log(BuildArmGen.build_general(app, ui, design, log))
log('NOT SAVED (EXO-444: bracket first, then save_exo444.py); isModified %s' % doc.isModified)
