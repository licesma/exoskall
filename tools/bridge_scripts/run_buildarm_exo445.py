"""EXO-445: rebuild `generated_R_arm_general` in `exo generated` with the END LINK g_link_7h on
J7's rotor (G1_SPEC['end_link'], armgen END_*: the standard rotor end, manual 01's hand flange and
clamp ear on the outer side, J7's link stop at the END_STOP_RANGE_DEG override) and the EXO-444
wrist side rule. It does NOT save: bracket_exo218.py (J1..J6) and check_end_link_exo445.py (J7 stop
in the model, end link against J5..J7 over the J6 / J7 ranges) run next, and save_exo445.py saves
only after all pass. Refuses to run on any other document.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/run_buildarm_exo445.py --timeout 2700
"""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, sweepgen, BuildArm, BuildArmGen
importlib.reload(armgen); importlib.reload(sweepgen); importlib.reload(BuildArm)
importlib.reload(BuildArmGen)
doc = app.activeDocument
if doc.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r, not %r' % (doc.name, BuildArm.GEN_DOC_NAME))
log('DOCUMENT %r v%s isModified %s; opening %s; JOINT_LABEL_ON %s; wrist_side %s; end_link %s'
    % (doc.name, doc.dataFile.versionNumber, doc.isModified, armgen.actuator_rect(),
       armgen.JOINT_LABEL_ON, BuildArmGen.G1_SPEC.get('wrist_side'), BuildArmGen.G1_SPEC.get('end_link')))
log(BuildArmGen.build_general(app, ui, design, log))
log('NOT SAVED (EXO-445: bracket and end-link checks first, then save_exo445.py); isModified %s' % doc.isModified)
