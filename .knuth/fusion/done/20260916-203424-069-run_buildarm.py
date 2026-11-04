"""Run BuildArm.build_arm through the bridge, write the report file, log it.

Builds armgen's default S placement, which is 'bar' since EXO-83 (Esteban's choice);
build_s_riser.py / build_s_bar.py state a placement explicitly.

Also logs the root occurrence count before/after so two runs in a row prove the
`generated_R_arm*` cleanup does not accumulate components.
"""
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)

root = design.rootComponent
before = [root.occurrences.item(i).component.name for i in range(root.occurrences.count)]
log('root occurrences before: %d (generated_R_arm*: %d)' % (
    len(before), sum(1 for n in before if n.startswith('generated_R_arm'))))

text = BuildArm.build_arm(app, ui, design, log=log, interactive=False)
BuildArm.write_report(text)

after = [root.occurrences.item(i).component.name for i in range(root.occurrences.count)]
log('root occurrences after: %d (generated_R_arm*: %d) -> %s' % (
    len(after), sum(1 for n in after if n.startswith('generated_R_arm')),
    [n for n in after if n.startswith('generated_R_arm')]))
