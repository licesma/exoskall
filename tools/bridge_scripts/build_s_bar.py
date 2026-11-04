# EXO-69 option B: build the arm with the S in the straight BAR, between the end of the
# bend and the stator slab; riser and bend stay straight, full width and flush with the
# J2 pad. One-line command:
#   /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/build_s_bar.py --timeout 180
# Identical to build_s_riser.py / run_buildarm.py except for the placement, so the option
# is selected by the script name and nothing else.
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
armgen.S_PLACEMENT = 'bar'            # after the reload: the reload resets it to the default
log('S placement: %s' % armgen.S_PLACEMENT)

text = BuildArm.build_arm(app, ui, design, log=log, interactive=False)
BuildArm.write_report(text)

root = design.rootComponent
after = [root.occurrences.item(i).component.name for i in range(root.occurrences.count)]
log('generated_R_arm* after: %s' % [n for n in after if n.startswith('generated_R_arm')])
