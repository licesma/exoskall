# EXO-69 option A: build the arm with the S in the RISER (EXO-64). EXO-83 made 'bar' the
# armgen default, so this script is now the only way to build the riser placement.
# One-line command:
#   /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/build_s_riser.py --timeout 180
# Identical to run_buildarm.py except that it states the placement explicitly, so the two
# options are selected by the script name and nothing else.
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
armgen.S_PLACEMENT = 'riser'          # after the reload: the reload resets it to the default ('bar')
log('S placement: %s' % armgen.S_PLACEMENT)

text = BuildArm.build_arm(app, ui, design, log=log, interactive=False)
BuildArm.write_report(text)

root = design.rootComponent
after = [root.occurrences.item(i).component.name for i in range(root.occurrences.count)]
log('generated_R_arm* after: %s' % [n for n in after if n.startswith('generated_R_arm')])
