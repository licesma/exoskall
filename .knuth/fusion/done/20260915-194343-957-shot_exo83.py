# EXO-83: the same cameras as shot_exo74_bar.py (the EXO-69 set plus the J2 rim close-ups),
# saved under the 'exo83' prefix, so exo74_bar and exo83 are comparable frame by frame.
#   /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo83.py --timeout 300
# The document must be on the 'bar' placement (the armgen default since EXO-83).
# Touches only the camera.
TAG = 'exo83'
exec(open('/Users/esteban/repos/exo/tools/bridge_scripts/shot_exo58.py').read())
exec(open('/Users/esteban/repos/exo/tools/bridge_scripts/shot_exo74_rim.py').read())
