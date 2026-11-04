# EXO-74: the EXO-69 camera set plus the J2 rim close-ups, saved under the 'riser' prefix.
#   /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo74_riser.py --timeout 300
# Same targets, directions, distances and extents as EXO-69, so exo69_riser and exo74_riser are
# comparable frame by frame. Touches only the camera.
TAG = 'exo74_riser'
exec(open('/Users/esteban/repos/exo/tools/bridge_scripts/shot_exo58.py').read())
exec(open('/Users/esteban/repos/exo/tools/bridge_scripts/shot_exo74_rim.py').read())
