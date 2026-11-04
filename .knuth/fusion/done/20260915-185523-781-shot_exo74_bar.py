# EXO-74: the EXO-69 camera set plus the J2 rim close-ups, saved under the 'bar' prefix.
#   /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo74_bar.py --timeout 300
# Same targets, directions, distances and extents as EXO-69, so exo69_bar and exo74_bar are
# comparable frame by frame. Touches only the camera.
TAG = 'exo74_bar'
exec(open('/Users/esteban/repos/exo/tools/bridge_scripts/shot_exo58.py').read())
exec(open('/Users/esteban/repos/exo/tools/bridge_scripts/shot_exo74_rim.py').read())
