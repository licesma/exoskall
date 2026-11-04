# EXO-69: the EXO-58/EXO-64 camera set, saved under the 'riser' S-placement prefix.
#   /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo69_riser.py --timeout 180
# Same targets, directions, distances and extents for both options, so the two sets are
# comparable frame by frame. Touches only the camera.
TAG = 'exo69_riser'
exec(open('/Users/esteban/repos/exo/tools/bridge_scripts/shot_exo58.py').read())
