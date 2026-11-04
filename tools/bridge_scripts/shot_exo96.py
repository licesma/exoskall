# EXO-96: exactly the EXO-81 camera set (the 13 EXO-83/EXO-74 cameras on link_23 plus the 7
# on link_12), saved under the 'exo96' prefix, so exo81 and exo96 are comparable frame by
# frame - which is the point: the refactor must not move a pixel.
#   /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo96.py --timeout 300
# shot_exo81.py hard-codes its own TAG, so its source is read and that one line replaced;
# nothing else about the cameras changes. Touches only the camera.
_SRC = '/Users/esteban/repos/exo/tools/bridge_scripts/shot_exo81.py'
_src = open(_SRC).read().replace("TAG = 'exo81'", "TAG = 'exo96'", 1)
if "TAG = 'exo96'" not in _src:
    raise RuntimeError('shot_exo81.py no longer defines TAG on its own line')
exec(_src)
