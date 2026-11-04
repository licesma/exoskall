# EXO-103 screenshots. Two sets, one prefix:
#   1. the whole EXO-81/EXO-96 camera set (link_12 and link_23) under the 'exo103' tag, so the
#      regression can be read frame by frame against generated_R_arm_exo96_*;
#   2. new cameras on link_34, the up-column pair: BOTH joins close up (the coplanar plate
#      leaving the J3 rotor pad, and the column meeting the J4 stator slab with its crown and
#      its blend), the side elevation - the plane the route is drawn in - a plan view down the
#      J3 pad axis where the 52 mm band and the offset tongue are, an iso of the link on its
#      own, link_23 and link_34 together at the joint they share, and the whole arm.
# Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo103.py --timeout 300
# Touches only the camera.
import math
import sys
import importlib

_SRC = '/Users/esteban/repos/exo/tools/bridge_scripts/shot_exo81.py'
_src = open(_SRC).read().replace("TAG = 'exo81'", "TAG = 'exo103'", 1)
if "TAG = 'exo103'" not in _src:
    raise RuntimeError('shot_exo81.py no longer defines TAG on its own line')
exec(_src)

OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/'
vp = app.activeViewport

# link_34's own frame, taken from the document exactly as shot_exo81 takes link_12's: the J3
# wrapper's transform maps the J3 module frame to world and armgen.route gives (u, w).
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
importlib.reload(armgen)
_j34, _by34 = armgen.parse_urdf('/Users/esteban/repos/exo/g1/g1_29dof.urdf')
_sh34, _ = armgen.joint_world_pose('left_shoulder_pitch_joint', _j34, _by34)
_F34 = {}
for _lab, (_nm, _sgn, _yh) in {'J3': ('right_shoulder_yaw_joint', -1, (-1, 0, 0)),
                               'J4': ('right_elbow_joint', -1, (-1, 0, 0))}.items():
    _R, _t = armgen.module_frame(_nm, _j34, _by34, _sgn, _yh)
    _F34[_lab] = (_R, [_t[k] - _sh34[k] for k in range(3)])
_G34 = armgen.route(_F34['J3'], _F34['J4'], 26.0, width=armgen.L_W)['geometry']
U3, W3 = _G34['u'], _G34['w']
AX = _G34['axial']                      # the J4 disk's height over the J3 pad plane
S_MID = (_G34['s_lo'] + _G34['s_hi']) / 2.0

_arm3 = None
for _i in range(design.rootComponent.occurrences.count):
    _o = design.rootComponent.occurrences.item(_i)
    if _o.component.name.startswith('generated_R_arm'):
        _arm3 = _o
_j3 = None
for _i in range(_arm3.childOccurrences.count):
    _o = _arm3.childOccurrences.item(_i)
    if _o.component.name.startswith('J3_'):
        _j3 = _o
_T3 = _j3.transform


def P3(s, w, z):
    """World point (cm) of the link_34-frame station (s, w, z) in mm."""
    q = adsk.core.Point3D.create((U3[0] * s + W3[0] * w) / 10.0,
                                 (U3[1] * s + W3[1] * w) / 10.0, z / 10.0)
    q.transformBy(_T3)
    return (q.x, q.y, q.z)


def D3(s, w, z):
    v = adsk.core.Vector3D.create(U3[0] * s + W3[0] * w, U3[1] * s + W3[1] * w, z)
    v.transformBy(_T3)
    return (v.x, v.y, v.z)


J3_JOIN = P3(26.0, 0.0, 4.0)            # the plate crossing the J3 pad rim
J3_ELBOW = P3(28.0, 0.0, 6.0)           # the elbow: r 5.5 outside, r 3 inside
J4_LAND = P3(S_MID, 0.0, AX)            # the column meeting the J4 stator disk
J4_CROWN = P3(S_MID, -20.0, AX - 20.0)  # where the crown fillet runs into the disk
COLUMN = P3(S_MID, 0.0, AX / 2.0)       # the middle of the column
WHOLE = P3(18.0, 0.0, AX / 2.0)
Z3 = D3(0.0, 0.0, 1.0)                  # the J3 pad axis (the plan view's eye direction)
U3W = D3(1.0, 0.0, 0.0)                 # the reach direction (the plan view's 'up')
W3W = D3(0.0, 1.0, 0.0)                 # across the band (the side view's eye direction)
# The J3 pad axis points along world -Z, so 'up the link' is the link frame's own +z: use it
# as the camera's up vector, and every view reads with the rotor pad at the bottom and the
# J4 slab at the top, the way the side profile is drawn.
UP = D3(0.0, 0.0, 1.0)
# (tag, target, eye direction in the LINK frame, distance cm, view extent cm, up)
views34 = (
    # join 1: the coplanar rotor-pad join, the one Esteban inspects
    ('l34_j3_front', J3_JOIN, D3(0.6, -1.0, 0.8), 40, 5.5, UP),
    ('l34_j3_side', J3_JOIN, D3(0.0, -1.0, 0.3), 40, 5.5, UP),
    ('l34_j3_elbow', J3_ELBOW, D3(0.7, -0.6, 1.0), 30, 4.5, UP),
    # join 2: the stator land - here the column IS the slab, so there is no buried end to see
    ('l34_j4_front', J4_LAND, D3(0.9, -0.7, 0.6), 45, 7.0, UP),
    ('l34_j4_side', J4_LAND, D3(0.0, -1.0, 0.0), 45, 7.0, UP),
    ('l34_j4_crown', J4_CROWN, D3(0.8, -0.8, 0.3), 35, 6.0, UP),
    # the side elevation: the plane the route is drawn in (plate, elbow, column, slab)
    ('l34_side', WHOLE, W3W, 90, 8.5, UP),
    # plan, straight UP the J3 pad axis from under the rotor pad (looking down it would be
    # looking through the J4 module): the 52 mm band, the pad it is tangent to, and the
    # tongue's 15.78 mm offset in silhouette behind it
    ('l34_plan', P3(14.0, 0.0, 4.0), D3(0.0, 0.0, 1.0), 60, 7.0, U3W),
    # the front elevation, looking back along the reach: this is where the lateral offset is,
    # the band centred on the pad and the J4 disk 15.78 mm off it
    ('l34_front', P3(28.5, 0.0, AX / 2.0), D3(-1.0, 0.0, 0.0), 70, 8.5, UP),
    ('l34_iso', P3(16.0, 0.0, AX / 2.0), D3(0.7, -0.8, 0.5), 80, 11.0, UP),
    ('l34_column', COLUMN, D3(0.8, -0.9, 0.2), 40, 6.0, UP),
    # the two links that share joint J3
    ('l23_l34', P3(10.0, 0.0, -20.0), D3(-0.8, -1.0, -0.4), 90, 14.0, UP),
)
for tag, tgt_cm, d, dist, ext, up in views34:
    n = math.sqrt(sum(v * v for v in d))
    tgt = adsk.core.Point3D.create(*tgt_cm)
    cam = vp.camera
    cam.isFitView = False
    cam.target = tgt
    cam.eye = adsk.core.Point3D.create(tgt.x + dist * d[0] / n, tgt.y + dist * d[1] / n,
                                       tgt.z + dist * d[2] / n)
    cam.upVector = adsk.core.Vector3D.create(*up)
    cam.viewExtents = ext
    cam.isSmoothTransition = False
    vp.camera = cam
    adsk.doEvents()
    log('%s -> %s' % (tag, screenshot(OUT + 'generated_R_arm_exo103_%s.png' % tag)))

# the whole arm, fitted
cam = vp.camera
cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
cam.isFitView = True
vp.camera = cam
adsk.doEvents(); vp.fit(); adsk.doEvents()
log('arm_iso -> %s' % screenshot(OUT + 'generated_R_arm_exo103_arm_iso.png'))
