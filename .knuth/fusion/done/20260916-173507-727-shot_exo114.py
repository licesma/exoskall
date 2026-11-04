# EXO-114 screenshots. Two sets, one prefix:
#   1. the whole EXO-103 camera set (link_12, link_23, link_34) re-shot under the 'exo114'
#      tag, so the regression can be read frame by frame against generated_R_arm_exo103_*;
#   2. new cameras on link_45, the ramp: BOTH joins close up (the coplanar plate leaving the
#      J4 rotor pad, and the run landing on the J5 stator disk's rim), the side elevation -
#      where the S that climbs over the actuator cutout is - a plan view down the J4 pad axis
#      and a front view along the reach, which is where the PLAN TAPER shows (the band is
#      tangent to the rotor rim on one side and to the stator disk on the other), an iso of
#      the link on its own, link_34 and link_45 at the joint they share, and the whole arm.
# Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo114.py --timeout 420
# Touches only the camera.
import math
import sys
import importlib

_SRC114 = '/Users/esteban/repos/exo/tools/bridge_scripts/shot_exo103.py'
_src114 = open(_SRC114).read().replace('exo103', 'exo114')
if "generated_R_arm_exo114_" not in _src114:
    raise RuntimeError('shot_exo103.py no longer writes generated_R_arm_exo103_*')
exec(_src114)

OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/'
vp = app.activeViewport

# link_45's own frame, taken from the document exactly as the earlier sets take theirs.
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
importlib.reload(armgen)
_j45, _by45 = armgen.parse_urdf('/Users/esteban/repos/exo/g1/g1_29dof.urdf')
_sh45, _ = armgen.joint_world_pose('left_shoulder_pitch_joint', _j45, _by45)
_F45 = {}
for _lab, (_nm, _sgn, _yh) in {'J4': ('right_elbow_joint', -1, (-1, 0, 0)),
                               'J5': ('right_wrist_roll_joint', +1, (0, 1, 0))}.items():
    _R, _t = armgen.module_frame(_nm, _j45, _by45, _sgn, _yh)
    _F45[_lab] = (_R, [_t[k] - _sh45[k] for k in range(3)])
_G45 = armgen.route(_F45['J4'], _F45['J5'], 27.0, width=44.0)['geometry']
U4, W4 = _G45['u'], _G45['w']
TP = _G45['taper']
S45 = (_G45['s_lo'] + _G45['s_hi']) / 2.0          # the middle of the J5 slab
ZRUN = (_G45['bar_z'][0] + _G45['bar_z'][1]) / 2.0  # the run's own level
CW = TP['centre']                                   # the band's centre, w_off / 2

_arm4 = None
for _i in range(design.rootComponent.occurrences.count):
    _o = design.rootComponent.occurrences.item(_i)
    if _o.component.name.startswith('generated_R_arm'):
        _arm4 = _o
_j4 = None
for _i in range(_arm4.childOccurrences.count):
    _o = _arm4.childOccurrences.item(_i)
    if _o.component.name.startswith('J4_'):
        _j4 = _o
_T4 = _j4.transform


def P4(s, w, z):
    """World point (cm) of the link_45-frame station (s, w, z) in mm."""
    q = adsk.core.Point3D.create((U4[0] * s + W4[0] * w) / 10.0,
                                 (U4[1] * s + W4[1] * w) / 10.0, z / 10.0)
    q.transformBy(_T4)
    return (q.x, q.y, q.z)


def D4(s, w, z):
    v = adsk.core.Vector3D.create(U4[0] * s + W4[0] * w, U4[1] * s + W4[1] * w, z)
    v.transformBy(_T4)
    return (v.x, v.y, v.z)


J4_JOIN = P4(27.0, CW, 4.0)                 # the plate crossing the J4 pad rim
J4_TAPER = P4(20.0, TP['w_hi'], 4.0)        # the taper arc leaving the rim, in plan
RISE = P4(_G45['rise_s'][0] + _G45['rise_run'] / 2.0, CW, _G45['rise'] / 2.0)
J5_LAND = P4(S45, CW, ZRUN)                 # the run landing on the J5 disk's rim
J5_BLEND = P4(S45, TP['w_lo'], ZRUN - 6.0)  # the tongue's blend into the rim
WHOLE45 = P4(34.0, CW, ZRUN / 2.0)
UP4 = D4(0.0, 0.0, 1.0)
U4W = D4(1.0, 0.0, 0.0)
W4W = D4(0.0, 1.0, 0.0)
# (tag, target, eye direction in the LINK frame, distance cm, view extent cm, up)
views45 = (
    # join 1: the coplanar rotor-pad join, the one Esteban inspects
    ('l45_j4_front', J4_JOIN, D4(0.6, -1.0, 0.8), 40, 6.5, UP4),
    ('l45_j4_side', J4_JOIN, D4(0.0, -1.0, 0.3), 40, 6.5, UP4),
    ('l45_j4_plan', P4(14.0, CW, 4.0), D4(0.0, 0.0, 1.0), 60, 7.5, U4W),
    # the plan taper: the wall tangent to the rotor rim and the r 50 arc leading the other
    # wall off it - this is where the 10 mm lateral is taken
    ('l45_taper', J4_TAPER, D4(-0.2, 0.3, 1.0), 45, 6.5, U4W),
    # the climb: the side-plane S of four r 20 arcs
    ('l45_rise', RISE, D4(0.0, -1.0, 0.0), 70, 8.0, UP4),
    # join 2: the run landing on the disk rim, and the far-face corner round
    ('l45_j5_front', J5_LAND, D4(0.9, -0.7, 0.6), 45, 8.0, UP4),
    ('l45_j5_side', J5_LAND, D4(0.0, -1.0, 0.0), 45, 8.0, UP4),
    ('l45_j5_blend', J5_BLEND, D4(0.6, -0.9, 0.4), 40, 7.0, UP4),
    # the side elevation: the plane the route is drawn in (plate, S, run, slab)
    ('l45_side', WHOLE45, W4W, 110, 10.0, UP4),
    # the front elevation, looking back along the reach: the band 44 wide between the two
    # rims, w_off/2 off centre at each end - the offset, in one frame
    ('l45_front', P4(30.0, CW, ZRUN / 2.0), D4(-1.0, 0.0, 0.0), 80, 9.0, UP4),
    ('l45_plan', P4(34.0, CW, 4.0), D4(0.0, 0.0, 1.0), 90, 10.0, U4W),
    ('l45_iso', P4(30.0, CW, ZRUN / 2.0), D4(0.7, -0.8, 0.5), 100, 12.0, UP4),
    # the two links that share joint J4
    ('l34_l45', P4(6.0, CW, 6.0), D4(-0.8, -1.0, 0.4), 110, 16.0, UP4),
)
for tag, tgt_cm, d, dist, ext, up in views45:
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
    log('%s -> %s' % (tag, screenshot(OUT + 'generated_R_arm_exo114_%s.png' % tag)))

# the whole arm, fitted
cam = vp.camera
cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
cam.isFitView = True
vp.camera = cam
adsk.doEvents(); vp.fit(); adsk.doEvents()
log('arm_iso -> %s' % screenshot(OUT + 'generated_R_arm_exo114_arm_iso.png'))
