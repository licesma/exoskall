# EXO-81 screenshots: the EXO-69/EXO-74 camera set aimed at link_23 (so exo83 and exo81 are
# comparable frame by frame) plus new cameras on link_12, which is now a routed profile L
# instead of a box bracket:
#   j1_front / j1_side  the plate leaving the J1 rotor pad - the join EXO-74 is about
#   j1_elbow            the elbow from below: the r 5.5 outer round and the r 3 inner fillet
#   j1_plan             straight down the J1 pad axis: the 25 mm band and the offset tongue
#   j1_slab             the J2 stator slab, its tongue and the two blends
#   j1_iso              link_12 on its own, three-quarter
#   j12_j23             both links together, the corridor EXO-3 is about
# Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo81.py --timeout 300
# Touches only the camera.
import math
import sys
import importlib

TAG = 'exo81'
exec(open('/Users/esteban/repos/exo/tools/bridge_scripts/shot_exo58.py').read())
exec(open('/Users/esteban/repos/exo/tools/bridge_scripts/shot_exo74_rim.py').read())

OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/'
vp = app.activeViewport

# link_12 targets. Rather than re-deriving the J1 frame by hand, take it from the document:
# the J1 wrapper occurrence's transform maps the J1 module frame to world, and armgen.route
# gives the link frame (u along the plate, w across it) inside it. P(s, w, z) is then the
# world point (cm) of a link-frame station.
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
importlib.reload(armgen)
_joints, _by_child = armgen.parse_urdf('/Users/esteban/repos/exo/g1/g1_29dof.urdf')
_shift, _ = armgen.joint_world_pose('left_shoulder_pitch_joint', _joints, _by_child)
_FR = {}
for _lab, (_nm, _sgn, _yh) in {'J1': ('right_shoulder_pitch_joint', -1, (0, 0, 1)),
                               'J2': ('right_shoulder_roll_joint', +1, (0, 1, 0))}.items():
    _R, _t = armgen.module_frame(_nm, _joints, _by_child, _sgn, _yh)
    _FR[_lab] = (_R, [_t[k] - _shift[k] for k in range(3)])
_LG = armgen.route(_FR['J1'], _FR['J2'], 26.0)['geometry']
U, W = _LG['u'], _LG['w']

_arm = None
for _i in range(design.rootComponent.occurrences.count):
    _o = design.rootComponent.occurrences.item(_i)
    if _o.component.name.startswith('generated_R_arm'):
        _arm = _o
_j1 = None
for _i in range(_arm.childOccurrences.count):
    _o = _arm.childOccurrences.item(_i)
    if _o.component.name.startswith('J1_'):
        _j1 = _o
_T = _j1.transform


def P(s, w, z):
    """World point (cm) of the link-frame station (s, w, z) in mm."""
    q = adsk.core.Point3D.create((U[0] * s + W[0] * w) / 10.0,
                                 (U[1] * s + W[1] * w) / 10.0, z / 10.0)
    q.transformBy(_T)
    return (q.x, q.y, q.z)


def D(s, w, z):
    """World direction of a link-frame direction."""
    v = adsk.core.Vector3D.create(U[0] * s + W[0] * w, U[1] * s + W[1] * w, z)
    v.transformBy(_T)
    return (v.x, v.y, v.z)


J1_JOIN = P(26.0, 0.0, 4.0)      # the plate crossing the pad rim
J1_ELBOW = P(29.0, 0.0, 4.0)     # the elbow's two radii
J1_SLAB = P(28.5, 0.0, 22.0)     # the stator slab and the tongue
J1_MID = P(20.0, 0.0, 12.0)      # the whole link
Z_A = D(0.0, 0.0, 1.0)           # the J1 pad axis, for the plan view
U_W = D(1.0, 0.0, 0.0)           # the plate direction, the plan view's 'up'
UP = (0, 0, 1)
views = (('j1_front', J1_JOIN, (-1.0, -0.9, 0.6), 40, 12.0, UP),
         ('j1_side', J1_JOIN, (-0.2, -1.0, -0.4), 40, 12.0, UP),
         ('j1_elbow', J1_ELBOW, (-0.9, -1.0, -0.8), 30, 8.0, UP),
         ('j1_slab', J1_SLAB, (-1.0, -0.6, 0.3), 40, 10.0, UP),
         ('j1_plan', J1_MID, Z_A, 40, 9.0, U_W),
         ('j1_iso', J1_MID, (-1.0, -1.0, 0.7), 40, 12.0, UP),
         ('j12_j23', P(14.0, 7.0, -20.0), (-1.0, -1.0, 0.5), 70, 22.0, UP))
for tag, tgt_cm, d, dist, ext, up in views:
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
    log('%s -> %s' % (tag, screenshot(OUT + 'generated_R_arm_%s_%s.png' % (TAG, tag))))
