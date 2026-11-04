# EXO-124 screenshots: the J4 join of link_45, where the ramp's lift-off reached the
# rotor-pad screw hole nearest the climb. Four cameras, all on FIXED stations of the J4
# rotor-pad frame (pad axis, hole radius, rim) and NOT on any number the route computes, so
# the before and after frames are the same camera and can be read side by side.
#
# The phase is taken from the BODY, not from a flag: the s at which the plate's outer face
# (z = PAD_T) stops being flat is measured on the built link, and the shot is tagged
# 'before' while that is still inside the hole edge (ROTOR_HOLE_RADIUS + HOLE_R) and 'after'
# once it clears it by HOLE_FLAT_RING. Nothing but the camera is touched.
#
# Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo124.py --timeout 300
import math
import sys
import importlib

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
importlib.reload(armgen)

OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/'
vp = app.activeViewport
root = design.rootComponent

arm = next(o for o in root.occurrences if o.component.name.startswith('generated_R_arm'))
l45 = next(o for o in arm.childOccurrences if o.component.name.startswith('link_45'))
j4 = next(o for o in arm.childOccurrences if o.component.name.startswith('J4_'))
T4 = j4.transform
body = l45.bRepBodies.item(0)

# ---- the link frame, from armgen (u, w only - the cameras use fixed stations on it) ----
_j, _by = armgen.parse_urdf('/Users/esteban/repos/exo/g1/g1_29dof.urdf')
_sh, _ = armgen.joint_world_pose('left_shoulder_pitch_joint', _j, _by)
_FR = {}
for _lab, (_nm, _sgn, _yh) in {'J4': ('right_elbow_joint', -1, (-1, 0, 0)),
                               'J5': ('right_wrist_roll_joint', +1, (0, 1, 0))}.items():
    _R, _t = armgen.module_frame(_nm, _j, _by, _sgn, _yh)
    _FR[_lab] = (_R, [_t[k] - _sh[k] for k in range(3)])
GEO = armgen.route(_FR['J4'], _FR['J5'], 27.0, width=44.0)['geometry']
U, W = GEO['u'], GEO['w']


def P(s, w, z):
    q = adsk.core.Point3D.create((U[0] * s + W[0] * w) / 10.0,
                                 (U[1] * s + W[1] * w) / 10.0, z / 10.0)
    q.transformBy(T4)
    return q


def D(s, w, z):
    v = adsk.core.Vector3D.create(U[0] * s + W[0] * w, U[1] * s + W[1] * w, z)
    v.transformBy(T4)
    return (v.x, v.y, v.z)


# ---- which phase this is, measured on the body: how far out the pad's OUTER face is flat ----
OW = T4.translation
UW3 = adsk.core.Vector3D.create(U[0], U[1], 0.0); UW3.transformBy(T4)
WW3 = adsk.core.Vector3D.create(W[0], W[1], 0.0); WW3.transformBy(T4)
ZW3 = adsk.core.Vector3D.create(0.0, 0.0, 1.0); ZW3.transformBy(T4)


def local(p):
    """(s, w, z) in mm of a world point (cm), in the J4 rotor-pad frame."""
    d = adsk.core.Vector3D.create(p.x - OW.x, p.y - OW.y, p.z - OW.z)
    return (10.0 * d.dotProduct(UW3), 10.0 * d.dotProduct(WW3), 10.0 * d.dotProduct(ZW3))


reach = 0.0
for f in body.faces:
    g = f.geometry
    if g.objectType != 'adsk::core::Plane':
        continue
    if abs(g.normal.dotProduct(ZW3)) < 0.999 or abs(local(g.origin)[2] - armgen.PAD_T) > 0.02:
        continue
    bb = f.boundingBox
    reach = max(reach, local(bb.minPoint)[0], local(bb.maxPoint)[0])
edge = armgen.ROTOR_HOLE_RADIUS + armgen.HOLE_R
phase = 'after' if reach >= edge + armgen.HOLE_FLAT_RING - 1e-6 else 'before'
log('the pad outer face (z = %.1f) is flat out to s = %.3f; the outermost hole edge is at '
    '%.3f -> this is the %r set' % (armgen.PAD_T, reach, edge, phase))

UP = D(0.0, 0.0, 1.0)
UW = D(1.0, 0.0, 0.0)
HOLE = P(armgen.ROTOR_HOLE_RADIUS, 0.0, armgen.PAD_T)     # the hole nearest the climb
PAD = P(0.0, 0.0, armgen.PAD_T)
views = (
    # straight down the pad axis: the four M3 holes, the rim, and the line where the plate
    # stops being flat - the whole of EXO-124 in one frame
    ('j4_holes_plan', PAD, D(0.0, 0.0, 1.0), 60, 4.2, UW),
    # the same join in 3/4, where a screw head's seat is readable
    ('j4_holes_iso', HOLE, D(-0.5, 0.45, 1.0), 45, 3.2, UW),
    # tight on the hole the climb reached
    ('j4_seat', HOLE, D(-0.25, 0.2, 1.0), 30, 1.6, UW),
    # the side elevation of the join: plate, lift-off, the start of the climb over the rim
    ('j4_side', P(16.0, -5.0, 8.0), D(0.0, -1.0, 0.0), 60, 5.5, UP),
)
for tag, tgt, d, dist, ext, up in views:
    n = math.sqrt(sum(v * v for v in d))
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
    log('%s -> %s' % (tag, screenshot(OUT + 'generated_R_arm_exo124_%s_%s.png'
                                      % (phase, tag))))
