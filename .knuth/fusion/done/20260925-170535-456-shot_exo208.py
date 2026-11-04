"""EXO-208 screenshots, TAG exo208: every limited joint at its URDF `lower` and `upper`,
close up on the two stop blocks where they meet, plus the arm.

Both blocks live in the 3.6 mm slot between the stator's top face (z = -4) and the link's
rotor pad (z = 0), inside r 27 - invisible from either end of the axis. The one direction
that sees into the slot is RADIALLY IN at the joint stop's own angle (168 deg), a hair below
the pad plane: that is where the two blocks meet at both limits.

Everything distal to the joint is carried round with it, so each frame reads as a pose of
the arm and not as a broken link.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo208.py --timeout 900
"""
import math
import sys
import importlib

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
import sweepgen
import BuildArm
import BuildArmGen
importlib.reload(armgen)
importlib.reload(sweepgen)
importlib.reload(BuildArm)
importlib.reload(BuildArmGen)

OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/'
TAG = 'exo208'
GEN = 'generated_R_arm_general'

vp = app.activeViewport
root = design.rootComponent
arm = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    o.isLightBulbOn = (o.component.name == GEN)
    if o.component.name == GEN:
        arm = o
if arm is None:
    raise RuntimeError('%s is not in this document' % GEN)


def bulbs_on(occ):
    n = 0
    for i in range(occ.childOccurrences.count):
        c = occ.childOccurrences.item(i)
        if not c.isLightBulbOn:
            c.isLightBulbOn = True
            n += 1
        n += bulbs_on(c)
    return n


log('switched %d occurrence light bulb(s) on under %s' % (bulbs_on(arm), GEN))

joints, by_child = armgen.parse_urdf(BuildArm.URDF_PATH)
shift, _ = armgen.joint_world_pose(BuildArm.ORIGIN_AT_JOINT, joints, by_child)
LINKS = [(L['name'], L['a'], L['b'], L['pad_r'], L['width'])
         for L in list(BuildArm.LINKS) + BuildArmGen.NEW_LINKS]
ORDER = ([j['label'] for j in BuildArm.JOINTS]
         + [j['label'] for j in BuildArmGen.NEW_JOINTS])
_s, _l, table = BuildArmGen.seed_frames(joints, by_child, shift)
seed, _combo, SLIDES, _rows = BuildArmGen.solve_axis_signs(joints, by_child, shift, LINKS,
                                                           table, lambda s: None, ORDER)
frames, spins = sweepgen.solve_spins(seed, LINKS)
JOINTS_ALL = list(BuildArm.JOINTS) + [dict(j, y_hint=table[j['label']]['y_hint'],
                                           axis_sign=1) for j in BuildArmGen.NEW_JOINTS]
for j in JOINTS_ALL[len(BuildArm.JOINTS):]:
    _t, ax = armgen.joint_world_pose(j['name'], joints, by_child)
    z = [seed[j['label']][0][i][2] for i in range(3)]
    j['axis_sign'] = 1 if armgen.dot(z, ax) > 0 else -1
by_label = {j['label']: j for j in JOINTS_ALL}

kids = {}
for i in range(arm.childOccurrences.count):
    c = arm.childOccurrences.item(i)
    kids[c.component.name] = c


def wrapper_of(lab):
    return [k for k in kids if k.startswith('G%s_' % lab)][0]


# the kinematic chain, proximal first: everything AFTER a joint's own module is carried by it
CHAIN = []
for lab, (name, a, b, pad_r, width) in zip(ORDER, LINKS + [(None,) * 5]):
    CHAIN.append(wrapper_of(lab))
    if name is not None:
        CHAIN.append('g_' + name)
CHAIN = [n for n in CHAIN if n in kids]
log('chain: %s' % ' -> '.join(CHAIN))


def pose(lab, q):
    """Carry everything distal to joint `lab` round by the URDF angle q."""
    jd = by_label[lab]
    R1, t1 = frames[lab]
    axis = adsk.core.Vector3D.create(*[R1[i][2] for i in range(3)])
    orig = adsk.core.Point3D.create(*[v * 100.0 for v in t1])
    m = adsk.core.Matrix3D.create()
    m.setToRotation(jd['axis_sign'] * q, axis, orig)
    k = CHAIN.index(wrapper_of(lab))
    for nm in CHAIN[k + 1:]:
        BuildArm.set_occ_matrix(kids[nm], m, arm)
    adsk.doEvents()


def rest():
    ident = adsk.core.Matrix3D.create()
    for nm in CHAIN:
        BuildArm.set_occ_matrix(kids[nm], ident, arm)
    adsk.doEvents()


def save(name):
    h = int(round(1600.0 * vp.height / vp.width))
    path = OUT + 'generated_R_arm_%s_%s.png' % (TAG, name)
    ok = vp.saveAsImageFile(path, 1600, h)
    log('  %-22s -> %s (%s)' % (name, path, ok))


def shoot(name, tgt_mm, d_world, dist_cm, ext_cm, up):
    n = math.sqrt(sum(v * v for v in d_world)) or 1.0
    t = adsk.core.Point3D.create(tgt_mm[0] / 10.0, tgt_mm[1] / 10.0, tgt_mm[2] / 10.0)
    cam = vp.camera
    cam.isFitView = False
    cam.target = t
    cam.eye = adsk.core.Point3D.create(t.x + dist_cm * d_world[0] / n,
                                       t.y + dist_cm * d_world[1] / n,
                                       t.z + dist_cm * d_world[2] / n)
    cam.upVector = adsk.core.Vector3D.create(*up)
    cam.viewExtents = ext_cm
    cam.isSmoothTransition = False
    vp.camera = cam
    adsk.doEvents()
    vp.camera = cam
    adsk.doEvents()
    save(name)


CA = armgen.JOINT_STOP_MID            # 168.00225 deg - where the two blocks meet
for name, a, b, pad_r, width in LINKS:
    lab = a
    if 'g_' + name not in kids:
        log('%s: g_%s is not in the arm (refused) - no stop block to photograph' % (lab, name))
        continue
    jd = by_label[lab]
    lj = armgen.limited_joint(joints[jd['name']], jd['axis_sign'], pad_r)
    R1, t1 = frames[lab]
    xb = [R1[i][0] for i in range(3)]
    yb = [R1[i][1] for i in range(3)]
    zb = [R1[i][2] for i in range(3)]
    p1 = [v * 1000.0 for v in t1]
    rad = [math.cos(CA) * xb[i] + math.sin(CA) * yb[i] for i in range(3)]
    # The slot the two blocks meet in is only 3.6 mm tall and 6 mm deep, so the camera sits
    # OUTSIDE the rim at the joint stop's own angle and looks radially in, tilted down just
    # enough to see over the stator's top face. 45 mm of view - the module is 54 mm across -
    # so the link stop's two edges and the joint stop are in the same picture.
    tgt = [p1[i] + rad[i] * 18.0 + zb[i] * (-2.0) for i in range(3)]
    d = [rad[i] - 0.30 * zb[i] for i in range(3)]
    log('%s %s: link stop %.5f deg wide at %.5f; joint stop %.1f deg at %.5f'
        % (lab, jd['name'], math.degrees(lj['arc']), lj['stop_angle'],
           armgen.JOINT_STOP_DEG, math.degrees(CA)))
    for what, q in (('lower', lj['lower']), ('upper', lj['upper'])):
        rest()
        pose(lab, q)
        shoot('%s_%s_stops' % (lab, what), tgt, d, 25, 4.5, zb)

rest()
p1 = [v * 1000.0 for v in frames['J1'][1]]
p7 = [v * 1000.0 for v in frames['J7'][1]]
mid = [(p1[i] + p7[i]) / 2.0 for i in range(3)]
span = math.sqrt(sum((p7[i] - p1[i]) ** 2 for i in range(3))) + 200.0
shoot('arm_iso', mid, [1.0, -1.0, 0.6], 300, span / 10.0 * 0.95, [0, 0, 1])
shoot('arm_front', mid, [1.0, 0.0, 0.0], 300, span / 10.0 * 0.95, [0, 0, 1])
log('EXO-208 screenshots done; every occurrence restored to the zero pose')
