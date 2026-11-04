"""EXO-201 screenshots: two actuators close up, with the proxy body gone.

EXO-187 built a proxy box on top of the real `XL330 v2/Body1` in every module; the two
coincident faces z-fought and every actuator showed grey patches. The proxy is gone
(EXO-201), so these close-ups must show one clean actuator back per module.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo201.py --timeout 420
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
TAG = 'exo201'
GEN = 'generated_R_arm_general'
CLOSE = ('J2', 'J4')          # the two actuators asked for
vp = app.activeViewport
root = design.rootComponent


def only(name):
    for i in range(root.occurrences.count):
        o = root.occurrences.item(i)
        o.isLightBulbOn = (o.component.name == name)


only(GEN)

# no XL330_* component may survive the rebuild
stale = [c.name for c in design.allComponents if c.name.startswith('XL330_')]
log('stale XL330_* proxy components in the document: %d %s' % (len(stale), stale or ''))

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
log('frames rebuilt; modules ' + ', '.join(ORDER))


def _save(path, name):
    h = int(round(1600.0 * vp.height / vp.width))
    ok = vp.saveAsImageFile(path, 1600, h)
    log('%s -> %s (1600x%d, %s)' % (name, path, h, ok))


def shoot(name, tgt_mm, d_world, dist_cm, ext_cm, up):
    n = math.sqrt(sum(v * v for v in d_world))
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
    _save(OUT + 'generated_R_arm_%s_%s.png' % (TAG, name), name)


for lab in CLOSE:
    R, t = frames[lab]
    xb = [R[i][0] for i in range(3)]
    yb = [R[i][1] for i in range(3)]
    zb = [R[i][2] for i in range(3)]
    p = [v * 1000.0 for v in t]
    zc = (armgen.ACT_Z[0] + armgen.STATOR_UNDERSIDE_Z) / 2.0
    tgt = [p[i] + zb[i] * zc for i in range(3)]
    # straight at the actuator's back, up its own axis from below, connector side +Y up
    shoot('%s_actuator_axis' % lab, tgt, [-v for v in zb], 40, 2.6, yb)
    # obliquely, so the four side faces of the XL330 read as single faces, not two
    d = [-0.7 * zb[i] - 0.65 * xb[i] + 0.3 * yb[i] for i in range(3)]
    shoot('%s_actuator_oblique' % lab, tgt, d, 40, 3.2, yb)

pa = [v * 1000.0 for v in frames['J1'][1]]
pz = [v * 1000.0 for v in frames['J7'][1]]
mid = [(pa[i] + pz[i]) / 2.0 for i in range(3)]
span = math.sqrt(sum((pz[i] - pa[i]) ** 2 for i in range(3))) + 120.0
shoot('arm_iso', mid, [1.0, -1.0, 0.7], 150, span / 10.0, [0.0, 0.0, 1.0])
log('EXO-201 screenshots done')
