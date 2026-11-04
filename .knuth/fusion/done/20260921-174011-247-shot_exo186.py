"""EXO-186 screenshots: the AXIS-ALIGNED spin, and the plate grown to cover the opening.

The landing shots are EXO-193's cameras (up module B's axis from the actuator side, and
obliquely). Added for EXO-186: two wrist views with J7's encoder / connector block in frame,
so that the module reads square to the link rather than turned along the 67 deg tongue, and
two more of link_67 alone with its r 29.46 landing plate. Then the whole arm.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo186.py --timeout 420
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
TAG = 'exo186'
GEN = 'generated_R_arm_general'
vp = app.activeViewport
root = design.rootComponent


def only(name):
    for i in range(root.occurrences.count):
        o = root.occurrences.item(i)
        o.isLightBulbOn = (o.component.name == name)


only(GEN)
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
log('frames rebuilt; slides ' + ', '.join('%s %+0.3f' % (k, SLIDES[k]) for k in ORDER))


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


for name, a, b, pad_r, width in LINKS:
    RB, tB = frames[b]
    xb = [RB[i][0] for i in range(3)]
    yb = [RB[i][1] for i in range(3)]
    zb = [RB[i][2] for i in range(3)]
    pb = [v * 1000.0 for v in tB]
    # the middle of the actuator's exposed back, in module B's frame -> world
    zc = (armgen.ACT_Z[0] + armgen.STATOR_UNDERSIDE_Z) / 2.0
    tgt = [pb[i] + zb[i] * zc for i in range(3)]
    # straight up the axis from below (the actuator side), connector side +Y up
    shoot('%s_landing_%s_axis' % (name, b), tgt, [-v for v in zb], 60, 7.5, yb)
    # and obliquely, from -X_B and below, so the proxy's height above the plate reads
    d = [-0.75 * zb[i] - 0.6 * xb[i] + 0.25 * yb[i] for i in range(3)]
    shoot('%s_landing_%s_oblique' % (name, b), tgt, d, 60, 9.0, yb)

# EXO-186: the wrist, twice, with J7's encoder / connector block in frame. The camera stands
# off J7's OWN +Y - the encoder side - so where the block points reads directly; the second
# shot is from J7's +X, square to the first, so the same block is seen edge on.
_R7, _t7 = frames['J7']
_p7 = [v * 1000.0 for v in _t7]
_R6, _t6 = frames['J6']
_p6 = [v * 1000.0 for v in _t6]
_mid67 = [(_p6[i] + _p7[i]) / 2.0 for i in range(3)]
_x7 = [_R7[i][0] for i in range(3)]
_y7 = [_R7[i][1] for i in range(3)]
_z7 = [_R7[i][2] for i in range(3)]
shoot('wrist_J7_encoder', _mid67, [0.7 * _y7[i] + 0.5 * _z7[i] + 0.3 * _x7[i] for i in range(3)],
      120, 13.0, _z7)
shoot('wrist_J7_encoder_side', _mid67, [0.9 * _x7[i] + 0.3 * _z7[i] for i in range(3)],
      120, 13.0, _z7)

# link_67 alone, twice: the grown landing plate from above and the whole link in plan
only(GEN)
shoot('link_67_plate', [_p7[i] - _z7[i] * 4.0 for i in range(3)], _z7, 80, 9.0, _y7)
shoot('link_67_plan', _mid67, [_x7[i] for i in range(3)], 120, 12.0, _z7)

# the whole arm, iso, proxies on
pa = [v * 1000.0 for v in frames['J1'][1]]
pz = [v * 1000.0 for v in frames['J7'][1]]
mid = [(pa[i] + pz[i]) / 2.0 for i in range(3)]
span = math.sqrt(sum((pz[i] - pa[i]) ** 2 for i in range(3))) + 120.0
shoot('arm_iso', mid, [1.0, -1.0, 0.7], 150, span / 10.0, [0.0, 0.0, 1.0])
log('done')
