"""EXO-193 screenshots: every landing from the ACTUATOR side, with the XL330 proxy visible.

Same cameras as EXO-187, one opening narrower: the plate's opening is now the XL330's own
footprint (`ACT_OPEN_MARGIN` 0), so the proxy fills it edge to edge instead of floating in a
1 mm frame of air. The camera stands below module B's stator underside, looking up its axis
at the plate's far face, +Y (the connector side) up; then obliquely; then the whole arm.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo193.py --timeout 420
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
TAG = 'exo193'
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

# the whole arm, iso, proxies on
pa = [v * 1000.0 for v in frames['J1'][1]]
pz = [v * 1000.0 for v in frames['J7'][1]]
mid = [(pa[i] + pz[i]) / 2.0 for i in range(3)]
span = math.sqrt(sum((pz[i] - pa[i]) ** 2 for i in range(3))) + 120.0
shoot('arm_iso', mid, [1.0, -1.0, 0.7], 150, span / 10.0, [0.0, 0.0, 1.0])
log('done')
