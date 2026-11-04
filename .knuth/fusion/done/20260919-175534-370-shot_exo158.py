"""EXO-158 screenshots: the two links past J5, and why only one of them exists.

`link_56` (J5 -> J6, wrist roll -> wrist pitch) is built: two views, plus its landing and
its rotor pad. `link_67` is REFUSED, so the third subject is the pair itself - modules J6
and J7 at the URDF spacing, where the two r 27 stators overlap by 8.01 mm. Then the whole
general arm in iso, all seven modules.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo158.py --timeout 420
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
TAG = 'exo158'
SHOW = ('right arm', 'generated_R_arm', 'generated_R_arm_general')
vp = app.activeViewport
root = design.rootComponent
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    o.isLightBulbOn = o.component.name in SHOW

joints, by_child = armgen.parse_urdf(BuildArm.URDF_PATH)
shift, _ = armgen.joint_world_pose(BuildArm.ORIGIN_AT_JOINT, joints, by_child)
LINKS = [(L['name'], L['a'], L['b'], L['pad_r'], L['width'])
         for L in list(BuildArm.LINKS) + BuildArmGen.NEW_LINKS]
_s, _l, table = BuildArmGen.seed_frames(joints, by_child, shift)
seed = BuildArmGen.solve_axis_signs(joints, by_child, shift, LINKS, table, log)
frames, spins = sweepgen.solve_spins(seed, LINKS)


def _save(path, name):
    h = int(round(1600.0 * vp.height / vp.width))
    ok = vp.saveAsImageFile(path, 1600, h)
    log('%s -> %s (1600x%d, %s)' % (name, path, h, ok))


SHIFT = 0.25


def shoot(name, tgt_mm, d_world, dist_cm, ext_cm, up, shift=None):
    n = math.sqrt(sum(v * v for v in d_world))
    fwd = [-v for v in d_world]
    right = armgen.unit(armgen.cross(fwd, up))
    wide = ext_cm * 9.5
    sh = SHIFT if shift is None else shift
    tgt_mm = [tgt_mm[i] + right[i] * sh * wide for i in range(3)]
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


for name, a, b, pad_r, width in LINKS[-2:]:
    RA, tA = frames[a]
    RB, tB = frames[b]
    R_ab, t_ab = armgen.relative_frame(RA, tA, RB, tB)
    pa = [v * 1000.0 for v in tA]
    pb = [v * 1000.0 for v in tB]
    tgt = [(pa[i] + pb[i]) / 2.0 for i in range(3)]
    span = math.sqrt(sum((pb[i] - pa[i]) ** 2 for i in range(3))) + 90.0
    try:
        route = sweepgen.sweep_route(R_ab, t_ab, pad_r, width)
    except ValueError as e:
        # the refused pair: shoot the two modules that will not fit, from two sides, with
        # only the general arm visible so nothing sits in front of the overlap.
        log('%s REFUSED: %s' % (name, e))
        for i in range(root.occurrences.count):
            o = root.occurrences.item(i)
            o.isLightBulbOn = o.component.name == 'generated_R_arm_general'
        zb = [R_ab[i][2] for i in range(3)]
        za = [RA[i][2] for i in range(3)]
        d = armgen.unit([pb[i] - pa[i] for i in range(3)])
        side = armgen.unit(armgen.cross(d, armgen.matvec(RA, [0, 0, 1])))
        shoot('%s_pair_a' % name, tgt, armgen.matvec(RA, [0, 0, 1]), 60, span / 11.0, d)
        shoot('%s_pair_b' % name, tgt, side, 60, span / 11.0,
              armgen.matvec(RA, [0, 0, 1]))
        for i in range(root.occurrences.count):
            o = root.occurrences.item(i)
            o.isLightBulbOn = o.component.name in SHOW
        continue
    u, w = route['u'], route['w']

    def W(s, ww, z, u=u, w=w, RA=RA):
        loc = [u[i] * s + w[i] * ww + (z if i == 2 else 0.0) for i in range(3)]
        return armgen.matvec(RA, loc)

    up = W(0.0, 0.0, 1.0)
    shoot('%s_a' % name, tgt, W(0.5, -1.0, 0.6), 80, span / 9.0, up)
    shoot('%s_b' % name, tgt, W(0.0, 0.0, 1.0), 80, span / 9.0, W(1.0, 0.0, 0.0))
    for i in range(root.occurrences.count):
        o = root.occurrences.item(i)
        o.isLightBulbOn = o.component.name == 'generated_R_arm_general'
    sc = route['numbers']['slab_centre']

    def P(sv, wv, zv, u=u, w=w, RA=RA, pa=pa):
        loc = [u[i] * sv + w[i] * wv + (zv if i == 2 else 0.0) for i in range(3)]
        return [armgen.matvec(RA, loc)[i] + pa[i] for i in range(3)]

    land = P(sc[0], sc[1], route['z_arr'])
    shoot('%s_landing' % name, [(land[i] + pb[i]) / 2.0 for i in range(3)],
          W(1.0, -0.6, 0.35), 90, 9.0, W(0.0, 0.0, route['numbers']['y_sign']), shift=0.0)
    shoot('%s_pad' % name, P(0.0, 0.0, 4.0), W(-0.4, -0.9, 0.6), 90, 5.5,
          W(0.0, 0.0, 1.0), shift=0.0)
    for i in range(root.occurrences.count):
        o = root.occurrences.item(i)
        o.isLightBulbOn = o.component.name in SHOW

cam = vp.camera
cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
cam.isFitView = False
cam.target = adsk.core.Point3D.create(9.0, -24.0, -12.0)
cam.eye = adsk.core.Point3D.create(9.0 + 90, -24.0 - 90, -12.0 + 60)
cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
cam.viewExtents = 52.0
vp.camera = cam
adsk.doEvents()
_save(OUT + 'generated_R_arm_%s_arm_iso.png' % TAG, 'arm_iso')
log('done')
