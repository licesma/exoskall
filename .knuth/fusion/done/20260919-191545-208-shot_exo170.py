"""EXO-170 screenshots: the wrist with module J6 slid along its own axis.

`slide` is solved from the module bodies (EXO-170): J6 moves 21.380 mm along its own Z and
nothing else moves. `link_56` pays it one for one as extra run and is built - two views,
plus its landing and its rotor pad. `link_67` is STILL refused, so there is no link_67 to
photograph; the number is in the build report. Then the whole arm in iso.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo170.py --timeout 420
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
TAG = 'exo170'
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
ORDER = ([j['label'] for j in BuildArm.JOINTS]
         + [j['label'] for j in BuildArmGen.NEW_JOINTS])
_s, _l, table = BuildArmGen.seed_frames(joints, by_child, shift)
seed, _combo, SLIDES, _rows = BuildArmGen.solve_axis_signs(joints, by_child, shift, LINKS,
                                                           table, log, ORDER)
log('slides: ' + ', '.join('%s %+0.3f' % (k, SLIDES[k]) for k in ORDER))
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
        # The refused pair has no link to photograph. Its own picture would be the two
        # modules overlapping, and the module bodies under `generated_R_arm_general` do not
        # render in a viewport shot (EXO-154), so the number is the evidence, not an image.
        log('%s REFUSED, nothing to shoot: %s' % (name, e))
        continue
    u, w = route['u'], route['w']
    # only the general arm: the hand-made arm and `generated_R_arm` sit on top of this link
    # and hide it (its modules do not render either way, EXO-154).
    for i in range(root.occurrences.count):
        o = root.occurrences.item(i)
        o.isLightBulbOn = o.component.name == 'generated_R_arm_general'

    def W(s, ww, z, u=u, w=w, RA=RA):
        loc = [u[i] * s + w[i] * ww + (z if i == 2 else 0.0) for i in range(3)]
        return armgen.matvec(RA, loc)

    up = W(0.0, 0.0, 1.0)
    shoot('%s_a' % name, tgt, W(0.5, -1.0, 0.6), 80, span / 14.0, up, shift=0.0)
    shoot('%s_b' % name, tgt, W(0.0, 0.0, 1.0), 80, span / 14.0, W(1.0, 0.0, 0.0),
          shift=0.0)
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

# ---- the wrist itself: J5 -> J6 -> J7, where the slide is spent. Two views.
_pa = [v * 1000.0 for v in frames['J5'][1]]
_pc = [v * 1000.0 for v in frames['J7'][1]]
_tgt = [(_pa[i] + _pc[i]) / 2.0 for i in range(3)]
_span = math.sqrt(sum((_pc[i] - _pa[i]) ** 2 for i in range(3))) + 150.0
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    o.isLightBulbOn = o.component.name == 'generated_R_arm_general'
shoot('wrist_a', _tgt, [0.6, -1.0, 0.45], 90, _span / 9.0, [0.0, 0.0, 1.0], shift=0.0)
shoot('wrist_b', _tgt, [0.0, 0.0, 1.0], 90, _span / 9.0, [1.0, 0.0, 0.0], shift=0.0)
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    o.isLightBulbOn = o.component.name in SHOW


def iso(name, ext, tgt):
    cam = vp.camera
    cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
    cam.isFitView = False
    cam.target = adsk.core.Point3D.create(*tgt)
    cam.eye = adsk.core.Point3D.create(tgt[0] + 90, tgt[1] - 90, tgt[2] + 60)
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.viewExtents = ext
    vp.camera = cam
    adsk.doEvents()
    vp.camera = cam
    adsk.doEvents()
    _save(OUT + 'generated_R_arm_%s_%s.png' % (TAG, name), name)


# all three arms, for the overlay
iso('arm_iso', 52.0, (9.0, -24.0, -12.0))
# the general arm alone - the five links it builds, J1 to J6. Its module bodies do not
# render in a viewport shot (EXO-154), so this is the links.
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    o.isLightBulbOn = o.component.name == 'generated_R_arm_general'
iso('arm_iso_general', 34.0, (9.0, -24.0, -12.0))
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    o.isLightBulbOn = o.component.name in SHOW
log('done')
