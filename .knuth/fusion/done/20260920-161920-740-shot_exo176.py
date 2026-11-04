"""EXO-176 screenshots: link_67, the pair the generator used to refuse.

The landing plate's tongue is no longer pinned to the rotor pad axis: it is AIMED from the
disk centre at the point where the ribbon enters the plate plane (EXO-176), it ends ON the
ribbon rather than on a plane square to itself (EXO-177), and the plate is measured against
module A (EXO-178). So `link_67` routes - three views of it, then the wrist twice, then the
whole arm in iso.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo176.py --timeout 420
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
TAG = 'exo176'
SHOW = ('right arm', 'generated_R_arm', 'generated_R_arm_general')
GEN = 'generated_R_arm_general'
vp = app.activeViewport
root = design.rootComponent


def only(name):
    for i in range(root.occurrences.count):
        o = root.occurrences.item(i)
        o.isLightBulbOn = (o.component.name == name) if name else (
            o.component.name in SHOW)


only(None)
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
log('J7 spin %.3f deg, tongue %.3f deg, landing %s'
    % (spins['J7']['spin'], spins['J7'].get('tongue_deg', 0.0),
       spins['J7'].get('landing')))


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


# ---- link_67, three views: the pair, its angled landing, its rotor pad.
name, a, b, pad_r, width = LINKS[-1]
RA, tA = frames[a]
RB, tB = frames[b]
R_ab, t_ab = armgen.relative_frame(RA, tA, RB, tB)
route = sweepgen.sweep_route(R_ab, t_ab, pad_r, width)
log('%s: %s landing, tongue %+.3f deg, %.3f mm of centre line, web %.3f mm'
    % (name, route['landing'], route['tongue_deg'], sweepgen.path_length(route),
       route['cut_web']))
u, w = route['u'], route['w']
pa = [v * 1000.0 for v in tA]
pb = [v * 1000.0 for v in tB]
tgt = [(pa[i] + pb[i]) / 2.0 for i in range(3)]
span = math.sqrt(sum((pb[i] - pa[i]) ** 2 for i in range(3))) + 90.0


def W(s, ww, z):
    loc = [u[i] * s + w[i] * ww + (z if i == 2 else 0.0) for i in range(3)]
    return armgen.matvec(RA, loc)


def P(sv, wv, zv):
    loc = [u[i] * sv + w[i] * wv + (zv if i == 2 else 0.0) for i in range(3)]
    return [armgen.matvec(RA, loc)[i] + pa[i] for i in range(3)]


# only the general arm: the hand-made arm and `generated_R_arm` sit on top of this link
only(GEN)
up = W(0.0, 0.0, 1.0)
shoot('link_67_a', tgt, W(0.5, -1.0, 0.6), 80, span / 13.0, up)
shoot('link_67_b', tgt, W(0.0, 0.0, 1.0), 80, span / 13.0, W(1.0, 0.0, 0.0))
# the landing itself - the diagonal tongue on the J7 stator
sc = route['numbers']['slab_centre']
land = P(sc[0], sc[1], sc[2])
shoot('link_67_landing', land, W(1.0, -0.6, 0.35), 90, 8.5,
      W(0.0, 0.0, route['numbers']['y_sign']))

# ---- the wrist: J5 -> J6 -> J7, twice.
_tgt = [v * 1000.0 for v in frames['J6'][1]]
shoot('wrist_a', _tgt, [0.7, -1.0, 0.5], 90, 16.0, [0.0, 0.0, 1.0])
shoot('wrist_b', _tgt, [-0.7, 1.0, 0.5], 90, 16.0, [0.0, 0.0, 1.0])


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


# the general arm alone - all six links, J1 to J7. Its module bodies do not render in a
# viewport shot (EXO-154), so this is the links.
iso('arm_iso_general', 34.0, (9.0, -24.0, -12.0))
only(None)
iso('arm_iso', 52.0, (9.0, -24.0, -12.0))
log('done')
