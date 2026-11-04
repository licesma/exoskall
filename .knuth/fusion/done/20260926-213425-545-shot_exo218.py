"""EXO-218 screenshots, TAG exo218.

Four frames: the Gento Luna arm as a whole (iso), its SHOULDER - R1/R2 coincident in the
URDF, opened by the slide - and its WRIST - R5/R6/R7 coincident, opened by two more slides;
and the G1 arm after the link stop went back to ONE block, so the revert has a picture too.

Nothing is posed: every occurrence is left where the builder put it.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo218.py --timeout 1200
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
TAG = 'exo218'

vp = app.activeViewport
root = design.rootComponent


def arm_occ(name):
    for i in range(root.occurrences.count):
        o = root.occurrences.item(i)
        if o.component.name == name:
            return o
    return None


def only_show(name):
    """Just this arm visible, every occurrence under it switched on."""
    for i in range(root.occurrences.count):
        o = root.occurrences.item(i)
        o.isLightBulbOn = (o.component.name == name)

    def bulbs(occ):
        n = 0
        for j in range(occ.childOccurrences.count):
            c = occ.childOccurrences.item(j)
            if not c.isLightBulbOn:
                c.isLightBulbOn = True
                n += 1
            n += bulbs(c)
        return n
    occ = arm_occ(name)
    if occ is None:
        return None
    log('%s: %d light bulb(s) switched on' % (name, bulbs(occ)))
    return occ


def frames_of(spec):
    """The module frames the builder placed, in mm off the document origin."""
    joints, by_child = armgen.parse_urdf(spec['urdf'])
    origin, _a = armgen.joint_world_pose(spec['origin_at'], joints, by_child)
    off = spec.get('offset', (0.0, 0.0, 0.0))
    shift = [origin[i] - off[i] for i in range(3)]
    links = sweepgen.spec_links(spec)
    order = [j['label'] for j in sweepgen.spec_joints(spec)]
    _s, _l, table = sweepgen.seed_frames(spec, joints, by_child, shift)
    seed, _combo, slides, _rows = sweepgen.solve_axis_signs(
        spec, joints, by_child, shift, links, table, lambda s: None, order)
    allj = list(spec['fixed']) + [dict(j, y_hint=table[j['label']]['y_hint'], axis_sign=1)
                                  for j in spec['derived']]
    for j in allj[len(spec['fixed']):]:
        _t, ax = armgen.joint_world_pose(j['name'], joints, by_child)
        z = [seed[j['label']][0][i][2] for i in range(3)]
        j['axis_sign'] = 1 if armgen.dot(z, ax) > 0 else -1
    stops = sweepgen.joint_stops(spec, joints, links,
                                 dict((j['label'], j['axis_sign']) for j in allj))
    frames, _spins = sweepgen.solve_spins(seed, links, stops=stops)
    pts = dict((lab, [v * 1000.0 for v in frames[lab][1]]) for lab in order)
    log('%s slides: %s' % (spec['key'],
                           ', '.join('%s %+.3f mm' % (l, slides.get(l, 0.0))
                                     for l in order)))
    return frames, pts, order


def save(name):
    h = int(round(1600.0 * vp.height / vp.width))
    path = OUT + '%s_%s.png' % (TAG, name)
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


def span(p, q, pad):
    return math.sqrt(sum((q[i] - p[i]) ** 2 for i in range(3))) + pad


def mid(p, q):
    return [(p[i] + q[i]) / 2.0 for i in range(3)]


# ---- Gento Luna ----
if only_show(BuildArmGen.LUNA_ARM_NAME) is None:
    log('%s is not in this document - build it first' % BuildArmGen.LUNA_ARM_NAME)
else:
    _f, p, order = frames_of(BuildArmGen.LUNA_SPEC)
    a, z = p[order[0]], p[order[-1]]
    shoot('luna_arm_iso', mid(a, z), [1.0, -1.0, 0.6], 400, span(a, z, 260) / 10.0 * 0.95,
          [0, 0, 1])
    # the shoulder: R1/R2 share an origin in the URDF and R3 is 264 mm down the arm
    shoot('luna_shoulder', mid(p['R1'], p['R3']), [1.0, -0.8, 0.45], 200,
          span(p['R1'], p['R3'], 150) / 10.0 * 0.95, [0, 0, 1])
    # the wrist: R5/R6/R7 share an origin, opened by the slide rule
    shoot('luna_wrist', mid(p['R4'], p['R7']), [1.0, -0.8, 0.45], 150,
          span(p['R4'], p['R7'], 130) / 10.0 * 0.95, [0, 0, 1])

# ---- G1, after the revert ----
if only_show(BuildArmGen.ARM_NAME) is None:
    log('%s is not in this document' % BuildArmGen.ARM_NAME)
else:
    _f, p, order = frames_of(BuildArmGen.G1_SPEC)
    a, z = p[order[0]], p[order[-1]]
    shoot('g1_arm_iso', mid(a, z), [1.0, -1.0, 0.6], 300, span(a, z, 200) / 10.0 * 0.95,
          [0, 0, 1])

for i in range(root.occurrences.count):
    root.occurrences.item(i).isLightBulbOn = True
log('EXO-218 screenshots done; nothing was posed and every root occurrence is visible again')
