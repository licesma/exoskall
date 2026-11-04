"""EXO-242 screenshots, TAG exo242: both arms rebuilt in the SAVED document, and the module.

Three frames, the ones EXO-242 asks for:

  generated_R_arm_exo242_g1_arm_iso.png     `generated_R_arm_general`, six links
  generated_R_arm_exo242_luna_arm_iso.png   `generated_R_arm_luna`, five links (EXO-224)
  exo_joint_v3_exo242_module_section.png    `sensor module 2 v3`, wedge 45..180 deg

Nothing is posed: every occurrence stays where the builder put it. `sensor module 2 v3`
itself is never cut - the section is a THROWAWAY copy built by the same recipe
(`BuildArm.build_v3_copy`) and deleted in this run, the EXO-231/EXO-238/EXO-240 method.
The document is SAVED at the end (EXO-242) so the throwaway's deletion is on disk too.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo242.py --timeout 1800
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
TAG = 'exo242'
TMP = 'sensor module 2 v3 section (throwaway)'
Z_CUT = (-40.0, 14.0)
WEDGE = (45.0, 180.0)      # 45 = a HORN SCREW axis, 180 = an M3 / nut pocket axis

vp = app.activeViewport
root = design.rootComponent
log('document: %r' % app.activeDocument.name)
n_comp_before = len(list(design.allComponents))


def save(name):
    h = int(round(1600.0 * vp.height / vp.width))
    path = OUT + ('generated_R_arm_%s_%s.png' % (TAG, name) if name.endswith('arm_iso')
                  else 'exo_joint_v3_%s_%s.png' % (TAG, name))
    ok = vp.saveAsImageFile(path, 1600, h)
    log('  %-18s -> %s (%s)' % (name, path, ok))


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
    vp.refresh()
    adsk.doEvents()
    save(name)


def arm_occ(name):
    for i in range(root.occurrences.count):
        o = root.occurrences.item(i)
        if o.component.name == name:
            return o
    return None


def only_show(name):
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
    joints, by_child = armgen.parse_urdf(spec['urdf'])
    origin, _a = armgen.joint_world_pose(spec['origin_at'], joints, by_child)
    off = spec.get('offset', (0.0, 0.0, 0.0))
    shift = [origin[i] - off[i] for i in range(3)]
    links = sweepgen.spec_links(spec)
    order = [j['label'] for j in sweepgen.spec_joints(spec)]
    _s, _l, table = sweepgen.seed_frames(spec, joints, by_child, shift)
    seed, _combo, _slides, _rows = sweepgen.solve_axis_signs(
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
    return dict((lab, [v * 1000.0 for v in frames[lab][1]]) for lab in order), order


def span(p, q, pad):
    return math.sqrt(sum((q[i] - p[i]) ** 2 for i in range(3))) + pad


def mid(p, q):
    return [(p[i] + q[i]) / 2.0 for i in range(3)]


# ---- 1. the two arms ----
for spec, name, key in ((BuildArmGen.G1_SPEC, BuildArmGen.ARM_NAME, 'g1'),
                        (BuildArmGen.LUNA_SPEC, BuildArmGen.LUNA_ARM_NAME, 'luna')):
    if only_show(name) is None:
        log('WARNING: %s is not in this document - no %s shot' % (name, key))
        continue
    p, order = frames_of(spec)
    a, z = p[order[0]], p[order[-1]]
    shoot('%s_arm_iso' % key, mid(a, z), [1.0, -1.0, 0.6], 400,
          span(a, z, 260) / 10.0 * 0.95, [0, 0, 1])

# ---- 2. the module, sectioned on a throwaway copy ----
for i in reversed(range(root.occurrences.count)):
    o = root.occurrences.item(i)
    if o.component.name.startswith(TMP):
        o.deleteMe()
comp, occ, how = BuildArm.build_v3_copy(design, TMP, log)
log('%s: %s' % (TMP, how))
ok, lines = BuildArm.measure_rotor_v3(occ)
if not ok:
    raise RuntimeError('the throwaway section copy does not read back:\n' + '\n'.join(lines))


def _half_plane_sketch(c, ang, side, dz, tag):
    u = (math.cos(math.radians(ang)), math.sin(math.radians(ang)))
    nrm = (side * -u[1], side * u[0])
    R = 70.0
    a = (-R * u[0], -R * u[1])
    b = (R * u[0], R * u[1])
    cc = (b[0] + R * nrm[0], b[1] + R * nrm[1])
    d = (a[0] + R * nrm[0], a[1] + R * nrm[1])
    segs = []
    armgen._line(segs, a, b)
    armgen._line(segs, b, cc)
    armgen._line(segs, cc, d)
    armgen._line(segs, d, a)
    sk = BuildArm.PlaneSketch(
        c, '%s %g deg' % (tag, ang),
        adsk.core.Point3D.create(0.0, 0.0,
                                 ((Z_CUT[0] + Z_CUT[1]) / 2.0 + dz) * BuildArm.MM_TO_CM),
        BuildArm.V(1, 0, 0), BuildArm.V(0, 1, 0))
    sk.loop(segs)
    return sk


def _centre_ang(b):
    bb = b.boundingBox
    return math.degrees(math.atan2((bb.minPoint.y + bb.maxPoint.y) / 2.0,
                                   (bb.minPoint.x + bb.maxPoint.x) / 2.0)) % 360.0


def half_cut(o, what):
    c = o.component
    dz = BuildArm._z_offset_of(o, what)
    for ang, side in ((WEDGE[0], -1.0), (WEDGE[1], +1.0)):
        sk = _half_plane_sketch(c, ang, side, dz, 'exo242 section')
        v = BuildArm._combine_tools(c, sk, 1, Z_CUT[1] - Z_CUT[0],
                                    adsk.fusion.FeatureOperations.CutFeatureOperation,
                                    'exo242 section %g deg' % ang)
        log('  %-14s cut at %6.1f deg: %+10.3f mm3' % (what, ang, v))


def trim_fasteners(o, what):
    if o is None:
        return
    c = o.component
    for i in reversed(range(c.bRepBodies.count)):
        b = c.bRepBodies.item(i)
        a = _centre_ang(b)
        if not (WEDGE[0] - 0.5 <= a <= WEDGE[1] + 0.5):
            b.deleteMe()
    for ang, side in ((WEDGE[0], -1.0), (WEDGE[1], +1.0)):
        for i in range(c.bRepBodies.count):
            b = c.bRepBodies.item(i)
            if abs(_centre_ang(b) - ang) > 0.5:
                continue
            sk = _half_plane_sketch(c, ang, side, 0.0, 'exo242 fastener section')
            feat = sk.extrude(1, Z_CUT[1] - Z_CUT[0],
                              adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            tool = feat.bodies.item(0)
            coll = adsk.core.ObjectCollection.create()
            coll.add(tool)
            cin = c.features.combineFeatures.createInput(b, coll)
            cin.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
            cin.isKeepToolBodies = False
            c.features.combineFeatures.add(cin)
            log('  %-14s %s halved at %6.1f deg' % (what, b.name, ang))
            break


up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
stator = BuildArm.module_stator_occ(occ, in_context=True)
nut_occ, bolt_occ = BuildArm.module_v3_fasteners(occ)
for o, what in ((up, 'upper'), (lo, 'lower'), (stator, 'stator')):
    half_cut(o, what)
for o, what in ((nut_occ, 'nuts'), (bolt_occ, 'bolts')):
    trim_fasteners(o, what)

for i in range(root.occurrences.count):
    root.occurrences.item(i).isLightBulbOn = (
        root.occurrences.item(i).component.name == TMP)
occ.isLightBulbOn = True
for i in range(occ.childOccurrences.count):
    occ.childOccurrences.item(i).isLightBulbOn = True
adsk.doEvents()
D = (math.cos(math.radians((WEDGE[0] + WEDGE[1]) / 2.0)),
     math.sin(math.radians((WEDGE[0] + WEDGE[1]) / 2.0)))
shoot('module_section', [0, 2, -10.0], [D[0], D[1], 0.40], 40.0, 7.0, [0, 0, 1])

occ.deleteMe()
for i in reversed(range(root.occurrences.count)):
    o = root.occurrences.item(i)
    if o.component.name.startswith(TMP):
        o.deleteMe()
n_comp_after = len(list(design.allComponents))
log('components %d -> %d after deleting the throwaway (%+d)'
    % (n_comp_before, n_comp_after, n_comp_after - n_comp_before))

for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    o.isLightBulbOn = not o.component.name.startswith(BuildArm.MODULE_V3_NAME)
log('EXO-242 screenshots done (TAG %s)' % TAG)
log(BuildArm.save_active_document(app))
