"""EXO-240 screenshots, TAG exo240: the O5.0 horn screw head column opened THROUGH the
upper rotor part, and the hex nut pocket tightened 6.0 -> 5.8 across flats.

Same method as EXO-231/EXO-236/EXO-238: `sensor module 2 v3` must not be touched, so
everything is done on a THROWAWAY copy built by the same recipe (`BuildArm.build_v3_copy`)
and deleted in this run.

The wedge kept is 45..180 deg. 45 deg is a HORN SCREW axis, so the 45 deg cut face runs
right down the middle of a head column with the M2 screw standing in it - the shot Esteban
asked for: the head reachable from the top. 180 deg is an M3 axis, so the same picture
carries the nut in its pocket and the bolt in the nut.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo240.py --timeout 1200
"""
import math
import sys
import importlib

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
import BuildArm
importlib.reload(armgen)
importlib.reload(BuildArm)

OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/'
TAG = 'exo240'
GEN = 'generated_R_arm_general'
TMP = 'sensor module 2 v3 section (throwaway)'
Z_CUT = (-40.0, 14.0)
WEDGE = (45.0, 180.0)   # 45 = a HORN SCREW axis, 180 = an M3 axis

vp = app.activeViewport
root = design.rootComponent
n_comp_before = len(list(design.allComponents))


def save(name):
    h = int(round(1600.0 * vp.height / vp.width))
    path = (OUT + 'generated_R_arm_%s_%s.png' % (TAG, name)) if name.startswith('arm') \
        else (OUT + 'exo_joint_v3_%s_%s.png' % (TAG, name))
    ok = vp.saveAsImageFile(path, 1600, h)
    log('  %-28s -> %s (%s)' % (name, path, ok))


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


def _half_plane_sketch(comp, ang, side, dz, tag):
    u = (math.cos(math.radians(ang)), math.sin(math.radians(ang)))
    nrm = (side * -u[1], side * u[0])
    R = 70.0
    a = (-R * u[0], -R * u[1])
    b = (R * u[0], R * u[1])
    c = (b[0] + R * nrm[0], b[1] + R * nrm[1])
    d = (a[0] + R * nrm[0], a[1] + R * nrm[1])
    segs = []
    armgen._line(segs, a, b)
    armgen._line(segs, b, c)
    armgen._line(segs, c, d)
    armgen._line(segs, d, a)
    sk = BuildArm.PlaneSketch(
        comp, '%s %g deg' % (tag, ang),
        adsk.core.Point3D.create(0.0, 0.0,
                                 ((Z_CUT[0] + Z_CUT[1]) / 2.0 + dz) * BuildArm.MM_TO_CM),
        BuildArm.V(1, 0, 0), BuildArm.V(0, 1, 0))
    sk.loop(segs)
    return sk


def half_cut(occ, what):
    comp = occ.component
    dz = BuildArm._z_offset_of(occ, what)
    for ang, side in ((WEDGE[0], -1.0), (WEDGE[1], +1.0)):
        sk = _half_plane_sketch(comp, ang, side, dz, 'exo240 section')
        v = BuildArm._combine_tools(comp, sk, 1, Z_CUT[1] - Z_CUT[0],
                                    adsk.fusion.FeatureOperations.CutFeatureOperation,
                                    'exo240 section %g deg' % ang)
        log('  %-14s cut at %6.1f deg: %+10.3f mm3' % (what, ang, v))


def _centre_ang(b):
    bb = b.boundingBox
    return math.degrees(math.atan2((bb.minPoint.y + bb.maxPoint.y) / 2.0,
                                   (bb.minPoint.x + bb.maxPoint.x) / 2.0)) % 360.0


def trim_fasteners(occ, what):
    if occ is None:
        return
    comp = occ.component
    for i in reversed(range(comp.bRepBodies.count)):
        b = comp.bRepBodies.item(i)
        a = _centre_ang(b)
        if not (WEDGE[0] - 0.5 <= a <= WEDGE[1] + 0.5):
            log('  %-14s body %s at %6.1f deg: outside the wedge, deleted' % (what, b.name, a))
            b.deleteMe()
    for ang, side in ((WEDGE[0], -1.0), (WEDGE[1], +1.0)):
        for i in range(comp.bRepBodies.count):
            b = comp.bRepBodies.item(i)
            if abs(_centre_ang(b) - ang) > 0.5:
                continue
            v0 = b.volume * 1000.0
            sk = _half_plane_sketch(comp, ang, side, 0.0, 'exo240 fastener section')
            feat = sk.extrude(1, Z_CUT[1] - Z_CUT[0],
                              adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            tool = feat.bodies.item(0)
            coll = adsk.core.ObjectCollection.create()
            coll.add(tool)
            cin = comp.features.combineFeatures.createInput(b, coll)
            cin.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
            cin.isKeepToolBodies = False
            comp.features.combineFeatures.add(cin)
            log('  %-14s body %s halved at %6.1f deg: %.3f -> %.3f mm3'
                % (what, b.name, ang, v0, b.volume * 1000.0))
            break


# ---- 1. the throwaway copy, built by the recipe ----
for i in reversed(range(root.occurrences.count)):
    o = root.occurrences.item(i)
    if o.component.name.startswith(TMP):
        o.deleteMe()
comp, occ, how = BuildArm.build_v3_copy(design, TMP, log)
log('%s: %s' % (TMP, how))
ok, lines = BuildArm.measure_rotor_v3(occ)
if not ok:
    raise RuntimeError('the throwaway section copy does not read back:\n' + '\n'.join(lines))

up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
stator = BuildArm.module_stator_occ(occ, in_context=True)
screws, _old_nuts = BuildArm.module_fastener_occs(occ)
nut_occ, bolt_occ = BuildArm.module_v3_fasteners(occ)
if nut_occ is None or bolt_occ is None:
    raise RuntimeError('the throwaway copy has no standard nuts / M3 bolts')

for i in range(root.occurrences.count):
    root.occurrences.item(i).isLightBulbOn = (root.occurrences.item(i).component.name == TMP)
occ.isLightBulbOn = True


def _ang_of(c):
    bb = c.bRepBodies.item(0).boundingBox
    return math.degrees(math.atan2((bb.minPoint.y + bb.maxPoint.y) / 2.0,
                                   (bb.minPoint.x + bb.maxPoint.x) / 2.0)) % 360.0


def show(names, angles=None):
    for i in range(occ.childOccurrences.count):
        c = occ.childOccurrences.item(i)
        on = c.component.name in names
        if on and angles is not None and c.component.bRepBodies.count == 1:
            a = _ang_of(c)
            on = angles[0] - 1.0 <= a <= angles[1] + 1.0
        c.isLightBulbOn = on
    adsk.doEvents()


ALL = set(occ.childOccurrences.item(i).component.name
          for i in range(occ.childOccurrences.count))
FAST = set([nut_occ.component.name, bolt_occ.component.name])
ROTOR = set([up.component.name, lo.component.name]
            + [x.component.name for x in screws]) | FAST
LOWER_ONLY = set([lo.component.name])
UPPER_ONLY = set([up.component.name])

# ---- 2. the WHOLE parts, before any cut ----
CX, CY = BuildArm._polar(BuildArm.V3_HORN_R, 45.0)     # a horn screw / head column axis
MX, MY = BuildArm._polar(BuildArm.V3_M3_R, 180.0)      # an M3 / nut pocket axis
log('the uncut parts (column axis r %.3f 45 deg = %.3f, %.3f; nut pocket axis r %.1f '
    '180 deg = %.3f, %.3f):'
    % (BuildArm.V3_HORN_R, CX, CY, BuildArm.V3_M3_R, MX, MY))

# the four O5.0 column mouths, open on the rotor face, and the screw heads down them
show(UPPER_ONLY | set(x.component.name for x in screws))
shoot('rotor_face_columns', [0, 0, 0.0], [0.0, 0.0, 1.0], 12.0, 2.6, [0, 1, 0])
shoot('rotor_face_columns_iso', [0, 0, -2.0], [0.35, 0.50, 1.0], 14.0, 3.2, [0, 1, 0])
shoot('column_from_above', [CX, CY, -4.0], [0.0, 0.0, 1.0], 8.0, 0.75, [0, 1, 0])
shoot('column_from_above_wide', [CX, CY, -4.0], [0.10, 0.10, 1.0], 9.0, 1.60, [0, 1, 0])
show(UPPER_ONLY)
shoot('upper_part_from_below', [0, 0, -9.0], [0.0, 0.0, -1.0], 12.0, 3.1, [0, 1, 0])

# the nut pockets from below, nuts in them
show(LOWER_ONLY | FAST)
shoot('nut_pockets_from_below', [0, 0, -13.2], [0.0, 0.0, -1.0], 12.0, 3.0, [0, 1, 0])
shoot('nut_pocket_from_below_close', [MX, MY, -13.0], [-0.25, 0.0, -1.0], 8.0, 0.95,
      [0, 1, 0])

# ---- 3. now the section ----
log('the %g..%g deg wedge:' % WEDGE)
half_cut(up, 'upper rotor')
half_cut(lo, 'lower rotor')
half_cut(stator, 'stator')
trim_fasteners(nut_occ, 'M3 nuts')
trim_fasteners(bolt_occ, 'M3 bolts')

# the 45 deg cut face is seen square-on from 315 deg - the outward normal of the wedge
D = (math.cos(math.radians(315.0)), math.sin(math.radians(315.0)))
SCREWS = set(x.component.name for x in screws)
show(ROTOR, WEDGE)
shoot('column_section', [CX, CY, -4.5], [D[0], D[1], 0.10], 14.0, 1.70, [0, 0, 1])
shoot('column_section_close', [CX, CY, -7.5], [D[0], D[1], 0.05], 10.0, 0.95, [0, 0, 1])
shoot('column_section_screw', [CX, CY, -8.2], [D[0], D[1], 0.25], 9.0, 0.60, [0, 0, 1])
shoot('rotor_stack_section', [0, 2, -6.5], [D[0], D[1], 0.45], 22.0, 4.2, [0, 0, 1])
shoot('nut_pocket_section', [MX, MY, -11.6], [0.0, -1.0, 0.10], 12.0, 1.20, [0, 0, 1])
shoot('nut_pocket_section_wide', [MX, MY, -10.5], [0.0, -1.0, 0.25], 14.0, 2.60, [0, 0, 1])
shoot('interface_section', [MX, 0.4, -9.8], [0.0, -1.0, 0.18], 12.0, 1.50, [0, 0, 1])
show(ALL, WEDGE)
shoot('module_section', [0, 2, -10.0], [D[0], D[1], 0.40], 40.0, 7.0, [0, 0, 1])

# ---- 4. the throwaway goes away ----
occ.deleteMe()
for i in reversed(range(root.occurrences.count)):
    o = root.occurrences.item(i)
    if o.component.name.startswith(TMP):
        o.deleteMe()
n_comp_after = len(list(design.allComponents))
log('components %d -> %d after deleting the throwaway (%+d)'
    % (n_comp_before, n_comp_after, n_comp_after - n_comp_before))

# ---- 5. the arm ----
arm = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    o.isLightBulbOn = (o.component.name == GEN)
    if o.component.name == GEN:
        arm = o
if arm is None:
    log('WARNING: %s is not in this document - no arm shot' % GEN)
else:
    adsk.doEvents()
    cam = vp.camera
    cam.isFitView = True
    cam.isSmoothTransition = False
    vp.camera = cam
    adsk.doEvents()
    vp.refresh()
    adsk.doEvents()
    save('arm_iso')
log('EXO-240 screenshots done (TAG %s)' % TAG)
