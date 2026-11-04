"""EXO-236 screenshots, TAG exo235 (the tag the issue asks for): the v3 rotor pair with the
two register pegs and the two sockets DELETED.

Same method as EXO-231: `sensor module 2 v3` must not be touched, so everything is done on a
THROWAWAY copy built by the same recipe (`BuildArm.build_v3_copy`) and deleted in this run.

The order matters. The two shots Esteban asked for - the LOWER PART FROM ABOVE, whose top
face z -9.0 is now flat where the two pegs used to stand, and the UPPER PART FROM BELOW,
whose mating face is now unbroken where the two sockets used to be - are taken on the WHOLE
uncut parts, BEFORE any section cut. Only then is the 45..180 deg wedge cut away for the
rotor-stack section (the 45 deg cut face is the horn screw axis, the 180 deg one the M3
axis, so one picture carries both fastener sets in true section).

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo236.py --timeout 1200
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
TAG = 'exo235'
GEN = 'generated_R_arm_general'
TMP = 'sensor module 2 v3 section (throwaway)'
Z_CUT = (-40.0, 14.0)
WEDGE = (45.0, 180.0)

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
        sk = _half_plane_sketch(comp, ang, side, dz, 'exo236 section')
        v = BuildArm._combine_tools(comp, sk, 1, Z_CUT[1] - Z_CUT[0],
                                    adsk.fusion.FeatureOperations.CutFeatureOperation,
                                    'exo236 section %g deg' % ang)
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
            sk = _half_plane_sketch(comp, ang, side, 0.0, 'exo236 fastener section')
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

# ---- 2. the WHOLE parts, before any cut: the two faces the pegs and sockets were on ----
log('the uncut parts, from the mating side:')
show(LOWER_ONLY)
shoot('lower_part_from_above', [0, 0, -9.0], [0.0, 0.0, 1.0], 12.0, 2.3, [0, 1, 0])
shoot('lower_part_from_above_iso', [0, 0, -10.5], [0.30, -0.45, 1.0], 14.0, 2.6, [0, 1, 0])
# the two old peg sites: r 12.2 at 67.5 / 112.5 deg = (+-4.67, 11.27), either side of +Y
shoot('lower_part_peg_site', [0.0, 11.3, -9.0], [0.10, -0.25, 1.0], 8.0, 0.95, [0, 1, 0])
show(UPPER_ONLY)
shoot('upper_part_from_below', [0, 0, -9.0], [0.0, 0.0, -1.0], 12.0, 2.5, [0, 1, 0])
shoot('upper_part_from_below_iso', [0, 0, -7.0], [0.30, 0.45, -1.0], 14.0, 2.8, [0, 1, 0])

# ---- 3. now the section ----
log('the 45..180 deg wedge:')
half_cut(up, 'upper rotor')
half_cut(lo, 'lower rotor')
half_cut(stator, 'stator')
trim_fasteners(nut_occ, 'M3 nuts')
trim_fasteners(bolt_occ, 'M3 bolts')

D = (math.cos(math.radians(292.5)), math.sin(math.radians(292.5)))
show(ROTOR, WEDGE)
shoot('rotor_stack_section', [0, 6, -7.0], [D[0], D[1], 0.45], 22.0, 2.6, [0, 0, 1])
shoot('rotor_stack_section_close', [2, 7, -9.5], [D[0], D[1], 0.25], 14.0, 1.5, [0, 0, 1])
shoot('interface_section', [-11.0, 0.4, -9.5], [0.0, -1.0, 0.18], 12.0, 0.75, [0, 0, 1])
show(ALL, WEDGE)
shoot('module_section', [0, 4, -9.0], [D[0], D[1], 0.40], 40.0, 4.6, [0, 0, 1])

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
log('EXO-236 screenshots done (TAG %s)' % TAG)
