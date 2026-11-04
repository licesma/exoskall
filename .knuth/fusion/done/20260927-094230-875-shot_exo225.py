"""EXO-225 screenshots, TAG exo225: the v3 rotor stack in section, with BOTH screw sets,
and the arm.

A real section needs material taken away, and `sensor module 2 v3` must not be touched, so
the section is made on a THROWAWAY copy built by the same recipe (`BuildArm.build_v3_copy`)
under another name, cut, photographed and deleted again in this one run. The kept wedge is
45..180 deg: its two cut faces are the 45 deg HORN SCREW axis and the 180 deg M3 axis, so
one picture carries both screw sets in true section - the M2 seated on the lower part's web
with its head inside the upper part's blind pocket, and the M3 hole running from the rotor
face down to the hex nut captured under the lower part.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo225.py --timeout 900
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
TAG = 'exo225'
GEN = 'generated_R_arm_general'
TMP = 'sensor module 2 v3 section (throwaway)'
Z_CUT = (-40.0, 12.0)          # the cut tools span the whole module in z

vp = app.activeViewport
root = design.rootComponent
n_comp_before = len(list(design.allComponents))


def save(name):
    h = int(round(1600.0 * vp.height / vp.width))
    path = OUT + 'exo_joint_v3_%s_%s.png' % (TAG, name) if not name.startswith('arm') \
        else OUT + 'generated_R_arm_%s_%s.png' % (TAG, name)
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
    vp.refresh()
    adsk.doEvents()
    save(name)


def half_cut(occ, keep_lo_deg, keep_hi_deg, what):
    """Take the half-space on the far side of the ray at `keep_lo_deg` (and again at
    `keep_hi_deg`) out of the occurrence's single body, leaving the wedge between them."""
    comp = occ.component
    dz = BuildArm._z_offset_of(occ, what)
    for ang, side in ((keep_lo_deg, -1.0), (keep_hi_deg, +1.0)):
        # remove everything on the `side` side of the line through the origin at `ang`
        u = (math.cos(math.radians(ang)), math.sin(math.radians(ang)))     # along the line
        nrm = (side * -u[1], side * u[0])                                  # away from the wedge
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
            comp, 'exo225 section %g deg' % ang,
            adsk.core.Point3D.create(0.0, 0.0,
                                     ((Z_CUT[0] + Z_CUT[1]) / 2.0 + dz) * BuildArm.MM_TO_CM),
            BuildArm.V(1, 0, 0), BuildArm.V(0, 1, 0))
        sk.loop(segs)
        v = BuildArm._combine_tools(comp, sk, 1, Z_CUT[1] - Z_CUT[0],
                                    adsk.fusion.FeatureOperations.CutFeatureOperation,
                                    'exo225 section %g deg' % ang)
        log('  %-14s cut at %6.1f deg: %+10.3f mm3' % (what, ang, v))


# ---- 1. the throwaway section copy ----
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
screws, nuts = BuildArm.module_fastener_occs(occ)
half_cut(up, 45.0, 180.0, 'upper rotor')
half_cut(lo, 45.0, 180.0, 'lower rotor')
half_cut(stator, 45.0, 180.0, 'stator')

# ---- 2. what is visible in each frame ----
for i in range(root.occurrences.count):
    root.occurrences.item(i).isLightBulbOn = (root.occurrences.item(i).component.name == TMP)
occ.isLightBulbOn = True


def _ang_of(c):
    """Where an occurrence's body sits round the module axis, deg."""
    bb = c.bRepBodies.item(0).boundingBox
    x = (bb.minPoint.x + bb.maxPoint.x) / 2.0
    y = (bb.minPoint.y + bb.maxPoint.y) / 2.0
    return math.degrees(math.atan2(y, x)) % 360.0


def show(names, angles=None):
    """Light bulbs by COMPONENT NAME - each fetch of a child occurrence is a fresh proxy,
    so identity is no use, and inside one module a name picks exactly one role. `angles`
    additionally keeps only the occurrences that sit inside the kept wedge, so no screw or
    nut is left hanging in the air where its material was cut away."""
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
ROTOR = set([up.component.name, lo.component.name]
            + [x.component.name for x in screws + nuts])
LOWER_ONLY = set([lo.component.name] + [x.component.name for x in screws + nuts])
WEDGE = (45.0, 180.0)

D = (math.cos(math.radians(292.5)), math.sin(math.radians(292.5)))   # into the cut wedge

show(ROTOR, WEDGE)
shoot('rotor_section', [0, 6, -7.0], [D[0], D[1], 0.45], 22.0, 2.6, [0, 0, 1])
shoot('rotor_section_close', [2, 7, -9.5], [D[0], D[1], 0.25], 14.0, 1.5, [0, 0, 1])
show(LOWER_ONLY, WEDGE)
shoot('lower_part_with_horn_screws', [0, 5, -10.5], [D[0], D[1], 0.75], 18.0, 2.4, [0, 0, 1])
shoot('nut_pockets_from_below', [0, 4, -12.0], [0.35, -0.65, -1.0], 18.0, 2.4, [0, 1, 0])
show(ALL, WEDGE)
shoot('module_section', [0, 4, -11.0], [D[0], D[1], 0.40], 40.0, 4.2, [0, 0, 1])

# ---- 3. the throwaway goes away ----
occ.deleteMe()
for i in reversed(range(root.occurrences.count)):
    o = root.occurrences.item(i)
    if o.component.name.startswith(TMP):
        o.deleteMe()
n_comp_after = len(list(design.allComponents))
log('components %d -> %d after deleting the throwaway (%+d)'
    % (n_comp_before, n_comp_after, n_comp_after - n_comp_before))

# ---- 4. the arm ----
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
log('EXO-225 screenshots done')
