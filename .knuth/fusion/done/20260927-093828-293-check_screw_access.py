import sys, importlib, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen; importlib.reload(armgen)
import adsk.core as ac, adsk.fusion as af

root = design.rootComponent
arm = next(o for o in root.occurrences if o.component.name.startswith('generated_R_arm'))
hr = armgen.HOLE_R

# every body in the design that is NOT the link under test
def other_bodies(skip_occ):
    out = []
    def walk(occ):
        for b in occ.bRepBodies: out.append((occ.name, b))
        for c in occ.childOccurrences: walk(c)
    for o in arm.childOccurrences:      # only the GENERATED arm's own joints/links
        if o is skip_occ: continue
        walk(o)
    return out

log('Screw access: from each hole, walk OUT along the hole axis and see what is in the way')
for sub in arm.childOccurrences:
    nm = sub.component.name
    if not nm.startswith('link_45') or sub.bRepBodies.count == 0: continue
    body = sub.bRepBodies.item(0)
    others = other_bodies(sub)
    log('%s  (checking against %d other bodies)' % (nm, len(others)))
    seen = []
    for f in body.faces:
        g = f.geometry
        if g.objectType != 'adsk::core::Cylinder' or abs(g.radius*10.0 - hr) > 0.05: continue
        k = (round(g.origin.x,4), round(g.origin.y,4), round(g.origin.z,4))
        if k in seen: continue
        seen.append(k)
        o, ax = g.origin, g.axis
        for sign in (+1, -1):
            hits = []
            for d_mm in [x*0.5 for x in range(1, 61)]:   # 0.5 .. 30 mm out
                p = ac.Point3D.create(o.x + sign*ax.x*d_mm/10.0,
                                      o.y + sign*ax.y*d_mm/10.0,
                                      o.z + sign*ax.z*d_mm/10.0)
                for onm, ob in others:
                    if ob.pointContainment(p) == af.PointContainment.PointInsidePointContainment:
                        hits.append((d_mm, onm, ob.name)); break
                if hits: break
            tag = 'clear to 30 mm' if not hits else 'BLOCKED at %.1f mm by %s/%s' % hits[0]
            log('  hole (%6.1f,%6.1f,%6.1f) dir %+d: %s' % (o.x*10, o.y*10, o.z*10, sign, tag))




# ---------------- EXO-225: the two screw sets of the v3 module ----------------
# The whole point of v3 is the ORDER: the four horn screws (M2x4, r 6.001, at 45/135/225/
# 315 deg) go into the LOWER rotor part alone, before the upper part and before the link
# that bolts to this rotor; the four M3 bolts (r 11, at 0/90/180/270 deg) come last, from
# above the link's rotor pad. Access is therefore checked in two stages, each against the
# bodies that are really there at that moment, and each along a DRIVER and not a line: the
# axis plus a ring of four points 1.5 mm out, so a Ø3 shaft has to fit too.
import math as _m
import importlib as _il
import BuildArm as _BA
_il.reload(_BA)

DRIVER_R = 1.5          # mm, half the shaft of the driver that has to reach the head
REACH = 30.0            # mm out along module +z from the head
STEP = 0.5

_gen = None
for _i in range(root.occurrences.count):
    _o = root.occurrences.item(_i)
    if _o.component.name == 'generated_R_arm_general':
        _gen = _o
if _gen is None:
    raise RuntimeError('generated_R_arm_general is not in this document')


def _module_of(wrapper):
    for i in range(wrapper.childOccurrences.count):
        c = wrapper.childOccurrences.item(i)
        if c.component.name.startswith('sensor module 2'):
            return c
    return None


def _bodies(occ, skip_names, out):
    """Every body under `occ`, skipping occurrences whose COMPONENT name is in
    `skip_names` - identity is no use here, because each fetch of a child occurrence is a
    fresh proxy, and within ONE module a component name picks exactly the parts meant."""
    if occ.component.name in skip_names:
        return out
    for b in occ.bRepBodies:
        out.append((occ.name, b, b.boundingBox))
    for i in range(occ.childOccurrences.count):
        _bodies(occ.childOccurrences.item(i), skip_names, out)
    return out


def _world(mat, x, y, z):
    a = mat.asArray()
    return ac.Point3D.create((a[0] * x + a[1] * y + a[2] * z) / 10.0 + a[3],
                             (a[4] * x + a[5] * y + a[6] * z) / 10.0 + a[7],
                             (a[8] * x + a[9] * y + a[10] * z) / 10.0 + a[11])


def _walk(mat, r, ang, z0, obstacles):
    """From (r, ang, z0) in the module frame walk out along module +z with a Ø3 driver.
    Bodies are pre-filtered on the bounding box of the swept corridor, so the containment
    test only ever runs against something that could possibly be in the way."""
    cx, cy = r * _m.cos(_m.radians(ang)), r * _m.sin(_m.radians(ang))
    ring = [(0.0, 0.0)] + [(DRIVER_R * _m.cos(_m.radians(a)),
                            DRIVER_R * _m.sin(_m.radians(a))) for a in (0, 90, 180, 270)]
    ends = [_world(mat, cx, cy, z0), _world(mat, cx, cy, z0 + REACH)]
    lo = [min(e.asArray()[i] for e in ends) - DRIVER_R / 10.0 for i in range(3)]
    hi = [max(e.asArray()[i] for e in ends) + DRIVER_R / 10.0 for i in range(3)]
    near = []
    for onm, ob, bb in obstacles:
        p0, p1 = bb.minPoint.asArray(), bb.maxPoint.asArray()
        if all(p0[i] <= hi[i] and p1[i] >= lo[i] for i in range(3)):
            near.append((onm, ob))
    for k in range(1, int(REACH / STEP) + 1):
        d = k * STEP
        for ox, oy in ring:
            p = _world(mat, cx + ox, cy + oy, z0 + d)
            for onm, ob in near:
                if ob.pointContainment(p) == af.PointContainment.PointInsidePointContainment:
                    return d, onm, ob.name
    return None, len(near)


log('')
log('=== EXO-225 screw access, both sets, every joint of generated_R_arm_general ===')
log('driver Ø%.1f mm, %.0f mm of reach out along module +z' % (2 * DRIVER_R, REACH))
HORN = (6.001, (45.0, 135.0, 225.0, 315.0), -9.0)    # r, angles, head seat z
M3 = (11.0, (0.0, 90.0, 180.0, 270.0), 8.0)          # r, angles, bolt head z = link pad top
bad_total = 0
for wi in range(_gen.childOccurrences.count):
    wr = _gen.childOccurrences.item(wi)
    mod = _module_of(wr)
    if mod is None:
        continue
    mat = mod.transform.copy()
    mat.transformBy(wr.transform)
    up, lo_part = _BA.module_rotor_occs(mod, in_context=True)
    screws, nuts = _BA.module_fastener_occs(mod)
    lab = wr.component.name.split('_')[0][1:]        # 'GJ3_...' -> 'J3'
    own_link = 'g_link_%s' % lab[1:]                 # the link that bolts to THIS rotor
    skip1 = set([up.component.name] + [s.component.name for s in screws])
    # stage 1: on the horn, nothing else yet
    obs1 = []
    for si in range(_gen.childOccurrences.count):
        sub = _gen.childOccurrences.item(si)
        if sub.component.name.startswith(own_link):
            continue                                  # this joint's own link is not on yet
        _bodies(sub, skip1 if si == wi else set(), obs1)
    # stage 2: the arm as it finally is
    obs2 = []
    for si in range(_gen.childOccurrences.count):
        _bodies(_gen.childOccurrences.item(si), set(), obs2)
    bad = 0
    for a in HORN[1]:
        hit = _walk(mat, HORN[0], a, HORN[2], obs1)
        ok = hit[0] is None
        bad += 0 if ok else 1
        log('  %-22s horn screw %5.1f deg from z %5.1f: %s'
            % (wr.component.name, a, HORN[2],
               'clear to %.0f mm (%d bodies in the corridor)' % (REACH, hit[1]) if ok
               else 'BLOCKED at %.1f mm by %s/%s' % hit))
    for a in M3[1]:
        hit = _walk(mat, M3[0], a, M3[2], obs2)
        ok = hit[0] is None
        bad += 0 if ok else 1
        log('  %-22s M3 bolt    %5.1f deg from z %5.1f: %s'
            % (wr.component.name, a, M3[2],
               'clear to %.0f mm (%d bodies in the corridor)' % (REACH, hit[1]) if ok
               else 'BLOCKED at %.1f mm by %s/%s' % hit))
    bad_total += bad
    log('  %-22s %s' % (wr.component.name, 'ACCESS OK' if bad == 0 else '%d BLOCKED' % bad))
log('EXO-225 screw access: %s' % ('ALL CLEAR' if bad_total == 0 else '%d BLOCKED' % bad_total))
