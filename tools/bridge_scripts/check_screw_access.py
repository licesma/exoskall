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




# ---------------- EXO-231: the two screw sets, in ASSEMBLY ORDER ----------------
# The whole point of v3 is the ORDER: the four horn screws (M2x4, r 6.001, at 45/135/225/
# 315 deg) go into the LOWER rotor part alone, before the upper part and before the link
# that bolts to this rotor; the four M3 bolts (M3x20, r 11, at 0/90/180/270 deg) come last,
# from above the link's rotor pad (module z = armgen.PAD_T = 8.0).
#
# EXO-231 fixes what EXO-225 got wrong here: the M3s were tested against the FINISHED arm,
# and that is not when they go in. The arm is built proximal to distal - J1, link_12, J2,
# link_23, ... - so when joint k's M3 bolts are driven the only things in the world are the
# modules J1..Jk and the links up to and including link_k(k+1), the one being bolted on.
# Nothing distal to joint k exists. Both sets are now tested against exactly the bodies
# that are there at that step.
#
# Two corridors are measured for the M3, not one:
#   * the BOLT corridor, radius `BOLT_R` (the Ø5.5 head plus a little): the bolt is held
#     with its tip on the pad face and lowered straight in, so it needs its own whole
#     length, `NEED_BOLT` = under-head 20.0 + head 3.0 = 23.0 mm, clear above the pad;
#   * the DRIVER corridor, radius `DRIVER_R` (a Ø3 shaft), `NEED_DRIVER` = 30 mm.
# The horn screws keep EXO-225's single Ø3 driver corridor.
import math as _m
import importlib as _il
import BuildArm as _BA
_il.reload(_BA)

DRIVER_R = 1.5          # mm, half the shaft of the driver that has to reach the head
BOLT_R = 2.9            # mm, the M3 head Ø5.5 + 0.15 all round
REACH = 40.0            # mm out along module +z from the head
STEP = 0.5
NEED_DRIVER = 30.0      # mm of driver reach
BOLT_LEN = _BA.V3_BOLT_L + _BA.V3_BOLT_HEAD_H     # 20.0 under-head + 3.0 head = 23.0
NEED_BOLT = BOLT_LEN    # straight insertion needs the bolt's whole length above the pad

_gen = None
for _i in range(root.occurrences.count):
    _o = root.occurrences.item(_i)
    if _o.component.name == 'generated_R_arm_general':
        _gen = _o
if _gen is None:
    raise RuntimeError('generated_R_arm_general is not in this document')


# The part names are read ONCE, off the root-level `sensor module 2 v3` occurrence, which
# is the only one placed at identity - `module_rotor_occs(in_context=True)` reads body
# boxes in WORLD coordinates, and a module placed on a joint is rotated, so asking a placed
# one would find nothing. Every joint places the same component, so the names carry over.
_v3 = None
for _i in range(root.occurrences.count):
    if root.occurrences.item(_i).component.name == _BA.MODULE_V3_NAME:
        _v3 = root.occurrences.item(_i)
if _v3 is None:
    raise RuntimeError('%s is not at the root of this document' % _BA.MODULE_V3_NAME)
_up0, _lo0 = _BA.module_rotor_occs(_v3, in_context=True)
_screws0, _nuts0 = _BA.module_fastener_occs(_v3)
if _up0 is None or _lo0 is None or len(_screws0) != 4:
    raise RuntimeError('could not read the v3 parts off %s' % _BA.MODULE_V3_NAME)
# EXO-231: the M3 bolts are bodies now. A bolt is never an obstacle to its own hole, so the
# component that carries them is left out of the joint under test in BOTH stages (at stage 1
# it is not in the world at all). Tolerated missing, so this check also runs on a document
# whose module predates EXO-231.
_nut_occ, _bolt_occ = _BA.module_v3_fasteners(_v3)
STAGE1_SKIP = set([_up0.component.name] + [s.component.name for s in _screws0])
if _bolt_occ is not None:
    STAGE1_SKIP.add(_bolt_occ.component.name)
BOLT_NAME = None if _bolt_occ is None else _bolt_occ.component.name
log('stage 1 leaves out, in the joint under test: %s' % sorted(STAGE1_SKIP))
log('the M3 bolt bodies live in: %s' % BOLT_NAME)


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


def _walk(mat, r, ang, z0, obstacles, probe_r=DRIVER_R, reach=REACH):
    """From (r, ang, z0) in the module frame walk out along module +z with a cylinder of
    radius `probe_r`. Returns (blocked_at_mm, occ_name, body_name) or (None, n_near).
    Bodies are pre-filtered on the bounding box of the swept corridor, so the containment
    test only ever runs against something that could possibly be in the way."""
    cx, cy = r * _m.cos(_m.radians(ang)), r * _m.sin(_m.radians(ang))
    ring = [(0.0, 0.0)] + [(probe_r * _m.cos(_m.radians(a)),
                            probe_r * _m.sin(_m.radians(a)))
                           for a in range(0, 360, 45)]
    ends = [_world(mat, cx, cy, z0), _world(mat, cx, cy, z0 + reach)]
    lo = [min(e.asArray()[i] for e in ends) - probe_r / 10.0 for i in range(3)]
    hi = [max(e.asArray()[i] for e in ends) + probe_r / 10.0 for i in range(3)]
    near = []
    for onm, ob, bb in obstacles:
        p0, p1 = bb.minPoint.asArray(), bb.maxPoint.asArray()
        if all(p0[i] <= hi[i] and p1[i] >= lo[i] for i in range(3)):
            near.append((onm, ob))
    for k in range(1, int(reach / STEP) + 1):
        d = k * STEP
        for ox, oy in ring:
            p = _world(mat, cx + ox, cy + oy, z0 + d)
            for onm, ob in near:
                if ob.pointContainment(p) == af.PointContainment.PointInsidePointContainment:
                    return d, onm, ob.name
    return None, len(near)


def _free(hit, reach=REACH):
    """How much straight room the walk found, mm."""
    return reach if hit[0] is None else hit[0] - STEP


log('')
log('=== EXO-231 screw access, both sets, every joint of generated_R_arm_general ===')
log('horn screw: Ø%.1f driver, %.0f mm of reach. M3: Ø%.1f bolt corridor needs %.1f mm '
    '(M3x%.0f, head %.1f), Ø%.1f driver needs %.0f mm.'
    % (2 * DRIVER_R, NEED_DRIVER, 2 * BOLT_R, NEED_BOLT, _BA.V3_BOLT_L,
       _BA.V3_BOLT_HEAD_H, 2 * DRIVER_R, NEED_DRIVER))
HORN = (6.001, (45.0, 135.0, 225.0, 315.0), -9.0)    # r, angles, head seat z
M3 = (_BA.V3_M3_R, _BA.V3_M3_ANG, armgen.PAD_T)      # r, angles, bolt head z = pad top
OLD_HEAD_Z = -5.0       # where the horn screw head sits in the module as DRAWN today
bad_total = 0
old_blocked = 0
wraps = [_gen.childOccurrences.item(i) for i in range(_gen.childOccurrences.count)]
for wi, wr in enumerate(wraps):
    mod = _module_of(wr)
    if mod is None:
        continue
    mat = mod.transform.copy()
    mat.transformBy(wr.transform)
    k = int(wr.component.name.split('_')[0][2:])     # 'GJ3_...' -> 3
    # Stage 1 is the moment this joint's horn screws go in. Nothing distal to this joint
    # exists yet: no link off this rotor, no module beyond it. This joint's own upper rotor
    # part, its own screws and its own M3 bolts are not there either.
    obs1 = []
    for si, sub in enumerate(wraps):
        n = sub.component.name
        if n.startswith('GJ') and int(n.split('_')[0][2:]) > k:
            continue
        if n.startswith('g_link_') and int(n.split('_')[2][0]) >= k:
            continue
        _bodies(sub, STAGE1_SKIP if si == wi else set(), obs1)
    # Stage 2 is the moment this joint's M3 bolts go in: the same modules J1..Jk, plus
    # link_k(k+1) - the link whose rotor pad these very bolts are fastening. Still nothing
    # beyond. The joint's own four bolts are not obstacles to their own holes.
    obs2 = []
    for si, sub in enumerate(wraps):
        n = sub.component.name
        if n.startswith('GJ') and int(n.split('_')[0][2:]) > k:
            continue
        if n.startswith('g_link_') and int(n.split('_')[2][0]) > k:
            continue
        _bodies(sub, set([BOLT_NAME]) if (si == wi and BOLT_NAME) else set(), obs2)
    # and the arm as it FINALLY is - what EXO-225 measured, kept so the two can be compared
    obs_done = []
    for si, sub in enumerate(wraps):
        _bodies(sub, set([BOLT_NAME]) if (si == wi and BOLT_NAME) else set(), obs_done)
    bad = 0
    for a in HORN[1]:
        hit = _walk(mat, HORN[0], a, HORN[2], obs1, DRIVER_R, NEED_DRIVER)
        ok = hit[0] is None
        bad += 0 if ok else 1
        # and the same hole as the module is DRAWN today, whose head sits 4 mm higher, so
        # any blockage can be told apart from one EXO-225 introduced
        was = _walk(mat, HORN[0], a, OLD_HEAD_Z, obs1, DRIVER_R, NEED_DRIVER)
        log('  %-22s horn screw %5.1f deg from z %5.1f: %s [as drawn, from z %5.1f: %s]'
            % (wr.component.name, a, HORN[2],
               'clear to %.0f mm (%d bodies in the corridor)' % (NEED_DRIVER, hit[1]) if ok
               else 'BLOCKED at %.1f mm by %s/%s' % hit,
               OLD_HEAD_Z, 'clear' if was[0] is None else 'BLOCKED at %.1f mm' % was[0]))
    for a in M3[1]:
        h_bolt = _walk(mat, M3[0], a, M3[2], obs2, BOLT_R, REACH)
        h_drv = _walk(mat, M3[0], a, M3[2], obs2, DRIVER_R, REACH)
        h_done = _walk(mat, M3[0], a, M3[2], obs_done, DRIVER_R, REACH)
        f_bolt, f_drv, f_done = _free(h_bolt), _free(h_drv), _free(h_done)
        ok = f_bolt >= NEED_BOLT and f_drv >= NEED_DRIVER
        bad += 0 if ok else 1
        if f_done < NEED_DRIVER:
            old_blocked += 1
        log('  %-22s M3 bolt    %5.1f deg from z %5.1f: bolt Ø%.1f free %5.1f (need %.1f) '
            '%s | driver Ø%.1f free %5.1f (need %.1f) %s | finished arm: driver free %5.1f %s'
            % (wr.component.name, a, M3[2], 2 * BOLT_R, f_bolt, NEED_BOLT,
               'OK' if f_bolt >= NEED_BOLT else 'by %s/%s' % (h_bolt[1], h_bolt[2]),
               2 * DRIVER_R, f_drv, NEED_DRIVER,
               'OK' if f_drv >= NEED_DRIVER else 'by %s/%s' % (h_drv[1], h_drv[2]),
               f_done, 'OK' if f_done >= NEED_DRIVER else 'BLOCKED'))
    bad_total += bad
    log('  %-22s %s' % (wr.component.name, 'ACCESS OK' if bad == 0 else '%d BLOCKED' % bad))
log('EXO-231 screw access, assembly order: %s'
    % ('ALL CLEAR' if bad_total == 0 else '%d BLOCKED' % bad_total))
log('EXO-230 item 1, re-measured: %d of 28 M3 holes are blocked against the FINISHED arm '
    '(what EXO-225 tested), %d against the arm as it stands when the bolts go in'
    % (old_blocked, bad_total))
