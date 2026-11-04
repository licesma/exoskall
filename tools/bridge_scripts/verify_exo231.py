"""EXO-231 evidence: the v3 module with a STANDARD ISO 4032 M3 nut in a 2.6 mm pocket and
the M3x20 bolt modelled, all read off the SOLID.

Runs `BuildArm.ensure_v3_module` (which re-makes the module if what is in the document does
not read back), then measures: the original is untouched, the pocket, the wall band, the
horn-disk clearance, the nut, the bolt, its tip, the thread engagement and the clearance
from the tip down to the XL330.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/verify_exo231.py --timeout 900
"""
import math, sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
import adsk.core as ac, adsk.fusion as af

root = design.rootComponent
INSIDE = (af.PointContainment.PointInsidePointContainment,
          af.PointContainment.PointOnPointContainment)
problems = []


def solid(body, r, ang, z):
    x, y = BuildArm._polar(r, ang)
    return body.pointContainment(ac.Point3D.create(x / 10.0, y / 10.0, z / 10.0)) in INSIDE


def edge_z(body, r, ang, z_in, z_out):
    """The face between a solid z and a void z on the (r, ang) line, bisected."""
    for _ in range(60):
        m = (z_in + z_out) / 2.0
        z_in, z_out = (m, z_out) if solid(body, r, ang, m) else (z_in, m)
    return (z_in + z_out) / 2.0


def edge_r(body, ang, z, r_in, r_out):
    for _ in range(60):
        m = (r_in + r_out) / 2.0
        r_in, r_out = (m, r_out) if solid(body, m, ang, z) else (r_in, m)
    return (r_in + r_out) / 2.0


comp, how = BuildArm.ensure_v3_module(design, log)
log('')
log('%s: %s' % (comp.name, how))

occ = None
for i in range(root.occurrences.count):
    if root.occurrences.item(i).component.name == comp.name:
        occ = root.occurrences.item(i)

# ---- 1. the LINKED original, which nothing here may have touched ----
ORIG = {'stator': 25729.996, 'upper': 6473.293, 'lower': 2950.793}
src = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.isReferencedComponent and o.component.name.startswith(BuildArm.MODULE_DOC_NAME):
        src = o
log('')
log('--- the ORIGINAL, linked ---')
if src is None:
    problems.append('the LINKED "%s" is not in this document' % BuildArm.MODULE_DOC_NAME)
else:
    up0, lo0 = BuildArm.module_rotor_occs(src, in_context=True)
    st0 = BuildArm.module_stator_occ(src, in_context=True)
    for what, c, want in (('stator', st0, ORIG['stator']), ('upper', up0, ORIG['upper']),
                          ('lower', lo0, ORIG['lower'])):
        v = c.bRepBodies.item(0).volume * 1000.0
        log('  %-7s V=%10.3f mm3 (was %10.3f)' % (what, v, want))
        if abs(v - want) > 1e-3:
            problems.append('ORIGINAL %s is %.3f mm3, was %.3f' % (what, v, want))
    s0, n0 = BuildArm.module_fastener_occs(src)
    nz = [BuildArm._body_extent(n)[2:] for n in n0]
    log('  %d horn screws, %d thin nuts %s' % (len(s0), len(n0),
        ', '.join('z %.3f..%.3f' % t for t in nz)))
    if len(n0) != 4:
        problems.append('the ORIGINAL has %d M3 nuts, expected 4' % len(n0))

# ---- 2. the v3 rotor pair, the pocket, the wall band ----
up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
lob = lo.bRepBodies.item(0)
upb = up.bRepBodies.item(0)
log('')
log('--- v3, the two rotor parts ---')
for tag, c in (('upper', up), ('lower', lo)):
    r0, r1, z0, z1 = BuildArm._body_extent(c)
    log('  %-6s %-14s r %7.3f..%7.3f z %8.3f..%8.3f V=%10.3f mm3'
        % (tag, c.component.name, r0, r1, z0, z1, c.bRepBodies.item(0).volume * 1000.0))

POCKET_ANG = 0.0
POCKET_R = 13.5                      # inside the hex pocket, clear of the M3 hole
ceil_z = edge_z(lob, POCKET_R, POCKET_ANG, -9.5, -12.0)
mouth_z = edge_z(lob, 16.0, POCKET_ANG, -12.0, -14.0)
top_z = edge_z(lob, POCKET_R, POCKET_ANG, -9.5, -8.0)
log('')
log('--- the nut pocket, measured on the lower part ---')
log('  ceiling z %.4f (asked %.3f), mouth = the part bottom face z %.4f, pocket %.4f deep'
    % (ceil_z, BuildArm.V3_NUT_Z[1], mouth_z, ceil_z - mouth_z))
log('  ceiling band: pocket ceiling %.4f to the part top face %.4f = %.4f mm of material'
    % (ceil_z, top_z, top_z - ceil_z))
if abs(ceil_z - BuildArm.V3_NUT_Z[1]) > 0.02:
    problems.append('the pocket ceiling reads %.4f, asked %.3f' % (ceil_z, BuildArm.V3_NUT_Z[1]))

# the 1.30 mm wall band: between the r 8.1 bore and the Ø3.2 M3 hole at r 11, on the
# 0 deg line, at a z just above the pocket ceiling and just below the top of the bore
log('  the 1.30 mm wall band (M3 hole inner edge r 9.400 to the bore wall r 8.100):')
for z in (-10.55, -10.51, -10.8, -11.0, -11.5):
    inn = solid(lob, 8.2, POCKET_ANG, z)
    out = solid(lob, 9.2, POCKET_ANG, z)
    log('    z %7.3f: r 8.2 %-5s r 9.2 %-5s' % (z, 'solid' if inn else 'void',
                                                'solid' if out else 'void'))
bore_top = edge_z(lob, 7.5, POCKET_ANG, -9.5, -11.5)
log('    the r 8.1 bore tops out at z %.4f, so the band survives z %.4f..%.4f = %.4f mm'
    % (bore_top, ceil_z, bore_top, bore_top - ceil_z))
log('    above z %.4f the M3 hole has the solid web all round it, not a 1.30 wall'
    % bore_top)

# ---- 3. the standard nuts and the M3 bolts ----
nut_occ, bolt_occ = BuildArm.module_v3_fasteners(occ)
if nut_occ is None or bolt_occ is None:
    problems.append('the standard nuts / the M3 bolts are not in the module')
else:
    log('')
    log('--- the four STANDARD ISO 4032 M3 nuts (%s) ---' % nut_occ.component.name)
    nc = nut_occ.component
    nz0 = nz1 = None
    for j in range(4):
        b = nc.bRepBodies.item(j)
        rs, zs = [], []
        for k in range(b.vertices.count):
            p = b.vertices.item(k).geometry
            rs.append(math.hypot(p.x, p.y) * 10.0); zs.append(p.z * 10.0)
        nz0, nz1 = min(zs), max(zs)
        log('  %-12s r %7.3f..%7.3f z %8.3f..%8.3f V=%8.4f mm3'
            % (b.name, min(rs), max(rs), nz0, nz1, b.volume * 1000.0))
    flat_r = BuildArm.V3_M3_R - BuildArm.V3_NUT_STD_AF / 2.0
    log('  thickness %.4f, top on the pocket ceiling %.4f, %.4f left under it inside the '
        'mouth %.4f' % (nz1 - nz0, ceil_z, nz0 - mouth_z, mouth_z))
    log('  inner FLAT r %.3f, XL330 horn disk r 8.000 -> %.3f clear' % (flat_r, flat_r - 8.0))
    if abs((nz1 - nz0) - 2.4) > 0.02:
        problems.append('the nut is %.4f thick, ISO 4032 M3 is 2.4' % (nz1 - nz0))
    if abs(nz1 - ceil_z) > 0.02:
        problems.append('the nut tops out at %.4f, the pocket ceiling is %.4f' % (nz1, ceil_z))

    log('')
    log('--- the four M3x%g bolts (%s) ---' % (BuildArm.V3_BOLT_L, bolt_occ.component.name))
    bc = bolt_occ.component
    tip = head_top = None
    for j in range(4):
        b = bc.bRepBodies.item(j)
        bb = b.boundingBox
        tip, head_top = bb.minPoint.z * 10.0, bb.maxPoint.z * 10.0
        log('  %-14s z %8.3f..%8.3f  V=%9.4f mm3'
            % (b.name, tip, head_top, b.volume * 1000.0))
    seat = armgen.PAD_T
    log('  head seated on the link rotor pad outer face z %.3f (armgen.PAD_T); under-head '
        'length %.3f = M3x%g; head %.3f high, top z %.3f'
        % (seat, seat - tip, BuildArm.V3_BOLT_L, head_top - seat, head_top))
    log('  TIP z %.3f' % tip)
    log('  THREAD ENGAGEMENT in the nut: nut %.3f..%.3f, tip %.3f -> %.3f mm (need >= 1.4)'
        % (nz0, nz1, tip, nz1 - tip))
    if nz1 - tip < 1.4 - 1e-6:
        problems.append('thread engagement is %.3f mm, need 1.4' % (nz1 - tip))

    # what the tip has under it, per angle, inside the module
    mine = []

    def walk(o):
        for b in o.bRepBodies:
            mine.append((o.component.name, b))
        for j in range(o.childOccurrences.count):
            walk(o.childOccurrences.item(j))
    walk(occ)
    mine = [(n, b) for n, b in mine if n not in (bolt_occ.component.name,
                                                 nut_occ.component.name)]
    log('  CLEARANCE below the tip, per bolt (module bodies only):')
    worst = 99.0
    for a in BuildArm.V3_M3_ANG:
        x, y = BuildArm._polar(BuildArm.V3_M3_R, a)
        first = None
        z = tip
        while z > -26.0:
            z -= 0.02
            p = ac.Point3D.create(x / 10.0, y / 10.0, z / 10.0)
            for nm, b in mine:
                if b.pointContainment(p) == af.PointContainment.PointInsidePointContainment:
                    first = (z, nm); break
            if first:
                break
        gap = tip - first[0] if first else float('inf')
        worst = min(worst, gap)
        log('    %5.1f deg: first solid at z %.3f (%s) -> %.3f mm under the tip'
            % (a, first[0], first[1], gap) if first else
            '    %5.1f deg: nothing under the tip for 14 mm' % a)
    log('  worst clearance under a tip: %.3f mm (need >= 1.0)' % worst)
    if worst < 1.0:
        problems.append('the tip clears the XL330 by %.3f mm, need 1.0' % worst)

log('')
if problems:
    log('EXO-231 PROBLEMS:')
    for p in problems:
        log('  - %s' % p)
    raise RuntimeError('%d problem(s)' % len(problems))
log('EXO-231 verify: every number above reads back.')
