"""EXO-240 evidence: the O5.0 horn screw head column opened THROUGH the upper rotor part,
and the hex nut pocket tightened from 6.0 to 5.8 across flats.

Runs `BuildArm.ensure_v3_module` - which finds the EXO-238 module in the document WRONG
against the new `V3_PROBES` (blind pocket, 6.0 AF pocket) and re-makes it from the LINKED
original - then measures, all on the SOLID in the module frame:

  1. the LINKED `sensor module 2` is untouched (three body volumes to 1e-3 mm3);
  2. both rotor part volumes against what the two changes predict;
  3. THE COLUMN: void on the horn screw axis at every height from the z -9 mating face to
     the rotor face z 0, its width at each of those heights, the walls it leaves to the
     M3 hole, to the nearest register peg and to the central boss, and the M2 screw head
     sitting in it - how far a driver has to reach and how much room it has;
  4. THE POCKET: both flats measured, so across-flats is read off the solid; the clearance
     to the standard nut per side; the clearance of the pocket's own inner flat to the
     XL330 horn disk; the ceiling, depth and the 1.30 mm wall band; the floor over the
     EXO-238 sockets on the point of deepest bite into both footprints;
  5. 4-fold symmetry, so the four assembly orientations stay identical.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/verify_exo240.py --timeout 900
"""
import math
import sys
import time
import importlib

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
import BuildArm
importlib.reload(armgen)
importlib.reload(BuildArm)
import adsk.core as ac
import adsk.fusion as af

root = design.rootComponent
INSIDE = (af.PointContainment.PointInsidePointContainment,
          af.PointContainment.PointOnPointContainment)
problems = []

# EXO-238/EXO-239, the state this change starts from.
WAS = {'upper': 7207.743, 'lower': 2487.889}
WAS_HEAD = (len(BuildArm.V3_HORN_ANG) * math.pi * BuildArm.V3_HEAD_R ** 2 * 2.0)
WAS_HEX = 238.839               # measured, EXO-233/EXO-239 (6.0 across flats)
N_HORN = len(BuildArm.V3_HORN_ANG)
D_HEAD = N_HORN * math.pi * BuildArm.V3_HEAD_R ** 2 * (0.0 - BuildArm.V3_UPPER_Z[0])
A_HEX = math.sqrt(3.0) / 2.0 * BuildArm.V3_NUT_AF ** 2
A_HOLE = math.pi * BuildArm.V3_M3_HOLE_R ** 2
DEEP = BuildArm.V3_NUT_Z[1] - BuildArm.V3_LOWER_Z[0]      # -10.6 - -13.2 = 2.6
D_HEX = 4 * (A_HEX - A_HOLE) * DEEP


def solid_xy(body, x, y, z):
    return body.pointContainment(ac.Point3D.create(x / 10.0, y / 10.0, z / 10.0)) in INSIDE


def solid(body, r, ang, z):
    x, y = BuildArm._polar(r, ang)
    return solid_xy(body, x, y, z)


def bisect(f, a_in, a_out):
    """`f(a_in)` true, `f(a_out)` false: the boundary between them."""
    for _ in range(60):
        m = (a_in + a_out) / 2.0
        a_in, a_out = (m, a_out) if f(m) else (a_in, m)
    return (a_in + a_out) / 2.0


def edge_along(body, p_in, p_out, z):
    x0, y0 = p_in
    x1, y1 = p_out
    t = bisect(lambda t: solid_xy(body, x0 + t * (x1 - x0), y0 + t * (y1 - y0), z), 0.0, 1.0)
    return ((x0 + t * (x1 - x0), y0 + t * (y1 - y0)), t * math.hypot(x1 - x0, y1 - y0))


comp, how = BuildArm.ensure_v3_module(design, log)
log('')
log('%s: %s' % (comp.name, how))

occ = None
for i in range(root.occurrences.count):
    if root.occurrences.item(i).component.name == comp.name:
        occ = root.occurrences.item(i)
if occ is None:
    raise RuntimeError('%s has no occurrence at the root' % comp.name)

# ---- 1. the LINKED original, untouched ----
ORIG = {'stator': 25729.996, 'upper': 6473.293, 'lower': 2950.793}
src = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.isReferencedComponent and o.component.name.startswith(BuildArm.MODULE_DOC_NAME):
        src = o
log('')
log('--- the ORIGINAL, still LINKED ---')
if src is None:
    problems.append('the LINKED "%s" is not in this document' % BuildArm.MODULE_DOC_NAME)
else:
    up0, lo0 = BuildArm.module_rotor_occs(src, in_context=True)
    st0 = BuildArm.module_stator_occ(src, in_context=True)
    for what, c, want in (('stator', st0, ORIG['stator']), ('upper', up0, ORIG['upper']),
                          ('lower', lo0, ORIG['lower'])):
        v = c.bRepBodies.item(0).volume * 1000.0
        r0, r1, z0, z1 = BuildArm._body_extent(c)
        log('  %-7s V=%10.3f mm3 (was %10.3f)  r %6.3f..%6.3f z %8.3f..%8.3f'
            % (what, v, want, r0, r1, z0, z1))
        if abs(v - want) > 1e-3:
            problems.append('ORIGINAL %s is %.3f mm3, was %.3f' % (what, v, want))

# ---- 2. the two v3 rotor parts ----
up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
upb, lob = up.bRepBodies.item(0), lo.bRepBodies.item(0)
log('')
log('--- v3 after EXO-240: the two rotor parts ---')
vols = {}
for tag, c in (('upper', up), ('lower', lo)):
    r0, r1, z0, z1 = BuildArm._body_extent(c)
    v = c.bRepBodies.item(0).volume * 1000.0
    vols[tag] = v
    log('  %-6s V=%10.3f mm3  r %6.3f..%6.3f z %8.3f..%8.3f' % (tag, v, r0, r1, z0, z1))
want_up = WAS['upper'] - (D_HEAD - WAS_HEAD)
want_lo = WAS['lower'] + WAS_HEX - D_HEX
log('  upper: %.3f (EXO-238) - (%.3f through, z %.1f..0 - %.3f blind, z %.1f..%.1f) = %.3f'
    % (WAS['upper'], D_HEAD, BuildArm.V3_UPPER_Z[0], WAS_HEAD, -9.0, -7.0, want_up))
log('  lower: %.3f (EXO-238) + %.3f (the 6.0 AF pockets) - %.3f (%.1f AF: hex area %.4f '
    'less the O3.2 hole %.4f, %.1f deep, x4) = %.3f'
    % (WAS['lower'], WAS_HEX, D_HEX, BuildArm.V3_NUT_AF, A_HEX, A_HOLE, DEEP, want_lo))
if abs(vols['upper'] - want_up) > 1e-3:
    problems.append('the upper part is %.3f mm3, the through column predicts %.3f'
                    % (vols['upper'], want_up))
log('  lower residual vs the analytic pocket: %+.3f mm3 (the 6.0 pocket overlapped the '
    'r 8.100 horn bore, the 5.8 one is tangent to it)' % (vols['lower'] - want_lo))
if abs(vols['lower'] - want_lo) > 1.0:
    problems.append('the lower part is %.3f mm3, the 5.8 AF pocket predicts %.3f'
                    % (vols['lower'], want_lo))

# ---- 3. THE COLUMN ----
log('')
log('--- THE HORN SCREW HEAD COLUMN, O%.1f at r %.3f, one per angle ---'
    % (2 * BuildArm.V3_HEAD_R, BuildArm.V3_HORN_R))
ZS = (-8.95, -8.5, -7.05, -6.95, -5.05, -4.95, -4.0, -2.0, -0.5, -0.05)
for a in BuildArm.V3_HORN_ANG:
    got = ''.join('S' if solid(upb, BuildArm.V3_HORN_R, a, z) else '.' for z in ZS)
    log('  %6.1f deg  axis at z %s: %s'
        % (a, '/'.join('%g' % z for z in ZS), got))
    if 'S' in got:
        problems.append('the column at %.1f deg is still blocked somewhere: %s' % (a, got))
log('  (all void = no ceiling anywhere between the z %.1f mating face and the rotor face 0)'
    % BuildArm.V3_UPPER_Z[0])

A = BuildArm.V3_HORN_ANG[0]                     # 45 deg
C = BuildArm._polar(BuildArm.V3_HORN_R, A)
widths = []
for z in (-8.5, -6.0, -4.0, -2.0, -0.5):
    r_in = bisect(lambda r: solid(upb, r, A, z), 0.5, BuildArm.V3_HORN_R)
    r_out = bisect(lambda r: solid(upb, r, A, z), 13.0, BuildArm.V3_HORN_R)
    widths.append(r_out - r_in)
    log('  z %7.3f: void r %.4f..%.4f -> width %.4f, centre r %.4f'
        % (z, r_in, r_out, r_out - r_in, (r_in + r_out) / 2.0))
log('  widest %.4f, narrowest %.4f over the whole height (O%.1f asked)'
    % (max(widths), min(widths), 2 * BuildArm.V3_HEAD_R))
if abs(max(widths) - 2 * BuildArm.V3_HEAD_R) > 0.02 or \
        abs(min(widths) - 2 * BuildArm.V3_HEAD_R) > 0.02:
    problems.append('the column is %.4f..%.4f wide, O%.1f asked'
                    % (min(widths), max(widths), 2 * BuildArm.V3_HEAD_R))

# the walls the column leaves, each off two measured edges
M = BuildArm._polar(BuildArm.V3_M3_R, 0.0)
d_CM = math.hypot(C[0] - M[0], C[1] - M[1])
MIDCM = ((C[0] + M[0]) / 2.0, (C[1] + M[1]) / 2.0)
(_, e_col) = edge_along(upb, MIDCM, C, -4.0)
(_, e_m3) = edge_along(upb, MIDCM, M, -4.0)
log('  column axis r %.3f %.1f deg to M3 axis r %.1f 0 deg: %.4f apart; column R %.4f, '
    'M3 hole R %.4f -> WALL %.4f mm'
    % (BuildArm.V3_HORN_R, A, BuildArm.V3_M3_R, d_CM, d_CM / 2.0 - e_col,
       d_CM / 2.0 - e_m3, e_col + e_m3))
wall_col_m3 = e_col + e_m3
P = BuildArm._polar(BuildArm.V3_PEG_AT, BuildArm.V3_PEG_ANG[0])
d_CP = math.hypot(C[0] - P[0], C[1] - P[1])
(_, e_peg) = edge_along(upb, P, C, -9.6)        # peg axis -> peg flank, at peg height
(_, e_col2) = edge_along(upb, MIDCM, C, -8.5)
r_col = d_CM / 2.0 - e_col2
log('  column axis to the nearest register peg axis (r %.1f %.1f deg): %.4f apart; peg R '
    '%.4f, column R %.4f -> WALL %.4f mm (they never share a height, so this is the plan gap)'
    % (BuildArm.V3_PEG_AT, BuildArm.V3_PEG_ANG[0], d_CP, e_peg, r_col,
       d_CP - e_peg - r_col))
wall_col_peg = d_CP - e_peg - r_col
r_boss = bisect(lambda r: solid(upb, r, A, -4.0), 0.0, BuildArm.V3_HORN_R)
log('  the central boss the four columns surround: solid from the axis out to r %.4f at '
    'z -4.0 (unbroken - the column\'s inner flank IS its wall)' % r_boss)
for what, w in (('column to M3 hole', wall_col_m3), ('column to peg', wall_col_peg)):
    if w < 1.2:
        problems.append('the %s wall is %.4f mm, under 1.2' % (what, w))

# the screw in the column
screws, _old = BuildArm.module_fastener_occs(occ)
log('  %d horn screws found' % len(screws))
for s in screws:
    r0, r1, z0, z1 = BuildArm._body_extent(s)
    head_r = r1 - BuildArm.V3_HORN_R
    log('  M2 screw: r %.4f..%.4f (head R %.4f off its own axis), z %.4f..%.4f'
        % (r0, r1, head_r, z0, z1))
    log('  -> head radial clearance in the column %.4f mm; the head top z %.4f is %.4f mm '
        'below the rotor face, and that is now OPEN sky: a driver reaches it from the top'
        % (BuildArm.V3_HEAD_R - head_r, z1, 0.0 - z1))
    if z1 > 0.0:
        problems.append('the screw head stands %.4f above the rotor face' % z1)

# ---- 4. THE POCKET ----
log('')
log('--- THE HEX NUT POCKET at %.1f across flats (EXO-240: was 6.0) ---' % BuildArm.V3_NUT_AF)
Y = 1.6            # off the pocket's own centre line, clear of the r 8.100 horn bore
x_inner = bisect(lambda x: solid_xy(lob, x, Y, -12.0), 8.0, 8.4)
x_outer = bisect(lambda x: not solid_xy(lob, x, Y, -12.0), 13.5, 14.3)
log('  at y %.3f, z -12.0: inner FLAT x %.4f, outer FLAT x %.4f -> ACROSS FLATS %.4f mm'
    % (Y, x_inner, x_outer, x_outer - x_inner))
if abs((x_outer - x_inner) - BuildArm.V3_NUT_AF) > 0.02:
    problems.append('the pocket measures %.4f across flats, asked %.2f'
                    % (x_outer - x_inner, BuildArm.V3_NUT_AF))
x_bore = bisect(lambda x: solid_xy(lob, x, Y, -12.0), 8.0, 7.7)
log('  the wall left between the pocket flat and the r %.3f horn bore, at y %.3f: '
    'x %.4f..%.4f = %.4f mm (it tapers to 0 on the centre line, where the flat is '
    'TANGENT to the bore - at 6.0 across flats the flat cut 0.100 INTO the bore)'
    % (math.hypot(x_bore, Y), Y, x_bore, x_inner, x_inner - x_bore))
flat_nut = BuildArm.V3_M3_R - BuildArm.V3_NUT_STD_AF / 2.0
log('  nut %.1f AF: inner flat r %.3f. Pocket flat r %.4f -> %.4f mm per side on the nut '
    '(6.0 gave %.4f)' % (BuildArm.V3_NUT_STD_AF, flat_nut, x_inner, flat_nut - x_inner,
                         flat_nut - 8.0))

# the XL330 horn disk, MEASURED, not quoted: the r 8.1 bore in the lower part is its
# clearance bore, and the nut pocket's inner flat is what has to stay off it.
nut_occ, bolt_occ = BuildArm.module_v3_fasteners(occ)
skip = set([up.component.name, lo.component.name] + [s.component.name for s in screws])
if nut_occ is not None:
    skip.add(nut_occ.component.name)
if bolt_occ is not None:
    skip.add(bolt_occ.component.name)

everything = []


def walk(o):
    if o.component.name not in skip:
        for b in o.bRepBodies:
            everything.append((o.component.name, b))
    for j in range(o.childOccurrences.count):
        walk(o.childOccurrences.item(j))


walk(occ)
horn_r = None
for name, b in everything:
    hits = []
    for z in (-10.6, -11.0, -12.0, -13.0, -13.19):
        start = None
        for rr in (7.9, 7.0, 5.0, 3.0, 1.0):
            if solid(b, rr, 0.0, z):
                start = rr
                break
        if start is None:
            continue
        hits.append((z, bisect(lambda r: solid(b, r, 0.0, z), start, 20.0)))
    if hits:
        log('  %s reaches into the lower part\'s bore: %s' % (
            name, ', '.join('z %.3f -> r %.4f' % h for h in hits)))
        r = max(h[1] for h in hits)
        if horn_r is None or r > horn_r:
            horn_r = r
if horn_r is None:
    log('  nothing but the rotor parts and the fasteners is inside the r 8.1 bore over the '
        'nut pocket band - the horn disk does not reach up here')
else:
    log('  XL330 HORN DISK r %.4f -> the pocket inner flat r %.4f clears it by %.4f mm '
        '(0.0000 at 6.0 across flats, where the flat sat ON r 8.000); the nut itself '
        'clears it by %.4f' % (horn_r, x_inner, x_inner - horn_r, flat_nut - horn_r))
    if x_inner < horn_r:
        problems.append('the pocket flat r %.4f is inside the horn disk r %.4f'
                        % (x_inner, horn_r))

POCKET_ANG = 0.0
ceil_z = bisect(lambda z: not solid(lob, 13.0, POCKET_ANG, z), -12.0, -10.0)
mouth_z = bisect(lambda z: solid(lob, 13.0, 45.0, z), -12.0, -13.6)
z_top = bisect(lambda z: solid(lob, 13.0, POCKET_ANG, z), -10.0, -8.5)
log('  ceiling z %.4f (asked %.3f), mouth = the part bottom face %.3f -> %.4f deep; '
    'ceiling to the part top face %.4f = %.4f mm of material'
    % (ceil_z, BuildArm.V3_NUT_Z[1], mouth_z, ceil_z - mouth_z, z_top, z_top - ceil_z))
if abs(ceil_z - BuildArm.V3_NUT_Z[1]) > 0.02:
    problems.append('the pocket ceiling reads %.4f, asked %.3f'
                    % (ceil_z, BuildArm.V3_NUT_Z[1]))
bore_top = bisect(lambda z: not solid(lob, 7.5, POCKET_ANG, z), -12.0, -9.2)
log('  the 1.30 mm wall band (M3 hole inner edge r 9.400 to the bore wall r 8.100):')
for z in (-10.55, -10.51, -10.65, -10.8, -11.5):
    log('    z %7.3f: r 8.2 %-5s r 9.2 %-5s'
        % (z, 'solid' if solid(lob, 8.2, POCKET_ANG, z) else 'void',
           'solid' if solid(lob, 9.2, POCKET_ANG, z) else 'void'))
log('    the bore tops out at z %.4f, so the band still survives z %.4f..%.4f = %.4f mm '
    '(EXO-233 read the same 0.1000 at 6.0 across flats)'
    % (bore_top, ceil_z, bore_top, bore_top - ceil_z))

# the nut and the bolt, unchanged by this issue but re-read
if nut_occ is not None:
    nz0 = nz1 = None
    for j in range(nut_occ.component.bRepBodies.count):
        b = nut_occ.component.bRepBodies.item(j)
        zs = [b.vertices.item(k).geometry.z * 10.0 for k in range(b.vertices.count)]
        rs = [math.hypot(b.vertices.item(k).geometry.x, b.vertices.item(k).geometry.y) * 10.0
              for k in range(b.vertices.count)]
        nz0, nz1 = min(zs), max(zs)
    log('  the standard nut: %.4f thick, z %.4f..%.4f, r %.4f..%.4f, V %.4f mm3 - top ON '
        'the ceiling %.4f, %.4f left under it inside the mouth'
        % (nz1 - nz0, nz0, nz1, min(rs), max(rs),
           nut_occ.component.bRepBodies.item(0).volume * 1000.0, ceil_z, nz0 - mouth_z))
    if abs(nz1 - ceil_z) > 0.02:
        problems.append('the nut tops out at %.4f, the ceiling is %.4f' % (nz1, ceil_z))

# the floor over the EXO-238 sockets, on the new point
FX, FY = 11.0, 3.185
rc = BuildArm.V3_NUT_AF / math.sqrt(3.0)
d_soc = math.hypot(FX - P[0], FY - P[1])
log('  floor probe xy (%.4f, %.4f) = r %.4f %.4f deg: inside the %.1f AF hex '
    '(circumradius %.4f) and %.4f off the socket axis (R %.3f)'
    % (FX, FY, math.hypot(FX, FY), math.degrees(math.atan2(FY, FX)), BuildArm.V3_NUT_AF,
       rc, d_soc, BuildArm.V3_SOCKET_R))
if not solid_xy(lob, FX, FY, -10.45):
    problems.append('there is no floor at all between the socket and the nut pocket')
    log('  THE FLOOR IS GONE: z -10.45 reads void')
else:
    z_sock_bot = bisect(lambda z: solid_xy(lob, FX, FY, z), -10.45, -10.00)
    z_hex_top = bisect(lambda z: solid_xy(lob, FX, FY, z), -10.45, -10.90)
    floor = z_sock_bot - z_hex_top
    log('  socket bottom z %.4f, hex pocket ceiling z %.4f -> FLOOR %.4f mm'
        % (z_sock_bot, z_hex_top, floor))
    if floor < 0.25:
        problems.append('the floor is %.4f mm' % floor)
log('  the pocket corner (11.000, %.4f) now pokes %.4f mm into the socket footprint in '
    'plan (0.4152 at 6.0 across flats)'
    % (rc, BuildArm.V3_SOCKET_R - math.hypot(P[0] - 11.0, P[1] - rc)))

# ---- 5. 4-fold symmetry ----
NOTCH = 'r >= 14.70 and z >= -1.55'


def scan(tag, body, z0, z1, r_max, dr, dz, da, budget):
    t0 = time.time()
    n = 0
    bad_notch, bad_other = [], []
    r = 0.13
    while r <= r_max + 1e-9:
        z = z0 + dz / 2.0
        while z <= z1 - dz / 2.0 + 1e-9:
            a = 0.0
            while a < 90.0 - 1e-9:
                v = [solid(body, r, a + 90.0 * k, z) for k in range(4)]
                n += 1
                if len(set(v)) != 1:
                    hit = (r, z, a, ''.join('S' if x else '.' for x in v))
                    if r >= 14.70 and z >= -1.55:
                        bad_notch.append(hit)
                    else:
                        bad_other.append(hit)
                a += da
            z += dz
        if time.time() - t0 > budget:
            log('  %s: BUDGET %gs spent at r %.2f - grid incomplete' % (tag, budget, r))
            break
        r += dr
    log('  %-6s %6d quadruples in %5.1fs (dr %.2f, dz %.2f, dang %.1f deg, r<=%.1f, '
        'z %.2f..%.2f)' % (tag, n, time.time() - t0, dr, dz, da, r_max, z0, z1))
    return n, bad_notch, bad_other


log('')
log('--- 4-FOLD SYMMETRY, swept grid (%s = the index notch, expected) ---' % NOTCH)
n_up, notch_up, other_up = scan('upper', upb, BuildArm.V3_PEG_Z[0], 0.0, 17.9,
                                0.5, 0.5, 5.0, 120.0)
n_lo, notch_lo, other_lo = scan('lower', lob, -13.2, -9.0, 17.4, 0.5, 0.5, 5.0, 120.0)
log('  upper: %d asymmetric quadruples inside the notch box, %d anywhere else'
    % (len(notch_up), len(other_up)))
log('  lower: %d asymmetric quadruples inside the notch box, %d anywhere else'
    % (len(notch_lo), len(other_lo)))
for tag, hits in (('upper', other_up), ('lower', other_lo)):
    for r, z, a, pat in hits[:20]:
        log('    %s ASYMMETRIC r %6.3f z %7.3f ang %5.1f  %s' % (tag, r, z, a, pat))
if other_up:
    problems.append('the upper part is asymmetric at %d grid points OUTSIDE the notch box'
                    % len(other_up))
if other_lo:
    problems.append('the lower part is asymmetric at %d grid points' % len(other_lo))
if not notch_up:
    problems.append('the index notch did not show up in the sweep - is it still there?')

log('')
if problems:
    log('PROBLEMS (%d):' % len(problems))
    for p in problems:
        log('  * %s' % p)
    raise RuntimeError('EXO-240 verification failed: %d problem(s)' % len(problems))
log('EXO-240: all checks pass.')
