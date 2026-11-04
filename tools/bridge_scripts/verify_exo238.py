"""EXO-238 evidence: EIGHT Ø3.0 register pegs on the UPPER rotor part, Ø3.3 sockets below.

Runs `BuildArm.ensure_v3_module` - which finds the peg-less EXO-236 module in the document
WRONG against the new `V3_PROBES` and re-makes it from the linked original - then measures,
all on the SOLID in the module frame:

  1. the LINKED `sensor module 2` is untouched (three body volumes to 1e-3 mm3);
  2. both rotor part volumes against the 8 pegs / 8 sockets they predict;
  3. the peg itself: root face, tip, height, and the axial clearance to the socket bottom;
  4. THE WALLS, each read off two measured edges:
       * peg (upper) to the Ø3.2 M3 hole, and socket (lower) to the same hole;
       * peg to the upper part's r 15 rim, socket to the lower part's r 15 rim;
       * the FLOOR between the socket bottom and the hex nut pocket ceiling, on a point
         that lies inside both footprints;
  5. ANGULAR SYMMETRY, the EXO-236 test unchanged except that the upper part's sweep now
     runs down to the peg tips: containment at ang, +90, +180, +270 must agree everywhere
     but the index notch, so all four assembly orientations stay identical.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/verify_exo238.py --timeout 900
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

# EXO-236/EXO-237, the state this change starts from.
WAS = {'upper': 7139.884, 'lower': 2576.840}
N = len(BuildArm.V3_PEG_ANG)
D_PEG = N * math.pi * BuildArm.V3_PEG_R ** 2 * (
    BuildArm.V3_LOWER_Z[1] - BuildArm.V3_PEG_Z[0])
D_SOCKET = N * math.pi * BuildArm.V3_SOCKET_R ** 2 * (
    BuildArm.V3_LOWER_Z[1] - BuildArm.V3_SOCKET_Z[0])


def solid_xy(body, x, y, z):
    return body.pointContainment(ac.Point3D.create(x / 10.0, y / 10.0, z / 10.0)) in INSIDE


def solid(body, r, ang, z):
    x, y = BuildArm._polar(r, ang)
    return solid_xy(body, x, y, z)


def bisect(f, a_in, a_out):
    """`f(a_in)` is solid, `f(a_out)` is void: the boundary between them."""
    for _ in range(60):
        m = (a_in + a_out) / 2.0
        a_in, a_out = (m, a_out) if f(m) else (a_in, m)
    return (a_in + a_out) / 2.0


def edge_along(body, p_in, p_out, z):
    """Walk the straight line p_in -> p_out at height z; return (point, distance from p_in)
    of the solid/void boundary. `p_in` must be solid and `p_out` void, both at z."""
    x0, y0 = p_in
    x1, y1 = p_out
    t = bisect(lambda t: solid_xy(body, x0 + t * (x1 - x0), y0 + t * (y1 - y0), z), 0.0, 1.0)
    return ((x0 + t * (x1 - x0), y0 + t * (y1 - y0)),
            t * math.hypot(x1 - x0, y1 - y0))


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
log('--- v3 after EXO-238: the two rotor parts ---')
vols = {}
for tag, c in (('upper', up), ('lower', lo)):
    r0, r1, z0, z1 = BuildArm._body_extent(c)
    v = c.bRepBodies.item(0).volume * 1000.0
    vols[tag] = v
    log('  %-6s V=%10.3f mm3  r %6.3f..%6.3f z %8.3f..%8.3f' % (tag, v, r0, r1, z0, z1))
log('  upper: %.3f (EXO-236, flat) + %.3f (%d x O%.1f x %.1f pegs) = %.3f'
    % (WAS['upper'], D_PEG, N, 2 * BuildArm.V3_PEG_R,
       BuildArm.V3_LOWER_Z[1] - BuildArm.V3_PEG_Z[0], WAS['upper'] + D_PEG))
log('  lower: %.3f (EXO-236, flat) - %.3f (%d x O%.1f x %.1f sockets) = %.3f'
    % (WAS['lower'], D_SOCKET, N, 2 * BuildArm.V3_SOCKET_R,
       BuildArm.V3_LOWER_Z[1] - BuildArm.V3_SOCKET_Z[0], WAS['lower'] - D_SOCKET))
for tag, want in (('upper', WAS['upper'] + D_PEG), ('lower', WAS['lower'] - D_SOCKET)):
    if abs(vols[tag] - want) > 1e-3:
        problems.append('%s part is %.3f mm3, the eight pegs/sockets predict %.3f'
                        % (tag, vols[tag], want))

# ---- 3. the peg and the socket, end to end ----
log('')
log('--- the register pegs (upper) and sockets (lower), one per site ---')
PEG_AT, PEG_R, SOCK_R = (BuildArm.V3_PEG_AT, BuildArm.V3_PEG_R, BuildArm.V3_SOCKET_R)
tips, floors = [], []
for a in BuildArm.V3_PEG_ANG:
    px, py = BuildArm._polar(PEG_AT, a)
    if not solid_xy(upb, px, py, -9.5):
        problems.append('no peg on the upper part at %.1f deg' % a)
        log('  %6.1f deg: NO PEG' % a)
        continue
    z_tip = bisect(lambda z: solid_xy(upb, px, py, z), -9.5, -11.5)
    if solid_xy(lob, px, py, -9.5):
        problems.append('no socket in the lower part at %.1f deg' % a)
        log('  %6.1f deg: NO SOCKET' % a)
        continue
    z_bot = bisect(lambda z: not solid_xy(lob, px, py, z), -9.5, -11.5)
    tips.append(z_tip)
    floors.append(z_bot)
    # the peg's own radius and the socket's, read outward along the same radial line
    r_peg = bisect(lambda r: solid(upb, r, a, -9.6), PEG_AT, PEG_AT + 3.0) - PEG_AT
    r_sock = bisect(lambda r: not solid(lob, r, a, -9.6), PEG_AT, PEG_AT + 2.0) - PEG_AT
    log('  %6.1f deg: peg z -9.0000..%8.4f (%.4f tall, R %.4f); socket floor %8.4f '
        '(%.4f deep, R %.4f); axial clearance %.4f, radial %.4f'
        % (a, z_tip, -9.0 - z_tip, r_peg, z_bot, -9.0 - z_bot, r_sock,
           z_tip - z_bot, r_sock - r_peg))

if tips:
    log('  all %d: peg tips z %.4f..%.4f, socket floors z %.4f..%.4f, smallest axial '
        'clearance %.4f' % (len(tips), min(tips), max(tips), min(floors), max(floors),
                            min(t - f for t, f in zip(tips, floors))))

# ---- 4. THE WALLS ----
log('')
log('--- THE WALLS, each from two edges read off the solid ---')
A = BuildArm.V3_PEG_ANG[0]                       # 22.5 deg, the site next to the M3 at 0
P = BuildArm._polar(PEG_AT, A)
M = BuildArm._polar(BuildArm.V3_M3_R, 0.0)
d_PM = math.hypot(P[0] - M[0], P[1] - M[1])
log('  peg axis r %.3f %.1f deg and M3 axis r %.3f 0.0 deg are %.4f apart'
    % (PEG_AT, A, BuildArm.V3_M3_R, d_PM))

MID = ((P[0] + M[0]) / 2.0, (P[1] + M[1]) / 2.0)  # solid in BOTH parts: between the two

# UPPER. The peg and the M3 hole never share a height - the hole stops AT the mating face
# z -9 and the peg starts there - so each flank is read on its own plane: the peg's at
# z -9.6 (mid peg), the M3 hole's at z -8.5 (in the web). Both are prismatic in z, so the
# gap between the two cylinders is exactly the difference of the two plan distances.
(_, d_peg_side) = edge_along(upb, P, M, -9.6)     # peg axis -> peg flank
(_, d_hole_up) = edge_along(upb, MID, M, -8.5)    # midpoint -> M3 hole flank
wall_peg_m3 = d_PM / 2.0 + d_hole_up - d_peg_side
log('  UPPER  peg R %.4f, M3 hole R %.4f  ->  peg-to-M3-hole wall %.4f mm'
    % (d_peg_side, d_PM / 2.0 - d_hole_up, wall_peg_m3))

# LOWER. Both flanks live at z -9.6, so one walk each way off the midpoint does it.
(_, e_sock) = edge_along(lob, MID, P, -9.6)       # midpoint -> socket flank
(_, e_hole) = edge_along(lob, MID, M, -9.6)       # midpoint -> M3 hole flank
wall_sock_m3 = e_sock + e_hole
log('  LOWER  socket R %.4f, M3 hole R %.4f  ->  socket-to-M3-hole wall %.4f mm'
    % (d_PM / 2.0 - e_sock, d_PM / 2.0 - e_hole, wall_sock_m3))

for what, w in (('peg to M3 hole', wall_peg_m3), ('socket to M3 hole', wall_sock_m3)):
    if w < 1.2:
        problems.append('the %s wall is %.4f mm, under the 1.2 mm Esteban set' % (what, w))

# the rims
rim_up = bisect(lambda r: solid(upb, r, A, -8.5), PEG_AT + 2.0, 30.0)
peg_out = bisect(lambda r: solid(upb, r, A, -9.6), PEG_AT, 30.0)
log('  UPPER  rim r %.4f at z -8.5; peg outer edge r %.4f  ->  peg-to-rim wall %.4f mm '
    '(the face annulus left outside the peg root)'
    % (rim_up, peg_out, rim_up - peg_out))
rim_lo = bisect(lambda r: solid(lob, r, A, -9.6), PEG_AT + 2.0, 30.0)
sock_out = bisect(lambda r: not solid(lob, r, A, -9.6), PEG_AT, PEG_AT + 2.0)
log('  LOWER  rim r %.4f at z -9.6; socket outer edge r %.4f  ->  socket-to-rim wall %.4f mm'
    % (rim_lo, sock_out, rim_lo - sock_out))

# the floor between the socket bottom and the hex nut pocket ceiling
FX, FY = 11.0, 3.40            # inside the hex pocket at r 11 / 0 deg AND inside the socket
d_hex = math.hypot(FX - 11.0, FY)
d_soc = math.hypot(FX - P[0], FY - P[1])
log('  floor probe xy (%.4f, %.4f) = r %.4f %.4f deg: %.4f off the hex pocket centre '
    '(circumradius %.4f) and %.4f off the socket axis (R %.3f) - inside both'
    % (FX, FY, math.hypot(FX, FY), math.degrees(math.atan2(FY, FX)), d_hex,
       BuildArm.V3_NUT_AF / math.sqrt(3.0), d_soc, SOCK_R))
if not solid_xy(lob, FX, FY, -10.45):
    problems.append('there is no floor at all between the socket and the nut pocket')
    log('  THE FLOOR IS GONE: z -10.45 reads void')
else:
    z_sock_bot = bisect(lambda z: solid_xy(lob, FX, FY, z), -10.45, -10.00)
    z_hex_top = bisect(lambda z: solid_xy(lob, FX, FY, z), -10.45, -10.90)
    log('  socket bottom z %.4f, hex nut pocket ceiling z %.4f  ->  FLOOR %.4f mm'
        % (z_sock_bot, z_hex_top, z_sock_bot - z_hex_top))
    floor = z_sock_bot - z_hex_top
    if floor < 0.25:
        problems.append('the floor is %.4f mm' % floor)
# how far the hex pocket corner reaches into the socket footprint, in plan
log('  the hex pocket corner pokes %.4f mm into the socket footprint in plan'
    % (SOCK_R - math.hypot(P[0] - 11.0, P[1] - BuildArm.V3_NUT_AF / math.sqrt(3.0))))

# ---- 5a. the features, probed at all four of their own angles ----
log('')
log('--- ANGULAR SYMMETRY, feature by feature (all four angles must agree) ---')
EIGHT = tuple(BuildArm.V3_PEG_ANG)
FEATS = [
    ('upper: M3 hole at the joint face', upb, 11.0, (0, 90, 180, 270), -8.8, False),
    ('upper: M3 hole mid depth',         upb, 11.0, (0, 90, 180, 270), -6.0, False),
    ('upper: M3 hole rim, solid',        upb, 13.0, (0, 90, 180, 270), -6.0, True),
    ('upper: filled hex nut pocket',     upb, 13.5, (0, 90, 180, 270), -7.0, True),
    ('upper: head pocket open',          upb,  6.001, (45, 135, 225, 315), -8.0, False),
    ('upper: head pocket ceiling solid', upb,  6.001, (45, 135, 225, 315), -6.8, True),
    ('upper: rotor face solid over it',  upb,  6.001, (45, 135, 225, 315), -0.2, True),
    ('upper: the eight pegs, mid height', upb, 12.2, EIGHT, -9.6, True),
    ('upper: the eight pegs, near tip',  upb, 12.2, EIGHT, -10.15, True),
    ('upper: air between the pegs',      upb, 12.2, (45, 135, 225, 315), -9.6, False),
    ('lower: M3 hole through the web',   lob, 11.0, (0, 90, 180, 270), -9.5, False),
    ('lower: hex nut pocket open',       lob, 13.5, (0, 90, 180, 270), -12.0, False),
    ('lower: nut pocket ceiling',        lob, 13.5, (0, 90, 180, 270), -10.5, True),
    ('lower: solid between the pockets', lob, 11.0, (45, 135, 225, 315), -12.0, True),
    ('lower: horn screw hole open',      lob,  6.001, (45, 135, 225, 315), -9.5, False),
    ('lower: the eight sockets',         lob, 12.2, EIGHT, -9.6, False),
    ('lower: the eight socket floors',   lob, 12.2, EIGHT, -10.45, True),
    ('lower: solid between the sockets', lob, 12.2, (45, 135, 225, 315), -9.6, True),
]
for what, body, r, angs, z, want in FEATS:
    got = [solid(body, r, a, z) for a in angs]
    ok = all(g == want for g in got)
    log('  %-35s r %6.3f z %7.3f  %s  %s' % (
        what, r, z, ' '.join(('S' if g else '.') for g in got),
        'OK' if ok else 'WRONG (want %s)' % ('solid' if want else 'void')))
    if not ok:
        problems.append('%s: %s' % (what, got))

# ---- 5b. the swept grid ----
NOTCH = 'r >= 14.70 and z >= -1.55'          # the index notch, EXO-226: 4.0 wide, r 14.75..17.9


def scan(tag, body, z0, z1, r_max, dr, dz, da, budget):
    """Every (r, z, ang) with ang < 90: containment at ang, +90, +180, +270 must agree."""
    t0 = time.time()
    n = 0
    bad_notch, bad_other = [], []
    # the r grid is offset 0.13 so it never lands ON a cylinder the part is drawn with
    # (8.1, 9.4, 12.6, 14.75, 15, 17.5, 17.9): a point exactly on a face reads "on", and
    # two of the four angles rounding differently there would be a false asymmetry.
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
log('--- ANGULAR SYMMETRY, swept grid (%s = the index notch, expected) ---' % NOTCH)
n_up, notch_up, other_up = scan('upper', upb, BuildArm.V3_PEG_Z[0], 0.0, 17.9,
                                0.5, 0.25, 2.0, 150.0)
n_lo, notch_lo, other_lo = scan('lower', lob, -13.2, -9.0, 17.4, 0.5, 0.25, 2.0, 150.0)
log('  upper: %d asymmetric quadruples inside the notch box, %d anywhere else'
    % (len(notch_up), len(other_up)))
log('  lower: %d asymmetric quadruples inside the notch box, %d anywhere else'
    % (len(notch_lo), len(other_lo)))
for tag, hits in (('upper', other_up), ('lower', other_lo)):
    for r, z, a, pat in hits[:40]:
        log('    %s ASYMMETRIC r %6.3f z %7.3f ang %5.1f  %s' % (tag, r, z, a, pat))
    if len(hits) > 40:
        log('    ... and %d more' % (len(hits) - 40))
if other_up:
    problems.append('the upper part is asymmetric at %d grid points OUTSIDE the notch box'
                    % len(other_up))
if other_lo:
    problems.append('the lower part is asymmetric at %d grid points' % len(other_lo))
if notch_up:
    log('  the upper part\'s ONLY 4-fold asymmetry is the index notch: %d points, '
        'r %.3f..%.3f z %.3f..%.3f, angles %s'
        % (len(notch_up), min(h[0] for h in notch_up), max(h[0] for h in notch_up),
           min(h[1] for h in notch_up), max(h[1] for h in notch_up),
           sorted(set(h[2] for h in notch_up))))
else:
    problems.append('the index notch did not show up in the sweep - is it still there?')

log('')
if problems:
    log('PROBLEMS (%d):' % len(problems))
    for p in problems:
        log('  * %s' % p)
    raise RuntimeError('EXO-238 verification failed: %d problem(s)' % len(problems))
log('EXO-238: all checks pass.')
