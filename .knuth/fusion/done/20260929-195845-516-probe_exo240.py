"""EXO-240 pre-measurement: the horn screw head column in the UPPER part, and the hex
nut pocket at 6.0 across flats, both read off the module ALREADY in the document.

Changes nothing. Answers the two questions EXO-240 needs before the recipe moves:

  * the O5.0 head pocket column at r 6.001: what is solid above it, and how much solid
    stands INSIDE (towards the axis) and OUTSIDE (towards the M3 holes at r 11) of it,
    so a through cut can be shown to break nothing else;
  * the hex nut pocket in the lower part: where its inner FLAT sits now, where the horn
    bore wall and its top are, and where a floor probe point would have to move when the
    pocket shrinks from 6.0 to 5.8 across flats.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/probe_exo240.py --timeout 300
"""
import math
import sys
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

occ = None
for i in range(root.occurrences.count):
    if root.occurrences.item(i).component.name == BuildArm.MODULE_V3_NAME:
        occ = root.occurrences.item(i)
if occ is None:
    raise RuntimeError('no "%s" occurrence at the root' % BuildArm.MODULE_V3_NAME)
up_occ, lo_occ = BuildArm.module_rotor_occs(occ, in_context=True)
UP = up_occ.bRepBodies.item(0)
LO = lo_occ.bRepBodies.item(0)
log('upper %.3f mm3   lower %.3f mm3   (V3_NUT_AF %.2f, V3_HEAD_Z %s)'
    % (UP.volume * 1000.0, LO.volume * 1000.0, BuildArm.V3_NUT_AF, BuildArm.V3_HEAD_Z))


def solid_xy(body, x, y, z):
    return body.pointContainment(ac.Point3D.create(x / 10.0, y / 10.0, z / 10.0)) in INSIDE


def solid(body, r, ang, z):
    x, y = BuildArm._polar(r, ang)
    return solid_xy(body, x, y, z)


def bisect(f, a_in, a_out):
    for _ in range(60):
        m = (a_in + a_out) / 2.0
        a_in, a_out = (m, a_out) if f(m) else (a_in, m)
    return (a_in + a_out) / 2.0


# ---- 1. the upper part along a horn screw radius, 45 deg ----
log('')
log('--- UPPER part, solid/void along r at 45 deg (a horn screw column, axis r 6.001) ---')
RS = (0.0, 1.0, 2.0, 3.0, 3.3, 3.4, 3.5, 3.6, 4.0, 6.001, 8.4, 8.5, 8.6, 8.7, 9.0,
      9.4, 10.0, 11.0, 13.0, 14.5, 15.5, 17.0, 17.9)
for z in (-0.2, -2.0, -4.0, -5.2, -6.0, -6.8, -7.5, -8.0, -8.9):
    log('  z %7.3f  ' % z + ''.join(('S' if solid(UP, r, 45.0, z) else '.') for r in RS))
log('  r order:  ' + ' '.join('%.3f' % r for r in RS))

log('')
log('--- UPPER part: the column itself, bisected ---')
for z in (-0.2, -2.0, -4.0, -5.2, -6.0, -6.8, -7.5, -8.0, -8.9):
    ax = 6.001
    got = solid(UP, ax, 45.0, z)
    if got:
        log('  z %7.3f  SOLID on the axis' % z)
        continue
    r_in = bisect(lambda r: solid(UP, r, 45.0, z), 0.5, ax)      # inward flank
    r_out = bisect(lambda r: solid(UP, r, 45.0, z), 12.0, ax)    # outward flank
    log('  z %7.3f  void r %.4f..%.4f (width %.4f, centre %.4f)'
        % (z, r_in, r_out, r_out - r_in, (r_in + r_out) / 2.0))

log('')
log('--- UPPER part: solid on the axis, and the r 6 column ceiling at each of the four ---')
for a in BuildArm.V3_HORN_ANG:
    z_ceil = bisect(lambda z: solid(UP, 6.001, a, z), -6.5, -9.0)
    log('  %6.1f deg: head pocket ceiling z %.4f; solid above it to the rotor face %s'
        % (a, z_ceil, solid(UP, 6.001, a, -0.05)))
log('  on the axis r 0: z -0.2 %s, z -4.0 %s, z -8.9 %s'
    % (solid(UP, 0.0, 0.0, -0.2), solid(UP, 0.0, 0.0, -4.0), solid(UP, 0.0, 0.0, -8.9)))
log('  r 3.3 at 45 deg: z -2.0 %s, -6.0 %s, -8.9 %s  | r 8.7 at 45 deg: z -2.0 %s, '
    '-6.0 %s, -8.9 %s'
    % (solid(UP, 3.3, 45.0, -2.0), solid(UP, 3.3, 45.0, -6.0), solid(UP, 3.3, 45.0, -8.9),
       solid(UP, 8.7, 45.0, -2.0), solid(UP, 8.7, 45.0, -6.0), solid(UP, 8.7, 45.0, -8.9)))

# ---- 2. the lower part: hex pocket inner flat, horn bore, wall band ----
log('')
log('--- LOWER part: the hex nut pocket inner flat and the horn bore, at 0 deg ---')
for z in (-9.5, -10.0, -10.45, -10.51, -10.55, -10.65, -11.0, -12.0, -13.0):
    row = []
    for r in (7.0, 7.9, 8.0, 8.05, 8.1, 8.15, 8.2, 8.5, 9.0, 9.4, 9.5):
        row.append('S' if solid(LO, r, 0.0, z) else '.')
    log('  z %7.3f  ' % z + ''.join(row))
log('  r order:  7.0 7.9 8.0 8.05 8.1 8.15 8.2 8.5 9.0 9.4 9.5')

log('')
log('--- LOWER part: bore wall and pocket flat, bisected at 0 deg ---')
for z in (-9.5, -10.0, -10.45, -10.55, -10.65, -11.0, -12.0, -13.0):
    if solid(LO, 8.6, 0.0, z):
        r_lo = bisect(lambda r: solid(LO, r, 0.0, z), 9.3, 6.0)
        log('  z %7.3f  solid starts at r %.4f going out from the axis' % (z, r_lo))
    else:
        log('  z %7.3f  void at r 8.6 (pocket or bore)' % z)
z_bore_top = bisect(lambda z: not solid(LO, 7.5, 0.0, z), -12.0, -9.2)
log('  horn bore (r 7.5, 0 deg) is void up to z %.4f' % z_bore_top)
r_bore = bisect(lambda r: not solid(LO, r, 0.0, -11.0), 6.0, 9.0)
log('  horn bore wall at z -11.0: r %.4f' % r_bore)

log('')
log('--- LOWER part: hex pocket flanks at z -12.0, across the pocket at r 11 / 0 deg ---')
for y in (0.0, 1.0, 1.5, 1.6, 1.674, 1.7, 1.732, 1.8):
    xs = []
    for x in (7.9, 8.0, 8.05, 8.1, 8.15, 8.2, 8.3, 13.9, 14.0, 14.1):
        xs.append('S' if solid_xy(LO, x, y, -12.0) else '.')
    log('  y %6.3f  ' % y + ''.join(xs))
log('  x order:  7.9 8.0 8.05 8.1 8.15 8.2 8.3 | 13.9 14.0 14.1')

# ---- 3. where the floor probe point goes at 5.8 across flats ----
log('')
log('--- the floor probe point for a 5.8 AF pocket ---')
P = BuildArm._polar(BuildArm.V3_PEG_AT, BuildArm.V3_PEG_ANG[0])
SOCK_R = BuildArm.V3_SOCKET_R
for af_mm in (6.0, 5.8):
    rc = af_mm / math.sqrt(3.0)
    ri = af_mm / 2.0
    log('  AF %.2f: circumradius %.4f, inradius %.4f, inner flat at r %.4f, corner '
        '(11, %.4f) is %.4f from the socket axis (R %.3f) -> pokes %.4f into it'
        % (af_mm, rc, ri, 11.0 - ri, rc, math.hypot(11.0 - P[0], rc - P[1]), SOCK_R,
           SOCK_R - math.hypot(11.0 - P[0], rc - P[1])))


def in_hex(x, y, af_mm):
    """inside the hex at (11,0), `af_mm` across flats, flats radial (first vertex 30 deg)"""
    dx, dy = x - 11.0, y
    for k in range(3):
        n = math.radians(60.0 * k)
        if abs(dx * math.cos(n) + dy * math.sin(n)) > af_mm / 2.0:
            return False
    return True


def margins(x, y, af_mm):
    dx, dy = x - 11.0, y
    m_hex = min(af_mm / 2.0 - abs(dx * math.cos(math.radians(60.0 * k))
                                  + dy * math.sin(math.radians(60.0 * k)))
                for k in range(3))
    m_soc = SOCK_R - math.hypot(x - P[0], y - P[1])
    return m_hex, m_soc


best = None
x = 10.0
while x <= 12.0:
    y = 2.0
    while y <= 4.5:
        m = min(margins(x, y, 5.8))
        if best is None or m > best[0]:
            best = (m, x, y)
        y += 0.005
    x += 0.005
m, bx, by = best
mh, ms = margins(bx, by, 5.8)
log('  best floor probe xy for AF 5.8: (%.4f, %.4f) = r %.4f, %.4f deg; %.4f inside the '
    'hex, %.4f inside the socket' % (bx, by, math.hypot(bx, by),
                                     math.degrees(math.atan2(by, bx)), mh, ms))
log('  the OLD point (11.000, 3.400): in a 6.0 hex %s, in a 5.8 hex %s'
    % (in_hex(11.0, 3.4, 6.0), in_hex(11.0, 3.4, 5.8)))

# ---- 4. what the two changes predict ----
log('')
log('--- predicted volumes ---')
d_head = (len(BuildArm.V3_HORN_ANG) * math.pi * BuildArm.V3_HEAD_R ** 2
          * (0.0 - BuildArm.V3_UPPER_Z[0]))
log('  head columns through (z %.1f..0, O%.1f x4): %.3f mm3 removed, vs %.3f now -> '
    'upper 7207.743 -> %.3f'
    % (BuildArm.V3_UPPER_Z[0], 2 * BuildArm.V3_HEAD_R, d_head,
       len(BuildArm.V3_HORN_ANG) * math.pi * BuildArm.V3_HEAD_R ** 2 * 2.0,
       7207.743 - (d_head - len(BuildArm.V3_HORN_ANG) * math.pi
                   * BuildArm.V3_HEAD_R ** 2 * 2.0)))
for af_mm in (6.0, 5.8):
    a_hex = math.sqrt(3.0) / 2.0 * af_mm ** 2
    a_hole = math.pi * BuildArm.V3_M3_HOLE_R ** 2
    d = 4 * (a_hex - a_hole) * (BuildArm.V3_NUT_Z[1] - BuildArm.V3_NUT_Z[0] - 0.1)
    log('  hex pockets at AF %.2f: hex area %.4f, minus the O3.2 hole %.4f, x 2.6 deep x4 '
        '= %.3f mm3 (before any bore overlap)' % (af_mm, a_hex, a_hole, -d))
log('EXO-240 probe done.')
