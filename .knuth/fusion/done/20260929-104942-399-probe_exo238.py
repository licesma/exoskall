"""EXO-238 pre-measurement: where eight Ø3.0 register pegs can go on the UPPER rotor part.

Measures the v3 module ALREADY in the document (EXO-236 state, no pegs) - it does not
build or change anything. Reports, all by point containment / bisection on the SOLID in
the module frame:

  * the outer radius of each rotor part over the z band the pegs and sockets would use;
  * the solid at the eight proposed peg sites (r 12.2, 22.5 + k*45 deg);
  * the hex nut pocket ceiling right under a socket, so the floor can be read off;
  * the analytic peg-to-M3-hole and peg-to-rim walls the chosen radius gives.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/probe_exo238.py --timeout 300
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
log('upper %.3f mm3   lower %.3f mm3' % (UP.volume * 1000.0, LO.volume * 1000.0))


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


def outer_r(body, ang, z):
    if not solid(body, 1.0, ang, z):
        return None
    return bisect(lambda r: solid(body, r, ang, z), 1.0, 30.0)


PEG_ANG = [22.5 + 45.0 * k for k in range(8)]

log('')
log('--- outer radius of the UPPER part (r 15 band expected at z -9..-4) ---')
for z in (-8.95, -8.5, -7.0, -5.5, -4.5, -4.0, -3.5, -2.0, -0.5):
    row = []
    for a in (0.0, 22.5, 45.0, 67.5, 90.0):
        r = outer_r(UP, a, z)
        row.append('%5.1f:%s' % (a, ('%8.4f' % r) if r else '    void')) 
    log('  z %7.3f  ' % z + '  '.join(row))

log('')
log('--- outer radius of the LOWER part ---')
for z in (-9.05, -9.5, -10.0, -10.4, -10.7, -11.5, -13.0):
    row = []
    for a in (0.0, 22.5, 45.0, 67.5, 90.0):
        r = outer_r(LO, a, z)
        row.append('%5.1f:%s' % (a, ('%8.4f' % r) if r else '    void'))
    log('  z %7.3f  ' % z + '  '.join(row))

log('')
log('--- the eight proposed peg sites, r 12.2, on the UPPER part (want SOLID) ---')
for a in PEG_ANG:
    got = [('%s' % ('S' if solid(UP, r, a, z) else '.'))
           for r in (10.7, 12.2, 13.7) for z in (-8.95, -8.5, -7.0)]
    log('  %6.1f deg  r10.7/12.2/13.7 x z -8.95/-8.5/-7.0 : %s' % (a, ' '.join(got)))

log('')
log('--- the eight proposed socket sites, r 12.2, on the LOWER part (want SOLID) ---')
for a in PEG_ANG:
    got = [('%s' % ('S' if solid(LO, r, a, z) else '.'))
           for r in (10.55, 12.2, 13.85) for z in (-9.05, -9.7, -10.25, -10.55)]
    log('  %6.1f deg  r10.55/12.2/13.85 x z -9.05/-9.7/-10.25/-10.55 : %s' % (a, ' '.join(got)))

log('')
log('--- the hex nut pocket right under a socket ---')
# nearest hex point to the socket at 22.5 deg is the pocket VERTEX at 90 deg from the
# pocket centre (11, 0): flats radial, circumradius 6.0/sqrt(3).
rc = BuildArm.V3_NUT_AF / math.sqrt(3.0)
vx, vy = 11.0, rc
sx, sy = BuildArm._polar(12.2, 22.5)
d = math.hypot(sx - vx, sy - vy)
log('  hex circumradius %.4f, vertex (%.4f, %.4f) = r %.4f, %.4f deg'
    % (rc, vx, vy, math.hypot(vx, vy), math.degrees(math.atan2(vy, vx))))
log('  socket centre (%.4f, %.4f); vertex is %.4f from it, socket radius 1.650 -> '
    'overlap %.4f mm in plan' % (sx, sy, d, 1.65 - d))
for (px, py, what) in ((vx, vy, 'the hex vertex'), (sx, sy, 'the socket axis')):
    if not solid_xy(LO, px, py, -10.0):
        log('  %s: NOT solid at z -10.0 - unexpected' % what)
        continue
    zc = bisect(lambda z: solid_xy(LO, px, py, z), -10.0, -13.3)
    log('  %-16s at (%.4f, %.4f): solid down to z %8.4f' % (what, px, py, zc))

log('')
log('--- analytic walls for Ø3.0 pegs / Ø3.3 sockets at r 12.2, 22.5 deg off the M3s ---')
for r_peg in (12.2, 12.3, 12.4, 12.5):
    dc = math.sqrt(r_peg ** 2 + 11.0 ** 2 - 2 * r_peg * 11.0 * math.cos(math.radians(22.5)))
    log('  r %5.2f: centres %.4f apart; peg Ø3.0 to Ø3.2 hole %.4f; socket Ø3.3 to hole '
        '%.4f; peg outer edge r %.3f, socket outer edge r %.3f'
        % (r_peg, dc, dc - 1.5 - 1.6, dc - 1.65 - 1.6, r_peg + 1.5, r_peg + 1.65))
log('done')
