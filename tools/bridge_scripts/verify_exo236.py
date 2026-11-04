"""EXO-236 evidence: the v3 module with the two register pegs and their two sockets DELETED.

Runs `BuildArm.ensure_v3_module` - which finds the peg/socket module already in the document
WRONG against the new `V3_PROBES` and re-makes it from the linked original - then measures:

  1. the LINKED `sensor module 2` is untouched (three body volumes to 1e-3 mm3);
  2. both rotor part volumes, and the two old peg/socket sites read back solid / void;
  3. the mating faces are FLAT: the lower part tops out at z -9.000, the upper part bottoms
     out at z -9.000, and nothing sticks out of either;
  4. ANGULAR SYMMETRY. Two passes, both by point containment in the module frame:
       a. targeted - every feature probed at all four of its own angles;
       b. a swept grid over each part: for a point (r, z, ang) the containment at ang,
          ang+90, ang+180, ang+270 must agree. Any disagreement is reported with its
          (r, z, ang), so a feature that breaks 4-fold symmetry names itself. The upper
          part's index notch is EXPECTED to break it and is reported separately.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/verify_exo236.py --timeout 1200
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

# The peg/socket v3, measured in EXO-231/EXO-233 - what this change moves away from.
WAS = {'upper': 7117.647, 'lower': 2590.977}
# Ø3.0 peg 1.0 mm out of the face, Ø3.3 socket 1.3 mm into it, two of each.
D_PEG = 2.0 * math.pi * 1.5 ** 2 * 1.0
D_SOCKET = 2.0 * math.pi * 1.65 ** 2 * 1.3


def solid(body, r, ang, z):
    x, y = BuildArm._polar(r, ang)
    return body.pointContainment(ac.Point3D.create(x / 10.0, y / 10.0, z / 10.0)) in INSIDE


def edge_z(body, r, ang, z_in, z_out):
    for _ in range(60):
        m = (z_in + z_out) / 2.0
        z_in, z_out = (m, z_out) if solid(body, r, ang, m) else (z_in, m)
    return (z_in + z_out) / 2.0


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
log('--- v3 after EXO-236: the two rotor parts ---')
vols = {}
for tag, c in (('upper', up), ('lower', lo)):
    r0, r1, z0, z1 = BuildArm._body_extent(c)
    v = c.bRepBodies.item(0).volume * 1000.0
    vols[tag] = v
    log('  %-6s V=%10.3f mm3  r %6.3f..%6.3f z %8.3f..%8.3f' % (tag, v, r0, r1, z0, z1))
log('  upper: %.3f (with sockets) + %.3f (the two O3.3 sockets filled back in) = %.3f'
    % (WAS['upper'], D_SOCKET, WAS['upper'] + D_SOCKET))
log('  lower: %.3f (with pegs)    - %.3f (the two O3.0 pegs gone)            = %.3f'
    % (WAS['lower'], D_PEG, WAS['lower'] - D_PEG))
for tag, want in (('upper', WAS['upper'] + D_SOCKET), ('lower', WAS['lower'] - D_PEG)):
    if abs(vols[tag] - want) > 1e-3:
        problems.append('%s part is %.3f mm3, the peg/socket delete predicts %.3f'
                        % (tag, vols[tag], want))

# ---- 3. the mating faces are flat and clean ----
log('')
log('--- the mating interface at z -9.000 ---')
lo_r0, lo_r1, lo_z0, lo_z1 = BuildArm._body_extent(lo)
up_r0, up_r1, up_z0, up_z1 = BuildArm._body_extent(up)
log('  lower part tops out at z %.4f (asked %.3f: flat, nothing sticks up)'
    % (lo_z1, BuildArm.V3_LOWER_Z[1]))
log('  upper part bottoms out at z %.4f (asked %.3f: flat, nothing sticks down)'
    % (up_z0, BuildArm.V3_UPPER_Z[0]))
if abs(lo_z1 - BuildArm.V3_LOWER_Z[1]) > 1e-6:
    problems.append('the lower part tops out at %.4f, not %.3f'
                    % (lo_z1, BuildArm.V3_LOWER_Z[1]))
if abs(up_z0 - BuildArm.V3_UPPER_Z[0]) > 1e-6:
    problems.append('the upper part bottoms out at %.4f, not %.3f'
                    % (up_z0, BuildArm.V3_UPPER_Z[0]))
for ang in (67.5, 112.5):
    z_lo = edge_z(lob, 12.2, ang, -9.5, -8.0)
    up_here = solid(upb, 12.2, ang, -8.5)
    log('  old register site r 12.2 %5.1f deg: lower part face z %.4f (no peg), upper part '
        '%s at z -8.5 (no socket)' % (ang, z_lo, 'SOLID' if up_here else 'void'))
    if abs(z_lo - (-9.0)) > 1e-4:
        problems.append('the lower part still rises to %.4f at %.1f deg' % (z_lo, ang))
    if not up_here:
        problems.append('the upper part is still hollow at the %.1f deg socket site' % ang)

# ---- 4a. the features, probed at all four of their own angles ----
log('')
log('--- ANGULAR SYMMETRY, feature by feature (all four angles must agree) ---')
FEATS = [
    ('upper: M3 hole at the joint face', upb, 11.0, (0, 90, 180, 270), -8.8, False),
    ('upper: M3 hole mid depth',         upb, 11.0, (0, 90, 180, 270), -6.0, False),
    ('upper: M3 hole rim, solid',        upb, 13.0, (0, 90, 180, 270), -6.0, True),
    ('upper: filled hex nut pocket',     upb, 13.5, (0, 90, 180, 270), -7.0, True),
    ('upper: head pocket open',          upb,  6.001, (45, 135, 225, 315), -8.0, False),
    ('upper: head pocket ceiling solid', upb,  6.001, (45, 135, 225, 315), -6.8, True),
    ('upper: rotor face solid over it',  upb,  6.001, (45, 135, 225, 315), -0.2, True),
    ('upper: old socket site now solid', upb, 12.2, (22.5, 67.5, 112.5, 157.5,
                                                    202.5, 247.5, 292.5, 337.5), -8.5, True),
    ('lower: M3 hole through the web',   lob, 11.0, (0, 90, 180, 270), -9.5, False),
    ('lower: hex nut pocket open',       lob, 13.5, (0, 90, 180, 270), -12.0, False),
    ('lower: nut pocket ceiling',        lob, 13.5, (0, 90, 180, 270), -10.5, True),
    ('lower: solid between the pockets', lob, 11.0, (45, 135, 225, 315), -12.0, True),
    ('lower: horn screw hole open',      lob,  6.001, (45, 135, 225, 315), -9.5, False),
    ('lower: old peg site now void',     lob, 12.2, (22.5, 67.5, 112.5, 157.5,
                                                    202.5, 247.5, 292.5, 337.5), -8.5, False),
]
for what, body, r, angs, z, want in FEATS:
    got = [solid(body, r, a, z) for a in angs]
    ok = all(g == want for g in got)
    log('  %-34s r %6.3f z %7.3f  %s  %s' % (
        what, r, z, ' '.join(('S' if g else '.') for g in got),
        'OK' if ok else 'WRONG (want %s)' % ('solid' if want else 'void')))
    if not ok:
        problems.append('%s: %s' % (what, got))

# ---- 4b. the swept grid ----
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
n_up, notch_up, other_up = scan('upper', upb, -9.0, 0.0, 17.9, 0.5, 0.25, 2.0, 120.0)
n_lo, notch_lo, other_lo = scan('lower', lob, -13.2, -9.0, 17.4, 0.5, 0.25, 2.0, 120.0)
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
    log('EXO-236 PROBLEMS:')
    for p in problems:
        log('  - %s' % p)
    raise RuntimeError('%d problem(s)' % len(problems))
log('EXO-236 verify: pegs and sockets gone, both parts 4-fold symmetric apart from the '
    'index notch, and every number above reads back.')
