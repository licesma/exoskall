"""Where every module is turned to about its own axis, and why (EXO-130).

Pure Python, no Fusion: run it directly.

    /usr/bin/python3 tools/bridge_scripts/module_spin_report.py

The URDF fixes a joint's position and its axis and says nothing about the module's rotation
ABOUT that axis. That rotation carries the three stator inserts (INSERT_ANGLES 0/180/270 -
a 90 deg gap between 180 and 270, another between 270 and 0, and a 180 deg one on the
connector side where the actuator cutout is), the cutout itself, the rotor's index notch and
the stator's stop block. It used to be a hand-set `y_hint`; `armgen.solve_module_spins`
solves it.

Part 1, per joint, is the decisive check: the insert angles as the LANDING sees them, the
arc of the insert circle the landing footprint blocks, and whether a rotation exists that
keeps every insert its `HOLE_FLAT_RING`.

Part 2 is the solve: the feasible windows, the angle chosen, the rule that chose it, and the
maximin angle it did NOT choose where the measured pose was kept.

Part 3 re-proves the chain closed on the solved frames and prints every screw hole's ring.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'fusion_scripts', 'BuildArm'))
sys.path.insert(0, HERE)
import armgen as g                                                    # noqa: E402
from geom_snapshot import g1_seed_frames, JOINTS, LINKS               # noqa: E402

ARC_STEP = 0.05     # deg, the resolution the blocked arc is reported at


def blocked_arc(spec):
    """The arc of the insert circle (r INSERT_RADIUS, in the SLAB's own drawing frame) that
    the landing footprint blocks: every angle at which a hole centred there would lose its
    flat ring on either face of the slab. This is the footprint the rotation has to fit into
    a gap, expressed as an angle so it can be compared with INSERT_ANGLES directly."""
    land = spec['route']['stator_land']
    run = g.route_runs(spec['route'])[-1]
    y_end = land['y_end']
    xw = [e['x'] for e in land['edges']]
    x_lo, x_hi = min(xw) - g.TONGUE_ROUND_MARGIN, max(xw) + g.TONGUE_ROUND_MARGIN
    reach = g.HOLE_R + g.HOLE_FLAT_RING
    r_far, r_near = land['corner_r_nominal'], land['fillet_r_nominal']
    faces = ((y_end - r_far, y_end), (y_end - run['thickness'] - r_near, y_end))
    hit = []
    n = int(round(360.0 / ARC_STEP))
    for k in range(n):
        phi = k * ARC_STEP
        x = g.INSERT_RADIUS * math.cos(math.radians(phi))
        y = g.INSERT_RADIUS * math.sin(math.radians(phi))
        if any(g._rect_point_dist((x, y), x_lo, x_hi, lo, hi) < reach for lo, hi in faces):
            hit.append(phi)
    runs = []
    for phi in hit:
        if runs and abs(phi - runs[-1][1] - ARC_STEP) < 1e-9:
            runs[-1][1] = phi
        else:
            runs.append([phi, phi])
    if len(runs) > 1 and runs[0][0] == 0.0 and runs[-1][1] >= 360.0 - ARC_STEP:
        runs[0][0] = runs[-1][0] - 360.0
        runs.pop()
    return runs, (x_lo, x_hi, faces[1][0])


def insert_angles_in_slab(spec):
    """Each stator insert's angle in the MODULE frame (always INSERT_ANGLES) and in the
    slab's own drawing frame, where the landing footprint lives."""
    out = []
    for h in g.stator_hole_stations(spec['route']):
        x, y = h['slab']
        out.append((h['angle'], math.degrees(math.atan2(y, x)) % 360.0, x, y))
    return out


def main():
    seed = g1_seed_frames()
    print('=== 1. decisive check: can a rotation put every landing in an insert gap? ===\n')
    print('    INSERT_ANGLES = %s deg in the module frame, HOLE_FLAT_RING = %.1f mm\n'
          % (', '.join('%.0f' % a for a in g.INSERT_ANGLES), g.HOLE_FLAT_RING))
    for name, a, b, pad_r, width in LINKS:
        R_ab, t_ab = g.relative_frame(seed[a][0], seed[a][1], seed[b][0], seed[b][1])
        spec = g.route_relative(R_ab, t_ab, pad_r, width=width)
        land = spec['route']['stator_land']
        arcs, (x_lo, x_hi, y_flat) = blocked_arc(spec)
        print('--- %s: the %s route lands on %s (stator pad) ---' % (name, spec['pattern'], b))
        print('    landing footprint at the MEASURED radii (fillet r %.3f, corner r %.3f): '
              'slab frame x %.2f..%.2f, near-face flat starts at y %.2f'
              % (land['fillet_r_nominal'], land['corner_r_nominal'], x_lo, x_hi, y_flat))
        if abs(land['fillet_r'] - land['fillet_r_nominal']) > 1e-9 or \
                abs(land['corner_r'] - land['corner_r_nominal']) > 1e-9:
            print('    as DRAWN today, cut back for an insert: fillet r %.3f, corner r %.3f'
                  % (land['fillet_r'], land['corner_r']))
        print('    it blocks insert-circle angles: %s'
              % (', '.join('%.1f..%.1f deg (%.1f wide)' % (lo, hi, hi - lo) for lo, hi in arcs)
                 or 'none - the landing never comes near the insert circle'))
        for ang, phi, x, y in insert_angles_in_slab(spec):
            inside = any(lo <= (phi if phi >= lo else phi + 360.0) <= hi for lo, hi in arcs)
            print('      insert %5.1f deg (module)  ->  %6.1f deg (slab), (%7.3f, %7.3f)  %s'
                  % (ang, phi, x, y, 'CROSSED' if inside else 'clear'))
        print('')

    print('=== 2. the solve ===\n')
    spins = g.solve_module_spins(seed, LINKS)
    for label, jname, _s, hint in JOINTS:
        f = spins[label]
        print('--- %s (seed y_hint %s%s) ---'
              % (label, hint, '' if 'from_link' not in f else ', stator end of %s'
                 % f['from_link']))
        print('    rotation %+9.4f deg   %s' % (f['spin'], f['why']))
        if f['margin'] is not None:
            print('    smallest stator flat ring at that rotation: %.3f mm (need %.2f)'
                  % (f['margin'], g.HOLE_FLAT_RING))
        if f['windows']:
            print('    feasible windows: %s'
                  % ', '.join('%.3f..%.3f (%.2f deg wide)' % (lo, hi, hi - lo)
                              for lo, hi in f['windows']))
        if f['best'] is not None and abs(f['best']['spin'] - f['spin']) > 1e-6:
            print('    NOT taken: the maximin rotation over the whole turn is %+.4f deg with '
                  '%.3f mm - keeping the measured pose costs %.3f mm and rewrites no link'
                  % (f['best']['spin'], f['best']['margin'], f['best']['margin'] - f['margin']))
        print('')

    res = g.spin_windows_resolved(spins)
    print('    sweep resolution: %.2f deg; narrowest window found %.2f deg (%s) - %s\n'
          % (res['step'], res['narrowest'], res['at'],
             'resolved' if res['ok'] else 'TOO CLOSE TO THE STEP, refine SPIN_STEP'))

    print('=== 3. the chain, closed on the solved frames ===\n')
    frames = dict((k, (g.frame_spun(R, spins[k]['spin']), t)) for k, (R, t) in seed.items())
    for c in g.spin_chain_check(frames, LINKS, spins):
        print('--- %s (%s route, %s -> %s) ---' % (c['link'], c['pattern'], c['a'], c['b']))
        print('    land fillet r %.3f, far-face corner round r %.3f, smallest stator ring '
              '%.3f mm' % (c['fillet_r'], c['corner_r'], c['stator_margin']))
        for h in sorted(c['rings'], key=lambda h: h['ring']):
            print('      %-6s %5.1f deg %-11s ring %8.3f  (%s)'
                  % (h['pad'], h['angle'], h['face'], h['ring'], h['bound']))
        print('')
    return 0


if __name__ == '__main__':
    sys.exit(main())
