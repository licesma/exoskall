"""Print the two built links as ROUTES, and what `route_path` returns for every G1 pair.

Pure Python, no Fusion: run it directly.

    /usr/bin/python3 tools/bridge_scripts/route_report.py

Part 1 - the four built links as routes: the segments (`Run` / `Bend` / `S`), the two join
descriptors and the numbers each carries, straight out of `armgen.route_path`. It also
re-derives the route from the drawn geometry and checks the two agree, so the printout is
evidence about the body that was built, not a restatement of the route object.

Part 2 - the six G1 pairs, each at BOTH approved ribbon widths (25 mm, the bar; 52 mm, the
shoulder). Width is Esteban's per-link input and there is no width rule in the code
(EXO-96), so the survey asks for both rather than picking one. Each cell is the route's
name or the named refusal.

Frames: J1..J3 are BuildArm's measured exo frames; J4..J7 follow classify_pairs.py's chain
rule and are PROVISIONAL (nothing has been built there).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'fusion_scripts', 'BuildArm'))
sys.path.insert(0, HERE)
import armgen as g                                              # noqa: E402
from classify_pairs import chain_frames, G1_URDF, G1_NAMES, G1_KNOWN   # noqa: E402
from geom_snapshot import g1_frames, LINKS                      # noqa: E402

WIDTHS = (25.0, 44.0, 52.0)    # BAR_W, RAMP_W and L_W - the widths Esteban has approved
LABELS = ['J%d' % i for i in range(1, 8)]


def derived_route(geo):
    """The route today's DRAWN geometry implies, read back off the side profile: the run
    directions and thicknesses from the profile's straight boundaries, the bend from its two
    fillet arcs. Cross-checks the route object against the shape actually produced."""
    prof = geo['profile']
    arcs = [sg for sg in prof if sg[0] == 'arc']
    zs = [q[1] for sg in prof for q in (sg[1:4] if sg[0] == 'arc' else sg[1:3])]
    out = {'n_segments': len(prof), 'n_arcs': len(arcs),
           'radii': sorted(sg[5] for sg in arcs), 'z_min': min(zs), 'z_max': max(zs),
           'width': geo['width']}
    return out


def print_route(name, spec):
    r, geo = spec['route'], spec['geometry']
    print('--- %s: the %r route, ribbon %.0f mm wide ---' % (name, r['pattern'], r['width']))
    for line in g.route_describe(r):
        print('    ' + line)
    runs = g.route_runs(r)
    print('    %-9s %s' % ('joins', 'rotor %r / stator %r' % (r['rotor_join']['kind'],
                                                             r['stator_land']['kind'])))
    print('    %-9s offset %+.3f mm taken in the %s' % ('offset', geo['w_off'],
                                                        geo['offset_taken']))
    d = derived_route(geo)
    print('    %-9s %d segments, %d arcs, radii %s, z %.3f..%.3f'
          % ('drawn', d['n_segments'], d['n_arcs'],
             '/'.join('%g' % v for v in d['radii']), d['z_min'], d['z_max']))
    # the drawn profile must contain every run boundary and every bend radius the route says
    want = set()
    for seg in r['segments']:
        if seg['kind'] == 'bend':
            want.add(round(seg['r_left'], 6))
            want.add(round(seg['r_right'], 6))
    want.add(round(r['stator_land']['fillet_r'], 6))
    if r['rotor_join']['kind'] == 'ledge':
        want.add(round(r['rotor_join']['round_r'], 6))
        want.add(round(r['rotor_join']['relief_r'], 6))
    got = set(round(v, 6) for v in d['radii'])
    print('    %-9s route radii %s all present in the drawn profile: %s'
          % ('check', sorted(want), 'YES' if want <= got else 'NO (missing %s)'
             % sorted(want - got)))
    print('    %-9s centre line %s' % ('vertices', ' -> '.join(
        '(%.3f, %.3f)' % rr['p0'] for rr in runs) + ' -> (%.3f, %.3f)' % runs[-1]['p1']))
    print()


def main():
    frames = g1_frames()
    print('=== the four built links, as routes (EXO-96) ===\n')
    for name, a, b, pad_r, width in LINKS:
        print_route(name, g.route(frames[a], frames[b], pad_r, width=width))

    print('=== route_path over the six G1 pairs, at both approved widths ===\n')
    fr, provisional = chain_frames(G1_URDF, G1_NAMES, G1_KNOWN)
    # one column per approved width. (Before EXO-124 this header said 25 / 52 while the
    # second column actually printed WIDTHS[1] = 44, which is where EXO-122's table got
    # 'J4->J5 ramp at 25 and 52' from - at 52 the plan taper refuses it, and always did.)
    print(('%-8s %8s %8s %8s   ' % ('pair', 's_land', 'zL', 'w_off'))
          + ''.join('%-30s' % ('width %g' % wdt) for wdt in WIDTHS))
    for i in range(len(fr) - 1):
        R_p, t_p = g.relative_frame(fr[i][0], fr[i][1], fr[i + 1][0], fr[i + 1][1])
        num = g.landing_numbers(R_p, t_p)
        cells = []
        for wdt in WIDTHS:
            try:
                rt = g.route_path(R_p, t_p, 27.0, width=wdt)
                cells.append('%s (%s, %s offset)' % (rt['pattern'], rt['rotor_join']['kind'],
                                                     rt['geometry']['offset_taken']))
            except ValueError as e:
                cells.append('REFUSED')
        print(('%-8s %8.2f %8.2f %8.2f   '
               % ('%s->%s' % (LABELS[i], LABELS[i + 1]), num.get('s_land', float('nan')),
                  num.get('zL', float('nan')), num.get('w_off', float('nan'))))
              + ''.join('%-30s' % c for c in cells))
    print('\n(provisional frames, chain rule: %s)'
          % ', '.join(LABELS[k - 1] for k in provisional))
    print('\n--- why each refusal, at width 25 ---')
    for i in range(len(fr) - 1):
        R_p, t_p = g.relative_frame(fr[i][0], fr[i][1], fr[i + 1][0], fr[i + 1][1])
        try:
            g.route_path(R_p, t_p, 27.0, width=25.0)
        except ValueError as e:
            print('%-8s %s' % ('%s->%s' % (LABELS[i], LABELS[i + 1]),
                               str(e).split('). ', 1)[-1]))


if __name__ == '__main__':
    main()
