"""EXO-218: solve Gento Luna's free axis signs and its slides OUTSIDE Fusion.

`sweepgen.solve_axis_signs` is pure Python - the URDF, the module frames and the router -
but on Luna it is 2^5 = 32 axis-sign combinations, each of which slides four candidates
against the router for up to eight rounds. That is far too long to spend inside a Fusion
session holding Esteban's document, so it is run here and the ANSWER is handed to
`BuildArmGen.LUNA_SPEC['solved']`, where `sweepgen.verify_solved` checks it at every build.

Nothing here is a rule of its own: it calls the same function the builder would have
called, with `only` (which combinations this process tries) and `scores` (every one it
tried, with the key the rule ranks by). Merging the JSON of several processes and taking
the smallest key is exactly what one process would have done.

    python3 tools/luna_solve.py --only 0,1,2,3 --out /tmp/luna_0.json
    python3 tools/luna_solve.py --merge /tmp/luna_*.json
"""
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '_adsk_stub'))
sys.path.insert(0, os.path.join(HERE, '..', 'fusion_scripts', 'BuildArm'))


def _spec():
    import BuildArmGen
    import armgen
    import sweepgen
    spec = BuildArmGen.LUNA_SPEC
    joints, by_child = armgen.parse_urdf(spec['urdf'])
    origin, _a = armgen.joint_world_pose(spec['origin_at'], joints, by_child)
    off = spec.get('offset', (0.0, 0.0, 0.0))
    shift = [origin[i] - off[i] for i in range(3)]
    links = sweepgen.spec_links(spec)
    order = [j['label'] for j in sweepgen.spec_joints(spec)]
    _s, _l, table = sweepgen.seed_frames(spec, joints, by_child, shift)
    return spec, joints, by_child, shift, links, order, table


def run(only, out):
    import sweepgen
    spec, joints, by_child, shift, links, order, table = _spec()
    scores = []
    t0 = time.time()
    try:
        sweepgen.solve_axis_signs(spec, joints, by_child, shift, links, table,
                                  lambda s: None, order, only=only, scores=scores)
    except ValueError as e:
        print('solve raised: %s' % e)
    for r in scores:
        print('k=%2d ok=%d total=%.3f slides=%s key=%s'
              % (r['k'], r['ok'], r['total'],
                 ' '.join('%s%+.3f' % (l, v) for l, v in sorted(r['slides'].items()) if abs(v) > 1e-9),
                 r['key']))
    print('%d combination(s) in %.1f s' % (len(scores), time.time() - t0))
    with open(out, 'w') as f:
        json.dump({'free': sorted(l for l, v in table.items() if v['solved']),
                   'order': order, 'scores': scores}, f, indent=1)


def merge(files):
    rows, free, order = [], None, None
    for p in files:
        d = json.load(open(p))
        free, order = d['free'], d['order']
        rows.extend(d['scores'])
    rows.sort(key=lambda r: r['key'])
    print('free signs: %s -> %d combinations tried' % (free, len(rows)))
    for r in rows[:6]:
        print('  k=%2d key=%s combo=%s slides=%s'
              % (r['k'], r['key'], r['combo'],
                 dict((l, round(v, 4)) for l, v in r['slides'].items() if abs(v) > 1e-9)))
    best = rows[0]
    print()
    print('TAKEN k=%d, %d of 6 links route, total centre line %.3f mm, total slide %.3f mm'
          % (best['k'], best['ok'], best['total'], best['key'][2]))
    print(json.dumps({'signs': best['combo'],
                      'slides': dict((l, round(v, 6)) for l, v in best['slides'].items())},
                     sort_keys=True))
    if best['why']:
        print('refusals at the taken combination:')
        for w in best['why']:
            print('  ' + w)


if __name__ == '__main__':
    a = sys.argv[1:]
    if a and a[0] == '--merge':
        merge(a[1:])
    else:
        only = [int(x) for x in a[a.index('--only') + 1].split(',')]
        run(only, a[a.index('--out') + 1])
