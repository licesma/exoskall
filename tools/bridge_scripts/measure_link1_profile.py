#!/usr/bin/env python3
"""Measure exo `Link 1 R` (J1 -> J2, the shoulder L) for EXO-81's L pattern.

Not a bridge script - plain python3, run from the repo root:
    /usr/bin/python3 tools/bridge_scripts/measure_link1_profile.py

Frame: the BuildArm J1R MODULE frame (armgen.module_frame's convention, which is the
exo `Module 0 R` occurrence turned 180 deg about its own Y):
    origin (0, -200.440, 0)   X = (1, 0, 0)
    Y = (0, 0.27564, 0.96126) (connector side)   Z = (0, -0.96126, 0.27564) (rotor axis)
The rotor pad is then z 0..8 and the J2 stator slab sits at x -32.5..-24.5 (EXO-24).

Reported in the LINK frame the L pattern is drawn in (same construction as the bar,
EXO-28): s along u = the J2 axis projected into the J1 pad plane, pointing at the
slab; w = Z_A x u; z = the pad axis. Side profile = (s, z), width = w.
"""
import math
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'fusion_scripts', 'BuildArm'))
import steplib                                                  # noqa: E402
import armgen as g                                              # noqa: E402

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
EXO = os.path.join(REPO, 'gento_luna/exo/exports/exo_skeleton_2026-08-08.step')
URDF = os.path.join(REPO, 'g1', 'g1_29dof.urdf')

J1R_O = [0.0, -200.440, 0.0]
X_A = [1.0, 0.0, 0.0]
Y_A = [0.0, 0.27564, 0.96126]
Z_A = [0.0, -0.96126, 0.27564]


def link_frame():
    """(origin, rows) of the (s, w, z) link frame in STEP world coordinates."""
    joints, by_child = g.parse_urdf(URDF)
    FA = g.module_frame('right_shoulder_pitch_joint', joints, by_child, -1, (0, 0, 1))
    FB = g.module_frame('right_shoulder_roll_joint', joints, by_child, +1, (0, 1, 0))
    R_ab, t_ab = g.relative_frame(FA[0], FA[1], FB[0], FB[1])
    n = g.landing_numbers(R_ab, t_ab)
    # u, w are expressed in frame A; map them into STEP world through (X_A, Y_A, Z_A)
    def to_world(v):
        return [v[0] * X_A[i] + v[1] * Y_A[i] + v[2] * Z_A[i] for i in range(3)]
    u_w, w_w = to_world(n['u']), to_world(n['w'])
    rows = [u_w, w_w, Z_A]
    return J1R_O, rows, R_ab, t_ab, n


def to_frame(p, o, rows):
    d = [p[i] - o[i] for i in range(3)]
    return [sum(rows[i][k] * d[k] for k in range(3)) for i in range(3)]


def to_dir(v, rows):
    return [sum(rows[i][k] * v[k] for k in range(3)) for i in range(3)]


def main():
    o, rows, R_ab, t_ab, n = link_frame()
    print('J1 -> J2 relative frame (mm, frame A = J1 module frame)')
    print('  t_ab = (%.3f, %.3f, %.3f)' % tuple(t_ab))
    print('  Z_B in A = (%.4f, %.4f, %.4f)   Y_B in A = (%.4f, %.4f, %.4f)'
          % tuple([R_ab[i][2] for i in range(3)] + [R_ab[i][1] for i in range(3)]))
    print('  link frame: u = (%.4f, %.4f, %.4f) in A, w = (%.4f, %.4f, %.4f) in A'
          % tuple(list(n['u']) + list(n['w'])))
    print('  s_land %.3f   zL %.3f   w_off %.3f   slab faces s %.3f / %.3f'
          % (n['s_land'], n['zL'], n['w_off'], n['s_land'] - g.PAD_T, n['s_land']))
    print('  stator axis in the pad plane: %.6f   |Y_B . Z_A| %.6f (tongue tilt %.2f deg)'
          % (n['zb_in_plane'], n['yb_axial'], math.degrees(math.acos(min(1.0, n['yb_axial'])))))
    print()

    s = steplib.Step(EXO)
    occs = s.find('Link 1 R')
    if not occs:
        print('!! Link 1 R not found')
        return
    path, _, pd, T = occs[0]
    verts = [to_frame(steplib.apply(T, v), o, rows) for v in s.vertices(pd)]
    ss = [v[0] for v in verts]
    ws = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    print('=== Link 1 R (%s), %d vertices ===' % (' > '.join(path), len(verts)))
    print('extent: s %.2f..%.2f   w %.2f..%.2f   z %.2f..%.2f'
          % (min(ss), max(ss), min(ws), max(ws), min(zs), max(zs)))
    print()

    print('-- w extent (width) and z extent per 5 mm s band --')
    bands = defaultdict(list)
    for v in verts:
        bands[int(math.floor(v[0] / 5.0)) * 5].append(v)
    for b in sorted(bands):
        vs = bands[b]
        print('  s %4d..%4d  w %7.2f..%7.2f  z %7.2f..%7.2f  (%d)'
              % (b, b + 5, min(v[1] for v in vs), max(v[1] for v in vs),
                 min(v[2] for v in vs), max(v[2] for v in vs), len(vs)))
    print()

    for lab, axis, idx in (('|| w (side-profile arcs), centre (s, z)', 1, (0, 2)),
                           ('|| z (about the J1 axis), centre (s, w)', 2, (0, 1)),
                           ('|| u (about the J2 axis), centre (w, z)', 0, (1, 2))):
        print('-- cylinders %s --' % lab)
        seen = set()
        for c, ax, r in s.cylinders(pd):
            cw = to_frame(steplib.apply(T, c), o, rows)
            aw = to_dir(steplib.rot(T, ax), rows)
            if abs(aw[axis]) > 0.99 and r < 200.0:
                key = (round(cw[idx[0]], 2), round(cw[idx[1]], 2), round(r, 3))
                if key not in seen:
                    seen.add(key)
        for key in sorted(seen, key=lambda k: (k[2], k[0])):
            print('   centre (%8.2f, %8.2f)  r %8.3f' % key)
        print()

    print('-- side-elevation vertices in the corridor |w| <= 13, unique (s, z) --')
    pts = sorted(set((round(v[0], 2), round(v[2], 2)) for v in verts if abs(v[1]) <= 13.0))
    for p in pts:
        print('   (%8.2f, %8.2f)' % p)
    print()

    print('-- the plate in the pad plane: vertices with 0 <= z <= 8 --')
    pl = sorted(set((round(v[0], 2), round(v[1], 2), round(v[2], 2))
                    for v in verts if -0.05 <= v[2] <= g.PAD_T + 0.05))
    for p in pl:
        print('   s %8.2f  w %8.2f  z %8.2f' % p)


if __name__ == '__main__':
    main()
