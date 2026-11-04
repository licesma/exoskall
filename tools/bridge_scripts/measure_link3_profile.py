#!/usr/bin/env python3
"""Measure the exo `Link 3 R` (= link 34) against the generated link_34 (EXO-103).

Not a bridge script - plain python3, run from anywhere:

    /usr/bin/python3 tools/bridge_scripts/measure_link3_profile.py

`Link 3 R` is the hand-built ground truth for the J3->J4 pair (EXO-19/EXO-41: exo
`Module k` = URDF joint k+1, so Link 3 R joins Module 2 R's rotor to Module 3 R's
stator). Everything is reported in the J3 ROTOR-PAD frame A, which is the exo
`Module 2 R` occurrence frame turned 180 degrees about its own Y - the convention
`armgen.module_frame` uses and the one BuildArm.JOINTS reproduces for J1..J3
(verified here against all five right-arm modules). In A: z is the pad axis (the
rotor face is z = 0, the pad is z 0..8), u = the stator axis projected into the pad
plane, s = the reach along u and w the lateral.

It prints
  * the module frames, the pair's (R_ab, t_ab) and armgen.landing_numbers,
  * every circular EDGE of Link 3 R with its two end points - radii are what the
    profile is made of, and an edge carries its extent, which a surface does not,
  * the side elevation (s, z) at the band walls, and
  * the slab-frame (x', y') outline of the tongue, where the lateral offset lives.
"""
import math
import os
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'tools'))
sys.path.insert(0, os.path.join(REPO, 'fusion_scripts', 'BuildArm'))
import steplib                                                  # noqa: E402
import armgen as g                                              # noqa: E402

EXO = os.path.join(REPO, 'gento_luna/exo/exports/exo_skeleton_2026-08-08.step')
# exo product name -> the joint whose module it is (EXO-41: Module k = URDF joint k+1)
MODULES = {'J1': 'Module 0 R', 'J2': 'Module 1 R', 'J3': 'Module 2 R',
           'J4': 'Module 3 R', 'J5': 'Module 4 R'}
LINK = 'Link 3 R'
PAIR = ('J3', 'J4')


def module_frame(step, occs, name):
    """(R, t_mm) of an exo module occurrence in armgen's module-frame convention.

    The STEP occurrence's own frame has its X and Z reversed with respect to
    `armgen.module_frame` (measured on all five right-arm modules: J1's Y is the
    same 16-degree tilt, J2 reproduces EXO-28's frame exactly). The turn is 180
    degrees about the module's Y, which leaves the actuator connector side alone.
    """
    for _p, n, _pd, T in occs:
        if n == name:
            R = [[-T[i][0], T[i][1], -T[i][2]] for i in range(3)]
            return R, [T[0][3], T[1][3], T[2][3]]
    raise SystemExit('%s not found in %s' % (name, EXO))


def to_frame(R, o, p):
    d = [p[i] - o[i] for i in range(3)]
    return [sum(R[k][i] * d[k] for k in range(3)) for i in range(3)]


def to_dir(R, v):
    return [sum(R[k][i] * v[k] for k in range(3)) for i in range(3)]


def circular_edges(step, pd, T_occ, R, o):
    """[(centre, axis, r, p0, p1)] of every EDGE_CURVE on a CIRCLE, in frame (R, o).

    Surfaces alone cannot say where an arc starts and stops; the edges can, which
    is what distinguishes a real fillet from the 2.5 mm edge break rolling over it.
    """
    out, seen = [], set()
    for eid, T_loc in step._walk_geom(pd, {'EDGE_CURVE'}):
        a = step.args(eid)
        if step.typ(a[3]) != 'CIRCLE':
            continue
        ca = step.args(a[3])
        P = steplib.mul(T_occ, steplib.mul(T_loc, step.placement(ca[1])))
        c = to_frame(R, o, [P[0][3], P[1][3], P[2][3]])
        ax = to_dir(R, [P[0][2], P[1][2], P[2][2]])
        ends = [to_frame(R, o, steplib.apply(T_occ, steplib.apply(T_loc, step.point(step.args(v)[1]))))
                for v in (a[1], a[2])]
        key = (tuple(round(v, 3) for v in c), round(ca[2], 3),
               tuple(round(v, 2) for v in ends[0]), tuple(round(v, 2) for v in ends[1]))
        if key in seen:
            continue
        seen.add(key)
        out.append((c, ax, ca[2], ends[0], ends[1]))
    return out


def main():
    step = steplib.Step(EXO)
    occs = step.occurrences()
    frames = dict((lab, module_frame(step, occs, nm)) for lab, nm in MODULES.items())

    print('=== module frames (armgen convention) from %s ===' % os.path.basename(EXO))
    for lab in sorted(frames):
        R, t = frames[lab]
        print('%-3s %-12s pos (%8.3f, %8.3f, %8.3f)  X=(%6.3f,%6.3f,%6.3f) '
              'Y=(%6.3f,%6.3f,%6.3f) Z=(%6.3f,%6.3f,%6.3f)'
              % (lab, MODULES[lab], t[0], t[1], t[2],
                 R[0][0], R[1][0], R[2][0], R[0][1], R[1][1], R[2][1],
                 R[0][2], R[1][2], R[2][2]))

    a, b = PAIR
    RA, tA = frames[a]
    RB, tB = frames[b]
    R_ab, t_ab = g.relative_frame(RA, [v / 1000.0 for v in tA], RB, [v / 1000.0 for v in tB])
    num = g.landing_numbers(R_ab, t_ab)
    print()
    print('=== the %s -> %s pair ===' % (a, b))
    print('t_ab (mm) = (%.3f, %.3f, %.3f)' % tuple(t_ab))
    print('R_ab      = %s' % ('; '.join('(%.4f, %.4f, %.4f)' % tuple(r) for r in R_ab)))
    for k in ('axis_angle_deg', 'zb_in_plane', 'yb_axial', 'axial', 'snap_rad',
              's_land', 'zL', 'w_off', 'y_sign', 'u_sign'):
        print('  %-14s %s' % (k, ('%.4f' % num[k]) if k in num else 'n/a'))
    print('  %-14s %s' % ('s_slab', tuple(round(v, 3) for v in num['s_slab'])))
    print('  %-14s %s' % ('slab_centre', tuple(round(v, 3) for v in num['slab_centre'])))
    print('  u = %s   w = %s' % ([round(v, 4) for v in num['u']],
                                 [round(v, 4) for v in num['w']]))

    occ = [o for o in occs if o[1] == LINK]
    if not occ:
        raise SystemExit('%s not found' % LINK)
    _path, _n, pd, T = occ[0]
    verts = [to_frame(RA, tA, steplib.apply(T, v)) for v in step.vertices(pd)]
    print()
    print('=== %s in frame A (x, y, z; s = -x here, u_sign %+.0f) ==='
          % (LINK, num['u_sign']))
    print('vertices %d   x %.3f..%.3f   y %.3f..%.3f   z %.3f..%.3f'
          % (len(verts), min(v[0] for v in verts), max(v[0] for v in verts),
             min(v[1] for v in verts), max(v[1] for v in verts),
             min(v[2] for v in verts), max(v[2] for v in verts)))

    print()
    print('=== circular edges, largest radius first (centre / axis / r / ends) ===')
    edges = circular_edges(step, pd, T, RA, tA)
    for c, ax, r, p0, p1 in sorted(edges, key=lambda e: -e[2]):
        print('  r %7.3f  centre (%8.3f,%8.3f,%8.3f) axis (%5.2f,%5.2f,%5.2f)  '
              '(%8.2f,%8.2f,%8.2f) -> (%8.2f,%8.2f,%8.2f)'
              % (r, c[0], c[1], c[2], ax[0], ax[1], ax[2],
                 p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]))

    print()
    print('=== side elevation: unique (x, z) per |y| band (the band walls are y = +/-26) ===')
    bands = defaultdict(set)
    for v in verts:
        bands[round(v[1], 1)].add((round(v[0], 3), round(v[2], 3)))
    for k in sorted(bands):
        print('  y = %8.2f : %s' % (k, sorted(bands[k])))

    print()
    print('=== the tongue in the slab frame B\' (x\' across, y\' from the disk to the plate) ===')
    x_sign = num['y_sign'] * num['u_sign']
    for lab, val in (('w_off', num['w_off']), ('axial', num['axial']),
                     ('x_off = -w_off * x_sign', -num['w_off'] * x_sign),
                     ('y_end = -y_sign * axial', num['y_sign'] * (0.0 - num['axial']))):
        print('  %-24s %9.3f' % (lab, val))
    for c, ax, r, p0, p1 in sorted(edges, key=lambda e: -e[2]):
        if abs(ax[0]) < 0.99 or r < 20.0:
            continue
        xp = x_sign * (-c[1] - num['w_off'])
        yp = num['y_sign'] * (c[2] - num['axial'])
        print("  r %7.3f centre (x'=%8.3f, y'=%8.3f)  |centre| %8.3f "
              "(rim r %.1f: sum %8.3f)" % (r, xp, yp, math.hypot(xp, yp),
                                           g.STATOR_PAD_R, math.hypot(xp, yp) - r))


if __name__ == '__main__':
    main()
