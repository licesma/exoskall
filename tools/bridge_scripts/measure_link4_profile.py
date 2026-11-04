#!/usr/bin/env python3
"""Measure the exo `Link 4 R` (= link 45) and confirm J5's placement (EXO-114).

Not a bridge script - plain python3, run from anywhere:

    /usr/bin/python3 tools/bridge_scripts/measure_link4_profile.py

`Link 4 R` is the hand-built ground truth for the J4->J5 pair (EXO-19/EXO-41: exo
`Module k` = URDF joint k+1, so Link 4 R joins Module 3 R's rotor to Module 4 R's
stator). Everything is reported in the J4 ROTOR-PAD frame A = the exo `Module 3 R`
occurrence frame turned 180 degrees about its own Y (armgen's convention, the same
turn `measure_link3_profile.py` verified on all five right-arm modules).

It prints
  * the exo module frames and, for J5, every (axis_sign, y_hint) candidate scored
    against the exo `Module 4 R` pose - the EXO-103 decisive check, repeated;
  * the J4->J5 pair in BOTH frames (URDF chain and the exo's own two modules),
    because past J3 they differ by 8.83 mm (EXO-113);
  * every circular EDGE of Link 4 R with its two end points (a surface cannot say
    where an arc starts and stops; an edge can);
  * the side elevation (s, z) per lateral band, and
  * the slab-frame (x', y') outline of the tongue, where the lateral offset would
    live if the slab took it, and the PLAN view (s, w), where an S would live.
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
G1_URDF = os.path.join(REPO, 'g1', 'g1_29dof.urdf')
# exo product name -> the joint whose module it is (EXO-41: Module k = URDF joint k+1)
MODULES = {'J1': 'Module 0 R', 'J2': 'Module 1 R', 'J3': 'Module 2 R',
           'J4': 'Module 3 R', 'J5': 'Module 4 R', 'J6': 'Module 5 R'}
JOINT_NAMES = {'J1': 'right_shoulder_pitch_joint', 'J2': 'right_shoulder_roll_joint',
               'J3': 'right_shoulder_yaw_joint', 'J4': 'right_elbow_joint',
               'J5': 'right_wrist_roll_joint', 'J6': 'right_wrist_pitch_joint'}
# BuildArm.JOINTS, as built (EXO-103)
KNOWN = {'J1': (-1, (0, 0, 1)), 'J2': (+1, (0, 1, 0)), 'J3': (-1, (-1, 0, 0)),
         'J4': (-1, (-1, 0, 0))}
LINK = 'Link 4 R'
PAIR = ('J4', 'J5')


def module_frame(occs, name):
    """(R, t_mm) of an exo module occurrence in armgen's module-frame convention."""
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


def orient_err(Ra, Rb):
    """Rotation angle (rad) between two frames."""
    tr = sum(sum(Ra[i][k] * Rb[i][k] for i in range(3)) for k in range(3))
    return math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))


def circular_edges(step, pd, T_occ, R, o):
    """[(centre, axis, r, p0, p1)] of every EDGE_CURVE on a CIRCLE, in frame (R, o)."""
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


def faces(step, pd, T_occ, R, o):
    """[(kind, dict)] of every ADVANCED_FACE, in frame (R, o).

    PLANE -> point + normal; CYLINDRICAL_SURFACE -> axis point, axis, radius.
    Surfaces say what the part is MADE of (thicknesses, flat walls); the edges say
    where each piece stops. Both are needed to read a hand-built part.
    """
    out = []
    for eid, T_loc in step._walk_geom(pd, {'ADVANCED_FACE'}):
        a = step.args(eid)
        sid = a[2]
        st = step.typ(sid)
        sa = step.args(sid)
        if st == 'PLANE':
            P = steplib.mul(T_occ, steplib.mul(T_loc, step.placement(sa[1])))
            out.append(('plane', {
                'p': to_frame(R, o, [P[0][3], P[1][3], P[2][3]]),
                'n': to_dir(R, [P[0][2], P[1][2], P[2][2]])}))
        elif st == 'CYLINDRICAL_SURFACE':
            P = steplib.mul(T_occ, steplib.mul(T_loc, step.placement(sa[1])))
            out.append(('cyl', {
                'c': to_frame(R, o, [P[0][3], P[1][3], P[2][3]]),
                'ax': to_dir(R, [P[0][2], P[1][2], P[2][2]]), 'r': sa[2]}))
        else:
            out.append((st.lower(), {}))
    return out


def pair_of(frames, a, b):
    RA, tA = frames[a]
    RB, tB = frames[b]
    return g.relative_frame(RA, [v / 1000.0 for v in tA], RB, [v / 1000.0 for v in tB])


def main():
    step = steplib.Step(EXO)
    occs = step.occurrences()
    exo = dict((lab, module_frame(occs, nm)) for lab, nm in MODULES.items())

    print('=== exo module frames (armgen convention) from %s ===' % os.path.basename(EXO))
    for lab in sorted(exo):
        R, t = exo[lab]
        print('%-3s %-12s pos (%8.3f, %8.3f, %8.3f)  X=(%6.3f,%6.3f,%6.3f) '
              'Y=(%6.3f,%6.3f,%6.3f) Z=(%6.3f,%6.3f,%6.3f)'
              % (lab, MODULES[lab], t[0], t[1], t[2],
                 R[0][0], R[1][0], R[2][0], R[0][1], R[1][1], R[2][1],
                 R[0][2], R[1][2], R[2][2]))

    # ---- J5 placement: every candidate scored against the exo Module 4 R pose ----
    joints, by_child = g.parse_urdf(G1_URDF)
    print()
    print('=== J5 (%s) placement candidates vs exo %s ==='
          % (JOINT_NAMES['J5'], MODULES['J5']))
    hints = [(0, 0, 1), (0, 0, -1), (0, 1, 0), (0, -1, 0), (1, 0, 0), (-1, 0, 0)]
    best = None
    for sign in (+1, -1):
        for hint in hints:
            try:
                R, t = g.module_frame(JOINT_NAMES['J5'], joints, by_child, sign, hint)
            except ValueError:
                continue
            err = orient_err(R, exo['J5'][0])
            row = (err, sign, hint, R, t)
            print('  axis_sign %+d  y_hint %-11s  orientation err %9.2e rad (%7.4f deg)'
                  % (sign, str(hint), err, math.degrees(err)))
            if best is None or err < best[0]:
                best = row
    err, sign, hint, R5, t5 = best
    print('  -> best: axis_sign %+d, y_hint %s, err %.2e rad' % (sign, str(hint), err))
    print('     URDF J5 pos (mm) (%.3f, %.3f, %.3f) vs exo (%.3f, %.3f, %.3f), distance %.3f mm'
          % (t5[0] * 1000, t5[1] * 1000, t5[2] * 1000,
             exo['J5'][1][0], exo['J5'][1][1], exo['J5'][1][2],
             math.sqrt(sum((t5[i] * 1000 - exo['J5'][1][i]) ** 2 for i in range(3)))))

    # ---- the pair, in both frames ----
    urdf = {}
    for lab in ('J4', 'J5'):
        if lab in KNOWN:
            s, h = KNOWN[lab]
        else:
            s, h = sign, hint
        Ru, tu = g.module_frame(JOINT_NAMES[lab], joints, by_child, s, h)
        urdf[lab] = (Ru, [v * 1000.0 for v in tu])

    for src, frames in (('URDF (BuildArm.JOINTS + the J5 candidate)', urdf),
                        ('exo (Module 3 R -> Module 4 R)', exo)):
        R_ab, t_ab = pair_of(frames, *PAIR)
        num = g.landing_numbers(R_ab, t_ab)
        print()
        print('=== the %s -> %s pair, %s ===' % (PAIR[0], PAIR[1], src))
        print('t_ab (mm) = (%.3f, %.3f, %.3f)' % tuple(t_ab))
        print('R_ab      = %s' % ('; '.join('(%.4f, %.4f, %.4f)' % tuple(r) for r in R_ab)))
        for k in ('axis_angle_deg', 'zb_in_plane', 'yb_axial', 'axial', 'snap_rad',
                  's_land', 'zL', 'w_off', 'y_sign', 'u_sign'):
            print('  %-14s %s' % (k, ('%.4f' % num[k]) if k in num else 'n/a'))
        print('  %-14s %s' % ('y_sign_from', num.get('y_sign_from', 'n/a')))
        print('  %-14s %s' % ('s_slab', tuple(round(v, 3) for v in num['s_slab'])))
        print('  %-14s %s' % ('slab_centre', tuple(round(v, 3) for v in num['slab_centre'])))
        print('  u = %s   w = %s' % ([round(v, 4) for v in num['u']],
                                     [round(v, 4) for v in num['w']]))
        if src.startswith('exo'):
            exo_num = num
        else:
            urdf_num = num

    # ---- the body ----
    RA, tA = exo[PAIR[0]]
    num = exo_num
    occ = [o for o in occs if o[1] == LINK]
    if not occ:
        raise SystemExit('%s not found' % LINK)
    _path, _n, pd, T = occ[0]
    verts = [to_frame(RA, tA, steplib.apply(T, v)) for v in step.vertices(pd)]
    u_sign = num['u_sign']
    print()
    print('=== %s in frame A (x, y, z; u_sign %+.0f, s = %s) ==='
          % (LINK, u_sign, 'x' if u_sign > 0 else '-x'))
    print('vertices %d   x %.3f..%.3f   y %.3f..%.3f   z %.3f..%.3f'
          % (len(verts), min(v[0] for v in verts), max(v[0] for v in verts),
             min(v[1] for v in verts), max(v[1] for v in verts),
             min(v[2] for v in verts), max(v[2] for v in verts)))
    print('solids %d' % len(step.solids(pd)))

    print()
    print('=== circular edges, largest radius first (centre / axis / r / ends) ===')
    edges = circular_edges(step, pd, T, RA, tA)
    for c, ax, r, p0, p1 in sorted(edges, key=lambda e: -e[2]):
        print('  r %7.3f  centre (%8.3f,%8.3f,%8.3f) axis (%5.2f,%5.2f,%5.2f)  '
              '(%8.2f,%8.2f,%8.2f) -> (%8.2f,%8.2f,%8.2f)'
              % (r, c[0], c[1], c[2], ax[0], ax[1], ax[2],
                 p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]))

    # the link frame (s, w, z)
    u, w = num['u'], num['w']

    def swz(p):
        return (p[0] * u[0] + p[1] * u[1], p[0] * w[0] + p[1] * w[1], p[2])

    print()
    print('=== side elevation: unique (s, z) per lateral band w ===')
    bands = defaultdict(set)
    for v in verts:
        s, wv, z = swz(v)
        bands[round(wv, 1)].add((round(s, 3), round(z, 3)))
    for k in sorted(bands):
        print('  w = %8.2f : %s' % (k, sorted(bands[k])))

    print()
    print('=== plan view: unique (s, w) per z band (where an S would show) ===')
    zb = defaultdict(set)
    for v in verts:
        s, wv, z = swz(v)
        zb[round(z, 1)].add((round(s, 3), round(wv, 3)))
    for k in sorted(zb):
        print('  z = %8.2f : %s' % (k, sorted(zb[k])))


    print()
    print('=== planar faces: unique (normal, offset) in the link frame (s, w, z) ===')
    fs = faces(step, pd, T, RA, tA)
    def lf(v):
        return (v[0] * u[0] + v[1] * u[1], v[0] * w[0] + v[1] * w[1], v[2])
    planes, cyls, other = {}, {}, {}
    for kind, d in fs:
        if kind == 'plane':
            n = lf(d['n'])
            p = lf(d['p'])
            off = sum(n[i] * p[i] for i in range(3))
            if off < 0 or (abs(off) < 1e-9 and min(n) < 0):
                n, off = tuple(-v for v in n), -off
            key = (tuple(round(v, 4) + 0.0 for v in n), round(off, 4) + 0.0)
            planes[key] = planes.get(key, 0) + 1
        elif kind == 'cyl':
            c, ax = lf(d['c']), lf(d['ax'])
            if ax[0] + ax[1] + ax[2] < 0:
                ax = tuple(-v for v in ax)
            key = (round(d['r'], 4), tuple(round(v, 3) + 0.0 for v in ax),
                   tuple(round(v, 3) + 0.0 for v in c))
            cyls[key] = cyls.get(key, 0) + 1
        else:
            other[kind] = other.get(kind, 0) + 1
    for key in sorted(planes, key=lambda k: (k[0], k[1])):
        print('  n (%6.3f,%6.3f,%6.3f)  offset %9.4f   x%d' % (key[0] + (key[1], planes[key])))
    print('  cylinders:')
    for key in sorted(cyls, key=lambda k: -k[0]):
        print('    r %8.4f axis (%5.2f,%5.2f,%5.2f) through (s=%8.3f, w=%8.3f, z=%8.3f)  x%d'
              % ((key[0],) + key[1] + key[2] + (cyls[key],)))
    if other:
        print('  other surfaces: %s' % other)

    print()
    print("=== the tongue in the slab frame B' (x' across, y' from the disk toward the pad) ===")
    x_sign = num['y_sign'] * num['u_sign']
    for lab, val in (('w_off', num['w_off']), ('axial', num['axial']),
                     ('x_off = -w_off * x_sign', -num['w_off'] * x_sign),
                     ('y_end = -y_sign * axial', num['y_sign'] * (0.0 - num['axial']))):
        print('  %-24s %9.3f' % (lab, val))
    for c, ax, r, p0, p1 in sorted(edges, key=lambda e: -e[2]):
        if r < 10.0:
            continue
        sc, wc, zc = swz(c)
        print("  r %7.3f centre (s=%8.3f, w=%8.3f, z=%8.3f) axis (%5.2f,%5.2f,%5.2f)"
              % (r, sc, wc, zc, ax[0], ax[1], ax[2]))


if __name__ == '__main__':
    main()
