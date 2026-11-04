#!/usr/bin/env python3
"""Measure the hand-made wrist: `Module 5/6 R` and `Link 5/6 R` (EXO-165).

Not a bridge script - plain python3, run from anywhere:

    /usr/bin/python3 tools/bridge_scripts/measure_wrist_profiles.py

EXO-163 says the generator cannot fit two r 27 modules on the G1's wrist pitch and
yaw joints, 46.00 mm apart on perpendicular axes.  The hand-made arm HAS both
(`Module 5 R`, `Module 6 R`, `Link 5 R`, `Link 6 R` in exo_skeleton_2026-08-08.step).
This script measures what it does instead, in three parts:

  1. the two module poses against URDF FK (`armgen.module_frame`), shifted onto the
     exo frame at J1 the way EXO-113 compared J1..J4, so the 8.83 mm the whole arm
     carries past J3 is subtracted and what is left is this wrist's own choice;
     then the real module-to-module clearance from the module BODIES, not from the
     infinite-cylinder model EXO-163 used;
  2. `Link 5 R` and `Link 6 R` in their rotor-pad frames (the exo `Module k R`
     occurrence frame turned 180 deg about its own Y - armgen's convention, the same
     turn measure_link3/4_profile.py verified on J1..J5): circular edges, side
     elevation, plan view and the planar faces;
  3. the numbers a generator would need: pad departure, stator landing, width,
     thickness, and how Link 6 R crosses the 46 mm.

Nothing is written.  No Fusion, no geometry change.
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
MODULES = dict(('J%d' % (k + 1), 'Module %d R' % k) for k in range(7))
JOINT_NAMES = {'J1': 'right_shoulder_pitch_joint', 'J2': 'right_shoulder_roll_joint',
               'J3': 'right_shoulder_yaw_joint', 'J4': 'right_elbow_joint',
               'J5': 'right_wrist_roll_joint', 'J6': 'right_wrist_pitch_joint',
               'J7': 'right_wrist_yaw_joint'}
# BuildArm.JOINTS as built (EXO-103 / EXO-114); J6 / J7 are solved below.
KNOWN = {'J1': (-1, (0, 0, 1)), 'J2': (+1, (0, 1, 0)), 'J3': (-1, (-1, 0, 0)),
         'J4': (-1, (-1, 0, 0)), 'J5': (+1, (0, 1, 0))}
HINTS = [(0, 0, 1), (0, 0, -1), (0, 1, 0), (0, -1, 0), (1, 0, 0), (-1, 0, 0)]


# ---------------- frames ----------------

def module_frame(occs, name):
    """(R, t_mm) of an exo module occurrence in armgen's module-frame convention.

    The right arm is a Fusion MIRROR of the left (EXO-50), so the occurrence's X and
    Z columns are negated: that is the 180 deg turn about Y which puts +Z out of the
    rotor face and +Y on the actuator connector side (EXO-23)."""
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
    tr = sum(sum(Ra[i][k] * Rb[i][k] for i in range(3)) for k in range(3))
    return math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))


def best_placement(lab, joints, by_child, exo):
    best = None
    for sign in (+1, -1):
        for hint in HINTS:
            try:
                R, t = g.module_frame(JOINT_NAMES[lab], joints, by_child, sign, hint)
            except ValueError:
                continue
            e = orient_err(R, exo[lab][0])
            if best is None or e < best[0]:
                best = (e, sign, hint)
    return best


# ---------------- STEP helpers (as measure_link4_profile.py) ----------------

def circular_edges(step, pd, T_occ, R, o):
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
    out = []
    for eid, T_loc in step._walk_geom(pd, {'ADVANCED_FACE'}):
        a = step.args(eid)
        sid = a[2]
        st = step.typ(sid)
        sa = step.args(sid)
        if st == 'PLANE':
            P = steplib.mul(T_occ, steplib.mul(T_loc, step.placement(sa[1])))
            out.append(('plane', {'p': to_frame(R, o, [P[0][3], P[1][3], P[2][3]]),
                                  'n': to_dir(R, [P[0][2], P[1][2], P[2][2]])}))
        elif st == 'CYLINDRICAL_SURFACE':
            P = steplib.mul(T_occ, steplib.mul(T_loc, step.placement(sa[1])))
            out.append(('cyl', {'c': to_frame(R, o, [P[0][3], P[1][3], P[2][3]]),
                                'ax': to_dir(R, [P[0][2], P[1][2], P[2][2]]), 'r': sa[2]}))
        else:
            out.append((st.lower(), {}))
    return out


def face_extents(step, pd, T_occ, R, o, u, w):
    """[(kind, key, n_verts, bbox in (s,w,z))] for every ADVANCED_FACE.

    A plane's normal and offset say what the part is made of; the face's own loop
    vertices say WHERE that plane is material.  Reading a hand-built join needs both.
    """
    def collect(eid, seen):
        out = []
        stack = [eid]
        while stack:
            e = stack.pop()
            if e in seen:
                continue
            seen.add(e)
            t = step.typ(e)
            if t == 'VERTEX_POINT':
                out.append(step.point(step.args(e)[1]))
                continue
            if t in ('CARTESIAN_POINT', 'DIRECTION'):
                continue
            for r in steplib._refs(step.ents[e][1]):
                stack.append(r)
        return out

    def lf(v):
        return (v[0] * u[0] + v[1] * u[1], v[0] * w[0] + v[1] * w[1], v[2])

    rows = []
    for eid, T_loc in step._walk_geom(pd, {'ADVANCED_FACE'}):
        a = step.args(eid)
        sid = a[2]
        st = step.typ(sid)
        sa = step.args(sid)
        pts = [lf(to_frame(R, o, steplib.apply(T_occ, steplib.apply(T_loc, q))))
               for q in collect(eid, set())]
        if not pts:
            continue
        bb = tuple((min(q[i] for q in pts), max(q[i] for q in pts)) for i in range(3))
        if st == 'PLANE':
            P = steplib.mul(T_occ, steplib.mul(T_loc, step.placement(sa[1])))
            n = lf(to_dir(R, [P[0][2], P[1][2], P[2][2]]))
            pp = lf(to_frame(R, o, [P[0][3], P[1][3], P[2][3]]))
            off = sum(n[i] * pp[i] for i in range(3))
            if off < 0 or (abs(off) < 1e-9 and min(n) < 0):
                n, off = tuple(-v for v in n), -off
            key = 'n (%6.3f,%6.3f,%6.3f) off %9.3f' % (n + (off,))
        elif st == 'CYLINDRICAL_SURFACE':
            P = steplib.mul(T_occ, steplib.mul(T_loc, step.placement(sa[1])))
            ax = lf(to_dir(R, [P[0][2], P[1][2], P[2][2]]))
            c = lf(to_frame(R, o, [P[0][3], P[1][3], P[2][3]]))
            if sum(ax) < 0:
                ax = tuple(-v for v in ax)
            key = 'r %7.3f ax (%5.2f,%5.2f,%5.2f) c (%7.3f,%7.3f,%7.3f)' % ((sa[2],) + ax + c)
        else:
            key = st.lower()
        rows.append((st.lower(), key, len(pts), bb))
    return rows


def module_parts(step, occs, mod_name):
    """[(name, [world vertices])] of every leaf under an exo `Module k R` occurrence."""
    out = []
    for path, name, pd, T in occs:
        if mod_name not in path or name == mod_name:
            continue
        vs = step.vertices(pd)
        if not vs:
            continue
        out.append((name, [steplib.apply(T, v) for v in vs]))
    return out


# ---------------- part 1: the two module poses ----------------

def part1(step, occs, exo, joints, by_child):
    print('=' * 78)
    print('PART 1  Module 5 R / Module 6 R against URDF J6 / J7')
    print('=' * 78)
    print()
    print('--- exo module frames (armgen convention), whole right arm ---')
    for lab in sorted(exo):
        R, t = exo[lab]
        print('%-3s %-12s pos (%8.3f,%8.3f,%8.3f)  Z=(%6.3f,%6.3f,%6.3f) '
              'Y=(%6.3f,%6.3f,%6.3f)'
              % (lab, MODULES[lab], t[0], t[1], t[2],
                 R[0][2], R[1][2], R[2][2], R[0][1], R[1][1], R[2][1]))

    place = {}
    print()
    for lab in ('J6', 'J7'):
        print('--- %s (%s) placement candidates vs exo %s ---'
              % (lab, JOINT_NAMES[lab], MODULES[lab]))
        rows = []
        for sign in (+1, -1):
            for hint in HINTS:
                try:
                    R, _t = g.module_frame(JOINT_NAMES[lab], joints, by_child, sign, hint)
                except ValueError:
                    continue
                rows.append((orient_err(R, exo[lab][0]), sign, hint))
        for e, sign, hint in sorted(rows):
            print('    axis_sign %+d  y_hint %-11s  err %9.2e rad (%8.4f deg)'
                  % (sign, str(hint), e, math.degrees(e)))
        e, sign, hint = sorted(rows)[0]
        place[lab] = (sign, hint)
        print('    -> axis_sign %+d, y_hint %s, orientation err %.3e rad' % (sign, str(hint), e))
        print()

    # world poses, shifted so URDF J1 sits on exo J1 (EXO-113's comparison)
    urdf = {}
    for lab in sorted(exo):
        sign, hint = KNOWN.get(lab, place.get(lab, (1, (0, 0, 1))))
        R, t = g.module_frame(JOINT_NAMES[lab], joints, by_child, sign, hint)
        urdf[lab] = (R, [v * 1000.0 for v in t])
    shift = [exo['J1'][1][i] - urdf['J1'][1][i] for i in range(3)]
    print('--- world poses, URDF shifted by (%.3f, %.3f, %.3f) so J1 lands on exo J1 ---'
          % tuple(shift))
    print('%-3s %-26s %-26s %-24s %8s %9s %8s'
          % ('', 'URDF (shifted, mm)', 'exo (mm)', 'exo - URDF', '|d|', 'along Z', 'perp'))
    resid = {}
    for lab in sorted(exo):
        tu = [urdf[lab][1][i] + shift[i] for i in range(3)]
        te = exo[lab][1]
        d = [te[i] - tu[i] for i in range(3)]
        Z = [exo[lab][0][i][2] for i in range(3)]
        ax = sum(d[i] * Z[i] for i in range(3))
        perp = math.sqrt(max(0.0, sum(v * v for v in d) - ax * ax))
        resid[lab] = (d, ax, perp)
        print('%-3s (%7.3f,%8.3f,%8.3f) (%7.3f,%8.3f,%8.3f) (%6.3f,%7.3f,%6.3f) %8.3f %9.3f %8.3f'
              % (lab, tu[0], tu[1], tu[2], te[0], te[1], te[2], d[0], d[1], d[2],
                 math.sqrt(sum(v * v for v in d)), ax, perp))
        print('    orientation err vs exo: %.3e rad (%.4f deg)'
              % (orient_err(urdf[lab][0], exo[lab][0]),
                 math.degrees(orient_err(urdf[lab][0], exo[lab][0]))))

    # the inherited EXO-113 offset, and what is left over at the wrist
    print()
    print('--- the EXO-113 inherited offset removed (J5 is the reference: it carries it) ---')
    base = resid['J5'][0]
    print('inherited offset (from J5) = (%.3f, %.3f, %.3f), |.| %.3f mm'
          % (base[0], base[1], base[2], math.sqrt(sum(v * v for v in base))))
    for lab in ('J6', 'J7'):
        d = resid[lab][0]
        r = [d[i] - base[i] for i in range(3)]
        Z = [exo[lab][0][i][2] for i in range(3)]
        ax = sum(r[i] * Z[i] for i in range(3))
        perp = math.sqrt(max(0.0, sum(v * v for v in r) - ax * ax))
        print('%-3s residual (%7.3f,%7.3f,%7.3f)  |.| %7.3f   ALONG ITS OWN AXIS %+8.3f  '
              'perpendicular %7.3f' % (lab, r[0], r[1], r[2],
                                       math.sqrt(sum(v * v for v in r)), ax, perp))

    # the pair, in both frames
    print()
    for a, b in (('J5', 'J6'), ('J6', 'J7')):
        for src, frames in (('URDF', urdf), ('exo ', exo)):
            RA, tA = frames[a]
            RB, tB = frames[b]
            R_ab, t_ab = g.relative_frame(RA, [v / 1000.0 for v in tA],
                                          RB, [v / 1000.0 for v in tB])
            num = g.landing_numbers(R_ab, t_ab)
            print('--- pair %s -> %s, %s ---' % (a, b, src))
            print('    t_ab (mm) = (%.3f, %.3f, %.3f)   |t| %.3f' % tuple(list(t_ab) + [num['dist']]))
            print('    R_ab = %s' % '; '.join('(%.4f,%.4f,%.4f)' % tuple(r) for r in R_ab))
            for k in ('axis_angle_deg', 'axial', 'lateral', 'zb_in_plane', 'yb_axial',
                      'snap_rad', 'cut_reach', 's_land', 'zL', 'w_off', 'y_sign', 'u_sign'):
                if k in num:
                    print('      %-14s %10.4f' % (k, num[k]))
            if 'u' in num:
                print('      u = %s  w = %s' % ([round(v, 4) for v in num['u']],
                                                [round(v, 4) for v in num['w']]))
        print()

    # ---- the real clearance, from the module bodies ----
    print('--- module-to-module clearance, from the BODIES (not the r 27 cylinder) ---')
    for lab in ('J5', 'J6', 'J7'):
        R, o = exo[lab]
        parts = module_parts(step, occs, MODULES[lab])
        allv = [to_frame(R, o, p) for _n, ps in parts for p in ps]
        print('  %-3s %-12s %d leaf bodies, %d vertices, in its own module frame:'
              % (lab, MODULES[lab], len(parts), len(allv)))
        for nm, ps in parts:
            q = [to_frame(R, o, p) for p in ps]
            rr = [math.hypot(v[0], v[1]) for v in q]
            zz = [v[2] for v in q]
            print('      %-26s r %7.3f..%7.3f  z %8.3f..%8.3f' % (nm, min(rr), max(rr), min(zz), max(zz)))
        rr = [math.hypot(v[0], v[1]) for v in allv]
        zz = [v[2] for v in allv]
        print('      %-26s r %7.3f..%7.3f  z %8.3f..%8.3f' % ('MODULE TOTAL', min(rr), max(rr), min(zz), max(zz)))

    print()
    for a, b in (('J5', 'J6'), ('J6', 'J7')):
        pa = [p for _n, ps in module_parts(step, occs, MODULES[a]) for p in ps]
        pb = [p for _n, ps in module_parts(step, occs, MODULES[b]) for p in ps]
        best = min((sum((x[i] - y[i]) ** 2 for i in range(3)), x, y) for x in pa for y in pb)
        print('  %s <-> %s : closest vertex pair %.3f mm   at (%.2f,%.2f,%.2f) / (%.2f,%.2f,%.2f)'
              % (a, b, math.sqrt(best[0]), best[1][0], best[1][1], best[1][2],
                 best[2][0], best[2][1], best[2][2]))
        # axial-extent view: module A's vertices in module B's frame
        RB, oB = exo[b]
        q = [to_frame(RB, oB, p) for p in pa]
        rr = [math.hypot(v[0], v[1]) for v in q]
        zz = [v[2] for v in q]
        print('      %s in %s frame: r %.3f..%.3f  z %.3f..%.3f (B itself r<=%.3f, z %.3f..%.3f)'
              % (MODULES[a], MODULES[b], min(rr), max(rr), min(zz), max(zz),
                 max(math.hypot(v[0], v[1]) for v in [to_frame(RB, oB, p) for p in pb]),
                 min(v[2] for v in [to_frame(RB, oB, p) for p in pb]),
                 max(v[2] for v in [to_frame(RB, oB, p) for p in pb])))
    return urdf, place


# ---------------- part 2: the two links ----------------

def part2(step, occs, exo, link, a, b, urdf, use_exo=True):
    frames = exo if use_exo else urdf
    RA, tA = frames[a]
    RB, tB = frames[b]
    R_ab, t_ab = g.relative_frame(RA, [v / 1000.0 for v in tA], RB, [v / 1000.0 for v in tB])
    num = g.landing_numbers(R_ab, t_ab)
    occ = [o for o in occs if o[1] == link]
    if not occ:
        raise SystemExit('%s not found' % link)
    _path, _n, pd, T = occ[0]
    verts = [to_frame(RA, tA, steplib.apply(T, v)) for v in step.vertices(pd)]

    print('=' * 78)
    print('PART 2  %s  in the %s rotor-pad frame A (exo %s, turned 180 about Y)'
          % (link, a, MODULES[a]))
    print('=' * 78)
    u = num.get('u', [1.0, 0.0, 0.0])
    w = num.get('w', [0.0, 1.0, 0.0])
    print('u_sign %+.0f   u = %s   w = %s' % (num.get('u_sign', 1.0),
                                              [round(v, 4) for v in u], [round(v, 4) for v in w]))
    print('vertices %d  solids %d   x %.3f..%.3f  y %.3f..%.3f  z %.3f..%.3f'
          % (len(verts), len(step.solids(pd)),
             min(v[0] for v in verts), max(v[0] for v in verts),
             min(v[1] for v in verts), max(v[1] for v in verts),
             min(v[2] for v in verts), max(v[2] for v in verts)))

    def swz(p):
        return (p[0] * u[0] + p[1] * u[1], p[0] * w[0] + p[1] * w[1], p[2])

    print()
    print('--- circular edges, largest radius first (centre s,w,z / axis / r / ends) ---')
    edges = circular_edges(step, pd, T, RA, tA)
    for c, ax, r, p0, p1 in sorted(edges, key=lambda e: -e[2]):
        sc = swz(c)
        axl = (ax[0] * u[0] + ax[1] * u[1], ax[0] * w[0] + ax[1] * w[1], ax[2])
        e0, e1 = swz(p0), swz(p1)
        print('  r %7.3f  c (s%8.3f, w%8.3f, z%8.3f) ax (%5.2f,%5.2f,%5.2f)  '
              '(%7.2f,%7.2f,%7.2f) -> (%7.2f,%7.2f,%7.2f)'
              % (r, sc[0], sc[1], sc[2], axl[0], axl[1], axl[2],
                 e0[0], e0[1], e0[2], e1[0], e1[1], e1[2]))

    print()
    print('--- side elevation: unique (s, z) per lateral band w ---')
    bands = defaultdict(set)
    for v in verts:
        s, wv, z = swz(v)
        bands[round(wv, 1)].add((round(s, 3), round(z, 3)))
    for k in sorted(bands):
        print('  w = %8.2f : %s' % (k, sorted(bands[k])))

    print()
    print('--- plan view: unique (s, w) per z band ---')
    zb = defaultdict(set)
    for v in verts:
        s, wv, z = swz(v)
        zb[round(z, 1)].add((round(s, 3), round(wv, 3)))
    for k in sorted(zb):
        print('  z = %8.2f : %s' % (k, sorted(zb[k])))

    print()
    print('--- planar faces (normal, offset) and cylinders, in (s, w, z) ---')
    fs = faces(step, pd, T, RA, tA)

    def lf(v):
        return (v[0] * u[0] + v[1] * u[1], v[0] * w[0] + v[1] * w[1], v[2])
    planes, cyls, other = {}, {}, {}
    for kind, d in fs:
        if kind == 'plane':
            n, p = lf(d['n']), lf(d['p'])
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
        print('    r %8.4f ax (%5.2f,%5.2f,%5.2f) through (s%8.3f, w%8.3f, z%8.3f)  x%d'
              % ((key[0],) + key[1] + key[2] + (cyls[key],)))
    if other:
        print('  other surfaces: %s' % other)

    print()
    print('--- every face with the (s, w, z) box its own loop vertices occupy ---')
    for kind, key, n, bb in sorted(face_extents(step, pd, T, RA, tA, u, w),
                                   key=lambda r: (r[0], r[1])):
        print('  %-10s %-52s v%-3d  s %8.3f..%8.3f  w %8.3f..%8.3f  z %8.3f..%8.3f'
              % (kind, key, n, bb[0][0], bb[0][1], bb[1][0], bb[1][1], bb[2][0], bb[2][1]))

    # where the link lands: module B in frame A
    print()
    print('--- module %s (the stator end) in this frame A ---' % b)
    RBf, oBf = frames[b]
    zb_dir = [R_ab[i][2] for i in range(3)]
    print('  B origin (s%8.3f, w%8.3f, z%8.3f)   Z_B (s%6.3f, w%6.3f, z%6.3f)'
          % (swz(t_ab) + tuple(lf(zb_dir))))
    parts = module_parts(step, occs, MODULES[b])
    q = [swz(to_frame(RA, tA, p)) for _n, ps in parts for p in ps]
    print('  body extent  s %.3f..%.3f  w %.3f..%.3f  z %.3f..%.3f'
          % (min(v[0] for v in q), max(v[0] for v in q), min(v[1] for v in q),
             max(v[1] for v in q), min(v[2] for v in q), max(v[2] for v in q)))
    print()
    return num, verts, edges, (u, w)


# ---------------- part 3: the clearance the slide buys, and the rule ----------------

# The module body as a profile of revolution about its own axis, read off the leaf bodies
# in part 1 (radius per axial band, module frame, mm). EXO-163 modelled a module as an
# INFINITE r 27 cylinder; it is not - it is 35 mm long, and that is the whole difference.
MODULE_PROFILE = [(-35.000, -24.501, 26.101),    # XL330 body
                  (-24.501, -0.400, 27.001),     # the module shell (the r 27 EXO-163 used)
                  (-0.400, 0.000, 18.000)]       # the rotor face


def _profile_dist(rho, zeta, prof):
    """Distance from (rho, zeta) to the solid of revolution `prof`, 0 inside."""
    best = 1e9
    for z0, z1, r in prof:
        dz = max(z0 - zeta, 0.0, zeta - z1)
        dr = max(rho - r, 0.0)
        best = min(best, math.hypot(dz, dr))
    return best


def module_gap(R_ab, t_ab, prof=MODULE_PROFILE, n_theta=360, n_z=141):
    """Smallest distance between module A (frame A) and module B (at R_ab, t_ab), mm.

    A's surface is sampled; every sample is taken to B's frame and measured against B's
    profile.  Negative is not produced - 0.0 means they interfere."""
    zs = [prof[0][0] + i * (prof[-1][1] - prof[0][0]) / (n_z - 1.0) for i in range(n_z)]
    RT = [[R_ab[k][i] for k in range(3)] for i in range(3)]      # transpose
    best, where = 1e9, None
    for zi in zs:
        r = max(rr for z0, z1, rr in prof if z0 - 1e-9 <= zi <= z1 + 1e-9)
        for k in range(n_theta):
            a = 2.0 * math.pi * k / n_theta
            p = [r * math.cos(a), r * math.sin(a), zi]
            d = [p[i] - t_ab[i] for i in range(3)]
            q = [sum(RT[i][j] * d[j] for j in range(3)) for i in range(3)]
            g = _profile_dist(math.hypot(q[0], q[1]), q[2], prof)
            if g < best:
                best, where = g, (p, q)
    return best, where


def part3(exo, urdf):
    print('=' * 78)
    print('PART 3  what the slide buys, and the rule behind it')
    print('=' * 78)
    for a, b in (('J5', 'J6'), ('J6', 'J7')):
        for src, frames in (('URDF (no slide)', urdf), ('exo  (as built)', exo)):
            RA, tA = frames[a]
            RB, tB = frames[b]
            R_ab, t_ab = g.relative_frame(RA, [v / 1000.0 for v in tA],
                                          RB, [v / 1000.0 for v in tB])
            gap, _w = module_gap(R_ab, t_ab)
            print('  %s -> %s  %-16s t_ab (%8.3f,%8.3f,%8.3f)  module gap %8.3f mm'
                  % (a, b, src, t_ab[0], t_ab[1], t_ab[2], gap))
        print()

    # the slide, swept: how far module J6 has to move along its own axis before its body
    # leaves module J7's
    RA, tA = urdf['J6']
    RB, tB = urdf['J7']
    R_ab, t_ab = g.relative_frame(RA, [v / 1000.0 for v in tA], RB, [v / 1000.0 for v in tB])
    print('  slide of module J6 along its own axis vs the J6/J7 module gap')
    print('  (sliding J6 along its own axis moves the pair by +slide along Z_A)')
    prev = None
    for slide in [0, 5, 10, 15, 17, 18, 19, 19.5, 20, 20.5, 21, 22, 23, 24, 25, 30]:
        t = [t_ab[0], t_ab[1], t_ab[2] + slide]
        gap, _w = module_gap(R_ab, t, n_theta=180, n_z=71)
        mark = ''
        if prev is not None and prev <= 0.0 < gap:
            mark = '   <- first clearance'
        print('    slide %6.2f mm  ->  gap %7.3f mm%s' % (slide, gap, mark))
        prev = gap
    print()
    print('  measured slide (exo Module 5 R, from part 1): 23.000 mm = half the 46.00 mm')
    print('  J6->J7 spacing. Link 5 R pays for it with 23.000 mm of extra run (s_land')
    print('  32.500 -> 55.500); Link 6 R is paid 23.000 mm of axial offset, which turns')
    print('  its nominal bar drop zL from -35.350 to -12.350 and lets it be a flat L.')


def main():
    step = steplib.Step(EXO)
    occs = step.occurrences()
    exo = dict((lab, module_frame(occs, nm)) for lab, nm in MODULES.items())
    joints, by_child = g.parse_urdf(G1_URDF)
    urdf, _place = part1(step, occs, exo, joints, by_child)
    print()
    part3(exo, urdf)
    print()
    part2(step, occs, exo, 'Link 5 R', 'J5', 'J6', urdf)
    part2(step, occs, exo, 'Link 6 R', 'J6', 'J7', urdf)


if __name__ == '__main__':
    main()
