#!/usr/bin/env python3
"""Measure exo `Link 2 R`'s side profile (EXO-58) and the generated link_23's step.

Not a bridge script - plain python3, run from the repo root:
    /usr/bin/python3 tools/bridge_scripts/measure_link2_profile.py

Everything is reported in the J2R MODULE frame (EXO-28): origin (0, -240.772, -2.791),
X = (0, 0, -1) (along the bar, toward J3), Y = (0, 1, 0) (bar width), Z = (1, 0, 0)
(J2 axis, rotor face normal). Side profile = (x, z); width = y.
"""
import os
import sys
import math
from collections import defaultdict

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import steplib  # noqa: E402

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
EXO = os.path.join(REPO, 'gento_luna/exo/exports/exo_skeleton_2026-08-08.step')
GEN = os.path.join(REPO, 'gento_luna/exo/exports/generated_R_arm_only.step')

J2R_O = [0.0, -240.772, -2.791]
J2R_R = [[0, 0, -1], [0, 1, 0], [1, 0, 0]]     # rows = X, Y, Z of the frame (R^T)


def to_frame(p, o=J2R_O, RT=J2R_R):
    d = [p[i] - o[i] for i in range(3)]
    return [sum(RT[i][k] * d[k] for k in range(3)) for i in range(3)]


def to_frame_dir(v, RT=J2R_R):
    return [sum(RT[i][k] * v[k] for k in range(3)) for i in range(3)]


def r3(v):
    return tuple(round(x, 2) for x in v)


def profile(step, name, o, RT, label):
    occs = step.find(name)
    if not occs:
        print('!! %s not found in %s' % (name, step.path))
        return
    path, _, pd, T = occs[0]
    print('=== %s (%s) ===' % (label, ' > '.join(path)))
    verts = [to_frame(steplib.apply(T, v), o, RT) for v in step.vertices(pd)]
    xs, ys, zs = [v[0] for v in verts], [v[1] for v in verts], [v[2] for v in verts]
    print('vertices: %d  x %.2f..%.2f  y %.2f..%.2f  z %.2f..%.2f' % (
        len(verts), min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)))

    # widths: y extent of vertices grouped by x-band (1 mm) - shows where the part is 25 vs wider
    print('-- y extent (width) per x band --')
    bands = defaultdict(list)
    for v in verts:
        bands[int(math.floor(v[0] / 5.0)) * 5].append(v)
    for b in sorted(bands):
        vs = bands[b]
        print('  x %4d..%4d  y %6.2f..%6.2f   z %6.2f..%6.2f  (%d verts)' % (
            b, b + 5, min(v[1] for v in vs), max(v[1] for v in vs),
            min(v[2] for v in vs), max(v[2] for v in vs), len(vs)))

    # profile arcs: cylindrical surfaces whose axis is along the width (y)
    print('-- cylinders with axis || Y (profile arcs), centre (x, z), radius --')
    seen = set()
    for c, ax, r in step.cylinders(pd):
        cw = to_frame(steplib.apply(T, c), o, RT)
        aw = to_frame_dir(steplib.rot(T, ax), RT)
        if abs(aw[1]) > 0.99:
            key = (round(cw[0], 2), round(cw[2], 2), round(r, 2))
            if key in seen:
                continue
            seen.add(key)
            print('  centre (x=%7.2f, z=%7.2f)  r %6.2f' % key)
    print('-- cylinders with axis || Z (pad / holes about the J2 axis) --')
    seen = set()
    for c, ax, r in step.cylinders(pd):
        cw = to_frame(steplib.apply(T, c), o, RT)
        aw = to_frame_dir(steplib.rot(T, ax), RT)
        if abs(aw[2]) > 0.99:
            key = (round(cw[0], 2), round(cw[1], 2), round(r, 2))
            if key in seen:
                continue
            seen.add(key)
            print('  centre (x=%7.2f, y=%7.2f)  r %6.2f' % key)
    print('-- cylinders with axis || X (slab / inserts about the J3 axis) --')
    seen = set()
    for c, ax, r in step.cylinders(pd):
        cw = to_frame(steplib.apply(T, c), o, RT)
        aw = to_frame_dir(steplib.rot(T, ax), RT)
        if abs(aw[0]) > 0.99:
            key = (round(cw[1], 2), round(cw[2], 2), round(r, 2))
            if key in seen:
                continue
            seen.add(key)
            print('  centre (y=%7.2f, z=%7.2f)  r %6.2f' % key)

    # side-elevation vertices in the bar corridor (|y| <= 13), unique (x, z)
    print('-- side-elevation vertices in the corridor |y| <= 13, unique (x, z), x > 20 --')
    pts = sorted(set((round(v[0], 2), round(v[2], 2)) for v in verts if abs(v[1]) <= 13.0 and v[0] > 20.0))
    for p in pts:
        print('  (%7.2f, %7.2f)' % p)
    # width at the riser / bar (vertices with x in the riser and bar ranges)
    for lab, lo, hi in (('pad extension', 20, 27.5), ('riser', 27.5, 36), ('bend', 36, 48),
                        ('bar', 48, 70), ('flange', 70.5, 79)):
        ws = sorted(set(round(v[1], 2) for v in verts if lo <= v[0] <= hi))
        print('  %-14s x %4.1f..%4.1f  y values: %s' % (lab, lo, hi, ws))
    return verts


def frame_of_occurrence(step, name_prefix):
    for p, name, pd, T in step.occurrences():
        if name.startswith(name_prefix):
            o = [T[0][3], T[1][3], T[2][3]]
            RT = [[T[0][0], T[1][0], T[2][0]], [T[0][1], T[1][1], T[2][1]], [T[0][2], T[1][2], T[2][2]]]
            return o, RT, name
    return None, None, None


def main():
    s = steplib.Step(EXO)
    # cross-check the frame against Module 1 R's own placement
    for p, name, pd, T in s.occurrences():
        if name == 'Module 1 R':
            print('Module 1 R occurrence: pos %s  Zcol %s' % (r3([T[0][3], T[1][3], T[2][3]]), r3([T[0][2], T[1][2], T[2][2]])))
    profile(s, 'Link 2 R', J2R_O, J2R_R, 'exo Link 2 R in the J2R module frame')

    if os.path.isfile(GEN):
        # Unsaved-document exports carry timestamp names: the J2 wrapper is the depth-2
        # occurrence at world y ~ -240.77 and link_23 the identity-placed product whose
        # geometry reaches toward J3 (world z < -50). Axes are the generator's J2 frame
        # (test_armgen: X=(0,0,-1), Y=(0,1,0), Z=(1,0,0)), same as the exo J2R frame.
        g = steplib.Step(GEN)
        occs = g.occurrences()
        o = None
        for p, name, pd, T in occs:
            if len(p) == 2 and abs(T[1][3] + 240.77) < 0.1:
                o = [T[0][3], T[1][3], T[2][3]]
                wrapper = name
        if o is None:
            print('!! no J2 wrapper (world y -240.77) in %s' % GEN)
            return
        print()
        print('generated J2 wrapper %s: origin %s (axes X=(0,0,-1) Y=(0,1,0) Z=(1,0,0))' % (wrapper, r3(o)))
        for p, name, pd, T in occs:
            if len(p) == 2 and all(abs(T[i][3]) < 1e-6 for i in range(3)):
                vs = [steplib.apply(T, v) for v in g.vertices(pd)]
                if vs and min(v[2] for v in vs) < -50:
                    profile(g, name, o, J2R_R, 'generated link_23 (last export, %s) in the J2 frame' % name)
    else:
        print('(no %s - skip generated link_23)' % GEN)


if __name__ == '__main__':
    main()
