#!/usr/bin/env python3
"""Measure holes / counterbores / hex pockets in a 3MF mesh (EXO-291).

    python3 -I tools/measure_holes.py parts/manual_inner_top.3mf [r:deg ...]

Default probes: r 11 at 0/90/180/270 and r 6 at 45/135/225/315. For each probe, the
vertices within 4 mm of the hole axis are grouped by z; the ring at each z level is
reported as: vertex count, fitted centre, min/max radius. A round hole gives min~=max
(diameter = 2r); a hex gives corner radius R (max) -> across-corners 2R, across-flats
sqrt(3) R, and 6 vertices at 60 deg spacing. Wall z-ranges come from vertical faces.
"""
import math
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_3mf import load, apply  # noqa: E402


def fit_circle(pts):
    """Kasa least-squares circle: returns cx, cy, r."""
    n = len(pts)
    sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts)
    sxx = sum(p[0] ** 2 for p in pts); syy = sum(p[1] ** 2 for p in pts)
    sxy = sum(p[0] * p[1] for p in pts)
    sxz = sum(p[0] * (p[0] ** 2 + p[1] ** 2) for p in pts)
    syz = sum(p[1] * (p[0] ** 2 + p[1] ** 2) for p in pts)
    sz = sum(p[0] ** 2 + p[1] ** 2 for p in pts)
    # solve [sxx sxy sx; sxy syy sy; sx sy n][a b c] = [sxz syz sz]
    M = [[sxx, sxy, sx, sxz], [sxy, syy, sy, syz], [sx, sy, n, sz]]
    for i in range(3):
        piv = max(range(i, 3), key=lambda k: abs(M[k][i])); M[i], M[piv] = M[piv], M[i]
        for k in range(3):
            if k != i:
                f = M[k][i] / M[i][i]
                M[k] = [M[k][j] - f * M[i][j] for j in range(4)]
    a, b, c = (M[i][3] / M[i][i] for i in range(3))
    cx, cy = a / 2, b / 2
    return cx, cy, math.sqrt(c + cx * cx + cy * cy)


def probe(vs, ts, x0, y0, rad=4.0):
    near = {}
    for x, y, z in vs:
        if math.hypot(x - x0, y - y0) < rad:
            near.setdefault(round(z, 3), []).append((x, y))
    out = []
    for z in sorted(near):
        pts = near[z]
        if len(pts) < 5:
            out.append((z, len(pts), None)); continue
        cx, cy, r = fit_circle(pts)
        ds = [math.hypot(p[0] - cx, p[1] - cy) for p in pts]
        out.append((z, len(pts), (cx, cy, r, min(ds), max(ds))))
    return out


def main():
    path = sys.argv[1]
    _, _, scale, _, objs, items = load(path)
    vs, ts = [], []
    for oid, t in items:
        _, ov, ot = objs[oid]
        ts += [(a + len(vs), b + len(vs), c + len(vs)) for a, b, c in ot]
        vs += [tuple(scale * k for k in apply(t, p)) for p in ov]
    probes = [tuple(map(float, a.split(':'))) for a in sys.argv[2:]] or \
        [(11, d) for d in (0, 90, 180, 270)] + [(6, d) for d in (45, 135, 225, 315)]
    for r, d in probes:
        x0, y0 = r * math.cos(math.radians(d)), r * math.sin(math.radians(d))
        print('== hole r %.1f @ %.0f deg  (%.3f, %.3f)' % (r, d, x0, y0))
        for z, n, f in probe(vs, ts, x0, y0):
            if f is None:
                print('  z %7.3f  n %3d' % (z, n)); continue
            cx, cy, rr, mn, mx = f
            hexish = mx - mn > 0.05
            extra = ('  HEX? AC %.3f AF(sqrt3 R) %.3f' % (2 * mx, math.sqrt(3) * mx)
                     if hexish else '')
            print('  z %7.3f  n %3d  ctr (%.3f,%.3f)  D %.3f  rmin %.3f rmax %.3f%s'
                  % (z, n, cx, cy, 2 * rr, mn, mx, extra))


if __name__ == '__main__':
    main()
