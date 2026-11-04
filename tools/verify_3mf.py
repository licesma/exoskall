#!/usr/bin/env python3
"""Read a 3MF back with the standard library and report what is in it (EXO-265).

    python3 -I tools/verify_3mf.py parts/*.3mf

Per file: objects, unit, build-item transforms, vertex/triangle counts, bbox (after the
build transform), signed mesh volume, edge check (watertight = every edge shared by
exactly 2 triangles), connected shells and genus per shell from Euler's formula
(genus = number of through-holes of a closed shell), and vertical through-columns:
a ray along z at (r, angle) that crosses no triangle is an open hole at that spot.
"""
import math
import re
import sys
import zipfile
import xml.etree.ElementTree as ET

NS = {'m': 'http://schemas.microsoft.com/3dmanufacturing/core/2015/02'}
UNIT = {'micron': 0.001, 'millimeter': 1.0, 'centimeter': 10.0, 'inch': 25.4,
        'foot': 304.8, 'meter': 1000.0}


def parse_transform(s):
    if not s:
        return None
    a = [float(x) for x in s.split()]
    return a  # m00 m01 m02 m10 m11 m12 m20 m21 m22 m30 m31 m32 (row vectors)


def apply(t, p):
    if t is None:
        return p
    x, y, z = p
    return (x * t[0] + y * t[3] + z * t[6] + t[9],
            x * t[1] + y * t[4] + z * t[7] + t[10],
            x * t[2] + y * t[5] + z * t[8] + t[11])


PNS = '{http://schemas.microsoft.com/3dmanufacturing/production/2015/06}path'


def mul(a, b):
    """Compose two 3MF transforms: apply a, then b."""
    if a is None:
        return b
    if b is None:
        return a
    o = apply(b, (0.0, 0.0, 0.0))
    rows = []
    for i in range(3):
        p = apply(b, apply(a, tuple(1.0 if k == i else 0.0 for k in range(3))))
        p0 = apply(b, apply(a, (0.0, 0.0, 0.0)))
        rows += [p[k] - p0[k] for k in range(3)]
    p0 = apply(b, apply(a, (0.0, 0.0, 0.0)))
    return rows + list(p0)


def load(path):
    """Objects as (name, vertices, triangles); component objects (Bambu Studio, 3MF
    production extension p:path) are flattened into one mesh."""
    z = zipfile.ZipFile(path)
    model = [n for n in z.namelist() if n.lower().endswith('.model')]
    main = '3D/3dmodel.model' if '3D/3dmodel.model' in model else model[0]
    trees = {}

    def tree(part):
        part = part.lstrip('/')
        if part not in trees:
            trees[part] = ET.fromstring(z.read(part))
        return trees[part]

    def mesh_of(part, oid, t):
        for o in tree(part).findall('m:resources/m:object', NS):
            if o.get('id') != oid:
                continue
            mesh = o.find('m:mesh', NS)
            if mesh is not None:
                vs = [apply(t, (float(v.get('x')), float(v.get('y')), float(v.get('z'))))
                      for v in mesh.findall('m:vertices/m:vertex', NS)]
                ts = [(int(q.get('v1')), int(q.get('v2')), int(q.get('v3')))
                      for q in mesh.findall('m:triangles/m:triangle', NS)]
                return o.get('name'), vs, ts
            vs, ts = [], []
            for c in o.findall('m:components/m:component', NS):
                _, cv, ct = mesh_of(c.get(PNS) or part, c.get('objectid'),
                                    mul(parse_transform(c.get('transform')), t))
                ts += [(a + len(vs), b + len(vs), d + len(vs)) for a, b, d in ct]
                vs += cv
            return o.get('name'), vs, ts
        raise KeyError('object %s not in %s' % (oid, part))

    root = tree(main)
    scale = UNIT[root.get('unit', 'millimeter')]
    meta = {m.get('name'): (m.text or '') for m in root.findall('m:metadata', NS)}
    objs = {}
    for o in root.findall('m:resources/m:object', NS):
        objs[o.get('id')] = mesh_of(main, o.get('id'), None)
    items = [(i.get('objectid'), parse_transform(i.get('transform')))
             for i in root.findall('m:build/m:item', NS)]
    return z.namelist(), root.get('unit', 'millimeter'), scale, meta, objs, items


def shells(nv, ts):
    parent = list(range(nv))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for a, b, c in ts:
        ra, rb, rc = find(a), find(b), find(c)
        parent[rb] = ra
        parent[find(rc)] = ra
    groups = {}
    for i, t in enumerate(ts):
        groups.setdefault(find(t[0]), []).append(i)
    return list(groups.values())


def column_open(vs, ts, x, y):
    """Number of triangles a vertical line at (x, y) crosses."""
    n = 0
    for a, b, c in ts:
        (x1, y1, _), (x2, y2, _), (x3, y3, _) = vs[a], vs[b], vs[c]
        d = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3)
        if abs(d) < 1e-12:
            continue
        l1 = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / d
        l2 = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / d
        if l1 >= 0 and l2 >= 0 and l1 + l2 <= 1:
            n += 1
    return n


def report(path):
    names, unit, scale, meta, objs, items = load(path)
    print('=== %s' % path)
    print('  parts: %s' % ', '.join(names))
    print('  unit %s; title %r; objects %d; build items %d'
          % (unit, meta.get('Title'), len(objs), len(items)))
    for oid, t in items:
        name, vs0, ts = objs[oid]
        vs = [tuple(c * scale for c in apply(t, p)) for p in vs0]
        lo = [min(p[k] for p in vs) for k in range(3)]
        hi = [max(p[k] for p in vs) for k in range(3)]
        vol = 0.0
        for a, b, c in ts:
            (x1, y1, z1), (x2, y2, z2), (x3, y3, z3) = vs[a], vs[b], vs[c]
            vol += (x1 * (y2 * z3 - y3 * z2) - x2 * (y1 * z3 - y3 * z1)
                    + x3 * (y1 * z2 - y2 * z1)) / 6.0
        edges = {}
        for a, b, c in ts:
            for e in ((a, b), (b, c), (c, a)):
                k = (min(e), max(e))
                edges[k] = edges.get(k, 0) + 1
        bad = sum(1 for n in edges.values() if n != 2)
        print('  object %s %r, transform %s' % (oid, name, t))
        print('    %d vertices, %d triangles, %d edges not shared by exactly 2'
              % (len(vs), len(ts), bad))
        print('    bbox %.3f x %.3f x %.3f  min (%.3f, %.3f, %.3f) max (%.3f, %.3f, %.3f)'
              % (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2], *lo, *hi))
        print('    signed volume %.3f mm3' % vol)
        for s in shells(len(vs), ts):
            sv = set()
            se = set()
            for i in s:
                a, b, c = ts[i]
                sv.update((a, b, c))
                se.update({(min(a, b), max(a, b)), (min(b, c), max(b, c)),
                           (min(c, a), max(c, a))})
            chi = len(sv) - len(se) + len(s)
            print('    shell: %d triangles, Euler %d -> genus (through-holes) %s'
                  % (len(s), chi, (2 - chi) / 2.0))
        cx, cy = (lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0
        print('    vertical columns from bbox centre (%.3f, %.3f): crossings per probe'
              % (cx, cy))
        for r in (0.0, 6.0, 11.0, 12.2):
            row = []
            for deg in range(0, 360, 45):
                x = cx + r * math.cos(math.radians(deg + 0.01))
                y = cy + r * math.sin(math.radians(deg + 0.01))
                row.append('%d' % column_open(vs, ts, x, y))
                if r == 0.0:
                    break
            print('      r %5.1f  at 0,45,..315 deg: %s' % (r, ' '.join(row)))
        if r == 12.2:
            row = []
            for k in range(8):
                deg = 22.5 + 45 * k
                row.append('%d' % column_open(vs, ts, cx + r * math.cos(math.radians(deg)),
                                              cy + r * math.sin(math.radians(deg))))
            print('      r  12.2  at 22.5+k45 deg: %s' % ' '.join(row))


if __name__ == '__main__':
    for p in sys.argv[1:]:
        report(p)
