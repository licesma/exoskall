"""EXO-124: measure the flat seating ring of every screw hole on a BUILT link body.

Importable from any bridge script (it touches nothing at import time):

    import sys, importlib
    sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
    import hole_ring_probe; importlib.reload(hole_ring_probe)
    rings = hole_ring_probe.measure(body, frame, armgen.HOLE_R)

This is the INDEPENDENT half of the EXO-124 gate. `armgen.hole_flat_rings` predicts the ring
from the route's own curves; this measures it on the solid Fusion actually built, and does
not use armgen at all:

  * a screw hole is a cylindrical face of radius HOLE_R;
  * the faces it SEATS on are the planar faces perpendicular to its axis that the hole
    breaks through - found by looking for an edge of that face which is an arc of radius
    HOLE_R about the hole's own axis, so a hole whose circle has been CUT (`Link 4 R`'s is,
    by 0.027 mm) is still recognised;
  * the ring is the distance from the hole's AXIS to the nearest point of every OTHER edge
    of that face, minus HOLE_R. That is signed by construction: an edge crossing inside the
    hole's own radius - the lift-off tangent of EXO-124's defect - gives a NEGATIVE ring,
    which a ray walk outward from the hole edge cannot report at all (the old
    check_hole_margins.py bottomed out at 0.00 and that is how the defect read as zero).

Nothing is sampled blind: the only approximation is the chord tolerance the boundary edges
are stroked at (STROKE_TOL), which bounds the error at 0.005 mm.
"""
import math

import adsk.core  # noqa: F401  (imported for its side effect: this only runs inside Fusion)

RADIUS_TOL = 0.05     # mm: a cylindrical face this close to HOLE_R is a screw hole
AXIS_TOL = 0.999      # |n . axis| above this is 'perpendicular to the hole'
STROKE_TOL = 0.0005   # cm: chord tolerance the boundary edges are stroked at (0.005 mm)
OWN_EDGE_TOL = 0.02   # mm: an edge every point of which is this close to the hole's own
                      # radius IS the hole's own circle (whole or cut) and bounds nothing


def _pt_seg_dist(p, a, b):
    """Distance from a 3D point to a 3D segment."""
    dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    den = dx * dx + dy * dy + dz * dz
    t = 0.0 if den <= 0.0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy
                                                  + (p[2] - a[2]) * dz) / den))
    return math.sqrt((p[0] - (a[0] + t * dx)) ** 2 + (p[1] - (a[1] + t * dy)) ** 2
                     + (p[2] - (a[2] + t * dz)) ** 2)


def _axis_dist(p, origin, axis):
    """Perpendicular distance from a point to a line (mm)."""
    d = (p[0] - origin[0], p[1] - origin[1], p[2] - origin[2])
    k = d[0] * axis[0] + d[1] * axis[1] + d[2] * axis[2]
    return math.sqrt(max(0.0, d[0] ** 2 + d[1] ** 2 + d[2] ** 2 - k * k))


def _seg_axis_dist(a, b, origin, axis):
    """Smallest distance from a segment to the hole's axis. The segment is an edge of a face
    perpendicular to that axis, so this is the in-plane point-to-segment distance."""
    def proj(p):
        d = (p[0] - origin[0], p[1] - origin[1], p[2] - origin[2])
        k = d[0] * axis[0] + d[1] * axis[1] + d[2] * axis[2]
        return (d[0] - k * axis[0], d[1] - k * axis[1], d[2] - k * axis[2])

    return _pt_seg_dist((0.0, 0.0, 0.0), proj(a), proj(b))


def _strokes(edge):
    """The edge as a polyline of world points in MILLIMETRES."""
    ev = edge.evaluator
    ok_ext, p0, p1 = ev.getParameterExtents()
    if not ok_ext:
        raise RuntimeError('an edge of the body has no parameter extents')
    ok, pts = ev.getStrokes(p0, p1, STROKE_TOL)
    if not ok or not pts:
        raise RuntimeError('cannot stroke an edge of the body')
    return [(p.x * 10.0, p.y * 10.0, p.z * 10.0) for p in pts]


def holes(body, hole_r):
    """Every screw hole: (a point on its axis, its unit axis) in world mm, deduped."""
    out = []
    for f in body.faces:
        g = f.geometry
        if g.objectType != 'adsk::core::Cylinder' or abs(g.radius * 10.0 - hole_r) > RADIUS_TOL:
            continue
        o = (g.origin.x * 10.0, g.origin.y * 10.0, g.origin.z * 10.0)
        ax = (g.axis.x, g.axis.y, g.axis.z)
        for p, a in out:
            if abs(abs(a[0] * ax[0] + a[1] * ax[1] + a[2] * ax[2]) - 1.0) < 1e-6 \
                    and _axis_dist(o, p, a) < 0.01:
                break
        else:
            out.append((o, ax))
    return out


def measure(body, frame, hole_r):
    """Every (hole, seating face) of `body` with the flat ring it really has.

    `frame` = (origin, u, w, z): the rotor-pad frame in world, origin an adsk Point3D in cm
    and the three axes unit Vector3D, so the result matches `armgen.hole_flat_rings` hole for
    hole. Each entry carries

        at     (s, w, z) mm: a point on the hole's axis, in the pad frame
        axis   (s, w, z): the hole's unit axis in the pad frame
        plane  ('z', value) or ('s', value): the seating face's own plane
        ring   mm of flat beyond the hole edge, NEGATIVE when the flat ends inside it
        bound  distance from the hole axis to the nearest other edge of that face
    """
    o, u, w, z = frame
    ow = (o.x * 10.0, o.y * 10.0, o.z * 10.0)
    U, W, Z = (u.x, u.y, u.z), (w.x, w.y, w.z), (z.x, z.y, z.z)

    def local(p):
        d = (p[0] - ow[0], p[1] - ow[1], p[2] - ow[2])
        return (d[0] * U[0] + d[1] * U[1] + d[2] * U[2],
                d[0] * W[0] + d[1] * W[1] + d[2] * W[2],
                d[0] * Z[0] + d[1] * Z[1] + d[2] * Z[2])

    def local_dir(v):
        return (v[0] * U[0] + v[1] * U[1] + v[2] * U[2],
                v[0] * W[0] + v[1] * W[1] + v[2] * W[2],
                v[0] * Z[0] + v[1] * Z[1] + v[2] * Z[2])

    planes = [f for f in body.faces if f.geometry.objectType == 'adsk::core::Plane']
    out = []
    for origin, axis in holes(body, hole_r):
        # EXO-124's first question, before any distance: does anything CURVED touch this
        # hole? A hole whose opening runs into a fillet has no seat at all. (Neighbours are
        # deduplicated - a fillet that meets the hole along two edges is ONE face, and
        # counting it twice is what made the first reading of link_45 report two.)
        curved, seen_f = [], []
        for hf in body.faces:
            g = hf.geometry
            if g.objectType != 'adsk::core::Cylinder' \
                    or abs(g.radius * 10.0 - hole_r) > RADIUS_TOL \
                    or _axis_dist((g.origin.x * 10.0, g.origin.y * 10.0, g.origin.z * 10.0),
                                  origin, axis) > 0.01:
                continue
            for e in hf.edges:
                for nf in e.faces:
                    if nf == hf or nf in seen_f:
                        continue
                    seen_f.append(nf)
                    if nf.geometry.objectType != 'adsk::core::Plane':
                        curved.append('%s area %.2f mm2%s'
                                      % (nf.geometry.objectType.split('::')[-1],
                                         nf.area * 100.0,
                                         ' r=%.3f' % (nf.geometry.radius * 10.0)
                                         if nf.geometry.objectType == 'adsk::core::Cylinder'
                                         else ''))
        for f in planes:
            n = f.geometry.normal
            if abs(n.x * axis[0] + n.y * axis[1] + n.z * axis[2]) < AXIS_TOL:
                continue
            strokes = [_strokes(e) for e in f.edges]
            own = [all(abs(_axis_dist(p, origin, axis) - hole_r) <= OWN_EDGE_TOL for p in st)
                   for st in strokes]
            if not any(own):
                continue                     # this hole does not break through this face
            best = None
            for st, is_own in zip(strokes, own):
                if is_own:
                    continue
                for i in range(len(st) - 1):
                    d = _seg_axis_dist(st[i], st[i + 1], origin, axis)
                    if best is None or d < best:
                        best = d
            if best is None:
                continue
            po = f.geometry.origin
            pl = local((po.x * 10.0, po.y * 10.0, po.z * 10.0))
            kind = 'z' if abs(local_dir((n.x, n.y, n.z))[2]) > AXIS_TOL else 's'
            out.append({'at': local(origin), 'axis': local_dir(axis),
                        'plane': (kind, pl[2] if kind == 'z' else pl[0]),
                        'ring': best - hole_r, 'bound': best, 'curved': list(curved)})
    return out


def match(measured, predicted, pos_tol=0.1, plane_tol=0.05):
    """Pair each measured (hole, face) with `armgen.hole_flat_rings`' entry for it - same
    hole axis, same plane. Returns (pairs, unmatched measured, unmatched predicted)."""
    left = list(predicted)
    pairs, orphans = [], []
    for m in measured:
        hit = None
        for p in left:
            if p['plane'][0] != m['plane'][0] or abs(p['plane'][1] - m['plane'][1]) > plane_tol:
                continue
            if _axis_dist(p['at'], m['at'], m['axis']) > pos_tol:
                continue
            hit = p
            break
        if hit is None:
            orphans.append(m)
        else:
            left.remove(hit)
            pairs.append((m, hit))
    return pairs, orphans, left


def gate(label, body, frame, spec, armgen, check, log, stop=None, tol=0.02):
    """EXO-124's gate, as the measured-body scripts run it: every hole of `body` opens onto
    PLANE at both ends, has `armgen.HOLE_FLAT_RING` mm of flat around each opening, and
    measures what `armgen.hole_flat_rings` solved the route for. `check(name, ok, detail)` is
    the caller's own check function, so the counts land in its own tally."""
    predicted = armgen.hole_flat_rings(spec['route'], stop=stop)
    measured = measure(body, frame, armgen.HOLE_R)
    pairs, orphans, missing = match(measured, predicted)
    check('%s: every hole opening the route knows about is on the body' % label,
          not missing and not orphans,
          'route-only %s, body-only %s'
          % ([(m['pad'], m['angle'], m['face']) for m in missing],
             [(round(o['at'][0], 1), o['plane']) for o in orphans]))
    for m, p in sorted(pairs, key=lambda mp: mp[0]['ring']):
        tag = '%s pad %.0f deg %s face' % (p['pad'], p['angle'], p['face'])
        log('    %-34s ring %7.3f mm (route %7.3f, bound: %s)'
            % (tag, m['ring'], p['ring'], p['bound']))
        check('%s: %s opens onto plane' % (label, tag), not m['curved'], '; '.join(m['curved']))
        check('%s: %s has its %.1f mm flat ring' % (label, tag, armgen.HOLE_FLAT_RING),
              m['ring'] >= armgen.HOLE_FLAT_RING - tol, 'measured %.3f mm' % m['ring'])
        check('%s: %s measures what the route solved for' % (label, tag),
              abs(m['ring'] - p['ring']) <= tol,
              'body %.3f vs route %.3f' % (m['ring'], p['ring']))
    return pairs


def pad_frame(joint_occurrence, geo, adsk_core):
    """(origin, u, w, z) in world for `measure`, from the rotor-pad joint's occurrence and
    the route's own (u, w). Every measured-body script already has both."""
    T = joint_occurrence.transform
    u, w = geo['u'], geo['w']
    uw = adsk_core.Vector3D.create(u[0], u[1], 0.0)
    ww = adsk_core.Vector3D.create(w[0], w[1], 0.0)
    zw = adsk_core.Vector3D.create(0.0, 0.0, 1.0)
    for v in (uw, ww, zw):
        v.transformBy(T)
    return (adsk_core.Point3D.create(T.translation.x, T.translation.y, T.translation.z),
            uw, ww, zw)
