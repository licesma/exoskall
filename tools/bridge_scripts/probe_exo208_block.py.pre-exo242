"""EXO-208 probe: measure the stator stop block of the LINKED `sensor module 2`, in Fusion,
in the module frame - angular extent, radii and z - by point containment on the solid.

Nothing is modified. The root occurrence `sensor module 2:1` sits at identity, so its
assembly context IS the module frame and Fusion world cm = module mm / 10.
"""
import math

root = design.rootComponent
mod = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name == 'sensor module 2':
        mod = o
        break
if mod is None:
    raise RuntimeError('no root `sensor module 2` occurrence')
m = mod.transform
log('module occ %r transform translation = (%.6f, %.6f, %.6f) cm'
    % (mod.name, m.translation.x, m.translation.y, m.translation.z))
log('  matrix %s' % ['%.4f' % v for v in m.asArray()])

for i in range(mod.childOccurrences.count):
    c = mod.childOccurrences.item(i)
    if c.component.name in ('Component1', 'Component2', 'Component3'):
        t = c.transform
        log('  child %-14r translation (%.6f, %.6f, %.6f) cm  bodies=%d'
            % (c.name, t.translation.x, t.translation.y, t.translation.z,
               c.bRepBodies.count))

stator = None
for i in range(mod.childOccurrences.count):
    c = mod.childOccurrences.item(i)
    if c.component.name == 'Component1':
        stator = c.bRepBodies.item(0)
log('stator body %r' % stator.name)
bb = stator.boundingBox
log('  bbox mm  x %.3f..%.3f  y %.3f..%.3f  z %.3f..%.3f'
    % (bb.minPoint.x * 10, bb.maxPoint.x * 10, bb.minPoint.y * 10, bb.maxPoint.y * 10,
       bb.minPoint.z * 10, bb.maxPoint.z * 10))
log('  volume %.3f mm3' % (stator.volume * 1000.0))

INSIDE = adsk.fusion.PointContainment.PointInsidePointContainment
ON = adsk.fusion.PointContainment.PointOnPointContainment


def solid(r_mm, a_deg, z_mm):
    p = adsk.core.Point3D.create(r_mm * math.cos(math.radians(a_deg)) / 10.0,
                                 r_mm * math.sin(math.radians(a_deg)) / 10.0, z_mm / 10.0)
    return stator.pointContainment(p) in (INSIDE, ON)


def edge(a_in, a_out, r_mm, z_mm, n=60):
    """Bisect for the angle where solid() flips, between an inside and an outside angle."""
    for _ in range(n):
        mid = (a_in + a_out) / 2.0
        if solid(r_mm, mid, z_mm):
            a_in = mid
        else:
            a_out = mid
    return (a_in + a_out) / 2.0


R_MID, Z_MID = 24.0, -2.2
hits = [a * 0.25 for a in range(1440) if solid(R_MID, a * 0.25, Z_MID)]
log('')
log('sweep r %.1f mm, z %.1f mm, 0.25 deg: %d of 1440 samples inside the solid'
    % (R_MID, Z_MID, len(hits)))
if hits:
    log('  coarse span %.3f .. %.3f deg' % (min(hits), max(hits)))
    lo = edge(min(hits), min(hits) - 0.25, R_MID, Z_MID)
    hi = edge(max(hits), max(hits) + 0.25, R_MID, Z_MID)
    log('  BLOCK  %.5f .. %.5f deg   arc %.5f   centre %.5f' % (lo, hi, hi - lo, (lo + hi) / 2))

# radii and z of the block, at its own centre angle
if hits:
    ac = (lo + hi) / 2.0
    rs = [r * 0.05 for r in range(300, 600) if solid(r * 0.05, ac, Z_MID)]
    log('  radial at %.3f deg, z %.1f: solid over r %.3f .. %.3f mm' % (ac, Z_MID,
        min(rs), max(rs)))
    zs = [-5.0 + k * 0.02 for k in range(300) if solid(R_MID, ac, -5.0 + k * 0.02)]
    log('  axial  at %.3f deg, r %.1f: solid over z %.3f .. %.3f mm' % (ac, R_MID,
        min(zs), max(zs)))
    # and the plain stator rim just below, to show where the block starts
    zs2 = [-5.0 + k * 0.02 for k in range(300) if solid(R_MID, (ac + 180.0) % 360.0,
                                                        -5.0 + k * 0.02)]
    log('  axial  at %.3f deg (opposite), r %.1f: solid over z %.3f .. %.3f mm'
        % ((ac + 180.0) % 360.0, R_MID, min(zs2), max(zs2)))
