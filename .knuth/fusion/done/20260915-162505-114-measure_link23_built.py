# EXO-58: measure the BUILT link_23 body in the J2 module frame (the J2_* wrapper's
# transform; arm at identity). Logs: bbox, every cylindrical face (radius, axis, centre),
# every planar face whose normal is along the bar width (Y_J2) with its y position and
# extent, so one-width / no-step / fillet-present can be checked against Link 2 R.
import math
root = design.rootComponent
arm = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name.startswith('generated_R_arm'):
        arm = o
j2 = link = None
for i in range(arm.childOccurrences.count):
    o = arm.childOccurrences.item(i)
    if o.component.name.startswith('J2_'):
        j2 = o
    if o.component.name.startswith('link_23'):
        link = o
inv = j2.transform.copy(); inv.invert()
body = link.bRepBodies.item(0)
def toJ2(p):
    q = p.copy(); q.transformBy(inv); return (q.x * 10, q.y * 10, q.z * 10)
def dirJ2(v):
    w = v.copy(); w.transformBy(inv); return (w.x, w.y, w.z)
bb = body.boundingBox
import itertools
pts = [toJ2(adsk.core.Point3D.create(x, y, z)) for x, y, z in itertools.product(
    (bb.minPoint.x, bb.maxPoint.x), (bb.minPoint.y, bb.maxPoint.y), (bb.minPoint.z, bb.maxPoint.z))]
log('link_23: %d faces, volume %.0f mm3, bbox in J2 frame x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]' % (
    body.faces.count, body.volume * 1000, min(p[0] for p in pts), max(p[0] for p in pts),
    min(p[1] for p in pts), max(p[1] for p in pts), min(p[2] for p in pts), max(p[2] for p in pts)))
ST = {0: 'plane', 1: 'cyl', 2: 'cone', 3: 'sphere', 4: 'torus', 5: 'ellcyl', 6: 'ellcone', 7: 'nurbs'}
kinds = {}
cyl, ywalls = [], []
for i in range(body.faces.count):
    f = body.faces.item(i)
    g = f.geometry
    k = ST.get(g.surfaceType, '?')
    kinds[k] = kinds.get(k, 0) + 1
    fb = f.boundingBox
    cs = [toJ2(adsk.core.Point3D.create(x, y, z)) for x, y, z in itertools.product(
        (fb.minPoint.x, fb.maxPoint.x), (fb.minPoint.y, fb.maxPoint.y), (fb.minPoint.z, fb.maxPoint.z))]
    ext = tuple((min(c[a] for c in cs), max(c[a] for c in cs)) for a in range(3))
    if k == 'cyl':
        ax = dirJ2(g.axis); o = toJ2(g.origin)
        cyl.append((round(g.radius * 10, 2), tuple(round(v, 2) for v in ax), tuple(round(v, 2) for v in o), ext, f.area * 100))
    elif k == 'plane':
        n = dirJ2(g.normal)
        if abs(n[1]) > 0.999:
            ywalls.append((round(toJ2(g.origin)[1], 2), ext, f.area * 100))
log('face kinds: %s' % kinds)
log('cylindrical faces (r, axis J2, a point on axis J2, face bbox J2, area mm2):')
for c in sorted(cyl):
    log('  r %6.2f axis %s at %s  x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f]  %.0f mm2' % (
        c[0], c[1], c[2], c[3][0][0], c[3][0][1], c[3][1][0], c[3][1][1], c[3][2][0], c[3][2][1], c[4]))
log('planar faces normal to Y_J2 (bar width direction): y, bbox, area')
for y, ext, a in sorted(ywalls):
    log('  y %7.2f  x[%.1f,%.1f] z[%.1f,%.1f]  %.0f mm2' % (y, ext[0][0], ext[0][1], ext[2][0], ext[2][1], a))
