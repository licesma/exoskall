"""EXO-350 probe (read-only): open/activate `exo generated`, list the faces of the body in
`stator EXO-341 export` (own frame = file frame, module z = file z - 7.5): axis-centred
cylinders (r, z range) and horizontal planes (z, area, r range)."""
import math
EXO_GEN_ID = 'urn:adsk.wipprod:dm.lineage:I6f3yGrzR8S8xg8G0BP0aw'
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
doc = None
for i in range(app.documents.count):
    d = app.documents.item(i)
    try:
        if d.dataFile is not None and d.dataFile.id == EXO_GEN_ID: doc = d
    except Exception: pass
if doc is None:
    doc = app.documents.open(app.data.findFileById(EXO_GEN_ID), True)
else:
    doc.activate()
des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
say('document %r v%s modified %s designType %s' % (doc.name, doc.dataFile.versionNumber, doc.isModified, des.designType))
root = des.rootComponent
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    say('  root occ %r comp %r bulb %s' % (o.name, o.component.name, o.isLightBulbOn))
occ = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name == 'stator EXO-341 export'][0]
b = occ.component.bRepBodies.item(0)
M = occ.transform2.asArray()
say('export occ transform %s' % ['%.4f' % v for v in M])
bb = b.boundingBox
say('body %r V %.4f faces %d bbox %s %s' % (b.name, b.volume*1000, b.faces.count,
    ['%.3f' % (v*10) for v in (bb.minPoint.x, bb.minPoint.y, bb.minPoint.z)], ['%.3f' % (v*10) for v in (bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z)]))
S = adsk.core.SurfaceTypes
for i in range(b.faces.count):
    f = b.faces.item(i); g = f.geometry; fb = f.boundingBox
    z0, z1 = fb.minPoint.z*10, fb.maxPoint.z*10
    if g.surfaceType == S.CylinderSurfaceType:
        c = g.origin
        say('cyl r %.4f centre %.3f,%.3f axis %.3f,%.3f,%.3f z %.4f..%.4f area %.2f' % (g.radius*10, c.x*10, c.y*10, g.axis.x, g.axis.y, g.axis.z, z0, z1, f.area*100))
    elif g.surfaceType == S.PlaneSurfaceType and abs(g.normal.z) > 0.999:
        rmax = max(math.hypot(x, y) for x in (fb.minPoint.x*10, fb.maxPoint.x*10) for y in (fb.minPoint.y*10, fb.maxPoint.y*10))
        say('plane z %.4f n %+.0f area %.2f bbox x %.2f..%.2f y %.2f..%.2f' % (z0, g.normal.z, f.area*100, fb.minPoint.x*10, fb.maxPoint.x*10, fb.minPoint.y*10, fb.maxPoint.y*10))
INS = (adsk.fusion.PointContainment.PointInsidePointContainment, adsk.fusion.PointContainment.PointOnPointContainment)
for ang in (45, 135, 168, 225, 315):
    a = math.radians(ang)
    for z in (-17.0+0.05, -5.0, -3.6, -3.4, 0.0, 3.4, 3.6, 5.0, 7.4):
        row = ''.join('#' if b.pointContainment(adsk.core.Point3D.create(r*math.cos(a)/10, r*math.sin(a)/10, z/10)) in INS else '.' for r in [15 + 0.25*k for k in range(53)])
        say('a %3d z %+6.2f r 15..28: %s' % (ang, z, row))
