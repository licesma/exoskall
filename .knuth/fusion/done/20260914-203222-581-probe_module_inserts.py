# Exact extents (module frame) of the stator body and the heat-set inserts under the J2 wrapper,
# to see whether the inserts sit proud of the stator underside (the 0.1 mm3 pad/insert slivers).
root = design.rootComponent
arm = None
for i in range(root.occurrences.count):
    occ = root.occurrences.item(i)
    if occ.component.name.startswith('generated_R_arm'):
        arm = occ; break
j2 = None
for i in range(arm.childOccurrences.count):
    o = arm.childOccurrences.item(i)
    if o.component.name.startswith('J2_'):
        j2 = o
inv = j2.transform.copy(); inv.invert()
import itertools
def walk(occ, prefix):
    for i in range(occ.bRepBodies.count):
        b = occ.bRepBodies.item(i)
        bb = b.boundingBox
        lo, hi = bb.minPoint, bb.maxPoint
        pts = []
        for cx, cy, cz in itertools.product((lo.x, hi.x), (lo.y, hi.y), (lo.z, hi.z)):
            p = adsk.core.Point3D.create(cx, cy, cz); p.transformBy(inv)
            pts.append((p.x*10, p.y*10, p.z*10))
        log('%-45s J2 frame x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] vol=%.2f mm3' % (
            prefix + '/' + b.name, min(p[0] for p in pts), max(p[0] for p in pts),
            min(p[1] for p in pts), max(p[1] for p in pts), min(p[2] for p in pts), max(p[2] for p in pts),
            b.volume*1000))
    for i in range(occ.childOccurrences.count):
        sub = occ.childOccurrences.item(i)
        walk(sub, prefix + '>' + sub.component.name)
walk(j2, j2.component.name)
