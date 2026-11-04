import sys, importlib, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen; importlib.reload(armgen)
import adsk.core as ac

root = design.rootComponent
arm = next(o for o in root.occurrences if o.component.name.startswith('generated_R_arm'))
hr = armgen.HOLE_R

inserts = []
def walk(occ):
    for b in occ.bRepBodies:
        if 'insert' in occ.component.name.lower() or 'insert' in b.name.lower():
            bb = b.boundingBox
            inserts.append((occ.fullPathName,
                            ac.Point3D.create((bb.minPoint.x+bb.maxPoint.x)/2,
                                              (bb.minPoint.y+bb.maxPoint.y)/2,
                                              (bb.minPoint.z+bb.maxPoint.z)/2)))
    for c in occ.childOccurrences: walk(c)
for o in arm.childOccurrences: walk(o)
log('heat-set inserts in the generated arm: %d' % len(inserts))

for sub in arm.childOccurrences:
    nm = sub.component.name
    if not nm.startswith('link_') or sub.bRepBodies.count == 0: continue
    body = sub.bRepBodies.item(0)
    seen = []
    log('')
    log('%s:' % nm)
    for f in body.faces:
        g = f.geometry
        if g.objectType != 'adsk::core::Cylinder' or abs(g.radius*10.0 - hr) > 0.05: continue
        k = (round(g.origin.x,4), round(g.origin.y,4), round(g.origin.z,4))
        if k in seen: continue
        seen.append(k)
        o, ax = g.origin, g.axis
        best = None
        for pth, c in inserts:
            d = ac.Vector3D.create(c.x-o.x, c.y-o.y, c.z-o.z)
            along = d.x*ax.x + d.y*ax.y + d.z*ax.z
            perp = math.sqrt(max(0.0, d.length**2 - along**2))*10.0
            if best is None or perp < best[0]:
                best = (perp, abs(along)*10.0, pth.split('+')[-1])
        status = 'ALIGNED' if best[0] < 1.0 else ('off by %.2f mm' % best[0])
        log('  hole (%6.1f,%6.1f,%6.1f): nearest insert %s, %.1f mm along the axis  -> %s'
            % (o.x*10, o.y*10, o.z*10, status, best[1], 'ok' if best[0] < 1.0 else 'CHECK'))
