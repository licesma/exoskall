import sys, importlib, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen; importlib.reload(armgen)

root = design.rootComponent
arm = next(o for o in root.occurrences if o.component.name.startswith('generated_R_arm'))
l45 = next(s for s in arm.childOccurrences if s.component.name.startswith('link_45'))
body = l45.bRepBodies.item(0)

# local frame of the link occurrence
m = l45.transform2
ox,oy,oz = m.translation.x, m.translation.y, m.translation.z
import adsk.core as ac
xa = m.getCell(0,0), m.getCell(1,0), m.getCell(2,0)
ya = m.getCell(0,1), m.getCell(1,1), m.getCell(2,1)
za = m.getCell(0,2), m.getCell(1,2), m.getCell(2,2)
def local(p):
    d = (p.x-ox, p.y-oy, p.z-oz)
    return (10*sum(d[i]*xa[i] for i in range(3)),
            10*sum(d[i]*ya[i] for i in range(3)),
            10*sum(d[i]*za[i] for i in range(3)))

hr = armgen.HOLE_R
log('hole centres in the link frame (s = along the route, w = across, z = up):')
seen=[]
for f in body.faces:
    g = f.geometry
    if g.objectType == 'adsk::core::Cylinder' and abs(g.radius*10.0 - hr) < 0.05:
        c = local(g.origin)
        key = (round(c[0],2), round(c[1],2))
        if key in seen: continue
        seen.append(key)
        log('  s=%8.3f  w=%8.3f  z=%8.3f   outer edge s=%8.3f' % (c[0], c[1], c[2], c[0]+hr))

# planar faces at the pad level, to find where the flat top ends
log('')
log('planar faces normal to z (the flat pad/plate top and bottom):')
for f in body.faces:
    g = f.geometry
    if g.objectType == 'adsk::core::Plane':
        n = g.normal
        nl = (n.x*za[0]+n.y*za[1]+n.z*za[2])
        if abs(abs(nl)-1.0) < 1e-6:
            bb = f.boundingBox
            p0, p1 = local(bb.minPoint), local(bb.maxPoint)
            log('  area %8.2f mm2   s %8.3f .. %8.3f   z ~ %7.3f' % (f.area*100, min(p0[0],p1[0]), max(p0[0],p1[0]), p0[2]))
