import sys, importlib, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen; importlib.reload(armgen)
import adsk.core as ac, adsk.fusion as af

root = design.rootComponent
arm = next(o for o in root.occurrences if o.component.name.startswith('generated_R_arm'))
hr = armgen.HOLE_R
l45 = next(s for s in arm.childOccurrences if s.component.name.startswith('link_45'))
body = l45.bRepBodies.item(0)

# all bodies of the generated arm except link_45
others = []
def walk(o):
    for b in o.bRepBodies: others.append((o.component.name, b))
    for c in o.childOccurrences: walk(c)
for o in arm.childOccurrences:
    if o is not l45: walk(o)

log('For the J5 stator-pad holes: which side is the module on, so where does the head bear?')
seen=[]
for f in body.faces:
    g = f.geometry
    if g.objectType != 'adsk::core::Cylinder' or abs(g.radius*10.0 - hr) > 0.05: continue
    k=(round(g.origin.x,4),round(g.origin.y,4),round(g.origin.z,4))
    if k in seen: continue
    seen.append(k)
    o, ax = g.origin, g.axis
    if abs(o.x*10 - 87.3) > 1: continue          # stator-pad holes only
    for sign in (+1,-1):
        hit=None
        for d in [x*0.25 for x in range(1,81)]:
            p = ac.Point3D.create(o.x+sign*ax.x*d/10, o.y+sign*ax.y*d/10, o.z+sign*ax.z*d/10)
            for nm,ob in others:
                if ob.pointContainment(p)==af.PointContainment.PointInsidePointContainment:
                    hit=(d,nm); break
            if hit: break
        log('  hole (%.1f,%.1f,%.1f) dir %+d: %s' % (o.x*10,o.y*10,o.z*10,sign,
            ('first solid at %.2f mm = %s' % hit) if hit else 'nothing out to 20 mm  <-- HEAD SIDE'))
