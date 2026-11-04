import sys, importlib, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen; importlib.reload(armgen)
import adsk.core as ac, adsk.fusion as af

root = design.rootComponent
arm = next(o for o in root.occurrences if o.component.name.startswith('generated_R_arm'))
hr = armgen.HOLE_R

# every body in the design that is NOT the link under test
def other_bodies(skip_occ):
    out = []
    def walk(occ):
        for b in occ.bRepBodies: out.append((occ.name, b))
        for c in occ.childOccurrences: walk(c)
    for o in root.occurrences:
        if o is skip_occ: continue
        walk(o)
    for o in arm.childOccurrences:
        if o is skip_occ: continue
        walk(o)
    return out

log('Screw access: from each hole, walk OUT along the hole axis and see what is in the way')
for sub in arm.childOccurrences:
    nm = sub.component.name
    if not nm.startswith('link_45') or sub.bRepBodies.count == 0: continue
    body = sub.bRepBodies.item(0)
    others = other_bodies(sub)
    log('%s  (checking against %d other bodies)' % (nm, len(others)))
    seen = []
    for f in body.faces:
        g = f.geometry
        if g.objectType != 'adsk::core::Cylinder' or abs(g.radius*10.0 - hr) > 0.05: continue
        k = (round(g.origin.x,4), round(g.origin.y,4), round(g.origin.z,4))
        if k in seen: continue
        seen.append(k)
        o, ax = g.origin, g.axis
        for sign in (+1, -1):
            hits = []
            for d_mm in [x*0.5 for x in range(1, 61)]:   # 0.5 .. 30 mm out
                p = ac.Point3D.create(o.x + sign*ax.x*d_mm/10.0,
                                      o.y + sign*ax.y*d_mm/10.0,
                                      o.z + sign*ax.z*d_mm/10.0)
                for onm, ob in others:
                    if ob.pointContainment(p) == af.PointContainment.PointInsidePointContainment:
                        hits.append((d_mm, onm, ob.name)); break
                if hits: break
            tag = 'clear to 30 mm' if not hits else 'BLOCKED at %.1f mm by %s/%s' % hits[0]
            log('  hole (%6.1f,%6.1f,%6.1f) dir %+d: %s' % (o.x*10, o.y*10, o.z*10, sign, tag))
