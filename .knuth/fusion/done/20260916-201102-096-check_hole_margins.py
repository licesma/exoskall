import sys, importlib, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen; importlib.reload(armgen)
import adsk.core as ac

root = design.rootComponent
arm = next(o for o in root.occurrences if o.component.name.startswith('generated_R_arm'))
hr = armgen.HOLE_R

def plane_dist(pl, p):
    n = pl.normal; o = pl.origin
    return abs((p.x-o.x)*n.x + (p.y-o.y)*n.y + (p.z-o.z)*n.z)

log('Flat seating ring around every screw hole (mm beyond the hole edge, worst direction)')
for sub in arm.childOccurrences:
    nm = sub.component.name
    if not nm.startswith('link_') or sub.bRepBodies.count == 0: continue
    body = sub.bRepBodies.item(0)
    log('')
    log('%s:' % nm)
    holes, planes = [], [f for f in body.faces if f.geometry.objectType == 'adsk::core::Plane']
    seen = []
    for f in body.faces:
        g = f.geometry
        if g.objectType == 'adsk::core::Cylinder' and abs(g.radius*10.0 - hr) < 0.05:
            k = (round(g.origin.x,4), round(g.origin.y,4), round(g.origin.z,4))
            if k in seen: continue
            seen.append(k); holes.append((g.origin, g.axis))
    for i, (o, ax) in enumerate(holes):
        best = None
        for p in planes:
            n = p.geometry.normal
            if abs(n.x*ax.x + n.y*ax.y + n.z*ax.z) < 0.999: continue
            d = plane_dist(p.geometry, o)
            if best is None or d < best[0]: best = (d, p)
        if best is None:
            log('  hole %d: no seating plane' % (i+1)); continue
        p = best[1]; n = p.geometry.normal
        a = ac.Vector3D.create(0,1,0) if abs(n.x) > 0.9 else ac.Vector3D.create(1,0,0)
        u = n.crossProduct(a); u.normalize()
        v = n.crossProduct(u); v.normalize()
        # centre projected onto the plane
        d = ((o.x-p.geometry.origin.x)*n.x + (o.y-p.geometry.origin.y)*n.y + (o.z-p.geometry.origin.z)*n.z)
        cx, cy, cz = o.x - d*n.x, o.y - d*n.y, o.z - d*n.z
        ev = p.evaluator
        worst, worst_th = 99.0, 0
        for k in range(72):
            th = 2*math.pi*k/72.0
            dx = u.x*math.cos(th) + v.x*math.sin(th)
            dy = u.y*math.cos(th) + v.y*math.sin(th)
            dz = u.z*math.cos(th) + v.z*math.sin(th)
            rr, last = hr, hr
            while rr < 12.0:
                rr += 0.05
                pt = ac.Point3D.create(cx + dx*rr/10.0, cy + dy*rr/10.0, cz + dz*rr/10.0)
                ok, prm = ev.getParameterAtPoint(pt)
                if ok and ev.isParameterOnFace(prm): last = rr
                else: break
            if last - hr < worst: worst, worst_th = last - hr, int(math.degrees(th))
        log('  hole %d: %.2f mm  (worst at %d deg)%s' % (i+1, worst, worst_th, '   <-- TIGHT' if worst < 1.5 else ''))
