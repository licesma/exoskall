import sys, importlib, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen; importlib.reload(armgen)

root = design.rootComponent
arm = next(o for o in root.occurrences if o.component.name.startswith('generated_R_arm'))

hr = armgen.HOLE_R
log('For every link: each screw hole, and how much FLAT face surrounds it')
log('(sampling the body just outside each hole on its own seating plane)')
for sub in arm.childOccurrences:
    nm = sub.component.name
    if not nm.startswith('link_'): continue
    if sub.bRepBodies.count == 0: continue
    body = sub.bRepBodies.item(0)
    log('')
    log('%s:' % nm)
    # collect hole cylinders and the planar faces they open onto
    holes = []
    for f in body.faces:
        g = f.geometry
        if g.objectType == 'adsk::core::Cylinder' and abs(g.radius*10.0 - hr) < 0.05:
            holes.append((g.origin, g.axis, f))
    planes = [f for f in body.faces if f.geometry.objectType == 'adsk::core::Plane']
    for i, (o, ax, hf) in enumerate(holes):
        # find the planar face whose normal is parallel to the hole axis and that is nearest the hole
        best = None
        for p in planes:
            n = p.geometry.normal
            d = abs(n.x*ax.x + n.y*ax.y + n.z*ax.z)
            if d < 0.999: continue
            # distance from hole origin to plane, along axis
            pl = p.geometry
            dist = abs(pl.distanceTo(o))
            if best is None or dist < best[0]: best = (dist, p)
        if best is None:
            log('  hole %d: no seating plane found' % (i+1)); continue
        p = best[1]
        # walk outward in 36 directions from the hole centre on that plane, find first r where the point leaves the face
        n = p.geometry.normal
        # build two in-plane axes
        a = adsk.core.Vector3D.create(1,0,0)
        if abs(n.x) > 0.9: a = adsk.core.Vector3D.create(0,1,0)
        u = n.crossProduct(a); u.normalize()
        v = n.crossProduct(u); v.normalize()
        # project hole origin onto the plane
        centre = adsk.core.Point3D.create(o.x, o.y, o.z)
        ev = p.geometry.intersectWithLine(adsk.core.InfiniteLine3D.create(centre, n))
        if ev: centre = ev
        worst = 99.0
        for k in range(72):
            th = 2*math.pi*k/72.0
            rr = hr
            step = 0.05
            last_ok = hr
            while rr < 12.0:
                rr += step
                px = adsk.core.Point3D.create(
                    centre.x + (u.x*math.cos(th) + v.x*math.sin(th))*rr/10.0,
                    centre.y + (u.y*math.cos(th) + v.y*math.sin(th))*rr/10.0,
                    centre.z + (u.z*math.cos(th) + v.z*math.sin(th))*rr/10.0)
                if p.evaluator.isParameterOnFace(p.evaluator.getParameterAtPoint(px)[1]):
                    last_ok = rr
                else:
                    break
            worst = min(worst, last_ok - hr)
        log('  hole %d: flat ring around it = %.2f mm (worst direction)%s'
            % (i+1, worst, '   <-- TIGHT' if worst < 1.5 else ''))
