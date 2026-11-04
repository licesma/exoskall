import adsk.core, adsk.fusion, math
root = design.rootComponent
for i in range(root.occurrences.count-1,-1,-1):
    o=root.occurrences.item(i)
    if o.component.name.startswith('PROBE_'): o.deleteMe()
occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
c = occ.component; c.name = 'PROBE_sweep'
sk = c.sketches.add(c.xYConstructionPlane)
P = adsk.core.Point3D.create
L = sk.sketchCurves.sketchLines; A = sk.sketchCurves.sketchArcs
l1 = L.addByTwoPoints(P(0,0,0), P(3,0,0))
a1 = A.addByThreePoints(P(3,0,0), P(3+3*0.7071,3*(1-0.7071),0), P(6,3,0))
l2 = L.addByTwoPoints(P(6,3,0), P(6,8,0))
coll = adsk.core.ObjectCollection.create()
for x in (l1,a1,l2): coll.add(x)
path=None
for label, fn in (('createPath(coll,False)', lambda: c.features.createPath(coll, False)),
                  ('createPath(l1,True)', lambda: c.features.createPath(l1, True))):
    try:
        p = fn(); log('%s -> count=%s' % (label, None if p is None else p.count))
        if path is None and p is not None: path=p
    except Exception as e:
        log('%s -> ERR %s' % (label, e))
# profile perpendicular at the start
pin = c.constructionPlanes.createInput()
pin.setByPlane(adsk.core.Plane.create(P(0,0,0), adsk.core.Vector3D.create(1,0,0)))
cp = c.constructionPlanes.add(pin)
sk2 = c.sketches.add(cp)
o = sk2.modelToSketchSpace(P(0,0,0))
pts = [(-1,-0.4),(1,-0.4),(1,0.4),(-1,0.4)]
def q(a,b):
    return sk2.modelToSketchSpace(P(0 , a, b))
lines2 = sk2.sketchCurves.sketchLines
for i in range(4):
    lines2.addByTwoPoints(q(*pts[i]), q(*pts[(i+1)%4]))
log('profiles=%d' % sk2.profiles.count)
sw = c.features.sweepFeatures
inp = sw.createInput(sk2.profiles.item(0), path, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
inp.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
f = sw.add(inp)
log('sweep bodies=%d faces=%d vol=%.3f' % (f.bodies.count, f.bodies.item(0).faces.count, f.bodies.item(0).volume))
