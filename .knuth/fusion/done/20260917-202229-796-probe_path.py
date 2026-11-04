import adsk.core, adsk.fusion
root = design.rootComponent
occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
c = occ.component; c.name = 'PROBE_sweep'
sk = c.sketches.add(c.xYConstructionPlane)
log('is3D before=%s' % sk.is3D)
sk.is3D = True
log('is3D after=%s' % sk.is3D)
P = adsk.core.Point3D.create
L = sk.sketchCurves.sketchLines
A = sk.sketchCurves.sketchArcs
l1 = L.addByTwoPoints(P(0,0,0), P(3,0,0))
a1 = A.addByThreePoints(P(3,0,0), P(3.7071,0.2929,0), P(4,1,0))
l2 = L.addByTwoPoints(P(4,1,0), P(4,1,-3))
log('curves: %d lines %d arcs' % (L.count, A.count))
coll = adsk.core.ObjectCollection.create()
for x in (l1,a1,l2): coll.add(x)
for label, fn in (('features.createPath(coll,False)', lambda: c.features.createPath(coll, False)),
                  ('features.createPath(l1,True)', lambda: c.features.createPath(l1, True)),
                  ('Path.create(l1,connected)', lambda: adsk.fusion.Path.create(l1, adsk.fusion.ChainedCurveOptions.connectedChainedCurves))):
    try:
        p = fn(); log('%s -> count=%s' % (label, None if p is None else p.count))
    except Exception as e:
        log('%s -> ERR %s' % (label, e))
