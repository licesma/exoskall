import adsk.core, adsk.fusion
root = design.rootComponent
for i in range(root.occurrences.count-1,-1,-1):
    o=root.occurrences.item(i)
    if o.component.name.startswith('PROBE_'): o.deleteMe()
pref = app.preferences.productPreferences.itemByName('Design')
log('pref=%s is3DSketchingAllowed=%s' % (pref, getattr(pref,'is3DSketchingAllowed','n/a')))
try:
    pref.is3DSketchingAllowed = True
    log('set -> %s' % pref.is3DSketchingAllowed)
except Exception as e:
    log('set ERR %s' % e)
occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
c = occ.component; c.name='PROBE_3d'
sk = c.sketches.add(c.xYConstructionPlane)
P = adsk.core.Point3D.create
l = sk.sketchCurves.sketchLines.addByTwoPoints(P(0,0,0), P(0,0,5))
sp = l.endSketchPoint.worldGeometry
log('line end world = (%.3f, %.3f, %.3f)' % (sp.x, sp.y, sp.z))
try:
    a = sk.sketchCurves.sketchArcs.addByThreePoints(P(0,0,5), P(1-0.7071*1, 0, 5+0.7071), P(1,0,6))
    w0=a.startSketchPoint.worldGeometry; w1=a.endSketchPoint.worldGeometry
    log('arc start (%.3f,%.3f,%.3f) end (%.3f,%.3f,%.3f)'%(w0.x,w0.y,w0.z,w1.x,w1.y,w1.z))
except Exception as e:
    log('arc ERR %s' % e)
