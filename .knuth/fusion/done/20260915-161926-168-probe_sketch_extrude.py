# EXO-58 API probe (direct design): construction plane by Plane, sketch on it, chained
# lines + 3-point arcs, symmetric extrude (full length), Join with participantBodies,
# Cut, bbox / volume / face read-back. Temporary root component, deleted at the end.
import math, traceback
root = design.rootComponent
identity = adsk.core.Matrix3D.create()
occ = root.occurrences.addNewComponent(identity)
comp = occ.component
comp.name = 'probe_sketch_extrude_tmp'
ST = {0: 'plane', 1: 'cyl', 2: 'cone', 3: 'sphere', 4: 'torus', 5: 'ellcyl', 6: 'ellcone', 7: 'nurbs'}
def desc(b, tag):
    bb = b.boundingBox
    kinds = {}
    for i in range(b.faces.count):
        k = ST.get(b.faces.item(i).geometry.surfaceType, '?')
        kinds[k] = kinds.get(k, 0) + 1
    log('%s: vol %.1f mm3 bbox mm x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f] faces %s' % (
        tag, b.volume * 1000, bb.minPoint.x*10, bb.maxPoint.x*10, bb.minPoint.y*10, bb.maxPoint.y*10,
        bb.minPoint.z*10, bb.maxPoint.z*10, kinds))
try:
    o = adsk.core.Point3D.create(1.0, 2.0, 3.0)
    e1 = adsk.core.Vector3D.create(0, 0, -1)
    e2 = adsk.core.Vector3D.create(1, 0, 0)
    n = e1.crossProduct(e2)
    pin = comp.constructionPlanes.createInput()
    pin.setByPlane(adsk.core.Plane.create(o, n))
    cp = comp.constructionPlanes.add(pin)
    sk = comp.sketches.add(cp)
    log('sketch origin %s x %s y %s' % ([round(v,3) for v in sk.origin.asArray()], [round(v,3) for v in sk.xDirection.asArray()], [round(v,3) for v in sk.yDirection.asArray()]))
    def W(a, b):
        return adsk.core.Point3D.create(o.x + (a*e1.x + b*e2.x)/10.0, o.y + (a*e1.y + b*e2.y)/10.0, o.z + (a*e1.z + b*e2.z)/10.0)
    def P2(a, b):
        return sk.modelToSketchSpace(W(a, b))
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    c1 = lines.addByTwoPoints(P2(0, 0), P2(30, 0))
    c2 = arcs.addByThreePoints(c1.endSketchPoint, P2(30 + 10*math.cos(math.radians(-45)), 10 + 10*math.sin(math.radians(-45))), P2(40, 10))
    c3 = lines.addByTwoPoints(c2.endSketchPoint, P2(40, 20))
    c4 = lines.addByTwoPoints(c3.endSketchPoint, P2(0, 20))
    c5 = lines.addByTwoPoints(c4.endSketchPoint, c1.startSketchPoint)
    for tag, c in (('c1', c1), ('c2', c2), ('c3', c3), ('c4', c4), ('c5', c5)):
        s, e = c.startSketchPoint.worldGeometry, c.endSketchPoint.worldGeometry
        log('  %s world start (%.1f,%.1f,%.1f) end (%.1f,%.1f,%.1f) mm' % (tag, s.x*10, s.y*10, s.z*10, e.x*10, e.y*10, e.z*10))
    log('arc radius %.2f mm centre world %s' % (c2.radius*10, [round(v*10,2) for v in c2.centerSketchPoint.worldGeometry.asArray()]))
    log('profiles: %d area %.3f cm2' % (sk.profiles.count, sk.profiles.item(0).areaProperties().area))
    ext = comp.features.extrudeFeatures
    ei = ext.createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ei.setSymmetricExtent(adsk.core.ValueInput.createByReal(2.5), True)   # 25 mm full length
    f = ext.add(ei)
    body = f.bodies.item(0)
    desc(body, 'body1 (expect 778.5*25=19463)')
    pin2 = comp.constructionPlanes.createInput()
    pin2.setByPlane(adsk.core.Plane.create(adsk.core.Point3D.create(1.0, 2.0, 2.5), adsk.core.Vector3D.create(0, 0, 1)))
    cp2 = comp.constructionPlanes.add(pin2)
    sk2 = comp.sketches.add(cp2)
    log('sketch2 origin %s x %s y %s' % ([round(v,3) for v in sk2.origin.asArray()], [round(v,3) for v in sk2.xDirection.asArray()], [round(v,3) for v in sk2.yDirection.asArray()]))
    cpt = sk2.modelToSketchSpace(adsk.core.Point3D.create(1.0, 2.0, 2.5))
    circ = sk2.sketchCurves.sketchCircles.addByCenterRadius(cpt, 2.0)
    log('circle centre world %s r %.2f' % ([round(v*10,2) for v in circ.centerSketchPoint.worldGeometry.asArray()], circ.radius*10))
    log('sketch2 profiles: %d' % sk2.profiles.count)
    ei2 = ext.createInput(sk2.profiles.item(0), adsk.fusion.FeatureOperations.JoinFeatureOperation)
    ei2.setSymmetricExtent(adsk.core.ValueInput.createByReal(0.8), True)
    ei2.participantBodies = [body]
    f2 = ext.add(ei2)
    log('join: bodies in comp %d, feature bodies %d' % (comp.bRepBodies.count, f2.bodies.count))
    desc(comp.bRepBodies.item(0), 'after join (expect 19463+6329=25792, x from -10)')
    # cut: a separate sketch with a small circle straddling the x=30 face
    sk3 = comp.sketches.add(cp2)
    sk3.sketchCurves.sketchCircles.addByCenterRadius(sk3.modelToSketchSpace(adsk.core.Point3D.create(3.0, 2.0, 2.5)), 0.3)
    log('sketch3 profiles: %d' % sk3.profiles.count)
    ei3 = ext.createInput(sk3.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
    ei3.setSymmetricExtent(adsk.core.ValueInput.createByReal(6.0), True)
    ei3.participantBodies = [comp.bRepBodies.item(0)]
    f3 = ext.add(ei3)
    desc(comp.bRepBodies.item(0), 'after cut (expect -353)')
    log('bodies %d' % comp.bRepBodies.count)
except Exception:
    log('EXC ' + traceback.format_exc())
finally:
    log('delete tmp: %s' % occ.deleteMe())
