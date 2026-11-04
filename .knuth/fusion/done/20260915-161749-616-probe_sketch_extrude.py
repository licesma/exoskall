# EXO-58 API probe (direct design): construction plane by Plane, sketch on it, chained
# lines + 3-point arcs, symmetric extrude (full length), Join with participantBodies,
# Cut, bbox read-back. Creates a temporary root component and deletes it at the end.
import math, traceback
root = design.rootComponent
identity = adsk.core.Matrix3D.create()
occ = root.occurrences.addNewComponent(identity)
comp = occ.component
comp.name = 'probe_sketch_extrude_tmp'
try:
    log('design type: %s' % design.designType)
    # oriented plane: origin (10, 20, 30) mm, e1 = (0,0,-1), e2 = (1,0,0) -> normal = e1 x e2 = (0,-1,0)... compute
    o = adsk.core.Point3D.create(1.0, 2.0, 3.0)
    e1 = adsk.core.Vector3D.create(0, 0, -1)
    e2 = adsk.core.Vector3D.create(1, 0, 0)
    n = e1.crossProduct(e2)
    log('normal %s' % ([n.x, n.y, n.z],))
    pin = comp.constructionPlanes.createInput()
    ok = pin.setByPlane(adsk.core.Plane.create(o, n))
    log('setByPlane -> %s' % ok)
    cp = comp.constructionPlanes.add(pin)
    log('cplane added: %s geometry origin %s normal %s' % (cp.name, [round(v,3) for v in cp.geometry.origin.asArray()], [round(v,3) for v in cp.geometry.normal.asArray()]))
    sk = comp.sketches.add(cp)
    log('sketch origin %s x %s y %s' % ([round(v,3) for v in sk.origin.asArray()], [round(v,3) for v in sk.xDirection.asArray()], [round(v,3) for v in sk.yDirection.asArray()]))
    def P2(a, b):  # mm in (e1, e2) -> sketch space
        w = adsk.core.Point3D.create(o.x + (a*e1.x + b*e2.x)/10.0, o.y + (a*e1.y + b*e2.y)/10.0, o.z + (a*e1.z + b*e2.z)/10.0)
        return sk.modelToSketchSpace(w)
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    # loop: (0,0)->(30,0) line, arc r 10 from (30,0) via (30+10*cos45.. ) to (40,10)? use quarter: centre (30,10): from (30,0) [270deg] through (30+7.071, 10-7.071) to (40,10)[0deg]
    c1 = lines.addByTwoPoints(P2(0, 0), P2(30, 0))
    c2 = arcs.addByThreePoints(c1.endSketchPoint, P2(30 + 10*math.cos(math.radians(-45)), 10 + 10*math.sin(math.radians(-45))), P2(40, 10))
    c3 = lines.addByTwoPoints(c2.endSketchPoint, P2(40, 20))
    c4 = lines.addByTwoPoints(c3.endSketchPoint, P2(0, 20))
    c5 = lines.addByTwoPoints(c4.endSketchPoint, c1.startSketchPoint)
    log('profiles: %d' % sk.profiles.count)
    ext = comp.features.extrudeFeatures
    ei = ext.createInput(sk.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ei.setSymmetricExtent(adsk.core.ValueInput.createByReal(2.5), True)   # 25 mm full length
    f = ext.add(ei)
    body = f.bodies.item(0)
    bb = body.boundingBox
    log('body1 bbox mm x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f] faces %d' % (bb.minPoint.x*10, bb.maxPoint.x*10, bb.minPoint.y*10, bb.maxPoint.y*10, bb.minPoint.z*10, bb.maxPoint.z*10, body.faces.count))
    # expected: e1=(0,0,-1) -> world z from 30 down to -10 ; e2 = x from 10 to 50 ; width +/-12.5 about y=20 (normal (0,-1,0)?) -> y 7.5..32.5
    # join a circle on an XY-parallel plane through the middle
    pin2 = comp.constructionPlanes.createInput()
    pin2.setByPlane(adsk.core.Plane.create(adsk.core.Point3D.create(1.0, 2.0, 2.5), adsk.core.Vector3D.create(0, 0, 1)))
    cp2 = comp.constructionPlanes.add(pin2)
    sk2 = comp.sketches.add(cp2)
    cpt = sk2.modelToSketchSpace(adsk.core.Point3D.create(1.0, 2.0, 2.5))
    sk2.sketchCurves.sketchCircles.addByCenterRadius(cpt, 2.0)
    sk2.sketchCurves.sketchCircles.addByCenterRadius(sk2.modelToSketchSpace(adsk.core.Point3D.create(3.0, 2.0, 2.5)), 0.3)
    log('sketch2 profiles: %d' % sk2.profiles.count)
    ei2 = ext.createInput(sk2.profiles.item(0), adsk.fusion.FeatureOperations.JoinFeatureOperation)
    ei2.setSymmetricExtent(adsk.core.ValueInput.createByReal(0.8), True)
    ei2.participantBodies = [body]
    f2 = ext.add(ei2)
    log('join: bodies in comp %d, feature bodies %d' % (comp.bRepBodies.count, f2.bodies.count))
    # cut the small circle through
    coll = adsk.core.ObjectCollection.create()
    for i in range(sk2.profiles.count):
        pr = sk2.profiles.item(i)
        ar = pr.areaProperties().area
        log('  profile %d area %.3f cm2' % (i, ar))
        if ar < 1.0:
            coll.add(pr)
    ei3 = ext.createInput(coll, adsk.fusion.FeatureOperations.CutFeatureOperation)
    ei3.setSymmetricExtent(adsk.core.ValueInput.createByReal(6.0), True)
    ei3.participantBodies = [body]
    f3 = ext.add(ei3)
    b = comp.bRepBodies.item(0)
    bb = b.boundingBox
    log('final: bodies %d, bbox mm x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f] vol %.1f mm3 faces %d' % (comp.bRepBodies.count, bb.minPoint.x*10, bb.maxPoint.x*10, bb.minPoint.y*10, bb.maxPoint.y*10, bb.minPoint.z*10, bb.maxPoint.z*10, b.volume*1000, b.faces.count))
    sk.isVisible = False; cp.isLightBulbOn = False
    log('hide ok')
except Exception:
    log('EXC ' + traceback.format_exc())
finally:
    log('delete tmp: %s' % occ.deleteMe())
