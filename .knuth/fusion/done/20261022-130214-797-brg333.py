import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
design = adsk.fusion.Design.cast(app.activeProduct)
root = design.rootComponent
occ = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name == BuildArm.MODULE_V3_NAME][0]
brg = [occ.childOccurrences.item(i) for i in range(occ.childOccurrences.count) if 'Bearing' in occ.childOccurrences.item(i).component.name][0]
up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
for i in range(brg.bRepBodies.count):
    b = brg.bRepBodies.item(i)
    rs, zs = [], []
    for k in range(b.vertices.count):
        p = b.vertices.item(k).geometry; rs.append(math.hypot(p.x, p.y) * 10); zs.append(p.z * 10)
    hit = []
    for tag, part in (('top', up), ('bottom', lo)):
        coll = adsk.core.ObjectCollection.create(); coll.add(b); coll.add(part.bRepBodies.item(0))
        inp = design.createInterferenceInput(coll)
        res = design.analyzeInterference(inp)
        if res and res.count:
            for j in range(res.count):
                ib = res.item(j).interferenceBody
                bb = ib.boundingBox
                hit.append('%s %.3f mm3 z %.3f..%.3f xy %.2f' % (tag, ib.volume * 1000, bb.minPoint.z * 10, bb.maxPoint.z * 10, max(abs(bb.maxPoint.x), abs(bb.minPoint.x)) * 10))
    say('%-8s r %s z %s %s' % (b.name, ('%.3f..%.3f' % (min(rs), max(rs))) if rs else 'n/a', ('%.3f..%.3f' % (min(zs), max(zs))) if zs else 'n/a', hit))
INS = (adsk.fusion.PointContainment.PointInsidePointContainment, adsk.fusion.PointContainment.PointOnPointContainment)
lb = lo.bRepBodies.item(0)
def s(r, z): return lb.pointContainment(adsk.core.Point3D.create(r / 10, 0, z / 10)) in INS
for z in (-12.0, -11.0):
    prof = ''.join('#' if s(7.9 + 0.01 * k, z) else '.' for k in range(0, 621))
    edges = [round(7.9 + 0.01 * k, 2) for k in range(1, 621) if prof[k] != prof[k - 1]]
    say('pocket line 0 deg z %.1f: solid/void edges at r %s' % (z, edges))
