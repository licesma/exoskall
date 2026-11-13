"""EXO-415: export parts/link_12.3mf, link_23.3mf, link_34.3mf, link_45.3mf (fused links of
`generated_R_arm_general`, actuator opening 20.4 x 34.4) at High refinement from the ACTIVE
`exo generated`, as export_exo370.py / export_exo402.py do. Links only: stator.3mf and bottom.3mf
are not written. No document change. Per link it bisects the solid in the module frame of the
link's STATOR joint (the opening: x and y walls at three depths through the pad, the three
r 19.25 holes, the pad faces) and in its ROTOR joint (pad top, counterbores, journal, sockets,
joint face), and logs both frames so the meshes can be sliced in the same frames.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/export_exo415.py --timeout 900"""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
P = '/Users/esteban/repos/exo/parts/'
LINKS = (('g_link_12', 'GJ1_', 'GJ2_'), ('g_link_23', 'GJ2_', 'GJ3_'),
         ('g_link_34', 'GJ3_', 'GJ4_'), ('g_link_45', 'GJ4_', 'GJ5_'))
doc = app.activeDocument
if doc.name != BuildArm.GEN_DOC_NAME: raise RuntimeError(doc.name)
say('document %r v%s isModified %s' % (doc.name, doc.dataFile.versionNumber, doc.isModified))
design = adsk.fusion.Design.cast(app.activeProduct); root = design.rootComponent
em = design.exportManager
INS = (adsk.fusion.PointContainment.PointInsidePointContainment, adsk.fusion.PointContainment.PointOnPointContainment)
def bis(f, a, b_, n=45):
    for _ in range(n):
        m = (a + b_) / 2
        a, b_ = (m, b_) if f(m) else (a, m)
    return (a + b_) / 2
def export(body, path):
    o = em.createC3MFExportOptions(body, path); o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    return em.execute(o)
def mat(m):
    a = m.asArray(); return [[a[4*i+j] for j in range(4)] for i in range(4)]
def occ_named(occs, name):
    for i in range(occs.count):
        if occs.item(i).component.name == name: return occs.item(i)
def occ_prefix(occs, pre):
    for i in range(occs.count):
        if occs.item(i).component.name.startswith(pre): return occs.item(i)
def frame(o):
    M = mat(o.transform)
    return ([M[r][3]*10 for r in range(3)], [M[r][0] for r in range(3)], [M[r][1] for r in range(3)], [M[r][2] for r in range(3)])
arm = occ_named(root.occurrences, 'generated_R_arm_general')
for link, JR, JS in LINKS:
    lo_ = occ_named(arm.childOccurrences, link)
    lb = lo_.component.bRepBodies.item(0)
    bb = lb.boundingBox
    say('%s: bodies %d V %.4f faces %d bbox (%.3f,%.3f,%.3f)..(%.3f,%.3f,%.3f)' % (
        link, lo_.component.bRepBodies.count, lb.volume*1000, lb.faces.count,
        bb.minPoint.x*10, bb.minPoint.y*10, bb.minPoint.z*10, bb.maxPoint.x*10, bb.maxPoint.y*10, bb.maxPoint.z*10))
    for tag, J in (('rotor', JR), ('stator', JS)):
        jo = occ_prefix(arm.childOccurrences, J)
        O, X, Y, Z = frame(jo)
        say('  %s frame %s: O %s X %s Y %s Z %s' % (tag, jo.component.name, ['%.6f' % v for v in O],
            ['%.8f' % v for v in X], ['%.8f' % v for v in Y], ['%.8f' % v for v in Z]))
        L = (lambda O, X, Y, Z: lambda x, y, z: lb.pointContainment(adsk.core.Point3D.create(
            *[(O[r] + x*X[r] + y*Y[r] + z*Z[r])/10 for r in range(3)])) in INS)(O, X, Y, Z)
        P2 = lambda r, a: (r*math.cos(math.radians(a)), r*math.sin(math.radians(a)))
        def at(r, a, z): x, y = P2(r, a); return L(x, y, z)
        if tag == 'rotor':
            pad = bis(lambda z: at(16.0, 135, z), 4.0, 14.0)
            floor = bis(lambda z: not at(13.35, 0, z), pad + 0.5, pad - 4.0)
            say('    pad top z %.5f; counterbore floor %.5f; journal r %.5f (@100) at -6.5; joint face z %.5f; socket top z %.5f; centre hole r %.5f' % (
                pad, floor, bis(lambda r: at(r, 100, -6.5), 14, 16), bis(lambda z: at(10, 67.5, z), -1.0, -8.5),
                bis(lambda z: not at(12.2, 22.5, z), -7.9, -4.0), bis(lambda r: not at(r, 30, 5.0), 5.0, 9.0)))
        else:
            zf = bis(lambda z: L(-15.0, -15.0, z), -28.5, -22.0)
            zb = bis(lambda z: L(-15.0, -15.0, z), -28.5, -36.0)
            say('    pad faces at (-15,-15): z %.5f .. %.5f' % (zb, zf))
            for z in (-24.7, -25.0, -26.5, -28.5, -30.5, -32.3):
                if not L(-15.0, -15.0, z):
                    say('    z %.2f: no pad at (-15,-15)' % z); continue
                xm = bis(lambda x: not L(x, 7.5, z), 0.0, -14.0); xp = bis(lambda x: not L(x, 7.5, z), 0.0, 14.0)
                ym = bis(lambda y: not L(0.0, y, z), 7.5, -14.0)
                yp = bis(lambda y: not L(-6.0, y, z), 7.5, 26.0)
                say('    opening at z %.2f: x %.5f..%.5f (%.5f) y %.5f..%.5f (%.5f)' % (z, xm, xp, xp - xm, ym, yp, yp - ym))
            for a in armgen.INSERT_ANGLES:
                c = P2(armgen.INSERT_RADIUS, a)
                rr = [bis(lambda r: not L(c[0] + r*math.cos(math.radians(k)), c[1] + r*math.sin(math.radians(k)), -28.5), 0.2, 3.5) for k in (0, 90, 180, 270)]
                say('    hole @%g: O %.5f..%.5f' % (a, 2*min(rr), 2*max(rr)))
            say('    mark groove at r 25.5 @90: floor z %.5f' % bis(lambda z: not L(0.0, 25.5, z), -24.0, -26.0))
    path = P + link[2:] + '.3mf'
    say('  export -> %s %s' % (export(lb, path), path))
say('document isModified %s' % doc.isModified)
