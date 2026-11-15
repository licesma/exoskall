"""EXO-425: first export of parts/link_56.3mf and parts/link_67.3mf (fused links g_link_56 / g_link_67
of `generated_R_arm_general`, actuator opening 20.2 x 34.2, joint numbers on both ends, EXO-421) at
High refinement from `exo generated`, with the probes of export_exo402.py (rotor end) and
export_exo421.py (stator end). Export only: never edits or saves. Uses `exo generated` if it is open
(it must be unmodified), otherwise opens it read-only and closes it unsaved; re-activates whichever
document was active. Refuses to overwrite an existing parts file. Logs both joint frames of each link
and the Fusion volume / centre of mass so the meshes can be sliced in the same frames.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/export_exo425.py --timeout 900"""
import math, os
GEN_ID = 'urn:adsk.wipprod:dm.lineage:I6f3yGrzR8S8xg8G0BP0aw'   # exo generated (EXO-283)
GEN_NAME = 'exo generated'
LINKS = (('g_link_56', 'GJ5_', 'GJ6_'), ('g_link_67', 'GJ6_', 'GJ7_'))
P = '/Users/esteban/repos/exo/parts/'
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
for link, _, _ in LINKS:
    if os.path.exists(P + link[2:] + '.3mf'): raise RuntimeError('exists, not overwriting: ' + link[2:])
prev = app.activeDocument
say('active document: %r' % (prev.name if prev else None))
gen = None
for i in range(app.documents.count):
    d = app.documents.item(i)
    try:
        if d.dataFile is not None and d.dataFile.id == GEN_ID: gen = d
    except Exception: pass
opened = gen is None
if opened:
    df = app.data.findFileById(GEN_ID)
    say('data file %r v%s latest v%s' % (df.name, df.versionNumber, df.latestVersionNumber))
    gen = app.documents.open(df, True)
if gen.name != GEN_NAME: raise RuntimeError(gen.name)
mod0 = gen.isModified
say('exo generated: opened by this script %s, v%s (latest v%s), isModified %s' % (
    opened, gen.dataFile.versionNumber, gen.dataFile.latestVersionNumber, mod0))
if mod0: raise RuntimeError('exo generated has unsaved changes: not exporting from it')
try:
    design = adsk.fusion.Design.cast(gen.products.itemByProductType('DesignProductType'))
    root = design.rootComponent
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
        com = lb.physicalProperties.centerOfMass
        say('%s: bodies %d V %.4f faces %d bbox (%.3f,%.3f,%.3f)..(%.3f,%.3f,%.3f) CoM (%.3f,%.3f,%.3f) occ transform identity %s' % (
            link, lo_.component.bRepBodies.count, lb.volume*1000, lb.faces.count,
            bb.minPoint.x*10, bb.minPoint.y*10, bb.minPoint.z*10, bb.maxPoint.x*10, bb.maxPoint.y*10, bb.maxPoint.z*10,
            com.x*10, com.y*10, com.z*10, lo_.transform.isEqualTo(adsk.core.Matrix3D.create())))
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
                cb = [bis(lambda r: not L(11*math.cos(math.radians(a)) + r*math.cos(math.radians(k)), 11*math.sin(math.radians(a)) + r*math.sin(math.radians(k)), pad - 0.3), 1.0, 4.5) for a in (0, 90, 180, 270) for k in (90, 270)]
                say('    pad top z %.5f (r 16 @135); counterbore floor %.5f -> depth %.5f; O %.5f..%.5f' % (pad, floor, pad - floor, 2*min(cb), 2*max(cb)))
                say('    recess r %.5f at z -6.0 (@100), ceiling z %.5f (r 7.5 @100)' % (bis(lambda r: not at(r, 100, -6.0), 7.0, 9.0), bis(lambda z: not at(7.5, 100, z), -7.0, -4.0)))
                say('    disk r %.5f at z -2 (@100), shoulder z %.5f (r 16.5 @100)' % (bis(lambda r: at(r, 100, -2.0), 16, 19.5), bis(lambda z: at(16.5, 100, z), -1.0, -6.0)))
                say('    journal r %.5f at z -6.5 (@100), %.5f (@200)' % (bis(lambda r: at(r, 100, -6.5), 14, 16), bis(lambda r: at(r, 200, -6.5), 14, 16)))
                say('    joint face z %.5f (r 10 @67.5)' % bis(lambda z: at(10, 67.5, z), -1.0, -8.5))
                say('    socket top z %.5f (r 12.2 @22.5), %.5f (@202.5); socket r %.5f at z -7.0' % (
                    bis(lambda z: not at(12.2, 22.5, z), -7.9, -4.0), bis(lambda z: not at(12.2, 202.5, z), -7.9, -4.0),
                    bis(lambda r: not L(P2(12.2, 22.5)[0] + r, P2(12.2, 22.5)[1], -7.0), 0.5, 2.5)))
                say('    centre hole r %.5f at z 5 (@30)' % bis(lambda r: not at(r, 30, 5.0), 5.0, 9.0))
                say('    nothing below the joint face at -8.05 (r 12.2 @22.5): %s; solid between sockets at -7.0 @45: %s' % (not at(12.2, 22.5, -8.05), at(12.2, 45, -7.0)))
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
                for a in (0, 180, 270):
                    c = P2(19.25, a)
                    rr = [bis(lambda r: not L(c[0] + r*math.cos(math.radians(k)), c[1] + r*math.sin(math.radians(k)), -28.5), 0.2, 3.5) for k in (0, 90, 180, 270)]
                    say('    hole @%g: O %.5f..%.5f' % (a, 2*min(rr), 2*max(rr)))
                say('    mark groove at r 25.5 @90: floor z %.5f' % bis(lambda z: not L(0.0, 25.5, z), -24.0, -26.0))
        path = P + link[2:] + '.3mf'
        say('  export -> %s %s' % (export(lb, path), path))
finally:
    say('exo generated isModified before %s after %s' % (mod0, gen.isModified))
    if opened and not gen.isModified:
        gen.close(False); say('closed without saving')
    else:
        say('left open as found (opened=%s, modified=%s)' % (opened, gen.isModified))
    try:
        if prev is not None and prev.isValid: prev.activate(); say('re-activated %r' % prev.name)
    except Exception as e: say('re-activate failed: %r' % e)
