"""EXO-402: export parts/link_34.3mf and parts/link_45.3mf (fused links g_link_34 / g_link_45 of
`generated_R_arm_general`) at High refinement from `exo generated`, and bisect the fused rotor end
of each link in its rotor joint's module frame (GJ3_ / GJ4_), as export_exo370.py does for 12 / 23.
Opens `exo generated` read-only if it is not open, never saves, never edits; closes it without
saving only if this script opened it and it is unmodified, then re-activates the previously active
document. Refuses to overwrite an existing parts file. No bottom export.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/export_exo402.py --timeout 600"""
import math, os
GEN_ID = 'urn:adsk.wipprod:dm.lineage:I6f3yGrzR8S8xg8G0BP0aw'   # exo generated (EXO-283)
GEN_NAME = 'exo generated'
LINKS = (('g_link_34', 'GJ3_'), ('g_link_45', 'GJ4_'))
P = '/Users/esteban/repos/exo/parts/'
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
for link, _ in LINKS:
    if os.path.exists(P + link[2:] + '.3mf'): raise RuntimeError('exists, not overwriting: ' + link[2:])
prev = app.activeDocument
gen = None
for i in range(app.documents.count):
    d = app.documents.item(i)
    try:
        if d.dataFile is not None and d.dataFile.id == GEN_ID: gen = d
    except Exception: pass
opened = gen is None
if opened:
    df = app.data.findFileById(GEN_ID)
    say('data file %r v%s latest v%s folder %r' % (df.name, df.versionNumber, df.latestVersionNumber, df.parentFolder.name))
    gen = app.documents.open(df, True)
else:
    say('doc already open: %r v%s' % (gen.name, gen.dataFile.versionNumber))
if gen.name != GEN_NAME: raise RuntimeError(gen.name)
mod0 = gen.isModified
say('opened=%s isModified %s' % (opened, mod0))
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
    arm = occ_named(root.occurrences, 'generated_R_arm_general')
    say('arm children: %s' % sorted(arm.childOccurrences.item(i).component.name for i in range(arm.childOccurrences.count)))
    for link, J in LINKS:
        lo_ = occ_named(arm.childOccurrences, link)
        lb = lo_.component.bRepBodies.item(0)
        jo = occ_prefix(arm.childOccurrences, J)
        M = mat(jo.transform)
        O = [M[r][3]*10 for r in range(3)]; X = [M[r][0] for r in range(3)]; Y = [M[r][1] for r in range(3)]; Z = [M[r][2] for r in range(3)]
        L = lambda x, y, z: lb.pointContainment(adsk.core.Point3D.create(*[(O[r] + x*X[r] + y*Y[r] + z*Z[r])/10 for r in range(3)])) in INS
        P2 = lambda r, a: (r*math.cos(math.radians(a)), r*math.sin(math.radians(a)))
        def at(r, a, z): x, y = P2(r, a); return L(x, y, z)
        bb = lb.boundingBox
        say('%s (rotor joint %s): bodies %d V %.4f faces %d; arm bbox (%.3f,%.3f,%.3f)..(%.3f,%.3f,%.3f)' % (
            link, jo.component.name, lo_.component.bRepBodies.count, lb.volume*1000, lb.faces.count,
            bb.minPoint.x*10, bb.minPoint.y*10, bb.minPoint.z*10, bb.maxPoint.x*10, bb.maxPoint.y*10, bb.maxPoint.z*10))
        say('  module frame: O (%.4f,%.4f,%.4f) Z (%.5f,%.5f,%.5f) Y (%.5f,%.5f,%.5f)' % tuple(O + Z + Y))
        pad = bis(lambda z: at(16.0, 135, z), 4.0, 14.0)
        say('  pad top z %.5f (r 16 @135); outside r 18 @135 z %.5f' % (pad, bis(lambda z: at(19.0, 135, z), 4.0, 14.0)))
        floor = bis(lambda z: not at(13.35, 0, z), pad + 0.5, pad - 4.0)
        cb = [bis(lambda r: not L(11*math.cos(math.radians(a)) + r*math.cos(math.radians(k)), 11*math.sin(math.radians(a)) + r*math.sin(math.radians(k)), pad - 0.3), 1.0, 4.5) for a in (0, 90, 180, 270) for k in (90, 270)]
        say('  counterbore floor z %.5f -> depth %.5f; O %.5f..%.5f' % (floor, pad - floor, 2*min(cb), 2*max(cb)))
        say('  recess r %.5f at z -6.0 (@100), ceiling z %.5f (r 7.5 @100)' % (bis(lambda r: not at(r, 100, -6.0), 7.0, 9.0), bis(lambda z: not at(7.5, 100, z), -7.0, -4.0)))
        say('  disk r %.5f at z -2 (@100), shoulder z %.5f (r 16.5 @100)' % (bis(lambda r: at(r, 100, -2.0), 16, 19.5), bis(lambda z: at(16.5, 100, z), -1.0, -6.0)))
        say('  journal r %.5f at z -6.5 (@100), %.5f (@200)' % (bis(lambda r: at(r, 100, -6.5), 14, 16), bis(lambda r: at(r, 200, -6.5), 14, 16)))
        say('  joint face z %.5f (r 10 @67.5)' % bis(lambda z: at(10, 67.5, z), -1.0, -8.5))
        say('  socket top z %.5f (r 12.2 @22.5), %.5f (@202.5); socket r %.5f at z -7.0' % (
            bis(lambda z: not at(12.2, 22.5, z), -7.9, -4.0), bis(lambda z: not at(12.2, 202.5, z), -7.9, -4.0),
            bis(lambda r: not L(P2(12.2, 22.5)[0] + r, P2(12.2, 22.5)[1], -7.0), 0.5, 2.5)))
        say('  centre hole r %.5f at z 5 (@30)' % bis(lambda r: not at(r, 30, 5.0), 5.0, 9.0))
        say('  nothing below the joint face at -8.05 (r 12.2 @22.5): %s; solid between sockets at -7.0 @45: %s' % (not at(12.2, 22.5, -8.05), at(12.2, 45, -7.0)))
        path = P + link[2:] + '.3mf'
        say('  export -> %s %s' % (export(lb, path), path))
finally:
    say('exo generated isModified before %s after %s' % (mod0, gen.isModified))
    if opened and not gen.isModified:
        gen.close(False); say('closed without saving')
    else:
        say('left open (opened=%s, modified=%s)' % (opened, gen.isModified))
    try:
        if prev is not None and prev.isValid: prev.activate(); say('re-activated %r' % prev.name)
    except Exception as e: say('re-activate failed: %r' % e)
