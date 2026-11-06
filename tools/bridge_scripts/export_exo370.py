"""EXO-370/375: export parts/bottom.3mf (module bottom, only with WITH_BOTTOM) and
parts/link_12_p<PAD>.3mf / link_23_p<PAD>.3mf (fused links, PAD = armgen.ROTOR_PAD_T of the
arm as built, read off the solid) at High refinement from `exo generated`, and bisect the fused rotor end of each link in
its rotor joint's module frame (pad top, disk, shoulder, journal, joint face, pegs, centre hole,
M3 holes). No document change.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/export_exo370.py --timeout 600"""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
P = '/Users/esteban/repos/exo/parts/'
doc = app.activeDocument
if doc.name != BuildArm.GEN_DOC_NAME: raise RuntimeError(doc.name)
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
occ = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name == BuildArm.MODULE_ROTOR_NAME][0]
ok, lines = BuildArm.measure_rotor_v3(occ)
say('module probes %s (%d)' % ('ALL OK' if ok else 'WRONG', len(lines)))
up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
b = lo.component.bRepBodies.item(0)
WITH_BOTTOM = globals().get('WITH_BOTTOM', True)
say('%s V %.3f -> %s' % (lo.component.name, b.volume*1000,
    export(b, P + 'bottom.3mf') if WITH_BOTTOM else 'not exported'))
def mat(m):
    a = m.asArray(); return [[a[4*i+j] for j in range(4)] for i in range(4)]
def occ_named(occs, name):
    for i in range(occs.count):
        if occs.item(i).component.name == name: return occs.item(i)
def occ_prefix(occs, pre):
    for i in range(occs.count):
        if occs.item(i).component.name.startswith(pre): return occs.item(i)
arm = occ_named(root.occurrences, 'generated_R_arm_general')
for link, J in (('g_link_12', 'GJ1_'), ('g_link_23', 'GJ2_')):
    lo_ = occ_named(arm.childOccurrences, link)
    lb = lo_.component.bRepBodies.item(0)
    M = mat(occ_prefix(arm.childOccurrences, J).transform)
    O = [M[r][3]*10 for r in range(3)]; X = [M[r][0] for r in range(3)]; Y = [M[r][1] for r in range(3)]; Z = [M[r][2] for r in range(3)]
    L = lambda x, y, z: lb.pointContainment(adsk.core.Point3D.create(*[(O[r] + x*X[r] + y*Y[r] + z*Z[r])/10 for r in range(3)])) in INS
    P2 = lambda r, a: (r*math.cos(math.radians(a)), r*math.sin(math.radians(a)))
    def at(r, a, z): x, y = P2(r, a); return L(x, y, z)
    say('%s: bodies %d V %.4f faces %d' % (link, lo_.component.bRepBodies.count, lb.volume*1000, lb.faces.count))
    pad = bis(lambda z: at(16.0, 135, z), 4.0, 14.0)
    say('  pad top z %.5f (r 16 @135, inside the head flat); outside r 18 @135 z %.5f' % (pad, bis(lambda z: at(19.0, 135, z), 4.0, 14.0)))
    say('  label: solid at (13.8 @45, z pad-0.2) %s, (13.8 @45, pad-0.45) %s' % (at(13.8, 45, pad - 0.2), at(13.8, 45, pad - 0.45)))
    say('  disk r %.5f at z -2 (@100), shoulder z %.5f (r 16.5 @100)' % (bis(lambda r: at(r, 100, -2.0), 16, 19.5), bis(lambda z: at(16.5, 100, z), -1.0, -6.0)))
    say('  journal r %.5f at z -6.5 (@100), %.5f (@200)' % (bis(lambda r: at(r, 100, -6.5), 14, 16), bis(lambda r: at(r, 200, -6.5), 14, 16)))
    say('  joint face z %.5f (r 10 @67.5)' % bis(lambda z: at(10, 67.5, z), -1.0, -8.5))
    say('  peg tip z %.5f (r 12.2 @22.5), peg r %.5f' % (bis(lambda z: at(12.2, 22.5, z), -8.5, -10.0),
        bis(lambda r: L(P2(12.2, 22.5)[0] + r, P2(12.2, 22.5)[1], -8.6), 0.5, 2.5)))
    say('  centre hole r %.5f at z 5 (@30), %.5f at z -7.5 (@200)' % (bis(lambda r: not at(r, 30, 5.0), 6.0, 9.0), bis(lambda r: not at(r, 200, -7.5), 6.0, 9.0)))
    for a in (0, 90, 180, 270):
        cx, cy = P2(11, a)
        hr = [bis(lambda r: not L(cx + r*math.cos(math.radians(k)), cy + r*math.sin(math.radians(k)), zz), 0.5, 2.5) for k in (45, 225) for zz in (3.0, -7.0)]
        say('  M3 @%3d: r %s; open at z pad-0.1 %s, -7.9 %s' % (a, ['%.4f' % v for v in hr], not L(cx, cy, pad - 0.1), not L(cx, cy, -7.9)))
    say('  air below the pegs at -9.25: %s; solid between pegs at -8.6 @45: %s' % (not at(12.2, 22.5, -9.25), at(12.2, 45, -8.6)))
    path = P + '%s_p%d.3mf' % (link[2:], int(round(pad)))
    say('  export -> %s %s' % (export(lb, path), path))
say('isModified %s' % app.activeDocument.isModified)
