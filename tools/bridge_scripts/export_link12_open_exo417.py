"""EXO-417: export link_12 of the ACTIVE (unsaved variant) `exo generated` to
parts/link_12_open<T>.3mf, T = the TOTAL clearance read back off the solid (opening width - 20.0),
at High refinement as export_exo415.py. Logs the opening in the J2 stator frame at six depths
through the pad, the stator holes, the pad label (empty / full points of the label box at
z 7.8 and 7.5 in the J1 rotor frame) and the rotor-side numbers. No document change.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/export_link12_open_exo417.py --timeout 600"""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
P = '/Users/esteban/repos/exo/parts/'
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
def occ_named(occs, name):
    for i in range(occs.count):
        if occs.item(i).component.name == name: return occs.item(i)
def occ_prefix(occs, pre):
    for i in range(occs.count):
        if occs.item(i).component.name.startswith(pre): return occs.item(i)
def frame(o):
    a = o.transform.asArray(); M = [[a[4*i+j] for j in range(4)] for i in range(4)]
    return ([M[r][3]*10 for r in range(3)], [M[r][0] for r in range(3)], [M[r][1] for r in range(3)], [M[r][2] for r in range(3)])
arm = occ_named(root.occurrences, 'generated_R_arm_general')
lo_ = occ_named(arm.childOccurrences, 'g_link_12')
lb = lo_.component.bRepBodies.item(0)
bb = lb.boundingBox
say('g_link_12: bodies %d V %.4f faces %d bbox (%.3f,%.3f,%.3f)..(%.3f,%.3f,%.3f)' % (
    lo_.component.bRepBodies.count, lb.volume*1000, lb.faces.count,
    bb.minPoint.x*10, bb.minPoint.y*10, bb.minPoint.z*10, bb.maxPoint.x*10, bb.maxPoint.y*10, bb.maxPoint.z*10))
def inside(F):
    O, X, Y, Z = F
    return lambda x, y, z: lb.pointContainment(adsk.core.Point3D.create(
        *[(O[r] + x*X[r] + y*Y[r] + z*Z[r])/10 for r in range(3)])) in INS
FR, FS = frame(occ_prefix(arm.childOccurrences, 'GJ1_')), frame(occ_prefix(arm.childOccurrences, 'GJ2_'))
for tag, F in (('rotor GJ1', FR), ('stator GJ2', FS)):
    say('  %s frame: O %s X %s Y %s Z %s' % (tag, ['%.6f' % v for v in F[0]], ['%.8f' % v for v in F[1]],
        ['%.8f' % v for v in F[2]], ['%.8f' % v for v in F[3]]))
L = inside(FS)
widths = []
say('  pad faces at (-15,-15): z %.5f .. %.5f' % (bis(lambda z: L(-15.0, -15.0, z), -28.5, -36.0),
                                                bis(lambda z: L(-15.0, -15.0, z), -28.5, -22.0)))
for z in (-24.7, -25.0, -26.5, -28.5, -30.5, -32.3):
    xm = bis(lambda x: not L(x, 7.5, z), 0.0, -14.0); xp = bis(lambda x: not L(x, 7.5, z), 0.0, 14.0)
    ym = bis(lambda y: not L(0.0, y, z), 7.5, -14.0); yp = bis(lambda y: not L(-6.0, y, z), 7.5, 26.0)
    widths.append((xp - xm, yp - ym))
    say('  opening at z %.2f: x %.5f..%.5f (%.5f) y %.5f..%.5f (%.5f)' % (z, xm, xp, xp - xm, ym, yp, yp - ym))
for a in armgen.INSERT_ANGLES:
    c = (armgen.INSERT_RADIUS*math.cos(math.radians(a)), armgen.INSERT_RADIUS*math.sin(math.radians(a)))
    rr = [bis(lambda r: not L(c[0] + r*math.cos(math.radians(k)), c[1] + r*math.sin(math.radians(k)), -28.5), 0.2, 3.5) for k in (0, 90, 180, 270)]
    say('  hole @%g: O %.5f..%.5f' % (a, 2*min(rr), 2*max(rr)))
say('  mark groove at r 25.5 @90: floor z %.5f' % bis(lambda z: not L(0.0, 25.5, z), -24.0, -26.0))
R = inside(FR)
say('  rotor: pad top z %.5f (r 16 @135); counterbore floor %.5f; joint face z %.5f; centre hole r %.5f' % (
    bis(lambda z: R(16.0*math.cos(math.radians(135)), 16.0*math.sin(math.radians(135)), z), 4.0, 14.0),
    bis(lambda z: not R(13.35, 0, z), 8.5, 4.0), bis(lambda z: R(10*math.cos(math.radians(67.5)), 10*math.sin(math.radians(67.5)), z), -1.0, -8.5),
    bis(lambda r: not R(r*math.cos(math.radians(30)), r*math.sin(math.radians(30)), 5.0), 5.0, 9.0)))
r, a = armgen.ROTOR_LABEL_AT
cx, cy = r*math.cos(math.radians(a)), r*math.sin(math.radians(a))
for z in (7.95, 7.8, 7.65, 7.5):
    empty = sum(1 for i in range(41) for j in range(21) if not R(cx - 6 + 0.3*i, cy - 3 + 0.3*j, z))
    say('  label box 12 x 6 at (%.3f, %.3f): z %.2f empty points %d of 861' % (cx, cy, z, empty))
w = sum(x for x, _ in widths) / len(widths)
T = round(w - 20.0, 1)
assert all(abs(x - (20.0 + T)) < 1e-3 and abs(y - (34.0 + T)) < 1e-3 for x, y in widths), widths
path = P + 'link_12_open%.1f.3mf' % T
o = em.createC3MFExportOptions(lb, path); o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
say('  export -> %s %s' % (em.execute(o), path))
say('document isModified %s' % doc.isModified)
