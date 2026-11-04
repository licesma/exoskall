"""EXO-363: the O42.2 seat is final. `stator EXO-341 export` (direct design, plain body) gets the
seat r 21.10 over file z -3.5..3.5 (cut as EXO-350; shoulder z -3.5 and the r 19.25 step
untouched), no label; the `stator d4x.x export` test components (EXO-350/355/358) are deleted;
parts/stator.3mf re-exported High; document saved (EXO-242). Nothing else written.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/final_stator_exo363.py --timeout 300
"""
import importlib, sys, math, re
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
NAME, R, R0, Z0, Z1 = 'stator EXO-341 export', 21.10, 21.05, -3.5, 3.5
OUT = '/Users/esteban/repos/exo/parts/stator.3mf'
doc = app.activeDocument
if doc.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r' % doc.name)
design = adsk.fusion.Design.cast(app.activeProduct)
root = design.rootComponent
INS = (adsk.fusion.PointContainment.PointInsidePointContainment, adsk.fusion.PointContainment.PointOnPointContainment)
P = lambda x, y, z: adsk.core.Point3D.create(x/10, y/10, z/10)
occ = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name == NAME][0]
comp = occ.component
if comp.bRepBodies.count != 1: raise RuntimeError('bodies %d' % comp.bRepBodies.count)
src = comp.bRepBodies.item(0)
v0 = src.volume*1000
mod_st = BuildArm.module_stator_occ([root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name == 'sensor module 2 rotor'][0], in_context=True).component.bRepBodies.item(0)
def vols(c): return sum(c.bRepBodies.item(i).volume for i in range(c.bRepBodies.count))*1000
other = {root.occurrences.item(i).component.name: root.occurrences.item(i).component for i in range(root.occurrences.count)
         if root.occurrences.item(i).component.name in ('sensor module 2', 'sensor module 2 rotor')}
other_v = {n: vols(c) for n, c in other.items()}
say('source V %.4f; module stator V %.4f; others %s' % (v0, mod_st.volume*1000, other_v))
def radius_at(b, ang, z, lo=19.0, hi=23.0):
    a = math.radians(ang)
    for _ in range(40):
        m = (lo + hi)/2
        if b.pointContainment(P(m*math.cos(a), m*math.sin(a), z)) in INS: hi = m
        else: lo = m
    return (lo + hi)/2
r_before = [radius_at(src, a, 0.0) for a in (0, 90, 180, 270)]
say('seat r before: %s' % ['%.5f' % r for r in r_before])
if max(abs(r - R0) for r in r_before) > 1e-4:
    raise RuntimeError('seat is not r 21.05: already changed?')
tbm = adsk.fusion.TemporaryBRepManager.get()
tmp = tbm.copy(src)
cyl = tbm.createCylinderOrCone(P(0, 0, Z0), R/10, P(0, 0, Z1), R/10)
tbm.booleanOperation(tmp, cyl, adsk.fusion.BooleanTypes.DifferenceBooleanType)
src.deleteMe()
b = comp.bRepBodies.add(tmp); b.name = 'stator'
v = b.volume*1000
say('%r: V %.4f = %.4f - %.4f (pi(R^2-R0^2)*7 = %.4f); lumps %d shells %d bodies %d bulb %s'
    % (NAME, v, v0, v0 - v, math.pi*(R*R - R0*R0)*7, b.lumps.count, b.shells.count, comp.bRepBodies.count, occ.isLightBulbOn))
rs = [radius_at(b, a, z) for a in (0, 45, 90, 135, 168, 225, 270, 315) for z in (-3.4, 0.0, 3.4)]
say('seat r (24 probes) min %.5f max %.5f -> O%.4f' % (min(rs), max(rs), 2*sum(rs)/len(rs)))
say('step bore r at z -3.6 (0/90/180/270): %s; stop block inner r z 5: %.4f' % (['%.4f' % radius_at(b, a, -3.6, 15.0, 23.0) for a in (0, 90, 180, 270)], radius_at(b, 168, 5.0)))
for i in range(b.faces.count):
    f = b.faces.item(i); g = f.geometry; fb = f.boundingBox
    if g.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType and math.hypot(g.origin.x, g.origin.y) < 1e-6 and g.radius*10 > 19:
        say('  axis cyl r %.4f z %.4f..%.4f' % (g.radius*10, fb.minPoint.z*10, fb.maxPoint.z*10))
    if g.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType and abs(fb.minPoint.z*10 + 3.5) < 1e-4 and abs(fb.maxPoint.z*10 + 3.5) < 1e-4:
        say('  plane z -3.5 area %.3f bbox x %.2f..%.2f' % (f.area*100, fb.minPoint.x*10, fb.maxPoint.x*10))
for i in reversed(range(root.occurrences.count)):
    n = root.occurrences.item(i).component.name
    if re.match(r'^stator d4\d\.\d export$', n):
        root.occurrences.item(i).deleteMe(); say('deleted %r' % n)
em = design.exportManager
opts = em.createC3MFExportOptions(b, OUT)
opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
say('export -> %s %s' % (em.execute(opts), OUT))
say('module stator after V %.4f' % (mod_st.volume*1000))
for n, c in other.items(): say('%r V after %.4f (before %.4f)' % (n, vols(c), other_v[n]))
say('root now: %s' % [root.occurrences.item(i).component.name for i in range(root.occurrences.count)])
say(BuildArm.save_active_document(app))
d = app.activeDocument
say('isModified %s v%s' % (d.isModified, d.dataFile.versionNumber))
