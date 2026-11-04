"""EXO-364: export parts/top.3mf (module rotor top, rotor key) and parts/link_12.3mf / link_23.3mf
(High) from `exo generated`; bottom.3mf to /tmp only (unchanged check). Probes on the solids:
socket / peg diameters, depths, positions in the module frame, notch / tab gone, peg clearance
to the link stop block. No document change.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/export_exo364.py --timeout 600"""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
exec(open('/Users/esteban/repos/exo/tools/bridge_scripts/key_measure_exo364.py').read())
P = '/Users/esteban/repos/exo/parts/'
doc = app.activeDocument
if doc.name != BuildArm.GEN_DOC_NAME: raise RuntimeError(doc.name)
design = adsk.fusion.Design.cast(app.activeProduct); root = design.rootComponent
em = design.exportManager
INS = (adsk.fusion.PointContainment.PointInsidePointContainment, adsk.fusion.PointContainment.PointOnPointContainment)
def bis(f, a, b_, n=40):
    for _ in range(n):
        m = (a + b_) / 2
        a, b_ = (m, b_) if f(m) else (a, m)
    return (a + b_) / 2
def export(body, path):
    o = em.createC3MFExportOptions(body, path); o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    return em.execute(o)
# ---- rotor top ----
occ = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name == BuildArm.MODULE_ROTOR_NAME][0]
ok, lines = BuildArm.measure_rotor_v3(occ)
say('module probes %s (%d)' % ('ALL OK' if ok else 'WRONG', len(BuildArm.V3_PROBES)))
up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
tb = up.bRepBodies.item(0)
S = lambda x, y, z: tb.pointContainment(adsk.core.Point3D.create(x/10, y/10, z/10)) in INS
key_report(S, say)
for o, path in ((up, P + 'top.3mf'), (lo, '/tmp/exo364/bottom_check.3mf')):
    b = o.component.bRepBodies.item(0)
    say('%s V %.3f -> %s %s' % (o.component.name, b.volume*1000, export(b, path), path))
# ---- links ----
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
    say('%s: bodies %d V %.3f faces %d; %s frame O %s Z %s' % (link, lo_.component.bRepBodies.count, lb.volume*1000, lb.faces.count, J, ['%.4f' % v for v in O], ['%.6f' % v for v in Z]))
    for ang in (45.0, 135.0, 225.0):
        cx, cy = 15*math.cos(math.radians(ang)), 15*math.sin(math.radians(ang))
        rs = [bis(lambda r: L(cx + r*math.cos(math.radians(22.5 + 45*k)), cy + r*math.sin(math.radians(22.5 + 45*k)), -0.6), 0.0, 2.5) for k in range(8)]
        tip = bis(lambda z: L(cx, cy, z), -0.01, -3.0)
        # centre: midpoints of x and y chords
        xa = bis(lambda t: L(cx + t, cy, -0.6), 0, -2.5); xb = bis(lambda t: L(cx + t, cy, -0.6), 0, 2.5)
        ya = bis(lambda t: L(cx, cy + t, -0.6), 0, -2.5); yb = bis(lambda t: L(cx, cy + t, -0.6), 0, 2.5)
        ccx, ccy = cx + (xa + xb)/2, cy + (ya + yb)/2
        # clearance to the nearest other solid (the stop block) at z -0.6 and -3.0 (stop block only)
        def free(rho, z):
            return not any(L(ccx + rho*math.cos(2*math.pi*k/360), ccy + rho*math.sin(2*math.pi*k/360), z) for k in range(360))
        rho = 1.55
        while rho < 12 and free(rho, -0.6): rho += 0.05
        say('  peg %3.0f: D %.4f (r %.4f..%.4f) tip z %.4f centre (%.4f, %.4f) = r %.4f @ %.3f deg; nearest other solid at z -0.6 within %.2f of the centre (edge clearance ~%.2f)'
            % (ang, 2*sum(rs)/len(rs), min(rs), max(rs), tip, ccx, ccy, math.hypot(ccx, ccy), math.degrees(math.atan2(ccy, ccx)) % 360, rho, rho - 1.5))
    say('  315 (empty) solid at z -0.6: %s; old tab spot (0, 16.3, -0.6) solid: %s; (0,16.3,-0.05): %s' % (L(15*math.cos(math.radians(315)), 15*math.sin(math.radians(315)), -0.6), L(0, 16.3, -0.6), L(0, 16.3, -0.05)))
    say('  anything below z 0 inside r 19.5 except the pegs (grid): %s' % [ (round(r,1), a) for r in (12, 13, 14, 15, 16, 17, 18, 19, 19.5) for a in range(0, 360, 5)
         if L(r*math.cos(math.radians(a)), r*math.sin(math.radians(a)), -0.6) and min(math.hypot(r*math.cos(math.radians(a)) - 15*math.cos(math.radians(k)), r*math.sin(math.radians(a)) - 15*math.sin(math.radians(k))) for k in (45, 135, 225)) > 1.55][:10])
    path = P + link[2:] + '.3mf'
    say('  export -> %s %s' % (export(lb, path), path))
d = app.activeDocument
say('isModified %s' % d.isModified)
