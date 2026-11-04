"""EXO-333 step 3: export the FINAL module rotor out of `exo generated` (`sensor module 2 rotor`):
parts/top.3mf (inner_top) and parts/bottom.3mf (inner_bottom), High refinement, each in its own
component frame (module z = file z - 7.5, as every earlier rotor file). Also: the module-internal
interference pairs of the final module vs the ORIGINAL (what the clamp and O30.1 journal add),
walls by containment, and the document's saved state. Read-only on the design: nothing is
edited, so nothing needs saving; the state is reported.
Run: python3 -I tools/fusion_run.py tools/bridge_scripts/export_rotor_exo333.py --timeout 300
"""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
doc = app.activeDocument
if doc.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r' % doc.name)
design = adsk.fusion.Design.cast(app.activeProduct)
root = design.rootComponent
occ = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name == BuildArm.MODULE_V3_NAME][0]
link = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).isReferencedComponent][0]
ok, lines = BuildArm.measure_rotor_v3(occ)
say('final probes: %s (%d)' % ('ALL OK' if ok else 'WRONG', len(BuildArm.V3_PROBES)))
up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
em = design.exportManager
for o, path in ((up, '/Users/esteban/repos/exo/parts/top.3mf'), (lo, '/Users/esteban/repos/exo/parts/bottom.3mf')):
    b = o.component.bRepBodies.item(0)
    r0, r1, z0, z1 = BuildArm._body_extent(o)
    opts = em.createC3MFExportOptions(b, path)
    opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    say('%s V %.3f module z %.4f..%.4f r %.4f..%.4f -> %s %s' % (o.component.name, b.volume * 1000, z0, z1, r0, r1, em.execute(opts), path))
INS = (adsk.fusion.PointContainment.PointInsidePointContainment, adsk.fusion.PointContainment.PointOnPointContainment)
def solid(body, r, a, z):
    x, y = BuildArm._polar(r, a)
    return body.pointContainment(adsk.core.Point3D.create(x / 10, y / 10, z / 10)) in INS
def edge_r(body, a, z, r_in, r_out):
    for _ in range(60):
        m = (r_in + r_out) / 2
        r_in, r_out = (m, r_out) if solid(body, m, a, z) else (r_in, m)
    return (r_in + r_out) / 2
def edge_z(body, r, a, z_in, z_out):
    for _ in range(60):
        m = (z_in + z_out) / 2
        z_in, z_out = (m, z_out) if solid(body, r, a, m) else (z_in, m)
    return (z_in + z_out) / 2
ub, lb = up.bRepBodies.item(0), lo.bRepBodies.item(0)
say('MEASURED (bisection on the solid, module frame):')
say('  lower journal r at z -8.0: %s' % ['%.5f' % edge_r(lb, a, -8.0, 14.0, 16.0) for a in (0, 90, 202.5)])
say('  upper journal r at z -6.0: %s' % ['%.5f' % edge_r(ub, a, -6.0, 14.0, 16.0) for a in (0, 90, 157.5)])
say('  lower joint face z (r 4, 0 deg): %.5f' % edge_z(lb, 4.0, 0.0, -7.5, -6.5))
say('  lower bore ceiling z (r 5, 0 deg): %.5f' % edge_z(lb, 5.0, 0.0, -9.5, -12.0))
say('  lower flange shoulder z (r 16.5): %.5f' % edge_z(lb, 16.5, 0.0, -12.0, -10.0))
say('  upper race shoulder z (r 16.5): %.5f' % edge_z(ub, 16.5, 0.0, -4.0, -6.0))
say('  upper joint face z (r 14, 0 deg): %.5f' % edge_z(ub, 14.0, 0.0, -6.5, -7.5))
say('  upper peg tip z (r 12.2, 22.5): %.5f' % edge_z(ub, 12.2, 22.5, -7.5, -9.0))
say('  lower socket floor z (r 12.2, 22.5): %.5f' % edge_z(lb, 12.2, 22.5, -9.0, -7.5))
# pocket: inner and outer flat along the 0 deg radial line at z -12
def first_solid(a, z, r_void, r_solid):
    for _ in range(60):
        m = (r_void + r_solid) / 2
        r_void, r_solid = (r_void, m) if solid(lb, m, a, z) else (m, r_solid)
    return (r_void + r_solid) / 2
x_in = first_solid(0.0, -12.0, 11.0, 7.5)
x_out = first_solid(0.0, -12.0, 11.0, 14.5)
say('  nut pocket at 0 deg z -12: flats r %.5f and %.5f -> across flats %.5f' % (x_in, x_out, x_out - x_in))
lj = edge_r(lb, 30.0, -10.8, 14.0, 16.0)
say('  pocket outer corner r %.4f (30 deg) -> wall to the r %.4f journal over z -11.001..-10.6: %.4f' % (math.hypot(BuildArm.V3_M3_R + BuildArm.V3_NUT_AF / 2.0, BuildArm.V3_NUT_AF / (2 * math.sqrt(3.0))), lj, lj - math.hypot(BuildArm.V3_M3_R + BuildArm.V3_NUT_AF / 2.0, BuildArm.V3_NUT_AF / (2 * math.sqrt(3.0)))))
say('  socket to journal wall: %.4f' % (edge_r(lb, 22.5, -7.6, 14.0, 16.0) - (BuildArm.V3_PEG_AT + BuildArm.V3_SOCKET_R)))
# module-internal pairs: final vs original
for tag, o in (('FINAL', occ), ('ORIGINAL', link)):
    bodies, names = BuildArm.collect_bodies(o)
    hits = BuildArm.analyze_interference(design, bodies)
    say('%s module alone: %d bodies, %d internal hit(s)' % (tag, bodies.count, len(hits)))
    agg = {}
    for a, b, vol, bb in hits:
        k = ' <-> '.join(sorted([a.split('/')[-2] if '/' in a else a, b.split('/')[-2] if '/' in b else b]))
        n, v = agg.get(k, (0, 0.0)); agg[k] = (n + 1, v + vol)
    for k in sorted(agg):
        say('    %-60s x%d  %.3f mm3' % (k, agg[k][0], agg[k][1]))
d, df, saved_ever, up_to_date = BuildArm.document_state(app)
say('DOCUMENT %r v%s saved_ever=%s up_to_date=%s isModified=%s (%s)' % (d.name, df.versionNumber if df else None, saved_ever, up_to_date, d.isModified, BuildArm.data_file_where(df)))
