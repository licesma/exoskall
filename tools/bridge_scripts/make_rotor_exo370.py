"""EXO-370: re-make `sensor module 2 rotor` in the SAVED document `exo generated` with the final
bottom (joint face -8.0, boss r 7.8 to -7.0, sockets floor -9.3) and NO upper part (fused into
every link) - `BuildArm.ensure_v3_module` rejects the old copy on `V3_PROBES` and re-makes it -
confirm the LINKED original is untouched, SAVE (EXO-242).
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/make_rotor_exo370.py --timeout 600"""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
ORIG = {'stator': 25729.996, 'upper': 6473.293, 'lower': 2950.793}
if app.activeDocument is None or app.activeDocument.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r' % (app.activeDocument.name if app.activeDocument else None))
design = adsk.fusion.Design.cast(app.activeProduct)
root = design.rootComponent
say(BuildArm.save_active_document(app))
comp, how = BuildArm.ensure_v3_module(design, say)
say('%s: %s' % (comp.name, how[-600:]))
occ = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name == comp.name][0]
problems = []
src = [root.occurrences.item(i) for i in range(root.occurrences.count)
       if root.occurrences.item(i).isReferencedComponent and root.occurrences.item(i).component.name.startswith(BuildArm.MODULE_DOC_NAME)][0]
up0, lo0 = BuildArm.module_rotor_occs(src, in_context=True)
st0 = BuildArm.module_stator_occ(src, in_context=True)
for what, c in (('stator', st0), ('upper', up0), ('lower', lo0)):
    v = c.bRepBodies.item(0).volume * 1000.0
    say('  ORIGINAL %-7s V=%10.3f (record %10.3f)' % (what, v, ORIG[what]))
    if abs(v - ORIG[what]) > 1e-3: problems.append('ORIGINAL %s' % what)
ok, lines = BuildArm.measure_rotor_v3(occ)
say('probes %s: %d' % ('ALL OK' if ok else 'WRONG', len(lines)))
for l in lines:
    if not l.endswith('OK'): say(l)
up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
say('upper in module: %s; lower %s V=%.3f' % (up, lo.component.name, lo.bRepBodies.item(0).volume * 1000.0))
nut, bolt = BuildArm.module_v3_fasteners(occ)
bb = bolt.component.bRepBodies.item(0).boundingBox
say('bolts %s: z %.3f..%.3f; nuts %s' % (bolt.component.name, bb.minPoint.z*10, bb.maxPoint.z*10, nut.component.name))
say('components named like the module: %s' % [c.name for c in design.allComponents if 'sensor module 2' in c.name or '(rejected)' in c.name])
say('PROBLEMS: %s' % problems)
say(BuildArm.save_active_document(app))
say('isModified %s' % app.activeDocument.isModified)
