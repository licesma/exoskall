"""EXO-364: re-make `sensor module 2 rotor` in the SAVED document `exo generated` with the rotor key
(notch filled, three sockets) - `BuildArm.ensure_v3_module` rejects the old one on `V3_PROBES` and
re-makes it - check against the dry run, confirm the LINKED original is untouched, SAVE (EXO-242).
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/make_rotor_exo364.py --timeout 600"""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
exec(open('/Users/esteban/repos/exo/tools/bridge_scripts/key_measure_exo364.py').read())
REC = {'upper': 5772.377, 'lower': 3865.869}          # EXO-364 dry run
ORIG = {'stator': 25729.996, 'upper': 6473.293, 'lower': 2950.793}
if app.activeDocument is None or app.activeDocument.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r' % (app.activeDocument.name if app.activeDocument else None))
design = adsk.fusion.Design.cast(app.activeProduct)
root = design.rootComponent
say(BuildArm.save_active_document(app))
comp, how = BuildArm.ensure_v3_module(design, say)
say('%s: %s' % (comp.name, how[-300:]))
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
up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
for tag, c in (('upper', up), ('lower', lo)):
    v = c.bRepBodies.item(0).volume * 1000.0
    say('  %s %-12s V=%10.3f (dry run %10.3f, %+.4f)' % (tag, c.component.name, v, REC[tag], v - REC[tag]))
    if abs(v - REC[tag]) > 1e-3: problems.append('%s %.3f' % (tag, v))
b = up.bRepBodies.item(0)
INS = (adsk.fusion.PointContainment.PointInsidePointContainment, adsk.fusion.PointContainment.PointOnPointContainment)
S = lambda x, y, z: b.pointContainment(adsk.core.Point3D.create(x/10, y/10, z/10)) in INS
key_report(S, say)
say('components named like the module: %s' % [c.name for c in design.allComponents if 'sensor module 2' in c.name or '(rejected)' in c.name])
say('root occurrences now: %s' % [root.occurrences.item(i).component.name for i in range(root.occurrences.count)])
say('PROBLEMS: %s' % problems)
say(BuildArm.save_active_document(app))
d = app.activeDocument
say('isModified %s' % d.isModified)
