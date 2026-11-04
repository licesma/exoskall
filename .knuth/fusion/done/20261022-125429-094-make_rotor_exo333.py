"""EXO-333 step 2a: make `sensor module 2 rotor` (the FINAL module rotor, BuildArm.ensure_v3_module)
in the SAVED document `exo generated`, check it against the dry-run numbers, confirm the LINKED
original is untouched, and SAVE (EXO-242 rule: save before and after).

Run: python3 -I tools/fusion_run.py tools/bridge_scripts/make_rotor_exo333.py --timeout 300
"""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
REC = {'upper': 5786.431, 'lower': 3865.869}          # EXO-333 dry run in a scratch document
ORIG = {'stator': 25729.996, 'upper': 6473.293, 'lower': 2950.793}
if app.activeDocument is None or app.activeDocument.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r, not %r' % (app.activeDocument.name if app.activeDocument else None, BuildArm.GEN_DOC_NAME))
design = adsk.fusion.Design.cast(app.activeProduct)
root = design.rootComponent
say(BuildArm.save_active_document(app))
comp, how = BuildArm.ensure_v3_module(design, say)
say('%s: %s' % (comp.name, how))
occ = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name == comp.name][0]
problems = []
src = [root.occurrences.item(i) for i in range(root.occurrences.count)
       if root.occurrences.item(i).isReferencedComponent and root.occurrences.item(i).component.name.startswith(BuildArm.MODULE_DOC_NAME)]
if not src:
    problems.append('no LINKED original')
else:
    s = src[0]
    try:
        say('link -> %r id %s' % (s.documentReference.dataFile.name, s.documentReference.dataFile.id))
    except Exception as e:
        say('link reference unreadable: %s' % e)
    up0, lo0 = BuildArm.module_rotor_occs(s, in_context=True)
    st0 = BuildArm.module_stator_occ(s, in_context=True)
    for what, c in (('stator', st0), ('upper', up0), ('lower', lo0)):
        v = c.bRepBodies.item(0).volume * 1000.0
        say('  ORIGINAL %-7s V=%10.3f (record %10.3f)' % (what, v, ORIG[what]))
        if abs(v - ORIG[what]) > 1e-3:
            problems.append('ORIGINAL %s %.3f' % (what, v))
up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
for tag, c in (('upper', up), ('lower', lo)):
    v = c.bRepBodies.item(0).volume * 1000.0
    r0, r1, z0, z1 = BuildArm._body_extent(c)
    say('  %s %-12s V=%10.3f (dry run %10.3f, %+.4f) r %.3f..%.3f z %.3f..%.3f' % (tag, c.component.name, v, REC[tag], v - REC[tag], r0, r1, z0, z1))
    if abs(v - REC[tag]) > 1e-3:
        problems.append('%s %.3f vs %.3f' % (tag, v, REC[tag]))
say('root occurrences now: %s' % [root.occurrences.item(i).component.name for i in range(root.occurrences.count)])
say(BuildArm.save_active_document(app))
d, df, saved_ever, up_to_date = BuildArm.document_state(app)
say('RESULT document %r v%s saved_ever=%s up_to_date=%s isModified=%s' % (d.name, df.versionNumber if df else None, saved_ever, up_to_date, d.isModified))
if problems:
    raise RuntimeError('EXO-333 step 2a: ' + '; '.join(problems))
