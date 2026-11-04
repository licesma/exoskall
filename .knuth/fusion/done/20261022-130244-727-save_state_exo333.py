"""EXO-333: save `exo generated` (EXO-242 rule) after the arm checks, and report its state: the
final module accepted as is, the LINKED original untouched, which module every arm places, and
what is left of the legacy `sensor module 2 v3`."""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
doc = app.activeDocument
if doc.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r' % doc.name)
design = adsk.fusion.Design.cast(app.activeProduct)
root = design.rootComponent
say('isModified before: %s' % doc.isModified)
comp, how = BuildArm.ensure_v3_module(design, None)
say('module: %s' % how)
if not how.startswith('already in the document'):
    raise RuntimeError('the module was re-made, not accepted')
link = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).isReferencedComponent][0]
up0, lo0 = BuildArm.module_rotor_occs(link, in_context=True)
st0 = BuildArm.module_stator_occ(link, in_context=True)
say('ORIGINAL (linked %r): stator %.3f upper %.3f lower %.3f' % (link.documentReference.dataFile.name,
    st0.bRepBodies.item(0).volume * 1000, up0.bRepBodies.item(0).volume * 1000, lo0.bRepBodies.item(0).volume * 1000))
mods = {}
for i in range(root.occurrences.count):
    a = root.occurrences.item(i)
    if not a.component.name.startswith('generated_R_arm'):
        continue
    for j in range(a.childOccurrences.count):
        w = a.childOccurrences.item(j)
        for k in range(w.childOccurrences.count):
            n = w.childOccurrences.item(k).component.name
            if n.startswith(BuildArm.MODULE_DOC_NAME):
                mods.setdefault(a.component.name, []).append(n)
for a, ns in mods.items():
    say('%s places %d modules: %s' % (a, len(ns), sorted(set(ns))))
say('root occurrences: %s' % [root.occurrences.item(i).component.name for i in range(root.occurrences.count)])
say('module-like components left: %s' % sorted(set(c.name for c in design.allComponents if c.name.startswith('sensor module') or '(legacy v3)' in c.name)))
say(BuildArm.save_active_document(app))
d, df, saved_ever, up_to_date = BuildArm.document_state(app)
say('DOCUMENT %r v%s saved_ever=%s up_to_date=%s isModified=%s (%s)' % (d.name, df.versionNumber, saved_ever, up_to_date, d.isModified, BuildArm.data_file_where(df)))
