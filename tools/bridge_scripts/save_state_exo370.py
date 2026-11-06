"""EXO-370/375: final state of `exo generated` - module components, what the arms place, original
untouched, SAVE (EXO-242). Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/save_state_exo370.py"""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
design = adsk.fusion.Design.cast(app.activeProduct); root = design.rootComponent
say('document %r isModified %s' % (app.activeDocument.name, app.activeDocument.isModified))
say('module-like components: %s' % [c.name for c in design.allComponents if 'sensor module 2' in c.name or 'rejected' in c.name])
for an in ('generated_R_arm_general', 'generated_R_arm_luna'):
    a = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name == an][0]
    names = {}
    def walk(o):
        for i in range(o.childOccurrences.count):
            c = o.childOccurrences.item(i)
            if 'sensor module 2' in c.component.name: names[c.component.name] = names.get(c.component.name, 0) + 1
            else: walk(c)
    walk(a)
    say('%s places %s' % (an, names))
src = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).isReferencedComponent][0]
up0, lo0 = BuildArm.module_rotor_occs(src, in_context=True); st0 = BuildArm.module_stator_occ(src, in_context=True)
say('ORIGINAL %r stator %.3f upper %.3f lower %.3f' % (src.component.name, st0.bRepBodies.item(0).volume*1000, up0.bRepBodies.item(0).volume*1000, lo0.bRepBodies.item(0).volume*1000))
st = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name == 'stator EXO-341 export'][0]
say('stator EXO-341 export V %.4f' % (st.component.bRepBodies.item(0).volume*1000))
say(BuildArm.save_active_document(app))
d, df, saved_ever, up_to_date = BuildArm.document_state(app)
say('RESULT %r v%s saved_ever=%s up_to_date=%s isModified=%s' % (d.name, df.versionNumber if df else None, saved_ever, up_to_date, d.isModified))
