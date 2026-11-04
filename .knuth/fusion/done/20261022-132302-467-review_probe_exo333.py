"""EXO-333 Tier 3 review probe. READ-ONLY: no edit, no save, no open, no activate.
Reports the `exo generated` document state, the module copy `sensor module 2 rotor` (rotor
volumes/bboxes, every child body incl. the bearing and the horn screws), the linked
original's body volumes, and which module each arm places."""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)

def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
def vol(b):
    try:
        return '%10.3f' % (b.volume * 1000)
    except Exception:
        return '  (no vol)'
def bb_mm(b):
    bb = b.boundingBox
    return ('x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'
            % (bb.minPoint.x * 10, bb.maxPoint.x * 10, bb.minPoint.y * 10, bb.maxPoint.y * 10,
               bb.minPoint.z * 10, bb.maxPoint.z * 10))

say('open documents: %s' % [(app.documents.item(i).name, app.documents.item(i).isModified)
                             for i in range(app.documents.count)])
doc = app.activeDocument
say('active document %r isModified=%s isSaved=%s' % (doc.name, doc.isModified, doc.isSaved))
if doc.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r, not %r - nothing touched' % (doc.name, BuildArm.GEN_DOC_NAME))
d, df, saved_ever, up_to_date = BuildArm.document_state(app)
say('data file v%s saved_ever=%s up_to_date=%s (%s)' % (df.versionNumber if df else None, saved_ever, up_to_date, BuildArm.data_file_where(df)))
design = adsk.fusion.Design.cast(app.activeProduct)
root = design.rootComponent
say('design type %s' % design.designType)
say('module-like components: %s' % sorted(c.name for c in design.allComponents
    if c.name.startswith('sensor module') or '(legacy' in c.name))
say('root occurrences:')
mod = None; link = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    t = o.transform.translation
    say('  %-40s ref=%s bulb=%s t=(%.3f,%.3f,%.3f)' % (o.component.name, o.isReferencedComponent, o.isLightBulbOn, t.x * 10, t.y * 10, t.z * 10))
    if o.component.name == BuildArm.MODULE_ROTOR_NAME and not o.isReferencedComponent:
        mod = o
    if o.isReferencedComponent and o.component.name.startswith(BuildArm.MODULE_DOC_NAME):
        link = o
say('MODULE_ROTOR_NAME = %r' % BuildArm.MODULE_ROTOR_NAME)
if mod is None:
    raise RuntimeError('no local root occurrence named %r' % BuildArm.MODULE_ROTOR_NAME)
up, lo = BuildArm.module_rotor_occs(mod, in_context=True)
st = BuildArm.module_stator_occ(mod, in_context=True)
for what, o in (('upper', up), ('lower', lo), ('stator', st)):
    b = o.bRepBodies.item(0)
    say('  rotor %-6s comp %r body %r V %.3f mm3 %s' % (what, o.component.name, b.name, b.volume * 1000, bb_mm(b)))
say('  module children (in context):')
for j in range(mod.childOccurrences.count):
    c = mod.childOccurrences.item(j)
    for k in range(c.bRepBodies.count):
        b = c.bRepBodies.item(k)
        say('    %-36s %-8s V %s %s' % (c.component.name[:36], b.name, vol(b), bb_mm(b)))
    for jj in range(c.childOccurrences.count):
        cc = c.childOccurrences.item(jj)
        for k in range(cc.bRepBodies.count):
            b = cc.bRepBodies.item(k)
            say('      %-34s %-8s V %s %s' % (cc.component.name[:34], b.name, vol(b), bb_mm(b)))
good, lines = BuildArm.measure_rotor_v3(mod)
say('  V3_PROBES: %d, all good=%s, wrong=%d' % (len(BuildArm.V3_PROBES), good, sum(1 for l in lines if l.endswith('WRONG'))))
for l in lines:
    if l.endswith('WRONG'):
        say('    ' + l)
if link is not None:
    up0, lo0 = BuildArm.module_rotor_occs(link, in_context=True)
    st0 = BuildArm.module_stator_occ(link, in_context=True)
    say('ORIGINAL linked %r (-> %r v%s): stator %.3f upper %.3f lower %.3f' % (
        link.component.name, link.documentReference.dataFile.name,
        link.documentReference.version,
        st0.bRepBodies.item(0).volume * 1000, up0.bRepBodies.item(0).volume * 1000, lo0.bRepBodies.item(0).volume * 1000))
else:
    say('ORIGINAL: no linked occurrence at the root')
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
say('END isModified=%s (nothing was changed or saved by this probe)' % doc.isModified)
