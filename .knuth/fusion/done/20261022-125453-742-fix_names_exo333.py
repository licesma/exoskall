"""EXO-333 step 2a fix: legacy `sensor module 2 v3` parts -> "(legacy v3)", then the printed parts
of `sensor module 2 rotor` get their exact names; ensure_v3_module must then ACCEPT the module
as it is (no re-make). Saves."""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
if app.activeDocument.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r' % app.activeDocument.name)
design = adsk.fusion.Design.cast(app.activeProduct)
root = design.rootComponent
BuildArm._retire_legacy_part_names(design, say)
occ = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name == BuildArm.MODULE_V3_NAME][0]
say(BuildArm.rename_v3_parts(occ))
comp, how = BuildArm.ensure_v3_module(design, None)
say('ensure_v3_module: %s' % how)
if not how.startswith('already in the document'):
    raise RuntimeError('the module was re-made, not accepted')
say(BuildArm.save_active_document(app))
say('isModified=%s' % app.activeDocument.isModified)
