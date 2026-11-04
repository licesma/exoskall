"""EXO-341: save `exo generated` after the stator export copy was added (retry of the save that hit a transient data-API error)."""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
assert app.activeDocument.name == BuildArm.GEN_DOC_NAME
log(BuildArm.save_active_document(app))
d = app.activeDocument
log('isModified %s v%s' % (d.isModified, d.dataFile.versionNumber))
