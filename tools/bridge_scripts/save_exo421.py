"""EXO-421: save `exo generated` (EXO-242 pipeline save) after the G1 rebuild and the bracket
both passed. Refuses any other active document.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/save_exo421.py --timeout 600
"""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)
doc = app.activeDocument
if doc.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r, not %r' % (doc.name, BuildArm.GEN_DOC_NAME))
log('before: v%s isModified %s' % (doc.dataFile.versionNumber, doc.isModified))
log(BuildArm.save_active_document(app, why='EXO-421 joint numbers on both ends of every link'))
log('after: v%s isModified %s' % (app.activeDocument.dataFile.versionNumber, app.activeDocument.isModified))
