"""EXO-260: open the EXO-258 organizer STEP as its own SAVED Fusion design + screenshot.

Idempotent like new_doc_exo242.py: an already-open "tool_organizer_exo258" is re-used,
otherwise the STEP is imported into a new document, saved (BuildArm.save_active_document,
into the folder that holds "sensor module 2"), fitted iso and screenshotted.
Geometry is NOT changed - it is the CadQuery export from EXO-258.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/import_organizer_exo260.py --timeout 300
"""
import importlib
import sys

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)

NAME = 'tool_organizer_exo258'
STEP = '/Users/esteban/repos/exo/gento_luna/exo/exports/tool_organizer_exo258.step'
SHOT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/tool_organizer_exo258_fusion_exo260.png'

doc = None
for i in range(app.documents.count):
    d = app.documents.item(i)
    if d.name.split(' v')[0] == NAME:
        doc = d
if doc is not None:
    log('re-using open %r' % doc.name)
    doc.activate()
else:
    im = app.importManager
    doc = im.importToNewDocument(im.createSTEPImportOptions(STEP))
    log('imported %s -> %r' % (STEP, doc.name if doc else None))
    if doc is None:
        raise RuntimeError('importToNewDocument returned None')
    doc.activate()
    log(BuildArm.save_active_document(app, name=NAME, why='EXO-260'))

des = adsk.fusion.Design.cast(app.activeProduct)
root = des.rootComponent
bodies = [b for b in root.bRepBodies] + [
    b for o in root.allOccurrences for b in o.bRepBodies]
log('bodies: %d %s' % (len(bodies), [b.name for b in bodies]))
bb = root.boundingBox
log('bbox mm: %.2f x %.2f x %.2f' % tuple(
    10 * (getattr(bb.maxPoint, a) - getattr(bb.minPoint, a)) for a in 'xyz'))

vp = app.activeViewport
cam = vp.camera
cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
cam.isFitView = True
vp.camera = cam
adsk.doEvents()
vp.fit()
adsk.doEvents()
log('screenshot -> %s' % screenshot(SHOT))
d, df, ever, up = BuildArm.document_state(app)
log('RESULT %r saved_ever=%s up_to_date=%s (%s)'
    % (d.name, ever, up, BuildArm.data_file_where(df)))
