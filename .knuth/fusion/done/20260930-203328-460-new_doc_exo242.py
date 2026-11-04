"""EXO-242 step 1: make the generated-work document and SAVE IT FIRST.

Every build since EXO-193 went into a fresh UNSAVED document (`new_doc_general.py`), which
is how closing Fusion threw away both arms and `sensor module 2 v3`. This script does the
same job the right way round: new direct-modelling document, **saveAs before anything is
built**, then the linked `sensor module 2` inserted and saved again. `BuildArm.GEN_DOC_NAME`
is the name; `BuildArm.document_folder` picks the folder that already holds the module, so
the arms land beside the part they are made of.

Nothing of Esteban's is opened or written: the active document is left alone (a new one is
added, which becomes active) and the module goes in LINKED, as a reference.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/new_doc_exo242.py --timeout 600
"""
import importlib
import sys

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)

was = app.activeDocument
log('active document before: %r (left untouched)' % (was.name if was else None))

folder, how = BuildArm.document_folder(app)
if folder is None:
    raise RuntimeError('nowhere to save: %s' % how)
log('target folder: %s' % how)

doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
des = adsk.fusion.Design.cast(app.activeProduct)
des.designType = adsk.fusion.DesignTypes.DirectDesignType
log('new document: %r (direct modelling)' % doc.name)

# SAVE FIRST - before a single body exists, so there is never a window where work is
# only in memory.
log(BuildArm.save_active_document(app, log))

comp = BuildArm.find_module_component(des)
if comp is None:
    df = BuildArm.find_data_file(app, BuildArm.MODULE_DOC_NAME)
    if df is None:
        raise RuntimeError('no "%s" in the active project' % BuildArm.MODULE_DOC_NAME)
    occ = des.rootComponent.occurrences.addByInsert(df, adsk.core.Matrix3D.create(), True)
    log('inserted %r as %r (linked: %s)'
        % (df.name, occ.component.name, occ.isReferencedComponent))
else:
    log('module already here: %r' % comp.name)

log(BuildArm.save_active_document(app, log))

d, df, saved_ever, up_to_date = BuildArm.document_state(app)
log('RESULT document %r: saved_ever=%s up_to_date=%s project=%r folder=%r'
    % (d.name, saved_ever, up_to_date,
       df.parentProject.name if df else None, df.parentFolder.name if df else None))
log('open documents now: %d' % app.documents.count)
