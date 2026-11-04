"""EXO-242 step 1: the generated-work document, SAVED FIRST - and this script is IDEMPOTENT.

Every build since EXO-193 went into a fresh UNSAVED document (`new_doc_general.py`), which
is how closing Fusion threw away both arms and `sensor module 2 v3`. This does the same job
the right way round: `BuildArm.GEN_DOC_NAME` is saved into the folder that already holds
`sensor module 2` (`BuildArm.document_folder`) BEFORE anything is built, then the linked
module is inserted and it is saved again.

Run it twice and nothing is duplicated: an already-open `exo generated` is activated and
re-used, a brand-new unsaved document is saved AS that name, and the module is only
inserted when the document does not hold one.

Nothing of Esteban's is opened or written - the module goes in LINKED, as a reference.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/new_doc_exo242.py --timeout 600
"""
import importlib
import sys

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)

WANT = BuildArm.GEN_DOC_NAME


def df_name(d):
    try:
        return d.dataFile.name if d.dataFile else None
    except Exception:
        return None


log('open documents: %d' % app.documents.count)
mine = []
for i in range(app.documents.count):
    d = app.documents.item(i)
    log('  [%d] name=%r dataFile=%r' % (i, d.name, df_name(d)))
    if d.name == WANT or df_name(d) == WANT:
        mine.append(d)
log('active document: %r' % (app.activeDocument.name if app.activeDocument else None))

if mine:
    doc = mine[0]
    log('re-using the %r already open (%d match)' % (doc.name, len(mine)))
    if app.activeDocument is not doc:
        doc.activate()
        log('activated %r' % doc.name)
else:
    folder, how = BuildArm.document_folder(app)
    if folder is None:
        raise RuntimeError('nowhere to save: %s' % how)
    log('target folder: %s' % how)
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    log('new document: %r' % doc.name)

des = adsk.fusion.Design.cast(app.activeProduct)
if des.designType != adsk.fusion.DesignTypes.DirectDesignType:
    des.designType = adsk.fusion.DesignTypes.DirectDesignType
    log('design type -> direct modelling')

# SAVE FIRST - before a single body exists, so there is never a window where work is
# only in memory.
log(BuildArm.save_active_document(app))

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

log(BuildArm.save_active_document(app))

d, df, saved_ever, up_to_date = BuildArm.document_state(app)
log('RESULT document %r: saved_ever=%s up_to_date=%s (%s)'
    % (d.name, saved_ever, up_to_date, BuildArm.data_file_where(df)))
log('root occurrences: %s'
    % [des.rootComponent.occurrences.item(i).component.name
       for i in range(des.rootComponent.occurrences.count)])
log('open documents now: %d' % app.documents.count)
