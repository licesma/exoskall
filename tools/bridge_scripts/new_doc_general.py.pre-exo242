"""Build `generated_R_arm_general` into a FRESH, UNSAVED document (EXO-196).

`BuildArmGen.build_general` needs exactly two things from the document: a component called
`sensor module 2` to place beside every joint, and somewhere to put the links. It never reads
the hand-made arm (the deviation section just reports 'nothing to compare against'). So when
no design is open - and the session that held one is gone - the honest move is a NEW document
with the module inserted from the project, not somebody's saved file opened and written to.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/new_doc_general.py --timeout 900
"""
import importlib
import sys

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
import sweepgen
import BuildArm
import BuildArmGen
importlib.reload(armgen)
importlib.reload(sweepgen)
importlib.reload(BuildArm)
importlib.reload(BuildArmGen)

doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
des = adsk.fusion.Design.cast(app.activeProduct)
des.designType = adsk.fusion.DesignTypes.DirectDesignType
log('new document: %s (direct)' % doc.name)

comp = BuildArm.find_module_component(des)
if comp is None:
    df = BuildArm.find_data_file(app, BuildArm.MODULE_DOC_NAME)
    if df is None:
        raise RuntimeError('no "%s" in the active project' % BuildArm.MODULE_DOC_NAME)
    occ = des.rootComponent.occurrences.addByInsert(df, adsk.core.Matrix3D.create(), True)
    log('inserted %s as %s' % (df.name, occ.component.name))

log(BuildArmGen.build_general(app, ui, des, log))
