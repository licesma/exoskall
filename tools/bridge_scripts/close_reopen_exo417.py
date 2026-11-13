"""EXO-417: after the variant exports, close `exo generated` WITHOUT saving (the variant build is
discarded) and reopen its saved version (v33, margin 0.2, no label); report version, isModified
and the link volumes. Refuses if the active document is another one.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/close_reopen_exo417.py --timeout 600"""
EXO_GEN_ID = 'urn:adsk.wipprod:dm.lineage:I6f3yGrzR8S8xg8G0BP0aw'
doc = app.activeDocument
if doc.name != 'exo generated' or doc.dataFile.id != EXO_GEN_ID: raise RuntimeError(doc.name)
log('before: %r v%s isModified %s' % (doc.name, doc.dataFile.versionNumber, doc.isModified))
log('close without saving: %s' % doc.close(False))
df = app.data.findFileById(EXO_GEN_ID)
log('data file %r latest v%s' % (df.name, df.versionNumber))
doc = app.documents.open(df, True)
des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
log('after: %r v%s isModified %s' % (doc.name, doc.dataFile.versionNumber, doc.isModified))
root = des.rootComponent
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name == 'generated_R_arm_general':
        for j in range(o.childOccurrences.count):
            c = o.childOccurrences.item(j)
            if c.component.name.startswith('g_link'):
                b = c.component.bRepBodies.item(0)
                log('%s V %.4f faces %d' % (c.component.name, b.volume*1000, b.faces.count))
