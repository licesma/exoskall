"""EXO-333: open `exo generated` (Estebans / G1 exeskeleton) and report what it holds. Read-only:
no edit, no save. If it is already open it is only activated."""
EXO_GEN_ID = 'urn:adsk.wipprod:dm.lineage:I6f3yGrzR8S8xg8G0BP0aw'
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
say('open before: %s' % [app.documents.item(i).name for i in range(app.documents.count)])
doc = None
for i in range(app.documents.count):
    d = app.documents.item(i)
    try:
        if d.dataFile is not None and d.dataFile.id == EXO_GEN_ID:
            doc = d
    except Exception:
        pass
if doc is None:
    df = app.data.findFileById(EXO_GEN_ID)
    say('data file %r v%s, folder %r, project %r' % (df.name, df.versionNumber, df.parentFolder.name, df.parentProject.name))
    doc = app.documents.open(df, True)
else:
    doc.activate()
des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
say('document %r v%s isModified=%s designType=%s' % (doc.name, doc.dataFile.versionNumber, doc.isModified, des.designType))
root = des.rootComponent
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    extra = ''
    if o.isReferencedComponent:
        try:
            r = o.documentReference
            extra = ' -> %r v%s id %s' % (r.dataFile.name, r.version.versionNumber if r.version else '?', r.dataFile.id)
        except Exception as e:
            extra = ' (reference: %s)' % e
    say('  root occ %r comp %r ref=%s visible=%s%s' % (o.name, o.component.name, o.isReferencedComponent, o.isLightBulbOn, extra))
say('components: %d' % des.allComponents.count)
names = sorted(set(c.name for c in des.allComponents if c.name.startswith('sensor module')))
say('module components: %s' % names)
