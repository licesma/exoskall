d = app.activeDocument
log('closing %r modified=%s dataFile=%s' % (d.name, d.isModified, d.dataFile))
if d.dataFile is not None: raise RuntimeError('refusing: active doc is saved')
d.close(False)
log('open docs: %s' % [app.documents.item(i).name for i in range(app.documents.count)])
