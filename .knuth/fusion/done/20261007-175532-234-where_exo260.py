d = app.activeDocument
df = d.dataFile
log('active %r; project %r; folder %r; version %s' % (d.name, df.parentProject.name, df.parentFolder.name, df.versionNumber))
