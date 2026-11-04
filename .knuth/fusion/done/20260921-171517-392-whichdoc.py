log('activeDocument: %s' % (app.activeDocument.name if app.activeDocument else None))
try:
    for i in range(app.documents.count):
        log('open doc %d: %s' % (i, app.documents.item(i).name))
except Exception as e:
    log('docs: %s' % e)
try:
    dfs = app.data.activeProject.rootFolder.dataFiles
    for i in range(dfs.count):
        log('dataFile %d: %s' % (i, dfs.item(i).name))
except Exception as e:
    log('data: %s' % e)
