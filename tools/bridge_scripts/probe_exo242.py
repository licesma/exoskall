"""EXO-242: find the project/folder that holds `sensor module 2`, and list open docs."""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)

log('active document: %r' % (app.activeDocument.name if app.activeDocument else None))
log('open documents: %d' % app.documents.count)
for i in range(app.documents.count):
    d = app.documents.item(i)
    try:
        saved = d.isSaved
    except Exception as e:
        saved = 'err %s' % e
    try:
        dfn = d.dataFile.name if d.dataFile else None
    except Exception as e:
        dfn = 'err %s' % e
    log('  [%d] name=%r isSaved=%s dataFile=%r' % (i, d.name, saved, dfn))

proj = app.data.activeProject
log('activeProject: %r' % (proj.name if proj else None))
hub = app.data.activeHub
log('activeHub: %r' % (hub.name if hub else None))

df = BuildArm.find_data_file(app, BuildArm.MODULE_DOC_NAME)
log('find_data_file(%r) -> %r' % (BuildArm.MODULE_DOC_NAME, df.name if df else None))
if df:
    log('  parentFolder: %r' % df.parentFolder.name)
    log('  parentProject: %r' % df.parentProject.name)
    log('  id: %s' % df.id)

# list all data files in the active project, top level
if proj:
    rf = proj.rootFolder
    log('rootFolder %r: %d files, %d folders' % (rf.name, rf.dataFiles.count, rf.dataFolders.count))
    for i in range(rf.dataFiles.count):
        log('   file: %r' % rf.dataFiles.item(i).name)
    for i in range(rf.dataFolders.count):
        log('   folder: %r' % rf.dataFolders.item(i).name)
