import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)
log('hub %s / project %s' % (app.data.activeHub.name, app.data.activeProject.name if app.data.activeProject else None))
log('open docs: %s' % [app.documents.item(i).name for i in range(app.documents.count)])
for n in ('sensor module 2', 'sensor module 2 v4', 'exo generated'):
    df = BuildArm.find_data_file(app, n)
    log('%r -> %s' % (n, None if df is None else '%s v%s id %s folder %s' % (df.name, df.versionNumber, df.id, df.parentFolder.name)))
log('V3_NUT_AF %s' % BuildArm.V3_NUT_AF)
