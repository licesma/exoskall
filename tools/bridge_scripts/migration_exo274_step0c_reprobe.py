"""EXO-274 step 0c: READ-ONLY reprobe after the step1 hang. No hub switch."""
import json, os, time
ST = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274/state.json'
state = json.load(open(ST))
def get(fn):
    try:
        return fn()
    except Exception as e:
        return 'ERR: %s' % (str(e).splitlines()[0] if str(e) else type(e).__name__)
r = {'time': time.strftime('%Y-%m-%d %H:%M:%S'),
     'active_hub': get(lambda: app.data.activeHub.name), 'active_project': get(lambda: app.data.activeProject.name),
     'active_doc': get(lambda: app.activeDocument.name)}
for i in range(app.data.dataHubs.count):
    h = app.data.dataHubs.item(i)
    if h.name == 'Estebans':
        ps = [h.dataProjects.item(j) for j in range(h.dataProjects.count)]
        r['estebans_projects'] = [p.name for p in ps]
        for p in ps:
            if p.name == 'G1 exeskeleton':
                rf = p.rootFolder
                r['target'] = {'id': p.id, 'root_files': [rf.dataFiles.item(k).name for k in range(rf.dataFiles.count)],
                               'root_folders': [rf.dataFolders.item(k).name for k in range(rf.dataFolders.count)]}
r['open_docs'] = [{'name': get(lambda d=d: d.name), 'isModified': get(lambda d=d: d.isModified),
                   'df_id': get(lambda d=d: d.dataFile.id if d.dataFile else None),
                   'df_ver': get(lambda d=d: d.dataFile.versionNumber if d.dataFile else None)}
                  for d in [app.documents.item(i) for i in range(app.documents.count)]]
log(json.dumps(r, indent=1))
state['reprobe'] = r
json.dump(state, open(ST + '.tmp', 'w'), indent=1, default=str); os.replace(ST + '.tmp', ST)
