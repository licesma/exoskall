"""EXO-274 step 1 rerun (idempotent, no hub switch): find/confirm target project G1 exeskeleton in hub Estebans,
must be empty except folder cuff; create cuff only if missing. Records ids in state.json."""
import json, os, time
ST = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274/state.json'
state = json.load(open(ST))
hub = [app.data.dataHubs.item(i) for i in range(app.data.dataHubs.count) if app.data.dataHubs.item(i).id == state['target']['hub_id']][0]
projs = [hub.dataProjects.item(i) for i in range(hub.dataProjects.count) if hub.dataProjects.item(i).name == 'G1 exeskeleton']
log('projects named G1 exeskeleton in %s: %d' % (hub.name, len(projs)))
assert len(projs) == 1
p = projs[0]
rf = p.rootFolder
files = [rf.dataFiles.item(i).name for i in range(rf.dataFiles.count)]
folders = [rf.dataFolders.item(i).name for i in range(rf.dataFolders.count)]
log('root files=%s folders=%s' % (files, folders))
if state.get('step1', '').startswith('done'):
    log('step1 already done; not re-checking emptiness')
else:
    assert not files and set(folders) <= {'cuff'}, 'target not empty: STOP'
cuff = rf.dataFolders.itemByName('cuff') or rf.dataFolders.add('cuff')
state['target'].update({'project_id': p.id, 'root_folder_id': rf.id, 'cuff_folder_id': cuff.id})
state['step1'] = 'done %s (project+cuff were created by the hung 2026-10-13 run; confirmed empty)' % time.strftime('%Y-%m-%d %H:%M:%S')
json.dump(state, open(ST + '.tmp', 'w'), indent=1, default=str); os.replace(ST + '.tmp', ST)
log('active hub/project: %s / %s' % (app.data.activeHub.name, app.data.activeProject.name))
log(state['target'])
