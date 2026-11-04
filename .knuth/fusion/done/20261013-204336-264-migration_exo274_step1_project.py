"""EXO-274 step 1: create project "G1 exeskeleton" in hub Estebans (+ /cuff). Idempotent via state.json.
Touches only the TARGET hub. Restores app.data.activeHub/activeProject in finally."""
import json, os, time
ST = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274/state.json'
state = json.load(open(ST))
TARGET_HUB_ID = 'a.YnVzaW5lc3M6Z21haWw2MDk1MjQ0'
NAME = 'G1 exeskeleton'
def save():
    json.dump(state, open(ST + '.tmp', 'w'), indent=1, default=str); os.replace(ST + '.tmp', ST)
start_hub = app.data.activeHub
start_proj = app.data.activeProject
try:
    hub = None
    for i in range(app.data.dataHubs.count):
        if app.data.dataHubs.item(i).id == TARGET_HUB_ID:
            hub = app.data.dataHubs.item(i)
    assert hub is not None, 'target hub not found'
    app.data.activeHub = hub
    log('activeHub now %s' % app.data.activeHub.name)
    t = state.get('target', {})
    proj = None
    if t.get('project_id'):
        proj = hub.dataProjects.itemById(t['project_id'])
        log('project from state: %s' % (proj.name if proj else None))
    if proj is None:
        same = [hub.dataProjects.item(i) for i in range(hub.dataProjects.count) if hub.dataProjects.item(i).name == NAME]
        if same:
            p = same[0]
            n = p.rootFolder.dataFiles.count + p.rootFolder.dataFolders.count
            log('project %r already exists in target, root items=%d' % (NAME, n))
            if n > 0:
                state['step1'] = 'STOP: existing non-empty project'
                save()
                raise RuntimeError('target project exists and is non-empty; stopping')
            proj = p
        else:
            assert hasattr(hub.dataProjects, 'add'), 'DataProjects.add not available'
            proj = hub.dataProjects.add(NAME)
            log('created project %s id=%s' % (proj.name, proj.id))
    t['hub_id'] = hub.id; t['hub_name'] = hub.name
    t['project_id'] = proj.id; t['project_name'] = proj.name
    t['root_folder_id'] = proj.rootFolder.id
    cuff = proj.rootFolder.dataFolders.itemByName('cuff')
    if cuff is None:
        cuff = proj.rootFolder.dataFolders.add('cuff')
        log('created folder cuff id=%s' % cuff.id)
    else:
        log('folder cuff exists id=%s' % cuff.id)
    t['cuff_folder_id'] = cuff.id
    state['target'] = t
    state['step1'] = 'done %s' % time.strftime('%Y-%m-%d %H:%M:%S')
    save()
    log('projects in target hub now: %s' % [hub.dataProjects.item(i).name for i in range(hub.dataProjects.count)])
finally:
    app.data.activeHub = start_hub
    try:
        app.data.activeProject = start_proj
    except Exception as e:
        log('restore project err %s' % e)
    log('restored activeHub=%s activeProject=%s' % (app.data.activeHub.name, app.data.activeProject.name))
