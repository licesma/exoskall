"""EXO-274 test: DataFile.copy of ONE source assembly (sensor module test) into the TARGET root folder.
Source is only read (copy creates a new file in the target). Logs the copy's references."""
import json, time
ST = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274/state.json'
state = json.load(open(ST))
def get(fn):
    try:
        return fn()
    except Exception as e:
        return 'ERR: %s' % (str(e).splitlines()[0] if str(e) else type(e).__name__)
src = app.data.findFileById('urn:adsk.wipprod:dm.lineage:5328GUuwS_SbsBlhkmhZCw')
hub = [app.data.dataHubs.item(i) for i in range(app.data.dataHubs.count) if app.data.dataHubs.item(i).id == state['target']['hub_id']][0]
proj = hub.dataProjects.itemById(state['target']['project_id'])
t = time.time()
c = get(lambda: src.copy(proj.rootFolder))
log('copy -> %s (%.1fs)' % (c if isinstance(c, str) else (c.name, c.id, get(lambda: c.parentProject.name), get(lambda: c.parentProject.parentHub.name)), time.time() - t))
if not isinstance(c, str) and c is not None:
    for i in range(c.childReferences.count):
        r = c.childReferences.item(i)
        log('  child ref: %s %s project=%s hub=%s' % (r.name, r.id, get(lambda: r.parentProject.name), get(lambda: r.parentProject.parentHub.name)))
rf = proj.rootFolder
log('target root now: %s' % [rf.dataFiles.item(i).name for i in range(rf.dataFiles.count)])
