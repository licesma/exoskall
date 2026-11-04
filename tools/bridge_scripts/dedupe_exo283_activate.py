"""EXO-283: make Estebans/G1 exeskeleton the active project (UI state only), then retry save of the open
relinked pGmH doc and report. Records the previous active hub/project in state.json."""
import sys, importlib, time
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import dedupe_exo283_common as C; importlib.reload(C)
g = C.get; s = C.load()
s.setdefault('active_start', {'hub': g(lambda: app.data.activeHub.name), 'project': g(lambda: app.data.activeProject.name),
                              'project_id': g(lambda: app.data.activeProject.id)})
hub = [h for h in C.items(app.data.dataHubs) if h.name == 'Estebans'][0]
tp = [p for p in C.items(hub.dataProjects) if p.id == C.PROJECT_ID][0]
app.data.activeHub = hub
app.data.activeProject = tp
log('active now %s / %s' % (g(lambda: app.data.activeHub.name), g(lambda: app.data.activeProject.name)))
d = [x for x in C.items(app.documents) if g(lambda x=x: x.dataFile.id) == C.ASM['sensor module pGmH']][0]
d.activate()
log('save -> %s; isModified=%s' % (d.save('EXO-283 relink shared parts to parts/'), g(lambda: d.isModified)))
s['active_after_switch'] = g(lambda: app.data.activeProject.name) + ' @ ' + g(lambda: app.data.activeHub.name)
C.save(s)
