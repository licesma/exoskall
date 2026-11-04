"""EXO-283 read-only: report active hub/project and the Fusion jobs/upload state."""
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import dedupe_exo283_common as C; importlib.reload(C)
g = C.get
log('active hub=%s project=%s' % (g(lambda: app.data.activeHub.name), g(lambda: app.data.activeProject.name)))
log('app.data attrs: %s' % [a for a in dir(app.data) if not a.startswith('_')])
log('isOffline=%s' % g(lambda: app.isOffLine))
