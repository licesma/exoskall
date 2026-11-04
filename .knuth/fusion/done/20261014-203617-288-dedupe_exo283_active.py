"""EXO-283 read-only: report active hub/project and the Fusion jobs/upload state."""
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import dedupe_exo283_common as C; importlib.reload(C)
g = C.get
log('active hub=%s project=%s' % (g(lambda: app.data.activeHub.name), g(lambda: app.data.activeProject.name)))
log('app.data attrs: %s' % [a for a in dir(app.data) if not a.startswith('_')])
log('isOffline=%s' % g(lambda: app.isOffLine))
pl = g(lambda: app.data.personalUseLimits)
log('personalUseLimits=%s' % pl)
if pl is not None and not isinstance(pl, str):
    log({a: g(lambda a=a: getattr(pl, a)) for a in dir(pl) if not a.startswith('_') and a not in ('this', 'thisown', 'cast')})
d = [x for x in C.items(app.documents) if g(lambda x=x: x.dataFile.id) == C.ASM['sensor module pGmH']]
if d:
    df = d[0].dataFile
    log({a: g(lambda a=a: getattr(df, a)) for a in ('isReadOnly', 'isLocked', 'lockedBy', 'isInUse', 'inUseBy', 'versionNumber', 'latestVersionNumber')})
    log('hub attrs: %s' % {a: g(lambda a=a: getattr(df.parentProject.parentHub, a)) for a in ('name', 'hubType', 'isValid')})
