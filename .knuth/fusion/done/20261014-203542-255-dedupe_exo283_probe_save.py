"""EXO-283 probe: re-save the open, already-relinked pGmH doc (only that doc) and report state."""
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import dedupe_exo283_common as C; importlib.reload(C)
g = C.get
d = [x for x in C.items(app.documents) if g(lambda x=x: x.dataFile.id) == C.ASM['sensor module pGmH']][0]
log('before: isModified=%s' % g(lambda: d.isModified))
log('saveDataVersion doc: %s' % (type(d).saveDataVersion.__doc__ or '')[:600])
log('save doc: %s' % (type(d).save.__doc__ or '')[:600])
try:
    r = d.save('EXO-283 relink shared parts to parts/')
    log('save -> %s' % r)
except Exception as e:
    log('save raised: %s' % e)
log('after: isModified=%s isSaved=%s' % (g(lambda: d.isModified), g(lambda: d.isSaved)))
