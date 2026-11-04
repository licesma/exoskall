"""EXO-283 read-only: current versionNumber of the assemblies."""
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import dedupe_exo283_common as C; importlib.reload(C)
for n, i in C.ASM.items():
    log('%s %s v%s' % (n, C.short(i), C.get(lambda i=i: C.find_file(app, i).versionNumber)))
log('open docs: %s' % [C.get(lambda d=d: d.name) for d in C.items(app.documents)])
