"""EXO-283 read-only: current versionNumber of the assemblies."""
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import dedupe_exo283_common as C; importlib.reload(C)
for n, i in C.ASM.items():
    df = C.find_file(app, i); log('%s %s v%s latest v%s' % (n, C.short(i), C.get(lambda: df.versionNumber), C.get(lambda: df.latestVersionNumber)))
log('open docs: %s' % [C.get(lambda d=d: d.name) for d in C.items(app.documents)])
