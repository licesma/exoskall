"""EXO-282: READ-ONLY. Are the delete targets still in root?"""
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import organize_exo282_common as C; importlib.reload(C)
tp = C.target(app); root = tp.rootFolder
fs = C.items(root.dataFiles)
log('root count %d' % len(fs))
for df in fs:
    if C.short(df.id) in ('szvVTe', 'p-BdRV'):
        log('present: %s %s isComplete=%s v%s' % (df.name, C.short(df.id), C.get(lambda: df.isComplete), C.get(lambda: df.latestVersionNumber)))
