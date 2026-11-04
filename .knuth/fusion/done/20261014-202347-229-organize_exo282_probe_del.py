"""EXO-282: probe the delete-me doc and retry deleteMe once."""
import sys, importlib, time
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import organize_exo282_common as C; importlib.reload(C)
s = C.load(); tp = C.target(app); root = tp.rootFolder
m = [f for f in C.items(root.dataFiles) if C.short(f.id) == 'p-BdRV']
df = m[0]
assert df.parentProject.id == s['project_id']
for a in ('name', 'isInUse', 'inUseBy', 'isReadOnly', 'hasChildReferences', 'hasParentReferences', 'hasOutofDateChildReferences', 'versionNumber', 'fileExtension'):
    v = C.get(lambda a=a: getattr(df, a))
    log('%s = %s' % (a, [u.displayName for u in v] if hasattr(v, 'count') and not isinstance(v, str) else v))
time.sleep(3)
r = C.get(lambda: df.deleteMe())
log('deleteMe -> %s' % r)
s['deletes']['arm complete (1-body test upload, delete me)']['retry2'] = r
C.save(s)
