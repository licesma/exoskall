"""EXO-282 step 3: delete empty 'cuff' folder and 2 docs (only if no parentReferences)."""
import sys, importlib, time
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import organize_exo282_common as C; importlib.reload(C)
s = C.load(); tp = C.target(app); root = tp.rootFolder
d = s.setdefault('deletes', {})
def prefs(df):
    return [(r.name, C.short(r.id), C.get(lambda r=r: r.parentFolder.name)) for r in C.items(df.parentReferences)]
old = C.path_folder(root, 'arm/old')
for df in C.items(old.dataFiles):
    if df.name == 'sensor module':
        d['sensor module ' + C.short(df.id)] = {'action': 'kept, moved to arm/old', 'parentReferences': prefs(df)}
        log('sensor module %s parents: %s' % (C.short(df.id), prefs(df)))
for df in C.items(root.dataFiles):
    if C.short(df.id) in ('szvVTe', 'p-BdRV'):
        assert df.parentProject.id == s['project_id']
        pr = prefs(df)
        log('%s %s parents=%s' % (df.name, C.short(df.id), pr))
        if pr:
            d[df.name] = {'action': 'SKIPPED, has parents', 'parentReferences': pr}; continue
        ok, errs = None, []
        for t in range(3):
            try:
                ok = df.deleteMe(); break
            except Exception as e:
                errs.append(str(e)); time.sleep(2)
        d[df.name] = {'id': df.id, 'action': 'deleted' if ok else 'delete FAILED', 'ok': ok, 'errors': errs}
        log('deleted %s: %s' % (df.name, ok))
cuff = C.subfolder(root, 'cuff')
if cuff is not None:
    n, m = cuff.dataFiles.count, cuff.dataFolders.count
    log('cuff folder files=%d folders=%d' % (n, m))
    if n == 0 and m == 0:
        d['folder cuff'] = {'id': cuff.id, 'action': 'deleted', 'ok': cuff.deleteMe()}
        log('deleted folder cuff: %s' % d['folder cuff']['ok'])
C.save(s)
