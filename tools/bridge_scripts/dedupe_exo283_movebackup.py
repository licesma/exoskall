"""EXO-283: move the 4 copies referenced by 'arm complete BACKUP 2026-08-08' from root to arm/old (DataFile.move;
BACKUP itself is not opened or modified). Idempotent; verifies destination."""
import sys, importlib, time
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import dedupe_exo283_common as C; importlib.reload(C)
g = C.get; s = C.load(); m = s.setdefault('backup_moves', {})
OLD = 'urn:adsk.wipprod:fs.folder:co.hEFWNiQ6SreB72cZWN1nYw'
old = C.retry(lambda: app.data.findFolderById(OLD)); assert old.name == 'old' and old.parentProject.id == C.PROJECT_ID
for fid in C.BACKUP_COPIES:
    df = C.find_file(app, fid)
    src = df.parentFolder
    if src.id == OLD:
        m[C.short(fid)] = m.get(C.short(fid), {'action': 'already in arm/old'}); continue
    ok = C.retry(lambda: df.move(old))
    now = C.find_file(app, fid).parentFolder.id
    m[C.short(fid)] = {'name': df.name, 'from': src.name, 'ok': ok, 'now_in_old': now == OLD,
                       'parents': [(r.name, C.short(r.id)) for r in C.items(df.parentReferences)]}
    log('%s %s: %s' % (C.short(fid), df.name[:30], m[C.short(fid)]))
C.save(s)
