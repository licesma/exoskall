"""EXO-283: report upload state of open docs; close a doc (unmodified only) once its new version exists."""
import sys, importlib, time
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import dedupe_exo283_common as C; importlib.reload(C)
g = C.get; s = C.load(); ids = set(C.ASM.values())
for d in C.items(app.documents):
    fid = g(lambda: d.dataFile.id)
    lv = g(lambda: C.find_file(app, fid).latestVersionNumber) if fid in ids else None
    vb = None
    for n, r in s.get('relink', {}).items():
        if C.ASM[n] == fid: vb = r['version_before']; nm = n
    log('%s %s isSaved=%s isModified=%s dataFile.v=%s latest=%s before=%s' % (g(lambda: d.name), C.short(fid), g(lambda: d.isSaved), g(lambda: d.isModified), g(lambda: d.dataFile.versionNumber), lv, vb))
    if vb is not None and isinstance(lv, int) and lv > vb and g(lambda: d.isModified) is False:
        d.close(False); s['relink'][nm]['version_after'] = lv; s['relink'][nm]['closed'] = time.strftime('%H:%M:%S')
        log('  closed; %s now v%s' % (nm, lv))
C.save(s)
