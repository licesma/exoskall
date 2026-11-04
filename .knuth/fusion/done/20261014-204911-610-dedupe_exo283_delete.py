"""EXO-283 delete step (param.json mode check|delete). For each root extra: list parentReferences; in 'delete'
mode deleteMe (Fusion trash) only if it has zero parents and is not protected. Repeats passes so a child
freed by deleting its parent (cuffs -> HEX) is handled. BACKUP copies are never deleted."""
import sys, importlib, time
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import dedupe_exo283_common as C; importlib.reload(C)
g = C.get; pm = C.param(); mode = pm['mode']; s = C.load(); d = s.setdefault('deletes', {})
# arm v1 stays on sensor module pGmH v1 (-> bYhy); arm test unchanged (-> i1pB, _vds, dW5W, 3GOu)
PROTECT = {C.P + x for x in ('bYhycXTtRDCF7RLAR_EaWQ', 'i1pB70ZESvaIAZxFwU7wSQ', '_vds__rqTwSDoxaVW9ch6g',
                             'dW5WClmISqWaUGbJ8iBm6g', '3GOuJvUxRySz5kPInpNDog')}
for p in range(3):
    changed = False
    for fid in list(C.EXTRA):
        if d.get(C.short(fid), {}).get('action') == 'deleted': continue
        df = g(lambda: C.find_file(app, fid))
        if isinstance(df, str): log('%s: %s' % (C.short(fid), df)); continue
        assert df.parentFolder.parentProject.id == C.PROJECT_ID
        pr = [(r.name, C.short(r.id), r.versionNumber) for r in C.items(df.parentReferences)]
        rec = {'id': fid, 'name': df.name, 'folder': g(lambda: df.parentFolder.name), 'parentReferences': pr}
        if pr or fid in PROTECT:
            rec['action'] = 'kept: ' + ('has parents' if pr else '') + (' protected' if fid in PROTECT else '')
        elif mode == 'delete':
            errs = []; ok = None
            for t in range(4):
                try: ok = df.deleteMe(); break
                except Exception as e: errs.append(str(e).splitlines()[0]); time.sleep(2)
            rec.update({'action': 'deleted' if ok else 'delete FAILED', 'ok': ok, 'errors': errs}); changed = changed or bool(ok)
        else:
            rec['action'] = 'would delete'
        d[C.short(fid)] = rec
        log('pass %d %s %s parents=%s -> %s' % (p, C.short(fid), df.name[:30], pr, rec['action']))
    C.save(s)
    if not changed: break
for fid in C.BACKUP_COPIES:
    df = C.find_file(app, fid)
    log('BACKUP copy %s %s parents=%s' % (C.short(fid), df.name[:30], [(r.name, C.short(r.id)) for r in C.items(df.parentReferences)]))
