"""EXO-282 step 2: DataFile.move for the entries in run_ids.json (idempotent; verifies each)."""
import sys, importlib, json, time
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import organize_exo282_common as C; importlib.reload(C)
RUN = '/Users/esteban/repos/exo/gento_luna/exo/exports/organize_exo282/run_ids.json'
s = C.load(); T0 = time.time()
tp = C.target(app); root = tp.rootFolder
srcs = {'/': root, '/cuff': C.subfolder(root, 'cuff')}
done = s.setdefault('moves', {})
for e in json.load(open(RUN)):
    if done.get(e['id'], {}).get('verified'):
        continue
    dest = C.path_folder(root, e['to'])
    assert dest is not None and dest.id == s['folders'][e['to']], e['to']
    already = [f for f in C.items(dest.dataFiles) if f.id == e['id']]
    if not already:
        src = srcs[e['from']]
        m = [f for f in C.items(src.dataFiles) if f.id == e['id']]
        assert len(m) == 1, 'not found %s in %s' % (e['name'], e['from'])
        df = m[0]
        assert df.parentProject.id == s['project_id'] and df.parentProject.parentHub.name == 'Estebans'
        ok = df.move(dest)
        log('move %s %s -> %s: %s' % (e['name'], C.short(e['id']), e['to'], ok))
    got = [f for f in C.items(dest.dataFiles) if f.id == e['id']]
    gone = not [f for f in C.items(srcs[e['from']].dataFiles) if f.id == e['id']]
    done[e['id']] = {'name': e['name'], 'to': e['to'], 'verified': len(got) == 1 and gone,
                     'parentFolder': C.get(lambda: got[0].parentFolder.name) if got else None}
    log('verify %s: in dest=%s gone from src=%s (%.0fs)' % (e['name'], len(got), gone, time.time() - T0))
    C.save(s)
log('verified %d/%d' % (sum(1 for v in done.values() if v['verified']), len(done)))
