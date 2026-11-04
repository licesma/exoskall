"""EXO-283 phase 1 READ-ONLY: open one assembly (param.json 'name'), inspect, close(False). Never saves."""
import sys, importlib, time
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import dedupe_exo283_common as C; importlib.reload(C)
T0 = time.time(); g = C.get
name = C.param()['name']; fid = C.ASM[name]
s = C.load(); s.setdefault('dryrun', {})
df = C.find_file(app, fid)
pre = [d for d in C.items(app.documents) if g(lambda d=d: d.dataFile.id) == fid]
before = app.documents.count
r = {'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'file': df.name, 'id': fid, 'version': df.versionNumber, 'reused_open': bool(pre)}
try:
    doc = pre[0] if pre else app.documents.open(df, True)
    log('opened %s in %.1fs' % (doc.name, time.time() - T0))
    r.update(C.inspect(adsk, doc))
    r['isModified'] = g(lambda: doc.isModified)
finally:
    if not pre:
        for d in reversed(C.items(app.documents)[before:]):
            try: d.close(False)
            except Exception as e: log('close: %s' % e)
    r['open_docs_after'] = [g(lambda d=d: d.name) for d in C.items(app.documents)]
    r['s'] = round(time.time() - T0, 1)
    s['dryrun'][name] = r; C.save(s)
log('%s v%s: comps %s bodies %s occ %s linked %s joints %s asBuilt %s rg %s health %s (%.1fs)' % (
    name, r['version'], r.get('components'), r.get('bodies'), r.get('occurrences'), r.get('linked'), r.get('joints'),
    r.get('asBuilt'), r.get('rigidGroups'), r.get('joint_health'), r['s']))
for x in r.get('linked_occ', []):
    log('  %s | owner=%s | %s %s v%s | %s | joints=%d rg=%d' % (x['occ'], x['owner_link'], x['file'], C.short(x['file_id']), x['ver'], x['class'], len(x['joints']), len(x['rigidGroups'])))
