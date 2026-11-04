"""EXO-283 save test on one doc (param.json name, route a|b|c): open, replace extras, verify counts,
save via the given route, pump events up to 90 s waiting for a new version, then ALWAYS close(False)."""
import sys, importlib, time
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import dedupe_exo283_common as C; importlib.reload(C)
T0 = time.time(); g = C.get
pm = C.param(); name = pm['name']; route = pm['route']; fid = C.ASM[name]
s = C.load(); s.setdefault('savetest', [])
df = C.find_file(app, fid); v0 = df.latestVersionNumber
r = {'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'name': name, 'route': route, 'v0': v0,
     'active': g(lambda: app.data.activeProject.name) + ' @ ' + g(lambda: app.data.activeHub.name), 'actions': []}
KEYS = ('components', 'bodies', 'occurrences', 'linked', 'joints', 'asBuilt', 'rigidGroups')
before_docs = app.documents.count
try:
    assert app.documents.count == 0, 'other docs open: %d' % app.documents.count
    doc = app.documents.open(df, True)
    des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    b = C.inspect(adsk, doc); r['before'] = {k: b[k] for k in KEYS}
    for _ in range(20):
        hit = None
        for o in C.items(des.rootComponent.allOccurrences):
            if g(lambda o=o: o.isReferencedComponent) is True:
                f = g(lambda o=o: o.documentReference.dataFile.id)
                if f in C.EXTRA: hit = (o, f); break
        if hit is None: break
        o, f = hit; kept = C.find_file(app, C.KEPT[C.EXTRA[f]])
        ok = o.replace(kept, True)
        r['actions'].append({'occ': g(lambda: o.fullPathName), 'from': C.short(f), 'to': C.short(kept.id), 'ok': ok})
        assert ok
    a = C.inspect(adsk, doc); r['after'] = {k: a[k] for k in KEYS}
    r['refs_after'] = [(C.short(d['id']), d['ver'], d['outOfDate']) for d in a['doc_refs']]
    r['counts_equal'] = r['before'] == r['after']
    r['no_extras'] = not any(d['id'] in C.EXTRA for d in a['doc_refs'])
    log('actions %s\nbefore %s\nafter  %s refs %s' % (r['actions'], r['before'], r['after'], r['refs_after']))
    if r['actions'] and r['counts_equal'] and r['no_extras']:
        r['isModified_pre'] = g(lambda: doc.isModified)
        if route == 'a':
            r['save'] = doc.save('EXO-283 relink shared parts to parts/')
        elif route == 'b':
            doc.activate(); r['save'] = doc.save('EXO-283 relink shared parts to parts/')
        elif route == 'c':
            doc.activate()
            cd = ui.commandDefinitions.itemById(pm.get('cmd', 'PLM360SaveCommand'))
            r['cmd'] = g(lambda: cd.id); r['save'] = cd.execute() if cd else None
        log('save route %s -> %s (isModified pre %s, post %s)' % (route, r.get('save'), r['isModified_pre'], g(lambda: doc.isModified)))
        t1 = time.time(); v = v0
        while time.time() - t1 < 90:
            adsk.doEvents(); time.sleep(0.5)
            v = g(lambda: doc.dataFile.latestVersionNumber)
            if isinstance(v, int) and v > v0: break
        r['wait_s'] = round(time.time() - t1, 1); r['v_in_doc'] = v; r['isModified_post'] = g(lambda: doc.isModified)
finally:
    for d in reversed(C.items(app.documents)[before_docs:]):
        try: d.close(False)
        except Exception: pass
    r['open_docs_after'] = [g(lambda d=d: d.name) for d in C.items(app.documents)]
    r['v_after_close'] = g(lambda: C.find_file(app, fid).latestVersionNumber)
    r['s'] = round(time.time() - T0, 1)
    s['savetest'].append(r); C.save(s)
log('%s route %s: v%s -> in-doc v%s, after close v%s, isModified_post %s, open after %s (%.1fs)' % (name, route, v0, r.get('v_in_doc'), r['v_after_close'], r.get('isModified_post'), r['open_docs_after'], r['s']))
