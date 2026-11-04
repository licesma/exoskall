"""EXO-283 phase 2: open one assembly (param.json name, mode relink|latest), repoint extras to kept copies
with Occurrence.replace(keptFile, True) (or update out-of-date refs to latest), verify counts, save, close.
Idempotent: if nothing to change, nothing is saved."""
import sys, importlib, time
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import dedupe_exo283_common as C; importlib.reload(C)
T0 = time.time(); g = C.get
pm = C.param(); name = pm['name']; mode = pm['mode']; fid = C.ASM[name]
s = C.load(); s.setdefault('relink', {})
df = C.find_file(app, fid)
before_docs = app.documents.count
r = {'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'mode': mode, 'version_before': df.versionNumber, 'actions': []}
KEYS = ('components', 'bodies', 'occurrences', 'linked', 'joints', 'asBuilt', 'rigidGroups')
def own(snap):
    return [x for x in snap['linked_occ'] if x['owner_link'] is None]
try:
    doc = app.documents.open(df, True)
    des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    b = C.inspect(adsk, doc); r['before'] = {k: b[k] for k in KEYS}
    r['before_files'] = sorted(set(C.short(x['file_id']) + ':' + x['class'] for x in own(b)))
    if mode == 'relink':
        for _ in range(20):
            occs = [o for o in C.items(des.rootComponent.allOccurrences) if g(lambda o=o: o.isReferencedComponent) is True]
            hit = None
            for o in occs:
                f = g(lambda o=o: o.documentReference.dataFile.id)
                if f in C.EXTRA:
                    hit = (o, f); break
            if hit is None: break
            o, f = hit
            kept = C.find_file(app, C.KEPT[C.EXTRA[f]])
            ok = o.replace(kept, True)
            r['actions'].append({'occ': g(lambda: o.fullPathName), 'from': f, 'to': kept.id, 'to_v': kept.versionNumber, 'ok': ok})
            log('replace %s %s -> %s: %s' % (r['actions'][-1]['occ'], C.short(f), C.short(kept.id), ok))
            assert ok, 'replace returned False'
    elif mode == 'latest':
        for dr in C.items(doc.documentReferences):
            if dr.isOutOfDate:
                v0 = dr.version; ok = dr.getLatestVersion()
                r['actions'].append({'ref': dr.dataFile.name, 'id': dr.dataFile.id, 'from_v': v0, 'to_v': g(lambda: dr.version), 'ok': ok})
                log('latest %s %s v%s -> v%s: %s' % (dr.dataFile.name, C.short(dr.dataFile.id), v0, r['actions'][-1]['to_v'], ok))
                assert ok, 'getLatestVersion returned False'
    a = C.inspect(adsk, doc); r['after'] = {k: a[k] for k in KEYS}
    r['after_files'] = sorted(set(C.short(x['file_id']) + ':' + x['class'] for x in own(a)))
    r['after_doc_refs'] = [(C.short(d['id']), d['ver'], d['outOfDate']) for d in a['doc_refs']]
    r['counts_equal'] = r['before'] == r['after']
    r['no_extras'] = not any(x['class'].startswith('extra') for x in own(a)) and not any(d['id'] in C.EXTRA for d in a['doc_refs'])
    r['no_outdated'] = not any(d['outOfDate'] is True for d in a['doc_refs'])
    log('before %s\nafter  %s\nfiles %s -> %s refs %s' % (r['before'], r['after'], r['before_files'], r['after_files'], r['after_doc_refs']))
    if not r['actions']:
        r['saved'] = False; r['note'] = 'nothing to change'
    elif r['counts_equal'] and r['no_extras'] and r['no_outdated']:
        r['isModified_before_save'] = g(lambda: doc.isModified)
        r['saved'] = doc.save('EXO-283 relink shared parts to parts/')
        log('saved: %s isModified was %s' % (r['saved'], r['isModified_before_save']))
    else:
        r['saved'] = False; r['note'] = 'verification failed; closed without saving'
finally:
    if not r.get('saved'):
        for d in reversed(C.items(app.documents)[before_docs:]):
            try: d.close(False)
            except Exception as e: pass
    else:
        r['note'] = 'saved; left open for upload, closed by dedupe_exo283_close.py'
    r['open_docs_after'] = [g(lambda d=d: d.name) for d in C.items(app.documents)]
    r['version_after'] = g(lambda: C.find_file(app, fid).latestVersionNumber)
    r['s'] = round(time.time() - T0, 1)
    s['relink'][name] = r; C.save(s)
log('%s: v%s -> v%s saved=%s counts_equal=%s no_extras=%s (%.1fs)' % (name, r['version_before'], r['version_after'], r.get('saved'), r.get('counts_equal'), r.get('no_extras'), r['s']))
