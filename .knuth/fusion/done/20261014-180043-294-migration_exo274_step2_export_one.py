"""EXO-274 step 2/3: export ONE pending item (next with status 'pending') from the SOURCE project.
Opens the source doc read-only, exports Fusion archive (.f3z/.f3d) or downloads non-Fusion data,
closes only documents this script opened (close(False)). Never saves. Skips docs open with unsaved changes.
Item status: pending -> exported | skipped_modified | export_failed. Idempotent via state.json."""
import json, os, time, shutil, traceback
BASE = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274'
ST = BASE + '/state.json'
state = json.load(open(ST))
T0 = time.time()
def save():
    json.dump(state, open(ST + '.tmp', 'w'), indent=1, default=str); os.replace(ST + '.tmp', ST)
def get(fn):
    try:
        return fn()
    except Exception as e:
        return 'ERR: %s' % (str(e).splitlines()[0] if str(e) else type(e).__name__)
def dockey(d):
    return (get(lambda: d.name), get(lambda: d.dataFile.id if d.dataFile else None), get(lambda: d.dataFile.versionNumber if d.dataFile else None))
def docs():
    return [app.documents.item(i) for i in range(app.documents.count)]

todo = [it for it in state['items'] if it['status'] == 'pending']
pri = [it for it in todo if it['n'] in state.get('priority', [])]
todo = pri + [it for it in todo if it not in pri]
if not todo:
    log('nothing pending'); raise SystemExit(0)
it = todo[0]
log('item %d %s v%s kind=%s' % (it['n'], it['src_name'], it['src_ver'], it['kind']))
active_before = app.activeDocument
opened_new = []
try:
    df = app.data.findFileById(it['src_id'])
    assert df is not None, 'findFileById returned None'
    it['src_check'] = {'name': df.name, 'ver': df.versionNumber, 'latest': df.latestVersionNumber, 'project': get(lambda: df.parentProject.name)}
    assert df.parentProject.name == 'G1 exeskeleton' and df.parentProject.parentHub.name == 'Team hub - Rodrigo Andrade'
    os.makedirs(BASE + '/exports', exist_ok=True)
    out = BASE + '/exports/' + it['export_file']
    if it['kind'] == 'download':
        tmpdir = BASE + '/dl_%02d' % it['n']
        os.makedirs(tmpdir, exist_ok=True)
        r = df.download(tmpdir, None)
        log('download returned %s; dir=%s' % (r, os.listdir(tmpdir)))
        files = os.listdir(tmpdir)
        assert files, 'download produced no file'
        shutil.copy2(os.path.join(tmpdir, files[0]), out)
        it['downloaded_as'] = files[0]
    else:
        # already open?
        pre = [d for d in docs() if get(lambda d=d: d.dataFile.id if d.dataFile else None) == it['src_id']]
        if any(get(lambda d=d: d.isModified) is True for d in pre):
            it['status'] = 'skipped_modified'
            it['note'] = 'open in Fusion with unsaved (recovered) changes: %s' % [get(lambda d=d: d.name) for d in pre]
            save(); log(it['note']); raise SystemExit(0)
        before = [dockey(d) for d in docs()]
        doc = None
        for d in pre:
            if get(lambda d=d: d.dataFile.versionNumber) == df.latestVersionNumber:
                doc = d; it['reused_open_doc'] = True
        if doc is None:
            doc = app.documents.open(df, True)
            it['reused_open_doc'] = False
        log('doc: %s ver=%s modified=%s (open %.1fs)' % (doc.name, get(lambda: doc.dataFile.versionNumber), doc.isModified, time.time() - T0))
        bl = list(before)
        for d in docs():
            k = dockey(d)
            if k in bl:
                bl.remove(k)
            else:
                opened_new.append(d)
        it['opened_new_docs'] = [get(lambda d=d: d.name) for d in opened_new]
        it['exported_from_version'] = get(lambda: doc.dataFile.versionNumber)
        des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        em = des.exportManager
        opts = em.createFusionArchiveExportOptions(out)
        ok = em.execute(opts)
        log('export execute=%s (%.1fs)' % (ok, time.time() - T0))
        cand = [f for f in os.listdir(BASE + '/exports') if f.startswith('%02d_' % it['n'])]
        if not os.path.exists(out) and cand:
            out = BASE + '/exports/' + cand[0]
        assert os.path.exists(out), 'no export file; candidates=%s' % cand
    it['export_path'] = out
    it['export_bytes'] = os.path.getsize(out)
    it['status'] = 'exported'
    log('exported %s (%d bytes)' % (out, it['export_bytes']))
except SystemExit:
    pass
except Exception as e:
    it['status'] = 'export_failed'
    it['error'] = traceback.format_exc()[-800:]
    log('FAILED: %s' % it['error'])
finally:
    closed = []
    for d in reversed(opened_new):
        try:
            if d.isValid and not d.isModified:
                n = d.name; d.close(False); closed.append(n)
            elif d.isValid:
                log('NOT closing modified doc %s' % d.name)
        except Exception as e:
            log('close err %s' % e)
    it['closed_docs'] = closed
    try:
        if active_before and active_before.isValid:
            active_before.activate()
    except Exception as e:
        log('reactivate err %s' % e)
    it['export_s'] = round(time.time() - T0, 1)
    save()
    log('closed=%s; remaining open docs=%d; %.1fs' % (closed, app.documents.count, time.time() - T0))
