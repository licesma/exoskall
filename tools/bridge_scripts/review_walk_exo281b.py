"""EXO-281 reviewer (b): READ-ONLY re-walk of the target root folder only, with per-item retry
(first run hit 'InternalValidationError : item' on DataFiles.item). Merges into review_exo281.json."""
import json, os, time
OUT = '/Users/esteban/repos/exo/gento_luna/exo/exports/review_exo281.json'
R = json.load(open(OUT))
T0 = time.time()

def get(fn):
    try:
        return fn()
    except Exception as e:
        return 'ERR: %s' % (str(e).splitlines()[0] if str(e) else type(e).__name__)

def iso(v):
    if isinstance(v, (int, float)) and v > 0:
        return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(v))
    return v

def coll_retry(c, tries=4):
    n = get(lambda: c.count)
    out, errs = [], []
    if not isinstance(n, int):
        return out, ['count: %s' % n]
    for i in range(n):
        it = None
        for t in range(tries):
            it = get(lambda: c.item(i))
            if not isinstance(it, str):
                break
            time.sleep(0.5)
        if isinstance(it, str):
            errs.append('item %d: %s' % (i, it))
        else:
            out.append(it)
    return out, errs

def flush(stage):
    R['stage'] = stage; R['elapsed_b_s'] = round(time.time() - T0, 1)
    with open(OUT + '.tmp', 'w') as f:
        json.dump(R, f, indent=1, default=str)
    os.replace(OUT + '.tmp', OUT)

hubs = {get(lambda h=h: h.name): h for h in [app.data.dataHubs.item(i) for i in range(app.data.dataHubs.count)]}
h = hubs['Estebans']
tp = [p for p in [h.dataProjects.item(i) for i in range(h.dataProjects.count)] if get(lambda p=p: p.name) == 'G1 exeskeleton']
assert len(tp) == 1, tp
tp = tp[0]
root = tp.rootFolder
log('root folder %s count=%s' % (get(lambda: root.name), get(lambda: root.dataFiles.count)))
dfs, errs = coll_retry(root.dataFiles)
log('root listed %d files, errors %s (%.0fs)' % (len(dfs), errs, time.time() - T0))
R['target']['folders'][0]['files_error'] = errs or None
R['target']['folders'][0]['file_count'] = len(dfs)
root_files = []
for df in dfs:
    r = {'path': ''}
    r['name'] = get(lambda: df.name)
    r['fileExtension'] = get(lambda: df.fileExtension)
    r['id'] = get(lambda: df.id)
    r['dateCreated'] = iso(get(lambda: df.dateCreated))
    r['dateModified'] = iso(get(lambda: df.dateModified))
    r['versionNumber'] = get(lambda: df.versionNumber)
    r['latestVersionNumber'] = get(lambda: df.latestVersionNumber)
    r['lastUpdatedBy'] = get(lambda: df.lastUpdatedBy.displayName)
    r['createdBy'] = get(lambda: df.createdBy.displayName)
    r['parentProject'] = get(lambda: df.parentProject.name)
    r['parentHub'] = get(lambda: df.parentProject.parentHub.name)
    r['hasChildReferences'] = get(lambda: df.hasChildReferences)
    kids, kerrs = coll_retry(get(lambda: df.childReferences)) if not isinstance(get(lambda: df.childReferences), str) else ([], ['childReferences: ' + get(lambda: df.childReferences)])
    r['childReferences_errors'] = kerrs or None
    r['childReferences'] = [{
        'name': get(lambda k=k: k.name), 'id': get(lambda k=k: k.id),
        'versionNumber': get(lambda k=k: k.versionNumber),
        'parentProject': get(lambda k=k: k.parentProject.name),
        'parentProject_id': get(lambda k=k: k.parentProject.id),
        'parentHub': get(lambda k=k: k.parentProject.parentHub.name),
        'parentFolder': get(lambda k=k: k.parentFolder.name),
    } for k in kids]
    root_files.append(r)
    if len(root_files) % 10 == 0:
        log('target root: %d files (%.0fs)' % (len(root_files), time.time() - T0))
        R['target']['files'] = root_files + [f for f in R['target']['files'] if f['path'] != '']
        flush('target_root_b')
R['target']['files'] = root_files + [f for f in R['target']['files'] if f['path'] != '']
R['active_hub_after_b'] = get(lambda: app.data.activeHub.name)
R['active_project_after_b'] = get(lambda: app.data.activeProject.name)
R['open_docs_after_b'] = [get(lambda i=i: app.documents.item(i).name) for i in range(app.documents.count)]
flush('target_root_b_done')
log('after: hub=%s project=%s open=%s' % (R['active_hub_after_b'], R['active_project_after_b'], R['open_docs_after_b']))
log('DONE target files total %d (%.0fs)' % (len(R['target']['files']), time.time() - T0))
