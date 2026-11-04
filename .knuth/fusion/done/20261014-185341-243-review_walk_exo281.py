"""EXO-281 reviewer: READ-ONLY walk of target (Estebans / G1 exeskeleton) with child-reference
resolution, plus pass-1 (names/versions/dates) of source (Team hub - Rodrigo Andrade / G1 exeskeleton).
Hubs are read via app.data.dataHubs[i].dataProjects without changing activeHub.
Nothing is opened, saved, moved, renamed, uploaded or deleted."""
import json, os, time
OUT = '/Users/esteban/repos/exo/gento_luna/exo/exports/review_exo281.json'
T0 = time.time()
R = {'issue': 'EXO-281', 'started': time.strftime('%Y-%m-%d %H:%M:%S'), 'errors': []}

def get(fn):
    try:
        return fn()
    except Exception as e:
        return 'ERR: %s' % (str(e).splitlines()[0] if str(e) else type(e).__name__)

def iso(v):
    if isinstance(v, (int, float)) and v > 0:
        return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(v))
    return v

def coll(c):
    return [c.item(i) for i in range(c.count)]

def flush(stage):
    R['stage'] = stage; R['elapsed_s'] = round(time.time() - T0, 1)
    with open(OUT + '.tmp', 'w') as f:
        json.dump(R, f, indent=1, default=str)
    os.replace(OUT + '.tmp', OUT)

R['active_hub_before'] = get(lambda: app.data.activeHub.name)
R['active_project_before'] = get(lambda: app.data.activeProject.name)
R['active_doc_before'] = get(lambda: app.activeDocument.name)
R['open_docs_before'] = [get(lambda i=i: app.documents.item(i).name) for i in range(app.documents.count)]
log('before: hub=%s project=%s doc=%s open=%s' % (R['active_hub_before'], R['active_project_before'], R['active_doc_before'], R['open_docs_before']))

hubs = {get(lambda h=h: h.name): h for h in coll(app.data.dataHubs)}
R['hubs'] = list(hubs.keys())

def find_project(hubname, projname):
    h = hubs.get(hubname)
    if h is None:
        return None, 'hub %r not found' % hubname
    hits = [p for p in coll(h.dataProjects) if get(lambda p=p: p.name) == projname]
    if len(hits) != 1:
        return None, 'hub %r: %d projects named %r' % (hubname, len(hits), projname)
    return hits[0], None

def walk(proj, path_prefix, with_refs, files, folders):
    def rec(folder, path):
        fr = {'path': '/'.join(path), 'name': get(lambda: folder.name), 'id': get(lambda: folder.id)}
        folders.append(fr)
        dfs = get(lambda: coll(folder.dataFiles))
        if not isinstance(dfs, list):
            fr['files_error'] = dfs; dfs = []
        fr['file_count'] = len(dfs)
        for df in dfs:
            r = {'path': '/'.join(path)}
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
            if with_refs:
                r['hasChildReferences'] = get(lambda: df.hasChildReferences)
                kids = get(lambda: coll(df.childReferences))
                if isinstance(kids, list):
                    r['childReferences'] = [{
                        'name': get(lambda k=k: k.name), 'id': get(lambda k=k: k.id),
                        'versionNumber': get(lambda k=k: k.versionNumber),
                        'parentProject': get(lambda k=k: k.parentProject.name),
                        'parentProject_id': get(lambda k=k: k.parentProject.id),
                        'parentHub': get(lambda k=k: k.parentProject.parentHub.name),
                        'parentFolder': get(lambda k=k: k.parentFolder.name),
                    } for k in kids]
                else:
                    r['childReferences'] = kids
            files.append(r)
            if len(files) % 10 == 0:
                log('%s: %d files (%.0fs)' % (path_prefix, len(files), time.time() - T0)); flush(path_prefix)
        subs = get(lambda: coll(folder.dataFolders))
        if not isinstance(subs, list):
            fr['folders_error'] = subs; subs = []
        fr['subfolder_count'] = len(subs)
        for s in subs:
            rec(s, path + [get(lambda s=s: s.name)])
    rec(proj.rootFolder, [''])

# --- target ---
tp, err = find_project('Estebans', 'G1 exeskeleton')
R['target'] = {'hub': 'Estebans', 'project': 'G1 exeskeleton', 'project_id': get(lambda: tp.id) if tp else None, 'error': err, 'folders': [], 'files': []}
if tp:
    walk(tp, 'target', True, R['target']['files'], R['target']['folders'])
    log('target done: %d folders %d files (%.0fs)' % (len(R['target']['folders']), len(R['target']['files']), time.time() - T0))
else:
    R['errors'].append(err); log('TARGET ERROR %s' % err)
flush('target_done')

# --- source (pass 1 only: no reference calls) ---
sp, err = find_project('Team hub - Rodrigo Andrade', 'G1 exeskeleton')
R['source'] = {'hub': 'Team hub - Rodrigo Andrade', 'project': 'G1 exeskeleton', 'project_id': get(lambda: sp.id) if sp else None, 'error': err, 'folders': [], 'files': []}
if sp:
    walk(sp, 'source', False, R['source']['files'], R['source']['folders'])
    log('source done: %d folders %d files (%.0fs)' % (len(R['source']['folders']), len(R['source']['files']), time.time() - T0))
else:
    R['errors'].append(err); log('SOURCE ERROR %s' % err)

R['active_hub_after'] = get(lambda: app.data.activeHub.name)
R['active_project_after'] = get(lambda: app.data.activeProject.name)
R['active_doc_after'] = get(lambda: app.activeDocument.name)
R['open_docs_after'] = [get(lambda i=i: app.documents.item(i).name) for i in range(app.documents.count)]
log('after: hub=%s project=%s doc=%s open=%s' % (R['active_hub_after'], R['active_project_after'], R['active_doc_after'], R['open_docs_after']))
flush('walk_done')
log('DONE %.0fs -> %s' % (time.time() - T0, OUT))
