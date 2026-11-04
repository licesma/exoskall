"""EXO-283 final (copied from EXO-282 step 4): READ-ONLY walk of Estebans/G1 exeskeleton; write final_tree.json."""
import sys, importlib, json, os, time
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import organize_exo282_common as C; importlib.reload(C)
OUT = '/Users/esteban/repos/exo/gento_luna/exo/exports/dedupe_exo283/final_tree.json'
import dedupe_exo283_common as D; importlib.reload(D)
s = C.load(); T0 = time.time(); g = C.get
tp = C.target(app)
R = {'issue': 'EXO-283', 'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'hub': 'Estebans', 'project': tp.name, 'folders': [], 'files': []}
def refs(c):
    if isinstance(c, str): return [{'error': c}]
    return [{'name': g(lambda k=k: k.name), 'id': g(lambda k=k: k.id), 'project_id': g(lambda k=k: k.parentProject.id),
             'hub': g(lambda k=k: k.parentProject.parentHub.name), 'folder_id': g(lambda k=k: k.parentFolder.id),
             'folder': g(lambda k=k: k.parentFolder.name)} for k in C.items(c)]
def walk(f, path):
    fs = C.items(f.dataFiles); subs = C.items(f.dataFolders)
    R['folders'].append({'path': path or '/', 'id': f.id, 'file_count': len(fs), 'subfolders': [x.name for x in subs]})
    for df in fs:
        R['files'].append({'path': path or '/', 'name': df.name, 'id': df.id, 'v': g(lambda: df.versionNumber),
                           'folder_id': f.id,
                           'childReferences': refs(g(lambda: df.childReferences)),
                           'parentReferences': refs(g(lambda: df.parentReferences))})
    log('%s: %d files (%.0fs)' % (path or '/', len(fs), time.time() - T0))
    for x in subs:
        walk(x, path + '/' + x.name)
walk(tp.rootFolder, '')
ids = {f['id']: f for f in R['files']}
deleted = {v['id'] for v in s.get('deletes', {}).values() if 'id' in v} | {v['id'] for v in D.load().get('deletes', {}).values() if v.get('action') == 'deleted'}
problems = []
for f in R['files']:
    for k in f['childReferences']:
        if 'error' in k or k['id'] not in ids or k['project_id'] != s['project_id'] or ids[k['id']]['folder_id'] != k['folder_id']:
            problems.append({'parent': f['name'], 'parent_id': f['id'], 'child': k})
        if k.get('id') in deleted:
            problems.append({'parent': f['name'], 'points_to_deleted': k})
    for k in f['parentReferences']:
        if 'error' in k or k['id'] not in ids:
            problems.append({'child': f['name'], 'parentRef_unresolved': k})
R['deleted_present'] = [i for i in deleted if i in ids]
R['edges'] = sum(len(f['childReferences']) for f in R['files'])
R['problems'] = problems
R['active_hub_after'] = g(lambda: app.data.activeHub.name); R['active_project_after'] = g(lambda: app.data.activeProject.name)
R['active_project_id_after'] = g(lambda: app.data.activeProject.id)
R['open_docs_after'] = [g(lambda i=i: app.documents.item(i).name) for i in range(app.documents.count)]
R['elapsed_s'] = round(time.time() - T0, 1)
with open(OUT + '.tmp', 'w') as fh: json.dump(R, fh, indent=1, default=str)
os.replace(OUT + '.tmp', OUT)
R['edges_to_extras'] = [(f['name'], D.short(f['id']), k.get('name'), D.short(k.get('id')), D.classify(k.get('id'))) for f in R['files'] for k in f['childReferences'] if D.classify(k.get('id')) != 'kept' and D.classify(k.get('id')) != 'other']
with open(OUT + '.tmp', 'w') as fh: json.dump(R, fh, indent=1, default=str)
os.replace(OUT + '.tmp', OUT)
log('edges not to kept copies: %s' % R['edges_to_extras'])
log('files %d edges %d problems %d deleted_present %s active=%s/%s' % (len(R['files']), R['edges'], len(problems), R['deleted_present'], R['active_hub_after'], R['active_project_after']))
