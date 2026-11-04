"""EXO-274 step 5: READ-ONLY walk of the TARGET project (hub Estebans / G1 exeskeleton), same logic as EXO-271.
No hub switch: hub/project found by id from migration_exo274/state.json.

(Original EXO-271 docstring follows.)
EXO-271: READ-ONLY inventory of the Fusion cloud project "G1 exeskeleton".

Walks app.data: active hub, every project name, then recursively every DataFolder /
DataFile of the target project. Pass 1 collects names/dates/versions; pass 2 collects
reference edges (childReferences / parentReferences), which are the slow calls.
Every property read is wrapped; failures are stored as "ERR: ..." strings.

NOTHING is moved, created, deleted, saved, opened or closed.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/inventory_fusion_data_exo271.py --timeout 600
"""
import json
import os
import time

OUT = '/Users/esteban/repos/exo/gento_luna/exo/exports/fusion_data_inventory_exo274_target.json'
WANT_PROJECT = 'G1 exeskeleton'
T0 = time.time()
inv = {'issue': 'EXO-274', 'started': time.strftime('%Y-%m-%d %H:%M:%S'), 'errors': []}


def flush(stage):
    inv['stage'] = stage
    inv['elapsed_s'] = round(time.time() - T0, 1)
    tmp = OUT + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(inv, f, indent=1, default=str)
    os.replace(tmp, OUT)


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
    # EXO-278: per-item tolerant (items still processing after an f3z upload raise InternalValidationError)
    out = []
    n = c.count
    for i in range(n):
        try:
            out.append(c.item(i))
        except Exception as e:
            inv.setdefault('item_errors', []).append('item %d/%d: %s' % (i, n, str(e).splitlines()[0] if str(e) else type(e).__name__))
    return out


_st = json.load(open('/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274/state.json'))['target']
hub = [app.data.dataHubs.item(i) for i in range(app.data.dataHubs.count) if app.data.dataHubs.item(i).id == _st['hub_id']][0]
inv['hub'] = get(lambda: hub.name)
log('hub: %s' % inv['hub'])
projects = get(lambda: coll(hub.dataProjects))
inv['projects'] = [get(lambda p=p: p.name) for p in projects] if isinstance(projects, list) else projects
log('projects (%s): %s' % (len(inv['projects']), inv['projects']))

proj = None
if isinstance(projects, list):
    for p in projects:
        if get(lambda p=p: p.name) == WANT_PROJECT:
            proj = p
if proj is None:
    proj = hub.dataProjects.itemById(_st['project_id'])
    inv['errors'].append('project %r not found by name; fell back to active doc project' % WANT_PROJECT)
inv['project'] = get(lambda: proj.name)
inv['project_id'] = get(lambda: proj.id)
log('project: %s' % inv['project'])

inv['folders'] = []
inv['files'] = []
handles = []  # (record, DataFile)


def walk(folder, path):
    frec = {'path': '/'.join(path), 'name': get(lambda: folder.name), 'id': get(lambda: folder.id)}
    inv['folders'].append(frec)
    files = get(lambda: coll(folder.dataFiles))
    if not isinstance(files, list):
        frec['files_error'] = files
        files = []
    frec['file_count'] = len(files)
    for df in files:
        r = {'path': '/'.join(path)}
        r['name'] = get(lambda: df.name)
        r['fileExtension'] = get(lambda: df.fileExtension)
        r['id'] = get(lambda: df.id)
        r['dateCreated'] = iso(get(lambda: df.dateCreated))
        r['dateModified'] = iso(get(lambda: df.dateModified))
        r['latestVersionNumber'] = get(lambda: df.latestVersionNumber)
        r['versionNumber'] = get(lambda: df.versionNumber)
        r['isComplete'] = get(lambda: df.isComplete)
        r['lastUpdatedBy'] = get(lambda: df.lastUpdatedBy.displayName)
        r['createdBy'] = get(lambda: df.createdBy.displayName)
        inv['files'].append(r)
        handles.append((r, df))
        if len(inv['files']) % 10 == 0:
            log('pass1 %d files (%.0fs)' % (len(inv['files']), time.time() - T0))
            flush('pass1')
    subs = get(lambda: coll(folder.dataFolders))
    if not isinstance(subs, list):
        frec['folders_error'] = subs
        subs = []
    frec['subfolder_count'] = len(subs)
    for s in subs:
        walk(s, path + [get(lambda s=s: s.name)])


walk(proj.rootFolder, [''])
log('pass1 done: %d folders, %d files (%.0fs)' % (len(inv['folders']), len(inv['files']), time.time() - T0))
flush('pass1_done')


def refs(c):
    out = []
    for d in coll(c):
        out.append({'name': get(lambda: d.name), 'id': get(lambda: d.id),
                    'versionNumber': get(lambda: d.versionNumber)})
    return out


for n, (r, df) in enumerate(handles, 1):
    r['hasChildReferences'] = get(lambda: df.hasChildReferences)
    r['hasParentReferences'] = get(lambda: df.hasParentReferences)
    r['childReferences'] = get(lambda: refs(df.childReferences))
    r['parentReferences'] = get(lambda: refs(df.parentReferences))
    if n % 10 == 0:
        log('pass2 %d/%d (%.0fs)' % (n, len(handles), time.time() - T0))
        flush('pass2')

flush('done')
log('DONE %d folders %d files in %.0fs -> %s' % (len(inv['folders']), len(inv['files']), time.time() - T0, OUT))
