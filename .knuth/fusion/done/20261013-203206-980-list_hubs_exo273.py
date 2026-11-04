"""EXO-273: READ-ONLY listing of Fusion hubs/projects + API-surface introspection.

Logs app.version, app.currentUser, every hub in app.data.dataHubs (name, id, hubType)
and each hub's projects (name, id), read via hub.dataProjects WITHOUT switching
app.data.activeHub. Then checks hasattr for DataFile.copy/move, DataFolder.uploadFile,
ExportManager.createFusionArchiveExportOptions. Writes a JSON next to the EXO-271 one.

NOTHING is copied, moved, uploaded, deleted, opened, closed or saved.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/list_hubs_exo273.py --timeout 300
"""
import json
import os
import time

OUT = '/Users/esteban/repos/exo/gento_luna/exo/exports/fusion_hubs_exo273.json'
T0 = time.time()
res = {'issue': 'EXO-273', 'started': time.strftime('%Y-%m-%d %H:%M:%S')}


def get(fn):
    try:
        return fn()
    except Exception as e:
        return 'ERR: %s' % (str(e).splitlines()[0] if str(e) else type(e).__name__)


def coll(c):
    return [c.item(i) for i in range(c.count)]


HUBTYPES = {}
for n in ('PersonalHubType', 'TeamHubType'):
    v = getattr(adsk.core.HubTypes, n, None)
    if v is not None:
        HUBTYPES[v] = n

res['version'] = get(lambda: app.version)
log('Fusion version: %s' % res['version'])
u = get(lambda: app.currentUser)
res['user'] = {k: get(lambda k=k: getattr(u, k)) for k in ('displayName', 'email', 'userName', 'userId')}
log('currentUser: %s' % res['user'])

active_hub = get(lambda: app.data.activeHub)
active_proj = get(lambda: app.data.activeProject)
res['active_hub_before'] = get(lambda: active_hub.name)
res['active_project_before'] = get(lambda: active_proj.name)
log('active hub/project before: %s / %s' % (res['active_hub_before'], res['active_project_before']))

res['hubs'] = []
for h in get(lambda: coll(app.data.dataHubs)) or []:
    ht = get(lambda: h.hubType)
    rec = {'name': get(lambda: h.name), 'id': get(lambda: h.id),
           'hubType': HUBTYPES.get(ht, ht)}
    projs = get(lambda: coll(h.dataProjects))
    if isinstance(projs, list):
        rec['projects'] = [{'name': get(lambda p=p: p.name), 'id': get(lambda p=p: p.id)} for p in projs]
    else:
        rec['projects'] = projs
    res['hubs'].append(rec)
    log('HUB %s | %s | %s' % (rec['name'], rec['hubType'], rec['id']))
    for p in rec['projects'] if isinstance(rec['projects'], list) else [rec['projects']]:
        log('    project: %s' % (p,))

res['active_hub_after'] = get(lambda: app.data.activeHub.name)
res['active_project_after'] = get(lambda: app.data.activeProject.name)
log('active hub/project after: %s / %s' % (res['active_hub_after'], res['active_project_after']))

api = {
    'DataFile.copy': hasattr(adsk.core.DataFile, 'copy'),
    'DataFile.move': hasattr(adsk.core.DataFile, 'move'),
    'DataFolder.uploadFile': hasattr(adsk.core.DataFolder, 'uploadFile'),
    'DataFolder.uploadAssembly': hasattr(adsk.core.DataFolder, 'uploadAssembly'),
    'ExportManager.createFusionArchiveExportOptions': hasattr(adsk.fusion.ExportManager, 'createFusionArchiveExportOptions'),
    'DataFile.copy_doc': get(lambda: (adsk.core.DataFile.copy.__doc__ or '')[:400]),
    'DataFile.move_doc': get(lambda: (adsk.core.DataFile.move.__doc__ or '')[:400]),
    'DataFolder.uploadFile_doc': get(lambda: (adsk.core.DataFolder.uploadFile.__doc__ or '')[:400]),
    'DataFolder.uploadAssembly_doc': get(lambda: (adsk.core.DataFolder.uploadAssembly.__doc__ or '')[:400]),
}
res['api'] = api
for k, v in api.items():
    log('API %s: %s' % (k, v))

res['elapsed_s'] = round(time.time() - T0, 1)
with open(OUT + '.tmp', 'w') as f:
    json.dump(res, f, indent=1, default=str)
os.replace(OUT + '.tmp', OUT)
log('DONE -> %s (%.0fs)' % (OUT, time.time() - T0))
