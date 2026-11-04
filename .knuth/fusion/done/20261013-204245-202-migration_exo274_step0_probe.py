"""EXO-274 step 0: READ-ONLY probe. Records start hub/project + open docs into state.json, checks API surface."""
import json, os, time
ST = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274/state.json'
state = json.load(open(ST)) if os.path.exists(ST) else {'issue': 'EXO-274'}

def get(fn):
    try:
        return fn()
    except Exception as e:
        return 'ERR: %s' % (str(e).splitlines()[0] if str(e) else type(e).__name__)

if 'start' not in state:
    state['start'] = {
        'hub_name': get(lambda: app.data.activeHub.name), 'hub_id': get(lambda: app.data.activeHub.id),
        'project_name': get(lambda: app.data.activeProject.name), 'project_id': get(lambda: app.data.activeProject.id),
        'open_docs': [get(lambda i=i: app.documents.item(i).name) for i in range(app.documents.count)],
        'active_doc': get(lambda: app.activeDocument.name),
        'time': time.strftime('%Y-%m-%d %H:%M:%S')}
log('start: %s' % state['start'])
api = {}
for cls, attrs in (('DataProjects', ['add']), ('DataFolders', ['add', 'itemByName']), ('DataFile', ['download', 'copy']),
                   ('DataUpload', ['uploadState', 'dataFile']), ('Documents', ['open']), ('DataFolder', ['uploadFile'])):
    c = getattr(adsk.core, cls, None)
    for a in attrs:
        api['%s.%s' % (cls, a)] = bool(c is not None and hasattr(c, a))
api['UploadStates'] = [n for n in dir(adsk.core.UploadStates) if not n.startswith('_')] if hasattr(adsk.core, 'UploadStates') else None
api['DataProjects.add_doc'] = get(lambda: (adsk.core.DataProjects.add.__doc__ or '')[:300])
api['DataFile.download_doc'] = get(lambda: (adsk.core.DataFile.download.__doc__ or '')[:400])
state['api'] = api
log('api: %s' % api)
for i in range(app.data.dataHubs.count):
    h = app.data.dataHubs.item(i)
    log('hub %s %s projects=%s' % (h.name, h.id, [h.dataProjects.item(j).name for j in range(h.dataProjects.count)]))
json.dump(state, open(ST + '.tmp', 'w'), indent=1, default=str); os.replace(ST + '.tmp', ST)
log('DONE')
