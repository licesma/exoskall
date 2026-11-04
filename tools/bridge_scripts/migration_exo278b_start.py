"""EXO-278b: READ-ONLY. Record active hub/project/doc and open docs before the remaining steps (once)."""
import json, os, time
ST = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274/state.json'
state = json.load(open(ST))
def get(fn):
    try: return fn()
    except Exception as e: return 'ERR: %s' % str(e).splitlines()[0]
r = state.setdefault('exo278b', {})
cur = {'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'active_hub': get(lambda: app.data.activeHub.name),
       'active_hub_id': get(lambda: app.data.activeHub.id), 'active_project': get(lambda: app.data.activeProject.name),
       'active_project_id': get(lambda: app.data.activeProject.id), 'active_doc': get(lambda: app.activeDocument.name),
       'open_docs': [(get(lambda i=i: app.documents.item(i).name), get(lambda i=i: app.documents.item(i).dataFile.id),
                      get(lambda i=i: app.documents.item(i).dataFile.versionNumber)) for i in range(app.documents.count)]}
r.setdefault('start', cur)
log(json.dumps(cur, default=str))
json.dump(state, open(ST + '.tmp', 'w'), indent=1, default=str); os.replace(ST + '.tmp', ST)
