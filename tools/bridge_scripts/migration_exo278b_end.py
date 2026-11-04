"""EXO-278b: READ-ONLY. Compare current active hub/project/open docs with exo278b.start."""
import json, os
ST = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274/state.json'
state = json.load(open(ST)); r = state['exo278b']; s = r['start']
def get(fn):
    try: return fn()
    except Exception as e: return 'ERR: %s' % str(e).splitlines()[0]
cur = {'active_hub_id': get(lambda: app.data.activeHub.id), 'active_project_id': get(lambda: app.data.activeProject.id),
       'active_doc': get(lambda: app.activeDocument.name),
       'open_docs': [[get(lambda i=i: app.documents.item(i).name), get(lambda i=i: app.documents.item(i).dataFile.id),
                      get(lambda i=i: app.documents.item(i).dataFile.versionNumber)] for i in range(app.documents.count)],
       'modified': [get(lambda i=i: app.documents.item(i).isModified) for i in range(app.documents.count)]}
same = all(cur[k] == s[k] for k in ('active_hub_id', 'active_project_id', 'active_doc')) and cur['open_docs'] == [list(x) for x in s['open_docs']]
r['end'] = cur; r['end_matches_start'] = same
log('matches start: %s %s' % (same, cur))
json.dump(state, open(ST + '.tmp', 'w'), indent=1, default=str); os.replace(ST + '.tmp', ST)
