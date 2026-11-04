"""EXO-274 step 0b: READ-ONLY. Record open documents' dataFile ids / isModified / isSaved into state.json."""
import json, os
ST = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274/state.json'
state = json.load(open(ST))
def get(fn):
    try:
        return fn()
    except Exception as e:
        return 'ERR: %s' % (str(e).splitlines()[0] if str(e) else type(e).__name__)
docs = []
for i in range(app.documents.count):
    d = app.documents.item(i)
    r = {'name': get(lambda: d.name), 'isModified': get(lambda: d.isModified), 'isSaved': get(lambda: d.isSaved),
         'df_id': get(lambda: d.dataFile.id if d.dataFile else None), 'df_ver': get(lambda: d.dataFile.versionNumber if d.dataFile else None),
         'df_project': get(lambda: d.dataFile.parentProject.name if d.dataFile else None), 'isVisible': get(lambda: d.isVisible)}
    docs.append(r); log(r)
state['start']['open_docs_detail'] = docs
json.dump(state, open(ST + '.tmp', 'w'), indent=1, default=str); os.replace(ST + '.tmp', ST)
