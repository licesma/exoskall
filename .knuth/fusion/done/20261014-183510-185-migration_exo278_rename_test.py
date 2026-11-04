"""EXO-278: rename the 1-body test upload in the TARGET root (lineage p-BdRVWoTmaJDy3jun2ljQ). Target only."""
import json, os
ST = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274/state.json'
state = json.load(open(ST))
df = app.data.findFileById('urn:adsk.wipprod:dm.lineage:p-BdRVWoTmaJDy3jun2ljQ')
hub = df.parentProject.parentHub.name
log('found %r in %s / %s / folder %s' % (df.name, hub, df.parentProject.name, df.parentFolder.name))
assert hub == 'Estebans' and df.parentProject.id == state['target']['project_id']
NEW = 'arm complete (1-body test upload, delete me)'
try:
    if df.name != NEW:
        df.name = NEW
    df2 = app.data.findFileById(df.id)
    state['exo278']['test_upload_rename'] = 'ok: %s' % df2.name
except Exception as e:
    state['exo278']['test_upload_rename'] = 'FAILED: %s' % e
log(state['exo278']['test_upload_rename'])
json.dump(state, open(ST + '.tmp', 'w'), indent=1, default=str); os.replace(ST + '.tmp', ST)
