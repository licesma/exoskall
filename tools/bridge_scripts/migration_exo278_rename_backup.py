"""EXO-278: rename the TARGET doc uploaded from 'arm complete v1.f3z' (lineage xWFsuHDcTy64WIBsxPfAiQ; its children
include the 25mm SHCS 7XBtxEtKQKecKXWSNQMdlQ returned by that upload) to 'arm complete BACKUP 2026-08-08'. Target only."""
import json, os
ST = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274/state.json'
state = json.load(open(ST))
df = app.data.findFileById('urn:adsk.wipprod:dm.lineage:xWFsuHDcTy64WIBsxPfAiQ')
assert df.parentProject.parentHub.name == 'Estebans' and df.parentProject.id == state['target']['project_id']
kids = [df.childReferences.item(i).id for i in range(df.childReferences.count)]
log('found %r; children %s' % (df.name, kids))
assert 'urn:adsk.wipprod:dm.lineage:7XBtxEtKQKecKXWSNQMdlQ' in kids
NEW = 'arm complete BACKUP 2026-08-08'
try:
    if df.name != NEW:
        df.name = NEW
    state['exo278']['backup_rename'] = 'ok: %s' % app.data.findFileById(df.id).name
except Exception as e:
    state['exo278']['backup_rename'] = 'FAILED: %s' % e
log(state['exo278']['backup_rename'])
json.dump(state, open(ST + '.tmp', 'w'), indent=1, default=str); os.replace(ST + '.tmp', ST)
