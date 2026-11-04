"""EXO-278b: TARGET only. jIKLpT (uploaded from archive of source MQGUPD v1) -> 'wrist_sketch3_with_straps (copy v1)'.
s_bSGtj (archive of source FrHFTv v5 = v4 doc re-saved by Esteban) keeps 'wrist_sketch3_with_straps'."""
import json, os
ST = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274/state.json'
state = json.load(open(ST)); r = state.setdefault('exo278b', {})
PLAN = [('urn:adsk.wipprod:dm.lineage:jIKLpTb3SCeDiKx5Latkcw', 'wrist_sketch3_with_straps (copy v1)'),
        ('urn:adsk.wipprod:dm.lineage:s_bSGtj1SL-TvkFV7MHp3Q', 'wrist_sketch3_with_straps')]
res = {}
for fid, new in PLAN:
    df = app.data.findFileById(fid)
    assert df.parentProject.parentHub.name == 'Estebans' and df.parentProject.id == state['target']['project_id']
    try:
        if df.name != new:
            df.name = new
        res[fid[-22:]] = 'ok: %s' % app.data.findFileById(fid).name
    except Exception as e:
        res[fid[-22:]] = 'FAILED: %s' % e
r['wrist_rename'] = res
log(res)
json.dump(state, open(ST + '.tmp', 'w'), indent=1, default=str); os.replace(ST + '.tmp', ST)
