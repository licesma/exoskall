"""EXO-278 (1): upload Esteban's UI-made archive/arm complete.f3d into TARGET Estebans/G1 exeskeleton root. No hub switch.
Polls DataFileFuture.uploadState with doEvents+sleep, max 200 s. Writes result into migration_exo274/state.json['exo278']."""
import json, os, time
BASE = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274'
ST = BASE + '/state.json'
SRC = '/Users/esteban/repos/exo/archive/arm complete.f3d'
state = json.load(open(ST))
r = state.setdefault('exo278', {})
if r.get('upload_status') == 'finished':
    log('already uploaded: %s' % r); raise SystemExit(0)
US = adsk.core.UploadStates
hub = [app.data.dataHubs.item(i) for i in range(app.data.dataHubs.count) if app.data.dataHubs.item(i).id == state['target']['hub_id']][0]
proj = hub.dataProjects.itemById(state['target']['project_id'])
t0 = time.time()
up = proj.rootFolder.uploadFile(SRC)
log('upload started %s (%d bytes)' % (SRC, os.path.getsize(SRC)))
st = None
while time.time() - t0 < 200:
    st = up.uploadState
    if st in (US.UploadFinished, US.UploadFailed):
        break
    adsk.doEvents(); time.sleep(1)
r['upload_s'] = round(time.time() - t0, 1)
r['upload_status'] = {US.UploadFinished: 'finished', US.UploadFailed: 'failed'}.get(st, 'processing')
if r['upload_status'] == 'finished':
    r['df_name'] = up.dataFile.name; r['df_id'] = up.dataFile.id
log('result: %s' % r)
json.dump(state, open(ST + '.tmp', 'w'), indent=1, default=str); os.replace(ST + '.tmp', ST)
