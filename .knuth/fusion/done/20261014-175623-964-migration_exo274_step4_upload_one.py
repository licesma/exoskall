"""EXO-274 step 4: upload items to the TARGET project (hub Estebans), no hub switch.
First re-polls any stashed in-flight DataUploads (adsk._exo274_up); then starts ONE upload for the next
'exported' item, copying the export to upload/<n>/<target_name><ext> so the uploaded doc gets the target name.
Polls uploadState with adsk.doEvents()+sleep(1) for at most MAXWAIT s, then stashes and returns.
Status: exported -> uploading -> uploaded | upload_failed."""
import json, os, time, shutil, traceback
BASE = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274'
ST = BASE + '/state.json'
MAXWAIT = 200
state = json.load(open(ST))
T0 = time.time()
US = adsk.core.UploadStates
def save():
    json.dump(state, open(ST + '.tmp', 'w'), indent=1, default=str); os.replace(ST + '.tmp', ST)
def get(fn):
    try:
        return fn()
    except Exception as e:
        return 'ERR: %s' % (str(e).splitlines()[0] if str(e) else type(e).__name__)
if not hasattr(adsk, '_exo274_up'):
    adsk._exo274_up = {}
hub = [app.data.dataHubs.item(i) for i in range(app.data.dataHubs.count) if app.data.dataHubs.item(i).id == state['target']['hub_id']][0]
proj = hub.dataProjects.itemById(state['target']['project_id'])
folders = {'root': proj.rootFolder, 'cuff': proj.rootFolder.dataFolders.itemByName('cuff')}

def poll(it, up, budget):
    t = time.time()
    while time.time() - t < budget:
        s = up.uploadState
        if s == US.UploadFinished:
            it['status'] = 'uploaded'
            it['upload_df_name'] = get(lambda: up.dataFile.name)
            it['upload_df_id'] = get(lambda: up.dataFile.id)
            it['upload_s'] = round(time.time() - it['upload_t0'], 1)
            adsk._exo274_up.pop(it['n'], None)
            log('item %d uploaded as %r (%.0fs)' % (it['n'], it['upload_df_name'], it['upload_s']))
            return
        if s == US.UploadFailed:
            it['status'] = 'upload_failed'
            adsk._exo274_up.pop(it['n'], None)
            log('item %d upload FAILED' % it['n'])
            return
        adsk.doEvents()
        time.sleep(1)
    log('item %d still processing after %.0fs total' % (it['n'], time.time() - it['upload_t0']))

for it in state['items']:
    if it['status'] == 'uploading':
        up = adsk._exo274_up.get(it['n'])
        if up is None:
            it['status'] = 'upload_lost_handle'
            log('item %d: lost DataUpload handle (verify in walk)' % it['n'])
        else:
            poll(it, up, max(5, MAXWAIT - (time.time() - T0)))
            if time.time() - it['upload_t0'] > 600 and it['status'] == 'uploading':
                it['status'] = 'upload_timeout'
        save()
if time.time() - T0 < MAXWAIT - 20 and not adsk._exo274_up:
    todo = [it for it in state['items'] if it['status'] == 'exported']
    if todo:
        it = todo[0]
        try:
            ext = os.path.splitext(it['export_path'])[1]
            if it['kind'] == 'download':
                fname = it['target_name'] if it['target_name'].lower().endswith(ext.lower()) else it['target_name'] + ext
            else:
                fname = it['target_name'] + ext
            d = BASE + '/upload/%02d' % it['n']
            os.makedirs(d, exist_ok=True)
            up_path = os.path.join(d, fname)
            shutil.copy2(it['export_path'], up_path)
            it['upload_path'] = up_path
            folder = folders[it['target_folder']]
            it['upload_t0'] = time.time()
            up = folder.uploadFile(up_path)
            assert up is not None, 'uploadFile returned None'
            adsk._exo274_up[it['n']] = up
            it['status'] = 'uploading'
            save()
            log('item %d upload started: %s -> %s/%s' % (it['n'], fname, proj.name, it['target_folder']))
            poll(it, up, MAXWAIT - (time.time() - T0))
        except Exception:
            it['status'] = 'upload_failed'
            it['error'] = traceback.format_exc()[-800:]
            log('FAILED %s' % it['error'])
        save()
    else:
        log('nothing to upload')
log('counts: %s' % {s: sum(1 for i in state['items'] if i['status'] == s) for s in set(i['status'] for i in state['items'])})
log('active hub/project: %s / %s (%.0fs)' % (app.data.activeHub.name, app.data.activeProject.name, time.time() - T0))
