"""EXO-278 (2): upload Esteban's UI-made .f3z archives (archive/) into the TARGET root, one per run.
Copies each to migration_exo274/upload_f3z/<target name>.f3z first. Re-polls a stashed in-flight
DataFileFuture (adsk._exo278_up) before starting the next. Each run polls <=200 s (bridge 5-min rule);
an upload is given up after 15 min total. Falls back to uploadAssembly([path]) if uploadFile raises/returns None."""
import json, os, time, shutil, traceback
BASE = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274'
ST = BASE + '/state.json'
ARC = '/Users/esteban/repos/exo/archive/'
state = json.load(open(ST))
T0 = time.time(); BUDGET = 200
US = adsk.core.UploadStates
def save():
    json.dump(state, open(ST + '.tmp', 'w'), indent=1, default=str); os.replace(ST + '.tmp', ST)
def get(fn):
    try:
        return fn()
    except Exception as e:
        return 'ERR: %s' % (str(e).splitlines()[0] if str(e) else type(e).__name__)
if not hasattr(adsk, '_exo278_up'):
    adsk._exo278_up = {}
hub = [app.data.dataHubs.item(i) for i in range(app.data.dataHubs.count) if app.data.dataHubs.item(i).id == state['target']['hub_id']][0]
proj = hub.dataProjects.itemById(state['target']['project_id'])
items = state['exo278']['f3z']

def poll(it, up):
    while time.time() - T0 < BUDGET:
        s = up.uploadState
        if s == US.UploadFinished:
            it['status'] = 'uploaded'; it['upload_s'] = round(time.time() - it['t0'], 1)
            it['df_name'] = get(lambda: up.dataFile.name); it['df_id'] = get(lambda: up.dataFile.id)
            adsk._exo278_up.pop(it['file'], None)
            log('%s uploaded as %r id=%s (%.0fs)' % (it['file'], it['df_name'], it['df_id'], it['upload_s'])); return
        if s == US.UploadFailed:
            it['status'] = 'upload_failed'; it['upload_s'] = round(time.time() - it['t0'], 1)
            adsk._exo278_up.pop(it['file'], None)
            log('%s UploadFailed after %.0fs' % (it['file'], it['upload_s'])); return
        adsk.doEvents(); time.sleep(1)
    el = time.time() - it['t0']
    if el > 900:
        it['status'] = 'upload_timeout_15min'; adsk._exo278_up.pop(it['file'], None)
    log('%s still processing after %.0fs' % (it['file'], el))

for it in items:
    if it['status'] == 'uploading':
        up = adsk._exo278_up.get(it['file'])
        if up is None:
            it['status'] = 'lost_handle'; log('%s lost handle; check walk' % it['file'])
        else:
            poll(it, up)
        save()
if not adsk._exo278_up and time.time() - T0 < BUDGET - 30:
    todo = [it for it in items if it['status'] == 'pending']
    if todo:
        it = todo[0]
        try:
            d = BASE + '/upload_f3z'; os.makedirs(d, exist_ok=True)
            path = os.path.join(d, it['target_name'] + '.f3z')
            shutil.copy2(ARC + it['file'], path)
            it['upload_path'] = path; it['bytes'] = os.path.getsize(path)
            it['t0'] = time.time()
            up = None
            try:
                up = proj.rootFolder.uploadFile(path); it['method'] = 'uploadFile'
            except Exception as e:
                it['uploadFile_error'] = str(e)[:300]
            if up is None:
                up = proj.rootFolder.uploadAssembly([path]); it['method'] = 'uploadAssembly'
            adsk._exo278_up[it['file']] = up
            it['status'] = 'uploading'; save()
            log('%s (%d B) upload started via %s as %s' % (it['file'], it['bytes'], it['method'], os.path.basename(path)))
            poll(it, up)
        except Exception:
            it['status'] = 'upload_error'; it['error'] = traceback.format_exc()[-600:]; log(it['error'])
        save()
    else:
        log('nothing pending')
log('f3z statuses: %s (%.0fs)' % ([(i['file'], i['status']) for i in items], time.time() - T0))
