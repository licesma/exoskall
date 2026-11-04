"""EXO-283: retry deleteMe on one zero-parent extra (param.json short) with 8 tries x 4 s; records result."""
import sys, importlib, time
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import dedupe_exo283_common as C; importlib.reload(C)
s = C.load(); sh = C.param()['short']
fid = [f for f in C.EXTRA if C.short(f) == sh][0]
df = C.find_file(app, fid); pr = C.items(df.parentReferences); assert not pr
log('versions: %s' % [v.versionNumber for v in C.items(df.versions)])
errs = []; ok = None
for t in range(8):
    try: ok = df.deleteMe(); break
    except Exception as e: errs.append(str(e)[:300]); time.sleep(4)
s['deletes'][sh].setdefault('retries', []).append({'ok': ok, 'errors': errs, 'time': time.strftime('%H:%M:%S')})
if ok: s['deletes'][sh]['action'] = 'deleted'
C.save(s); log('%s ok=%s errs=%s' % (sh, ok, errs[:2]))
