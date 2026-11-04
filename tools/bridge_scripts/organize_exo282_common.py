"""EXO-282 shared helpers (imported by organize_exo282_*.py bridge scripts)."""
import json, os, time
ST = '/Users/esteban/repos/exo/gento_luna/exo/exports/organize_exo282/state.json'

def load():
    return json.load(open(ST)) if os.path.exists(ST) else {}

def save(state):
    with open(ST + '.tmp', 'w') as f:
        json.dump(state, f, indent=1, default=str)
    os.replace(ST + '.tmp', ST)

def get(fn):
    try:
        return fn()
    except Exception as e:
        return 'ERR: %s' % (str(e).splitlines()[0] if str(e) else type(e).__name__)

def items(c, tries=5):
    n = c.count
    out = []
    for i in range(n):
        last = None
        for t in range(tries):
            try:
                out.append(c.item(i)); last = None; break
            except Exception as e:
                last = e; time.sleep(0.5)
        if last is not None:
            raise RuntimeError('item %d failed: %s' % (i, last))
    return out

def target(app):
    hubs = [h for h in items(app.data.dataHubs) if h.name == 'Estebans']
    assert len(hubs) == 1, 'hub'
    ps = [p for p in items(hubs[0].dataProjects) if p.name == 'G1 exeskeleton']
    assert len(ps) == 1, 'project'
    assert ps[0].parentHub.name == 'Estebans'
    return ps[0]

def subfolder(folder, name):
    m = [f for f in items(folder.dataFolders) if f.name == name]
    assert len(m) <= 1, 'dup folder %s' % name
    return m[0] if m else None

def path_folder(root, path):
    f = root
    for part in [p for p in path.split('/') if p]:
        f = subfolder(f, part)
        if f is None:
            return None
    return f

def short(i):
    return i.split(':')[-1][:6]
