"""EXO-282 step 1: create folder tree in Estebans/G1 exeskeleton (idempotent)."""
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import organize_exo282_common as C; importlib.reload(C)
s = C.load()
PATHS = ['arm/upper body/joint/rotor', 'arm/upper body/joint/stators', 'arm/upper body/link',
         'arm/exo upper body/joint', 'arm/exo upper body/link', 'arm/old', 'body mount',
         'hand/cuff', 'hand/robot/wuji', 'hand/robot/inspire', 'parts', 'other']
tp = C.target(app); root = tp.rootFolder
made = s.setdefault('folders', {})
for p in PATHS:
    f = root; cur = ''
    for part in p.split('/'):
        cur = (cur + '/' + part).strip('/')
        sub = C.subfolder(f, part)
        if sub is None:
            sub = f.dataFolders.add(part)
            log('created %s' % cur)
        made[cur] = sub.id
        f = sub
C.save(s)
log('folders: %s' % sorted(made))
