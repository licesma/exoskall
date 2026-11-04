import sys
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import dedupe_exo283_common as C
d = [x for x in C.items(app.documents) if C.get(lambda x=x: x.dataFile.id) == C.ASM['sensor module pGmH']][0]
log('active doc: %s' % C.get(lambda: app.activeDocument.name))
log('dataFile.isReadOnly? %s' % C.get(lambda: d.dataFile.isReadOnly))
log('perm: %s' % [a for a in dir(d) if 'ave' in a or 'ead' in a])
log('ui active cmd: %s' % C.get(lambda: ui.activeCommand))
