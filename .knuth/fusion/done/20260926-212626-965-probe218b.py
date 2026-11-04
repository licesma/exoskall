"""EXO-218: re-run the interference check on BOTH arms, so the Luna 0-hit answer is measured
against the G1 one rather than believed."""
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm, armgen
importlib.reload(BuildArm); importlib.reload(armgen)
root = design.rootComponent
for name in ('generated_R_arm_general', 'generated_R_arm_luna'):
    occ = None
    for i in range(root.occurrences.count):
        if root.occurrences.item(i).component.name == name:
            occ = root.occurrences.item(i)
    if occ is None:
        log('%s: absent' % name); continue
    n, hits = BuildArm.check_interference(design, occ)
    kinds = {}
    for h in hits:
        kinds[h[0]] = kinds.get(h[0], 0) + 1
    log('%-26s %d bodies, %d hits %s' % (name, n, len(hits), kinds))
    for h in hits[:4]:
        log('      %s' % (h,))
    fails = [h for h in hits if h[0] == 'FAIL']
    log('      FAIL: %d' % len(fails))
    for h in fails[:8]:
        log('        %s' % (h,))
