"""EXO-218: what is in the document after the Luna build - both arms, body counts."""
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm, armgen
importlib.reload(BuildArm)
root = design.rootComponent
log('root occurrences: %d' % root.occurrences.count)
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    bs, n = BuildArm.collect_bodies(o)
    log('  %-32s children %2d  bodies %4d  visible %s'
        % (o.component.name, o.childOccurrences.count, bs.count, o.isLightBulbOn))
log('components in design: %d' % design.allComponents.count)
names = sorted(design.allComponents.item(i).name for i in range(design.allComponents.count))
log('  stop15-ish: %s' % [n for n in names if 'stop15' in n or 'sensor module' in n])
for arm in ('generated_R_arm_general', 'generated_R_arm_luna'):
    occ = None
    for i in range(root.occurrences.count):
        if root.occurrences.item(i).component.name == arm:
            occ = root.occurrences.item(i)
    if occ is None:
        log('%s: NOT PRESENT' % arm)
        continue
    kids = [occ.childOccurrences.item(i) for i in range(occ.childOccurrences.count)]
    log('%s: %d children' % (arm, len(kids)))
    for c in kids:
        bs, _n = BuildArm.collect_bodies(c)
        log('    %-18s bodies %3d  sub %d' % (c.component.name, bs.count,
                                              c.childOccurrences.count))
