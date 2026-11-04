"""EXO-208: why do the placed modules not render? (EXO-203's bulb bug, re-checked)"""
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)

root = design.rootComponent
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    log('root %-38r bulb=%s vis=%s' % (o.component.name, o.isLightBulbOn, o.isVisible))
arm = [root.occurrences.item(i) for i in range(root.occurrences.count)
       if root.occurrences.item(i).component.name.startswith('generated_R_arm_general')][0]
for i in range(arm.childOccurrences.count):
    w = arm.childOccurrences.item(i)
    log('  %-34r bulb=%s vis=%s bodies=%d' % (w.component.name, w.isLightBulbOn, w.isVisible,
                                              w.bRepBodies.count))
    for j in range(w.childOccurrences.count):
        m = w.childOccurrences.item(j)
        log('      %-30r bulb=%s vis=%s' % (m.component.name, m.isLightBulbOn, m.isVisible))
        for k in range(min(3, m.childOccurrences.count)):
            c = m.childOccurrences.item(k)
            b = c.bRepBodies.item(0) if c.bRepBodies.count else None
            log('          %-26r bulb=%s vis=%s body.isVisible=%s'
                % (c.component.name, c.isLightBulbOn, c.isVisible,
                   b.isVisible if b else 'n/a'))
