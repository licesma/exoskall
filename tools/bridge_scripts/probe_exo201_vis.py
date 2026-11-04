"""EXO-201: why do the modules not render? Report the light-bulb state down the tree."""
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm, BuildArmGen
importlib.reload(BuildArm); importlib.reload(BuildArmGen)

root = design.rootComponent
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    log('root: %-34s bulb=%s' % (o.component.name, o.isLightBulbOn))

arm = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name.startswith(BuildArmGen.ARM_NAME):
        arm = o
log('--- under %s (bulb=%s, isVisible=%s) ---' % (arm.component.name, arm.isLightBulbOn,
                                                  arm.isVisible))
for i in range(arm.childOccurrences.count):
    c = arm.childOccurrences.item(i)
    log('  %-34s bulb=%s vis=%s bodies=%d' % (c.component.name, c.isLightBulbOn,
                                              c.isVisible, c.bRepBodies.count))
    for k in range(c.childOccurrences.count):
        g = c.childOccurrences.item(k)
        log('    %-30s bulb=%s vis=%s children=%d' % (g.component.name, g.isLightBulbOn,
                                                      g.isVisible, g.childOccurrences.count))
        for m in range(min(g.childOccurrences.count, 30)):
            gg = g.childOccurrences.item(m)
            vb = [gg.bRepBodies.item(x).isVisible for x in range(gg.bRepBodies.count)]
            log('      %-26s bulb=%s vis=%s bodyvis=%s' % (gg.component.name,
                                                           gg.isLightBulbOn, gg.isVisible,
                                                           vb[:3]))
