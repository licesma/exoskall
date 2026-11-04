"""EXO-201: does the ROOT `sensor module 2` bulb drive the nested ones?"""
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm, BuildArmGen
importlib.reload(BuildArm); importlib.reload(BuildArmGen)
root = design.rootComponent
rootmod = arm = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name == BuildArm.MODULE_COMPONENT_NAME if hasattr(BuildArm, 'MODULE_COMPONENT_NAME') else o.component.name == 'sensor module 2':
        rootmod = o
    if o.component.name.startswith(BuildArmGen.ARM_NAME):
        arm = o


def nested():
    out = []
    for i in range(arm.childOccurrences.count):
        c = arm.childOccurrences.item(i)
        for k in range(c.childOccurrences.count):
            out.append(c.childOccurrences.item(k).isLightBulbOn)
    return out


log('root sensor module 2 bulb=%s; nested bulbs=%s' % (rootmod.isLightBulbOn, nested()))
rootmod.isLightBulbOn = False
log('after root bulb=False       ; nested bulbs=%s' % (nested(),))
rootmod.isLightBulbOn = True
log('after root bulb=True        ; nested bulbs=%s' % (nested(),))
