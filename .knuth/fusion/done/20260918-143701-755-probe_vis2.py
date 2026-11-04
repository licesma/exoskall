root = design.rootComponent
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    o.isLightBulbOn = o.component.name == 'generated_R_arm_general'
def walk(occ, d=0):
    log('%s%-28s on=%s visible=%s bodies=%d' % ('  '*d, occ.component.name,
        occ.isLightBulbOn, occ.isVisible, occ.component.bRepBodies.count))
    for k in range(occ.component.occurrences.count):
        walk(occ.component.occurrences.item(k), d+1)
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name == 'generated_R_arm_general':
        walk(o)
