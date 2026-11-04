root = design.rootComponent
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    log('ROOT %-30s on=%s children=%d' % (o.component.name, o.isLightBulbOn, o.component.occurrences.count))
    if o.component.name == 'generated_R_arm_general':
        for k in range(o.component.occurrences.count):
            c = o.component.occurrences.item(k)
            log('   %-28s on=%s bodies=%d subocc=%d' % (c.component.name, c.isLightBulbOn,
                c.component.bRepBodies.count, c.component.occurrences.count))
            for m in range(c.component.occurrences.count):
                d = c.component.occurrences.item(m)
                log('      sub %-22s on=%s' % (d.component.name, d.isLightBulbOn))
