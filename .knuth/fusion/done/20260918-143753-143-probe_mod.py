root = design.rootComponent
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name == 'generated_R_arm_general':
        for k in range(o.component.occurrences.count):
            c = o.component.occurrences.item(k)
            if not c.component.name.startswith('GJ'):
                continue
            m = c.component.occurrences.item(0)
            log('%s -> %s on=%s vis=%s' % (c.component.name, m.component.name, m.isLightBulbOn, m.isVisible))
            for q in range(m.component.occurrences.count):
                s = m.component.occurrences.item(q)
                nb = s.component.bRepBodies.count
                vis = [s.component.bRepBodies.item(b).isLightBulbOn for b in range(nb)]
                log('    %-24s occ_on=%s bodies_on=%s' % (s.component.name, s.isLightBulbOn, vis[:4]))
            break
