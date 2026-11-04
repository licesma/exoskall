root = design.rootComponent
log('top-level occurrences:')
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    log('  %-30s light=%s' % (o.component.name, o.isLightBulbOn))
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name != 'right arm':
        continue
    for j in range(o.childOccurrences.count):
        c = o.childOccurrences.item(j)
        t = c.transform2.translation
        bb = c.boundingBox
        log('  %-22s light=%-5s t=(%8.3f,%8.3f,%8.3f) bb=(%.2f,%.2f,%.2f)..(%.2f,%.2f,%.2f)'
            % (c.component.name, c.isLightBulbOn, t.x, t.y, t.z,
               bb.minPoint.x, bb.minPoint.y, bb.minPoint.z,
               bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z) if bb else
            '  %-22s light=%s t=(%8.3f,%8.3f,%8.3f) no bb' % (c.component.name, c.isLightBulbOn, t.x, t.y, t.z))
        for k in range(c.childOccurrences.count):
            g = c.childOccurrences.item(k)
            log('      %-26s light=%s' % (g.component.name, g.isLightBulbOn))
