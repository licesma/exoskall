import adsk.core, adsk.fusion
root = design.rootComponent
def walk(occ, path, depth=0):
    c = occ.component
    for i in range(c.bRepBodies.count):
        b = c.bRepBodies.item(i)
        bb = b.boundingBox
        log('%s%s / %s  vol %.1f mm3  bbox x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]' % (
            '  '*depth, path, b.name, b.volume*1000.0,
            bb.minPoint.x*10, bb.maxPoint.x*10, bb.minPoint.y*10, bb.maxPoint.y*10,
            bb.minPoint.z*10, bb.maxPoint.z*10))
    for i in range(c.occurrences.count):
        o = c.occurrences.item(i)
        if depth < 2:
            walk(o, path + '/' + o.component.name, depth+1)
log('--- root components with "Component" in the name ---')
for cc in design.allComponents:
    if cc.name.startswith('Component'):
        log('component %s: %d bodies, %d occurrences' % (cc.name, cc.bRepBodies.count, cc.occurrences.count))
        for i in range(cc.bRepBodies.count):
            b = cc.bRepBodies.item(i)
            bb = b.boundingBox
            log('   body %s vol %.1f mm3 bbox x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f] visible=%s'
                % (b.name, b.volume*1000.0, bb.minPoint.x*10, bb.maxPoint.x*10,
                   bb.minPoint.y*10, bb.maxPoint.y*10, bb.minPoint.z*10, bb.maxPoint.z*10,
                   b.isVisible))
log('--- root-level occurrences ---')
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    log('  %-40s bodies=%d light=%s' % (o.component.name, o.component.bRepBodies.count, o.isLightBulbOn))
