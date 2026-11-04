d = adsk.fusion.Design.cast(app.activeProduct)
log('doc %r' % app.activeDocument.name)
root = d.rootComponent
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    log('ROOT %r ref=%s' % (o.component.name, o.isReferencedComponent))
    for j in range(o.childOccurrences.count):
        c = o.childOccurrences.item(j)
        n = c.bRepBodies.count
        s = ''
        if n:
            bb = c.bRepBodies.item(0).boundingBox
            s = 'x %.3f..%.3f z %.3f..%.3f' % (bb.minPoint.x*10, bb.maxPoint.x*10, bb.minPoint.z*10, bb.maxPoint.z*10)
        log('   %r bodies %d (comp bodies %d) %s' % (c.component.name, n, c.component.bRepBodies.count, s))
