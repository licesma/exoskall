"""Where did the J1/J2/J3 wrappers put the module? Read-only."""
root = design.rootComponent
gen = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name.startswith('generated_R_arm'):
        gen = o
if gen is None:
    log('no generated_R_arm')
else:
    log('gen children: %s' % [gen.childOccurrences.item(i).component.name
                              for i in range(gen.childOccurrences.count)])
    for i in range(gen.childOccurrences.count):
        w = gen.childOccurrences.item(i)
        if not w.component.name.startswith('J'):
            continue
        t = w.transform.translation
        log('%s: ctx=%s transform t=(%.2f, %.2f, %.2f) isLightBulbOn=%s isVisible=%s' % (
            w.component.name, w.assemblyContext.component.name if w.assemblyContext else 'None',
            t.x, t.y, t.z, w.isLightBulbOn, w.isVisible))
        bb = w.boundingBox
        log('   occ.boundingBox: %s' % ('empty' if not bb else
            'min=(%.2f, %.2f, %.2f) max=(%.2f, %.2f, %.2f)' % (
                bb.minPoint.x, bb.minPoint.y, bb.minPoint.z,
                bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z)))
        for k in range(w.childOccurrences.count):
            c = w.childOccurrences.item(k)
            ct = c.transform.translation
            log('   child %s ctx=%s t=(%.2f, %.2f, %.2f) bodies=%d children=%d' % (
                c.component.name, c.assemblyContext.component.name if c.assemblyContext else 'None',
                ct.x, ct.y, ct.z, c.bRepBodies.count, c.childOccurrences.count))
            cb = c.boundingBox
            log('      occ.boundingBox: %s' % ('empty' if not cb else
                'min=(%.2f, %.2f, %.2f) max=(%.2f, %.2f, %.2f)' % (
                    cb.minPoint.x, cb.minPoint.y, cb.minPoint.z,
                    cb.maxPoint.x, cb.maxPoint.y, cb.maxPoint.z)))
            for b in range(min(c.bRepBodies.count, 2)):
                bd = c.bRepBodies.item(b)
                bbb = bd.boundingBox
                log('      body %s ctx=%s min=(%.2f, %.2f, %.2f) max=(%.2f, %.2f, %.2f)' % (
                    bd.name, 'proxy' if bd.assemblyContext else 'native',
                    bbb.minPoint.x, bbb.minPoint.y, bbb.minPoint.z,
                    bbb.maxPoint.x, bbb.maxPoint.y, bbb.maxPoint.z))
            for g in range(min(c.childOccurrences.count, 2)):
                gc = c.childOccurrences.item(g)
                gt = gc.transform.translation
                gb = gc.boundingBox
                log('      grandchild %s ctx=%s t=(%.2f, %.2f, %.2f) bodies=%d bbox=%s' % (
                    gc.component.name,
                    gc.assemblyContext.component.name if gc.assemblyContext else 'None',
                    gt.x, gt.y, gt.z, gc.bRepBodies.count,
                    'empty' if not gb else 'min=(%.2f, %.2f, %.2f) max=(%.2f, %.2f, %.2f)' % (
                        gb.minPoint.x, gb.minPoint.y, gb.minPoint.z,
                        gb.maxPoint.x, gb.maxPoint.y, gb.maxPoint.z)))
