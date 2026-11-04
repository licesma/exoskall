"""Read-only: list root occurrences, flag 'sensor module 2*' copies at root."""
root = design.rootComponent
log('document: %s  design type: %s' % (app.activeDocument.name,
    'parametric' if design.designType == adsk.fusion.DesignTypes.ParametricDesignType else 'direct'))
log('root component: %s   root occurrences: %d' % (root.name, root.occurrences.count))
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    t = o.transform.translation
    bb = o.boundingBox
    log('  [%d] %-28s vis=%s t=(%.2f, %.2f, %.2f) cm  bbox=%s' % (
        i, o.component.name, o.isLightBulbOn, t.x, t.y, t.z,
        ('(%.1f,%.1f,%.1f)-(%.1f,%.1f,%.1f)' % (bb.minPoint.x, bb.minPoint.y, bb.minPoint.z,
         bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z)) if bb else 'none'))
names = [c.name for c in design.allComponents if c.name.startswith('sensor module 2')]
log('components named sensor module 2*: %s' % names)
# every occurrence of those components, anywhere
for c in design.allComponents:
    if not c.name.startswith('sensor module 2'):
        continue
    occs = design.rootComponent.allOccurrencesByComponent(c)
    log('component %s -> %d occurrences' % (c.name, occs.count))
    for k in range(occs.count):
        o = occs.item(k)
        ctx = o.assemblyContext.component.name if o.assemblyContext else 'ROOT'
        t = o.transform2.translation if hasattr(o, 'transform2') else o.transform.translation
        log('   under %-28s t=(%.2f, %.2f, %.2f) cm' % (ctx, t.x, t.y, t.z))
