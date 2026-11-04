import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)
for i in range(app.documents.count):
    d = app.documents.item(i)
    log('doc %d: %r saved=%s' % (i, d.name, d.isSaved))
    try:
        des = adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
        root = des.rootComponent
        for j in range(root.occurrences.count):
            o = root.occurrences.item(j)
            if 'sensor module' in o.component.name or 'module' in o.component.name.lower():
                log('   occ %r comp %r referenced=%s children=%d' % (o.name, o.component.name, o.isReferencedComponent, o.childOccurrences.count))
    except Exception as e:
        log('   (%s)' % e)
