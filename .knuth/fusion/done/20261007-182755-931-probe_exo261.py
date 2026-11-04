org = app.activeDocument
log('active %r' % org.name)
des = adsk.fusion.Design.cast(app.activeProduct)
log('design type: %s' % ('parametric' if des.designType == adsk.fusion.DesignTypes.ParametricDesignType else 'direct'))
root = des.rootComponent
bb = root.boundingBox
log('organizer bbox cm min (%.3f %.3f %.3f) max (%.3f %.3f %.3f)' % (bb.minPoint.x, bb.minPoint.y, bb.minPoint.z, bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z))
log('root bodies %d occ %d features(base) %d' % (root.bRepBodies.count, root.occurrences.count, root.features.baseFeatures.count))
im = app.importManager
STEP = '/Users/esteban/repos/exo/gento_luna/Gento Luna_URDF 20260428/urdf/sensor module 2.step'
tmp = im.importToNewDocument(im.createSTEPImportOptions(STEP))
tdes = adsk.fusion.Design.cast(tmp.products.itemByProductType('DesignProductType'))
troot = tdes.rootComponent
log('tmp doc %r root occ %d bodies %d' % (tmp.name, troot.occurrences.count, troot.bRepBodies.count))
for o in troot.allOccurrences:
    n = o.component.name
    if any(k in n.lower() for k in ('bearing', 'xl330', '5972')):
        b = o.boundingBox
        log('  %r path=%r bodies=%d sub=%d bbox mm %.3f x %.3f x %.3f' % (n, o.fullPathName, o.bRepBodies.count, o.childOccurrences.count,
            10*(b.maxPoint.x-b.minPoint.x), 10*(b.maxPoint.y-b.minPoint.y), 10*(b.maxPoint.z-b.minPoint.z)))
names = sorted(set(o.component.name for o in troot.allOccurrences))
log('all component names (%d): %s' % (len(names), names[:80]))
tmp.close(False)
org.activate()
log('closed tmp, active %r' % app.activeDocument.name)
