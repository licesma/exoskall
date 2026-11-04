root = design.rootComponent
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    o.isLightBulbOn = o.component.name == 'generated_R_arm_general'
vp = app.activeViewport
cam = vp.camera
cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
cam.isFitView = True
vp.camera = cam
adsk.doEvents()
vp.fit(); adsk.doEvents()
h = int(round(1600.0 * vp.height / vp.width))
vp.saveAsImageFile('/Users/esteban/repos/exo/gento_luna/exo/screenshots/_probe_general_fit.png', 1600, h)
log('saved')
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name == 'generated_R_arm_general':
        for k in range(o.component.occurrences.count):
            c = o.component.occurrences.item(k)
            bb = None
            try:
                bb = c.boundingBox
            except Exception as e:
                log('%s bbox err %s' % (c.component.name, e))
            if bb:
                log('%-30s bbox %.1f %.1f %.1f .. %.1f %.1f %.1f (cm)' % (c.component.name,
                    bb.minPoint.x, bb.minPoint.y, bb.minPoint.z, bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z))
