"""Fit the view on the whole design and take the hand-off screenshot.
Also logs which root occurrences are currently visible (STEP export only writes
visible geometry, which is why an export of this document is 11 MB, not 112 MB).
"""
root = design.rootComponent
vis = [(root.occurrences.item(i).component.name, root.occurrences.item(i).isLightBulbOn,
        root.occurrences.item(i).isVisible) for i in range(root.occurrences.count)]
log('root occurrences (name, lightBulb, visible):')
for n, b, v in vis:
    if n.startswith(('left arm', 'right arm', 'chest', 'Camera', 'sensor', 'generated')):
        log('   %-24s bulb=%s visible=%s' % (n, b, v))

vp = app.activeViewport
cam = vp.camera
cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
cam.isFitView = True
vp.camera = cam
adsk.doEvents()
vp.fit()
adsk.doEvents()
log('screenshot -> %s' % screenshot(
    '/Users/esteban/repos/exo/gento_luna/exo/screenshots/generated_R_arm.png'))
