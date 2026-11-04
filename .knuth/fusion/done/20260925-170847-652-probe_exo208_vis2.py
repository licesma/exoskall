"""EXO-208: is the arm's module display state tied to the stop module's ROOT occurrence?"""
root = design.rootComponent
vp = app.activeViewport
OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/exo208_vistest_%s.png'
stop = [root.occurrences.item(i) for i in range(root.occurrences.count)
        if root.occurrences.item(i).component.name.startswith('sensor module 2 stop')][0]
for state in (True, False):
    stop.isLightBulbOn = state
    adsk.doEvents()
    cam = vp.camera
    cam.isFitView = True
    vp.camera = cam
    adsk.doEvents()
    h = int(round(1600.0 * vp.height / vp.width))
    log('root stop15 bulb=%s vis=%s -> %s'
        % (stop.isLightBulbOn, stop.isVisible, vp.saveAsImageFile(OUT % state, 1600, h)))
