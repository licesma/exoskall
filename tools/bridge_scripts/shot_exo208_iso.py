"""EXO-208: one iso of the arm, with an explicit redraw first."""
vp = app.activeViewport
root = design.rootComponent
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    o.isLightBulbOn = o.component.name.startswith('generated_R_arm_general')
adsk.doEvents()
vp.refresh()
adsk.doEvents()
cam = vp.camera
cam.isFitView = True
vp.camera = cam
adsk.doEvents()
vp.refresh()
adsk.doEvents()
h = int(round(1600.0 * vp.height / vp.width))
p = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/generated_R_arm_exo208_arm_iso.png'
log('saved %s -> %s' % (p, vp.saveAsImageFile(p, 1600, h)))
