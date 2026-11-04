"""EXO-261/262: screenshots of the example bearing + XL330 bins in tool_organizer_exo258.
Ortho top view of the whole organizer (tilted views were tried: the 78 mm walls crop the
parts). Set TAG below per issue. Run via tools/fusion_run.py.
"""
TAG = 'exo262'
OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/tool_organizer_exo258_%s_%s.png'
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
d = app.activeDocument
log('active %r version %s' % (d.name, d.dataFile.versionNumber if d.dataFile else None))
log('root occurrences: %s' % [o.component.name for o in design.rootComponent.occurrences])
vp = app.activeViewport
for tag, tgt, off, ext in (('top', (-7.425, 0, 2), (0, 0, 60), 23.0),):
    cam = vp.camera
    cam.isSmoothTransition = False
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    cam.target = P3(*tgt)
    cam.eye = P3(*(tgt[k] + off[k] for k in range(3)))
    cam.upVector = V3(0, 1, 0)
    cam.isFitView = False
    cam.viewExtents = ext
    vp.camera = cam
    adsk.doEvents()
    vp.refresh()
    screenshot(OUT % (TAG, tag))
