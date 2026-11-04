"""EXO-261/262: screenshots of the example bearing + XL330 bins in tool_organizer_exo258.
Ortho cameras: a top view of column 0 (both bins), then one view per bin tilted 23 deg
from vertical (steeper than the 31 deg at which the 78 mm walls start hiding the floor).
TAG env-free: edit TAG below. Run via tools/fusion_run.py.
"""
TAG = 'exo261'
OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/tool_organizer_exo258_%s_%s.png'
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
d = app.activeDocument
log('active %r version %s' % (d.name, d.dataFile.versionNumber if d.dataFile else None))
log('root occurrences: %s' % [o.component.name for o in design.rootComponent.occurrences])
vp = app.activeViewport
for tag, tgt, off, ext in (('top', (-7.425, 0, 2), (0, 0, 60), 23.0),
                           ('bearing_bin', (-7.425, 6.6, 2), (-12, -17, 50), 10.0),
                           ('xl330_bin', (-7.425, -6.6, 2), (-12, -17, 50), 10.0)):
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
