# EXO-58 screenshots: the same three views as shot_knuth2.py (iso overview, J2 front, J2
# side) plus a close-up of link_23's bend from below the bar. Touches only the camera.
import math
OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/'
vp = app.activeViewport
cam = vp.camera
cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
cam.isFitView = True
vp.camera = cam
adsk.doEvents(); vp.fit(); adsk.doEvents()
log('overview -> %s' % screenshot(OUT + 'generated_R_arm_exo58_iso.png'))

# close-ups: target = J2 module origin (cm); directions as in shot_knuth2 plus 'j2_bend'
tgt = adsk.core.Point3D.create(0.001, -24.077, -0.282)
views = (('j2_front', (1.0, -0.9, 0.6), 40, 12.0),
         ('j2_side', (0.2, -1.0, -0.4), 40, 12.0),
         ('j2_bend', (0.9, -1.0, -0.8), 30, 8.0))
for tag, d, dist, ext in views:
    n = math.sqrt(sum(v * v for v in d))
    cam = vp.camera
    cam.isFitView = False
    cam.target = tgt
    cam.eye = adsk.core.Point3D.create(tgt.x + dist * d[0] / n, tgt.y + dist * d[1] / n, tgt.z + dist * d[2] / n)
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.viewExtents = ext
    cam.isSmoothTransition = False
    vp.camera = cam
    adsk.doEvents()
    log('%s -> %s' % (tag, screenshot(OUT + 'generated_R_arm_exo58_%s.png' % tag)))
