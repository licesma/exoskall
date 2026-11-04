# KNUTH-2 screenshots: iso overview of the whole design, and a close-up on the J2 module
# where link_23's riser passes link_12's stator slab. Touches only the camera.
import math
OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/'
vp = app.activeViewport
cam = vp.camera
cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
cam.isFitView = True
vp.camera = cam
adsk.doEvents(); vp.fit(); adsk.doEvents()
log('overview -> %s' % screenshot(OUT + 'generated_R_arm_knuth2_iso.png'))

# close-up: target = J2 module origin (cm), look from +X/-Y/+Z-ish so the riser side is seen
tgt = adsk.core.Point3D.create(0.001, -24.077, -0.282)
for tag, d in (('j2_front', (1.0, -0.9, 0.6)), ('j2_side', (0.2, -1.0, -0.4))):
    n = math.sqrt(sum(v * v for v in d))
    cam = vp.camera
    cam.isFitView = False
    cam.target = tgt
    cam.eye = adsk.core.Point3D.create(tgt.x + 40 * d[0] / n, tgt.y + 40 * d[1] / n, tgt.z + 40 * d[2] / n)
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.viewExtents = 12.0
    cam.isSmoothTransition = False
    vp.camera = cam
    adsk.doEvents()
    log('%s -> %s' % (tag, screenshot(OUT + 'generated_R_arm_knuth2_%s.png' % tag)))
