# EXO-74: close-ups of the J2 rotor-pad rim join - the pocket that was filled. Three
# cameras, all aimed at the rim where the band leaves the pad (J2 frame (27, 0, 4) mm):
#   j2_rim       three-quarter from outside and above the pad's outer face
#   j2_rim_side  true side elevation along the band width: the join's silhouette
#   j2_rim_plan  straight down the pad axis: the outer face across the rim
# Exec'd by shot_exo74_riser.py / shot_exo74_bar.py after the EXO-69 camera set, so TAG is
# already defined. Touches only the camera.
import math
OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/'
vp = app.activeViewport
# J2 origin in world (cm) and the J2 frame's axes in world: pad axis +X, band width +Y,
# bar direction -Z (checked against the J3_JOIN target in shot_exo58.py).
J2 = (0.001, -24.077, -0.282)
RIM = tuple(J2[i] + (0.4, 0.0, -2.7)[i] for i in range(3))     # J2 frame (27, 0, 4) mm
# the eye stays on -Y: the J1 module (axis nearly world -Y) hides the join from +Y
views = (('j2_rim', RIM, (0.35, -0.3, -1.0), 20, 4.0, (1, 0, 0)),
         ('j2_rim_side', RIM, (0.15, -1.0, -0.1), 20, 4.5, (1, 0, 0)),
         ('j2_rim_plan', RIM, (1.0, 0.0, 0.0), 25, 4.0, (0, 0, -1)))
for tag, tgt_cm, d, dist, ext, up in views:
    n = math.sqrt(sum(v * v for v in d))
    tgt = adsk.core.Point3D.create(*tgt_cm)
    cam = vp.camera
    cam.isFitView = False
    cam.target = tgt
    cam.eye = adsk.core.Point3D.create(tgt.x + dist * d[0] / n, tgt.y + dist * d[1] / n,
                                       tgt.z + dist * d[2] / n)
    cam.upVector = adsk.core.Vector3D.create(*up)
    cam.viewExtents = ext
    cam.isSmoothTransition = False
    vp.camera = cam
    adsk.doEvents()
    log('%s -> %s' % (tag, screenshot(OUT + 'generated_R_arm_%s_%s.png' % (TAG, tag))))
