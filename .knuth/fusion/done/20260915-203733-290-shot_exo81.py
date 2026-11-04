# EXO-81 screenshots: the EXO-69/EXO-74 camera set aimed at link_23 (so exo83 and exo81 are
# comparable frame by frame) plus new cameras on link_12, which is now a routed profile L
# instead of a box bracket:
#   j1_front / j1_side  the plate leaving the J1 rotor pad - the join EXO-74 is about
#   j1_elbow            the elbow from below: the r 5.5 outer round and the r 3 inner fillet
#   j1_plan             straight down the J1 pad axis: the 25 mm band and the offset tongue
#   j1_slab             the J2 stator slab, its tongue and the two blends
#   j1_iso              link_12 on its own, three-quarter
#   j12_j23             both links together, the corridor EXO-3 is about
# Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo81.py --timeout 300
# Touches only the camera.
import math

TAG = 'exo81'
exec(open('/Users/esteban/repos/exo/tools/bridge_scripts/shot_exo58.py').read())
exec(open('/Users/esteban/repos/exo/tools/bridge_scripts/shot_exo74_rim.py').read())

OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/'
vp = app.activeViewport

# link_12 targets, in cm. The J1 module frame sits at world (0, -200.44, 0) with
# X = (1, 0, 0), Y = (0, 0.27564, 0.96126), Z = (0, -0.96126, 0.27564); the link frame's
# u is -X_A, so a point (s, w, z) is world (-s, -200.44 + 0.96126 z, 0.27564 z).
def P(s, w, z):
    return (-s / 10.0, (-200.44 - w + 0.96126 * z) / 10.0, (0.27564 * z) / 10.0)


J1_JOIN = P(26.0, 0.0, 4.0)      # the plate crossing the pad rim
J1_ELBOW = P(29.0, 0.0, 4.0)     # the elbow's two radii
J1_SLAB = P(28.5, 0.0, 22.0)     # the stator slab and the tongue
J1_MID = P(20.0, 0.0, 12.0)      # the whole link
Z_A = (0.0, -0.96126, 0.27564)   # the J1 pad axis, for the plan view
UP = (0, 0, 1)
views = (('j1_front', J1_JOIN, (-1.0, -0.9, 0.6), 40, 12.0, UP),
         ('j1_side', J1_JOIN, (-0.2, -1.0, -0.4), 40, 12.0, UP),
         ('j1_elbow', J1_ELBOW, (-0.9, -1.0, -0.8), 30, 8.0, UP),
         ('j1_slab', J1_SLAB, (-1.0, -0.6, 0.3), 40, 10.0, UP),
         ('j1_plan', J1_MID, Z_A, 40, 9.0, (-1.0, 0.0, 0.0)),
         ('j1_iso', J1_MID, (-1.0, -1.0, 0.7), 40, 12.0, UP),
         ('j12_j23', (-1.5, -22.0, -3.5), (1.0, -1.0, 0.5), 60, 18.0, UP))
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
