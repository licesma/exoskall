# EXO-58 screenshots: iso overview, J2 front, J2 side, plus a close-up of link_23's bend
# from below the bar. EXO-64 adds the same close-up directions aimed at the OTHER join (the
# J3 stator pad) and a PLAN view straight down the J2 pad axis, where the riser S is the
# shape to look at. TAG selects the file-name prefix so an earlier set is never overwritten.
# Touches only the camera.
import math
OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/'
# EXO-69: a caller may predefine TAG (see shot_exo69_riser.py / shot_exo69_bar.py) so
# the two S-placement options get the same views under different file-name prefixes.
try:
    TAG
except NameError:
    TAG = 'exo64'
vp = app.activeViewport
cam = vp.camera
cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
cam.isFitView = True
vp.camera = cam
adsk.doEvents(); vp.fit(); adsk.doEvents()
log('overview -> %s' % screenshot(OUT + 'generated_R_arm_%s_iso.png' % TAG))

# Targets in cm. J2 = the rotor-pad join; J3_JOIN = where the bar meets the J3 stator slab
# (J2 frame (70.7, -6.24, -35.35)); S_MID = the middle of the riser S (J2 frame (37.4, -3.1, -20)).
J2 = (0.001, -24.077, -0.282)
J3_JOIN = (-3.534, -24.701, -7.352)
S_MID = (-2.0, -24.38, -3.98)
BAR_S_MID = (-2.0, -24.389, -6.186)   # middle of the EXO-69 bar-placed S (s = 59.04)
# (tag, target, eye direction, distance cm, view extent cm, up vector)
UP = (0, 0, 1)
views = (('j2_front', J2, (1.0, -0.9, 0.6), 40, 12.0, UP),        # EXO-58 angles, J2 join
         ('j2_side', J2, (0.2, -1.0, -0.4), 40, 12.0, UP),
         ('j2_bend', J2, (0.9, -1.0, -0.8), 30, 8.0, UP),
         ('j3_front', J3_JOIN, (1.0, -0.9, 0.6), 40, 10.0, UP),   # same angles, J3 join
         ('j3_side', J3_JOIN, (0.2, -1.0, -0.4), 40, 10.0, UP),
         ('j3_bend', J3_JOIN, (0.9, -1.0, -0.8), 30, 7.0, UP),
         # plan view: the J2 pad axis is world +X, so look along -X with the bar running up
         ('s_plan', S_MID, (1.0, 0.0, 0.0), 40, 7.0, (0, 0, -1)),
         ('s_plan_wide', J2, (1.0, 0.0, 0.0), 60, 13.0, (0, 0, -1)),
         # EXO-69: the same plan view aimed at the middle of a BAR-placed S
         # (J2 frame (59.04, -3.12, -20)); taken for both options, same camera
         ('s_plan_bar', BAR_S_MID, (1.0, 0.0, 0.0), 40, 7.0, (0, 0, -1)))
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
