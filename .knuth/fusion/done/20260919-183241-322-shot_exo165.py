"""EXO-165 screenshots: the hand-made wrist - Module 5 R / 6 R, Link 5 R / 6 R.

Two views of each subject, plus the pair that EXO-163 says cannot fit: J6 and J7,
46.00 mm apart on perpendicular axes, which the hand builder fits by sliding
`Module 5 R` 23.000 mm ALONG ITS OWN AXIS.  Nothing is built or moved - only the
light bulbs of the existing occurrences are toggled, and restored at the end.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo165.py --timeout 300
"""
import math

OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/'
TAG = 'exo165'
vp = app.activeViewport
root = design.rootComponent

# exo world positions (mm), measured from exo_skeleton_2026-08-08.step
J5 = (122.023, -242.648, -196.509)
J6 = (160.023, -219.648, -196.508)
J7 = (206.023, -242.648, -196.509)
WRIST = [(J5[i] + J7[i]) / 2.0 for i in range(3)]

WANTED = ('Link 5 R', 'Link 6 R', 'Module 4 R', 'Module 5 R', 'Module 6 R')

saved = []
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    saved.append((o, o.isLightBulbOn))
    o.isLightBulbOn = o.component.name == 'right arm'
arm = [root.occurrences.item(i) for i in range(root.occurrences.count)
       if root.occurrences.item(i).component.name == 'right arm'][0]
kids = []
for j in range(arm.childOccurrences.count):
    c = arm.childOccurrences.item(j)
    kids.append((c, c.isLightBulbOn))


def show(names):
    for c, _was in kids:
        c.isLightBulbOn = c.component.name in names


def save(name):
    h = int(round(1600.0 * vp.height / vp.width))
    path = OUT + 'exo_wrist_%s_%s.png' % (TAG, name)
    ok = vp.saveAsImageFile(path, 1600, h)
    log('%-22s -> %s (1600x%d, %s)' % (name, path, h, ok))


def shoot(name, tgt_mm, d, pad, up):
    """Frame whatever is lit from direction `d`. `isFitView` does the zoom: the
    subject changes size between shots and a fixed `viewExtents` cannot follow it.
    `pad` then backs the camera off by that factor so the part is not cropped."""
    n = math.sqrt(sum(v * v for v in d))
    t = adsk.core.Point3D.create(tgt_mm[0] / 10.0, tgt_mm[1] / 10.0, tgt_mm[2] / 10.0)
    cam = vp.camera
    cam.target = t
    cam.eye = adsk.core.Point3D.create(t.x + 80.0 * d[0] / n, t.y + 80.0 * d[1] / n,
                                       t.z + 80.0 * d[2] / n)
    cam.upVector = adsk.core.Vector3D.create(*up)
    cam.isSmoothTransition = False
    cam.isFitView = True
    vp.camera = cam
    adsk.doEvents()
    cam = vp.camera
    cam.isFitView = False
    cam.viewExtents = cam.viewExtents * pad
    vp.camera = cam
    adsk.doEvents()
    save(name)


# 1. the whole wrist, two views: along the J6 axis (the slide is edge on) and from above
show(WANTED)
shoot('wrist_iso', WRIST, (0.55, -0.75, 0.38), 1.35, (0, 0, 1))
shoot('wrist_alongJ6', WRIST, (0.0, 1.0, 0.0), 1.35, (0, 0, 1))
shoot('wrist_top', WRIST, (0.0, 0.0, 1.0), 1.35, (1, 0, 0))

# 2. Link 5 R (J5 rotor pad -> J6 stator), two views
show(('Link 5 R', 'Module 4 R', 'Module 5 R'))
mid56 = [(J5[i] + J6[i]) / 2.0 for i in range(3)]
shoot('link5_iso', mid56, (0.5, -0.8, 0.45), 1.35, (0, 0, 1))
shoot('link5_side', mid56, (0.0, 0.0, 1.0), 1.35, (1, 0, 0))
show(('Link 5 R',))
shoot('link5_alone', mid56, (0.4, -0.9, 0.3), 1.35, (0, 0, 1))

# 3. Link 6 R (J6 rotor pad -> J7 stator), two views
show(('Link 6 R', 'Module 5 R', 'Module 6 R'))
mid67 = [(J6[i] + J7[i]) / 2.0 for i in range(3)]
shoot('link6_iso', mid67, (0.45, -0.8, 0.5), 1.35, (0, 0, 1))
shoot('link6_alongJ6', mid67, (0.0, 1.0, 0.0), 1.35, (0, 0, 1))
show(('Link 6 R',))
shoot('link6_alone', mid67, (0.4, -0.85, 0.35), 1.35, (0, 0, 1))

# 4. the pair EXO-163 refuses: the two modules only, and the 23 mm slide that clears them
show(('Module 5 R', 'Module 6 R'))
shoot('modules_J6_J7', mid67, (0.0, 0.0, 1.0), 1.35, (1, 0, 0))
shoot('modules_slide', mid67, (0.0, -1.0, 0.0), 1.35, (0, 0, 1))

for c, was in kids:
    c.isLightBulbOn = was
for o, was in saved:
    o.isLightBulbOn = was
log('done')
