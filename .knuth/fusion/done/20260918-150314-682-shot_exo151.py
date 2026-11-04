# EXO-151 screenshots (the EXO-150 spin rule plus the default rounding): each link of the GENERAL arm, now spun by the encoder rule (+Y, the
# encoder / connector side, faces the direction the link arrives from at that stator pad).
# Two link views as in EXO-144, plus a THIRD view per link looking straight down the ribbon's
# own approach at the stator landing plate, which is where the rule shows: the plate's tongue
# and the module's encoder overhang point the same way.
# All three arms are visible so Esteban can overlay them - the hand-made `right arm`, the
# current generator's `generated_R_arm`, and `generated_R_arm_general`.
#
# Every camera is on a station of the link's OWN frame (the mid point between the two joint
# origins, looking along two directions of the rotor-pad frame), so the two views of a link
# are comparable and so is any later run of this script.
#
# Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo144.py --timeout 420
import math
import sys
import importlib

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
import sweepgen
import BuildArm
importlib.reload(armgen)
importlib.reload(sweepgen)
importlib.reload(BuildArm)

OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/'
TAG = 'exo151'
SHOW = ('right arm', 'generated_R_arm', 'generated_R_arm_general')
JUNK = ('PROBE_', 'DBG_')
vp = app.activeViewport
root = design.rootComponent

for i in range(root.occurrences.count - 1, -1, -1):
    o = root.occurrences.item(i)
    if any(o.component.name.startswith(j) for j in JUNK):
        log('deleted scratch component %s' % o.component.name)
        o.deleteMe()
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    o.isLightBulbOn = o.component.name in SHOW
log('visible: %s' % ', '.join(root.occurrences.item(i).component.name
                              for i in range(root.occurrences.count)
                              if root.occurrences.item(i).isLightBulbOn))

joints, by_child = armgen.parse_urdf(BuildArm.URDF_PATH)
shift, _ = armgen.joint_world_pose(BuildArm.ORIGIN_AT_JOINT, joints, by_child)
seed = {}
for j in BuildArm.JOINTS:
    R, t = armgen.module_frame(j['name'], joints, by_child, j['axis_sign'], j['y_hint'])
    seed[j['label']] = (R, [t[i] - shift[i] for i in range(3)])
LINKS = [(L['name'], L['a'], L['b'], L['pad_r'], L['width']) for L in BuildArm.LINKS]
frames, spins = sweepgen.solve_spins(seed, LINKS)
log('module rotations (EXO-150 encoder rule): %s'
    % ', '.join('%s %+.3f' % (k, spins[k]['spin']) for k in sorted(spins)))


def _save(path, name):
    """Save at the VIEWPORT's own aspect ratio. The bridge's `screenshot` helper asks for a
    fixed 1600 x 1000 and Fusion then renders the view into a frame of a different shape, so
    the camera target lands off centre and a third of the image is empty."""
    h = int(round(1600.0 * vp.height / vp.width))
    ok = vp.saveAsImageFile(path, 1600, h)
    log('%s -> %s (1600x%d, %s)' % (name, path, h, ok))


# Measured on this document: the rendered frame is not centred on `camera.target` - the
# subject lands about a quarter of the frame width to the RIGHT. `SHIFT` moves the target
# back along the screen-right direction by that fraction of the view width.
SHIFT = 0.25


def shoot(name, tgt_mm, d_world, dist_cm, ext_cm, up, shift=None):
    n = math.sqrt(sum(v * v for v in d_world))
    fwd = [-v for v in d_world]                     # eye -> target
    right = armgen.unit(armgen.cross(fwd, up))
    wide = ext_cm * 9.5                             # frame width in mm, measured
    sh = SHIFT if shift is None else shift
    tgt_mm = [tgt_mm[i] + right[i] * sh * wide for i in range(3)]
    t = adsk.core.Point3D.create(tgt_mm[0] / 10.0, tgt_mm[1] / 10.0, tgt_mm[2] / 10.0)
    cam = vp.camera
    cam.isFitView = False
    cam.target = t
    cam.eye = adsk.core.Point3D.create(t.x + dist_cm * d_world[0] / n,
                                       t.y + dist_cm * d_world[1] / n,
                                       t.z + dist_cm * d_world[2] / n)
    cam.upVector = adsk.core.Vector3D.create(*up)
    cam.viewExtents = ext_cm
    cam.isSmoothTransition = False
    vp.camera = cam
    adsk.doEvents()
    # Fusion re-fits the first assignment on this document; assigning the same camera a
    # second time is what actually leaves the target where it was asked for (measured).
    vp.camera = cam
    adsk.doEvents()
    _save(OUT + 'generated_R_arm_%s_%s.png' % (TAG, name), name)


LANDINGS = []
for name, a, b, pad_r, width in LINKS:
    RA, tA = frames[a]
    RB, tB = frames[b]
    R_ab, t_ab = armgen.relative_frame(RA, tA, RB, tB)
    route = sweepgen.sweep_route(R_ab, t_ab, pad_r, width)
    u, w = route['u'], route['w']
    pa = [v * 1000.0 for v in tA]
    pb = [v * 1000.0 for v in tB]
    tgt = [(pa[i] + pb[i]) / 2.0 for i in range(3)]
    span = math.sqrt(sum((pb[i] - pa[i]) ** 2 for i in range(3))) + 90.0

    def W(s, ww, z):
        """A direction of the rotor-pad frame (s along u, w, z) in world."""
        loc = [u[i] * s + w[i] * ww + (z if i == 2 else 0.0) for i in range(3)]
        return armgen.matvec(RA, loc)

    up = W(0.0, 0.0, 1.0)
    # viewExtents measured on this document: about 0.6 x the frame height in cm, so
    # span/6.5 puts the whole link plus both its modules in the frame.
    shoot('%s_a' % name, tgt, W(0.5, -1.0, 0.6), 80, span / 9.0, up)
    shoot('%s_b' % name, tgt, W(0.0, 0.0, 1.0), 80, span / 9.0, W(1.0, 0.0, 0.0))
    # aim between the plate's disk centre and the point the ribbon lands on it
    sc = route['numbers']['slab_centre']
    def P(sv, wv, zv):
        loc = [u[i] * sv + w[i] * wv + (zv if i == 2 else 0.0) for i in range(3)]
        return [armgen.matvec(RA, loc)[i] + pa[i] for i in range(3)]
    land = P(sc[0], sc[1], route['z_arr'])
    tgt_l = [(land[i] + pb[i]) / 2.0 for i in range(3)]
    # SIDEWAYS, along the route's own w: the module's encoder overhang stands out of the
    # cylinder on one side and the plate's tongue runs out under it on the SAME side, which
    # is what the rule asserts. Looking down the ribbon instead only shows the plate, which
    # is the module's own bottom cover and hides everything behind it.
    LANDINGS.append((name + '_landing', tgt_l, W(1.0, -0.6, 0.35),
                     W(0.0, 0.0, route['numbers']['y_sign']), 9.0))
    # EXO-151: the rotor-pad end, where the ribbon's walls run out through the r pad_r rim
    # and the inner round r 3 blends them. link_12 / 34 are tangent to the rim and have none.
    LANDINGS.append((name + '_pad', P(0.0, 0.0, 4.0), W(-0.4, -0.9, 0.6),
                     W(0.0, 0.0, 1.0), 5.5))

# EXO-150: the landing, seen from where the ribbon comes in, with ONLY the general arm and
# its modules visible - the other two arms sit on top of this view and hide it. `y_sign` is
# the side the link arrives from, so putting it UP in frame shows the plate's tongue and the
# module's encoder overhang together, which is the whole point of the rule.
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    o.isLightBulbOn = o.component.name == 'generated_R_arm_general'
for name, tg, d, up, ext in LANDINGS:
    shoot(name, tg, d, 90, ext, up, shift=0.0)
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    o.isLightBulbOn = o.component.name in SHOW

cam = vp.camera
cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
cam.isFitView = False
cam.target = adsk.core.Point3D.create(4.0, -22.0, -10.0)
cam.eye = adsk.core.Point3D.create(4.0 + 90, -22.0 - 90, -10.0 + 60)
cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
cam.viewExtents = 46.0
vp.camera = cam
adsk.doEvents()
_save(OUT + 'generated_R_arm_%s_arm_iso.png' % TAG, 'arm_iso')
