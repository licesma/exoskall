# EXO-144 screenshots: each link of the GENERAL arm from two views, with all three arms
# visible so Esteban can overlay them - the hand-made `right arm`, the current generator's
# `generated_R_arm`, and `generated_R_arm_general` (one swept ribbon + one landing plate).
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
TAG = 'exo144'
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
log('module rotations: %s' % ', '.join('%s %+.3f' % (k, spins[k]['spin'])
                                       for k in sorted(spins)))


def shoot(name, tgt_mm, d_world, dist_cm, ext_cm, up):
    n = math.sqrt(sum(v * v for v in d_world))
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
    log('%s -> %s' % (name, screenshot(OUT + 'generated_R_arm_%s_%s.png' % (TAG, name))))


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

cam = vp.camera
cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
cam.isFitView = False
cam.target = adsk.core.Point3D.create(4.0, -22.0, -10.0)
cam.eye = adsk.core.Point3D.create(4.0 + 90, -22.0 - 90, -10.0 + 60)
cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
cam.viewExtents = 46.0
vp.camera = cam
adsk.doEvents()
log('arm_iso -> %s' % screenshot(OUT + 'generated_R_arm_%s_arm_iso.png' % TAG))
