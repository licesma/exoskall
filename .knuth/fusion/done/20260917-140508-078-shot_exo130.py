# EXO-130 screenshots: the four LANDINGS, one set per link, after each module was turned to
# its solved rotation about its own axis. What to look at is the same thing in all four
# frames - the run crossing the stator slab, the three heat-set inserts around it, and
# whether the run's land fillet comes anywhere near one. link_45 is the one that changed:
# its 270 deg insert used to sit dead under the middle of the landing (which is why EXO-124
# had to shrink the land fillet to r 0.150) and now sits 45 deg off it, in the gap, with the
# fillet back at the measured r 5.000.
#
# Every camera is on a FIXED station of the link's own frame - the stator slab's centre and
# its axis, taken from the route - so the four landings are directly comparable, and so is
# any later run of this script.
#
# The cameras are the EXO-114 landing directions - oblique to the reach, because the stator
# axis lies in the rotor pad's own plane on every pair built so far, so a view straight down
# it looks through the whole rest of the document. Nothing but the camera is touched.
#
# Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo130.py --timeout 420
import math
import sys
import importlib

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
import BuildArm
importlib.reload(armgen)
importlib.reload(BuildArm)

OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/'
TAG = 'exo130'
vp = app.activeViewport
root = design.rootComponent

arm = next(o for o in root.occurrences if o.component.name.startswith('generated_R_arm'))

joints, by_child = armgen.parse_urdf(BuildArm.URDF_PATH)
shift, _ = armgen.joint_world_pose(BuildArm.ORIGIN_AT_JOINT, joints, by_child)
FR, SPINS, CHAIN = BuildArm.arm_frames(joints, by_child, shift)
log('module rotations: %s'
    % ', '.join('%s %+.3f deg' % (j['label'], SPINS[j['label']]['spin'])
                for j in BuildArm.JOINTS))


def shoot(name, tgt, d, dist, ext, up):
    n = math.sqrt(sum(v * v for v in d))
    t = adsk.core.Point3D.create(*tgt)
    cam = vp.camera
    cam.isFitView = False
    cam.target = t
    cam.eye = adsk.core.Point3D.create(t.x + dist * d[0] / n, t.y + dist * d[1] / n,
                                       t.z + dist * d[2] / n)
    cam.upVector = adsk.core.Vector3D.create(*up)
    cam.viewExtents = ext
    cam.isSmoothTransition = False
    vp.camera = cam
    adsk.doEvents()
    log('%s -> %s' % (name, screenshot(OUT + 'generated_R_arm_%s_%s.png' % (TAG, name))))


for L in BuildArm.LINKS:
    spec = armgen.route(FR[L['a']], FR[L['b']], L['pad_r'], width=L['width'])
    geo, num, land = spec['geometry'], spec['numbers'], spec['route']['stator_land']
    u, w = geo['u'], geo['w']
    ja = next(o for o in arm.childOccurrences
              if o.component.name.startswith(L['a'] + '_'))
    T = ja.transform

    def P(s, ww, z):
        """World point (cm) of the rotor-pad-frame station (s, w, z) in mm."""
        q = adsk.core.Point3D.create((u[0] * s + w[0] * ww) / 10.0,
                                     (u[1] * s + w[1] * ww) / 10.0, z / 10.0)
        q.transformBy(T)
        return (q.x, q.y, q.z)

    def D(s, ww, z):
        v = adsk.core.Vector3D.create(u[0] * s + w[0] * ww, u[1] * s + w[1] * ww, z)
        v.transformBy(T)
        return (v.x, v.y, v.z)

    C = num['slab_centre']            # (s, w, z) of the slab disk's centre
    tgt = P(*C)
    rings = [h for h in geo['hole_rings'] if h['pad'] == 'stator']
    tight = min(rings, key=lambda h: h['ring'])
    UP = D(0.0, 0.0, 1.0)
    # (name, target, eye direction in the LINK frame, distance cm, view extent cm)
    for nm, t, d, dist, ext in (
            ('%s_land_front' % L['name'], tgt, D(0.9, -0.7, 0.6), 45, 8.0),
            ('%s_land_side' % L['name'], tgt, D(0.0, -1.0, 0.0), 45, 8.0),
            ('%s_insert_%.0fdeg' % (L['name'], tight['angle']), P(*tight['at']),
             D(0.6, -0.9, 0.4), 40, 3.5)):
        shoot(nm, t, d, dist, ext, UP)
    log('   %s: %s route, land fillet r %.3f (nominal %.3f), corner r %.3f, tightest stator '
        'insert %.0f deg ring %.3f mm bound by %s'
        % (L['name'], spec['pattern'], land['fillet_r'], land['fillet_r_nominal'],
           land['corner_r'], tight['angle'], tight['ring'], tight['bound']))

cam = vp.camera
cam.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
cam.isFitView = True
vp.camera = cam
adsk.doEvents(); vp.fit(); adsk.doEvents()
log('arm_iso -> %s' % screenshot(OUT + 'generated_R_arm_%s_arm_iso.png' % TAG))
