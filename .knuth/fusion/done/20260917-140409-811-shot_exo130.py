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
# The face-on frames look straight down the STATOR MODULE'S AXIS at the slab. On every pair
# built so far that axis lies in the rotor pad's own plane, so this is also the view back
# along the link's reach: from the far side it is the module itself that is in the way, and
# from the near side the whole rest of the arm is. So the module occurrence at that joint is
# switched OFF for those two frames and switched straight back on - the `_iso` frame and the
# arm view have it on. Nothing else in the document is touched.
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

    # the slab's own axes in the pad frame's (s, w, z): its axis (the module axis), the
    # tongue direction and the one across the tongue
    R_s = geo['R_b']
    axes = []
    for c in range(3):
        v = [R_s[i][c] for i in range(3)]
        axes.append((armgen.dot(v, u), armgen.dot(v, w), v[2]))
    SX, SY, SZ = axes
    C = num['slab_centre']            # (s, w, z) of the slab disk's centre
    tgt = P(*C)
    # face on to the slab's FREE face - the one that bolts to module B is on the far side,
    # so this looks at the disk with nothing in front of it: the three inserts, the actuator
    # cutout and the run crossing them are all in this one frame
    eye = D(SZ[0], SZ[1], SZ[2])      # from beyond the slab, looking back down the reach
    up = D(SY[0], SY[1], SY[2])       # the tongue points up the frame
    rings = [h for h in geo['hole_rings'] if h['pad'] == 'stator']
    tight = min(rings, key=lambda h: h['ring'])
    jb = next(o for o in arm.childOccurrences
              if o.component.name.startswith(L['b'] + '_'))
    was_on = jb.isLightBulbOn
    jb.isLightBulbOn = False
    adsk.doEvents()
    try:
        # the landing itself: the run crossing the slab and the three inserts round it
        shoot('%s_land_face' % L['name'], tgt, eye, 16, 8.5, up)
        # and the insert this landing comes closest to, close up
        shoot('%s_insert_%.0fdeg' % (L['name'], tight['angle']),
              P(*tight['at']), eye, 12, 3.5, up)
    finally:
        jb.isLightBulbOn = was_on
        adsk.doEvents()
    # the same landing in place, module and all
    shoot('%s_land_iso' % L['name'], tgt,
          D(0.55 * SZ[0] + 0.55 * SX[0] + 0.62 * SY[0],
            0.55 * SZ[1] + 0.55 * SX[1] + 0.62 * SY[1],
            0.55 * SZ[2] + 0.55 * SX[2] + 0.62 * SY[2]), 22, 11.0, up)
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
