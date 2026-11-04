"""EXO-204: verify the generated J1 link stop IN FUSION, and photograph the three poses.

The rule is checked in pure Python (`armgen.stop_sweep`, `tools/test_armgen.py`). This is
the same claim against the built solid: rotate the `g_link_12` occurrence about the J1 axis
and look for a real interference with the J1 module. First contact is BRACKETED - a quarter
of a degree inside each limit must be clear, a quarter of a degree outside must interfere -
which pins it to better than the 0.5 deg the issue asks for.

Then three screenshots, TAG exo204: J1 at `lower`, at zero, at `upper`, with everything
distal to J1 carried round so the arm reads as a pose and not as a broken link.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/shot_exo204.py --timeout 600
"""
import math
import sys
import importlib

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
import sweepgen
import BuildArm
import BuildArmGen
importlib.reload(armgen)
importlib.reload(sweepgen)
importlib.reload(BuildArm)
importlib.reload(BuildArmGen)

OUT = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/'
TAG = 'exo204'
GEN = 'generated_R_arm_general'
BRACKET = math.radians(0.25)          # half of the 0.5 deg the issue allows

vp = app.activeViewport
root = design.rootComponent

arm = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    o.isLightBulbOn = (o.component.name == GEN)
    if o.component.name == GEN:
        arm = o
if arm is None:
    raise RuntimeError('%s is not in this document' % GEN)


def bulbs_on(occ):
    n = 0
    for i in range(occ.childOccurrences.count):
        c = occ.childOccurrences.item(i)
        if not c.isLightBulbOn:
            c.isLightBulbOn = True
            n += 1
        n += bulbs_on(c)
    return n


log('switched %d occurrence light bulb(s) on under %s' % (bulbs_on(arm), GEN))

# ---- the frames, rebuilt exactly as the builder does ----
joints, by_child = armgen.parse_urdf(BuildArm.URDF_PATH)
shift, _ = armgen.joint_world_pose(BuildArm.ORIGIN_AT_JOINT, joints, by_child)
LINKS = [(L['name'], L['a'], L['b'], L['pad_r'], L['width'])
         for L in list(BuildArm.LINKS) + BuildArmGen.NEW_LINKS]
ORDER = ([j['label'] for j in BuildArm.JOINTS]
         + [j['label'] for j in BuildArmGen.NEW_JOINTS])
_s, _l, table = BuildArmGen.seed_frames(joints, by_child, shift)
seed, _combo, SLIDES, _rows = BuildArmGen.solve_axis_signs(joints, by_child, shift, LINKS,
                                                           table, lambda s: None, ORDER)
frames, spins = sweepgen.solve_spins(seed, LINKS)

J1 = BuildArm.JOINTS[0]
LJ = armgen.limited_joint(joints[J1['name']], J1['axis_sign'], BuildArm.LINKS[0]['pad_r'])
log('EXO-204 rule: %s, axis_sign %+d, block %.5f deg +/- %.5f, r %.1f..%.1f x %.1f mm'
    % (LJ['joint'], LJ['axis_sign'], LJ['stop_angle'], LJ['stop_half'], LJ['r_in'],
       LJ['r_out'], LJ['stop_h']))
_lo, _hi, _n = armgen.stop_sweep(LJ)
log('EXO-204 pure-Python sweep: %d poses, first contact %+.6f / %+.6f deg, URDF %+.6f / '
    '%+.6f, error %.1e / %.1e deg'
    % (_n, math.degrees(_lo), math.degrees(_hi), math.degrees(LJ['lower']),
       math.degrees(LJ['upper']), abs(math.degrees(_lo - LJ['lower'])),
       abs(math.degrees(_hi - LJ['upper']))))

R1, t1 = frames['J1']
AXIS = [R1[i][2] for i in range(3)]                     # module +Z, world
ORIG = [v * 100.0 for v in t1]                          # world cm (frames are metres)
# the module's +Z is axis_sign * the URDF axis, so a URDF q is axis_sign * q about +Z


def pose_matrix(q):
    """World rotation of `q` rad about the J1 URDF axis, through the J1 origin."""
    m = adsk.core.Matrix3D.create()
    m.setToRotation(LJ['axis_sign'] * q,
                    adsk.core.Vector3D.create(*AXIS),
                    adsk.core.Point3D.create(*ORIG))
    return m


kids = {}
for i in range(arm.childOccurrences.count):
    c = arm.childOccurrences.item(i)
    kids[c.component.name] = c
log('arm children: %s' % ', '.join(sorted(kids)))

LINK12 = 'g_link_12'
MODJ1 = [k for k in kids if k.startswith('GJ1_')][0]
DISTAL = [k for k in kids if k != MODJ1]                 # everything J1 carries


def place(names, q):
    m = pose_matrix(q)
    for nm in names:
        BuildArm.set_occ_matrix(kids[nm], m, arm)
    adsk.doEvents()


STATOR = 'Component1/'          # the module part that does NOT turn: it carries the joint stop
ROTOR = ('Component2/', 'Component3/')


def hits(a_name, b_name):
    """Link-vs-STATOR interferences only, slivers dropped.

    Two whole classes of hit have to be excluded or the answer is noise:
      * MODULE-INTERNAL (stator vs its own press-fit `m3_6mm_insert`s) - present at every
        pose, nothing to do with the link (EXO-201 counts 49 of them across the arm);
      * link vs the module's ROTOR. In the machine the rotor is BOLTED to the link and turns
        with it; only this script's rotation leaves it behind. It shows up the moment the
        link is turned off zero, because the link's index TAB lifts out of the rotor's notch
        and into the rotor's face disk - which is a nice independent demonstration that the
        tab keys the link to exactly one orientation (EXO-204's zero pose), and no more.
    What is left is the only pair that can be a mechanical limit: the link's stop block
    against the stator's."""
    bodies = adsk.core.ObjectCollection.create()
    for nm in (a_name, b_name):
        bs, _n = BuildArm.collect_bodies(kids[nm])
        for j in range(bs.count):
            bodies.add(bs.item(j))
    out, other = [], []
    for a, b, vol, bb in BuildArm.analyze_interference(design, bodies):
        if armgen.classify_interference(a, b, bb) != 'FAIL':
            continue
        pair = (a, b)
        link_side = [x for x in pair if x.startswith(a_name + '/')]
        stator_side = [x for x in pair if x.startswith(STATOR)]
        if link_side and stator_side:
            out.append((a, b, vol))
        else:
            other.append((a, b, vol))
    return out, other


log('')
log('--- EXO-204 in Fusion: bracketing first contact on the built solid ---')
rows = []
for what, q0 in (('lower', LJ['lower']), ('upper', LJ['upper'])):
    inside = q0 + BRACKET if what == 'lower' else q0 - BRACKET
    outside = q0 - BRACKET if what == 'lower' else q0 + BRACKET
    place([LINK12], inside)
    h_in, o_in = hits(LINK12, MODJ1)
    place([LINK12], outside)
    h_out, o_out = hits(LINK12, MODJ1)
    ok = (not h_in) and bool(h_out)
    rows.append(ok)
    log('  %s = %+.5f deg' % (what, math.degrees(q0)))
    log('      %+.5f deg (0.25 INSIDE) : %d link-vs-stator hit(s) %s  [+%d excluded]'
        % (math.degrees(inside), len(h_in), h_in[0][:2] if h_in else '', len(o_in)))
    log('      %+.5f deg (0.25 OUTSIDE): %d link-vs-stator hit(s) %s  [+%d excluded]'
        % (math.degrees(outside), len(h_out),
           '%s <-> %s %.3f mm3' % h_out[0] if h_out else '', len(o_out)))
    log('      %s' % ('PASS - first contact is between them' if ok else 'FAIL'))
place([LINK12], 0.0)
h0, o0 = hits(LINK12, MODJ1)
log('  zero pose: %d link-vs-stator hit(s) [+%d excluded: %s]   %s'
    % (len(h0), len(o0), ', '.join(sorted(set('%s<->%s' % (a, b) for a, b, _v in o0))),
       'PASS' if not h0 else 'FAIL'))
log('  => first contact is inside %+.3f .. %+.3f deg of each URDF limit: %s'
    % (-math.degrees(BRACKET), math.degrees(BRACKET),
       'ALL PASS' if all(rows) and not h0 else 'FAILED'))
log('')


def _save(path, name):
    h = int(round(1600.0 * vp.height / vp.width))
    ok = vp.saveAsImageFile(path, 1600, h)
    log('%s -> %s (1600x%d, %s)' % (name, path, h, ok))


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
    vp.camera = cam
    adsk.doEvents()
    _save(OUT + 'generated_R_arm_%s_%s.png' % (TAG, name), name)


xb = [R1[i][0] for i in range(3)]
yb = [R1[i][1] for i in range(3)]
zb = [R1[i][2] for i in range(3)]
p1 = [v * 1000.0 for v in t1]
p7 = [v * 1000.0 for v in frames['J7'][1]]
mid = [(p1[i] + p7[i]) / 2.0 for i in range(3)]
span = math.sqrt(sum((p7[i] - p1[i]) ** 2 for i in range(3))) + 160.0
# the two blocks live at z -4 .. 0 in the module frame; look straight up the axis at them
stop_tgt = [p1[i] + zb[i] * (-2.0) for i in range(3)]

for name, q in (('lower', LJ['lower']), ('zero', 0.0), ('upper', LJ['upper'])):
    place(DISTAL, q)
    shoot('J1_%s_stops' % name, stop_tgt, [-v for v in zb], 40, 4.5, yb)
    shoot('J1_%s_arm' % name, mid, [1.0, -1.0, 0.7], 150, span / 10.0, [0.0, 0.0, 1.0])

place(DISTAL, 0.0)
log('restored every occurrence to the zero pose')
log('EXO-204 screenshots done')
