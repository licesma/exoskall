"""EXO-187 measurement: how much link material sits inside the XL330's own volume.

The XL330 is NOT a body in `sensor module 2` (`.knuth/docs/exo-joint.md`: 'Actuator | no'),
so `check_interference` has never been able to see a link sitting in its volume. This builds
the volume as a PROXY BODY under every module occurrence of `generated_R_arm_general` and
measures every link body against it, at a ladder of clearance margins, so the table says
which links foul it, by how much, and where in the module's own frame.

One `analyzeInterference` call per margin over (all link bodies + all module boxes), not one
per pair: the pair is read back off `entityOne` / `entityTwo`, so the run is 5 calls, not 210.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/measure_exo187.py --timeout 600
"""
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

root = design.rootComponent

# the XL330's own box in the module frame (.knuth/docs/exo-joint.md, EXO-10)
ACT_X = (-10.0, 10.0)
ACT_Y = (-9.5, 24.5)
ACT_Z = (-36.5, -10.5)
UNDERSIDE = armgen.STATOR_UNDERSIDE_Z          # -24.5  (stator underside = plate top face)
PLATE_LO = UNDERSIDE - armgen.PAD_T            # -32.5  (plate far face)

MARGINS = [0.0, 0.5, 1.0, 2.0, 3.0]

# ---- the frames the build used ------------------------------------------------------
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
log('frames rebuilt; modules %s' % ', '.join(ORDER))

# ---- the arm and its link bodies ----------------------------------------------------
arm_occ = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name.startswith(BuildArmGen.ARM_NAME):
        arm_occ = o
if arm_occ is None:
    raise RuntimeError('no %s in this document' % BuildArmGen.ARM_NAME)
log('arm: ' + arm_occ.component.name)

link_bodies = {}
for i in range(arm_occ.childOccurrences.count):
    sub = arm_occ.childOccurrences.item(i)
    for k in range(sub.bRepBodies.count):
        b = sub.bRepBodies.item(k)
        if b.name.startswith('g_link_'):
            link_bodies[b.name] = b
log('link bodies (%d): %s' % (len(link_bodies), ', '.join(sorted(link_bodies))))

# ---- a scratch component to hold the proxies ----------------------------------------
identity = adsk.core.Matrix3D.create()
for i in range(root.occurrences.count - 1, -1, -1):
    o = root.occurrences.item(i)
    if o.component.name.startswith('exo187_probe'):
        o.deleteMe()
probe_occ = root.occurrences.addNewComponent(identity)
probe_occ.component.name = 'exo187_probe'
pc = probe_occ.component
PROBE = pc.name


def make_box(label, m):
    """The actuator volume of module `label`, grown by `m` mm all round, as a body.
    The top is capped at the stator underside: above that is the stator's own solid."""
    R, t = frames[label]
    F = BuildArm.Frame(R, [v * 1000.0 for v in t])
    x0, x1 = ACT_X[0] - m, ACT_X[1] + m
    y0, y1 = ACT_Y[0] - m, ACT_Y[1] + m
    z0, z1 = ACT_Z[0] - m, UNDERSIDE
    cz = (z0 + z1) / 2.0
    sk = BuildArm.PlaneSketch(pc, 'act %s %.1f' % (label, m),
                              F.point([0, 0, cz]), F.vector([1, 0, 0]), F.vector([0, 1, 0]))
    sk.loop(armgen.rect_outline(x0, y0, x1, y1))
    feat = sk.extrude(1, z1 - z0, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    b = feat.bodies.item(0)
    b.name = 'act_%s' % label
    return b, F


def b_coords(F, p_cm):
    d = [p_cm.x * 10.0 - F.t[0], p_cm.y * 10.0 - F.t[1], p_cm.z * 10.0 - F.t[2]]
    return armgen.matvec(armgen.transpose(F.R), d)


def where(lo, hi):
    """Which band of the landing the overlap is in, in the module's own frame."""
    if lo[2] >= PLATE_LO - 1e-6:
        return 'plate band'
    if hi[2] <= PLATE_LO + 1e-6:
        return 'below plate'
    return 'plate + below'


log('')
log('EXO-187: link material inside the XL330 volume, module frame '
    'x %.1f..%.1f  y %.1f..%.1f  z %.1f..%.1f (top capped at the stator underside %.1f), '
    'grown by each margin all round.'
    % (ACT_X[0], ACT_X[1], ACT_Y[0], ACT_Y[1], ACT_Z[0], ACT_Z[1], UNDERSIDE))
log('')

rows = []
for m in MARGINS:
    boxes = {}
    coll = adsk.core.ObjectCollection.create()
    for lab in ORDER:
        boxes[lab] = make_box(lab, m)
        coll.add(boxes[lab][0])
    for lname in sorted(link_bodies):
        coll.add(link_bodies[lname])
    inp = design.createInterferenceInput(coll)
    inp.areCoincidentFacesIncluded = False
    res = design.analyzeInterference(inp)
    n = 0 if res is None else res.count
    for i in range(n):
        r = res.item(i)
        a, b = r.entityOne, r.entityTwo
        la = '%s/%s' % (a.parentComponent.name, a.name)
        lb = '%s/%s' % (b.parentComponent.name, b.name)
        # keep only link-vs-box pairs
        pair = None
        for (p, q) in ((la, lb), (lb, la)):
            if p.split('/')[-1].startswith('g_link_') and q.startswith(PROBE + '/act_'):
                pair = (p.split('/')[-1], q.split('/act_')[-1])
        if pair is None:
            continue
        lname, lab = pair
        ib = r.interferenceBody
        if ib is None:
            rows.append((m, lname, lab, -1.0, None, None))
            log('  m %.1f  %-11s vs %-3s   (no interference body)' % (m, lname, lab))
            continue
        vol = ib.volume * 1000.0
        F = boxes[lab][1]
        lo, hi = [1e9] * 3, [-1e9] * 3
        for v in range(ib.vertices.count):
            q = b_coords(F, ib.vertices.item(v).geometry)
            for k in range(3):
                lo[k] = min(lo[k], q[k])
                hi[k] = max(hi[k], q[k])
        rows.append((m, lname, lab, vol, tuple(lo), tuple(hi)))
        log('  m %.1f  %-11s vs %-3s %10.2f mm3  x %7.2f..%7.2f y %7.2f..%7.2f '
            'z %7.2f..%7.2f  %s'
            % (m, lname, lab, vol, lo[0], hi[0], lo[1], hi[1], lo[2], hi[2], where(lo, hi)))
    for lab in ORDER:
        boxes[lab][0].deleteMe()
    log('  --- margin %.1f done: %d hit(s) cumulative' % (m, len(rows)))

log('')
log('summary: mm3 of link material inside the actuator volume, by margin')
log('  %-11s %s' % ('link', ' '.join('%9s' % ('m%.1f' % m) for m in MARGINS)))
for lname in sorted(link_bodies):
    cells = []
    for m in MARGINS:
        v = sum(r[3] for r in rows if r[0] == m and r[1] == lname and r[3] > 0)
        cells.append('%9.2f' % v)
    at = sorted(set(r[2] for r in rows if r[1] == lname))
    log('  %-11s %s   at %s' % (lname, ' '.join(cells), ', '.join(at) or '-'))
probe_occ.deleteMe()
log('done')
