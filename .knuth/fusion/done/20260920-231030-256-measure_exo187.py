"""EXO-187 measurement: how much link material sits inside the XL330's own volume.

The XL330 is NOT a body in `sensor module 2` (`.knuth/docs/exo-joint.md`: 'Actuator | no'),
so `check_interference` has never been able to see a link sitting in its volume. This builds
the volume as a PROXY BODY under every module occurrence of `generated_R_arm_general` and
measures every link body against it, at a ladder of clearance margins, so the table says
which links foul it, by how much, and where in the module's own frame.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/measure_exo187.py --timeout 900
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

root = design.rootComponent
MM = 0.1

# the XL330's own box in the module frame (EXO-10 / exo-joint.md)
ACT_X = (-10.0, 10.0)
ACT_Y = (-9.5, 24.5)
ACT_Z = (-36.5, -10.5)
UNDERSIDE = armgen.STATOR_UNDERSIDE_Z          # -24.5

MARGINS = [0.0, 0.5, 1.0, 2.0, 3.0]

# ---- the frames the build used -------------------------------------------------------
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
log('frames rebuilt; J7 spin %.3f, tongue %.3f'
    % (spins['J7']['spin'], spins['J7'].get('tongue_deg', 0.0)))

# ---- the arm and its link bodies ------------------------------------------------------
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
log('link bodies: ' + ', '.join(sorted(link_bodies)))

# ---- a scratch component to hold the proxies -----------------------------------------
identity = adsk.core.Matrix3D.create()
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name.startswith('exo187_probe'):
        o.deleteMe()
probe_occ = root.occurrences.addNewComponent(identity)
probe_occ.component.name = 'exo187_probe'
pc = probe_occ.component


def make_box(label, m):
    """The actuator volume of module `label`, grown by `m` mm all round, as a body."""
    R, t = frames[label]
    F = BuildArm.Frame(R, [v * 1000.0 for v in t])
    x0, x1 = ACT_X[0] - m, ACT_X[1] + m
    y0, y1 = ACT_Y[0] - m, ACT_Y[1] + m
    z0, z1 = ACT_Z[0] - m, UNDERSIDE + m
    cz = (z0 + z1) / 2.0
    sk = BuildArm.PlaneSketch(pc, 'act %s %.1f' % (label, m),
                              F.point([0, 0, cz]), F.vector([1, 0, 0]), F.vector([0, 1, 0]))
    sk.loop(armgen.rect_outline(x0, y0, x1, y1))
    feat = sk.extrude(1, z1 - z0, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    b = feat.bodies.item(0)
    b.name = 'act_%s_m%.1f' % (label, m)
    return b, F


def b_coords(F, p_cm):
    d = [p_cm.x * 10.0 - F.t[0], p_cm.y * 10.0 - F.t[1], p_cm.z * 10.0 - F.t[2]]
    loc = armgen.matvec(armgen.transpose(F.R), d)
    return loc


log('')
log('EXO-187 measurement: link material inside the XL330 volume '
    '(x %.1f..%.1f, y %.1f..%.1f, z %.1f..%.1f in the module frame), '
    'at a ladder of clearance margins.'
    % (ACT_X[0], ACT_X[1], ACT_Y[0], ACT_Y[1], ACT_Z[0], UNDERSIDE))

rows = []
for m in MARGINS:
    boxes = {}
    for lab in ORDER:
        boxes[lab] = make_box(lab, m)
    for lname in sorted(link_bodies):
        for lab in ORDER:
            box, F = boxes[lab]
            coll = adsk.core.ObjectCollection.create()
            coll.add(link_bodies[lname])
            coll.add(box)
            inp = design.createInterferenceInput(coll)
            inp.areCoincidentFacesIncluded = False
            res = design.analyzeInterference(inp)
            if res is None or res.count == 0:
                continue
            for i in range(res.count):
                ib = res.item(i).interferenceBody
                if ib is None:
                    continue
                vol = ib.volume * 1000.0
                lo = [1e9] * 3
                hi = [-1e9] * 3
                for v in range(ib.vertices.count):
                    q = b_coords(F, ib.vertices.item(v).geometry)
                    for k in range(3):
                        lo[k] = min(lo[k], q[k])
                        hi[k] = max(hi[k], q[k])
                rows.append((m, lname, lab, vol, tuple(lo), tuple(hi)))
                log('  m %.1f  %-10s vs %-3s  %9.2f mm3   x %7.2f..%7.2f  y %7.2f..%7.2f  '
                    'z %7.2f..%7.2f' % (m, lname, lab, vol, lo[0], hi[0], lo[1], hi[1],
                                        lo[2], hi[2]))
    for lab in ORDER:
        boxes[lab][0].deleteMe()
    log('  --- margin %.1f done: %d hit(s) so far' % (m, len(rows)))

log('')
log('summary (mm3 inside the volume, by margin):')
log('  %-10s %s' % ('link', '  '.join('m%.1f' % m for m in MARGINS)))
for lname in sorted(link_bodies):
    cells = []
    for m in MARGINS:
        v = sum(r[3] for r in rows if r[0] == m and r[1] == lname)
        cells.append('%7.2f' % v)
    log('  %-10s %s' % (lname, '  '.join(cells)))
probe_occ.deleteMe()
log('done')
