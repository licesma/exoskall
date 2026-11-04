"""EXO-208: bracket first contact in FUSION, on the built solids, for every limited joint
that a link carries (J1..J6).

The rule is checked in pure Python (`armgen.stop_sweep`, `tools/test_armgen.py`). This is
the same claim against the geometry: rotate the link occurrence about its rotor joint's axis
and look for a real link-vs-STATOR interference. A quarter of a degree INSIDE each URDF
limit must be clear; a quarter of a degree OUTSIDE must interfere. That brackets first
contact to +/-0.25 deg, better than the 0.5 the issue asks for.

J7 is not here and cannot be: the link stop lives on the rotor pad of the link that LEAVES
the joint, and J7 is the last joint of the chain (LINKS run link_12 .. link_67).

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/bracket_exo208.py --timeout 900
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

GEN = 'generated_R_arm_general'
BRACKET = math.radians(0.25)

root = design.rootComponent
arm = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name == GEN:
        arm = o
if arm is None:
    raise RuntimeError('%s is not in this document' % GEN)

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
JOINTS_ALL = list(BuildArm.JOINTS) + [dict(j, y_hint=table[j['label']]['y_hint'],
                                           axis_sign=1) for j in BuildArmGen.NEW_JOINTS]
for j in JOINTS_ALL[len(BuildArm.JOINTS):]:
    _t, ax = armgen.joint_world_pose(j['name'], joints, by_child)
    z = [seed[j['label']][0][i][2] for i in range(3)]
    j['axis_sign'] = 1 if armgen.dot(z, ax) > 0 else -1
by_label = {j['label']: j for j in JOINTS_ALL}

kids = {}
for i in range(arm.childOccurrences.count):
    c = arm.childOccurrences.item(i)
    kids[c.component.name] = c


def wrapper_of(lab):
    return [k for k in kids if k.startswith('G%s_' % lab)][0]


def stator_prefix(lab):
    """The component name of the module part that does NOT turn - the one that carries the
    joint stop. Read off the solid (single body, radius > 26 mm), never hard-coded, because
    EXO-208's module is a breakLink copy and Fusion is free to rename its children."""
    wr = kids[wrapper_of(lab)]
    mod = wr.childOccurrences.item(0)
    for i in range(mod.childOccurrences.count):
        c = mod.childOccurrences.item(i)
        if c.bRepBodies.count != 1:
            continue
        bb = c.bRepBodies.item(0).boundingBox
        if max(abs(bb.minPoint.x), abs(bb.maxPoint.x)) * 10.0 > 26.0 \
                and bb.maxPoint.z * 10.0 > -1.0:
            return c.component.name + '/'
    raise RuntimeError('%s: no stator found under %s' % (lab, wr.component.name))


def hits(link_name, lab, stator):
    """Link-vs-STATOR interferences only, slivers dropped. Module-internal hits (the
    stator against its own press-fit inserts) and link-vs-ROTOR hits are excluded: in the
    machine the rotor is bolted to the link and turns with it, and only this script's
    rotation leaves it behind (EXO-204)."""
    bodies = adsk.core.ObjectCollection.create()
    for nm in (link_name, wrapper_of(lab)):
        bs, _n = BuildArm.collect_bodies(kids[nm])
        for j in range(bs.count):
            bodies.add(bs.item(j))
    out, other = [], []
    for a, b, vol, bb in BuildArm.analyze_interference(design, bodies):
        if armgen.classify_interference(a, b, bb) != 'FAIL':
            continue
        link_side = [x for x in (a, b) if x.startswith(link_name + '/')]
        stator_side = [x for x in (a, b) if x.startswith(stator)]
        (out if (link_side and stator_side) else other).append((a, b, vol))
    return out, other


log('EXO-208: JOINT_STOP_DEG = %.4f deg, block %.5f..%.5f about the measured centre %.5f'
    % (armgen.JOINT_STOP_DEG, armgen.JOINT_STOP_LO_DEG, armgen.JOINT_STOP_HI_DEG,
       math.degrees(armgen.JOINT_STOP_MID)))
log('')
all_ok, rows = True, []
for name, a, b, pad_r, width in LINKS:
    lab, gname = a, 'g_' + name
    if gname not in kids:
        log('%-3s no %s in the arm - skipped' % (lab, gname))
        continue
    jd = by_label[lab]
    lj = armgen.limited_joint(joints[jd['name']], jd['axis_sign'], pad_r)
    lo_s, hi_s, n = armgen.stop_sweep(lj)
    stator = stator_prefix(lab)
    R1, t1 = frames[lab]
    AXIS = [R1[i][2] for i in range(3)]
    ORIG = [v * 100.0 for v in t1]

    def place(q):
        m = adsk.core.Matrix3D.create()
        m.setToRotation(lj['axis_sign'] * q, adsk.core.Vector3D.create(*AXIS),
                        adsk.core.Point3D.create(*ORIG))
        BuildArm.set_occ_matrix(kids[gname], m, arm)
        adsk.doEvents()

    log('%-3s %s (%s), axis_sign %+d, stator %s' % (lab, lj['joint'], gname,
                                                    lj['axis_sign'], stator[:-1]))
    log('    URDF  %+.5f .. %+.5f deg, range %.5f; link stop %.5f deg wide, centre %.5f'
        % (math.degrees(lj['lower']), math.degrees(lj['upper']),
           math.degrees(lj['range']), math.degrees(lj['arc']), lj['stop_angle']))
    log('    PY    %d poses, first contact %+.6f / %+.6f deg, error %.1e / %.1e'
        % (n, math.degrees(lo_s), math.degrees(hi_s),
           abs(math.degrees(lo_s - lj['lower'])), abs(math.degrees(hi_s - lj['upper']))))
    ok = True
    for what, q0 in (('lower', lj['lower']), ('upper', lj['upper'])):
        inside = q0 + BRACKET if what == 'lower' else q0 - BRACKET
        outside = q0 - BRACKET if what == 'lower' else q0 + BRACKET
        place(inside)
        h_in, o_in = hits(gname, lab, stator)
        place(outside)
        h_out, o_out = hits(gname, lab, stator)
        good = (not h_in) and bool(h_out)
        ok = ok and good
        log('    %-5s %+.5f deg: 0.25 INSIDE -> %d hit(s) [+%d excluded]; '
            '0.25 OUTSIDE -> %d hit(s) %s [+%d excluded]   %s'
            % (what, math.degrees(q0), len(h_in), len(o_in), len(h_out),
               ('%s<->%s %.3f mm3' % h_out[0]) if h_out else '', len(o_out),
               'PASS' if good else 'FAIL'))
    place(0.0)
    h0, o0 = hits(gname, lab, stator)
    ok = ok and not h0
    log('    zero  %d link-vs-stator hit(s) [+%d excluded]   %s'
        % (len(h0), len(o0), 'PASS' if not h0 else 'FAIL'))
    rows.append((lab, ok))
    all_ok = all_ok and ok

log('')
for lab in ORDER:
    if lab not in [r[0] for r in rows]:
        log('%-3s NOT BRACKETED - no link carries its stop block (end of the chain)' % lab)
log('EXO-208 Fusion bracket: %s   (%s)'
    % ('ALL PASS' if all_ok else 'FAILED',
       ', '.join('%s %s' % (l, 'ok' if o else 'FAIL') for l, o in rows)))
