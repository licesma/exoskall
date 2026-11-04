"""EXO-208: why `link_67` is refused, in the frames the builder actually solves.

The EXO-181 chord cuts the rotor pad back so it clears module B. J6's link stop is 160 deg
wide, so it covers most of the rim - including the part the chord takes. Reported here
against BOTH samplings of the block, so it is clear whether EXO-208's finer sampling of a
wide block CAUSED the refusal or only described it better.
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

joints, by_child = armgen.parse_urdf(BuildArm.URDF_PATH)
shift, _ = armgen.joint_world_pose(BuildArm.ORIGIN_AT_JOINT, joints, by_child)
LINKS = [(L['name'], L['a'], L['b'], L['pad_r'], L['width'])
         for L in list(BuildArm.LINKS) + BuildArmGen.NEW_LINKS]
ORDER = ([j['label'] for j in BuildArm.JOINTS]
         + [j['label'] for j in BuildArmGen.NEW_JOINTS])
_s, _l, table = BuildArmGen.seed_frames(joints, by_child, shift)
seed, _c, _sl, _r = BuildArmGen.solve_axis_signs(joints, by_child, shift, LINKS, table,
                                                 lambda s: None, ORDER)
frames, spins = sweepgen.solve_spins(seed, LINKS)
JOINTS_ALL = list(BuildArm.JOINTS) + [dict(j, y_hint=table[j['label']]['y_hint'],
                                           axis_sign=1) for j in BuildArmGen.NEW_JOINTS]
for j in JOINTS_ALL[len(BuildArm.JOINTS):]:
    _t, ax = armgen.joint_world_pose(j['name'], joints, by_child)
    z = [seed[j['label']][0][i][2] for i in range(3)]
    j['axis_sign'] = 1 if armgen.dot(z, ax) > 0 else -1
by_label = {j['label']: j for j in JOINTS_ALL}

for name, a, b, pad_r, width in LINKS:
    RA, tA = frames[a]
    RB, tB = frames[b]
    R_ab, t_ab = armgen.relative_frame(RA, tA, RB, tB)
    num = armgen.landing_numbers(R_ab, t_ab)
    trim, info = sweepgen.solve_pad_trim(num, pad_r)
    jd = by_label[a]
    lj = armgen.limited_joint(joints[jd['name']], jd['axis_sign'], pad_r)
    ang, half = lj['stop_angle'], lj['stop_half']
    if trim is None:
        log('%-8s %s: no EXO-181 chord - the pad already clears module B; block %.3f +/- '
            '%.3f deg is untouched' % (name, a, ang, half))
        continue
    n0, n1, c = trim
    log('%-8s %s: EXO-181 chord %.3f mm deep, normal %.3f deg (the pad is kept on the side '
        'n.p >= %.3f); block %.3f +/- %.3f deg, r %.1f..%.1f'
        % (name, a, info['depth'], math.degrees(math.atan2(n1, n0)) % 360.0, c, ang, half,
           armgen.STOP_R_IN, pad_r))
    old = [(r_ * math.cos(math.radians(ang + k * half)),
            r_ * math.sin(math.radians(ang + k * half)))
           for r_ in (armgen.STOP_R_IN, pad_r) for k in (-1.0, 0.0, 1.0)]
    new = [pt for sg in armgen.stop_block_outline(ang, half, armgen.STOP_R_IN, pad_r)
           for pt in armgen._seg_points(sg, 0.5)]
    for lbl, pts in (('EXO-204 sampling (2 radii x 3 angles)', old),
                     ('EXO-208 sampling (the whole outline)', new)):
        d = min(px * n0 + py * n1 - c for px, py in pts)
        w = min(pts, key=lambda p: p[0] * n0 + p[1] * n1 - c)
        log('    %-38s %d pts, worst %+.4f mm at %.3f deg r %.3f -> %s'
            % (lbl, len(pts), d, math.degrees(math.atan2(w[1], w[0])) % 360.0,
               math.hypot(*w), 'REFUSE' if d < 0.0 else 'pass'))
    for k, lbl in ((-1.0, 'lower'), (+1.0, 'upper')):
        aa = ang + k * half
        ds = [(r_, r_ * math.cos(math.radians(aa)) * n0
               + r_ * math.sin(math.radians(aa)) * n1 - c)
              for r_ in (armgen.JOINT_STOP_R_IN, pad_r)]
        log('    the %s CONTACT edge at %.4f deg: %s  -> %s' % (
            lbl, aa % 360.0, ', '.join('r %.2f %+0.3f mm' % d for d in ds),
            'INSIDE the chord - the edge itself would be cut away and the limit would move'
            if min(d for _r, d in ds) < 0 else 'clear of the chord'))
