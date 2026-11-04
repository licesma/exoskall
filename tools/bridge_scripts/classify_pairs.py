"""Classify every joint pair of an arm from its URDF (EXO-81).

Pure Python, no Fusion: run it directly.

    /usr/bin/python3 tools/bridge_scripts/classify_pairs.py

For each consecutive pair (rotor pad of joint N, stator pad of joint N+1) it prints
the numbers `armgen.route` classifies on:

  ang        angle between the two joint axes (deg)
  dist       distance between the joint origins (mm)
  axial      component of that along the rotor pad axis Z_A (mm)
  lat        component of it in the rotor pad plane (mm)
  |Zb_ip|    how much of the stator axis lies in the rotor pad plane (1 = fully)
  s_land     reach: distance from axis A, in the pad plane, to the stator slab's
             outer face where the link lands
  zL         height of the landing above the rotor pad's rotor-side plane
  w_off      lateral offset of the landing from the u-z plane through axis A
  pattern    what `armgen.route` picks (or why it refuses)

The module frame convention is armgen.module_frame's: Z = rotor-face normal, Y =
the actuator's connector side. The first joint's (axis_sign, y_hint) come from
BuildArm.JOINTS; every later joint follows the chain rule that the bar/L patterns
need - the connector side faces back down the arm, y_hint = -Z of the previous
joint - with the axis sign chosen so the stator pad faces the previous joint.
Those chained frames are PROVISIONAL for J6..J7 (nothing has been built there);
J1..J5 are the measured exo frames from BuildArm.JOINTS.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'fusion_scripts', 'BuildArm'))
import armgen as g                                              # noqa: E402

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
G1_URDF = os.path.join(REPO, 'g1', 'g1_29dof.urdf')
GL_URDF = os.path.join(REPO, 'gento_luna', 'Gento Luna_URDF 20260428', 'urdf',
                       'Gento Luna_URDF.urdf')

# Ground truth for the G1 right arm (BuildArm.JOINTS, measured against the exo STEP).
G1_NAMES = ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint', 'right_elbow_joint',
            'right_wrist_roll_joint', 'right_wrist_pitch_joint',
            'right_wrist_yaw_joint']
# J1..J5 are the measured exo frames (BuildArm.JOINTS): J4 was confirmed against the exo
# `Module 3 R` pose in EXO-103 and J5 against `Module 4 R` in EXO-114, so neither is the
# chain rule's guess any more.
G1_KNOWN = {1: (-1, (0, 0, 1)), 2: (+1, (0, 1, 0)), 3: (-1, (-1, 0, 0)),
            4: (-1, (-1, 0, 0)), 5: (+1, (0, 1, 0))}
GL_NAMES = ['Arm_R%d_Joint' % i for i in range(1, 8)]
GL_KNOWN = {1: (+1, (0, 0, 1))}
# the rotor pad radius BuildArm.LINKS uses at each rotor joint (the rest take the default)
PAD_R = {'J2': 27.0, 'J4': 27.0}


def chain_frames(path, names, known):
    """Module frames for a joint chain. `known` maps 1-based index -> (axis_sign, y_hint);
    any joint not in it follows the chain rule (see the module docstring)."""
    joints, by_child = g.parse_urdf(path)
    frames, provisional = [], []
    for i, name in enumerate(names):
        idx = i + 1
        if idx in known:
            sign, hint = known[idx]
        else:
            z_prev = [frames[-1][0][k][2] for k in range(3)]
            t_prev = frames[-1][1]
            t_j, axis = g.joint_world_pose(name, joints, by_child)
            back = [t_prev[k] - t_j[k] for k in range(3)]
            sign = -1 if g.dot(axis, back) > 1e-9 else 1
            hint = [-v for v in z_prev]
            provisional.append(idx)
        frames.append(g.module_frame(name, joints, by_child, sign, hint))
    return frames, provisional


def pair_numbers(frame_a, frame_b):
    """The classification numbers of one pair. `frame_a`/`frame_b` = (R, t_meters)."""
    R_ab, t_ab = g.relative_frame(frame_a[0], frame_a[1], frame_b[0], frame_b[1])
    zb = [R_ab[i][2] for i in range(3)]
    yb = [R_ab[i][1] for i in range(3)]
    out = {
        'R_ab': R_ab, 't_ab': t_ab,
        'ang': math.degrees(math.acos(max(-1.0, min(1.0, abs(zb[2]))))),
        'dist': math.sqrt(g.dot(t_ab, t_ab)),
        'axial': t_ab[2],
        'lat': math.hypot(t_ab[0], t_ab[1]),
        'zb_in_plane': math.hypot(zb[0], zb[1]),
        'yb_axial': abs(yb[2]),
    }
    if out['zb_in_plane'] > 1e-6:
        out.update(g.landing_numbers(R_ab, t_ab))
    return out


def table(title, path, names, known, labels):
    frames, provisional = chain_frames(path, names, known)
    print('=== %s ===' % title)
    if provisional:
        print('    provisional frames (chain rule): %s'
              % ', '.join(labels[i - 1] for i in provisional))
    print('%-9s %6s %8s %8s %8s %8s %8s %8s %8s  %s'
          % ('pair', 'ang', 'dist', 'axial', 'lat', 's_land', 'zL', 'w_off', '|Zb_ip|', 'pattern'))
    for i in range(len(frames) - 1):
        n = pair_numbers(frames[i], frames[i + 1])
        try:
            spec = g.route(frames[i], frames[i + 1], PAD_R.get(labels[i], g.ROTOR_PAD_R))
            verdict = spec['pattern']
        except ValueError as e:
            verdict = 'NONE: %s' % (str(e).split('). ', 1)[-1][:150],)
        num = lambda k: n.get(k, float('nan'))                  # noqa: E731
        print('%-9s %6.2f %8.2f %8.2f %8.2f %8.2f %8.2f %8.2f %8.4f  %s'
              % ('%s->%s' % (labels[i], labels[i + 1]), n['ang'], n['dist'], n['axial'],
                 n['lat'], num('s_land'), num('zL'), num('w_off'), n['zb_in_plane'], verdict))
    print()


def main():
    table('G1 right arm (%s)' % os.path.basename(G1_URDF), G1_URDF, G1_NAMES, G1_KNOWN,
          ['J%d' % i for i in range(1, 8)])
    table('Gento Luna right arm (Arm_R1..R7_Joint)', GL_URDF, GL_NAMES, GL_KNOWN,
          ['R%d' % i for i in range(1, 8)])


if __name__ == '__main__':
    main()
