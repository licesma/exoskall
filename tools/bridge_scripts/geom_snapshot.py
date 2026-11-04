"""Shared G1 fixture: the module frames and the link table, without Fusion.

    /usr/bin/python3 tools/bridge_scripts/geom_snapshot.py dump [file.json]

EXO-144 removed this file's other half. It used to hold a saved baseline of every number
`armgen.route` produced for the four built links and `check` exited 1 when any of them
moved - the bit-for-bit regression against the HAND-MADE arm that EXO-142 named as the
thing freezing the algorithm (EXO-134 item 2, EXO-112 item 4, EXO-135). Esteban's restated
'Done when' (EXO-41, 2026-09-17) replaces it: the bar is now axes within tolerance and all
constraints passing - interference 0 FAIL, `HOLE_FLAT_RING` on every hole, the actuator
cutout cleared - and NOT identical faces or volume. `dump` still writes the same tree, for
looking at; nothing compares it to anything and nothing here can fail a build.

What the rest of the bridge scripts import from here is only the fixture: `g1_seed_frames`,
`g1_frames`, `JOINTS`, `LINKS`.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'fusion_scripts', 'BuildArm'))
import armgen as g                                              # noqa: E402

URDF = os.path.join(REPO, 'g1', 'g1_29dof.urdf')
# BuildArm.JOINTS / BuildArm.LINKS, kept here so the snapshot does not need Fusion.
JOINTS = [('J1', 'right_shoulder_pitch_joint', -1, (0, 0, 1)),
          ('J2', 'right_shoulder_roll_joint', +1, (0, 1, 0)),
          ('J3', 'right_shoulder_yaw_joint', -1, (-1, 0, 0)),
          ('J4', 'right_elbow_joint', -1, (-1, 0, 0)),
          ('J5', 'right_wrist_roll_joint', +1, (0, 1, 0))]
# EXO-130: all four built links are in the snapshot now, not only the two EXO-96 refactored.
LINKS = [('link_12', 'J1', 'J2', 26.0, 52.0), ('link_23', 'J2', 'J3', 27.0, 25.0),
         ('link_34', 'J3', 'J4', 26.0, 52.0), ('link_45', 'J4', 'J5', 27.0, 44.0)]
ORIGIN_AT_JOINT = 'left_shoulder_pitch_joint'


def g1_seed_frames():
    """The module frames at the SEED rotation about each axis - `y_hint`, the pose measured
    in the exo STEP - before EXO-130's rotation solve."""
    joints, by_child = g.parse_urdf(URDF)
    shift, _ = g.joint_world_pose(ORIGIN_AT_JOINT, joints, by_child)
    out = {}
    for label, name, sign, hint in JOINTS:
        R, t = g.module_frame(name, joints, by_child, sign, hint)
        out[label] = (R, [t[i] - shift[i] for i in range(3)])
    return out


def g1_frames(spins=None):
    """The arm as BuildArm builds it: the seed frames with every module at its SOLVED
    rotation about its own axis (EXO-130). Returns the frames; pass a dict to receive the
    spin table the solve produced."""
    frames, table = g.spun_frames(g1_seed_frames(), LINKS)
    if spins is not None:
        spins.update(table)
    return frames


def _plain(v):
    """JSON-able, tuples as lists, floats rounded nowhere (full precision)."""
    if isinstance(v, (list, tuple)):
        return [_plain(x) for x in v]
    if isinstance(v, dict):
        return {str(k): _plain(x) for k, x in v.items()}
    if isinstance(v, float):
        return v if math.isfinite(v) else str(v)
    if isinstance(v, (int, str, bool)) or v is None:
        return v
    raise TypeError('cannot snapshot %r' % (type(v),))


def snapshot():
    frames = g1_frames()
    out = {}
    for name, a, b, pad_r, width in LINKS:
        spec = g.route(frames[a], frames[b], pad_r, width=width)
        out[name] = {'pattern': spec['pattern'], 'geometry': _plain(spec['geometry']),
                     'numbers': _plain(spec['numbers'])}
    return out


def main(argv):
    if len(argv) < 2 or argv[1] != 'dump':
        print(__doc__)
        return 2
    now = snapshot()
    path = argv[2] if len(argv) > 2 else None
    if path:
        with open(path, 'w') as f:
            json.dump(now, f, indent=1, sort_keys=True)
    print('dumped %s: %d geometry keys%s'
          % (', '.join(sorted(now)), sum(len(v['geometry']) for v in now.values()),
             ' -> ' + path if path else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
