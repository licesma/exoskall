"""EXO-204 evidence: the two measurements the limited-joint rule stands on.

Both are read off `sensor module 2.step` in the MODULE frame (origin = rotor-face centre,
Z = the axis out of that face, +Y = the actuator connector side):

  1. the JOINT stop - the fixed block on the stator rim - its angular span, radii and z;
  2. the rotor's INDEX NOTCH - the feature that makes 'the link as drawn' the one and only
     orientation the link can be bolted on in, and therefore fixes URDF q = 0.

Run: /usr/bin/python3 tools/bridge_scripts/measure_exo204.py
Pure Python (steplib), no Fusion. Nothing is written.
"""
import os
import sys
import math

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
sys.path.insert(0, os.path.join(ROOT, 'fusion_scripts', 'BuildArm'))
import steplib                                     # noqa: E402
import armgen as g                                 # noqa: E402

# EXO-242: the stator and the rotor face disk, under either of their two names - this STEP
# is the ORIGINAL `sensor module 2`, which keeps Fusion's `Component1` / `Component2`, but
# a v3 export carries `stator` / `inner_top`.
C1 = g.module_part_names('stator')
C2 = g.module_part_names('inner_top')

STEP = os.path.join(ROOT, 'gento_luna', 'Gento Luna_URDF 20260428', 'urdf',
                    'sensor module 2.step')


def ang(p):
    a = math.degrees(math.atan2(p[1], p[0]))
    return a + 360.0 if a < 0 else a


def main():
    s = steplib.Step(STEP)
    stop, notch = [], []
    for _path, name, pd, T in s.occurrences():
        vs = s.vertices(pd)
        if not vs:
            continue
        w = [steplib._apply(T, v) for v in vs]
        if name in C1:                             # stator (EXO-242: either name)
            # the block's own annulus and z band. The stator's r 27 CYLINDER has its seam
            # vertex in the same band, on the +X axis - drop anything within 1e-3 deg of
            # 0/360, which no face of a block at 158..178 can be.
            stop = [p for p in w
                    if 20.5 <= math.hypot(p[0], p[1]) <= 27.5 and -4.05 <= p[2] <= -0.35
                    and min(ang(p), 360.0 - ang(p)) > 1e-3]
        if name in C2:                             # rotor face disk (either name)
            notch = [p for p in w
                     if 14.5 <= math.hypot(p[0], p[1]) <= 18.1 and -1.6 <= p[2] <= 0.05
                     and abs(p[0]) < 10.0]

    print('--- 1. the JOINT stop (stator rim, %s), module frame ---' % ' / '.join(C1))
    angs = sorted(ang(p) for p in stop)
    lo, hi = angs[0], angs[-1]
    print('  angles   %.4f .. %.4f deg   ->  arc a = %.4f deg = %.6f rad'
          % (lo, hi, hi - lo, math.radians(hi - lo)))
    print('  centre   %.4f deg' % ((lo + hi) / 2.0))
    print('  radii    %.3f .. %.3f' % (min(math.hypot(p[0], p[1]) for p in stop),
                                       max(math.hypot(p[0], p[1]) for p in stop)))
    print('  z        %.3f .. %.3f' % (min(p[2] for p in stop), max(p[2] for p in stop)))
    print('  armgen   %.4f .. %.4f deg  (JOINT_STOP_LO_DEG / _HI_DEG)'
          % (g.JOINT_STOP_LO_DEG, g.JOINT_STOP_HI_DEG))

    print('--- 2. the rotor INDEX NOTCH (%s), module frame ---' % ' / '.join(C2))
    print('  x        %.3f .. %.3f   (centre %.4f, i.e. %.4f deg = +Y)'
          % (min(p[0] for p in notch), max(p[0] for p in notch),
             (min(p[0] for p in notch) + max(p[0] for p in notch)) / 2.0,
             ang(((min(p[0] for p in notch) + max(p[0] for p in notch)) / 2.0,
                  max(p[1] for p in notch), 0.0))))
    print('  y        %.3f .. %.3f' % (min(p[1] for p in notch), max(p[1] for p in notch)))
    print('  z        %.3f .. %.3f   (1.5 deep)'
          % (min(p[2] for p in notch), max(p[2] for p in notch)))
    print('  the link\'s mating tab: x +/-%.2f, r %.2f..%.2f, %.1f proud (TAB_*)'
          % (g.TAB_W / 2.0, g.TAB_R_IN, g.TAB_R_OUT, g.TAB_H))

    print('--- 3. what the rule makes of G1 J1 ---')
    joints, by_child = g.parse_urdf(os.path.join(ROOT, 'g1', 'g1_29dof.urdf'))
    j = joints['right_shoulder_pitch_joint']
    for sign in (-1, +1):
        sp = g.limited_joint(j, sign, 26.0)
        q_lo, q_hi, n = g.stop_sweep(sp)
        print('  axis_sign %+d: b %.5f deg (%.6f rad), centre %.5f deg; swept %d poses, '
              'first contact %+.5f / %+.5f deg, error %.1e / %.1e deg'
              % (sign, math.degrees(sp['arc']), sp['arc'], sp['stop_angle'], n,
                 math.degrees(q_lo), math.degrees(q_hi),
                 abs(math.degrees(q_lo - j['lower'])), abs(math.degrees(q_hi - j['upper']))))


if __name__ == '__main__':
    main()
