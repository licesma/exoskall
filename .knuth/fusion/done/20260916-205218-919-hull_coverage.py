#!/usr/bin/env python3
"""Prove that a route's clearance hulls contain its whole rendered body (EXO-102/EXO-103).

Pure Python, no Fusion:

    /usr/bin/python3 tools/bridge_scripts/hull_coverage.py

`armgen.link_clearance` samples one link's body against another link's HULLS, so the number
it returns is only a lower bound of the real clearance while every point of both bodies is
inside some hull box. EXO-102 measured that it was not: the rotor pad disk, the run's buried
end (at the bar's half depth) and the whole stator slab were outside `hull_parts`.

This script samples the body `render_route` actually draws - the side profile swept over the
plan band, the rotor pad disk down to the stop block, and the stator slab outline swept over
its own thickness - and checks each point against `hull_parts` + `slab_hulls`, in the link
frame where those boxes are axis aligned. It prints the coverage and the worst point outside,
then re-reports the link_12 / link_23 sampled clearance so a hull change that moved it cannot
pass unnoticed. Exit 1 if any body point is outside its own hulls, or if the clearance moved.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'fusion_scripts', 'BuildArm'))
import armgen as g                                              # noqa: E402

URDF = os.path.join(REPO, 'g1', 'g1_29dof.urdf')
ORIGIN_AT_JOINT = 'left_shoulder_pitch_joint'
# BuildArm.JOINTS / LINKS, repeated so this needs no Fusion (kept in step with BuildArm).
JOINTS = [('J1', 'right_shoulder_pitch_joint', -1, (0, 0, 1)),
          ('J2', 'right_shoulder_roll_joint', +1, (0, 1, 0)),
          ('J3', 'right_shoulder_yaw_joint', -1, (-1, 0, 0)),
          ('J4', 'right_elbow_joint', -1, (-1, 0, 0)),
          ('J5', 'right_wrist_roll_joint', +1, (0, 1, 0))]
LINKS = [('link_12', 'J1', 'J2', 26.0, 52.0), ('link_23', 'J2', 'J3', 27.0, 25.0),
         ('link_34', 'J3', 'J4', 26.0, 52.0), ('link_45', 'J4', 'J5', 27.0, 44.0)]
CLEARANCE_WAS = {('link_12', 'link_23'): 1.44}   # EXO-96/EXO-100; must not move
TOL = 1e-6            # mm a body point may sit outside every box before it counts as outside
PROFILE_STEP = 0.25   # mm along the profile / outline
W_SAMPLES = 9         # stations across the band


def frames():
    joints, by_child = g.parse_urdf(URDF)
    shift, _ = g.joint_world_pose(ORIGIN_AT_JOINT, joints, by_child)
    out = {}
    for label, name, sign, hint in JOINTS:
        R, t = g.module_frame(name, joints, by_child, sign, hint)
        out[label] = (R, [t[i] - shift[i] for i in range(3)])
    return out


def body_points(route, geo):
    """(s, w, z) points on the body `render_route` draws, in the link frame."""
    half_w, sc = route['width'] / 2.0, geo['s_curve']
    tp = geo.get('taper')
    j, land = route['rotor_join'], route['stator_land']
    pts = []
    # 1. the side profile, swept over the plan band (the S or the taper, when there is one)
    for seg in geo['profile']:
        for (s, z) in g._seg_points(seg, PROFILE_STEP):
            span = g.s_curve_w(sc, s) if sc else (g.taper_w(tp, s) if tp else None)
            lo, hi = (min(span), max(span)) if span else (geo['profile_w'] - half_w,
                                                          geo['profile_w'] + half_w)
            for k in range(W_SAMPLES):
                pts.append((s, lo + (hi - lo) * k / (W_SAMPLES - 1), z))
    # 2. the rotor pad disk, its rim and the stop block / index tab under it
    pad_r = j['pad_r']
    n = max(8, int(math.ceil(2 * math.pi * pad_r / PROFILE_STEP)))
    for k in range(n + 1):
        a = 2 * math.pi * k / n
        for rr in (pad_r, pad_r * 0.5, 0.0):
            for z in (g.STOP_BLOCK_Z, 0.0, j['pad_t']):
                if rr > g.STOP_R_IN or z >= 0.0:
                    pts.append((rr * math.cos(a), rr * math.sin(a), z))
    # 3. the stator slab: its outline swept over its own thickness
    num = route['numbers']
    u, w_hat, t_ab = route['u'], route['w'], num['t_ab']
    x_s, y_s, z_s = route.get('slab_axes') or (num['x_s'], num['y_s'], num['z_s'])
    for seg in land['flange']:
        for (xp, yp) in g._seg_points(seg, PROFILE_STEP):
            for zp in (g.STATOR_UNDERSIDE_Z - g.PAD_T, g.STATOR_UNDERSIDE_Z):
                p = [t_ab[i] + xp * x_s[i] + yp * y_s[i] + zp * z_s[i] for i in range(3)]
                pts.append((g.dot(p, u), g.dot(p, w_hat), p[2]))
    return pts


def outside(parts, p):
    """How far (mm) p lies outside the nearest box of `parts`; 0 when it is inside one."""
    best = float('inf')
    for (s0, s1), (z0, z1), (wc, wh) in parts:
        d = max(s0 - p[0], p[0] - s1, z0 - p[2], p[2] - z1, abs(p[1] - wc) - wh)
        best = min(best, max(d, 0.0))
        if best <= 0.0:
            return 0.0
    return best


def main():
    F = frames()
    geos, bad = {}, 0
    for name, a, b, pad_r, width in LINKS:
        R_ab, t_ab = g.relative_frame(F[a][0], F[a][1], F[b][0], F[b][1])
        spec = g.route_relative(R_ab, t_ab, pad_r, width=width)
        route, geo = spec['route'], spec['geometry']
        geos[name] = (R_ab, t_ab, geo)
        parts = list(geo['hull_parts']) + list(geo['slab_hulls'])
        pts = body_points(route, geo)
        out = [(outside(parts, p), p) for p in pts]
        n_out = sum(1 for d, _ in out if d > TOL)
        worst, at = max(out, key=lambda q: q[0])
        print('%-8s %s route, %d hull boxes (%d body + %d slab), %d body points: '
              'coverage %6.2f %%  worst outside %.2e mm at (s %.2f, w %.2f, z %.2f)'
              % (name, spec['pattern'], len(parts), len(geo['hull_parts']),
                 len(geo['slab_hulls']), len(pts),
                 100.0 * (len(pts) - n_out) / len(pts), worst, at[0], at[1], at[2]))
        if n_out:
            print('    %d point(s) OUTSIDE every hull box' % n_out)
            bad += 1

    for (n1, n2), want in sorted(CLEARANCE_WAS.items()):
        R1, t1, g1 = geos[n1]
        R2, t2, g2 = geos[n2]
        clr = g.link_clearance(R1, t1, g1, R2, t2, g2)
        ok = abs(clr - want) <= 0.005
        print('%s / %s sampled clearance %.3f mm (was %.3f) -> %s'
              % (n1, n2, clr, want, 'unchanged' if ok else 'MOVED'))
        if not ok:
            bad += 1
    print('hull_coverage: %s' % ('OK' if not bad else '%d FAILURE(S)' % bad))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
