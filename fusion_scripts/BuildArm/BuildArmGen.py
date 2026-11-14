"""BuildArmGen (EXO-144): build the arm by the GENERAL method, beside the existing one.

One body primitive and one landing primitive, for every link:

  * the body is a ribbon `width` x `thickness` SWEPT (Fusion `sweepFeatures`) along the 3D
    centre line `sweepgen.sweep_route` computes - straight runs joined by tangent bends in
    any plane. There is no side profile, no plan band, no S device, no taper and no named
    shape anywhere in this file.
  * the landing is a wrapping plate on every stator: the disk, a tongue centred on the
    plate's own axis and wide enough to carry the ribbon, the three insert holes and the
    actuator cutout.
  * the rotor join is the pad disk (ROTOR_PAD_T, 8 by default), its four M3 holes with O6.2 head
    counterbores (EXO-383), the stop block and,
    under the pad, the fused rotor top (EXO-370: disk, journal, eight register pegs, O16
    centre hole, boss recess r 8.0 to -5.4).

Everything is created under a NEW root component `generated_R_arm_general`, so the
existing `generated_R_arm` and the hand-made arm stay where they are and all three can be
overlaid. Run it through the AgentBridge:

    import importlib, sys
    sys.path.insert(0, '<repo>/fusion_scripts/BuildArm')
    import BuildArmGen; importlib.reload(BuildArmGen)
    log(BuildArmGen.build_general(app, ui, design, log))
"""
import os
import sys
import math

import adsk.core
import adsk.fusion

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import importlib
import armgen                      # noqa: E402
import sweepgen                    # noqa: E402
import BuildArm                    # noqa: E402
importlib.reload(armgen)
importlib.reload(sweepgen)
importlib.reload(BuildArm)

ARM_NAME = 'generated_R_arm_general'
REPORT_PATH = os.path.join(BuildArm.REPO_DIR, 'gento_luna', 'exo', 'exports',
                           'buildarm_general_report.txt')
MM_TO_CM = 0.1
ROUND_TRIES = 6        # EXO-184: how many times a link may be rebuilt at half the round

# ---------------- the arm this builder builds (EXO-158) ----------------
# `BuildArm.JOINTS` / `BuildArm.LINKS` stop at J5, because the arm they describe is the
# hand-built one and EXO-43 sacrificed its wrist pitch: there is no `Module 5` and no J6 to
# copy anything from. The general method does not copy anything, so it carries the WHOLE
# URDF chain here and leaves `BuildArm`'s two lists - and therefore `generated_R_arm` and
# the hand-made arm - exactly as they were.
#
# J6 and J7 come from the URDF only. `y_hint` is a SEED that the EXO-150 encoder rule
# overwrites at every stator end (both of them are one), so it only has to be off the axis;
# it is taken as the previous module's axis, reversed. `axis_sign` is the one thing the URDF
# does not fix and it is NOT copied: `sweepgen.axis_sign_rule` derives it where the pair's
# own geometry decides it and returns None where it does not, and `solve_axis_signs` then
# solves it from the link, the way the spin is solved.
NEW_JOINTS = [
    {'label': 'J6', 'name': 'right_wrist_pitch_joint'},
    {'label': 'J7', 'name': 'right_wrist_yaw_joint'},
]
NEW_LINKS = [
    {'a': 'J5', 'b': 'J6', 'name': 'link_56', 'width': 25.0, 'pad_r': 27.0},
    {'a': 'J6', 'b': 'J7', 'name': 'link_67', 'width': 25.0, 'pad_r': 27.0},
]

# EXO-204: which joints get a GENERATED link stop - a block sized and placed from that
# joint's own URDF <limit> by `armgen.limited_joint`, so that the rotor meets the module's
# joint stop at exactly `lower` and exactly `upper`.
#
# EXO-208: ALL OF THEM. The PENDING path is gone - no joint of this arm carries a
# placement-only block any more, and no `stop_angle / stop_half` in `BuildArm.LINKS` is
# read. Nothing in the rule was ever per-joint; turning it on for J2..J7 is this tuple.
#
# J7 is in the list and is SIZED by the rule, but no body carries it: the link stop lives on
# the rotor pad of the link that LEAVES the joint, and J7 is the last joint of the chain -
# `LINKS` runs link_12 .. link_67, so J1..J6 have a rotor pad and J7 does not. Whatever
# mounts on J7's rotor (the handle) has to carry J7's block; `report_limits` says so by name
# rather than letting the joint disappear from the table.
LIMITED_JOINTS = ('J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7')


# ---------------- the arms this builder can build (EXO-218) ----------------
# Everything that was G1 above is now one ARM SPEC, and Gento Luna is a second one. The
# builder reads nothing else: an arm is a URDF, a list of joints, a list of links with one
# WIDTH each, and a name to build under.
#
#   fixed    joints whose (axis_sign, y_hint) are known from a measurement - G1's J1..J5,
#            the poses EXO-103 / EXO-114 confirmed in the exo STEP. Luna has none: no
#            hand-made Luna arm exists to measure.
#   derived  joints whose axis_sign `sweepgen.axis_sign_rule` derives from the pair, or,
#            where the pair decides nothing, that `solve_axis_signs` solves with the spins
#            and the slides (EXO-160). All seven of Luna's are derived.
#   offset   where the arm is PARKED in the document, in metres, off `origin_at`. Display
#            only: it moves the whole chain rigidly, and every axis check adds it back, so
#            the kinematics are the URDF's whatever it is. It exists because both arms live
#            in one document and two arms on one origin cannot be looked at.
G1_SPEC = {
    'key': 'g1',
    'name': ARM_NAME,
    'urdf': BuildArm.URDF_PATH,
    'origin_at': BuildArm.ORIGIN_AT_JOINT,
    'offset': (0.0, 0.0, 0.0),
    'fixed': BuildArm.JOINTS,
    'derived': NEW_JOINTS,
    'links': [dict(L) for L in list(BuildArm.LINKS) + NEW_LINKS],
    'limited': LIMITED_JOINTS,
    'report': REPORT_PATH,
}

# Gento Luna, right arm (EXO-15/EXO-95). Seven revolute joints Arm_R1..R7_Joint; R1/R2 share
# an origin and R5/R6/R7 share another, so three of the six pairs start with their modules on
# top of one another and the EXO-170/214/218 slide rule has to open every one of them.
LUNA_URDF = os.path.join(BuildArm.REPO_DIR, 'gento_luna',
                         'Gento Luna_URDF 20260428', 'urdf', 'Gento Luna_URDF.urdf')
LUNA_ARM_NAME = 'generated_R_arm_luna'
LUNA_REPORT_PATH = os.path.join(BuildArm.REPO_DIR, 'gento_luna', 'exo', 'exports',
                                'buildarm_luna_report.txt')
LUNA_JOINTS = [{'label': 'R%d' % k, 'name': 'Arm_R%d_Joint' % k} for k in range(1, 8)]
# WIDTH is Esteban's one input per link (EXO-96) and there is no hand-made Luna link to take
# it from, so each link DEFAULTS to the width the G1 arm's link of the same index carries -
# 52 / 25 / 52 / 44 / 25 / 25 - and the same for the rotor pad radius, 26 / 27 / 26 / 27 /
# 27 / 27. Both are reported as defaults at every run. There is no per-joint override.
LUNA_DEFAULT_WIDTHS = tuple(L['width'] for L in list(BuildArm.LINKS) + NEW_LINKS)
LUNA_DEFAULT_PAD_R = tuple(L['pad_r'] for L in list(BuildArm.LINKS) + NEW_LINKS)
LUNA_LINKS = [{'a': 'R%d' % (k + 1), 'b': 'R%d' % (k + 2),
               'name': 'link_R%d%d' % (k + 1, k + 2),
               'width': LUNA_DEFAULT_WIDTHS[k], 'pad_r': LUNA_DEFAULT_PAD_R[k],
               'width_why': 'DEFAULT - the G1 arm\'s link %d width' % (k + 1)}
              for k in range(6)]
# The free axis signs and the slides, SOLVED by `sweepgen.solve_axis_signs` - the same
# function the builder would call - run outside Fusion by `tools/luna_solve.py` and checked
# here at every build by `sweepgen.verify_solved`. It is precomputed because the solve is
# 2^5 = 32 axis-sign combinations, each sliding four candidates against the router for up to
# eight rounds: 40 minutes of pure Python per combination, which is not something to spend
# inside a Fusion session holding an unsaved document. Nothing about the RULE changes - the
# build re-checks that every pair fits at these numbers, that no module pair is under
# MIN_CLEARANCE, and that every non-zero slide is minimal.
LUNA_SOLVED = {
    'signs': {'R1': -1, 'R2': 1, 'R4': -1, 'R6': 1, 'R7': 1},
    'slides': {'R1': -38.0, 'R2': 29.500977, 'R3': -37.000977, 'R4': 0.0,
               'R5': -38.0, 'R6': -38.0, 'R7': 0.0},
    'why': ('All 32 combinations were tried (k = 5). 25 of them route all six links; this '
            'one has the shortest total centre line, 625.710 mm - the rule of '
            '`solve_axis_signs`, unchanged. Total slide 180.502 mm. R6 and R7 do not '
            'change the answer (k = 5 / 13 / 21 / 29 tie on every number), so the rule\'s '
            'last tie-break, the lowest combination, takes k = 5.'),
}
LUNA_SPEC = {
    'key': 'luna',
    'name': LUNA_ARM_NAME,
    'urdf': LUNA_URDF,
    'origin_at': 'Arm_R1_Joint',
    # parked 0.6 m off the G1 arm's shoulder along +Y so the two arms can be looked at
    'offset': (0.0, 0.6, 0.0),
    'fixed': [],
    'derived': LUNA_JOINTS,
    'links': LUNA_LINKS,
    'limited': tuple(j['label'] for j in LUNA_JOINTS),
    'report': LUNA_REPORT_PATH,
    'solved': LUNA_SOLVED,
}


def report_limits(rep, spec, limits, order, unbuilt=()):
    """EXO-204/EXO-208: the joint limit every generated link stop actually gives, by sweep."""
    rep('LIMITED JOINTS (EXO-204, generalised by EXO-208). The module\'s joint stop is '
        'JOINT_STOP_DEG = %.1f deg wide - r %.2f..%.1f, z %.1f..%.1f, %.5f..%.5f deg in the '
        'module frame, the measured centre %.5f of `sensor module 2` with the width re-cut '
        'in the copy `%s`; the same at every joint. The link stop is generated: its ARC is '
        'what is left of the circle after the joint stop and the URDF\'s own range, and its '
        'ANGLE is the one that makes the rotor touch at exactly `lower`. Zero pose: URDF '
        'q = 0 is the link as drawn, which is the one orientation the rotor key allows '
        '(EXO-364: three pegs at 45/135/225 deg, the empty position at 315 deg). The joint stop is the MINIMUM: b >= a = %.1f deg '
        '(+/- %.3f measurement tolerance), so the widest range this module can limit is '
        '%.4f deg.'
        % (armgen.JOINT_STOP_DEG, armgen.JOINT_STOP_R_IN, armgen.MODULE_R,
           armgen.JOINT_STOP_Z[0], armgen.JOINT_STOP_Z[1], armgen.JOINT_STOP_LO_DEG,
           armgen.JOINT_STOP_HI_DEG, math.degrees(armgen.JOINT_STOP_MID),
           BuildArm.MODULE_V3_NAME, armgen.JOINT_STOP_DEG,
           math.degrees(armgen.LINK_STOP_B_TOL), 360.0 - 2.0 * armgen.JOINT_STOP_DEG))
    allj = list(limits) + [(None, lab, lj, None) for lab, lj in unbuilt]
    capped = [(lab, lj) for _n, lab, lj, _r in allj if lj is not None and lj['capped']]
    rep('  RANGE CAP (EXO-218): a joint whose URDF range is wider than the %.4f deg this '
        'module can limit is CAPPED to it, symmetrically about the URDF midpoint, and named '
        'here. JOINT_STOP_DEG is NOT changed for it. %s'
        % (360.0 - 2.0 * armgen.JOINT_STOP_DEG,
           'No joint of this arm is capped.' if not capped else
           'CAPPED: %s - each gives up %s deg at each end.'
           % (', '.join('%s (%s, %.3f deg of range)'
                        % (lab, lj['joint'], math.degrees(lj['urdf_range']))
                        for lab, lj in capped),
              ' / '.join('%.5f' % (math.degrees(lj['urdf_range'] - lj['range']) / 2.0)
                         for lab, lj in capped))))
    done = set(lab for _n, lab, _lj, _r in limits)
    nolink = dict(unbuilt)
    for lab in order:
        if lab in spec['limited'] and lab not in done:
            lj = nolink.get(lab)
            rep('  %-3s NO LINK CARRIES IT - %s is the last joint of the chain, so no '
                'generated link has its rotor pad and there is nowhere to put a link stop. '
                'The rule sizes it anyway: %s. Whatever mounts on this rotor has to carry '
                'that block.'
                % (lab, lab,
                   'range %.5f deg -> b %.5f deg (half %.5f), centre %.5f deg, '
                   'axis_sign %+d, built as ONE block (EXO-218)'
                   % (math.degrees(lj['range']), math.degrees(lj['arc']), lj['stop_half'],
                      lj['stop_angle'], lj['axis_sign'])
                   if lj else 'no URDF limits'))
        elif lab not in spec['limited']:
            rep('  %-3s PENDING - placement-only block at %.0f +/- 10 deg, no limit designed'
                % (lab, sweepgen.STOP_ANGLE))
    for name, lab, lj, route in limits:
        lo, hi, n = armgen.stop_sweep(lj)
        ov = sweepgen.stop_ribbon_overlap(route, lj)
        rep('  %-3s %s (%s), axis_sign %+d' % (lab, lj['joint'], name, lj['axis_sign']))
        rep('      URDF   lower %+.4f  upper %+.4f rad  = %+.3f .. %+.3f deg, range %.3f deg'
            % (lj['urdf_lower'], lj['urdf_upper'], math.degrees(lj['urdf_lower']),
               math.degrees(lj['urdf_upper']), math.degrees(lj['urdf_range'])))
        rep('      CAP    %s'
            % ('the URDF range fits: %.5f deg, under the %.5f deg this module can limit - '
               'nothing is given up' % (math.degrees(lj['urdf_range']),
                                        math.degrees(lj['max_range']))
               if not lj['capped'] else lj['cap_why']))
        rep('      RULE   b = 2*pi - a - range USED (%.5f deg) = %.6f rad = %.5f deg '
            '(%.5f deg %s the %.1f deg joint stop), centre %.5f deg, block r %.1f..%.1f '
            'x %.1f mm'
            % (math.degrees(lj['range']), lj['arc'], math.degrees(lj['arc']),
               abs(math.degrees(lj['b_margin'])),
               'over' if lj['b_margin'] >= 0 else 'UNDER', armgen.JOINT_STOP_DEG,
               lj['stop_angle'], lj['r_in'], lj['r_out'], lj['stop_h']))
        rep('      BUILT  ONE solid block, %.5f +/- %.5f deg (EXO-218). The contact faces '
            'are its two ends, %.5f and %.5f deg.'
            % (lj['stop_angle'], lj['stop_half'],
               (lj['stop_angle'] - lj['stop_half']) % 360.0,
               (lj['stop_angle'] + lj['stop_half']) % 360.0))
        chord = route.get('stop_chord_margins') or ()
        rep('      CHORD  %s'
            % ('no EXO-181 chord on this pad: nothing to miss'
               if not route.get('pad_trim') else
               'the %.3f mm chord clears %s (need %.1f, EXO-214/EXO-218)'
               % (armgen.pad_trim_depth(route['pad_r'], route['pad_trim']),
                  ', '.join('%s by %.3f mm' % (w, d) for w, d in chord),
                  sweepgen.STOP_CHORD_MARGIN)))
        rep('      SWEEP  %d poses: first contact at %+.5f and %+.5f deg; error vs the URDF '
            '%.2e / %.2e deg (need < 0.5). Gap at q = 0: %.3f deg.'
            % (n, math.degrees(lo), math.degrees(hi),
               abs(math.degrees(lo - lj['lower'])), abs(math.degrees(hi - lj['upper'])),
               math.degrees(lj['zero_gap'])))
        rep('      RIBBON the stop block spans %.3f..%.3f deg; the ribbon root crosses its '
            'annulus over %s (%d samples) - nearest approach %.3f deg: %s'
            % (ov['stop_sector_deg'][0], ov['stop_sector_deg'][1],
               'nothing' if ov['band_deg'] is None
               else '%.3f..%.3f deg' % ov['band_deg'], ov['n'], ov['gap_deg'],
               'CLEAR' if ov['clear']
               else 'OVERLAPS - REPORT ONLY (EXO-204/EXO-216): the block\'s angle is the '
               'joint\'s limits and the ribbon\'s is the pair; the only free variable is '
               'the module spin, which is solved for other reasons (EXO-150) and is not '
               'touched here - and the block is UNDER the pad while the ribbon root is ON '
               'it, so an overlap in plan is material under the ribbon\'s foot, not a clash'))
    rep('')


def report_slides(rep, frames, order, slides, rows):
    """The EXO-170 slide table: what moved along its own axis, what it was solved from, and
    every candidate the rule did not take."""
    rep('')
    rep('Module position ALONG ITS OWN AXIS - `slide`, the degree of freedom a revolute '
        'joint leaves free (EXO-170, generalised by EXO-214). SOLVED, never an input, no '
        'per-joint override. The axis LINE is untouched, so the kinematics stay the '
        'URDF\'s exactly.')
    rep('  EXO-214: a slide is the MINIMUM along the module\'s own axis that makes the '
        'whole pair FIT - not just the minimum that opens the two bodies. Fitting is three '
        'things, each of them a rule that already existed: the module BODIES clear '
        '(armgen.MODULE_PROFILE, the real solid of revolution EXO-166 measured, not an '
        'infinite cylinder); the LINK between them routes, sized with that joint\'s own '
        'link stop; and whatever pad chord EXO-181 still needs MISSES that stop block '
        'by STOP_CHORD_MARGIN %.1f mm, with the link\'s rotor join clearing '
        'module B by the same target the bodies are asked for. The cost is paid one for one '
        'as run by the link that lands on the slid module.'
        % sweepgen.STOP_CHORD_MARGIN)
    rep('  target gap %.2f mm = MIN_CLEARANCE %.1f + SLIDE_MARGIN %.1f; scanned at %.1f mm '
        'and bisected to %.0e mm; module surface sampled %d x %d'
        % (armgen.MIN_CLEARANCE + sweepgen.SLIDE_MARGIN, armgen.MIN_CLEARANCE,
           sweepgen.SLIDE_MARGIN, sweepgen.SLIDE_STEP, sweepgen.SLIDE_TOL,
           sweepgen.SLIDE_N_THETA, sweepgen.SLIDE_N_Z))
    for lab in order:
        v = slides.get(lab, 0.0)
        rep('  %-3s slide %+9.3f mm%s' % (lab, v,
            '' if abs(v) > 1e-9 else '  (its neighbours already clear: nothing moves)'))
    for r in rows:
        a, b = r['pair']
        rep('  %s <-> %s: %s. Four candidates, each the smallest slide of ONE module in '
            'ONE direction that makes the pair fit:'
            % (a, b, r.get('trigger') or 'the two module bodies are %.3f mm apart, under '
               'MIN_CLEARANCE %.1f' % (r['gap0'], armgen.MIN_CLEARANCE)))
        for c in r['candidates']:
            if c['slide'] is None:
                rep('      %-3s %+d: refused - %s' % (c['module'], c['sign'], c['why']))
            else:
                rep('      %-3s %+d: %8.3f mm -> gap %6.3f mm%s%s'
                    % (c['module'], c['sign'], c['slide'], c['gap'],
                       '' if not c['why'] else '  REJECTED: ' + c['why'],
                       '   <- TAKEN (the smallest)'
                       if r['taken'] is not None and c is r['taken'] else ''))
        if r['taken'] is None:
            rep('      none of them works: %s' % r['why'])
        else:
            second = sorted([c['slide'] for c in r['candidates']
                             if c['slide'] is not None and not c['why']])
            if len(second) > 1:
                rep('      the rule chose by %.3f mm (the next candidate is %.3f mm)'
                    % (second[1] - second[0], second[1]))
    rep('  module-to-module gap at every pair, nearest first (need >= %.1f):'
        % armgen.MIN_CLEARANCE)
    for a, b, gp in sweepgen.module_pairs(frames, order)[:8]:
        rep('      %s <-> %s  %8.3f mm  %s' % (a, b, gp,
            'OK' if gp >= armgen.MIN_CLEARANCE - 1e-9 else 'FAIL'))


def _p(v):
    """mm triple -> Point3D in cm."""
    return adsk.core.Point3D.create(v[0] * MM_TO_CM, v[1] * MM_TO_CM, v[2] * MM_TO_CM)


def _unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v]


class Arm:
    """The pad frame A, and the (s, w, z) -> world map the route is expressed in."""

    def __init__(self, R, t_mm, u, w):
        self.F = BuildArm.Frame(R, t_mm)
        self.u, self.w = u, w

    def xyz(self, p):
        """A point (s, w, z) of the route, in the arm (= world) frame, mm."""
        loc = [self.u[i] * p[0] + self.w[i] * p[1] + (1.0 if i == 2 else 0.0) * p[2]
               for i in range(3)]
        return self.F.xyz(loc)

    def dir(self, d):
        loc = [self.u[i] * d[0] + self.w[i] * d[1] + (1.0 if i == 2 else 0.0) * d[2]
               for i in range(3)]
        return armgen.matvec(self.F.R, loc)

    def swz(self, p_cm):
        """A Fusion point (cm, world) back in the route's own (s, w, z), mm."""
        d = [p_cm.x * 10.0 - self.F.t[0], p_cm.y * 10.0 - self.F.t[1],
             p_cm.z * 10.0 - self.F.t[2]]
        loc = armgen.matvec(armgen.transpose(self.F.R), d)
        return (armgen.dot(loc, self.u), armgen.dot(loc, self.w), loc[2])


def build_ribbon(comp, arm, route, name):
    """The one body primitive: the ribbon swept along the route's 3D centre line."""
    # Fusion only lets a sketch hold off-plane geometry when this preference is on
    # (measured: `Sketch` has no `is3D` in this build, and without the preference every
    # point is silently projected onto the sketch plane).
    pref = adsk.core.Application.get().preferences.productPreferences.itemByName('Design')
    if not pref.is3DSketchingAllowed:
        pref.is3DSketchingAllowed = True
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = name + ' centre line'
    curves = adsk.core.ObjectCollection.create()
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    for sg in route['path']:
        if sg[0] == 'line':
            c = lines.addByTwoPoints(_p(arm.xyz(sg[1])), _p(arm.xyz(sg[2])))
        else:
            c = arcs.addByThreePoints(_p(arm.xyz(sg[1])), _p(arm.xyz(sg[2])),
                                      _p(arm.xyz(sg[3])))
        curves.add(c)
    path = comp.features.createPath(curves, False)
    if path is None or path.count != len(route['path']):
        raise RuntimeError('%s: the sweep path took %s of %d curves'
                           % (name, 'None' if path is None else path.count,
                              len(route['path'])))

    sg0 = route['path'][0]
    p0 = sg0[1]
    d = _unit([sg0[2][i] - sg0[1][i] for i in range(3)])         # in (s, w, z)
    wd = _unit([-d[1], d[0], 0.0]) if abs(d[0]) + abs(d[1]) > 1e-9 else [0.0, 1.0, 0.0]
    td = [0.0, 0.0, 1.0]
    origin = _p(arm.xyz(p0))
    e1 = adsk.core.Vector3D.create(*arm.dir(wd))
    e2 = adsk.core.Vector3D.create(*arm.dir(td))
    ps = BuildArm.PlaneSketch(comp, name + ' ribbon', origin, e1, e2)
    hw, ht = route['width'] / 2.0, route['thickness'] / 2.0
    ps.loop(armgen.rect_outline(-hw, -ht, hw, ht))
    if ps.sketch.profiles.count != 1:
        raise RuntimeError('%s: ribbon section sketch has %d profiles'
                           % (name, ps.sketch.profiles.count))
    sweeps = comp.features.sweepFeatures
    inp = sweeps.createInput(ps.sketch.profiles.item(0), path,
                             adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    inp.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
    feat = sweeps.add(inp)
    if feat is None or feat.bodies.count != 1:
        raise RuntimeError('%s: sweep produced %s bodies' % (name, None if feat is None
                                                             else feat.bodies.count))
    ps.sketch.isVisible = False
    ps.cplane.isLightBulbOn = False
    sk.isVisible = False
    return feat.bodies.item(0)


# ---------------- EXO-151: the default rounding ----------------

def _outward_normal(face, p):
    """The face's OUTWARD normal at `p`. `getNormalAtPoint` returns the underlying surface's
    normal, so `isParamReversed` has to be applied or half the faces point into the solid."""
    _ok, n = face.evaluator.getNormalAtPoint(p)
    sgn = -1.0 if face.isParamReversed else 1.0
    return [sgn * n.x, sgn * n.y, sgn * n.z]


def edge_convexity(body, edge, eps=0.003):
    """'convex', 'concave' or 'smooth' for one edge.

    Measured here, the obvious test does NOT work: stepping off the edge along -(n1 + n2)
    lands INSIDE the solid for a convex edge AND for a concave one, so it classifies every
    edge the same way (77 of 77 on g_link_12). Stepping along (n1 - n2) does separate them -
    it points out of the solid at a convex edge and into it at a concave one - and it agrees
    with the geometry on the cases that are known by hand (a hole rim is convex, the
    ribbon-to-plate junction is concave, a tangent bend is smooth)."""
    p = edge.pointOnEdge
    f1, f2 = edge.faces.item(0), edge.faces.item(1)
    n1, n2 = _outward_normal(f1, p), _outward_normal(f2, p)
    d = [n1[i] - n2[i] for i in range(3)]
    n = math.sqrt(sum(v * v for v in d))
    if n < 1e-6:
        return 'smooth'
    q = adsk.core.Point3D.create(p.x + eps * d[0] / n, p.y + eps * d[1] / n,
                                 p.z + eps * d[2] / n)
    inside = body.pointContainment(q) == adsk.fusion.PointContainment.PointInsidePointContainment
    return 'concave' if inside else 'convex'


def elbow_edges(body, arm, route, tol=5e-3):
    """Pick the edges the EXO-151 rule rounds, as INDEXES into `body.edges`. Three sets, each
    named by where it is:

      * `inside` - every CONCAVE edge lying in the landing plate's NEAR face, which is where
        the ribbon meets the plate. The actuator cutout's own concave corners sit at mid
        thickness, not on that face, and the insert and screw holes are convex, so this
        predicate picks the elbow and nothing else.
      * `pad` - every CONCAVE edge on the rotor pad's rim cylinder that SPANS the pad's own
        thickness: the corner a ribbon narrower than the pad leaves where its wall runs out
        through the rim. Spanning the thickness is guard EXO-77/84 enforced by construction -
        such a blend lives entirely in 0 <= z <= `PAD_T` and so adds nothing below the pad
        plane, where the stator's stop block turns. A ribbon tangent to the rim leaves a
        SMOOTH edge and is skipped.
      * `outside` - every CONVEX edge lying in the plate's FAR face on the tongue-end plane:
        the outer corner of the same elbow.
    """
    num = route['numbers']
    # EXO-176: the tongue's END is a plane `y_end` ALONG THE TONGUE from the plate axis, not
    # a plane of constant z - those are the same thing only when the tongue is square to the
    # pad axis. The edge's own coordinate along the tongue is what the predicate reads.
    x_t = route.get('x_t') or num['x_s']
    y_t = route.get('y_t') or num['y_s']
    u_r, w_r = route['u'], route['w']
    # EXO-176/177: the tongue's END is a polyline ON THE RIBBON, in the tongue's own frame -
    # one flat line square to the tongue only when the tongue is square to the pad axis. An
    # edge belongs to the `outside` set when its own plate coordinates sit on that polyline.
    end_pts = route['tongue_end']
    end_segs = [('line', end_pts[k], end_pts[k + 1]) for k in range(len(end_pts) - 1)]

    def _on_tongue_end(s, ww, z):
        loc = [u_r[i] * s + w_r[i] * ww + (z if i == 2 else 0.0) for i in range(3)]
        d = [loc[i] - num['t_ab'][i] for i in range(3)]
        return min(armgen._seg_point_dist((armgen.dot(d, x_t), armgen.dot(d, y_t)), sg)
                   for sg in end_segs)

    pad_r = route['pad_r']
    sets = {'inside': [], 'pad': [], 'outside': []}
    rows = []
    for i in range(body.edges.count):
        e = body.edges.item(i)
        s, w, z = arm.swz(e.pointOnEdge)
        c = edge_convexity(body, e)
        L = e.length * 10.0
        z0 = arm.swz(e.startVertex.geometry)[2] if e.startVertex else z
        z1 = arm.swz(e.endVertex.geometry)[2] if e.endVertex else z
        where = None
        if c == 'concave' and abs(s - route['s_near']) < tol:
            where = 'inside'
        elif (c == 'concave' and abs(math.hypot(s, w) - pad_r) < tol
              and -tol <= z <= armgen.PAD_T + tol
              and abs(z1 - z0) > armgen.PAD_T / 2.0):
            where = 'pad'
        elif (c == 'convex' and abs(s - route['s_far']) < tol
              and _on_tongue_end(s, w, z) < tol):
            where = 'outside'
        if where:
            sets[where].append(i)
            rows.append((where, c, L, s, w, z))
    return sets, rows


def _fillet(comp, idx, r):
    """One constant-radius fillet on `idx` edges of the component's single body. Returns the
    feature, or None when Fusion refuses that radius."""
    body = comp.bRepBodies.item(0)
    coll = adsk.core.ObjectCollection.create()
    for i in idx:
        coll.add(body.edges.item(i))
    fs = comp.features.filletFeatures
    inp = fs.createInput()
    inp.isRollingBallCorner = True
    inp.addConstantRadiusEdgeSet(coll, adsk.core.ValueInput.createByReal(r * MM_TO_CM), False)
    try:
        return fs.add(inp)
    except Exception:
        return None


def _ring_frame(arm):
    return (adsk.core.Point3D.create(arm.F.t[0] / 10.0, arm.F.t[1] / 10.0, arm.F.t[2] / 10.0),
            adsk.core.Vector3D.create(*arm.dir([1, 0, 0])),
            adsk.core.Vector3D.create(*arm.dir([0, 1, 0])),
            adsk.core.Vector3D.create(*arm.dir([0, 0, 1])))


FILLET_BACKOFF = 0.9        # EXO-186: how far a radius that probed but would not apply
                            # is backed off before the next try


def fillet_elbows(comp, body, arm, route, name, rep, steps=7, floor=0.05, r_out_cap=None):
    """Apply the two defaults, each already reduced by `sweepgen.elbow_radii`'s guards.

    A radius the GUARDS allow but Fusion itself refuses is bisected down to the largest one
    that builds (7 halvings, so 0.08 mm on a 10 mm round) and the radius that survived is
    reported with the one it was asked for. A fillet is never dropped silently; if nothing
    down to `floor` builds, the build stops."""
    rad = sweepgen.elbow_radii(route)
    sets, rows = elbow_edges(comp.bRepBodies.item(0), arm, route)
    rep('    rounding (EXO-151): inside r %.3f%s, outside r %.3f%s'
        % (rad['r_in'], ' (asked %.1f; %s)' % (rad['r_in_asked'], rad['why_in'])
           if rad['why_in'] else '',
           rad['r_out'], ' (asked %.1f; %s)' % (rad['r_out_asked'], rad['why_out'])
           if rad['why_out'] else ''))
    for where, c, L, s, w, z in rows:
        rep('      %-7s %-7s edge %6.2f mm long at (s %7.2f, w %7.2f, z %7.2f)'
            % (where, c, L, s, w, z))
    applied = []
    r_out = rad['r_out'] if r_out_cap is None else min(rad['r_out'], r_out_cap)
    for where, r0, why in (('inside', rad['r_in'], rad['why_in']),
                           ('pad', rad['r_in'], rad['why_in']),
                           ('outside', r_out, rad['why_out'])):
        sets, _rows = elbow_edges(comp.bRepBodies.item(0), arm, route)
        idx = sets[where]
        if not idx:
            rep('      %-7s: no such edge on this link' % where)
            applied.append({'where': where, 'n': 0, 'r': None, 'asked': None,
                            'cut': False, 'feature': None})
            continue
        feat = _fillet(comp, idx, r0)
        r = r0
        why_ring = ''
        if feat is None:
            lo, hi = 0.0, r0
            for _ in range(steps):
                mid = (lo + hi) / 2.0
                f = _fillet(comp, idx, mid)
                if f is None:
                    hi = mid
                else:
                    f.deleteMe()
                    lo = mid
            if lo <= floor:
                raise RuntimeError('%s: no radius down to %.2f mm fillets the %s set'
                                   % (name, floor, where))
            # EXO-186: the probe's own features are deleted between halvings and direct
            # modelling does not always put the body back exactly as it was, so the edge set
            # is re-derived and the radius backed off until the APPLY takes. The message
            # named its arguments in the wrong order and could never be printed (it raised
            # a TypeError instead); that is fixed here too.
            r, feat = lo, None
            for _ in range(steps):
                sets, _rows = elbow_edges(comp.bRepBodies.item(0), arm, route)
                idx = sets[where]
                feat = _fillet(comp, idx, r) if idx else None
                if feat is not None:
                    break
                r *= FILLET_BACKOFF
                if r <= floor:
                    break
            if feat is None:
                raise RuntimeError('%s: r %.3f fillets the %s set on the probe but no radius '
                                   'down to %.3f takes on the apply' % (name, lo, where, r))
        if comp.bRepBodies.count != 1:
            raise RuntimeError('%s: %d bodies after the %s fillet'
                               % (name, comp.bRepBodies.count, where))
        applied.append({'where': where, 'n': len(idx), 'r': r, 'asked': r0,
                        'cut': r < r0 - 1e-9, 'feature': feat})
        rep('      %-7s: %d edge(s) at r %.3f%s' % (where, len(idx), r,
            '' if r >= r0 - 1e-9 else
            (why_ring if why_ring else
             ' - REDUCED from %.3f: Fusion refuses the larger radius on this geometry' % r0)))
    return comp.bRepBodies.item(0), rad, applied


def worst_ring(comp, arm):
    """The smallest flat seating ring on the body as it now stands, and how many openings it
    has - measured by `hole_ring_probe`, the independent half of the EXO-124 gate."""
    sys.path.insert(0, os.path.join(BuildArm.REPO_DIR, 'tools', 'bridge_scripts'))
    import hole_ring_probe                                    # noqa: E402
    importlib.reload(hole_ring_probe)
    frame = (adsk.core.Point3D.create(arm.F.t[0] / 10.0, arm.F.t[1] / 10.0, arm.F.t[2] / 10.0),
             adsk.core.Vector3D.create(*arm.dir([1, 0, 0])),
             adsk.core.Vector3D.create(*arm.dir([0, 1, 0])),
             adsk.core.Vector3D.create(*arm.dir([0, 0, 1])))
    rings = seating_rings(hole_ring_probe.measure(comp.bRepBodies.item(0), frame, armgen.HOLE_R))
    return (min(h['ring'] for h in rings) if rings else -1.0), len(rings)


def seating_rings(rings):
    """EXO-370: drop the rotor M3 holes' openings in the fused top's JOINT FACE (pad-frame
    plane z = TOP_JOINT_Z -8.0, hole axis at r ROTOR_HOLE_RADIUS). That face bears on the
    bottom - nothing seats a head or a nut there. The counterbore-floor opening (bounded by
    the O6.2 wall, 1.5 by design) is held to the M3 head instead (below). Every other
    opening is a seating face and stays."""
    out = []
    for h in rings:
        rotor_hole = (h['plane'][0] == 'z'
                      and abs(math.hypot(h['at'][0], h['at'][1]) - armgen.ROTOR_HOLE_RADIUS) < 0.1)
        if rotor_hole and abs(h['plane'][1] - armgen.TOP_JOINT_Z) < 0.01:
            continue
        # EXO-383: the M3 head seats on the floor of its O6.2 counterbore, whose wall is the
        # floor's nearest edge, 1.5 past the hole edge BY DESIGN. The M3 head (O5.5) needs 1.15
        # of it, so that opening is held to the head, not to HOLE_FLAT_RING.
        if (rotor_hole and abs(h['bound'] - armgen.ROTOR_CBORE_R) < 0.01
                and abs(h['plane'][1] - armgen.rotor_head_seat_z()) < 0.01):
            if h['bound'] < BuildArm.V3_BOLT_HEAD_R - 1e-6:
                raise RuntimeError('the M3 head (r %.2f) does not fit its counterbore (bound %.3f)'
                                   % (BuildArm.V3_BOLT_HEAD_R, h['bound']))
            continue
        out.append(h)
    return out


def _merge(comp, body, tool_feature, op, step, name):
    """Boolean the freshly extruded tool body into `body` with a COMBINE feature.

    Measured here (link_23's stop block, `tools/.../dbg23b.py`): an extrude whose operation
    is Join and whose `participantBodies` is the main body sometimes leaves the tool as a
    SECOND body even when the two overlap - the interference check reports 57.42 mm3 of
    shared volume and Fusion still does not merge them. `combineFeatures` on the same pair
    merges them every time, so every add and every cut in this builder goes through it.
    """
    tools = adsk.core.ObjectCollection.create()
    for i in range(tool_feature.bodies.count):
        tools.add(tool_feature.bodies.item(i))
    cf = comp.features.combineFeatures
    inp = cf.createInput(body, tools)
    inp.operation = op
    inp.isKeepToolBodies = False
    inp.isNewComponent = False
    cf.add(inp)
    if comp.bRepBodies.count != 1:
        raise RuntimeError('%s: %d bodies after "%s", expected one solid'
                           % (name, comp.bRepBodies.count, step))
    return comp.bRepBodies.item(0)


def build_link(comp, arm, R_ab, t_ab, route, name, rep=None, r_out_cap=None):
    """The whole link: ribbon + rotor pad + fused rotor top + landing plate + stop block, then
    the cuts (EXO-370: the rotor pad IS the rotor top)."""
    g = armgen
    NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
    CUT = adsk.fusion.FeatureOperations.CutFeatureOperation
    eps = 0.5
    FA = arm.F
    FB = FA.child(R_ab, t_ab)                    # the true stator frame: holes and cutout
    # EXO-176: the plate's drawing frame is the TONGUE's - the snapped frame turned
    # until its +Y is the tongue. For a square landing that IS `R_b` (theta = 0).
    FS = FA.child(route.get('R_t') or route['R_b'], t_ab)
    ex, ey = FA.vector([1, 0, 0]), FA.vector([0, 1, 0])
    bx, by = FB.vector([1, 0, 0]), FB.vector([0, 1, 0])
    sx, sy = FS.vector([1, 0, 0]), FS.vector([0, 1, 0])
    z_slab_mid = g.STATOR_UNDERSIDE_Z - g.PAD_T / 2.0
    pad_r = route['pad_r']

    body = build_ribbon(comp, arm, route, name)

    # EXO-375/383: the rotor pad is ROTOR_PAD_T thick (8 default), z 0..ROTOR_PAD_T; with a pad
    # thinner than the ribbon (PAD_T 8) the ribbon is cut down to it inside the r 18 head flat
    sk = BuildArm.PlaneSketch(comp, name + ' rotor pad', FA.point([0, 0, g.ROTOR_PAD_T / 2]), ex, ey)
    # EXO-181: the plain disk unless module B is in the way, in which case one solved chord
    # comes off it. `pad_outline` is None for every link whose pad already clears, so those
    # links draw exactly the circle they always drew.
    if route.get('pad_outline'):
        sk.loop(route['pad_outline'])
    else:
        sk.circle((0, 0), pad_r)
    body = _merge(comp, body, sk.extrude(1, g.ROTOR_PAD_T, NEW), JOIN, 'rotor pad disk', name)

    stop_h = route.get('stop_h', armgen.STOP_H)
    half, ang = route['stop_half'], route['stop_angle']
    # EXO-218: the link stop is ONE solid block of the arc the rule solved - back to what
    # EXO-204 drew. Its two ends are the contact faces.
    sk = BuildArm.PlaneSketch(comp, name + ' stop block',
                              FA.point([0, 0, (BuildArm.JOIN_OVERLAP - stop_h) / 2]), ex, ey)
    sk.loop(g.stop_block_outline(ang, half, g.STOP_R_IN, pad_r))
    body = _merge(comp, body, sk.extrude(1, stop_h + BuildArm.JOIN_OVERLAP, NEW), JOIN,
                  'stop block', name)

    # EXO-370/394: the FUSED ROTOR TOP under the pad, in the module frame A (`armgen.TOP_*`):
    # disk r 18 whose underside is the flat race shoulder -5.001 on r 15.05..16.2 and a 45 deg
    # chamfer up to the rim (r 18, -3.201) - one REVOLVED profile - and the journal O30.1 down to
    # the joint face -8.0; the register sockets are cut below. Each tool overlaps the solid above it (no
    # coincident join face). The EXO-364 key pegs and the EXO-204 tab are gone: nothing keys
    # the link's angle on the rotor beyond the four M3 holes, so URDF q = 0 <-> the link as
    # drawn is set at assembly.
    def _band(label, z_lo, z_hi, draw, n):
        sk_ = BuildArm.PlaneSketch(comp, name + ' ' + label, FA.point([0, 0, (z_lo + z_hi) / 2]),
                                   ex, ey)
        draw(sk_)
        return sk_.extrude(n, z_hi - z_lo, NEW)
    ez = FA.vector([0, 0, 1])
    sk = BuildArm.PlaneSketch(comp, name + ' top disk profile', FA.point([0, 0, 0]), ex, ez)
    r0, j1 = 6.0, BuildArm.JOIN_OVERLAP      # r 0..6 is inside the O13.8 centre hole anyway
    pts = [(r0, j1), (g.TOP_DISK_R, j1), (g.TOP_DISK_R, g.TOP_CHAMFER_Z),
           (g.TOP_SHOULDER_R, g.TOP_SHOULDER_Z), (r0, g.TOP_SHOULDER_Z)]
    segs = []
    for k in range(len(pts)):
        g._line(segs, pts[k], pts[(k + 1) % len(pts)])
    sk.loop(segs)
    axis = sk.sketch.sketchCurves.sketchLines.addByTwoPoints(sk.pt(0.0, -6.0), sk.pt(0.0, 2.0))
    axis.isConstruction = True
    if sk.sketch.profiles.count != 1:
        raise RuntimeError('%s: top disk profile has %d profiles' % (name, sk.sketch.profiles.count))
    rin = comp.features.revolveFeatures.createInput(sk.sketch.profiles.item(0), axis, NEW)
    rin.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
    rf = comp.features.revolveFeatures.add(rin)
    sk.sketch.isVisible = False
    sk.cplane.isLightBulbOn = False
    body = _merge(comp, body, rf, JOIN, 'fused top disk (shoulder + 45 deg chamfer)', name)
    body = _merge(comp, body, _band('top journal', g.TOP_JOINT_Z, g.TOP_SHOULDER_Z + 0.5,
                                    lambda k: k.circle((0, 0), g.TOP_JOURNAL_R), 1),
                  JOIN, 'fused top journal', name)

    sk = BuildArm.PlaneSketch(comp, name + ' landing plate',
                              FS.point([0, 0, z_slab_mid]), sx, sy)
    sk.loop(route['flange'])
    body = _merge(comp, body, sk.extrude(1, g.PAD_T, NEW), JOIN, 'landing plate', name)

    applied = []
    if rep is not None:
        body, _rad, applied = fillet_elbows(comp, body, arm, route, name, rep,
                                            r_out_cap=r_out_cap)

    # EXO-370/383: the four M3 holes and the O13.8 centre hole run through pad AND top (z -8.0 ..
    # ROTOR_PAD_T, eps past both faces); the underside recess r 8.0 up to -5.4 takes the bottom's
    # r 7.8 boss and the M2 heads on it; O6.2 counterbores ROTOR_CBORE_D deep take the M3 heads.
    z_lo_t, z_hi_t = g.TOP_JOINT_Z - eps, max(g.PAD_T, g.ROTOR_PAD_T) + eps
    # EXO-375: the HEAD FLAT. With a pad thinner than the ribbon (PAD_T 8), the ribbon that
    # crosses the pad stands proud of the pad face; inside the fused top's r 18 it is cut down
    # to z ROTOR_PAD_T, so the four M3 heads seat on one plane (ring to the r 18 step 5.4).
    if g.ROTOR_PAD_T < g.PAD_T - 1e-9:
        body = _merge(comp, body, _band('head flat', g.ROTOR_PAD_T, g.PAD_T + eps,
                                        lambda k: k.circle((0, 0), g.TOP_DISK_R), 1),
                      CUT, 'head flat', name)

    def _holes(k):
        for a in g.ROTOR_HOLE_ANGLES:
            k.circle((g.ROTOR_HOLE_RADIUS * math.cos(math.radians(a)),
                      g.ROTOR_HOLE_RADIUS * math.sin(math.radians(a))), g.HOLE_R)
    body = _merge(comp, body, _band('rotor holes', z_lo_t, z_hi_t, _holes,
                                    len(g.ROTOR_HOLE_ANGLES)), CUT, 'rotor holes', name)
    body = _merge(comp, body, _band('centre hole', z_lo_t, z_hi_t,
                                    lambda k: k.circle((0, 0), g.TOP_CENTRE_R), 1),
                  CUT, 'centre hole', name)
    body = _merge(comp, body, _band('boss recess', z_lo_t, g.TOP_RECESS_Z[1],
                                    lambda k: k.circle((0, 0), g.TOP_RECESS_R), 1),
                  CUT, 'boss recess', name)

    def _cbores(k):
        for a in g.ROTOR_HOLE_ANGLES:
            k.circle((g.ROTOR_HOLE_RADIUS * math.cos(math.radians(a)),
                      g.ROTOR_HOLE_RADIUS * math.sin(math.radians(a))), g.ROTOR_CBORE_R)
    body = _merge(comp, body, _band('head counterbores', g.rotor_head_seat_z(), z_hi_t, _cbores,
                                    len(g.ROTOR_HOLE_ANGLES)), CUT, 'head counterbores', name)
    # EXO-394: eight O3.3 x 2.1 register sockets from the joint face up, for the bottom's pegs
    def _sockets(k):
        for c in g.top_peg_centres():
            k.circle(c, g.TOP_SOCKET_R)
    body = _merge(comp, body, _band('register sockets', z_lo_t, g.TOP_JOINT_Z + g.TOP_SOCKET_D,
                                    _sockets, len(g.TOP_PEG_ANGLES)), CUT, 'register sockets', name)
    if g.ROTOR_LABEL_ON:
        body = _rotor_label(comp, body, FA, ex, ey, name)

    sk = BuildArm.PlaneSketch(comp, name + ' stator holes',
                              FB.point([0, 0, z_slab_mid]), bx, by)
    for a in g.INSERT_ANGLES:
        sk.circle((g.INSERT_RADIUS * math.cos(math.radians(a)),
                   g.INSERT_RADIUS * math.sin(math.radians(a))), g.HOLE_R)
    body = _merge(comp, body, sk.extrude(len(g.INSERT_ANGLES), g.PAD_T + 2 * eps, NEW),
                  CUT, 'stator holes', name)
    # EXO-187/EXO-193: the actuator OPENING - the XL330's footprint grown by
    # `ACT_OPEN_MARGIN` (0.1 per side since EXO-419; 0.2 in EXO-415, 0 in EXO-193), in the
    # true stator frame B, cut THROUGH the whole exclusion volume (the actuator's back up to
    # the stator underside), not to the plate's depth. `sweepgen.actuator_check` has already
    # refused any route whose ribbon, collar or rotor join sits in that volume, so what this
    # removes is landing plate and nothing else.
    ax0, ax1, ay0, ay1 = g.actuator_rect(g.ACT_OPEN_MARGIN)
    cz, c_len = g.actuator_cut_z(g.ACT_OPEN_MARGIN, eps)
    sk = BuildArm.PlaneSketch(comp, name + ' actuator opening', FB.point([0, 0, cz]), bx, by)
    sk.loop(g.rect_outline(ax0, ay0, ax1, ay1))
    body = _merge(comp, body, sk.extrude(1, c_len, NEW), CUT, 'actuator opening', name)
    # EXO-341: the assembly mark on the stator-pad face (z = STATOR_UNDERSIDE_Z), a radial
    # line at ALIGN_MARK_ANGLE in the true stator frame B, ALIGN_MARK_D deep. The tool runs
    # 0.1 above the face so the cut is not coplanar with it; nothing of the link is there
    # (r < 27.6 above the underside is the stator's room).
    ca, sa = math.cos(math.radians(g.ALIGN_MARK_ANGLE)), math.sin(math.radians(g.ALIGN_MARK_ANGLE))
    mz0, mz1 = g.STATOR_UNDERSIDE_Z - g.ALIGN_MARK_D, g.STATOR_UNDERSIDE_Z + 0.1
    sk = BuildArm.PlaneSketch(comp, name + ' assembly mark', FB.point([0, 0, (mz0 + mz1) / 2]),
                              FB.vector([ca, sa, 0]), FB.vector([-sa, ca, 0]))
    sk.loop(g.rect_outline(g.ALIGN_MARK_R_IN, -g.ALIGN_MARK_W / 2,
                           g.ALIGN_MARK_R_OUT, g.ALIGN_MARK_W / 2))
    body = _merge(comp, body, sk.extrude(1, mz1 - mz0, NEW), CUT, 'assembly mark', name)
    return body


def _rotor_label(comp, body, FA, ex, ey, name):
    """EXO-375: `armgen.rotor_label_text()` ("c1.0" / "c1.5" / "c2.0") debossed ROTOR_LABEL_D into the rotor pad's outer face
    z ROTOR_PAD_T, centred at ROTOR_LABEL_AT (r 13.0, 45 deg: between two M3 counterbores), readable from outside. Glyphs extruded as new bodies from 0.1 above the
    face down to the label floor, then combine-cut; refused if a glyph leaves r 8..18."""
    g = armgen
    text = g.rotor_label_text()
    r, a = g.ROTOR_LABEL_AT
    cx, cy = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
    z_lo, z_hi = g.ROTOR_PAD_T - g.ROTOR_LABEL_D, g.ROTOR_PAD_T + 0.1
    sk = BuildArm.PlaneSketch(comp, name + ' pad label', FA.point([0, 0, z_lo]), ex, ey)
    n = sk.sketch.xDirection.crossProduct(sk.sketch.yDirection)
    ez = ex.crossProduct(ey)
    if n.dotProduct(ez) < 0.99:
        raise RuntimeError('%s: label sketch normal is not the pad normal (%.3f)' % (name, n.dotProduct(ez)))
    h = g.ROTOR_LABEL_H
    ti = sk.sketch.sketchTexts.createInput2(text, h * BuildArm.MM_TO_CM)
    ti.setAsMultiLine(sk.pt(cx - 4.0 * h / 2, cy - h), sk.pt(cx + 4.0 * h / 2, cy + h),
                      adsk.core.HorizontalAlignments.CenterHorizontalAlignment,
                      adsk.core.VerticalAlignments.MiddleVerticalAlignment, 0)
    st = sk.sketch.sketchTexts.add(ti)
    keep = set(comp.bRepBodies.item(i).entityToken for i in range(comp.bRepBodies.count))
    feats = comp.features.extrudeFeatures
    ein = feats.createInput(st, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ein.setDistanceExtent(False, adsk.core.ValueInput.createByReal((z_hi - z_lo) * BuildArm.MM_TO_CM))
    feats.add(ein)
    tools = adsk.core.ObjectCollection.create()
    for i in range(comp.bRepBodies.count):
        b = comp.bRepBodies.item(i)
        if b.entityToken in keep:
            continue
        for k in range(b.vertices.count):
            q = FA.local(b.vertices.item(k).geometry)
            if not (8.0 <= math.hypot(q[0], q[1]) <= 18.0
                                      and z_lo - 1e-3 <= q[2] <= z_hi + 1e-3):
                raise RuntimeError('%s: label glyph vertex at r %.3f z %.3f'
                                   % (name, math.hypot(q[0], q[1]), q[2]))
        tools.add(b)
    if tools.count == 0:
        raise RuntimeError('%s: the label made no glyph bodies' % name)
    cin = comp.features.combineFeatures.createInput(body, tools)
    cin.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
    cin.isKeepToolBodies = False
    comp.features.combineFeatures.add(cin)
    if comp.bRepBodies.count != 1:
        raise RuntimeError('%s: %d bodies after the label' % (name, comp.bRepBodies.count))
    sk.sketch.isVisible = False
    sk.cplane.isLightBulbOn = False
    return comp.bRepBodies.item(0)


def actuator_bodies(wrapper_occ):
    """EXO-201: the REAL actuator bodies under a placed module wrapper - every body whose
    component or body name carries `BuildArm.ACTUATOR_BODY_TOKEN`.

    EXO-187 built a proxy box beside each module because `exo-joint.md` was read as saying
    the module carries no actuator. It does: `XL330 v2/Body1`, measured (EXO-201) at exactly
    the proxy's footprint and back - x -10..10, y -9.5..24.5, back at z -36.5 - and carrying
    on up to z -10.5 inside the stator. The proxy only duplicated it (z-fighting in the
    viewport), so it is gone and this reports the real thing instead.
    Returns [(component_name, body), ...]."""
    out = []

    def walk(occ):
        for i in range(occ.childOccurrences.count):
            sub = occ.childOccurrences.item(i)
            nm = sub.component.name
            for bi in range(sub.bRepBodies.count):
                b = sub.bRepBodies.item(bi)
                if BuildArm.ACTUATOR_BODY_TOKEN in nm or BuildArm.ACTUATOR_BODY_TOKEN in b.name:
                    out.append((nm, b))
            walk(sub)

    walk(wrapper_occ)
    return out


def body_box_in(body, occ):
    """Axis-aligned box of `body` in `occ`'s own frame, mm: ((x0,y0,z0),(x1,y1,z1))."""
    m = occ.transform.copy()
    if not m.invert():
        raise RuntimeError('occurrence matrix not invertible: %s' % occ.component.name)
    tbm = adsk.fusion.TemporaryBRepManager.get()
    tmp = tbm.copy(body)
    tbm.transform(tmp, m)
    bb = tmp.boundingBox
    return ((bb.minPoint.x * 10.0, bb.minPoint.y * 10.0, bb.minPoint.z * 10.0),
            (bb.maxPoint.x * 10.0, bb.maxPoint.y * 10.0, bb.maxPoint.z * 10.0))


def delete_previous(root, arm_name=ARM_NAME):
    gone = []
    victims = [root.occurrences.item(i) for i in range(root.occurrences.count)
               if root.occurrences.item(i).component.name == arm_name
               or root.occurrences.item(i).component.name.startswith(arm_name + ' (')]
    for occ in victims:
        n = occ.component.name
        if not occ.deleteMe():
            raise RuntimeError('could not delete previous component "%s"' % n)
        gone.append(n)
    return gone


def find_body(design, comp_name, body_name=None):
    for c in design.allComponents:
        if c.name == comp_name:
            for i in range(c.bRepBodies.count):
                b = c.bRepBodies.item(i)
                if body_name is None or b.name == body_name:
                    return b
    return None


def deviation(design, new_body, old_body):
    """How far the general method's link is from the one the current generator builds.

    `measureManager.measureMinimumDistance` refuses every argument in this Fusion build
    (measured: body/body, vertex/body, point/body and face/face all raise "invalid
    argument"), so the deviation is reported as two numbers that only need APIs that work:

      * the SYMMETRIC DIFFERENCE volume - how much material the two solids do not share,
        from `analyzeInterference` (shared) and the two volumes;
      * the largest distance from a vertex of the new link to the NEAREST VERTEX of the old
        one, which is an upper bound on how far a corner moved.
    """
    if old_body is None or new_body is None:
        return None
    coll = adsk.core.ObjectCollection.create()
    coll.add(new_body)
    coll.add(old_body)
    shared = 0.0
    inp = design.createInterferenceInput(coll)
    inp.areCoincidentFacesIncluded = False
    res = design.analyzeInterference(inp)
    if res is not None:
        for i in range(res.count):
            ib = res.item(i).interferenceBody
            if ib is not None:
                shared += ib.volume * 1000.0

    def verts(b):
        out = []
        for i in range(b.vertices.count):
            p = b.vertices.item(i).geometry
            out.append((p.x * 10.0, p.y * 10.0, p.z * 10.0))
        return out

    vn, vo = verts(new_body), verts(old_body)
    worst = 0.0
    for p in vn:
        worst = max(worst, min(math.sqrt(sum((p[k] - q[k]) ** 2 for k in range(3)))
                               for q in vo))
    v_new, v_old = new_body.volume * 1000.0, old_body.volume * 1000.0
    bbn, bbo = new_body.boundingBox, old_body.boundingBox
    bbox = max(abs(getattr(bbn.minPoint, k) - getattr(bbo.minPoint, k)) for k in 'xyz')
    bbox = max(bbox, max(abs(getattr(bbn.maxPoint, k) - getattr(bbo.maxPoint, k))
                         for k in 'xyz')) * 10.0
    return {'v_new': v_new, 'v_old': v_old, 'shared': shared,
            'symdiff': v_new + v_old - 2 * shared, 'worst_vertex': worst, 'bbox': bbox,
            'faces_new': new_body.faces.count, 'faces_old': old_body.faces.count}


def build_arm(spec, app, ui, design, log=None, interactive=False):
    """Build one ARM SPEC (EXO-218). `build_general` is this with G1_SPEC and
    `build_luna` is this with LUNA_SPEC; nothing below reads a joint or a link name."""
    report = []

    def rep(line):
        report.append(line)
        if log is not None:
            log(line)

    if not design:
        raise RuntimeError('Open a Fusion design first.')
    urdf = spec['urdf']
    if not os.path.isfile(urdf):
        raise RuntimeError('URDF not found: %s' % urdf)
    joints, by_child = armgen.parse_urdf(urdf)
    origin, _ = armgen.joint_world_pose(spec['origin_at'], joints, by_child)
    off = spec.get('offset', (0.0, 0.0, 0.0))
    shift = [origin[i] - off[i] for i in range(3)]

    links = sweepgen.spec_links(spec)
    order = [j['label'] for j in sweepgen.spec_joints(spec)]
    _seed, _labels, table = sweepgen.seed_frames(spec, joints, by_child, shift)
    seed, _combo, slides, srows = sweepgen.solve_axis_signs(
        spec, joints, by_child, shift, links, table, rep, order)
    report_slides(rep, seed, order, slides, srows)
    JOINTS_ALL = list(spec['fixed']) + [dict(j, y_hint=table[j['label']]['y_hint'],
                                             axis_sign=1) for j in spec['derived']]
    for j in JOINTS_ALL[len(spec['fixed']):]:
        _t, ax = armgen.joint_world_pose(j['name'], joints, by_child)
        z = [seed[j['label']][0][i][2] for i in range(3)]
        j['axis_sign'] = 1 if armgen.dot(z, ax) > 0 else -1
    rep('')
    # EXO-214: the chain is spun and sized ONCE, each link with its own joint's link stop,
    # so the spin table's `refused` is the same refusal the build reports.
    stops = sweepgen.joint_stops(
        spec, joints, links, dict((j['label'], j['axis_sign']) for j in JOINTS_ALL))
    frames, spins = sweepgen.solve_spins(seed, links, stops=stops)

    by_label = {j['label']: j for j in JOINTS_ALL}
    specs, refused, limits = [], [], []
    # EXO-208: every joint in LIMITED_JOINTS is SIZED here, whether or not a link carries
    # it. `unbuilt` is the ones with no rotor pad in this arm (J7, the end of the chain).
    rotor_joints = set(L[1] for L in links)
    unbuilt = [(lab, armgen.limited_joint(dict(joints[by_label[lab]['name']],
                                               name=by_label[lab]['name']),
                                          by_label[lab]['axis_sign'], armgen.ROTOR_PAD_R))
               for lab in spec['limited'] if lab not in rotor_joints and lab in by_label]
    for name, a, b, pad_r, width in links:
        RA, tA = frames[a]
        RB, tB = frames[b]
        R_ab, t_ab = armgen.relative_frame(RA, tA, RB, tB)
        # EXO-204: the link stop of a LIMITED joint is not a number anybody picked - it is
        # the arc and the angle that give joint `a` exactly the travel its URDF states.
        stop_kw, lj = dict(stops.get(a, {})), None
        if a in spec['limited']:
            jd = by_label[a]
            lj = armgen.limited_joint(dict(joints[jd['name']], name=jd['name']),
                                      jd['axis_sign'], pad_r)
        try:
            specs.append((name, a, b, R_ab, t_ab,
                          sweepgen.sweep_route(R_ab, t_ab, pad_r, width, **stop_kw)))
            if lj is not None:
                limits.append((name, a, lj, specs[-1][5]))
        except ValueError as e:
            # EXO-82 still holds: a pair the method cannot size is a pair nobody builds. It
            # no longer stops the arm, because the links that DO size still have to be built
            # and measured; the refusal is reported by name, with every leg it tried.
            refused.append((name, a, b, pad_r, width, str(e)))

    for line in ['URDF: ' + urdf,
                 'Method: one ribbon swept on a 3D centre line + one wrapping landing plate '
                 '(EXO-144), spun by the encoder rule (EXO-150). Ribbon thickness %.1f mm, '
                 'bend r %.1f, wall r min %.1f, land web '
                 '%.1f mm - defaults, none measured off a hand-built link.'
                 % (sweepgen.RIBBON_T, sweepgen.BEND_R, sweepgen.WALL_R_MIN,
                    sweepgen.LAND_WEB),
                 'Origin at: %s%s' % (spec['origin_at'],
                                       '' if max(abs(v) for v in off) < 1e-12 else
                                       ', parked %+.0f %+.0f %+.0f mm off it so both arms '
                                       'fit in one document - display only, every axis '
                                       'check adds it back (EXO-218)'
                                       % tuple(1000.0 * v for v in off)),
                 'Links: ' + ', '.join(
                     '%s %.0f mm wide on a r %.0f pad%s'
                     % (L['name'], L['width'], L['pad_r'],
                        '' if not L.get('width_why') else ' [%s]' % L['width_why'])
                     for L in spec['links']), '']:
        rep(line)
    rep('Module rotation about its own axis, FIXED BY THE LINK (EXO-150): +Y - the encoder / '
        'connector side - faces the direction the link arrives from at that stator pad, and '
        'the landing plate then extends the pad outward exactly where the encoder overhangs. '
        'The hole / cutout solve is a CHECK.')
    for j in JOINTS_ALL:
        f = spins[j['label']]
        rep('  %-3s %+9.3f deg - %s' % (j['label'], f['spin'], f['why']))
        if f['margin'] is not None:
            rep('      encoder +Y is %.2e deg off the AXIS-ALIGNED direction it was '
                'snapped to (%+.1f deg, EXO-186), the tongue the route solved is %+.3f deg '
                'off the pad axis and the one it draws there is %+.3f; landing plate disk '
                'r %.4f (grown from r %.1f, EXO-186)'
                % (f['residual_deg'], f.get('spin_snap_deg', 0.0),
                   f.get('tongue_solved_deg', 0.0), f.get('tongue_deg', 0.0),
                   f.get('land_r', armgen.STATOR_PAD_R), armgen.STATOR_PAD_R))
            rep('      CHECK: smallest rotor ring %.3f mm, smallest stator ring %.3f mm '
                '(need %.2f); opening web %.3f mm (need %.1f); opening reach %.3f mm; '
                'landing y %.3f, drop %+.3f mm; nearest non-plate BODY %.3f mm from '
                'the XL330 (the %s, need %.1f), EXO-183 collar %.3f mm off it (a blend '
                'tangent to the plate, need 0.0 - EXO-193)'
                % (f['rotor_ring'], f['margin'], armgen.HOLE_FLAT_RING, f['web'],
                   sweepgen.LAND_WEB, f['cut_reach'], f['y_land'], f['drop'],
                   f['act_gap'], f['act_part'], armgen.MIN_CLEARANCE,
                   -1.0 if f['act_collar_gap'] is None else f['act_collar_gap']))
        if f.get('refused'):
            rep('      the spin is fixed, but the ROUTE refuses this pair: %s'
                % f['refused'])
    rep('')
    rep('Which LANDING primitive the pair gets (EXO-176) and where the level change is '
        'taken inside it (EXO-158). The landing plate is a flat disk, so its plane carries '
        'two free directions; `square` is EXO-144\'s, with the tongue pinned to the rotor '
        'pad axis, and `angled` aims the tongue FROM THE DISK CENTRE AT THE POINT WHERE THE '
        'RIBBON ENTERS THE PLANE, so the pair may spend its lateral offset inside the plate '
        'plane, where it costs no straight run. Both are sized in full, %s first, and the '
        'first feasible one is taken - so a pair the square landing can take keeps exactly '
        'the route it had.' % ' then '.join(sweepgen.LANDINGS))
    for name, a, b, R_ab, t_ab, route in specs:
        rep('  %-8s %s landing, tongue %+.3f deg off the pad axis, %.2f mm wide, reaching '
            '%.2f mm from the plate axis' % (name, route['landing'], route['tongue_deg'],
                                             2.0 * route['tongue_half_w'], route['y_end']))
        rep('      %s' % route['tongue_why'])
        rep('      level change: %s' % route['level_taken_by'])
        for ln, o, L, e in route['landing_tried']:
            rep('      %-6s landing, %-7s leg: %s'
                % (ln, o, 'route %.3f mm long' % L if L else e))
        rep('      opening web %.3f mm along the tongue, true planar gap %.3f mm (need %.1f); '
            'the plate is %s from axis A below z %.1f (need %.1f outside r %.1f)'
            % (route['cut_web'], route['cut_gap'], sweepgen.LAND_WEB,
               'never below it' if route['clearance_plate'] is None
               else '%.3f mm' % route['clearance_plate'], sweepgen.CLEAR_Z,
               armgen.MIN_CLEARANCE, sweepgen.CLEAR_R))
        rep('      rotor pad vs module B (EXO-181): %s' % route['pad_trim_why'])
        rep('      ribbon vs module B: %s bare, %s with the EXO-183 collar the EXO-151 '
            'fillet puts on it where it enters the plate (need %.1f and %.1f)'
            % ('never alongside it' if route['clearance_b'] is None
               else '%.3f mm' % route['clearance_b'],
               'n/a' if route['clearance_b_collar'] is None
               else '%.3f mm' % route['clearance_b_collar'],
               sweepgen.CLEAR_R + armgen.MIN_CLEARANCE, sweepgen.CLEAR_R))
    for name, a, b, pad_r, width, why in refused:
        rep('  %-8s REFUSED (%s -> %s, ribbon %.0f mm on a r %.0f pad): %s'
            % (name, a, b, width, pad_r, why))
    rep('')
    report_limits(rep, spec, limits, order, unbuilt)

    root = design.rootComponent
    gone = delete_previous(root, spec['name'])
    rep('Deleted previous: %s' % (', '.join(gone) if gone else 'none'))
    # EXO-225: every module this arm places is the v3 copy - the EXO-208 narrowed stop AND
    # the reworked rotor pair (horn screws on the lower part alone, the four M3 bolts
    # joining upper to lower). Made here if the document does not hold it yet.
    # `sensor module 2` itself is never modified - see `BuildArm.ensure_v3_module` for the
    # recipe, which is the whole record of how the copy was made.
    module_comp, how = BuildArm.ensure_v3_module(design, rep)
    rep('Module placed at every joint: %s (%s)' % (module_comp.name, how))
    rep('')

    identity = adsk.core.Matrix3D.create()
    arm_occ = root.occurrences.addNewComponent(identity)
    arm_occ.component.name = BuildArm.unique_name(design, spec['name'])
    arm_comp = arm_occ.component
    rep('Component: ' + arm_comp.name)
    rep('')

    built = []
    for name, a, b, R_ab, t_ab, route in specs:
        RA, tA = frames[a]
        gname = 'g_' + name
        arm = Arm(RA, [v * 1000.0 for v in tA], route['u'], route['w'])
        # EXO-184: `sweepgen.elbow_radii` caps each EXO-151 round by the geometry the round
        # IDEALLY sweeps, and that is not always what Fusion sweeps - the tongue's wall, its
        # blend and the disk rim are TANGENT to one another by construction, so a round on an
        # edge that ends on that chain rolls along the whole chain. On link_67 it took an
        # insert's ring from 6.150 to 1.791 mm. The guard is therefore MEASURED on the
        # finished body, and a round that breaks it is halved and the link built again: this
        # design is DIRECT modelling and a fillet cannot be deleted once the screw holes are
        # cut over it (ASM_REM_BAD_FACE), so the rollback is the whole link.
        # EXO-218: a link Fusion will not BUILD stops that link, not the arm - the same
        # rule EXO-82 gives a pair the router refuses. The five links that do build still
        # have to be measured, so the failure is caught, named with its number, and the
        # half-made occurrence removed.
        cap, occ, body, flines = None, None, None, []
        try:
          for attempt in range(ROUND_TRIES):
            occ = arm_comp.occurrences.addNewComponent(identity)
            occ.component.name = BuildArm.unique_name(design, gname)
            flines = []
            body = build_link(occ.component, arm, R_ab, t_ab, route, gname, flines.append,
                              r_out_cap=cap)
            ring, n_open = worst_ring(occ.component, arm)
            if ring >= armgen.HOLE_FLAT_RING - 1e-6:
                if attempt:
                    flines.append('      EXO-184: the outer round was capped at r %.3f and '
                                  'built on attempt %d; smallest ring %.3f mm over %d '
                                  'openings' % (cap, attempt + 1, ring, n_open))
                break
            asked = sweepgen.elbow_radii(route)['r_out'] if cap is None else cap
            cap = asked / 2.0
            rep('    %s: the EXO-151 outer round at r %.3f leaves a screw %.3f mm of flat '
                'ring over %d openings (need %.2f) - EXO-184: halving it to r %.3f and '
                'building the link again' % (gname, asked, ring, n_open,
                                             armgen.HOLE_FLAT_RING, cap))
            occ.deleteMe()
          else:
            raise RuntimeError('%s: no outer round down to r %.3f seats every screw'
                               % (gname, cap))
        except Exception as _e:
            if occ is not None:
                try:
                    occ.deleteMe()
                except Exception:
                    pass
            refused.append((name, a, b, pad_r, width,
                            'the route sizes but Fusion will not build it: %s' % _e))
            rep('    %s: NOT BUILT - %s' % (gname, _e))
            continue
        body.name = gname
        built.append((name, gname, body, route, arm))
        rep('%s -> %s: %d features, %d faces, volume %.0f mm3'
            % (name, gname, occ.component.features.count, body.faces.count,
               body.volume * 1000.0))
        for line in sweepgen.describe(route):
            rep('    ' + line)
        if spins.get(b, {}).get('refused'):
            rep('    NOTE: %s\'s spin check REFUSED the route (%s); the link is built at the '
                'tongue-0 spin the chain walked on with, and its opening web is measured '
                'against the actuator where it really is (EXO-187/192)'
                % (b, spins[b]['refused']))
        for line in flines:
            rep(line)

    wrappers = []
    for j in JOINTS_ALL:
        R, t = frames[j['label']]
        m = BuildArm.matrix_from_frame(R, [v * BuildArm.M_TO_CM for v in t])
        wr = arm_comp.occurrences.addNewComponent(m)
        wr.component.name = BuildArm.unique_name(design, 'G%s_%s' % (j['label'], j['name']))
        BuildArm.set_occ_matrix(wr, m, arm_occ)
        # EXO-201: the ROOT-context proxy of the wrapper. A NATIVE occurrence's children and
        # their body geometry do not read in world coordinates (see BuildArm.occ_matrix), and
        # `isLightBulbOn = True` does not stick on one either - measured: after the build the
        # module read bulb=False and no actuator rendered, which is most of what "the
        # actuators look wrong" was.
        wrp = BuildArm.occ_root_proxy(wr, arm_occ)
        mod_occ = wr.component.occurrences.addExistingComponent(module_comp, m)
        mod_occ.isLightBulbOn = True
        try:
            mod_occ.createForAssemblyContext(wrp).isLightBulbOn = True
        except RuntimeError:
            pass
        # No proxy body: the placed module already carries the real actuator
        # (`XL330 v2/Body1`), which `check_interference` sees as a body of this wrapper -
        # so XL330-vs-own-stator is module-internal and XL330-vs-link is a FAIL, exactly the
        # grouping EXO-187's proxy was built to get.
        wrappers.append((j['label'], wrp))
    rep('')
    rep('EXO-201: the XL330 is an exclusion volume and the module CARRIES it - the proxy body '
        'EXO-187 built beside every module is gone (it only duplicated the real one and '
        'z-fought with it in the viewport). The real actuator bodies below take part in the '
        'interference check; measured, `XL330 v2/Body1` is x %.1f..%.1f, y %.1f..%.1f from its '
        'back z %.1f on up (past the stator underside %.1f, where it is inside the stator\'s '
        'own solid) - the proxy rectangle to 0.000 mm. Each landing plate\'s opening is that '
        'footprint grown by ACT_OPEN_MARGIN %.1f (EXO-419: 0.1 per side; 0.2 in EXO-415, 0 in EXO-193), cut through the whole volume (z %.1f..%.1f); every BODY of '
        'every link was checked against the volume grown by MIN_CLEARANCE %.1f in pure Python '
        'before the build and would have been refused by name; the EXO-183 collar is a blend '
        'tangent to the plate, so its bar is the XL330\'s own box (EXO-193).'
        % (armgen.CUTOUT_X[0], armgen.CUTOUT_X[1], armgen.CUTOUT_Y[0], armgen.CUTOUT_Y[1],
           armgen.ACT_Z[0], armgen.STATOR_UNDERSIDE_Z, armgen.ACT_OPEN_MARGIN,
           armgen.actuator_volume()[4], armgen.actuator_volume()[5], armgen.MIN_CLEARANCE))
    n_act = 0
    for lab, wr in wrappers:
        acts = actuator_bodies(wr)
        n_act += len(acts)
        for cname, b in acts:
            lo, hi = body_box_in(b, wr)
            try:
                vol = '%.1f mm3' % (b.volume * 1000.0)
            except RuntimeError:
                vol = 'n/a'
            rep('  %-3s actuator body %s/%s in the module frame x %.3f..%.3f y %.3f..%.3f '
                'z %.3f..%.3f, volume %s'
                % (lab, cname, b.name, lo[0], hi[0], lo[1], hi[1], lo[2], hi[2], vol))
    for name, gname, body, route, _arm in built:
        rep('  %-9s opening takes %.0f mm3 off the plate (%s); nearest BODY: the '
            '%s, %.3f mm from the XL330 (need %.1f); EXO-183 collar %.3f mm off it; worst '
            'clearance to each part\'s own bar %+.3f (the %s); cable '
            'exit along +Y below the plate: %s'
            % (gname, route['act_cut_mm3'],
               'inside the plate outline' if route['act_opening_in_plate']
               else 'BREAKS OUT of the plate outline',
               route['act_gap_part'], route['act_gap'], armgen.MIN_CLEARANCE,
               -1.0 if route['act_collar_gap'] is None else route['act_collar_gap'],
               route['act_clear'], route['act_part'],
               'nothing there' if route['cable_exit'] is None
               else '%.3f mm to the %s' % (route['cable_exit'], route['cable_exit_part'])))

    # ---- module axis vs the URDF joint, on the built document ----
    rep('')
    rep('Module axis vs the URDF joint. With EXO-170 the origin is no longer ON the joint '
        'origin: it is on the joint AXIS, `slide` mm along it. So what is checked is the '
        'axis direction and the PERPENDICULAR distance from the module origin to the URDF '
        'axis line - the two numbers the kinematics depend on - plus the slide itself.')
    for j in JOINTS_ALL:
        t_w, ax_w = armgen.joint_world_pose(j['name'], joints, by_child)
        R, t = frames[j['label']]
        z = [R[i][2] for i in range(3)]
        want = [j['axis_sign'] * v for v in ax_w]
        ang = math.degrees(math.acos(max(-1.0, min(1.0, armgen.dot(z, want)))))
        d = [((t[i] + shift[i]) - t_w[i]) * 1000.0 for i in range(3)]
        along = armgen.dot(d, z)
        perp = math.sqrt(max(0.0, armgen.dot(d, d) - along ** 2))
        rep('  %-3s axis vs URDF %.2e deg, origin off the URDF AXIS LINE %.2e mm, '
            'slide along it %+.3f mm (solved %+.3f)'
            % (j['label'], ang, perp, along, slides.get(j['label'], 0.0)))

    # ---- EXO-151 guards 1-3, measured on the FILLETED bodies ----
    sys.path.insert(0, os.path.join(BuildArm.REPO_DIR, 'tools', 'bridge_scripts'))
    import hole_ring_probe                                    # noqa: E402
    importlib.reload(hole_ring_probe)
    rep('')
    rep('EXO-151 guards, measured on the filleted solids:')
    for name, gname, body, route, arm in built:
        frame = (adsk.core.Point3D.create(arm.F.t[0] / 10.0, arm.F.t[1] / 10.0,
                                          arm.F.t[2] / 10.0),
                 adsk.core.Vector3D.create(*arm.dir([1, 0, 0])),
                 adsk.core.Vector3D.create(*arm.dir([0, 1, 0])),
                 adsk.core.Vector3D.create(*arm.dir([0, 0, 1])))
        rings = seating_rings(hole_ring_probe.measure(body, frame, armgen.HOLE_R))
        rot = [h for h in rings if h['plane'][0] == 'z']
        sta = [h for h in rings if h['plane'][0] == 's']
        rad = sweepgen.elbow_radii(route)
        # guard 1 (EXO-77/84): nothing new below the rotor pad plane inside r 27. The pad set
        # only ever holds edges that span the pad's own thickness, so no blend there reaches
        # below z = 0; this is the lowest material the body actually has inside that radius.
        zmin, at = None, None
        for i in range(body.vertices.count):
            q = arm.swz(body.vertices.item(i).geometry)
            if math.hypot(q[0], q[1]) <= armgen.STATOR_PAD_R + 1e-6:
                if zmin is None or q[2] < zmin:
                    zmin, at = q[2], q
        # guard 2 (EXO-79): the plate's far face is module B's seating face and must stay flat
        # where it crosses the rim; the outer round starts this far from the plate axis.
        start = route.get('y_round', route['y_end']) - rad['r_out']
        rep('  %s' % gname)
        rep('    guard 1 (EXO-77/84): lowest material inside r %.1f of the pad axis z %.3f mm '
            '(EXO-370/394: the fused rotor top\'s joint face is z %.1f, inside module A; '
            'the link stop block %.1f mm deep; no blend is below the pad plane '
            'by construction)' % (armgen.STATOR_PAD_R, zmin, armgen.TOP_Z_MIN, armgen.STOP_H))
        rep('    guard 2 (EXO-79): the outer round starts %.3f mm from the plate axis, '
            'need >= %.1f + %.1f = %.1f -> %s'
            % (start, armgen.STATOR_PAD_R, sweepgen.RIM_MARGIN,
               armgen.STATOR_PAD_R + sweepgen.RIM_MARGIN,
               'OK' if start >= armgen.STATOR_PAD_R + sweepgen.RIM_MARGIN - 1e-9 else 'FAIL'))
        rep('    guard 3 (EXO-124/128): %d openings measured on the filleted body; smallest '
            'rotor-pad ring %.3f mm (nan: every M3 opening is a counterbore floor, held to '
            'the M3 head, EXO-383), smallest landing-plate ring %.3f mm, need %.2f -> %s'
            % (len(rings), min([h['ring'] for h in rot] or [float('nan')]),
               min(h['ring'] for h in sta),
               armgen.HOLE_FLAT_RING,
               'OK' if min(h['ring'] for h in rings) >= armgen.HOLE_FLAT_RING - 1e-6
               else 'FAIL'))
        worst = min(rings, key=lambda h: h['ring'])
        rep('      worst opening: %s %.3f at (s %.2f, w %.2f, z %.2f), bound %.3f mm from the '
            'axis' % (worst['plane'][0], worst['plane'][1], worst['at'][0], worst['at'][1],
                      worst['at'][2], worst['bound']))

    # ---- interference over the whole general arm ----
    rep('')
    n_bodies, hits, how = BuildArm.check_interference(design, arm_occ)
    fails = [h for h in hits if h[0] == 'FAIL']
    slivers = [h for h in hits if h[0] == 'sliver']
    internal = [h for h in hits if h[0] == 'module-internal']
    clamps = [h for h in hits if h[0] == 'race clamp']
    rep('Interference: %d bodies (incl. %d real XL330 bodies), %d hit(s): %d FAIL, '
        '%d sliver(s), %d module-internal, %d race clamp (EXO-373) - HOW: %s'
        % (n_bodies, n_act, len(hits), len(fails), len(slivers), len(internal), len(clamps), how))
    for kind, a, b, vol, bb in clamps:
        rep('    race clamp %s <-> %s volume=%.3f mm3 bbox=%s' % (a, b, vol, BuildArm.fmt_bbox(bb)))
    px_hits = [h for h in hits if BuildArm.ACTUATOR_BODY_TOKEN in (h[1] + h[2])]
    rep('    hits involving an XL330: %d%s'
        % (len(px_hits), '' if not px_hits else ' - ' + '; '.join(
            '%s %s <-> %s %.2f mm3' % (k, a, b, v) for k, a, b, v, _bb in px_hits)))
    for kind, a, b, vol, bb in fails:
        rep('    FAIL %s <-> %s volume=%.1f mm3 bbox=%s' % (a, b, vol, BuildArm.fmt_bbox(bb)))
    for kind, a, b, vol, bb in slivers:
        rep('    sliver %s <-> %s volume=%.2f mm3 depth=%.3f mm'
            % (a, b, vol, min(bb[1][i] - bb[0][i] for i in range(3))))

    # ---- deviation from the link the current generator builds ----
    rep('')
    for name, gname, body, route, _arm in built:
        old = find_body(design, name)
        d = deviation(design, body, old)
        if d is None:
            rep('  %s: no %s in this document to compare against' % (gname, name))
            continue
        rep('  %s vs %s: volume %.0f -> %.0f mm3 (%+.1f %%), faces %d -> %d; the two solids '
            'share %.0f mm3, symmetric difference %.0f mm3 (%.0f %% of the old link); '
            'largest distance from a new vertex to the nearest old vertex %.2f mm; largest '
            'bounding-box corner move %.2f mm'
            % (gname, name, d['v_old'], d['v_new'],
               100.0 * (d['v_new'] - d['v_old']) / d['v_old'], d['faces_old'],
               d['faces_new'], d['shared'], d['symdiff'],
               100.0 * d['symdiff'] / d['v_old'], d['worst_vertex'], d['bbox']))

    head = ('BuildArmGen done (%s).' % spec['key'] if not fails
            else 'BuildArmGen (%s) finished with %d interference FAIL(s).'
            % (spec['key'], len(fails)))
    text = head + '\n\n' + '\n'.join(report)
    try:
        dd = os.path.dirname(spec['report'])
        if not os.path.isdir(dd):
            os.makedirs(dd)
        with open(spec['report'], 'w') as f:
            f.write(text)
    except Exception:
        pass
    return text


def build_general(app, ui, design, log=None, interactive=False):
    """The G1 arm, into `generated_R_arm_general` - what this file always built."""
    return build_arm(G1_SPEC, app, ui, design, log, interactive)


def build_luna(app, ui, design, log=None, interactive=False):
    """The Gento Luna right arm, into `generated_R_arm_luna` (EXO-218)."""
    return build_arm(LUNA_SPEC, app, ui, design, log, interactive)
