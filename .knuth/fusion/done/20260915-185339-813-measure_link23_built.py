# EXO-58 / EXO-64: measure the BUILT link_23 body in the J2 module frame (the J2_* wrapper's
# transform; arm at identity). Dumps bbox, every cylindrical face (radius, axis, centre) and
# every planar face whose normal is along the bar width, then CHECKS the built geometry
# against armgen's numbers and raises if anything is off:
#   - one solid body
#   - the side walls are exactly two planes at the PAD end (centred on the J2 rotor pad)
#     and exactly two planes at the SLAB end (centred on the J3 stator pad)
#   - the riser S arc faces exist, with the intended radii, axes and centres, and the four
#     wall pieces (plane, arc, arc, plane) meet without a gap along the bar
#   - the riser inner face still sits RISER_CLEAR outside the r 27 module
#   - EXO-74: the filled rotor-pad join - the pad's outer face and the fill are one plane
#     running past the rim to the r JOIN_ROUND_R round, the pad's rotor-side plane carries
#     the RISER_CLEAR ledge, the old r 12.5 cove / r 5 relief are gone, the r 27 rim face
#     stops where the 25-wide band leaves it, and sampled points just outside the rim over
#     the whole pad thickness are inside the link (rim covered across the 25 mm width)
# EXO-63: a green interference check is not proof - this is the measurement of the real body.
import math
import itertools
import sys
import importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
importlib.reload(armgen)

REPO = '/Users/esteban/repos/exo'
TOL = 0.02          # mm: face positions must match the computed profile this closely

# ---- expected numbers, recomputed from the URDF exactly as BuildArm does ----
joints, by_child = armgen.parse_urdf(REPO + '/g1/g1_29dof.urdf')
shift, _ = armgen.joint_world_pose('left_shoulder_pitch_joint', joints, by_child)
JSPEC = {'J2': ('right_shoulder_roll_joint', +1, (0, 1, 0)),
         'J3': ('right_shoulder_yaw_joint', -1, (-1, 0, 0))}
FR = {}
for lab, (nm, sgn, yh) in JSPEC.items():
    R, t = armgen.module_frame(nm, joints, by_child, sgn, yh)
    FR[lab] = (R, [t[i] - shift[i] for i in range(3)])
R_23, t_23 = armgen.relative_frame(FR['J2'][0], FR['J2'][1], FR['J3'][0], FR['J3'][1])
# EXO-69: measure against the placement the DOCUMENT was actually built with. BuildArm
# writes 'S placement: riser/bar' into gento_luna/exo/exports/buildarm_report.txt; armgen's
# fallback when there is no report (then the checks below are about the default build).
PLACEMENT = armgen.S_PLACEMENT
try:
    with open(REPO + '/gento_luna/exo/exports/buildarm_report.txt') as _f:
        for _line in _f:
            if _line.startswith('S placement:'):
                PLACEMENT = _line.split(':', 1)[1].strip()
                break
except IOError:
    pass
if PLACEMENT not in armgen.S_PLACEMENTS:
    raise RuntimeError('unknown S placement %r in the build report' % PLACEMENT)
BG = armgen.link_bar_geometry(R_23, t_23, 27.0, placement=PLACEMENT)
SC = BG['s_curve']
U, W = BG['u'], BG['w']            # bar direction and width direction, in the J2 frame
HALF = armgen.BAR_W / 2.0
PAD_EXIT = math.sqrt(27.0 ** 2 - HALF ** 2)      # where the 25-wide band leaves the pad rim

fails = []


def check(name, ok, detail=''):
    log('%s %s%s' % ('ok  ' if ok else 'FAIL', name, (' - ' + detail) if detail else ''))
    if not ok:
        fails.append(name)


def near(name, got, want, tol=TOL):
    got = list(got) if hasattr(got, '__iter__') else [got]
    want = list(want) if hasattr(want, '__iter__') else [want]
    err = max(abs(a - b) for a, b in zip(got, want))
    check(name, err <= tol, 'got %s want %s (err %.4f, tol %g)'
          % ([round(v, 3) for v in got], [round(v, 3) for v in want], err, tol))


# ---- the built body, in the J2 module frame ----
root = design.rootComponent
arm = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name.startswith('generated_R_arm'):
        arm = o
j2 = link = None
for i in range(arm.childOccurrences.count):
    o = arm.childOccurrences.item(i)
    if o.component.name.startswith('J2_'):
        j2 = o
    if o.component.name.startswith('link_23'):
        link = o
inv = j2.transform.copy()
inv.invert()
check('link_23 is ONE solid body', link.bRepBodies.count == 1, '%d bodies' % link.bRepBodies.count)
body = link.bRepBodies.item(0)


def toJ2(p):
    q = p.copy()
    q.transformBy(inv)
    return (q.x * 10, q.y * 10, q.z * 10)


def dirJ2(v):
    w = v.copy()
    w.transformBy(inv)
    return (w.x, w.y, w.z)


def sw(p):
    """(s, w) of a J2-frame point in the bar's own plan coordinates."""
    return (p[0] * U[0] + p[1] * U[1], p[0] * W[0] + p[1] * W[1])


bb = body.boundingBox
pts = [toJ2(adsk.core.Point3D.create(x, y, z)) for x, y, z in itertools.product(
    (bb.minPoint.x, bb.maxPoint.x), (bb.minPoint.y, bb.maxPoint.y), (bb.minPoint.z, bb.maxPoint.z))]
log('link_23: %d faces, volume %.0f mm3, bbox in J2 frame x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]' % (
    body.faces.count, body.volume * 1000, min(p[0] for p in pts), max(p[0] for p in pts),
    min(p[1] for p in pts), max(p[1] for p in pts), min(p[2] for p in pts), max(p[2] for p in pts)))
log('expected S (placement %r): shift %.3f mm over s %.3f..%.3f, centre arcs r %.1f, wall arcs '
    'r %.1f / %.1f about (s,w) %s and %s' % (PLACEMENT, abs(SC['w_off']), SC['s_start'], SC['s_end'], SC['r'],
                               SC['wall_r'][0], SC['wall_r'][1],
                               tuple(round(v, 3) for v in SC['centres'][0]),
                               tuple(round(v, 3) for v in SC['centres'][1])))

ST = {0: 'plane', 1: 'cyl', 2: 'cone', 3: 'sphere', 4: 'torus', 5: 'ellcyl', 6: 'ellcone', 7: 'nurbs'}
kinds = {}
cyl, ywalls, uwalls, zfaces = [], [], [], []
for i in range(body.faces.count):
    f = body.faces.item(i)
    g = f.geometry
    k = ST.get(g.surfaceType, '?')
    kinds[k] = kinds.get(k, 0) + 1
    fb = f.boundingBox
    cs = [toJ2(adsk.core.Point3D.create(x, y, z)) for x, y, z in itertools.product(
        (fb.minPoint.x, fb.maxPoint.x), (fb.minPoint.y, fb.maxPoint.y), (fb.minPoint.z, fb.maxPoint.z))]
    ext = tuple((min(c[a] for c in cs), max(c[a] for c in cs)) for a in range(3))
    s_ext = (min(sw(c)[0] for c in cs), max(sw(c)[0] for c in cs))
    w_ext = (min(sw(c)[1] for c in cs), max(sw(c)[1] for c in cs))
    if k == 'cyl':
        ax = dirJ2(g.axis)
        o = toJ2(g.origin)
        cyl.append((round(g.radius * 10, 2), tuple(round(v, 2) for v in ax),
                    tuple(round(v, 2) for v in o), ext, f.area * 100, sw(o), s_ext, w_ext))
    elif k == 'plane':
        n = dirJ2(g.normal)
        o = toJ2(g.origin)
        if abs(n[0] * W[0] + n[1] * W[1]) > 0.999:
            ywalls.append((round(sw(o)[1], 3), ext, f.area * 100, s_ext))
        elif abs(n[0] * U[0] + n[1] * U[1]) > 0.999:
            uwalls.append((round(sw(o)[0], 3), ext, f.area * 100, w_ext))
        elif abs(n[2]) > 0.999:
            zfaces.append((round(o[2], 3), ext, f.area * 100, s_ext, w_ext))
log('face kinds: %s' % kinds)
log('cylindrical faces (r, axis J2, a point on axis J2, face bbox J2, area mm2):')
for c in sorted(cyl):
    log('  r %6.2f axis %s at %s  x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f]  %.0f mm2' % (
        c[0], c[1], c[2], c[3][0][0], c[3][0][1], c[3][1][0], c[3][1][1], c[3][2][0], c[3][2][1], c[4]))
log('planar side-wall faces (normal along the bar width): w, s range, bbox, area')
for w, ext, a, s_ext in sorted(ywalls):
    log('  w %7.2f  s[%.2f,%.2f]  x[%.1f,%.1f] z[%.1f,%.1f]  %.0f mm2'
        % (w, s_ext[0], s_ext[1], ext[0][0], ext[0][1], ext[2][0], ext[2][1], a))
log('planar faces normal to the bar direction u: s, w range, area')
for s, ext, a, w_ext in sorted(uwalls):
    log('  s %7.2f  w[%.2f,%.2f]  z[%.1f,%.1f]  %.0f mm2' % (s, w_ext[0], w_ext[1], ext[2][0], ext[2][1], a))
log('planar faces normal to the pad axis: z, s range, w range, area')
for z, ext, a, s_ext, w_ext in sorted(zfaces):
    log('  z %7.2f  s[%.2f,%.2f]  w[%.2f,%.2f]  %.0f mm2' % (z, s_ext[0], s_ext[1], w_ext[0], w_ext[1], a))

# ---- 1. side walls: two planes at the pad end, two at the slab end, each pair centred ----
pad_walls = [t for t in ywalls if t[3][1] <= SC['s_start'] + TOL and t[3][1] > PAD_EXIT]
slab_walls = [t for t in ywalls if t[3][0] >= SC['s_end'] - TOL and t[3][1] > BG['s_face']
              and abs(abs(t[0] - SC['w_off']) - HALF) < TOL]
check('exactly two side-wall planes at the PAD end', len(pad_walls) == 2,
      'got %s' % [t[0] for t in pad_walls])
check('exactly two side-wall planes at the SLAB end', len(slab_walls) == 2,
      'got %s' % [t[0] for t in slab_walls])
if len(pad_walls) == 2:
    w0, w1 = sorted(t[0] for t in pad_walls)
    near('pad-end walls are at +/-BAR_W/2 on the J2 rotor pad', (w0, w1), (-HALF, HALF))
    near('pad-end walls centred on the J2 pad axis', (w0 + w1) / 2.0, 0.0)
    near('pad-end walls are BAR_W apart', w1 - w0, armgen.BAR_W)
    log('    pad-end wall planes run s %.2f..%.2f (pad rim exit %.2f, S starts %.2f), %.0f / %.0f mm2'
        % (min(t[3][0] for t in pad_walls), max(t[3][1] for t in pad_walls), PAD_EXIT,
           SC['s_start'], pad_walls[0][2], pad_walls[1][2]))
if len(slab_walls) == 2:
    v0, v1 = sorted(t[0] for t in slab_walls)
    near('slab-end walls are at +/-BAR_W/2 on the J3 stator pad',
         (v0, v1), (SC['w_off'] - HALF, SC['w_off'] + HALF))
    near('slab-end walls centred on the J3 stator pad', (v0 + v1) / 2.0, SC['w_off'])
    near('slab-end walls are BAR_W apart', v1 - v0, armgen.BAR_W)

# ---- 2. the riser S arc faces: radius, axis along the pad axis, centre ----
# Cylindrical faces whose axis is the PAD axis and whose radius is one of the two wall
# radii. Which wall an arc belongs to follows from its centre and radius: about the first
# centre the wall at lateral offset d has radius R - sign*d, about the second R + sign*d.
s_faces = []
for c in cyl:
    if abs(c[1][2]) < 0.999:
        continue
    for k, ctr in enumerate(SC['centres']):
        if max(abs(c[5][0] - ctr[0]), abs(c[5][1] - ctr[1])) > TOL:
            continue
        d = (SC['r'] - c[0]) / SC['sign'] if k == 0 else (c[0] - SC['r']) / SC['sign']
        if abs(abs(d) - HALF) < TOL:
            s_faces.append((c, k, d))
check('four riser-S wall arc faces (two per wall)', len(s_faces) == 4,
      'got %d: %s' % (len(s_faces), [(c[0], tuple(round(v, 2) for v in c[5])) for c, _, _ in s_faces]))
for c, k, d in sorted(s_faces, key=lambda t: (t[2], t[0][6][0])):
    near('S arc r %.1f (wall %+.1f) axis on the computed centre %d' % (c[0], d, k + 1),
         c[5], SC['centres'][k])
    near('S arc r %.1f (wall %+.1f) axis parallel to the pad axis' % (c[0], d), abs(c[1][2]), 1.0, 1e-3)
    log('    S arc r %.1f on wall %+.1f about (s,w) = (%.3f, %.3f), s %.2f..%.2f, w %.2f..%.2f, '
        'z %.1f..%.1f, %.0f mm2'
        % (c[0], d, c[5][0], c[5][1], c[6][0], c[6][1], c[7][0], c[7][1], c[3][2][0], c[3][2][1], c[4]))
got_r = sorted(round(c[0], 2) for c, _, _ in s_faces)
near('S wall arc radii are r %.1f x2 and r %.1f x2' % SC['wall_r'], got_r,
     sorted([SC['wall_r'][0]] * 2 + [SC['wall_r'][1]] * 2))

# ---- 3. each wall is plane -> arc -> arc -> plane with no gap along the bar ----
for side, sign in (('-', -1.0), ('+', 1.0)):
    w_pad = sign * HALF
    w_slab = SC['w_off'] + sign * HALF
    pw = [t for t in pad_walls if abs(t[0] - w_pad) < TOL]
    sw_ = [t for t in slab_walls if abs(t[0] - w_slab) < TOL]
    arcs = sorted([c for c, _k, d in s_faces if abs(d - sign * HALF) < TOL], key=lambda c: c[6][0])
    if len(pw) == 1 and len(sw_) == 1 and len(arcs) == 2:
        near('wall %s12.5: plane ends where the first S arc starts' % side,
             arcs[0][6][0], pw[0][3][1])
        near('wall %s12.5: the two S arcs meet with no gap' % side, arcs[0][6][1], arcs[1][6][0])
        # the two walls turn over at different stations: the wall arcs are concentric with
        # the centre arcs, so the join sits (r -/+ BAR_W/2) sin(theta) past the S start
        r_first = SC['r'] - SC['sign'] * sign * HALF
        near('wall %s12.5: arcs meet at the computed station' % side, arcs[0][6][1],
             SC['s_start'] + r_first * math.sin(math.radians(SC['theta_deg'])), 0.05)
        near('wall %s12.5: the second S arc ends where the slab-end plane starts' % side,
             arcs[1][6][1], sw_[0][3][0])
        near('wall %s12.5: S arcs span exactly the computed run' % side,
             (arcs[0][6][0], arcs[1][6][1]), (SC['s_start'], SC['s_end']), 0.05)
    else:
        check('wall %s12.5 is plane -> arc -> arc -> plane' % side, False,
              '%d pad planes, %d slab planes, %d arcs' % (len(pw), len(sw_), len(arcs)))

# ---- 4. the riser inner face still clears the module by RISER_CLEAR ----
inner = [t for t in uwalls if abs(t[0] - BG['s_in']) < TOL]
check('riser inner face is one plane at s = s_in (%.2f)' % BG['s_in'], len(inner) == 1,
      'planes normal to u at s = %s' % [t[0] for t in sorted(uwalls)])
if inner:
    near('riser inner face is RISER_CLEAR outside r 27', inner[0][0] - 27.0, armgen.RISER_CLEAR)
    log('    riser inner face at s=%.3f spans w %.3f..%.3f (centre %.3f, %.2f wide), %.0f mm2'
        % (inner[0][0], inner[0][3][0], inner[0][3][1],
           (inner[0][3][0] + inner[0][3][1]) / 2.0, inner[0][3][1] - inner[0][3][0], inner[0][2]))
    near('riser inner face is centred on the J2 pad',
         (inner[0][3][0] + inner[0][3][1]) / 2.0, 0.0, 0.1)

# ---- 4b. the defining property of the chosen placement, measured on the body ----
if PLACEMENT == 'bar':
    # riser and bend are straight, full width and centred on the J2 pad: the pad-end wall
    # planes must run past the end of the bend, and the S must sit in the straight bar with
    # armgen.S_BAR_MARGIN of straight bar on each side of it.
    if len(pad_walls) == 2:
        reach = min(t[3][1] for t in pad_walls)
        check('bar placement: pad-end walls run past the end of the bend (s %.2f)'
              % BG['s_bend_end'], reach >= BG['s_bend_end'] - TOL,
              'walls stop at s=%.2f' % reach)
    arc_s = [c[6] for c, _k, _d in s_faces]
    if arc_s:
        first, last = min(a[0] for a in arc_s), max(a[1] for a in arc_s)
        check('bar placement: straight bar before the S (>= %.2f mm)' % armgen.S_BAR_MARGIN,
              first - BG['s_bend_end'] >= armgen.S_BAR_MARGIN - TOL,
              'S starts %.3f mm past the bend end' % (first - BG['s_bend_end']))
        check('bar placement: straight bar after the S (>= %.2f mm)' % armgen.S_BAR_MARGIN,
              BG['s_face'] - last >= armgen.S_BAR_MARGIN - TOL,
              'S ends %.3f mm before the slab face' % (BG['s_face'] - last))
        near('bar placement: the two straight margins are equal', first - BG['s_bend_end'],
             BG['s_face'] - last, 0.05)
    if inner:
        near('bar placement: riser inner face is the full BAR_W, centred on the J2 pad',
             (inner[0][3][0], inner[0][3][1]), (-HALF, HALF))
else:
    check('riser placement: the S starts inside the riser (s %.2f, riser %.2f..%.2f)'
          % (SC['s_start'], BG['s_in'], BG['s_out']),
          BG['s_in'] - armgen.FILLET_IN_TOP <= SC['s_start'] <= BG['s_out'])

# ---- 4c. EXO-74: the filled rotor-pad join, measured on the body ----
PAD_R = 27.0
S_ROUND = BG['s_out'] - armgen.JOIN_ROUND_R          # 29.88: where the outer face turns down
# the outside surface: ONE plane at the pad's outer face, running from the far side of the
# pad out past the rim to the round - the r 12.5 cove that scooped the pocket is gone
top = [t for t in zfaces if abs(t[0] - armgen.PAD_T) < TOL]
check('EXO-74: one plane at the pad outer face (z=%.1f)' % armgen.PAD_T, len(top) == 1,
      'got %s' % [t[0] for t in zfaces])
if len(top) == 1:
    near('EXO-74: outer face runs out to the join round, past the rim', top[0][3][1], S_ROUND)
    check('EXO-74: outer face reaches %.2f mm past the rim' % (S_ROUND - PAD_R),
          top[0][3][1] - PAD_R >= armgen.JOIN_RIM_MARGIN - TOL,
          'outer face stops at s=%.3f (rim %.1f)' % (top[0][3][1], PAD_R))
    # it is the pad's own outer face continued outward: it still spans the whole r 27 disk
    check('EXO-74: the outer face is the pad face and the fill in ONE plane',
          top[0][4][1] - top[0][4][0] >= armgen.BAR_W - TOL and top[0][3][0] <= -PAD_R + TOL,
          's %.2f..%.2f, w %.2f..%.2f' % (top[0][3][0], top[0][3][1], top[0][4][0], top[0][4][1]))
    log('    outer face: s %.2f..%.2f, w %.2f..%.2f, %.0f mm2'
        % (top[0][3][0], top[0][3][1], top[0][4][0], top[0][4][1], top[0][2]))
# the join round: r JOIN_ROUND_R about (s, z) = (S_ROUND, PAD_T - R), axis along the width
rounds = [c for c in cyl if abs(c[0] - armgen.JOIN_ROUND_R) < TOL
          and abs(c[1][2]) < 0.01 and abs(c[5][0] - S_ROUND) < TOL
          and abs(c[2][2] - (armgen.PAD_T - armgen.JOIN_ROUND_R)) < TOL]
check('EXO-74: the join round r %.1f is there, about (s,z) = (%.2f, %.2f)'
      % (armgen.JOIN_ROUND_R, S_ROUND, armgen.PAD_T - armgen.JOIN_ROUND_R), len(rounds) == 1,
      'got %d' % len(rounds))
if rounds:
    log('    join round r %.1f: s %.2f..%.2f, z %.2f..%.2f, %.0f mm2'
        % (rounds[0][0], rounds[0][6][0], rounds[0][6][1], rounds[0][3][2][0], rounds[0][3][2][1],
           rounds[0][4]))
# the two fillets the fill replaced must NOT be back: r 12.5 about (22.38, -4.5) and r 5
# about (s_in - 5, -5), both with their axis along the band width
gone = [c for c in cyl if abs(c[1][2]) < 0.01
        and ((abs(c[0] - 12.5) < TOL and abs(c[5][0] - (BG['s_out'] - 12.5)) < TOL
              and abs(c[2][2] - (armgen.PAD_T - 12.5)) < TOL)
             or (abs(c[0] - 5.0) < TOL and abs(c[5][0] - (BG['s_in'] - 5.0)) < TOL
                 and abs(c[2][2] + 5.0) < TOL))]
check('EXO-74: no pocket fillet at the join (old r 12.5 cove / r 5 relief)', not gone,
      'got %s' % [(c[0], round(c[5][0], 2), c[2][2]) for c in gone])
# the inside: the pad's rotor-side plane runs out to the riser inner face - that strip IS
# the RISER_CLEAR ledge - and nothing of the body sits below it inside r 27
ledge = [t for t in zfaces if abs(t[0]) < TOL]
check('EXO-74: one plane at the pad rotor-side face (z=0)', len(ledge) == 1,
      'got %s' % [t[0] for t in zfaces])
if len(ledge) == 1:
    near('EXO-74: ledge runs out to the riser inner face', ledge[0][3][1], BG['s_in'])
    near('EXO-74: ledge is RISER_CLEAR wide outside the rim', ledge[0][3][1] - PAD_R,
         armgen.RISER_CLEAR)
    log('    pad rotor-side face + ledge: s %.2f..%.2f, w %.2f..%.2f, %.0f mm2'
        % (ledge[0][3][0], ledge[0][3][1], ledge[0][4][0], ledge[0][4][1], ledge[0][2]))
# the rim face itself: after the fill the rotor pad's r 27 cylinder must STOP where the
# 25-wide band leaves it (s = 23.932). Anything beyond that is rim the link does not cover -
# the pocket left a crescent of rim up to 0.885 mm tall reaching all the way to s = 27.
rim = [c for c in cyl if abs(c[0] - PAD_R) < TOL and abs(c[1][2]) > 0.999
       and c[3][2][1] > armgen.PAD_T - TOL]
check('EXO-74: one rotor-pad rim face', len(rim) == 1, 'got %d' % len(rim))
if rim:
    near('EXO-74: the rim face stops where the band leaves it (rim covered from there on)',
         rim[0][6][1], PAD_EXIT, 0.05)
    log('    rim face r %.1f: s %.2f..%.2f, z %.2f..%.2f, %.0f mm2 (uncovered rim; the band '
        'covers %.1f deg of it)' % (rim[0][0], rim[0][6][0], rim[0][6][1], rim[0][3][2][0],
                                    rim[0][3][2][1], rim[0][4],
                                    2 * math.degrees(math.asin(HALF / PAD_R))))
# and the same thing sampled: covered by link material across the whole BAR_W, over the whole
# pad thickness. Sampled with pointContainment just outside r 27 (the direct statement of what
# Esteban asked for); the face-based checks above stand on their own if it is unavailable.
try:
    INSIDE = adsk.fusion.PointContainment.PointInsidePointContainment
    ON = adsk.fusion.PointContainment.PointOnPointContainment
    fwd = j2.transform
    missed, tested = [], 0
    for wk in range(-12, 13):
        w_at = wk * 1.0
        s_at = math.sqrt((PAD_R + 0.05) ** 2 - w_at ** 2)
        for zk in range(1, 16):
            z_at = zk * 0.5
            p = adsk.core.Point3D.create((U[0] * s_at + W[0] * w_at) / 10.0,
                                         (U[1] * s_at + W[1] * w_at) / 10.0, z_at / 10.0)
            p.transformBy(fwd)
            tested += 1
            if body.pointContainment(p) not in (INSIDE, ON):
                missed.append((round(w_at, 1), round(z_at, 2)))
    check('EXO-74: rim covered - %d points at r %.2f across the 25 mm width, z 0.5..7.5, '
          'are inside the link' % (tested, PAD_R + 0.05), not missed,
          'uncovered at (w, z) %s' % missed[:8])
    # and the clearance past the pad is untouched: just inside the riser face, below the
    # stop block's top plane, the link must NOT be there
    out = []
    for wk in range(-12, 13):
        w_at = wk * 1.0
        s_at = math.sqrt(max(0.0, (PAD_R + armgen.RISER_CLEAR - 0.05) ** 2 - w_at ** 2))
        for z_at in (-0.5, -2.0, -4.0, -8.0, -16.0):
            p = adsk.core.Point3D.create((U[0] * s_at + W[0] * w_at) / 10.0,
                                         (U[1] * s_at + W[1] * w_at) / 10.0, z_at / 10.0)
            p.transformBy(fwd)
            if body.pointContainment(p) == INSIDE:
                out.append((round(w_at, 1), z_at))
    check('EXO-74: nothing inside r %.2f below the pad (RISER_CLEAR kept)'
          % (PAD_R + armgen.RISER_CLEAR - 0.05), not out, 'material at (w, z) %s' % out[:8])
except AttributeError as _e:
    log('note: pointContainment unavailable (%s); the join is checked by its faces only' % _e)

# ---- 5. EXO-61's two defect signatures must not come back ----
tiny = []
for i in range(body.faces.count):
    a = body.faces.item(i).area * 100
    if a < 0.5:
        tiny.append(round(a, 4))
check('no degenerate faces (< 0.5 mm2)', not tiny, 'got %s' % tiny)
dupes = []
for i in range(len(ywalls)):
    for j in range(i + 1, len(ywalls)):
        if abs(ywalls[i][0] - ywalls[j][0]) < 0.05:
            dupes.append((ywalls[i][0], ywalls[j][0]))
check('no two side-wall planes nearly coplanar (no seam)', not dupes, 'got %s' % dupes)

log('')
if fails:
    log('MEASURED-BODY CHECK FAILED: %s' % fails)
    raise RuntimeError('built link_23 does not match armgen: %d check(s) failed' % len(fails))
log('measured-body check: all checks passed')
