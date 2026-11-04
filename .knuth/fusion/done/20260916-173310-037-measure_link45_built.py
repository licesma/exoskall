# EXO-114: measure the BUILT link_45 body (the ramp pair J4->J5) in the J4 module frame and
# check it against armgen.route's numbers AND against the hand-built `Link 4 R`
# (EXO-115/EXO-116). Same discipline as measure_link34_built.py (EXO-63: a green interference
# check is not proof - this is the measurement of the real body):
#   - one solid body, no degenerate faces, no two nearly-coplanar side walls
#   - the plate is COPLANAR with the rotor pad: one face on z = 0 and one on z = PAD_T
#   - the side-plane S: FOUR r RAMP_RISE_R cylinders across the band, two per boundary, at
#     the centres the route computed, turning +theta and back (Link 4 R: all four r 20.000)
#   - the run lands with its centre line ON the stator disk's rim, away from the connector
#   - the PLAN TAPER: one wall flat on the rotor pad's tangent line, one flat on the stator
#     disk's, and one r PLAN_TAPER_R cylinder about an axis PARALLEL TO THE PAD AXIS whose
#     centre is pad_r + PLAN_TAPER_R from axis A - externally tangent to the rim
#   - the land: the fillet on the run's UNDERSIDE into the slab's near face and the corner
#     round on TOP at its far face, both r RAMP_FILLET
#   - nothing anywhere below z = 0 except the link's own stop block and index tab
#   - the rim is covered over the pad's whole thickness across the band (EXO-74's standard)
import math
import itertools
import sys
import importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
importlib.reload(armgen)

REPO = '/Users/esteban/repos/exo'
TOL = 0.02          # mm: face positions must match the computed profile this closely
PAD_R = 27.0        # link_45's rotor pad (BuildArm.LINKS, = Link 4 R's measured r 27)
WIDTH = 44.0        # BuildArm.LINKS
STOP_H = 3.6

# ---- expected numbers, recomputed from the URDF exactly as BuildArm does ----
joints, by_child = armgen.parse_urdf(REPO + '/g1/g1_29dof.urdf')
shift, _ = armgen.joint_world_pose('left_shoulder_pitch_joint', joints, by_child)
JSPEC = {'J4': ('right_elbow_joint', -1, (-1, 0, 0)),
         'J5': ('right_wrist_roll_joint', +1, (0, 1, 0))}
FR = {}
for lab, (nm, sgn, yh) in JSPEC.items():
    R, t = armgen.module_frame(nm, joints, by_child, sgn, yh)
    FR[lab] = (R, [t[i] - shift[i] for i in range(3)])
SPEC = armgen.route(FR['J4'], FR['J5'], PAD_R, width=WIDTH)
if SPEC['pattern'] != 'ramp':
    raise RuntimeError('the router no longer picks the ramp for J4->J5: %r' % SPEC['pattern'])
LG = SPEC['geometry']
U, W = LG['u'], LG['w']
HALF = LG['half_w']
TAPER = LG['taper']
S_LO, S_HI = LG['s_lo'], LG['s_hi']
AXIAL = LG['axial']
BAR_LO, BAR_HI = LG['bar_z']
R_F = LG['r_in']                       # RAMP_FILLET, both the land fillet and the round
RISE_R = LG['rise_r']
EDGES = LG['edges']
# the four S arc centres the route computed, taken from the drawn profile
RISE_C = sorted(tuple(round(v, 6) for v in sg[4])
                for sg in LG['profile'] if sg[0] == 'arc' and abs(sg[5] - RISE_R) < 1e-9)

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


# ---- the built body, in the J4 module frame ----
root = design.rootComponent
arm = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name.startswith('generated_R_arm'):
        arm = o
if arm is None:
    raise RuntimeError('no generated_R_arm occurrence in the document')
j4 = link = None
for i in range(arm.childOccurrences.count):
    o = arm.childOccurrences.item(i)
    if o.component.name.startswith('J4_'):
        j4 = o
    if o.component.name.startswith('link_45'):
        link = o
if j4 is None or link is None:
    raise RuntimeError('J4 wrapper or link_45 not found under %s' % arm.component.name)
inv = j4.transform.copy()
inv.invert()
check('link_45 is ONE solid body', link.bRepBodies.count == 1,
      '%d bodies' % link.bRepBodies.count)
body = link.bRepBodies.item(0)


def toJ4(p):
    q = p.copy()
    q.transformBy(inv)
    return (q.x * 10, q.y * 10, q.z * 10)


def dirJ4(v):
    w = v.copy()
    w.transformBy(inv)
    return (w.x, w.y, w.z)


def sw(p):
    """(s, w) of a J4-frame point in the link's own plan coordinates."""
    return (p[0] * U[0] + p[1] * U[1], p[0] * W[0] + p[1] * W[1])


bb = body.boundingBox
pts = [toJ4(adsk.core.Point3D.create(x, y, z)) for x, y, z in itertools.product(
    (bb.minPoint.x, bb.maxPoint.x), (bb.minPoint.y, bb.maxPoint.y), (bb.minPoint.z, bb.maxPoint.z))]
log('link_45 (route %s, ribbon %.0f wide): %d faces, volume %.0f mm3, bbox in J4 frame '
    'x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]'
    % (SPEC['pattern'], LG['width'], body.faces.count, body.volume * 1000,
       min(p[0] for p in pts), max(p[0] for p in pts), min(p[1] for p in pts),
       max(p[1] for p in pts), min(p[2] for p in pts), max(p[2] for p in pts)))
log('expected: plate z %.1f..%.1f in the pad plane; S of four r %.1f arcs at (s, z) %s '
    'rising %.3f mm over %.3f; run z %.3f..%.3f landing on the disk rim (Y_B %.1f); slab '
    'faces s %.2f/%.2f; band w %.3f..%.3f (centre %.3f); taper r %.1f; land fillet and far '
    'round r %.1f; tongue edges %s'
    % (LG['plate_z'][0], LG['plate_z'][1], RISE_R,
       ' '.join('(%.3f, %.3f)' % c for c in RISE_C), LG['rise'], LG['rise_run'],
       BAR_LO, BAR_HI, LG['y_land'], S_LO, S_HI, TAPER['w_lo'], TAPER['w_hi'],
       TAPER['centre'], TAPER['taper_r'], R_F,
       ' / '.join('%s r %.3f' % (e['kind'], e['r']) for e in EDGES)))
log('Link 4 R for comparison: plate z 0..6 on a r 27 pad, band w -27..17, S of four r 20 '
    'arcs turning 69.3396 deg, run z 25.889..31.889 (6 thick), slab faces s 69.5/75.5, '
    'plan taper r 50 tangent to the rim at (13.306, 23.494), land fillet r 5 at (64.5, '
    '20.889), far round r 5 at (70.5, 26.889), tongue toe r 49')

ST = {0: 'plane', 1: 'cyl', 2: 'cone', 3: 'sphere', 4: 'torus', 5: 'ellcyl', 6: 'ellcone', 7: 'nurbs'}
kinds = {}
cyl, wwalls, uwalls, zfaces, areas = [], [], [], [], []
for i in range(body.faces.count):
    f = body.faces.item(i)
    g = f.geometry
    k = ST.get(g.surfaceType, '?')
    kinds[k] = kinds.get(k, 0) + 1
    areas.append(f.area * 100)
    fb = f.boundingBox
    cs = [toJ4(adsk.core.Point3D.create(x, y, z)) for x, y, z in itertools.product(
        (fb.minPoint.x, fb.maxPoint.x), (fb.minPoint.y, fb.maxPoint.y), (fb.minPoint.z, fb.maxPoint.z))]
    ext = tuple((min(c[a] for c in cs), max(c[a] for c in cs)) for a in range(3))
    s_ext = (min(sw(c)[0] for c in cs), max(sw(c)[0] for c in cs))
    w_ext = (min(sw(c)[1] for c in cs), max(sw(c)[1] for c in cs))
    z_ext = ext[2]
    if k == 'cyl':
        ax = dirJ4(g.axis)
        o = toJ4(g.origin)
        cyl.append((round(g.radius * 10, 3), tuple(round(v, 3) for v in ax),
                    tuple(round(v, 3) for v in o), ext, f.area * 100, sw(o), s_ext, w_ext,
                    z_ext))
    elif k == 'plane':
        n = dirJ4(g.normal)
        o = toJ4(g.origin)
        if abs(n[0] * W[0] + n[1] * W[1]) > 0.999:
            wwalls.append((round(sw(o)[1], 3), ext, f.area * 100, s_ext, z_ext))
        elif abs(n[0] * U[0] + n[1] * U[1]) > 0.999:
            uwalls.append((round(sw(o)[0], 3), ext, f.area * 100, w_ext, z_ext))
        elif abs(n[2]) > 0.999:
            zfaces.append((round(o[2], 3), ext, f.area * 100, s_ext, w_ext))
log('face kinds: %s' % kinds)
log('cylindrical faces (r, axis J4, a point on axis J4, area mm2):')
for c in sorted(cyl):
    log('  r %8.3f axis %s at %s  s[%.1f,%.1f] w[%.1f,%.1f] z[%.1f,%.1f]  %.0f mm2'
        % (c[0], c[1], c[2], c[6][0], c[6][1], c[7][0], c[7][1], c[8][0], c[8][1], c[4]))
log('planar side-wall faces (normal across the band): w, s range, z range, area')
for w, ext, a, s_ext, z_ext in sorted(wwalls):
    log('  w %7.2f  s[%.2f,%.2f]  z[%.2f,%.2f]  %.0f mm2'
        % (w, s_ext[0], s_ext[1], z_ext[0], z_ext[1], a))
log('planar faces normal to the pad axis: z, s range, w range, area')
for z, ext, a, s_ext, w_ext in sorted(zfaces):
    log('  z %7.2f  s[%.2f,%.2f]  w[%.2f,%.2f]  %.0f mm2'
        % (z, s_ext[0], s_ext[1], w_ext[0], w_ext[1], a))
log('planar faces normal to u (the two slab faces): s, w range, z range, area')
for s, ext, a, w_ext, z_ext in sorted(uwalls):
    log('  s %7.2f  w[%.2f,%.2f]  z[%.2f,%.2f]  %.0f mm2'
        % (s, w_ext[0], w_ext[1], z_ext[0], z_ext[1], a))


def axis_cyl(r, axis, tol=0.01):
    """Cylindrical faces of radius r whose axis runs along `axis` ('w', 'u' or 'z')."""
    out = []
    for c in cyl:
        d = (abs(c[1][0] * W[0] + c[1][1] * W[1]) if axis == 'w' else
             abs(c[1][0] * U[0] + c[1][1] * U[1]) if axis == 'u' else abs(c[1][2]))
        if abs(c[0] - r) < max(tol, r * 1e-4) and d > 0.99:
            out.append(c)
    return out


# ---- 1. the plate is coplanar with the rotor pad: ONE face per pad plane (EXO-92's join) ----
z0 = [t for t in zfaces if abs(t[0]) < TOL]
zt = [t for t in zfaces if abs(t[0] - armgen.PAD_T) < TOL]
check('the pad rotor-side plane z = 0 and the plate are ONE face', len(z0) == 1,
      'got %d faces at z = 0' % len(z0))
check('the pad outer plane z = %.1f and the plate are ONE face' % armgen.PAD_T, len(zt) == 1,
      'got %d faces at z = %.1f' % (len(zt), armgen.PAD_T))
# where each pad face hands over to the S. The ramp's climb starts INSIDE the rim (Link 4 R:
# lower boundary leaves z = 0 at s 16.0 on a r 27 pad), so unlike the shoulder L the plate
# does NOT run out past the rim: the pad's own disk fills under the climb and the rim stays a
# complete cylinder. Both hand-overs are TANGENT, which is what EXO-74 is really about - no
# cove, no step, nothing below z = 0.
S_DEP_LO = max(c[0] for c in RISE_C if abs(c[1] - RISE_R) < 1e-6)
S_DEP_HI = min(c[0] for c in RISE_C if abs(c[1] - armgen.PAD_T - RISE_R) < 1e-6)
if z0:
    near('the rotor-side face runs to the pad rim, the S leaving it tangentially at s = %.3f'
         % S_DEP_LO, z0[0][3][1], max(S_DEP_LO, PAD_R), 0.05)
    check('the climb starts inside the rim, as Link 4 R\'s does (s %.3f <= %.1f)'
          % (S_DEP_LO, PAD_R), S_DEP_LO <= PAD_R + TOL, 'leaves z = 0 at s = %.3f' % S_DEP_LO)
if zt:
    near('the outer face runs out to the S', zt[0][3][1], S_DEP_HI, 0.05)
rim = [c for c in axis_cyl(PAD_R, 'z') if abs(c[2][0]) < TOL and abs(c[2][1]) < TOL]
check('the rotor pad rim is the r %.1f cylinder about axis A, and nothing else' % PAD_R,
      len(rim) >= 1 and all(abs(c[8][0]) <= STOP_H + TOL
                            and c[8][1] <= armgen.PAD_T + TOL for c in rim),
      '%d face(s), z %s' % (len(rim), [c[8] for c in rim]))

# ---- 2. the plan taper: two straight walls, one on each rim's tangent line ----
lo_wall = [t for t in wwalls if abs(t[0] - TAPER['w_lo']) < TOL]
hi_wall = [t for t in wwalls if abs(t[0] - TAPER['w_hi']) < TOL]
check('ONE flat wall on the rotor pad rim tangent line w = %.3f' % TAPER['w_lo'],
      len(lo_wall) == 1, 'got %s' % [t[0] for t in wwalls])
check('ONE flat wall on the stator disk tangent line w = %.3f' % TAPER['w_hi'],
      len(hi_wall) == 1, 'got %s' % [t[0] for t in wwalls])
near('the band is %.0f mm across and sits w_off/2 off centre' % WIDTH,
     (TAPER['w_hi'] - TAPER['w_lo'], (TAPER['w_hi'] + TAPER['w_lo']) / 2.0),
     (WIDTH, LG['w_off'] / 2.0), 1e-9)
near('the far wall is tangent to the r %.1f rotor pad rim' % PAD_R, abs(TAPER['w_lo']),
     PAD_R, 1e-9)
near('the near wall is tangent to the r %.1f stator disk' % armgen.STATOR_PAD_R,
     abs(TAPER['w_hi'] - LG['w_off']), armgen.STATOR_PAD_R, 1e-9)
tw = TAPER['walls'][+1]
tap = axis_cyl(tw['r'], 'z')
check('one r %.1f taper cylinder about an axis parallel to the pad axis' % tw['r'],
      len(tap) == 1, '%d faces' % len(tap))
if tap:
    near('the taper axis is where the route put it (s, w)', tap[0][5], tw['c'], 0.05)
    near('and pad_r + r from axis A - EXTERNALLY tangent to the rim',
         math.hypot(*tap[0][5]), PAD_R + tw['r'], 0.05)
if lo_wall and hi_wall:
    near('the near wall starts where the taper leaves it (s = %.3f)' % tw['s_tan'],
         hi_wall[0][3][0], tw['s_tan'], 0.05)
    near('the far wall starts on the pad rim (s = 0, where w = -pad_r touches it)',
         lo_wall[0][3][0], 0.0, 0.05)

# ---- 3. the side-plane S: four r RISE_R cylinders across the band, at the route's centres --
rise_faces = axis_cyl(RISE_R, 'w')
check('FOUR r %.1f cylinders across the band (the S, two per boundary)' % RISE_R,
      len(rise_faces) == 4, '%d faces' % len(rise_faces))
got_c = sorted((round(c[5][0], 3), round(c[2][2], 3)) for c in rise_faces)
for want, got in zip(RISE_C, got_c):
    near('S arc centre (s, z)', got, want, 0.05)
near('the S rises %.3f mm (the run centre line over the plate\'s)' % LG['rise'],
     (BAR_LO + BAR_HI) / 2.0 - armgen.PAD_T / 2.0, LG['rise'], 1e-9)
# the arcs' own extents: each stays inside the plan band (they are cut by the taper near the
# pad and by the pad disk below, so their AREAS are not the full theta R width)
for c in rise_faces:
    check('S arc at (%.2f, %.2f) stays inside the plan band' % (c[5][0], c[2][2]),
          c[7][0] >= TAPER['w_lo'] - 0.05 and c[7][1] <= PAD_R + 0.05,
          'w[%.2f, %.2f]' % c[7])

# ---- 4. the run: PAD_T thick, centre line ON the stator disk's rim ----
zb_lo = [t for t in zfaces if abs(t[0] - BAR_LO) < TOL]
zb_hi = [t for t in zfaces if abs(t[0] - BAR_HI) < TOL]
check('the run\'s underside z = %.3f is ONE face' % BAR_LO, len(zb_lo) == 1,
      'got %d' % len(zb_lo))
check('the run\'s top z = %.3f is ONE face' % BAR_HI, len(zb_hi) == 1, 'got %d' % len(zb_hi))
near('the run is PAD_T thick', BAR_HI - BAR_LO, armgen.PAD_T, 1e-9)
near('its centre line lands ON the r %.1f disk rim, away from the connector'
     % armgen.STATOR_PAD_R, (BAR_LO + BAR_HI) / 2.0 - AXIAL, armgen.STATOR_PAD_R, 1e-9)

# ---- 5. the land: fillet UNDER the run, corner round on TOP at the far face ----
fil = [c for c in axis_cyl(R_F, 'w') if abs(c[2][2] - (BAR_LO - R_F)) < 0.1]
rnd = [c for c in axis_cyl(R_F, 'w') if abs(c[2][2] - (BAR_HI - R_F)) < 0.1]
check('one r %.1f land fillet under the run at z = %.3f' % (R_F, BAR_LO - R_F),
      len(fil) == 1, '%d faces' % len(fil))
check('one r %.1f corner round on top at the slab far face' % R_F, len(rnd) == 1,
      '%d faces' % len(rnd))
if fil:
    near('land fillet centre (s, z)', (fil[0][5][0], fil[0][2][2]), (S_LO - R_F, BAR_LO - R_F))
if rnd:
    near('corner round centre (s, z)', (rnd[0][5][0], rnd[0][2][2]), (S_HI - R_F, BAR_HI - R_F))
near_face = [t for t in uwalls if abs(t[0] - S_LO) < TOL]
far_face = [t for t in uwalls if abs(t[0] - S_HI) < TOL]
check('the slab near face s = %.2f is a real face' % S_LO, len(near_face) >= 1,
      'got %d' % len(near_face))
check('the slab far face s = %.2f is a real face' % S_HI, len(far_face) >= 1,
      'got %d' % len(far_face))
near('the slab is PAD_T thick', S_HI - S_LO, armgen.PAD_T, 1e-9)

# ---- 6. the stator slab: the disk about the J5 axis and the tongue's two edges ----
disk = axis_cyl(armgen.STATOR_PAD_R, 'u')
check('the r %.1f stator disk is there, axis along u' % armgen.STATOR_PAD_R, len(disk) >= 1,
      '%d faces' % len(disk))
for e in EDGES:
    if e['kind'] == 'tangent':
        w_wall = LG['w_off'] + e['x'] * LG['x_sign']
        check('the tangent tongue wall at w %.3f needs no edge face and IS the band wall'
              % w_wall, abs(w_wall - TAPER['w_hi']) < TOL or abs(w_wall - TAPER['w_lo']) < TOL,
              'w_wall %.3f vs band %.3f/%.3f' % (w_wall, TAPER['w_lo'], TAPER['w_hi']))
    else:
        bl = axis_cyl(e['r'], 'u')
        check('tongue %s edge: an r %.3f face about the J5 axis' % (e['kind'], e['r']),
              len(bl) >= 1, '%d faces' % len(bl))

# ---- 7. nothing below the pad's rotor-side plane except the stop block and index tab ----
# (the actuator cutout is cut THROUGH the stator slab and reaches z = -22.6 there; it is
# 67.5 mm away along u, nowhere near module A, so it is excluded by station and not ignored)
deep = [t for t in zfaces if t[0] < -TOL and t[3][1] < S_LO - TOL]
slab_deep = [t for t in zfaces if t[0] < -TOL and t[3][0] >= S_LO - TOL]
near('the lowest planar face near the pad is the stop block at -stop_h',
     min(t[0] for t in deep), -STOP_H, 0.05)
check('every face below z = 0 near the pad belongs to the stop block / index tab',
      all(abs(t[3][0]) <= PAD_R + TOL and abs(t[3][1]) <= PAD_R + TOL for t in deep),
      'got %s' % [(t[0], t[3]) for t in deep])
check('every other face below z = 0 is inside the stator slab (s >= %.2f)' % S_LO,
      all(t[3][0] >= S_LO - TOL for t in slab_deep),
      'got %s' % [(t[0], t[3]) for t in slab_deep])

# ---- 8. the rim is covered over the whole pad thickness, across the band ----
try:
    INSIDE = adsk.fusion.PointContainment.PointInsidePointContainment
    ON = adsk.fusion.PointContainment.PointOnPointContainment
    fwd = j4.transform

    def at(s_at, w_at, z_at):
        p = adsk.core.Point3D.create((U[0] * s_at + W[0] * w_at) / 10.0,
                                     (U[1] * s_at + W[1] * w_at) / 10.0, z_at / 10.0)
        p.transformBy(fwd)
        return body.pointContainment(p)

    # EXO-74, as it applies to a ramp. The plate does NOT run out past the rim across the
    # whole band (Link 4 R's does not either - the climb starts at s 16 on a r 27 pad), so
    # the test is not the shoulder L's. What must hold is that there is no POCKET: over the
    # pad's own thickness, every point just outside the rim is either inside the link OR
    # under nothing at all, i.e. the rim is a bare complete cylinder there and the ribbon has
    # already lifted clear of it. A cove would put material ABOVE an empty rim point.
    # What the ramp can guarantee, measured rather than assumed. At each station just
    # outside the rim the material must be ONE contiguous interval in z - no cove, no
    # undercut, nothing floating - and the strip of bare rim under it is the S's own tangent
    # rise at that station, which is reported. Nothing about it is a pocket: the pad's
    # rotor-side face is complete out to the rim and the ribbon leaves it TANGENT.
    NZ = 40
    split, bare, tested = [], 0.0, 0
    for wk in range(int(TAPER['w_lo']) + 2, int(TAPER['w_hi'])):
        w_at = wk * 1.0
        s_at = math.sqrt(max(0.0, (PAD_R + 0.05) ** 2 - w_at ** 2))
        col = [(zk * armgen.PAD_T / NZ, at(s_at, w_at, zk * armgen.PAD_T / NZ) in (INSIDE, ON))
               for zk in range(1, NZ)]
        tested += len(col)
        runs = sum(1 for i in range(1, len(col)) if col[i][1] and not col[i - 1][1])
        runs += 1 if col[0][1] else 0
        if runs > 1:
            split.append(round(w_at, 1))
        solid = [z for z, ok in col if ok]
        if solid:
            bare = max(bare, min(solid))
    check('EXO-74: no cove and no undercut - %d points just outside r %.1f, at most ONE '
          'solid interval in z per station' % (tested, PAD_R), not split,
          'split columns at w %s' % split[:8])
    # the deepest bare strip, and what the S's own geometry says it must be at w = 0
    want_bare = RISE_R - math.sqrt(max(0.0, RISE_R ** 2 - (PAD_R - S_DEP_LO) ** 2))
    near('the bare strip of rim under the climb is the S\'s own tangent rise '
         '(Link 4 R: 3.30 mm at r 27)', bare, want_bare, 0.4)
    log('    (the plate leaves the pad plane at s = %.2f, %.2f mm inside the r %.1f rim, so '
        'up to %.2f mm of rim shows under it - Link 4 R leaves at s 16.0 and shows 3.30)'
        % (S_DEP_LO, PAD_R - S_DEP_LO, PAD_R, bare))
    # the ribbon is solid all the way along its centre line
    gap = []
    for k in range(1, 40):
        # start past the rotor pad: inside it the 4x M3 clearance holes are real voids
        s_at = PAD_R + 1.0 + (S_LO - PAD_R - 3.0) * k / 40.0
        span = armgen.taper_w(TAPER, s_at)
        z_lo = min(q[1] for sg in LG['profile'] for q in armgen._seg_points(sg, 0.5)
                   if abs(q[0] - s_at) < 0.6) if True else 0.0
        z_hi = max(q[1] for sg in LG['profile'] for q in armgen._seg_points(sg, 0.5)
                   if abs(q[0] - s_at) < 0.6)
        if at(s_at, (span[0] + span[1]) / 2.0, (z_lo + z_hi) / 2.0) not in (INSIDE, ON):
            gap.append(round(s_at, 1))
    check('the ribbon is solid from the pad to the slab', not gap, 'hollow at s %s' % gap[:8])
    # and nothing sticks out past the taper: 1 mm outside the near wall must be empty
    out = [round(s_at, 1) for s_at in [20.0 + 4.0 * k for k in range(11)]
           if at(s_at, armgen.taper_w(TAPER, s_at)[1] + 1.0, armgen.PAD_T / 2.0) in (INSIDE, ON)]
    check('nothing outside the tapered near wall', not out, 'material at s %s' % out[:8])
except Exception as _e:                                     # noqa: BLE001
    log('note: pointContainment unavailable (%s); the joins are checked by their faces only' % _e)

# ---- 9. EXO-61's two defect signatures must not come back ----
tiny = [round(a, 4) for a in areas if a < 0.5]
check('no degenerate faces (< 0.5 mm2)', not tiny, 'got %s' % tiny)
dupes = []
for i in range(len(wwalls)):
    for j in range(i + 1, len(wwalls)):
        if abs(wwalls[i][0] - wwalls[j][0]) < 0.05:
            dupes.append((wwalls[i][0], wwalls[j][0]))
check('no two side-wall planes nearly coplanar (no seam)', not dupes, 'got %s' % dupes)

log('')
if fails:
    log('MEASURED-BODY CHECK FAILED: %s' % fails)
    raise RuntimeError('built link_45 does not match armgen: %d check(s) failed' % len(fails))
log('measured-body check: all checks passed')
