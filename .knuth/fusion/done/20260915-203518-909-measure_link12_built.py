# EXO-81: measure the BUILT link_12 body (the shoulder 'L') in the J1 module frame and check
# it against armgen.route's numbers. Same discipline as measure_link23_built.py (EXO-63: a
# green interference check is not proof - this is the measurement of the real body):
#   - one solid body, no degenerate faces, no two nearly-coplanar side walls
#   - the plate is COPLANAR with the rotor pad: its rotor-side face is the pad's own z = 0
#     plane and its outer face the pad's z = PAD_T plane, one face each, running out past
#     the rim (EXO-74's join standard, which the L meets by construction)
#   - exactly two side-wall planes at w = -/+ BAR_W/2, centred on the rotor pad axis
#   - the elbow: one r L_FILLET_IN concave fillet and one r `r_out` convex round, both with
#     their axis along the plate width and their measured Link 1 R centres
#   - nothing anywhere below z = 0 except the link's own stop block and index tab
#   - the rim is covered: sampled points just outside the pad rim, over the whole pad
#     thickness and the whole band width, are inside the link
#   - the stator slab: the r 27 disk about the J2 axis, the tongue's two walls in line with
#     the plate's, and the two blends at the radii and centres armgen chose
import math
import itertools
import sys
import importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
importlib.reload(armgen)

REPO = '/Users/esteban/repos/exo'
TOL = 0.02          # mm: face positions must match the computed profile this closely
PAD_R = 26.0        # link_12's rotor pad (BuildArm.LINKS)

# ---- expected numbers, recomputed from the URDF exactly as BuildArm does ----
joints, by_child = armgen.parse_urdf(REPO + '/g1/g1_29dof.urdf')
shift, _ = armgen.joint_world_pose('left_shoulder_pitch_joint', joints, by_child)
JSPEC = {'J1': ('right_shoulder_pitch_joint', -1, (0, 0, 1)),
         'J2': ('right_shoulder_roll_joint', +1, (0, 1, 0))}
FR = {}
for lab, (nm, sgn, yh) in JSPEC.items():
    R, t = armgen.module_frame(nm, joints, by_child, sgn, yh)
    FR[lab] = (R, [t[i] - shift[i] for i in range(3)])
SPEC = armgen.route(FR['J1'], FR['J2'], PAD_R)
if SPEC['pattern'] != 'L':
    raise RuntimeError('the router no longer picks the L pattern for J1->J2: %r' % SPEC['pattern'])
LG = SPEC['geometry']
U, W = LG['u'], LG['w']            # plate direction and width direction, in the J1 frame
HALF = LG['half_w']
PAD_EXIT = math.sqrt(PAD_R ** 2 - HALF ** 2)     # where the band leaves the pad rim
R_IN, R_OUT = LG['r_in'], LG['r_out']
S_LO, S_HI = LG['s_lo'], LG['s_hi']

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


# ---- the built body, in the J1 module frame ----
root = design.rootComponent
arm = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name.startswith('generated_R_arm'):
        arm = o
if arm is None:
    raise RuntimeError('no generated_R_arm occurrence in the document')
j1 = link = None
for i in range(arm.childOccurrences.count):
    o = arm.childOccurrences.item(i)
    if o.component.name.startswith('J1_'):
        j1 = o
    if o.component.name.startswith('link_12'):
        link = o
if j1 is None or link is None:
    raise RuntimeError('J1 wrapper or link_12 not found under %s' % arm.component.name)
inv = j1.transform.copy()
inv.invert()
check('link_12 is ONE solid body', link.bRepBodies.count == 1,
      '%d bodies' % link.bRepBodies.count)
body = link.bRepBodies.item(0)


def toJ1(p):
    q = p.copy()
    q.transformBy(inv)
    return (q.x * 10, q.y * 10, q.z * 10)


def dirJ1(v):
    w = v.copy()
    w.transformBy(inv)
    return (w.x, w.y, w.z)


def sw(p):
    """(s, w) of a J1-frame point in the plate's own plan coordinates."""
    return (p[0] * U[0] + p[1] * U[1], p[0] * W[0] + p[1] * W[1])


bb = body.boundingBox
pts = [toJ1(adsk.core.Point3D.create(x, y, z)) for x, y, z in itertools.product(
    (bb.minPoint.x, bb.maxPoint.x), (bb.minPoint.y, bb.maxPoint.y), (bb.minPoint.z, bb.maxPoint.z))]
log('link_12 (pattern %s): %d faces, volume %.0f mm3, bbox in J1 frame '
    'x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]'
    % (SPEC['pattern'], body.faces.count, body.volume * 1000,
       min(p[0] for p in pts), max(p[0] for p in pts), min(p[1] for p in pts),
       max(p[1] for p in pts), min(p[2] for p in pts), max(p[2] for p in pts)))
log('expected: plate z %.1f..%.1f out to the slab faces s %.2f/%.2f, elbow fillet r %.1f at '
    '(s,z) (%.2f, %.2f) and round r %.2f at (%.2f, %.2f); tongue x_off %.3f, blends r %.1f / %.1f'
    % (LG['plate_z'][0], LG['plate_z'][1], S_LO, S_HI, R_IN, S_LO - R_IN,
       armgen.PAD_T + R_IN, R_OUT, S_HI - R_OUT, R_OUT, LG['x_off'],
       LG['blend_r'][0], LG['blend_r'][1]))

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
    cs = [toJ1(adsk.core.Point3D.create(x, y, z)) for x, y, z in itertools.product(
        (fb.minPoint.x, fb.maxPoint.x), (fb.minPoint.y, fb.maxPoint.y), (fb.minPoint.z, fb.maxPoint.z))]
    ext = tuple((min(c[a] for c in cs), max(c[a] for c in cs)) for a in range(3))
    s_ext = (min(sw(c)[0] for c in cs), max(sw(c)[0] for c in cs))
    w_ext = (min(sw(c)[1] for c in cs), max(sw(c)[1] for c in cs))
    z_ext = ext[2]
    if k == 'cyl':
        ax = dirJ1(g.axis)
        o = toJ1(g.origin)
        cyl.append((round(g.radius * 10, 3), tuple(round(v, 3) for v in ax),
                    tuple(round(v, 3) for v in o), ext, f.area * 100, sw(o), s_ext, w_ext))
    elif k == 'plane':
        n = dirJ1(g.normal)
        o = toJ1(g.origin)
        if abs(n[0] * W[0] + n[1] * W[1]) > 0.999:
            wwalls.append((round(sw(o)[1], 3), ext, f.area * 100, s_ext, z_ext))
        elif abs(n[0] * U[0] + n[1] * U[1]) > 0.999:
            uwalls.append((round(sw(o)[0], 3), ext, f.area * 100, w_ext, z_ext))
        elif abs(n[2]) > 0.999:
            zfaces.append((round(o[2], 3), ext, f.area * 100, s_ext, w_ext))
log('face kinds: %s' % kinds)
log('cylindrical faces (r, axis J1, a point on axis J1, bbox J1, area mm2):')
for c in sorted(cyl):
    log('  r %7.3f axis %s at %s  x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f]  %.0f mm2'
        % (c[0], c[1], c[2], c[3][0][0], c[3][0][1], c[3][1][0], c[3][1][1],
           c[3][2][0], c[3][2][1], c[4]))
log('planar side-wall faces (normal along the plate width): w, s range, z range, area')
for w, ext, a, s_ext, z_ext in sorted(wwalls):
    log('  w %7.2f  s[%.2f,%.2f]  z[%.2f,%.2f]  %.0f mm2'
        % (w, s_ext[0], s_ext[1], z_ext[0], z_ext[1], a))
log('planar faces normal to the pad axis: z, s range, w range, area')
for z, ext, a, s_ext, w_ext in sorted(zfaces):
    log('  z %7.2f  s[%.2f,%.2f]  w[%.2f,%.2f]  %.0f mm2'
        % (z, s_ext[0], s_ext[1], w_ext[0], w_ext[1], a))
log('planar faces normal to the plate direction u: s, w range, z range, area')
for s, ext, a, w_ext, z_ext in sorted(uwalls):
    log('  s %7.2f  w[%.2f,%.2f]  z[%.2f,%.2f]  %.0f mm2'
        % (s, w_ext[0], w_ext[1], z_ext[0], z_ext[1], a))

# ---- 1. the plate is coplanar with the rotor pad: ONE face per pad plane ----
z0 = [t for t in zfaces if abs(t[0]) < TOL]
z8 = [t for t in zfaces if abs(t[0] - armgen.PAD_T) < TOL]
check('the pad rotor-side plane z = 0 and the plate are ONE face', len(z0) == 1,
      'got %d faces at z = 0' % len(z0))
check('the pad outer plane z = %.1f and the plate are ONE face' % armgen.PAD_T, len(z8) == 1,
      'got %d faces at z = %.1f' % (len(z8), armgen.PAD_T))
if z0:
    near('rotor-side face runs out to the outer round (s_hi - r_out)', z0[0][3][1],
         S_HI - R_OUT, 0.05)
    check('rotor-side face crosses the rim FLAT (%.2f mm past r %.1f)'
          % (S_HI - R_OUT - PAD_R, PAD_R), z0[0][3][1] >= PAD_R + armgen.JOIN_RIM_MARGIN - TOL,
          'reaches s = %.3f' % z0[0][3][1])
if z8:
    near('outer face runs out to the inner fillet (s_lo - r_in)', z8[0][3][1], S_LO - R_IN, 0.05)

# ---- 2. exactly two side-wall planes, centred on the rotor pad axis ----
plate_walls = [t for t in wwalls if t[3][0] < PAD_EXIT + TOL and t[4][1] <= armgen.PAD_T + TOL]
check('exactly two side-wall planes on the plate', len(plate_walls) == 2,
      'got %s' % [t[0] for t in plate_walls])
if len(plate_walls) == 2:
    lo, hi = sorted(t[0] for t in plate_walls)
    near('plate side walls at -/+ BAR_W/2', (lo, hi), (-HALF, HALF))
    near('plate band width / centre', (hi - lo, (hi + lo) / 2.0), (armgen.BAR_W, 0.0))

# ---- 3. the elbow: the inner fillet and the outer round, axes along the plate width ----
def width_cyl(r):
    return [c for c in cyl if abs(c[0] - r) < 0.01
            and abs(c[1][0] * W[0] + c[1][1] * W[1]) > 0.99]


fil = width_cyl(R_IN)
rnd = width_cyl(R_OUT)
check('one r %.1f inner fillet across the plate width' % R_IN, len(fil) == 1,
      '%d faces' % len(fil))
check('one r %.2f outer round across the plate width' % R_OUT, len(rnd) == 1,
      '%d faces' % len(rnd))
if fil:
    near('inner fillet centre (s, z) = Link 1 R (%.2f, %.2f)' % (S_LO - R_IN, armgen.PAD_T + R_IN),
         (fil[0][5][0], fil[0][2][2]), (S_LO - R_IN, armgen.PAD_T + R_IN))
    near('inner fillet spans the whole band', (fil[0][7][0], fil[0][7][1]), (-HALF, HALF), 0.05)
if rnd:
    near('outer round centre (s, z)', (rnd[0][5][0], rnd[0][2][2]), (S_HI - R_OUT, R_OUT))
check('the measured r %.1f round is NOT used (the rim rule caps it at r %.2f)'
      % (LG['r_out_measured'], R_OUT), not width_cyl(LG['r_out_measured']),
      'an r %.1f face is present' % LG['r_out_measured'])

# ---- 4. the stator slab: disk, tongue walls in line with the plate, blends ----
axis_u = [c for c in cyl if abs(c[1][0] * U[0] + c[1][1] * U[1]) > 0.99]
disk = [c for c in axis_u if abs(c[0] - armgen.STATOR_PAD_R) < 0.01]
check('the r %.1f stator disk is there, axis along u' % armgen.STATOR_PAD_R, len(disk) >= 1,
      '%d faces' % len(disk))
for i, r in enumerate(LG['blend_r']):
    bl = [c for c in axis_u if abs(c[0] - r) < 0.01]
    check('tongue blend %d: an r %.1f face about the J2 axis' % (i, r), len(bl) >= 1,
          '%d faces' % len(bl))
slab_walls = [t for t in wwalls if t[3][0] >= S_LO - TOL and t[4][1] > armgen.PAD_T]
if slab_walls:
    near('tongue walls are the plate walls (no step)',
         (min(t[0] for t in slab_walls), max(t[0] for t in slab_walls)), (-HALF, HALF), 0.05)

# ---- 5. nothing below the pad's rotor-side plane except the stop block and index tab ----
deep = [t for t in zfaces if t[0] < -TOL]
low = min(p[2] for p in pts)
near('the body reaches only the stop block depth below z = 0', low, -3.6, 0.05)
check('every face below z = 0 belongs to the stop block / index tab (|s| <= pad_r)',
      all(abs(t[3][0]) <= PAD_R + TOL and abs(t[3][1]) <= PAD_R + TOL for t in deep),
      'got %s' % [(t[0], t[3]) for t in deep])

# ---- 6. the rim is covered over the whole pad thickness, across the band ----
try:
    INSIDE = adsk.fusion.PointContainment.PointInsidePointContainment
    ON = adsk.fusion.PointContainment.PointOnPointContainment
    fwd = j1.transform

    def at(s_at, w_at, z_at):
        p = adsk.core.Point3D.create((U[0] * s_at + W[0] * w_at) / 10.0,
                                     (U[1] * s_at + W[1] * w_at) / 10.0, z_at / 10.0)
        p.transformBy(fwd)
        return body.pointContainment(p)

    missed, tested = [], 0
    for wk in range(-12, 13):
        w_at = wk * 1.0
        s_at = math.sqrt((PAD_R + 0.05) ** 2 - w_at ** 2)
        for zk in range(1, 16):
            z_at = zk * 0.5
            tested += 1
            if at(s_at, w_at, z_at) not in (INSIDE, ON):
                missed.append((round(w_at, 1), round(z_at, 2)))
    check('EXO-74: rim covered - %d points at r %.2f across the %.0f mm width, z 0.5..7.5, '
          'are inside the link' % (tested, PAD_R + 0.05, armgen.BAR_W), not missed,
          'uncovered at (w, z) %s' % missed[:8])
    # and nothing of the L is below the rotor face outside the pad: the whole EXO-77
    # stop-block annulus along the plate is free
    below = [(round(w_at, 1), round(z_at, 2))
             for wk in range(-12, 13) for zk in (1, 2, 4, 8)
             for w_at in (wk * 1.0,) for z_at in (-zk * 0.4,)
             if at(math.sqrt(max(0.0, (PAD_R + 0.05) ** 2 - w_at ** 2)) + 0.5, w_at, z_at)
             in (INSIDE, ON)]
    check('nothing of the L reaches below the rotor face outside the rim', not below,
          'found at (w, z) %s' % below[:8])
except Exception as _e:                                     # noqa: BLE001
    log('note: pointContainment unavailable (%s); the join is checked by its faces only' % _e)

# ---- 7. EXO-61's two defect signatures must not come back ----
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
    raise RuntimeError('built link_12 does not match armgen: %d check(s) failed' % len(fails))
log('measured-body check: all checks passed')
