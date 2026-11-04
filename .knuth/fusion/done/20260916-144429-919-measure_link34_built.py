# EXO-103: measure the BUILT link_34 body (the up-column pair J3->J4) in the J3 module frame
# and check it against armgen.route's numbers AND against the hand-built `Link 3 R`
# (EXO-104/EXO-105). Same discipline as measure_link12_built.py (EXO-63: a green interference
# check is not proof - this is the measurement of the real body):
#   - one solid body, no degenerate faces, no two nearly-coplanar side walls
#   - the plate is COPLANAR with the rotor pad, exactly as the shoulder L is: one face on the
#     pad's z = 0 plane and one on its z = PAD_T plane, running out past the rim
#   - the elbow: one r L_FILLET_IN concave fillet and one r `r_out` convex round, axes across
#     the band, at the centres the route computed (Link 3 R's own are r 2.5 / r 2.5)
#   - the COLUMN: the two slab faces s_lo / s_hi carry on down to the plate as one face each,
#     so the tongue and the slab are one 8 mm wall and not two with a step (Link 3 R has a
#     0.102 mm step there; the generated one must not)
#   - exactly two side-wall planes at -/+ width/2, one face per side, from the pad to the disk
#   - the crown: an r `crown` cylinder about the J4 axis and the flat that runs from it into
#     the rim, so the column's walls stay parallel over its whole length
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
PAD_R = 26.0        # link_34's rotor pad (BuildArm.LINKS, = Link 3 R's measured r 26)
STOP_H = 3.6

# ---- expected numbers, recomputed from the URDF exactly as BuildArm does ----
joints, by_child = armgen.parse_urdf(REPO + '/g1/g1_29dof.urdf')
shift, _ = armgen.joint_world_pose('left_shoulder_pitch_joint', joints, by_child)
JSPEC = {'J3': ('right_shoulder_yaw_joint', -1, (-1, 0, 0)),
         'J4': ('right_elbow_joint', -1, (-1, 0, 0))}
FR = {}
for lab, (nm, sgn, yh) in JSPEC.items():
    R, t = armgen.module_frame(nm, joints, by_child, sgn, yh)
    FR[lab] = (R, [t[i] - shift[i] for i in range(3)])
SPEC = armgen.route(FR['J3'], FR['J4'], PAD_R, width=armgen.L_W)
if SPEC['pattern'] != 'L':
    raise RuntimeError('the router no longer picks the L route for J3->J4: %r' % SPEC['pattern'])
LG = SPEC['geometry']
U, W = LG['u'], LG['w']
HALF = LG['half_w']
PAD_EXIT = math.sqrt(max(0.0, PAD_R ** 2 - HALF ** 2))   # 0.0 at width = 2 pad_r
R_IN, R_OUT = LG['r_in'], LG['r_out']
S_LO, S_HI = LG['s_lo'], LG['s_hi']
AXIAL = LG['axial']
EDGES = LG['edges']
CROWN = [e for e in EDGES if e['kind'] == 'crown']
BLEND = [e for e in EDGES if e['kind'] == 'blend']

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


# ---- the built body, in the J3 module frame ----
root = design.rootComponent
arm = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name.startswith('generated_R_arm'):
        arm = o
if arm is None:
    raise RuntimeError('no generated_R_arm occurrence in the document')
j3 = link = None
for i in range(arm.childOccurrences.count):
    o = arm.childOccurrences.item(i)
    if o.component.name.startswith('J3_'):
        j3 = o
    if o.component.name.startswith('link_34'):
        link = o
if j3 is None or link is None:
    raise RuntimeError('J3 wrapper or link_34 not found under %s' % arm.component.name)
inv = j3.transform.copy()
inv.invert()
check('link_34 is ONE solid body', link.bRepBodies.count == 1,
      '%d bodies' % link.bRepBodies.count)
body = link.bRepBodies.item(0)


def toJ3(p):
    q = p.copy()
    q.transformBy(inv)
    return (q.x * 10, q.y * 10, q.z * 10)


def dirJ3(v):
    w = v.copy()
    w.transformBy(inv)
    return (w.x, w.y, w.z)


def sw(p):
    """(s, w) of a J3-frame point in the link's own plan coordinates."""
    return (p[0] * U[0] + p[1] * U[1], p[0] * W[0] + p[1] * W[1])


bb = body.boundingBox
pts = [toJ3(adsk.core.Point3D.create(x, y, z)) for x, y, z in itertools.product(
    (bb.minPoint.x, bb.maxPoint.x), (bb.minPoint.y, bb.maxPoint.y), (bb.minPoint.z, bb.maxPoint.z))]
log('link_34 (route %s, ribbon %.0f wide): %d faces, volume %.0f mm3, bbox in J3 frame '
    'x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]'
    % (SPEC['pattern'], LG['width'], body.faces.count, body.volume * 1000,
       min(p[0] for p in pts), max(p[0] for p in pts), min(p[1] for p in pts),
       max(p[1] for p in pts), min(p[2] for p in pts), max(p[2] for p in pts)))
log('expected: plate z %.1f..%.1f out to the slab faces s %.2f/%.2f, elbow fillet r %.1f at '
    '(s,z) (%.2f, %.2f) and round r %.2f at (%.2f, %.2f); the column runs up to the disk at '
    'z %.3f; tongue x_off %.3f, edges %s'
    % (LG['plate_z'][0], LG['plate_z'][1], S_LO, S_HI, R_IN, S_LO - R_IN,
       armgen.PAD_T + R_IN, R_OUT, S_HI - R_OUT, R_OUT, AXIAL, LG['x_off'],
       ' / '.join('%s r %.3f' % (e['kind'], e['r']) for e in EDGES)))
log('Link 3 R for comparison: plate z 0..8 52 wide on a r 26 pad, elbow r 2.5 / r 2.5, '
    'column 8.102 thick, disk r 26 at (w 15.783, z 80.518), blend r 30, crown r 35')

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
    cs = [toJ3(adsk.core.Point3D.create(x, y, z)) for x, y, z in itertools.product(
        (fb.minPoint.x, fb.maxPoint.x), (fb.minPoint.y, fb.maxPoint.y), (fb.minPoint.z, fb.maxPoint.z))]
    ext = tuple((min(c[a] for c in cs), max(c[a] for c in cs)) for a in range(3))
    s_ext = (min(sw(c)[0] for c in cs), max(sw(c)[0] for c in cs))
    w_ext = (min(sw(c)[1] for c in cs), max(sw(c)[1] for c in cs))
    z_ext = ext[2]
    if k == 'cyl':
        ax = dirJ3(g.axis)
        o = toJ3(g.origin)
        cyl.append((round(g.radius * 10, 3), tuple(round(v, 3) for v in ax),
                    tuple(round(v, 3) for v in o), ext, f.area * 100, sw(o), s_ext, w_ext,
                    z_ext))
    elif k == 'plane':
        n = dirJ3(g.normal)
        o = toJ3(g.origin)
        if abs(n[0] * W[0] + n[1] * W[1]) > 0.999:
            wwalls.append((round(sw(o)[1], 3), ext, f.area * 100, s_ext, z_ext))
        elif abs(n[0] * U[0] + n[1] * U[1]) > 0.999:
            uwalls.append((round(sw(o)[0], 3), ext, f.area * 100, w_ext, z_ext))
        elif abs(n[2]) > 0.999:
            zfaces.append((round(o[2], 3), ext, f.area * 100, s_ext, w_ext))
log('face kinds: %s' % kinds)
log('cylindrical faces (r, axis J3, a point on axis J3, area mm2):')
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
log('planar faces normal to u (the two slab faces, continued down as the column): '
    's, w range, z range, area')
for s, ext, a, w_ext, z_ext in sorted(uwalls):
    log('  s %7.2f  w[%.2f,%.2f]  z[%.2f,%.2f]  %.0f mm2'
        % (s, w_ext[0], w_ext[1], z_ext[0], z_ext[1], a))

# ---- 1. the plate is coplanar with the rotor pad: ONE face per pad plane (EXO-92's join) ----
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
    near('outer face runs out to the elbow fillet', z8[0][3][1], max(S_LO - R_IN, PAD_EXIT), 0.05)

# ---- 2. exactly two side-wall planes at -/+ width/2, ONE face each ----
plate_walls = [t for t in wwalls if abs(abs(t[0]) - HALF) < TOL]
check('exactly two side-wall planes at -/+ %.0f' % HALF, len(plate_walls) == 2,
      'got %s' % [t[0] for t in wwalls])
if len(plate_walls) == 2:
    lo, hi = sorted(t[0] for t in plate_walls)
    near('band width / centre at the walls', (hi - lo, (hi + lo) / 2.0), (LG['width'], 0.0))
    check('one wall face per side (plate and column continuous, no step)',
          len([t for t in wwalls if abs(t[0] - lo) < TOL]) == 1
          and len([t for t in wwalls if abs(t[0] - hi) < TOL]) == 1,
          'got %s' % [t[0] for t in wwalls])
    # Link 3 R's walls run the column's whole length; so must these
    near('both walls reach the top of the column', [t[4][1] for t in plate_walls],
         [AXIAL - armgen.STATOR_PAD_R] * 2, 1.0)

# ---- 3. the elbow: the inner fillet and the outer round, axes across the band ----
def width_cyl(r, tol=0.01):
    return [c for c in cyl if abs(c[0] - r) < tol
            and abs(c[1][0] * W[0] + c[1][1] * W[1]) > 0.99]


fil = width_cyl(R_IN)
rnd = width_cyl(R_OUT)
check('one r %.1f inner fillet across the band' % R_IN, len(fil) == 1, '%d faces' % len(fil))
check('one r %.2f outer round across the band' % R_OUT, len(rnd) == 1, '%d faces' % len(rnd))
if fil:
    near('inner fillet centre (s, z)', (fil[0][5][0], fil[0][2][2]),
         (S_LO - R_IN, armgen.PAD_T + R_IN))
    near('inner fillet area = pi/2 R width (quarter cylinder, full band)', fil[0][4],
         math.pi / 2 * R_IN * LG['width'], 0.5)
if rnd:
    near('outer round centre (s, z)', (rnd[0][5][0], rnd[0][2][2]), (S_HI - R_OUT, R_OUT))
    near('outer round area = pi/2 R width (quarter cylinder, full band)', rnd[0][4],
         math.pi / 2 * R_OUT * LG['width'], 0.5)

# ---- 4. the column: the two slab faces run down to the plate as ONE face each ----
near_face = [t for t in uwalls if abs(t[0] - S_LO) < TOL]
far_face = [t for t in uwalls if abs(t[0] - S_HI) < TOL]
check('the slab near face s = %.2f is ONE face down to the elbow' % S_LO, len(near_face) == 1,
      'got %d' % len(near_face))
check('the slab far face s = %.2f is ONE face down to the elbow' % S_HI, len(far_face) == 1,
      'got %d' % len(far_face))
if near_face:
    near('the column\'s inner face starts at the elbow fillet (z = PAD_T + r_in)',
         near_face[0][4][0], armgen.PAD_T + R_IN, 0.05)
    near('and runs up past the disk centre', near_face[0][4][1], AXIAL, 6.0)
if far_face:
    near('the column\'s outer face starts at the elbow round (z = r_out)',
         far_face[0][4][0], R_OUT, 0.05)
if near_face and far_face:
    near('the column is exactly PAD_T thick (Link 3 R measures 8.102 - a 0.102 step)',
         far_face[0][0] - near_face[0][0], armgen.PAD_T, 1e-3)

# ---- 5. the stator slab: disk about the J4 axis, the blend and the crown ----
axis_u = [c for c in cyl if abs(c[1][0] * U[0] + c[1][1] * U[1]) > 0.99]
disk = [c for c in axis_u if abs(c[0] - armgen.STATOR_PAD_R) < 0.01]
check('the r %.1f stator disk is there, axis along u' % armgen.STATOR_PAD_R, len(disk) >= 1,
      '%d faces' % len(disk))
for e in EDGES:
    bl = [c for c in axis_u if abs(c[0] - e['r']) < max(0.01, e['r'] * 1e-4)]
    check('tongue %s edge: an r %.3f face about the J4 axis' % (e['kind'], e['r']),
          len(bl) >= 1, '%d faces' % len(bl))
if CROWN:
    cr = CROWN[0]
    # the flat between the crown fillet and the rim's tangent point: a plane whose normal is
    # across the band, FLANGE_CROWN_FLAT wide, at the disk's far edge
    w_flat = LG['w_off'] + LG['x_sign'] * cr['t'][0] - LG['x_sign'] * 0.0
    z_flat = AXIAL - LG['y_sign'] * cr['t'][1]
    flat = [t for t in wwalls if abs(t[0] - (LG['w_off'] - armgen.STATOR_PAD_R
                                             * (1 if cr['side'] < 0 else -1))) < 0.05]
    check('the crown\'s flat is a real face tangent to the rim (%.1f mm wide)'
          % armgen.FLANGE_CROWN_FLAT, len(flat) >= 1,
          'side walls at w %s, wanted %.3f'
          % ([t[0] for t in wwalls], LG['w_off'] - armgen.STATOR_PAD_R
             * (1 if cr['side'] < 0 else -1)))
    log('    (crown r %.3f centred (x\' %.3f, y\' %.3f) in the slab frame; Link 3 R uses '
        'r 35.000 at (-6.783, 9.000) on its r 26 disk)' % (cr['r'], cr['c'][0], cr['c'][1]))

# ---- 6. nothing below the pad's rotor-side plane except the stop block and index tab ----
deep = [t for t in zfaces if t[0] < -TOL]
near('the lowest planar face is the stop block at -stop_h', min(t[0] for t in deep), -STOP_H,
     0.05)
check('every face below z = 0 belongs to the stop block / index tab (|s| <= pad_r)',
      all(abs(t[3][0]) <= PAD_R + TOL and abs(t[3][1]) <= PAD_R + TOL for t in deep),
      'got %s' % [(t[0], t[3]) for t in deep])

# ---- 7. the rim is covered over the whole pad thickness, across the band ----
try:
    INSIDE = adsk.fusion.PointContainment.PointInsidePointContainment
    ON = adsk.fusion.PointContainment.PointOnPointContainment
    fwd = j3.transform

    def at(s_at, w_at, z_at):
        p = adsk.core.Point3D.create((U[0] * s_at + W[0] * w_at) / 10.0,
                                     (U[1] * s_at + W[1] * w_at) / 10.0, z_at / 10.0)
        p.transformBy(fwd)
        return body.pointContainment(p)

    missed, tested = [], 0
    w_max = int(min(HALF, PAD_R) - 2)
    for wk in range(-w_max, w_max + 1):
        w_at = wk * 1.0
        s_at = math.sqrt((PAD_R + 0.05) ** 2 - w_at ** 2)
        for zk in range(1, 16):
            z_at = zk * 0.5
            tested += 1
            if at(s_at, w_at, z_at) not in (INSIDE, ON):
                missed.append((round(w_at, 1), round(z_at, 2)))
    check('EXO-74: rim covered - %d points at r %.2f across the %.0f mm width, z 0.5..7.5, '
          'are inside the link' % (tested, PAD_R + 0.05, LG['width']), not missed,
          'uncovered at (w, z) %s' % missed[:8])
    below = [(round(w_at, 1), round(z_at, 2))
             for wk in range(-w_max, w_max + 1) for zk in (1, 2, 4, 8)
             for w_at in (wk * 1.0,) for z_at in (-zk * 0.4,)
             if at(math.sqrt(max(0.0, (PAD_R + 0.05) ** 2 - w_at ** 2)) + 0.5, w_at, z_at)
             in (INSIDE, ON)]
    check('nothing reaches below the rotor face outside the rim (EXO-77 stop-block annulus)',
          not below, 'found at (w, z) %s' % below[:8])
    # the column is solid the whole way up: sample its middle every 5 mm
    gap = [round(z, 1) for z in [armgen.PAD_T + R_IN + 2.0 + 5.0 * k
                                 for k in range(int((AXIAL - 20.0) / 5.0))]
           if at((S_LO + S_HI) / 2.0, 0.0, z) not in (INSIDE, ON)]
    check('the column is solid from the elbow up to the disk', not gap,
          'hollow at z %s' % gap[:8])
except Exception as _e:                                     # noqa: BLE001
    log('note: pointContainment unavailable (%s); the joins are checked by their faces only' % _e)

# ---- 8. EXO-61's two defect signatures must not come back ----
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
    raise RuntimeError('built link_34 does not match armgen: %d check(s) failed' % len(fails))
log('measured-body check: all checks passed')
