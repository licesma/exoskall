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
BG = armgen.link_bar_geometry(R_23, t_23, 27.0)
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
log('expected riser S: shift %.3f mm over s %.3f..%.3f, centre arcs r %.1f, wall arcs r %.1f / %.1f '
    'about (s,w) %s and %s' % (abs(SC['w_off']), SC['s_start'], SC['s_end'], SC['r'],
                               SC['wall_r'][0], SC['wall_r'][1],
                               tuple(round(v, 3) for v in SC['centres'][0]),
                               tuple(round(v, 3) for v in SC['centres'][1])))

ST = {0: 'plane', 1: 'cyl', 2: 'cone', 3: 'sphere', 4: 'torus', 5: 'ellcyl', 6: 'ellcone', 7: 'nurbs'}
kinds = {}
cyl, ywalls, uwalls = [], [], []
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
s_faces = [c for c in cyl if abs(c[1][2]) > 0.999 and c[3][2][1] < 0.0
           and min(abs(c[0] - SC['wall_r'][0]), abs(c[0] - SC['wall_r'][1])) < TOL]
check('four riser-S wall arc faces (two per wall)', len(s_faces) == 4,
      'got %d: %s' % (len(s_faces), [(c[0], tuple(round(v, 2) for v in c[5])) for c in s_faces]))
want_axes = {}
for k, c in enumerate(SC['centres']):
    for r in SC['wall_r']:
        want_axes[(round(r, 2), k)] = c
for c in s_faces:
    r = SC['wall_r'][0] if abs(c[0] - SC['wall_r'][0]) < TOL else SC['wall_r'][1]
    k = 0 if abs(c[5][0] - SC['centres'][0][0]) < abs(c[5][0] - SC['centres'][1][0]) else 1
    near('S arc r %.1f axis on the computed centre %d' % (r, k + 1), c[5], SC['centres'][k])
    near('S arc r %.1f axis parallel to the pad axis' % r, abs(c[1][2]), 1.0, 1e-3)
    log('    S arc r %.1f about (s,w) = (%.3f, %.3f), s %.2f..%.2f, w %.2f..%.2f, z %.1f..%.1f, %.0f mm2'
        % (c[0], c[5][0], c[5][1], c[6][0], c[6][1], c[7][0], c[7][1], c[3][2][0], c[3][2][1], c[4]))
got_r = sorted(round(c[0], 2) for c in s_faces)
near('S wall arc radii are r %.1f x2 and r %.1f x2' % SC['wall_r'], got_r,
     sorted([SC['wall_r'][0]] * 2 + [SC['wall_r'][1]] * 2))

# ---- 3. each wall is plane -> arc -> arc -> plane with no gap along the bar ----
for side, sign in (('-', -1.0), ('+', 1.0)):
    w_pad = sign * HALF
    w_slab = SC['w_off'] + sign * HALF
    pw = [t for t in pad_walls if abs(t[0] - w_pad) < TOL]
    sw_ = [t for t in slab_walls if abs(t[0] - w_slab) < TOL]
    arcs = [c for c in s_faces if abs(c[7][0] - min(w_pad, w_slab)) < 1.0
            or abs(c[7][1] - max(w_pad, w_slab)) < 1.0]
    arcs = sorted([c for c in s_faces
                   if min(abs(c[7][0] - w_pad), abs(c[7][1] - w_pad),
                          abs(c[7][0] - w_slab), abs(c[7][1] - w_slab)) < 0.05], key=lambda c: c[6][0])
    if len(pw) == 1 and len(sw_) == 1 and len(arcs) == 2:
        near('wall %s12.5: plane ends where the first S arc starts' % side,
             arcs[0][6][0], pw[0][3][1])
        near('wall %s12.5: the two S arcs meet (inflection at s=%.2f)' % (side, SC['inflection'][0]),
             arcs[0][6][1], arcs[1][6][0])
        near('wall %s12.5: inflection at the computed station' % side,
             arcs[0][6][1], SC['inflection'][0], 0.05)
        near('wall %s12.5: the second S arc ends where the slab-end plane starts' % side,
             arcs[1][6][1], sw_[0][3][0])
        near('wall %s12.5: S arcs span exactly the computed run' % side,
             (arcs[0][6][0], arcs[1][6][1]), (SC['s_start'], SC['s_end']), 0.05)
    else:
        check('wall %s12.5 is plane -> arc -> arc -> plane' % side, False,
              '%d pad planes, %d slab planes, %d arcs' % (len(pw), len(sw_), len(arcs)))

# ---- 4. the riser inner face still clears the module by RISER_CLEAR ----
inner = [t for t in uwalls if abs(t[0] - BG['s_in']) < TOL]
check('riser inner face is one plane at s = s_in', len(inner) == 1, 'got %s' % [t[0] for t in uwalls])
if inner:
    near('riser inner face is RISER_CLEAR outside r 27', inner[0][0] - 27.0, armgen.RISER_CLEAR)
    log('    riser inner face at s=%.3f spans w %.3f..%.3f (centre %.3f, %.2f wide), %.0f mm2'
        % (inner[0][0], inner[0][3][0], inner[0][3][1],
           (inner[0][3][0] + inner[0][3][1]) / 2.0, inner[0][3][1] - inner[0][3][0], inner[0][2]))
    near('riser inner face is centred on the J2 pad',
         (inner[0][3][0] + inner[0][3][1]) / 2.0, 0.0, 0.1)

log('')
if fails:
    log('MEASURED-BODY CHECK FAILED: %s' % fails)
    raise RuntimeError('built link_23 does not match armgen: %d check(s) failed' % len(fails))
log('measured-body check: all checks passed')
