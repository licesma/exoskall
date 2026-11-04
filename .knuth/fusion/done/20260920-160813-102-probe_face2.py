import sys, importlib, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm, armgen, sweepgen, BuildArmGen
for m in (armgen, sweepgen, BuildArm, BuildArmGen): importlib.reload(m)
import adsk.core, adsk.fusion
root = design.rootComponent
arm = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name == 'generated_R_arm_general': arm = o
joints, by_child = armgen.parse_urdf(BuildArm.URDF_PATH)
shift, _ = armgen.joint_world_pose(BuildArm.ORIGIN_AT_JOINT, joints, by_child)
LINKS = [(L['name'], L['a'], L['b'], L['pad_r'], L['width'])
         for L in list(BuildArm.LINKS) + BuildArmGen.NEW_LINKS]
ORDER = [j['label'] for j in BuildArm.JOINTS] + [j['label'] for j in BuildArmGen.NEW_JOINTS]
_s, _l, table = BuildArmGen.seed_frames(joints, by_child, shift)
seed, _c, SL, _r = BuildArmGen.solve_axis_signs(joints, by_child, shift, LINKS, table, lambda m: None, ORDER)
frames, spins = sweepgen.solve_spins(seed, LINKS)
RA, tA = frames['J6']; RB, tB = frames['J7']
R_ab, t_ab = armgen.relative_frame(RA, tA, RB, tB)
route = sweepgen.sweep_route(R_ab, t_ab, 27.0, 25.0)
armf = BuildArmGen.Arm(RA, [v*1000.0 for v in tA], route['u'], route['w'])
sc = route['numbers']['slab_centre']; x_t = route['x_t']; y_t = route['y_t']
def tong(p):
    q = armf.swz(p)
    d = (0.0, q[1]-sc[1], q[2]-sc[2])
    return (d[1]*x_t[1]+d[2]*x_t[2], d[1]*y_t[1]+d[2]*y_t[2], q[0])
body = None
for cc in design.allComponents:
    for i in range(cc.bRepBodies.count):
        b = cc.bRepBodies.item(i)
        if b.name == 'g_link_67': body = b
for i in range(body.faces.count):
    f = body.faces.item(i)
    if f.geometry.objectType != adsk.core.Plane.classType(): continue
    if abs(armf.swz(f.pointOnFace)[0] - route['s_near']) > 5e-3: continue
    log('--- face %d, %d edges, area %.1f mm2' % (i, f.edges.count, f.area*100.0))
    for k in range(f.edges.count):
        e = f.edges.item(k)
        ev = e.evaluator; ok0,p0,p1 = ev.getParameterExtents(); ok,pts = ev.getStrokes(p0,p1,0.0005)
        a, b2 = tong(pts[0]), tong(pts[-1])
        log('    %-9s (%8.3f,%8.3f) -> (%8.3f,%8.3f)  len %.2f mm'
            % (e.geometry.objectType.split('::')[-1], a[0], a[1], b2[0], b2[1], e.length*10.0))
log('computed outline, tongue coords:')
for sg in route['flange']:
    e = sg[3] if sg[0] == 'arc' else sg[2]
    log('    %-4s (%8.3f,%8.3f) -> (%8.3f,%8.3f)' % (sg[0], sg[1][0], sg[1][1], e[0], e[1]))
log('done')
