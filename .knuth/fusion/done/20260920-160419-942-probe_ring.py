import sys, importlib, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import BuildArm, armgen, sweepgen, BuildArmGen, hole_ring_probe
for m in (armgen, sweepgen, BuildArm, BuildArmGen, hole_ring_probe): importlib.reload(m)
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
seed, _c, SL, _r = BuildArmGen.solve_axis_signs(joints, by_child, shift, LINKS, table, log, ORDER)
frames, spins = sweepgen.solve_spins(seed, LINKS)
RA, tA = frames['J6']; RB, tB = frames['J7']
R_ab, t_ab = armgen.relative_frame(RA, tA, RB, tB)
route = sweepgen.sweep_route(R_ab, t_ab, 27.0, 25.0)
armf = BuildArmGen.Arm(RA, [v*1000.0 for v in tA], route['u'], route['w'])
body = None
for cc in design.allComponents:
    for i in range(cc.bRepBodies.count):
        b = cc.bRepBodies.item(i)
        if b.name == 'g_link_67': body = b
frame = (adsk.core.Point3D.create(armf.F.t[0]/10.0, armf.F.t[1]/10.0, armf.F.t[2]/10.0),
         adsk.core.Vector3D.create(*armf.dir([1,0,0])),
         adsk.core.Vector3D.create(*armf.dir([0,1,0])),
         adsk.core.Vector3D.create(*armf.dir([0,0,1])))
rings = hole_ring_probe.measure(body, frame, armgen.HOLE_R)
log('openings: %d' % len(rings))
for h in sorted(rings, key=lambda h: h['ring'])[:6]:
    log('  %-3s %8.3f ring %8.3f bound %8.3f at (s %7.2f w %7.2f z %7.2f)'
        % (h['plane'][0], h['plane'][1], h['ring'], h['bound'], h['at'][0], h['at'][1], h['at'][2]))
    for k in ('bound_at', 'edge', 'why'):
        if k in h: log('        %s=%s' % (k, h[k]))
log('keys: %s' % sorted(rings[0].keys()))
log('done')
