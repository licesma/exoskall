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
seed, _c, SLIDES, _r = BuildArmGen.solve_axis_signs(joints, by_child, shift, LINKS, table, log, ORDER)
frames, spins = sweepgen.solve_spins(seed, LINKS)
RA, tA = frames['J6']; RB, tB = frames['J7']
R_ab, t_ab = armgen.relative_frame(RA, tA, RB, tB)
route = sweepgen.sweep_route(R_ab, t_ab, 27.0, 25.0)
armf = BuildArmGen.Arm(RA, [v*1000.0 for v in tA], route['u'], route['w'])
bodies, _n = BuildArm.collect_bodies(arm)
link = None; j7 = []
for i in range(bodies.count):
    b = bodies.item(i)
    if b.name == 'g_link_67': link = b
    ctx = b.assemblyContext.fullPathName if b.assemblyContext else ''
    if '+GJ7_' in ctx and (b.assemblyContext.component.name == 'Component1'): j7.append(b)
for b in j7:
    coll = adsk.core.ObjectCollection.create(); coll.add(link); coll.add(b)
    inp = design.createInterferenceInput(coll); inp.areCoincidentFacesIncluded = False
    res = design.analyzeInterference(inp)
    if res is None: continue
    for i in range(res.count):
        ib = res.item(i).interferenceBody
        if ib is None: continue
        pts = [armf.swz(ib.vertices.item(k).geometry) for k in range(ib.vertices.count)]
        log('interference vol %.3f mm3, %d vertices' % (ib.volume*1000.0, len(pts)))
        log('  s %.3f..%.3f   w %.3f..%.3f   z %.3f..%.3f' % (
            min(p[0] for p in pts), max(p[0] for p in pts),
            min(p[1] for p in pts), max(p[1] for p in pts),
            min(p[2] for p in pts), max(p[2] for p in pts)))
        for p in sorted(pts)[:10]:
            log('   (s %8.3f w %8.3f z %8.3f)' % p)
log('done')
