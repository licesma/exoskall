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
body = None
for cc in design.allComponents:
    for i in range(cc.bRepBodies.count):
        b = cc.bRepBodies.item(i)
        if b.name == 'g_link_67': body = b
for i in range(body.faces.count):
    f = body.faces.item(i)
    if f.geometry.objectType != adsk.core.Plane.classType(): continue
    if abs(armf.swz(f.pointOnFace)[0] - route['s_near']) > 5e-3: continue
    for k in range(f.edges.count):
        e = f.edges.item(k)
        if abs(e.length*10.0 - 29.51) > 0.2: continue
        log('the 29.51 mm edge, faces:')
        for m in range(e.faces.count):
            g2 = e.faces.item(m)
            geo = g2.geometry
            kind = geo.objectType.split('::')[-1]
            p = armf.swz(g2.pointOnFace)
            extra = ''
            if kind == 'Plane':
                n = geo.normal
                nl = armf.swz(adsk.core.Point3D.create(
                    armf.F.t[0]/10.0 + n.x, armf.F.t[1]/10.0 + n.y, armf.F.t[2]/10.0 + n.z))
                extra = ' normal(swz) %s' % [round(v, 3) for v in nl]
            elif kind == 'Cylinder':
                extra = ' radius %.3f mm' % (geo.radius * 10.0)
            log('   face %d %s area %.1f mm2 at swz %s%s'
                % (g2.tempId if hasattr(g2, 'tempId') else m, kind, g2.area*100.0,
                   [round(v, 2) for v in p], extra))
log('done')
