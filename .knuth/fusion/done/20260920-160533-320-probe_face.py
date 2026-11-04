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
# the insert axis in world: rebuild from the true stator frame
FA = BuildArm.Frame(RA, [v*1000.0 for v in tA])
FB = FA.child(R_ab, t_ab)
z_mid = armgen.STATOR_UNDERSIDE_Z - armgen.PAD_T/2.0
tgt = FB.point([armgen.INSERT_RADIUS*math.cos(math.pi), armgen.INSERT_RADIUS*math.sin(math.pi), z_mid])
ax = FB.vector([0,0,1])
log('insert axis point swz %s' % [round(v,3) for v in armf.swz(tgt)])
def axdist(p):
    d=(p.x*10-tgt.x*10, p.y*10-tgt.y*10, p.z*10-tgt.z*10)
    k=d[0]*ax.x+d[1]*ax.y+d[2]*ax.z
    return math.sqrt(max(0.0,d[0]**2+d[1]**2+d[2]**2-k*k))
# find the planar face at s = 24.5 that contains this hole
for i in range(body.faces.count):
    f = body.faces.item(i)
    if f.geometry.objectType != adsk.core.Plane.classType(): continue
    p = f.pointOnFace
    swz = armf.swz(p)
    if abs(swz[0]-route['s_near']) > 5e-3: continue
    # is our insert on this face?
    best=None
    for k in range(f.edges.count):
        e=f.edges.item(k)
        ev=e.evaluator; okx,p0,p1=ev.getParameterExtents(); ok,pts=ev.getStrokes(p0,p1,0.0005)
        dmin=min(axdist(q) for q in pts)
        dmax=max(axdist(q) for q in pts)
        if best is None or dmin<best[0]: best=(dmin,dmax,k,e.geometry.objectType.split('::')[-1],
            [round(v,2) for v in armf.swz(pts[len(pts)//2])])
    log('face %d at s %.3f, %d edges, nearest edge to the insert axis: %.3f..%.3f  %s at swz %s'
        % (i, swz[0], f.edges.count, best[0], best[1], best[3], best[4]))
    if best[0] < 12.0:
        for k in range(f.edges.count):
            e=f.edges.item(k)
            ev=e.evaluator; okx,p0,p1=ev.getParameterExtents(); ok,pts=ev.getStrokes(p0,p1,0.0005)
            dmin=min(axdist(q) for q in pts)
            if dmin < 12.0:
                log('    edge %d %s dmin %.3f at swz %s .. %s' % (k,
                    e.geometry.objectType.split('::')[-1], dmin,
                    [round(v,2) for v in armf.swz(pts[0])],
                    [round(v,2) for v in armf.swz(pts[-1])]))
log('done')
