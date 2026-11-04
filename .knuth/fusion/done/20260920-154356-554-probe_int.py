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
t_w, ax = armgen.joint_world_pose('right_wrist_yaw_joint', joints, by_child)
o7 = [(t_w[i]-shift[i])*1000.0 for i in range(3)]
log('J7 origin (arm mm) %s  axis %s' % ([round(v,3) for v in o7], [round(v,4) for v in ax]))
bodies, _n = BuildArm.collect_bodies(arm)
link = None; j7 = []
for i in range(bodies.count):
    b = bodies.item(i)
    if b.name == 'g_link_67': link = b
    ctx = b.assemblyContext.fullPathName if b.assemblyContext else ''
    if '+GJ7_' in ctx: j7.append(b)
log('J7 bodies: %d' % len(j7))
for b in j7:
    coll = adsk.core.ObjectCollection.create()
    coll.add(link); coll.add(b)
    inp = design.createInterferenceInput(coll)
    inp.areCoincidentFacesIncluded = False
    res = design.analyzeInterference(inp)
    if res is None: continue
    for i in range(res.count):
        ib = res.item(i).interferenceBody
        if ib is None: continue
        bb = ib.boundingBox
        # deepest point of the interference body, in J7 frame
        worst = None
        for k in range(ib.vertices.count):
            p = ib.vertices.item(k).geometry
            d = [p.x*10-o7[0], p.y*10-o7[1], p.z*10-o7[2]]
            al = armgen.dot(d, ax)
            r = math.sqrt(max(0.0, armgen.dot(d,d)-al*al))
            if worst is None or r < worst[0]: worst = (r, al)
        log('HIT with %s (comp %s): vol %.2f mm3, deepest radius %.3f at along %.3f'
            % (b.name, b.assemblyContext.component.name if b.assemblyContext else '?',
               ib.volume*1000.0, worst[0] if worst else -1, worst[1] if worst else -1))
        rs = []
        for k in range(ib.vertices.count):
            p = ib.vertices.item(k).geometry
            d = [p.x*10-o7[0], p.y*10-o7[1], p.z*10-o7[2]]
            al = armgen.dot(d, ax)
            rs.append((round(math.sqrt(max(0.0, armgen.dot(d,d)-al*al)),3), round(al,3)))
        log('    its vertices (radius, along): %s' % sorted(rs)[:10])
log('done')
