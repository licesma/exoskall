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
bodies, _n = BuildArm.collect_bodies(arm)
link = None
for i in range(bodies.count):
    b = bodies.item(i)
    if b.name == 'g_link_67': link = b
log('link %s, %d vertices' % (link.name, link.vertices.count))
rows = []
for k in range(link.vertices.count):
    p = link.vertices.item(k).geometry
    d = [p.x*10-o7[0], p.y*10-o7[1], p.z*10-o7[2]]
    al = armgen.dot(d, ax)
    r = math.sqrt(max(0.0, armgen.dot(d,d)-al*al))
    rows.append((r, al))
inb = [x for x in rows if -36.5 <= x[1] <= 0.0]
log('vertices inside the module band (-36.5..0 along J7 axis): %d' % len(inb))
inb.sort()
for r, al in inb[:12]:
    log('   radius %8.3f  along %8.3f' % (r, al))
log('smallest radius over the WHOLE link: %.3f (along %.3f)' % min(rows))
# and the module's own along-axis span, plus where its r 27 shell is
log('STATOR_UNDERSIDE_Z %.1f ; check band used by sweepgen: %.1f .. 0.0'
    % (armgen.STATOR_UNDERSIDE_Z, armgen.STATOR_UNDERSIDE_Z))
log('done')
