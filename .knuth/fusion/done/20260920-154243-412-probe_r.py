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
NAMES = {'J1':'right_shoulder_pitch_joint','J2':'right_shoulder_roll_joint',
         'J3':'right_shoulder_yaw_joint','J4':'right_elbow_joint',
         'J5':'right_wrist_roll_joint','J6':'right_wrist_pitch_joint',
         'J7':'right_wrist_yaw_joint'}
bodies, _n = BuildArm.collect_bodies(arm)
# group bodies by which GJ wrapper they are under
groups = {}
for i in range(bodies.count):
    b = bodies.item(i)
    ctx = b.assemblyContext.fullPathName if b.assemblyContext else ''
    lab = None
    for k in NAMES:
        if '+G%s_' % k in ctx: lab = k
    if lab is None: continue
    groups.setdefault(lab, []).append(b)

for lab in sorted(NAMES):
    t_w, ax = armgen.joint_world_pose(NAMES[lab], joints, by_child)
    o = [(t_w[i]-shift[i])*1000.0 for i in range(3)]
    best_r, best_lo, best_hi = 0.0, 1e9, -1e9
    n = 0
    for b in groups.get(lab, []):
        for k in range(b.vertices.count):
            p = b.vertices.item(k).geometry
            d = [p.x*10-o[0], p.y*10-o[1], p.z*10-o[2]]
            al = armgen.dot(d, ax)
            r = math.sqrt(max(0.0, armgen.dot(d,d)-al*al))
            best_r = max(best_r, r); best_lo=min(best_lo,al); best_hi=max(best_hi,al)
            n += 1
    log('%s: %d bodies, %d vertices; max radius from its own axis %.3f mm; along axis %.3f .. %.3f'
        % (lab, len(groups.get(lab,[])), n, best_r, best_lo, best_hi))
log('MODULE_PROFILE max r %.3f, z %.3f .. %.3f' % (max(b[2] for b in armgen.MODULE_PROFILE),
    armgen.MODULE_PROFILE[0][0], armgen.MODULE_PROFILE[-1][1]))
log('done')
