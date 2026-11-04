import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import adsk.core, adsk.fusion
import armgen, sweepgen, BuildArm, BuildArmGen
for m in (armgen, sweepgen, BuildArm, BuildArmGen): importlib.reload(m)
g=armgen
root=design.rootComponent
for i in range(root.occurrences.count-1,-1,-1):
    o=root.occurrences.item(i)
    if o.component.name.startswith('DBG_'): o.deleteMe()
joints,by_child=g.parse_urdf(BuildArm.URDF_PATH)
shift,_=g.joint_world_pose(BuildArm.ORIGIN_AT_JOINT,joints,by_child)
seed={}
for j in BuildArm.JOINTS:
    R,t=g.module_frame(j['name'],joints,by_child,j['axis_sign'],j['y_hint'])
    seed[j['label']]=(R,[t[i]-shift[i] for i in range(3)])
links=[(L['name'],L['a'],L['b'],L['pad_r'],L['width']) for L in BuildArm.LINKS]
frames,spins=sweepgen.solve_spins(seed,links)
name,a,b,pad_r,width='link_23','J2','J3',27.0,25.0
RA,tA=frames[a]; RB,tB=frames[b]
R_ab,t_ab=g.relative_frame(RA,tA,RB,tB)
route=sweepgen.sweep_route(R_ab,t_ab,pad_r,width)
occ=root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
c=occ.component; c.name='DBG_23'
arm=BuildArmGen.Arm(RA,[v*1000.0 for v in tA],route['u'],route['w'])
def show(tag):
    log('%-16s bodies=%d %s'%(tag,c.bRepBodies.count,
        ' | '.join('%s v=%.0f bb=(%.1f,%.1f,%.1f)-(%.1f,%.1f,%.1f)'%(c.bRepBodies.item(i).name,
          c.bRepBodies.item(i).volume*1000,
          c.bRepBodies.item(i).boundingBox.minPoint.x*10,c.bRepBodies.item(i).boundingBox.minPoint.y*10,c.bRepBodies.item(i).boundingBox.minPoint.z*10,
          c.bRepBodies.item(i).boundingBox.maxPoint.x*10,c.bRepBodies.item(i).boundingBox.maxPoint.y*10,c.bRepBodies.item(i).boundingBox.maxPoint.z*10)
          for i in range(c.bRepBodies.count))))
body=BuildArmGen.build_ribbon(c,arm,route,'dbg')
show('ribbon')
JOIN=adsk.fusion.FeatureOperations.JoinFeatureOperation
FA=arm.F; FS=FA.child(route['R_b'],t_ab)
ex,ey=FA.vector([1,0,0]),FA.vector([0,1,0]); sx,sy=FS.vector([1,0,0]),FS.vector([0,1,0])
sk=BuildArm.PlaneSketch(c,'pad',FA.point([0,0,g.PAD_T/2]),ex,ey); sk.circle((0,0),pad_r); sk.extrude(1,g.PAD_T,JOIN,body)
show('after pad')
z_mid=g.STATOR_UNDERSIDE_Z-g.PAD_T/2.0
sk=BuildArm.PlaneSketch(c,'plate',FS.point([0,0,z_mid]),sx,sy); sk.loop(route['flange']); sk.extrude(1,g.PAD_T,JOIN,body)
show('after plate')
# where does the route say the ribbon ends, in world?
p_end=route['path'][-1][2]
log('route end (s,w,z)=(%.2f,%.2f,%.2f) world=%s'%(p_end[0],p_end[1],p_end[2],['%.2f'%v for v in arm.xyz(p_end)]))
log('plate centre world=%s'%['%.2f'%v for v in FS.xyz([0,0,z_mid])])
