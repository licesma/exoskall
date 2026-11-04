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
RA,tA=frames['J2']; RB,tB=frames['J3']
R_ab,t_ab=g.relative_frame(RA,tA,RB,tB)
route=sweepgen.sweep_route(R_ab,t_ab,27.0,25.0)
occ=root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); c=occ.component; c.name='DBG_B'
arm=BuildArmGen.Arm(RA,[v*1000.0 for v in tA],route['u'],route['w'])
FA=arm.F; ex,ey=FA.vector([1,0,0]),FA.vector([0,1,0])
JOIN=adsk.fusion.FeatureOperations.JoinFeatureOperation
NEW=adsk.fusion.FeatureOperations.NewBodyFeatureOperation
body=BuildArmGen.build_ribbon(c,arm,route,'dbg')
sk=BuildArm.PlaneSketch(c,'pad',FA.point([0,0,4.0]),ex,ey); sk.circle((0,0),27.0); sk.extrude(1,8.0,JOIN,body)
log('after pad bodies=%d'%c.bRepBodies.count)
body=c.bRepBodies.item(0)
# build the block as a NEW body first and measure the overlap
sk=BuildArm.PlaneSketch(c,'blk',FA.point([0,0,(1.0-3.6)/2]),ex,ey)
sk.loop(g.stop_block_outline(route['stop_angle'],route['stop_half'],20.0,27.0))
f=sk.extrude(1,4.6,NEW)
blk=f.bodies.item(0)
def bb(b):
    p,q=b.boundingBox.minPoint,b.boundingBox.maxPoint
    return '(%.2f,%.2f,%.2f)-(%.2f,%.2f,%.2f)'%(p.x*10,p.y*10,p.z*10,q.x*10,q.y*10,q.z*10)
log('main v=%.0f bb=%s'%(body.volume*1000,bb(body)))
log('blk  v=%.0f bb=%s'%(blk.volume*1000,bb(blk)))
coll=adsk.core.ObjectCollection.create(); coll.add(body); coll.add(blk)
inp=design.createInterferenceInput(coll); inp.areCoincidentFacesIncluded=False
res=design.analyzeInterference(inp)
log('interference count=%s vol=%s'%(0 if res is None else res.count, 'n/a' if not res or not res.count else '%.3f'%(res.item(0).interferenceBody.volume*1000)))
# try combine
cf=c.features.combineFeatures
tools=adsk.core.ObjectCollection.create(); tools.add(blk)
ci=cf.createInput(body, tools); ci.operation=JOIN; ci.isKeepToolBodies=False; ci.isNewComponent=False
cf.add(ci)
log('after combine bodies=%d v=%.0f'%(c.bRepBodies.count, c.bRepBodies.item(0).volume*1000))
