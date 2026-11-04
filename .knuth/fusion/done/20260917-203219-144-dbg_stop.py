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
RA,tA=frames['J2']
FA=BuildArm.Frame(RA,[v*1000.0 for v in tA])
ex,ey=FA.vector([1,0,0]),FA.vector([0,1,0])
occ=root.occurrences.addNewComponent(adsk.core.Matrix3D.create()); c=occ.component; c.name='DBG_STOP'
NEW=adsk.fusion.FeatureOperations.NewBodyFeatureOperation
sk=BuildArm.PlaneSketch(c,'disk',FA.point([0,0,4.0]),ex,ey); sk.circle((0,0),27.0); f1=sk.extrude(1,8.0,NEW)
sk=BuildArm.PlaneSketch(c,'blk',FA.point([0,0,(1.0-3.6)/2]),ex,ey); sk.loop(g.stop_block_outline(348.0,10.0,20.0,27.0)); f2=sk.extrude(1,4.6,NEW)
b1,b2=f1.bodies.item(0),f2.bodies.item(0)
def bb(b):
    p,q=b.boundingBox.minPoint,b.boundingBox.maxPoint
    return '(%.2f,%.2f,%.2f)-(%.2f,%.2f,%.2f)'%(p.x*10,p.y*10,p.z*10,q.x*10,q.y*10,q.z*10)
log('disk v=%.0f bb=%s'%(b1.volume*1000,bb(b1)))
log('blk  v=%.0f bb=%s'%(b2.volume*1000,bb(b2)))
coll=adsk.core.ObjectCollection.create(); coll.add(b1); coll.add(b2)
inp=design.createInterferenceInput(coll); inp.areCoincidentFacesIncluded=False
res=design.analyzeInterference(inp)
log('interference count=%s'%(0 if res is None else res.count))
if res and res.count: log('  vol=%.3f'%(res.item(0).interferenceBody.volume*1000))
r=app.measureManager.measureMinimumDistance(b1,b2)
log('min distance disk-block = %.6f mm'%(r.value*10))
