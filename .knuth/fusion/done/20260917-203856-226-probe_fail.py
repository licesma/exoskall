import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import adsk.core, adsk.fusion
import armgen, BuildArm
for m in (armgen, BuildArm): importlib.reload(m)
g=armgen
root=design.rootComponent
arm=None
for i in range(root.occurrences.count):
    o=root.occurrences.item(i)
    if o.component.name=='generated_R_arm_general': arm=o
bodies,names=BuildArm.collect_bodies(arm)
inp=design.createInterferenceInput(bodies); inp.areCoincidentFacesIncluded=False
res=design.analyzeInterference(inp)
joints,by_child=g.parse_urdf(BuildArm.URDF_PATH)
shift,_=g.joint_world_pose(BuildArm.ORIGIN_AT_JOINT,joints,by_child)
J1=BuildArm.JOINTS[0]
R,t=g.module_frame(J1['name'],joints,by_child,J1['axis_sign'],J1['y_hint'])
t=[(t[i]-shift[i])*1000 for i in range(3)]
Rt=g.transpose(R)
def tolocal(p):
    d=[p[i]-t[i] for i in range(3)]
    return g.matvec(Rt,d)
for i in range(res.count):
    r=res.item(i)
    a,b=BuildArm.body_label(r.entityOne),BuildArm.body_label(r.entityTwo)
    if 'g_link' not in a+b: continue
    ib=r.interferenceBody
    if ib is None or ib.volume*1000 < 1.0: continue
    log('%s <-> %s vol=%.2f' % (a,b,ib.volume*1000))
    pts=[]
    for k in range(ib.vertices.count):
        v=ib.vertices.item(k).geometry
        p=tolocal([v.x*10,v.y*10,v.z*10]); pts.append(p)
    if pts:
        log('   J1-frame verts: x %.2f..%.2f  y %.2f..%.2f  z %.2f..%.2f  r %.2f..%.2f  ang %.1f..%.1f'%(
            min(p[0] for p in pts),max(p[0] for p in pts),min(p[1] for p in pts),max(p[1] for p in pts),
            min(p[2] for p in pts),max(p[2] for p in pts),
            min(math.hypot(p[0],p[1]) for p in pts),max(math.hypot(p[0],p[1]) for p in pts),
            min(math.degrees(math.atan2(p[1],p[0]))%360 for p in pts),max(math.degrees(math.atan2(p[1],p[0]))%360 for p in pts)))
