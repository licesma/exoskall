import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import adsk.core, adsk.fusion
import armgen, BuildArm
for m in (armgen, BuildArm): importlib.reload(m)
g=armgen
root=design.rootComponent
comp=None
for c in design.allComponents:
    if c.name=='g_link_12': comp=c
b=comp.bRepBodies.item(0)
joints,by_child=g.parse_urdf(BuildArm.URDF_PATH)
shift,_=g.joint_world_pose(BuildArm.ORIGIN_AT_JOINT,joints,by_child)
J1=BuildArm.JOINTS[0]
R,t=g.module_frame(J1['name'],joints,by_child,J1['axis_sign'],J1['y_hint'])
t=[(t[i]-shift[i])*1000 for i in range(3)]
Rt=g.transpose(R)
def tolocal(p):
    return g.matvec(Rt,[p[i]-t[i] for i in range(3)])
zs=[]
for k in range(b.vertices.count):
    v=b.vertices.item(k).geometry
    p=tolocal([v.x*10,v.y*10,v.z*10]); zs.append(p)
zs.sort(key=lambda p:p[2])
log('vertices=%d, lowest 12 in J1 frame (x,y,z,r,ang):'%len(zs))
for p in zs[:12]:
    log('   %8.2f %8.2f %8.2f  r=%6.2f ang=%6.1f'%(p[0],p[1],p[2],math.hypot(p[0],p[1]),math.degrees(math.atan2(p[1],p[0]))%360))
log('faces=%d'%b.faces.count)
