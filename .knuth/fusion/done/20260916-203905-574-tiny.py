import sys, importlib, math
sys.path.insert(0,'/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen; importlib.reload(armgen)
root = design.rootComponent
arm = next(o for o in root.occurrences if o.component.name.startswith('generated_R_arm'))
l45 = next(s for s in arm.childOccurrences if s.component.name.startswith('link_45'))
j4  = next(s for s in arm.childOccurrences if s.component.name.startswith('J4_'))
body = l45.bRepBodies.item(0)
import adsk.core as ac
_j,_by = armgen.parse_urdf('/Users/esteban/repos/exo/g1/g1_29dof.urdf')
_sh,_ = armgen.joint_world_pose('left_shoulder_pitch_joint', _j, _by)
FR={}
for lab,(nm,sg,yh) in {'J4':('right_elbow_joint',-1,(-1,0,0)),'J5':('right_wrist_roll_joint',1,(0,1,0))}.items():
    R,t = armgen.module_frame(nm,_j,_by,sg,yh); FR[lab]=(R,[t[k]-_sh[k] for k in range(3)])
GEO = armgen.route(FR['J4'],FR['J5'],27.0,width=44.0)['geometry']
U,W = GEO['u'],GEO['w']; T=j4.transform
uw=ac.Vector3D.create(U[0],U[1],0.0); uw.transformBy(T)
ww=ac.Vector3D.create(W[0],W[1],0.0); ww.transformBy(T)
zw=ac.Vector3D.create(0.0,0.0,1.0); zw.transformBy(T)
o=T.translation
def loc(p):
    d=ac.Vector3D.create(p.x-o.x,p.y-o.y,p.z-o.z)
    return (10*d.dotProduct(uw),10*d.dotProduct(ww),10*d.dotProduct(zw))
for f in body.faces:
    a=f.area*100
    if a<2.0:
        g=f.geometry; bb=f.boundingBox; p0,p1=loc(bb.minPoint),loc(bb.maxPoint)
        extra=''
        if g.objectType.endswith('Cylinder'): extra=' r=%.3f'%(g.radius*10)
        log('area %8.4f  %-10s s[%.3f,%.3f] w[%.3f,%.3f] z[%.3f,%.3f]%s'
            %(a,g.objectType.split('::')[-1],p0[0],p1[0],p0[1],p1[1],p0[2],p1[2],extra))
