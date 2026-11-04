import sys, importlib, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen; importlib.reload(armgen)

root = design.rootComponent
arm = next(o for o in root.occurrences if o.component.name.startswith('generated_R_arm'))
l45 = next(s for s in arm.childOccurrences if s.component.name.startswith('link_45'))
j4  = next(s for s in arm.childOccurrences if s.component.name.startswith('J4_'))
body = l45.bRepBodies.item(0)
T = j4.transform
import adsk.core as ac
GEO = None
# link frame axes in world
_j,_by = armgen.parse_urdf('/Users/esteban/repos/exo/g1/g1_29dof.urdf')
_sh,_ = armgen.joint_world_pose('left_shoulder_pitch_joint', _j, _by)
FR={}
for lab,(nm,sg,yh) in {'J4':('right_elbow_joint',-1,(-1,0,0)),'J5':('right_wrist_roll_joint',1,(0,1,0))}.items():
    R,t = armgen.module_frame(nm,_j,_by,sg,yh); FR[lab]=(R,[t[k]-_sh[k] for k in range(3)])
GEO = armgen.route(FR['J4'],FR['J5'],27.0,width=44.0)['geometry']
U,W = GEO['u'],GEO['w']
uw = ac.Vector3D.create(U[0],U[1],0.0); uw.transformBy(T)
ww = ac.Vector3D.create(W[0],W[1],0.0); ww.transformBy(T)
zw = ac.Vector3D.create(0.0,0.0,1.0); zw.transformBy(T)
o  = T.translation
def loc(p):
    d = ac.Vector3D.create(p.x-o.x, p.y-o.y, p.z-o.z)
    return (10*d.dotProduct(uw), 10*d.dotProduct(ww), 10*d.dotProduct(zw))
def locd(v):
    return (v.dotProduct(uw), v.dotProduct(ww), v.dotProduct(zw))

hr = armgen.HOLE_R
seen=[]
for f in body.faces:
    g=f.geometry
    if g.objectType!='adsk::core::Cylinder' or abs(g.radius*10-hr)>0.05: continue
    k=(round(g.origin.x,3),round(g.origin.y,3),round(g.origin.z,3))
    if k in seen: continue
    seen.append(k)
    c = loc(g.origin); ax = locd(g.axis)
    bb=f.boundingBox; p0,p1 = loc(bb.minPoint), loc(bb.maxPoint)
    log('')
    log('hole axis through (s %.3f w %.3f z %.3f) dir (%.3f %.3f %.3f)  extent s[%.2f,%.2f] w[%.2f,%.2f] z[%.2f,%.2f]'
        %(c[0],c[1],c[2],ax[0],ax[1],ax[2],p0[0],p1[0],p0[1],p1[1],p0[2],p1[2]))
    done=[]
    for e in f.edges:
        for nf in e.faces:
            if nf==f: continue
            if nf in done: continue
            done.append(nf)
            gg=nf.geometry; t=gg.objectType.split('::')[-1]
            bb=nf.boundingBox; q0,q1 = loc(bb.minPoint), loc(bb.maxPoint)
            extra=''
            if t=='Cylinder':
                cc=loc(gg.origin); aa=locd(gg.axis)
                extra=' r=%.3f axis(%.2f,%.2f,%.2f) through (s %.3f w %.3f z %.3f)'%(gg.radius*10,aa[0],aa[1],aa[2],cc[0],cc[1],cc[2])
            elif t=='Plane':
                nn=locd(gg.normal); pp=loc(gg.origin)
                extra=' n(%.3f,%.3f,%.3f) at (s %.3f w %.3f z %.3f)'%(nn[0],nn[1],nn[2],pp[0],pp[1],pp[2])
            log('   %-9s area %8.2f  bbox s[%.2f,%.2f] w[%.2f,%.2f] z[%.2f,%.2f]%s'
                %(t,nf.area*100,q0[0],q1[0],q0[1],q1[1],q0[2],q1[2],extra))
