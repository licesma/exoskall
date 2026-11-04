import math
OUT='/Users/esteban/repos/exo/gento_luna/exo/screenshots/'
vp=app.activeViewport; root=design.rootComponent
saved=[]
for i in range(root.occurrences.count):
    o=root.occurrences.item(i); saved.append((o,o.isLightBulbOn)); o.isLightBulbOn = o.component.name=='right arm'
arm=[root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name=='right arm'][0]
kids=[(arm.childOccurrences.item(j), arm.childOccurrences.item(j).isLightBulbOn) for j in range(arm.childOccurrences.count)]
W=('Link 5 R','Link 6 R','Module 4 R','Module 5 R','Module 6 R')
for c,_ in kids: c.isLightBulbOn = c.component.name in W
tgt=(164.023,-235.0,-196.5)
for ext in (40.0,90.0,160.0):
    d=(0.55,-0.75,0.38); n=math.sqrt(sum(v*v for v in d))
    t=adsk.core.Point3D.create(tgt[0]/10.,tgt[1]/10.,tgt[2]/10.)
    cam=vp.camera; cam.isFitView=False; cam.target=t
    cam.eye=adsk.core.Point3D.create(t.x+80*d[0]/n,t.y+80*d[1]/n,t.z+80*d[2]/n)
    cam.upVector=adsk.core.Vector3D.create(0,0,1); cam.viewExtents=ext; cam.isSmoothTransition=False
    vp.camera=cam; adsk.doEvents(); vp.camera=cam; adsk.doEvents()
    vp.saveAsImageFile(OUT+'cal_%d.png'%int(ext),1600,1000)
    log('ext %s -> actual %s' % (ext, vp.camera.viewExtents))
for c,w in kids: c.isLightBulbOn=w
for o,w in saved: o.isLightBulbOn=w
