import adsk.core, adsk.fusion
root=design.rootComponent
a=b=None
for c in design.allComponents:
    if c.name=='g_link_12' and c.bRepBodies.count: a=c.bRepBodies.item(0)
    if c.name=='link_12' and c.bRepBodies.count: b=c.bRepBodies.item(0)
log('a=%s b=%s'%(a,b))
mm=app.measureManager
for label, x, y in (('body,body', a, b),
                    ('vertex,body', a.vertices.item(0), b),
                    ('point,body', a.vertices.item(0).geometry, b),
                    ('face,face', a.faces.item(0), b.faces.item(0))):
    try:
        r=mm.measureMinimumDistance(x,y); log('%s -> %.4f mm'%(label, r.value*10))
    except Exception as e: log('%s -> ERR %s'%(label,e))
tbm=adsk.fusion.TemporaryBRepManager.get()
ca=tbm.copy(a); cb=tbm.copy(b)
ok=tbm.booleanOperation(ca, cb, adsk.fusion.BooleanTypes.IntersectionBooleanType)
log('intersect ok=%s vol=%.1f  a=%.1f b=%.1f'%(ok, ca.volume*1000, a.volume*1000, b.volume*1000))
