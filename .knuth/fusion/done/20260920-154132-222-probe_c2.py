import adsk.core, adsk.fusion
root = design.rootComponent
TARGET = (174.86, 180.49, -267.76, -259.26, -221.05, -200.54)
arm = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name.startswith('generated_R_arm_general'):
        arm = o
log('arm: %s' % (arm.component.name if arm else None))

def walk(occ, path, depth=0):
    c = occ.component
    for i in range(c.bRepBodies.count):
        b = c.bRepBodies.item(i)
        pb = b.createForAssemblyContext(occ) if occ.assemblyContext is not None or True else b
        try:
            bb = pb.boundingBox
        except Exception:
            bb = b.boundingBox
        v = (bb.minPoint.x*10, bb.maxPoint.x*10, bb.minPoint.y*10, bb.maxPoint.y*10,
             bb.minPoint.z*10, bb.maxPoint.z*10)
        if all(abs(v[k]-TARGET[k]) < 8.0 for k in range(6)) or (
            v[0] < TARGET[1]+2 and v[1] > TARGET[0]-2 and
            v[2] < TARGET[3]+2 and v[3] > TARGET[2]-2 and
            v[4] < TARGET[5]+2 and v[5] > TARGET[4]-2):
            log('HIT %s / %s  vol %.1f  bbox x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]'
                % (path, b.name, pb.volume*1000.0, v[0], v[1], v[2], v[3], v[4], v[5]))
    if depth < 4:
        for i in range(c.occurrences.count):
            sub = c.occurrences.item(i)
            p = sub.createForAssemblyContext(occ) if occ.assemblyContext is None and sub.assemblyContext is None else sub
            walk(sub if p is None else p, path + '/' + sub.component.name, depth+1)

walk(arm, arm.component.name)
log('done')
