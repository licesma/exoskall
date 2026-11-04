import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm, armgen
importlib.reload(BuildArm); importlib.reload(armgen)
import adsk.core, adsk.fusion
root = design.rootComponent
arm = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name == 'generated_R_arm_general':
        arm = o
bodies, names = BuildArm.collect_bodies(arm)
log('bodies %d, names %s' % (bodies.count, type(names)))
for i in range(bodies.count):
    b = bodies.item(i)
    nm = b.assemblyContext.component.name if b.assemblyContext else b.parentComponent.name
    if nm == 'Component1':
        bb = b.boundingBox
        log('Component1/%s vol %.1f bbox x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]  ctx=%s'
            % (b.name, b.volume*1000.0, bb.minPoint.x*10, bb.maxPoint.x*10,
               bb.minPoint.y*10, bb.maxPoint.y*10, bb.minPoint.z*10, bb.maxPoint.z*10,
               b.assemblyContext.fullPathName if b.assemblyContext else '-'))
log('done')
