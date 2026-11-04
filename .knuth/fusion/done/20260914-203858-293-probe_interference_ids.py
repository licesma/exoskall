# How do bodies returned by analyzeInterference identify themselves vs the bodies we fed in?
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)
root = design.rootComponent
arm = [root.occurrences.item(i) for i in range(root.occurrences.count)
       if root.occurrences.item(i).component.name.startswith('generated_R_arm')][0]
bodies, names = BuildArm.collect_bodies(arm)
def ident(b):
    ac = b.assemblyContext
    return '%s | ctx=%s | parentComp=%s | native=%s' % (
        b.name, ac.fullPathName if ac else None, b.parentComponent.name,
        (b.nativeObject is not None))
fed = {}
for i in range(bodies.count):
    b = bodies.item(i)
    fed[b.entityToken] = b
inp = design.createInterferenceInput(bodies); inp.areCoincidentFacesIncluded = False
res = design.analyzeInterference(inp)
for i in range(res.count):
    r = res.item(i)
    e1, e2 = r.entityOne, r.entityTwo
    ib = r.interferenceBody
    bb = ib.boundingBox
    log('%d: vol=%.2f  token1_known=%s token2_known=%s' % (i, ib.volume*1000, e1.entityToken in fed, e2.entityToken in fed))
    log('    one: %s' % ident(e1))
    log('    two: %s' % ident(e2))
    log('    bbox world x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]' % (bb.minPoint.x*10, bb.maxPoint.x*10, bb.minPoint.y*10, bb.maxPoint.y*10, bb.minPoint.z*10, bb.maxPoint.z*10))
