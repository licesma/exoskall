"""EXO-341 probe (read-only): the general arm's structure, link_12 and the J2 module frame,
the parked module's stator, and point probes at +Y of the J2 frame near the stator underside."""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
doc = app.activeDocument
assert doc.name == BuildArm.GEN_DOC_NAME, doc.name
design = adsk.fusion.Design.cast(app.activeProduct)
say('designType %s isModified %s v%s' % (design.designType, doc.isModified, doc.dataFile.versionNumber))
root = design.rootComponent
def mat(m):
    a = m.asArray(); return [[a[4*i+j] for j in range(4)] for i in range(4)]
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    say('root occ %r comp %r' % (o.name, o.component.name))
arm = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name == 'generated_R_arm_general'][0]
for i in range(arm.childOccurrences.count):
    o = arm.childOccurrences.item(i)
    M = mat(o.transform)
    say('  arm child %r comp %r bodies %d  origin(mm) %s  Z %s' % (o.name, o.component.name, o.component.bRepBodies.count,
        ['%.3f' % (M[r][3]*10) for r in range(3)], ['%.4f' % M[r][2] for r in range(3)]))
    for b in range(o.component.bRepBodies.count):
        bb = o.component.bRepBodies.item(b)
        say('     body %r V %.3f' % (bb.name, bb.volume*1000))
