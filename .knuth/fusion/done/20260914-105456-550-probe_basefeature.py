"""Probe 2: does a body/base feature reset nested occurrence transforms in this
(direct) design, and do world positions really follow `transform`?

Builds root/generated_R_arm_probe2 with two wrappers created via
addNewComponent(m) at different poses, each holding a 1 cm cube, then adds a body
to a third component (both ways: plain and via baseFeatures) and re-reads every
wrapper transform + the WORLD bbox of its cube. Deletes the probe at the end.
"""
root = design.rootComponent
parametric = design.designType == adsk.fusion.DesignTypes.ParametricDesignType
log('design type: %s' % ('parametric' if parametric else 'direct'))
for i in range(root.occurrences.count - 1, -1, -1):
    o = root.occurrences.item(i)
    if o.component.name.startswith('generated_R_arm'):
        log('deleting leftover %s' % o.component.name)
        o.deleteMe()

ident = adsk.core.Matrix3D.create()
tbm = adsk.fusion.TemporaryBRepManager.get()


def cube():
    return tbm.createBox(adsk.core.OrientedBoundingBox3D.create(
        adsk.core.Point3D.create(0, 0, 0), adsk.core.Vector3D.create(1, 0, 0),
        adsk.core.Vector3D.create(0, 1, 0), 1.0, 1.0, 1.0))


def mat(tx, ty, tz):
    m = adsk.core.Matrix3D.create()
    m.setWithArray([1, 0, 0, tx, 0, 1, 0, ty, 0, 0, 1, tz, 0, 0, 0, 1])
    return m


def state(tag, arm_occ):
    for occ in arm_occ.childOccurrences:
        t = occ.transform.translation
        bbs = 'no body'
        if occ.bRepBodies.count:
            bb = occ.bRepBodies.item(0).boundingBox
            bbs = 'world body centre=(%.3f, %.3f, %.3f)' % (
                (bb.minPoint.x + bb.maxPoint.x) / 2,
                (bb.minPoint.y + bb.maxPoint.y) / 2,
                (bb.minPoint.z + bb.maxPoint.z) / 2)
        log('%s  %-12s transform t=(%.3f, %.3f, %.3f)  %s' % (
            tag, occ.component.name, t.x, t.y, t.z, bbs))


arm_occ = root.occurrences.addNewComponent(ident)
arm_occ.component.name = 'generated_R_arm_probe2'
arm = arm_occ.component

wraps = []
for name, m in (('wA', mat(5, 0, 0)), ('wB', mat(0, 7, 0))):
    w = arm.occurrences.addNewComponent(m)
    w.component.name = name
    w.component.bRepBodies.add(cube())
    wraps.append(w)
state('after create  ', arm_occ)

# plain body add on a third component
c3 = arm.occurrences.addNewComponent(mat(0, 0, 9))
c3.component.name = 'wC_plain'
c3.component.bRepBodies.add(cube())
state('after plain   ', arm_occ)

# base feature add (the round-1 path)
c4 = arm.occurrences.addNewComponent(mat(0, 0, -9))
c4.component.name = 'wD_basefeat'
try:
    bf = c4.component.features.baseFeatures.add()
    bf.startEdit()
    try:
        c4.component.bRepBodies.add(cube(), bf)
    finally:
        bf.finishEdit()
    log('baseFeature path OK')
except Exception as e:
    log('baseFeature path FAILED: %s' % e)
state('after basefeat', arm_occ)

if parametric:
    log('hasPendingSnapshot=%s' % design.snapshots.hasPendingSnapshot)

arm_occ.deleteMe()
log('probe2 deleted')
