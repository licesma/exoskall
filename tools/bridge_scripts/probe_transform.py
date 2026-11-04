"""Decisive check: which call actually moves a nested occurrence in this design?

Creates root/generated_R_arm_probe/{W_nested, W_root}, tries to set a non-identity
matrix each way, reads back transform / transform2, then deletes the probe.
Nothing outside generated_R_arm_probe is touched.
"""
root = design.rootComponent
log('design type: %s' % ('parametric' if design.designType ==
    adsk.fusion.DesignTypes.ParametricDesignType else 'direct'))

# clean any leftover probe
for i in range(root.occurrences.count - 1, -1, -1):
    o = root.occurrences.item(i)
    if o.component.name.startswith('generated_R_arm'):
        log('deleting leftover %s' % o.component.name)
        o.deleteMe()

ident = adsk.core.Matrix3D.create()


def mat(tx, ty, tz):
    m = adsk.core.Matrix3D.create()
    m.setWithArray([0, -1, 0, tx,  1, 0, 0, ty,  0, 0, 1, tz,  0, 0, 0, 1])
    return m


def show(tag, occ):
    t = occ.transform.translation
    line = '%s: transform t=(%.3f, %.3f, %.3f)' % (tag, t.x, t.y, t.z)
    try:
        t2 = occ.transform2.translation
        line += ' | transform2 t=(%.3f, %.3f, %.3f)' % (t2.x, t2.y, t2.z)
    except Exception as e:
        line += ' | transform2 READ FAILED: %s' % e
    try:
        line += ' | assemblyContext=%s' % (
            occ.assemblyContext.name if occ.assemblyContext else 'None(root)')
    except Exception as e:
        line += ' | assemblyContext err %s' % e
    log(line)


arm_occ = root.occurrences.addNewComponent(ident)
arm_occ.component.name = 'generated_R_arm_probe'
arm = arm_occ.component
log('arm_occ.assemblyContext = %s' % (arm_occ.assemblyContext.name if arm_occ.assemblyContext else 'None(root)'))

# --- case A: nested occurrence created with a non-identity matrix ---
m = mat(1.0, 2.0, 3.0)
wA = arm.occurrences.addNewComponent(m)
wA.component.name = 'probe_W_created_with_matrix'
show('A after addNewComponent(m)', wA)

# --- case B: same, then try transform2 = m ---
wB = arm.occurrences.addNewComponent(ident)
wB.component.name = 'probe_W_transform2'
try:
    wB.transform2 = mat(4.0, 5.0, 6.0)
    log('B: transform2 assignment OK')
except Exception as e:
    log('B: transform2 assignment FAILED: %s' % e)
show('B', wB)

# --- case C: same, then try transform = m (deprecated setter) ---
wC = arm.occurrences.addNewComponent(ident)
wC.component.name = 'probe_W_transform'
try:
    wC.transform = mat(7.0, 8.0, 9.0)
    log('C: transform assignment OK')
except Exception as e:
    log('C: transform assignment FAILED: %s' % e)
show('C', wC)

# --- case D: proxy in root context via createForAssemblyContext(arm_occ) ---
wD = arm.occurrences.addNewComponent(ident)
wD.component.name = 'probe_W_proxy'
try:
    pD = wD.createForAssemblyContext(arm_occ)
    log('D: proxy made; isValid=%s ctx=%s' % (pD.isValid,
        pD.assemblyContext.name if pD.assemblyContext else 'None'))
    try:
        pD.transform2 = mat(10.0, 11.0, 12.0)
        log('D: proxy.transform2 assignment OK')
    except Exception as e:
        log('D: proxy.transform2 assignment FAILED: %s' % e)
    show('D proxy', pD)
except Exception as e:
    log('D: createForAssemblyContext FAILED: %s' % e)
show('D native', wD)

# --- what the root-context list says about each ---
for occ in arm_occ.childOccurrences:
    t = occ.transform.translation
    log('root-context child %s: transform t=(%.3f, %.3f, %.3f) ctx=%s' % (
        occ.component.name, t.x, t.y, t.z,
        occ.assemblyContext.name if occ.assemblyContext else 'None'))

# --- where is the geometry really? bounding box of a body we add to each ---
tbm = adsk.fusion.TemporaryBRepManager.get()
for occ in arm_occ.childOccurrences:
    b = tbm.createBox(adsk.core.OrientedBoundingBox3D.create(
        adsk.core.Point3D.create(0, 0, 0),
        adsk.core.Vector3D.create(1, 0, 0),
        adsk.core.Vector3D.create(0, 1, 0), 0.5, 0.5, 0.5))
    occ.component.bRepBodies.add(b)
for occ in arm_occ.childOccurrences:
    bb = occ.boundingBox
    if bb:
        log('WORLD bbox of %s: min=(%.3f, %.3f, %.3f) max=(%.3f, %.3f, %.3f)' % (
            occ.component.name, bb.minPoint.x, bb.minPoint.y, bb.minPoint.z,
            bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z))
    else:
        log('WORLD bbox of %s: none' % occ.component.name)

log('KEEP_PROBE = %s' % globals().get('KEEP_PROBE', False))
if not globals().get('KEEP_PROBE', False):
    arm_occ.deleteMe()
    log('probe deleted')
