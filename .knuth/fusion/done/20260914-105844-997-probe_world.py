"""Where is the generated module geometry really? Compare several measurements
against a known-good existing occurrence. Read-only: creates and deletes nothing.
"""
root = design.rootComponent


def walk(occ, depth=0, path=''):
    name = occ.component.name
    ctx = occ.assemblyContext.component.name if occ.assemblyContext else 'None'
    log('%s%s (ctx=%s) bodies=%d children=%d' % ('  ' * depth, name, ctx,
                                                 occ.bRepBodies.count, occ.childOccurrences.count))
    for i in range(occ.bRepBodies.count):
        b = occ.bRepBodies.item(i)
        bb = b.boundingBox
        log('%s  body %s ctx=%s bbox min=(%.2f, %.2f, %.2f) max=(%.2f, %.2f, %.2f)' % (
            '  ' * depth, b.name, 'proxy' if b.assemblyContext else 'native',
            bb.minPoint.x, bb.minPoint.y, bb.minPoint.z,
            bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z))
    bb = occ.boundingBox
    if bb:
        log('%s  occ.boundingBox min=(%.2f, %.2f, %.2f) max=(%.2f, %.2f, %.2f)' % (
            '  ' * depth, bb.minPoint.x, bb.minPoint.y, bb.minPoint.z,
            bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z))
    else:
        log('%s  occ.boundingBox: None/empty' % ('  ' * depth))
    t = occ.transform.translation
    log('%s  transform t=(%.2f, %.2f, %.2f)' % ('  ' * depth, t.x, t.y, t.z))
    if depth < 2:
        for i in range(min(occ.childOccurrences.count, 2)):
            walk(occ.childOccurrences.item(i), depth + 1)


gen = None
mod0 = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name.startswith('generated_R_arm'):
        gen = o
    if o.component.name.startswith('Module 0 R'):
        mod0 = o
log('--- generated arm ---')
if gen:
    walk(gen)
else:
    log('no generated_R_arm found')
log('--- existing Module 0 R (known good) ---')
if mod0:
    walk(mod0)
else:
    log('no root-level "Module 0 R"; root occurrences: %s' % (
        [root.occurrences.item(i).component.name for i in range(root.occurrences.count)],))
