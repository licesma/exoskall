"""EXO-225: what is in the document, and what the module's children look like."""
import math
root = design.rootComponent
log('root occurrences: %d' % root.occurrences.count)
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    t = o.transform.translation
    log('  [%d] %-34s ref=%s bodies=%d children=%d t=(%.2f,%.2f,%.2f)'
        % (i, o.component.name, o.isReferencedComponent, o.component.bRepBodies.count,
           o.childOccurrences.count, t.x*10, t.y*10, t.z*10))

def dump(occ, ind='    '):
    for j in range(occ.childOccurrences.count):
        c = occ.childOccurrences.item(j)
        comp = c.component
        tr = c.transform
        a = tr.asArray()
        bbs = []
        for k in range(c.bRepBodies.count):
            b = c.bRepBodies.item(k)
            bb = b.boundingBox
            bbs.append('%s r<=%.3f z %.3f..%.3f V=%.1f'
                       % (b.name,
                          max(abs(bb.minPoint.x), abs(bb.maxPoint.x),
                              abs(bb.minPoint.y), abs(bb.maxPoint.y))*10,
                          bb.minPoint.z*10, bb.maxPoint.z*10, b.volume*1000))
        log('%s%-24s ref=%s nb=%d t=(%.3f,%.3f,%.3f) rot=[%.3f %.3f %.3f | %.3f %.3f %.3f]'
            % (ind, comp.name, c.isReferencedComponent, c.bRepBodies.count,
               tr.translation.x*10, tr.translation.y*10, tr.translation.z*10,
               a[0], a[1], a[2], a[4], a[5], a[6]))
        for s in bbs:
            log('%s    %s' % (ind, s))
        dump(c, ind + '  ')

for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    n = o.component.name
    if n.startswith('sensor module 2'):
        log('--- children of %s (occ %d) ---' % (n, i))
        dump(o)
        break
