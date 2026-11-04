"""EXO-225: the module's children, transforms and bodies (volume guarded)."""
root = design.rootComponent
def vol(b):
    try:
        return b.volume * 1000.0
    except Exception:
        return float('nan')
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if not o.component.name.startswith('sensor module 2'):
        continue
    log('=== %s (ref=%s) ===' % (o.component.name, o.isReferencedComponent))
    for j in range(o.childOccurrences.count):
        c = o.childOccurrences.item(j)
        a = c.transform.asArray()
        log('  %-26s nb=%d t=(%.3f,%.3f,%.3f) R=[%.3f %.3f %.3f ; %.3f %.3f %.3f ; %.3f %.3f %.3f]'
            % (c.component.name, c.bRepBodies.count,
               a[3]*10, a[7]*10, a[11]*10,
               a[0], a[1], a[2], a[4], a[5], a[6], a[8], a[9], a[10]))
        if 'Bearing' in c.component.name:
            continue
        for k in range(c.bRepBodies.count):
            b = c.bRepBodies.item(k)
            bb = b.boundingBox
            log('      %-10s x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f  V=%.3f'
                % (b.name, bb.minPoint.x*10, bb.maxPoint.x*10, bb.minPoint.y*10,
                   bb.maxPoint.y*10, bb.minPoint.z*10, bb.maxPoint.z*10, vol(b)))
    log('')
    # components (not occurrences) that are distinct
    seen = {}
    for j in range(o.childOccurrences.count):
        c = o.childOccurrences.item(j)
        seen.setdefault(c.component.name, 0)
        seen[c.component.name] += 1
    log('  distinct child components: %s' % sorted(seen.items()))
