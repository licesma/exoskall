import adsk.core, adsk.fusion
root = design.rootComponent
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if not o.component.name.startswith('generated_R_arm'):
        continue
    for j in range(o.childOccurrences.count):
        s = o.childOccurrences.item(j)
        if not s.component.name.startswith('link_'):
            continue
        b = s.bRepBodies.item(0)
        kinds = {}
        for k in range(b.faces.count):
            n = b.faces.item(k).geometry.surfaceType
            kinds[n] = kinds.get(n, 0) + 1
        names = {0: 'plane', 1: 'cylinder', 2: 'cone', 3: 'sphere', 4: 'torus',
                 5: 'ellipticalCylinder', 6: 'ellipticalCone', 7: 'nurbs'}
        log('%s: %d faces, volume %.1f mm3, %s' % (
            s.component.name, b.faces.count, b.volume * 1000.0,
            ', '.join('%d %s' % (v, names.get(k, k)) for k, v in sorted(kinds.items()))))
