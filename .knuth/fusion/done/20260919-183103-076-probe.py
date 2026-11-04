root = design.rootComponent
def walk(occ, depth, path):
    for i in range(occ.childOccurrences.count if depth else root.occurrences.count):
        o = (occ.childOccurrences.item(i) if depth else root.occurrences.item(i))
        nm = o.component.name
        if depth <= 2:
            t = o.transform2 if hasattr(o,'transform2') else o.transform
            tr = t.translation
            log('%s%-28s  t=(%8.3f,%8.3f,%8.3f) cm  light=%s' % ('  '*depth, nm, tr.x, tr.y, tr.z, o.isLightBulbOn))
        if depth < 2:
            walk(o, depth+1, path+'/'+nm)
walk(None, 0, '')
