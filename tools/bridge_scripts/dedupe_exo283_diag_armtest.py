"""EXO-283 diagnostic (never saves): arm test - replace step by step (param replaceAll flag), log counts and
cuffs occurrences after each step; always close(False)."""
import sys, importlib, time
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import dedupe_exo283_common as C; importlib.reload(C)
g = C.get; pm = C.param(); RA = pm.get('replaceAll', True)
fid = C.ASM['arm test']; df = C.find_file(app, fid)
assert app.documents.count == 0
doc = app.documents.open(df, True)
try:
    des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType')); root = des.rootComponent
    def snap(tag):
        occs = C.items(root.allOccurrences)
        cuffs = [(o.fullPathName, C.short(g(lambda o=o: o.documentReference.dataFile.id)), g(lambda o=o: o.childOccurrences.count), g(lambda o=o: o.component.name)) for o in occs if 'cuffs' in o.fullPathName and g(lambda o=o: o.isReferencedComponent) and not '+' in o.fullPathName.split('cuffs',1)[1]]
        log('%s: comps %d occ %d linked %d cuffs %s' % (tag, des.allComponents.count, len(occs), sum(1 for o in occs if g(lambda o=o: o.isReferencedComponent) is True), cuffs))
    snap('start')
    for _ in range(25):
        hit = None
        for o in C.items(root.allOccurrences):
            if g(lambda o=o: o.isReferencedComponent) is True:
                f = g(lambda o=o: o.documentReference.dataFile.id)
                if f in C.EXTRA: hit = (o, f); break
        if hit is None: break
        o, f = hit; p_ = o.fullPathName
        ok = o.replace(C.find_file(app, C.KEPT[C.EXTRA[f]]), RA)
        snap('after %s %s->%s ok=%s' % (p_, C.short(f), C.EXTRA[f], ok))
finally:
    doc.close(False)
log('open after: %d' % app.documents.count)
