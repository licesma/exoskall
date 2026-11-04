"""EXO-350 read-only check: document saved state/version, root occurrences, volumes of the
untouched components (all bodies in all sub-occurrences) and of the new export copies."""
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
d = app.activeDocument
des = adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
say('doc %r isModified %s isSaved %s dataFile v%s latest v%s' % (d.name, d.isModified, d.isSaved, d.dataFile.versionNumber, d.dataFile.latestVersionNumber))
root = des.rootComponent
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    v = sum(o.component.bRepBodies.item(k).volume for k in range(o.component.bRepBodies.count))
    for j in range(o.component.allOccurrences.count):
        c = o.component.allOccurrences.item(j).component
        v += sum(c.bRepBodies.item(k).volume for k in range(c.bRepBodies.count))
    say('%r comp %r bulb %s bodies-volume %.4f (sum over sub-occurrence components)' % (o.name, o.component.name, o.isLightBulbOn, v*1000))
