"""EXO-350 read-only check: document saved state/version, root occurrences, volumes of the
untouched components (all bodies in all sub-occurrences) and of the new export copies."""
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
d = app.activeDocument
des = adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
say('doc %r isModified %s isSaved %s dataFile v%s latest v%s' % (d.name, d.isModified, d.isSaved, d.dataFile.versionNumber, d.dataFile.latestVersionNumber))
root = des.rootComponent
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name.startswith('generated_R_arm'):
        say('%r comp %r bulb %s (arm, not summed)' % (o.name, o.component.name, o.isLightBulbOn)); continue
    comps = [o.component] + [o.component.allOccurrences.item(j).component for j in range(o.component.allOccurrences.count)]
    v, bad, n = 0.0, 0, 0
    for c in comps:
        for k in range(c.bRepBodies.count):
            n += 1
            try: v += c.bRepBodies.item(k).volume
            except Exception: bad += 1
    say('%r comp %r bulb %s bodies %d, volume sum %.4f (%d unreadable)' % (o.name, o.component.name, o.isLightBulbOn, n, v*1000, bad))
