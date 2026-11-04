d = adsk.fusion.Design.cast(app.activeProduct)
root = d.rootComponent
for i in range(root.occurrences.count):
    o = root.occurrences.item(i); c = o.component
    log('%r joints %d asBuilt %d rigidGroups %d jointOrigins %d' % (c.name, c.joints.count, c.asBuiltJoints.count, c.rigidGroups.count, c.jointOrigins.count))
    for j in range(c.joints.count):
        jt = c.joints.item(j)
        try: log('   joint %r o1 %r o2 %r suppressed %s' % (jt.name, jt.occurrenceOne.name if jt.occurrenceOne else None, jt.occurrenceTwo.name if jt.occurrenceTwo else None, jt.isSuppressed))
        except Exception as e: log('   joint %r err %s' % (jt.name, e))
    for j in range(c.asBuiltJoints.count):
        jt = c.asBuiltJoints.item(j)
        try: log('   asbuilt %r o1 %r o2 %r' % (jt.name, jt.occurrenceOne.name if jt.occurrenceOne else None, jt.occurrenceTwo.name if jt.occurrenceTwo else None))
        except Exception as e: log('   asbuilt %r err %s' % (jt.name, e))
    for j in range(c.occurrences.count):
        oo = c.occurrences.item(j)
        log('   occ %r grounded %s' % (oo.name, oo.isGrounded))
log('design snapshots %s' % d.snapshots.count)
