# Interference check among all bodies under generated_R_arm (links + modules).
root = design.rootComponent
arm = None
for occ in root.occurrences:
    if occ.component.name.startswith('generated_R_arm'):
        arm = occ; break
if arm is None:
    raise RuntimeError('generated_R_arm not found')
bodies = adsk.core.ObjectCollection.create()
names = {}
for b in arm.bRepBodies:
    bodies.add(b); names[b.entityToken] = arm.component.name + '/' + b.name
for sub in arm.childOccurrences:
    for b in sub.bRepBodies:
        bodies.add(b); names[b.entityToken] = sub.name + '/' + b.name
    for sub2 in sub.childOccurrences:
        for b in sub2.bRepBodies:
            bodies.add(b); names[b.entityToken] = sub.name + '>' + sub2.name + '/' + b.name
log('bodies checked: %d' % bodies.count)
inp = design.createInterferenceInput(bodies)
inp.areCoincidentFacesIncluded = False
res = design.analyzeInterference(inp)
if res is None or res.count == 0:
    log('NO INTERFERENCE')
else:
    log('INTERFERENCES: %d' % res.count)
    for i in range(res.count):
        r = res.item(i)
        a = names.get(r.entityOne.entityToken, r.entityOne.name)
        b = names.get(r.entityTwo.entityToken, r.entityTwo.name)
        vol = r.interferenceBody.volume if r.interferenceBody else -1
        log('  %s  <->  %s   volume=%.1f mm3' % (a, b, vol * 1000.0))
