"""EXO-218: find where design.analyzeInterference stops answering on the Luna arm.
Adds the arm's children one at a time and reports res is None vs res.count."""
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)
root = design.rootComponent
arms = {}
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    arms[o.component.name] = o


def raw(bodies):
    if bodies.count < 2:
        return 'n/a', 0
    inp = design.createInterferenceInput(bodies)
    inp.areCoincidentFacesIncluded = False
    res = design.analyzeInterference(inp)
    return ('None' if res is None else 'ok'), (0 if res is None else res.count)


for arm in ('generated_R_arm_luna', 'generated_R_arm_general'):
    a = arms[arm]
    kids = [a.childOccurrences.item(i) for i in range(a.childOccurrences.count)]
    coll = adsk.core.ObjectCollection.create()
    log(arm)
    for c in kids:
        bs, _n = BuildArm.collect_bodies(c)
        for j in range(bs.count):
            coll.add(bs.item(j))
        st, n = raw(coll)
        log('  +%-20s bodies %3d -> res %s, %d hit(s)' % (c.component.name, coll.count,
                                                          st, n))
