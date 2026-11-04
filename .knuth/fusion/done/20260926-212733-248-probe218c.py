"""EXO-218: why does the Luna arm report 0 interference hits where G1 reports 67?
Analyse ONE module wrapper of each arm on its own, and one link + its own module."""
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm, armgen
importlib.reload(BuildArm); importlib.reload(armgen)
root = design.rootComponent
arms = {}
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    arms[o.component.name] = o

def kid(arm, pre):
    a = arms[arm]
    for i in range(a.childOccurrences.count):
        c = a.childOccurrences.item(i)
        if c.component.name.startswith(pre):
            return c
    return None

for arm, wpre, lname in (('generated_R_arm_general', 'GJ1_', 'g_link_12'),
                         ('generated_R_arm_luna', 'GR1_', 'g_link_R12')):
    w = kid(arm, wpre)
    bs, names = BuildArm.collect_bodies(w)
    hits = BuildArm.analyze_interference(design, bs)
    log('%s / %s: %d bodies, %d internal hit(s)' % (arm, w.component.name, bs.count, len(hits)))
    log('    module component under it: %s' % w.childOccurrences.item(0).component.name)
    log('    first 6 body names: %s' % names[:6])
    for h in hits[:3]:
        log('    %s <-> %s  %.3f mm3' % (h[0], h[1], h[2]))
    lk = kid(arm, lname)
    both = adsk.core.ObjectCollection.create()
    for src in (lk, w):
        b2, _n = BuildArm.collect_bodies(src)
        for j in range(b2.count):
            both.add(b2.item(j))
    h2 = BuildArm.analyze_interference(design, both)
    log('    %s + its own module: %d bodies, %d hit(s)' % (lname, both.count, len(h2)))
    for h in h2[:3]:
        log('        %s <-> %s  %.4f mm3' % (h[0], h[1], h[2]))
