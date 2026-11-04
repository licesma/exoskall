"""EXO-158: which two modules of the general arm actually interfere."""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm, BuildArmGen
importlib.reload(armgen); importlib.reload(BuildArm); importlib.reload(BuildArmGen)

root = design.rootComponent
arm = None
for i in range(root.occurrences.count):
    if root.occurrences.item(i).component.name == BuildArmGen.ARM_NAME:
        arm = root.occurrences.item(i)
if arm is None:
    raise RuntimeError('no %s' % BuildArmGen.ARM_NAME)
subs = [arm.childOccurrences.item(i) for i in range(arm.childOccurrences.count)]
wrappers = [s for s in subs if s.component.name.startswith('GJ')]
links = [s for s in subs if s.component.name.startswith('g_link')]
log('wrappers: %s' % ', '.join(w.component.name for w in wrappers))
for i in range(len(wrappers)):
    for j in range(i + 1, len(wrappers)):
        coll = adsk.core.ObjectCollection.create()
        for w in (wrappers[i], wrappers[j]):
            b, _n = BuildArm.collect_bodies(w)
            for k in range(b.count):
                coll.add(b.item(k))
        hits = BuildArm.analyze_interference(design, coll)
        bad = []
        for a, b, vol, bb in hits:
            if bb is None:
                bad.append((a, b, vol, None)); continue
            if min(bb[1][k] - bb[0][k] for k in range(3)) > armgen.INTERFERENCE_DEPTH_TOL:
                bad.append((a, b, vol, bb))
        if bad:
            log('%s <-> %s: %d deep hit(s)' % (wrappers[i].component.name,
                                               wrappers[j].component.name, len(bad)))
            for a, b, vol, bb in bad:
                log('    %s <-> %s volume=%.1f mm3 bbox=%s' % (a, b, vol,
                                                               BuildArm.fmt_bbox(bb)))
for w in wrappers:
    for l in links:
        coll = adsk.core.ObjectCollection.create()
        for o in (w, l):
            b, _n = BuildArm.collect_bodies(o)
            for k in range(b.count):
                coll.add(b.item(k))
        for a, b, vol, bb in BuildArm.analyze_interference(design, coll):
            if bb is not None and min(bb[1][k] - bb[0][k] for k in range(3)) > armgen.INTERFERENCE_DEPTH_TOL:
                log('LINK HIT %s <-> %s: %s <-> %s volume=%.1f' % (
                    l.component.name, w.component.name, a, b, vol))
log('done')
