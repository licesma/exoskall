"""EXO-158: the 3 FAILs of the general arm - which two occurrences, and how deep."""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm, BuildArmGen
importlib.reload(armgen); importlib.reload(BuildArm); importlib.reload(BuildArmGen)

root = design.rootComponent
arm = None
for i in range(root.occurrences.count):
    if root.occurrences.item(i).component.name == BuildArmGen.ARM_NAME:
        arm = root.occurrences.item(i)
subs = [arm.childOccurrences.item(i) for i in range(arm.childOccurrences.count)]
wraps = dict((s.component.name, s) for s in subs if s.component.name.startswith('GJ'))


def hits(occs):
    coll = adsk.core.ObjectCollection.create()
    for o in occs:
        b, _n = BuildArm.collect_bodies(o)
        for k in range(b.count):
            coll.add(b.item(k))
    return BuildArm.analyze_interference(design, coll)


own = {}
for n, w in wraps.items():
    own[n] = set(BuildArm.hit_key(*h) for h in hits([w]))
names = sorted(wraps)
for i in range(len(names)):
    for j in range(i + 1, len(names)):
        a, b = names[i], names[j]
        cross = [h for h in hits([wraps[a], wraps[b]])
                 if BuildArm.hit_key(*h) not in own[a] and BuildArm.hit_key(*h) not in own[b]]
        if cross:
            log('%s <-> %s: %d CROSS hit(s)' % (a, b, len(cross)))
            for na, nb, vol, bb in cross:
                d = None if bb is None else min(bb[1][k] - bb[0][k] for k in range(3))
                log('    %s <-> %s volume=%.1f mm3 depth=%s bbox=%s'
                    % (na, nb, vol, 'n/a' if d is None else '%.3f mm' % d,
                       BuildArm.fmt_bbox(bb)))
# how far apart the wrist modules actually are, and how wide they are
for n in ('GJ5_right_wrist_roll_joint', 'GJ6_right_wrist_pitch_joint',
          'GJ7_right_wrist_yaw_joint'):
    w = wraps[n]
    b, _ = BuildArm.collect_bodies(w)
    lo = [1e9] * 3
    hi = [-1e9] * 3
    for k in range(b.count):
        bb = b.item(k).boundingBox
        for t in range(3):
            lo[t] = min(lo[t], [bb.minPoint.x, bb.minPoint.y, bb.minPoint.z][t] * 10)
            hi[t] = max(hi[t], [bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z][t] * 10)
    log('%s bbox x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]'
        % (n, lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
log('done')
