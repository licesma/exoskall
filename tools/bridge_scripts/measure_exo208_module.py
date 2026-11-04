"""EXO-208: measure the narrowed joint stop on the module copy, off the SOLID, in Fusion.

Point containment on the stator body, in the module frame, at r 24 / z -2.2 - the same
probe that measured the original block at 158.00258..178.00192 (arc 19.99935, centre
168.00225, r 21.050..27.000, z -4.000..-0.400). Nothing is modified.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/measure_exo208_module.py
"""
import math
import sys
import importlib

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
import BuildArm
importlib.reload(armgen)
importlib.reload(BuildArm)

root = design.rootComponent
INSIDE = adsk.fusion.PointContainment.PointInsidePointContainment
ON = adsk.fusion.PointContainment.PointOnPointContainment


def stator_of(occ):
    """The r > 26 single-body child of a module occurrence, as a proxy in `occ`'s context."""
    best, out = None, None
    for i in range(occ.childOccurrences.count):
        c = occ.childOccurrences.item(i)
        if c.bRepBodies.count != 1:
            continue
        b = c.bRepBodies.item(0)
        bb = b.boundingBox
        r = max(abs(bb.minPoint.x), abs(bb.maxPoint.x)) * 10.0
        if r > 26.0 and (best is None or bb.maxPoint.z > best):
            best, out = bb.maxPoint.z, b
    return out


def measure(body, label):
    def solid(r_mm, a_deg, z_mm):
        p = adsk.core.Point3D.create(r_mm * math.cos(math.radians(a_deg)) / 10.0,
                                     r_mm * math.sin(math.radians(a_deg)) / 10.0, z_mm / 10.0)
        return body.pointContainment(p) in (INSIDE, ON)

    def edge(a_in, a_out, r_mm, z_mm):
        for _ in range(60):
            mid = (a_in + a_out) / 2.0
            if solid(r_mm, mid, z_mm):
                a_in = mid
            else:
                a_out = mid
        return (a_in + a_out) / 2.0

    R_MID, Z_MID = 24.0, -2.2
    hits = [a * 0.25 for a in range(1440) if solid(R_MID, a * 0.25, Z_MID)]
    bb = body.boundingBox
    log('%s: body %r, volume %.3f mm3, bbox z %.3f..%.3f'
        % (label, body.name, body.volume * 1000.0, bb.minPoint.z * 10, bb.maxPoint.z * 10))
    if not hits:
        log('  NO BLOCK at r %.0f / z %.1f' % (R_MID, Z_MID))
        return None
    runs, cur = [], [hits[0]]
    for a in hits[1:]:
        (cur.append(a) if a - cur[-1] <= 0.2501 else (runs.append(cur), cur.clear(),
                                                      cur.append(a)))
    runs.append(list(cur))
    log('  %d sample(s) inside, %d contiguous run(s)' % (len(hits), len(runs)))
    out = []
    for run in runs:
        lo = edge(run[0], run[0] - 0.25, R_MID, Z_MID)
        hi = edge(run[-1], run[-1] + 0.25, R_MID, Z_MID)
        ac = (lo + hi) / 2.0
        rs = [r * 0.02 for r in range(750, 1500) if solid(r * 0.02, ac, Z_MID)]
        zs = [-6.0 + k * 0.01 for k in range(700) if solid(R_MID, ac, -6.0 + k * 0.01)]
        log('  BLOCK %.5f .. %.5f deg   arc %.5f   centre %.5f'
            % (lo, hi, hi - lo, ac))
        log('        r %.3f..%.3f mm, z %.3f..%.3f mm (at the block centre)'
            % (min(rs), max(rs), min(zs), max(zs)))
        out.append((lo, hi, min(rs), max(rs), min(zs), max(zs)))
    return out


found = {}
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name in (BuildArm.MODULE_DOC_NAME, BuildArm.MODULE_STOP_NAME):
        found.setdefault(o.component.name, o)
for name in (BuildArm.MODULE_DOC_NAME, BuildArm.MODULE_STOP_NAME):
    o = found.get(name)
    if o is None:
        log('%s: no root occurrence' % name)
        continue
    st = stator_of(o)
    res = measure(st, name)
    if name == BuildArm.MODULE_DOC_NAME and res:
        was_ = res[0]
    if name == BuildArm.MODULE_STOP_NAME and res:
        lo, hi, r0, r1, z0, z1 = res[0]
        # the ANGLES are compared to what the rule asks for; the RADII and Z are compared to
        # the ORIGINAL block measured the same way a moment ago, because EXO-208 promises
        # they do not move - not to a nominal, which would only measure this probe's own
        # step size.
        d_ang = [abs(lo - armgen.JOINT_STOP_LO_DEG), abs(hi - armgen.JOINT_STOP_HI_DEG)]
        d_geo = [abs(r0 - was_[2]), abs(r1 - was_[3]), abs(z0 - was_[4]), abs(z1 - was_[5])]
        log('  ASKED  %.5f .. %.5f deg (JOINT_STOP_DEG %.1f on the measured centre %.5f)'
            % (armgen.JOINT_STOP_LO_DEG, armgen.JOINT_STOP_HI_DEG, armgen.JOINT_STOP_DEG,
               math.degrees(armgen.JOINT_STOP_MID)))
        log('  ANGLE  error %.2e / %.2e deg -> %s'
            % (d_ang[0], d_ang[1], 'PASS' if max(d_ang) < 0.01 else 'FAIL'))
        log('  UNMOVED vs the original block: r %.3f / %.3f mm, z %.3f / %.3f mm -> %s'
            % tuple(d_geo + ['PASS' if max(d_geo) < 0.011 else 'FAIL']))
        log('  %d block(s) left on the rim (must be 1) -> %s'
            % (len(res), 'PASS' if len(res) == 1 else 'FAIL'))

# and the placed modules in the arm: every one of them must be the narrowed component
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if not o.component.name.startswith('generated_R_arm_general'):
        continue
    log('')
    names = []
    for k in range(o.childOccurrences.count):
        w = o.childOccurrences.item(k)
        for m in range(w.childOccurrences.count):
            names.append(w.childOccurrences.item(m).component.name)
    from collections import Counter
    log('%s: modules placed -> %s' % (o.component.name, dict(Counter(names))))
