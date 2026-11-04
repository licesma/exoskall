"""EXO-225 evidence: the ORIGINAL module is untouched, v3 is what it says, and every joint
of `generated_R_arm_general` places v3.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/verify_exo225.py --timeout 300
"""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)

root = design.rootComponent
ORIG = {'stator': 25729.996, 'upper': 6473.293, 'lower': 2950.793}
problems = []


def measure(occ, tag):
    up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
    st = BuildArm.module_stator_occ(occ, in_context=True)
    screws, nuts = BuildArm.module_fastener_occs(occ)
    log('--- %s (%s) ---' % (tag, occ.component.name))
    out = {}
    for what, c in (('stator', st), ('upper', up), ('lower', lo)):
        r0, r1, z0, z1 = BuildArm._body_extent(c)
        v = c.bRepBodies.item(0).volume * 1000.0
        out[what] = v
        log('  %-7s %-14s r %7.3f..%7.3f  z %8.3f..%8.3f  V=%10.3f mm3'
            % (what, c.component.name, r0, r1, z0, z1, v))
    for i, s in enumerate(screws):
        r0, r1, z0, z1 = BuildArm._body_extent(s)
        log('  horn screw %d  r %7.3f..%7.3f  z %8.3f..%8.3f (under-head %.3f)'
            % (i, r0, r1, z0, z1, -9.0 - z0 if z1 < -6 else -5.0 - z0))
    for i, n in enumerate(nuts):
        r0, r1, z0, z1 = BuildArm._body_extent(n)
        log('  M3 nut %d      r %7.3f..%7.3f  z %8.3f..%8.3f' % (i, r0, r1, z0, z1))
    log('  stop block %s'
        % ', '.join('%.5f..%.5f (arc %.5f)' % (a, b, b - a)
                    for a, b in BuildArm._stop_block_of(occ)))
    return out, len(screws), len(nuts)


# ---- 1. the LINKED original, which nothing in EXO-225 may have touched ----
src = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.isReferencedComponent and o.component.name.startswith(BuildArm.MODULE_DOC_NAME):
        src = o
if src is None:
    problems.append('the LINKED "%s" is not in this document' % BuildArm.MODULE_DOC_NAME)
else:
    vols, ns, nn = measure(src, 'ORIGINAL, linked')
    for k, want in ORIG.items():
        if abs(vols[k] - want) > 1e-3:
            problems.append('ORIGINAL %s is %.3f mm3, was %.3f' % (k, vols[k], want))
    ok, lines = BuildArm.measure_rotor_v3(src)
    log('  the original does NOT read as v3: %s (expected)' % (not ok))
    if ok:
        problems.append('the ORIGINAL module reads as v3 - it has been modified')

# ---- 2. v3 ----
v3 = None
for i in range(root.occurrences.count):
    if root.occurrences.item(i).component.name == BuildArm.MODULE_V3_NAME:
        v3 = root.occurrences.item(i)
if v3 is None:
    problems.append('%s is not in this document' % BuildArm.MODULE_V3_NAME)
else:
    measure(v3, 'v3')
    ok, lines = BuildArm.measure_rotor_v3(v3)
    for l in lines:
        log(l)
    if not ok:
        problems.append('v3 does not read back')

# ---- 3. every joint of the arm places v3 ----
gen = None
for i in range(root.occurrences.count):
    if root.occurrences.item(i).component.name == 'generated_R_arm_general':
        gen = root.occurrences.item(i)
if gen is None:
    problems.append('generated_R_arm_general is not in this document')
else:
    names = {}
    for i in range(gen.childOccurrences.count):
        wr = gen.childOccurrences.item(i)
        for j in range(wr.childOccurrences.count):
            c = wr.childOccurrences.item(j)
            if c.component.name.startswith('sensor module 2'):
                names.setdefault(c.component.name, []).append(wr.component.name)
    log('--- modules placed in generated_R_arm_general ---')
    for n, ws in sorted(names.items()):
        log('  %-26s x%d  %s' % (n, len(ws), ', '.join(sorted(ws))))
    if list(names) != [BuildArm.MODULE_V3_NAME]:
        problems.append('the arm places %s, not only %s'
                        % (sorted(names), BuildArm.MODULE_V3_NAME))

log('')
log('EXO-225 verification: %s' % ('ALL OK' if not problems else '%d PROBLEM(S)' % len(problems)))
for p in problems:
    log('  PROBLEM: %s' % p)
if problems:
    raise RuntimeError('EXO-225 verification failed')
