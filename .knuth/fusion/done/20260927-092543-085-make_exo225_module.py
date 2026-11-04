"""EXO-225: make (or re-measure) `sensor module 2 v3` and report every number.

Runs `BuildArm.ensure_v3_module`, which holds the whole recipe. `sensor module 2` itself
is never touched. Re-runnable: a v3 already in the document is measured, not re-made,
unless it fails to read back.
"""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)

comp, how = BuildArm.ensure_v3_module(design, log)
log('')
log('%s: %s' % (comp.name, how))

root = design.rootComponent
occ = None
for i in range(root.occurrences.count):
    if root.occurrences.item(i).component.name == comp.name:
        occ = root.occurrences.item(i)
log('')
log('--- the v3 module, measured on the solid (module frame, mm) ---')
up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
for tag, c in (('upper rotor', up), ('lower rotor', lo)):
    r0, r1, z0, z1 = BuildArm._body_extent(c)
    log('%-12s %-14s r %7.3f..%7.3f  z %8.3f..%8.3f  V=%.3f mm3'
        % (tag, c.component.name, r0, r1, z0, z1, c.bRepBodies.item(0).volume * 1000.0))
screws, nuts = BuildArm._fastener_occs(occ)
for i, s in enumerate(screws):
    r0, r1, z0, z1 = BuildArm._body_extent(s)
    log('horn screw %d  r %7.3f..%7.3f  z %8.3f..%8.3f' % (i, r0, r1, z0, z1))
for i, n in enumerate(nuts):
    r0, r1, z0, z1 = BuildArm._body_extent(n)
    log('M3 nut %d      r %7.3f..%7.3f  z %8.3f..%8.3f' % (i, r0, r1, z0, z1))
st = BuildArm.module_stator_occ(occ, in_context=True)
bb = st.bRepBodies.item(0).boundingBox
log('stator       %-14s z %8.3f..%8.3f  V=%.3f mm3 (block %s)'
    % (st.component.name, bb.minPoint.z * 10, bb.maxPoint.z * 10,
       st.bRepBodies.item(0).volume * 1000.0,
       ', '.join('%.5f..%.5f' % r for r in BuildArm._stop_block_of(occ))))
