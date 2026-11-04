"""EXO-242 step 2: make `sensor module 2 v3` in the SAVED document, and SAVE.

`BuildArm.ensure_v3_module` is the whole recipe (EXO-225 copy + breakLink, EXO-208 15 deg
stop, EXO-238 eight pegs, EXO-240 through column and 5.8 AF nut pocket) and it refuses a
module that does not read back on all `V3_PROBES`. This script runs it, reads the two
rotor part volumes off the solid against the EXO-240/EXO-241 record, confirms the LINKED
original is untouched, and SAVES the document.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/make_v3_exo242.py --timeout 900
"""
import importlib
import sys

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)

# EXO-240 / EXO-241, read off the solid then. These are the numbers to reproduce.
REC = {'upper': 6657.964, 'lower': 2507.386}
ORIG = {'stator': 25729.996, 'upper': 6473.293, 'lower': 2950.793}

root = design.rootComponent
log('document: %r' % app.activeDocument.name)
log(BuildArm.save_active_document(app))

comp, how = BuildArm.ensure_v3_module(design, log)
log('%s: %s' % (comp.name, how))

occ = None
for i in range(root.occurrences.count):
    if root.occurrences.item(i).component.name == comp.name:
        occ = root.occurrences.item(i)
if occ is None:
    raise RuntimeError('%s has no occurrence at the root' % comp.name)

problems = []
src = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.isReferencedComponent and o.component.name.startswith(BuildArm.MODULE_DOC_NAME):
        src = o
log('')
log('--- the ORIGINAL, still LINKED ---')
if src is None:
    problems.append('the LINKED "%s" is not in this document' % BuildArm.MODULE_DOC_NAME)
else:
    up0, lo0 = BuildArm.module_rotor_occs(src, in_context=True)
    st0 = BuildArm.module_stator_occ(src, in_context=True)
    for what, c in (('stator', st0), ('upper', up0), ('lower', lo0)):
        v = c.bRepBodies.item(0).volume * 1000.0
        log('  %-7s V=%10.3f mm3 (record %10.3f)' % (what, v, ORIG[what]))
        if abs(v - ORIG[what]) > 1e-3:
            problems.append('ORIGINAL %s is %.3f mm3, record %.3f' % (what, v, ORIG[what]))

log('')
log('--- v3 rotor parts against the EXO-240 record ---')
up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
for tag, c in (('upper', up), ('lower', lo)):
    r0, r1, z0, z1 = BuildArm._body_extent(c)
    v = c.bRepBodies.item(0).volume * 1000.0
    d = v - REC[tag]
    log('  %-6s V=%10.3f mm3 (record %10.3f, %+0.4f)  r %6.3f..%6.3f z %8.3f..%8.3f'
        % (tag, v, REC[tag], d, r0, r1, z0, z1))
    if abs(d) > 1e-3:
        problems.append('v3 %s is %.3f mm3, the record is %.3f' % (tag, v, REC[tag]))

nut, bolt = BuildArm.module_v3_fasteners(occ)
log('  fasteners: nuts %s, bolts %s' % (nut is not None, bolt is not None))
if nut is None or bolt is None:
    problems.append('the standard nuts / bolts are missing')

log('')
log(BuildArm.save_active_document(app))
if problems:
    raise RuntimeError('EXO-242 step 2 FAILED:\n  ' + '\n  '.join(problems))
log('EXO-242 step 2: module reproduces its recorded numbers, document saved.')
