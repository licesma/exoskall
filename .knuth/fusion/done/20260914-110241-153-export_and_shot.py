"""Exercise the bridge helpers: screenshot + STEP export of the whole root, plus
a light STEP of just the generated arm component (world == arm frame, it is at
identity) for measurement."""
import time
t0 = time.time()
p = screenshot('/Users/esteban/repos/exo/gento_luna/exo/screenshots/generated_R_arm.png')
log('screenshot -> %s (%.1fs)' % (p, time.time() - t0))

root = design.rootComponent
gen = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name.startswith('generated_R_arm'):
        gen = o
if gen is not None:
    t0 = time.time()
    path = '/Users/esteban/repos/exo/gento_luna/exo/exports/generated_R_arm_only.step'
    em = design.exportManager
    opts = em.createSTEPExportOptions(path, gen.component)
    log('component-only STEP: %s (%.1fs)' % (em.execute(opts), time.time() - t0))
else:
    log('no generated_R_arm occurrence found')

t0 = time.time()
p = export_step('/Users/esteban/repos/exo/gento_luna/exo/exports/generated_R_arm.step')
log('export_step -> %s (%.1fs)' % (p, time.time() - t0))
