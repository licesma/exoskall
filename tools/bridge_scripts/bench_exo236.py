"""EXO-236: how fast is pointContainment? Times it on the LINKED original's rotor bodies so
the symmetry sweep in verify_exo236.py can be sized without freezing Fusion for minutes."""
import sys, time, importlib, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)
import adsk.core as ac, adsk.fusion as af

root = design.rootComponent
src = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.isReferencedComponent and o.component.name.startswith(BuildArm.MODULE_DOC_NAME):
        src = o
up, lo = BuildArm.module_rotor_occs(src, in_context=True)
b = up.bRepBodies.item(0)
N = 4000
t0 = time.time()
for k in range(N):
    a = (k * 0.31) % 360.0
    r = 2.0 + (k % 30) * 0.5
    z = -8.5 + (k % 17) * 0.5
    x, y = BuildArm._polar(r, a)
    b.pointContainment(ac.Point3D.create(x / 10.0, y / 10.0, z / 10.0))
dt = time.time() - t0
log('%d pointContainment calls in %.3fs = %.1f us each, %.0f calls/s' % (N, dt, dt / N * 1e6, N / dt))
log('a 466k-call sweep would take %.0f s' % (466000 * dt / N))
