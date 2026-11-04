import sys, importlib, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen; importlib.reload(armgen)

root = design.rootComponent
arm = next(o for o in root.occurrences if o.component.name.startswith('generated_R_arm'))
l45 = next(s for s in arm.childOccurrences if s.component.name.startswith('link_45'))
body = l45.bRepBodies.item(0)

hr, t = armgen.HOLE_R, armgen.PAD_T
full = 2*math.pi*hr*t
log('expected FULL hole cylinder area = 2*pi*%.1f*%.1f = %.2f mm2' % (hr, t, full))
n = 0
for f in body.faces:
    g = f.geometry
    if g.objectType == 'adsk::core::Cylinder' and abs(g.radius*10.0 - hr) < 0.05:
        a = f.area*100.0          # cm2 -> mm2
        n += 1
        pct = 100.0*a/full
        log('  hole %d: area %.2f mm2 = %.1f%% of full  -> %s' %
            (n, a, pct, 'INTACT' if pct > 99.0 else 'CUT BY THE RAMP'))
log('%d hole faces checked' % n)
