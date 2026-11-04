"""EXO-231 probe 3: where the link's rotor pad really is, in each joint's module frame."""
import math, sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
import adsk.core as ac, adsk.fusion as af

root = design.rootComponent
gen = None
for i in range(root.occurrences.count):
    if root.occurrences.item(i).component.name == 'generated_R_arm_general':
        gen = root.occurrences.item(i)

def module_of(wr):
    for i in range(wr.childOccurrences.count):
        c = wr.childOccurrences.item(i)
        if c.component.name.startswith('sensor module 2'):
            return c
    return None

def world(mat, x, y, z):
    a = mat.asArray()
    return ac.Point3D.create((a[0]*x + a[1]*y + a[2]*z)/10.0 + a[3],
                             (a[4]*x + a[5]*y + a[6]*z)/10.0 + a[7],
                             (a[8]*x + a[9]*y + a[10]*z)/10.0 + a[11])

def runs_xy(mat, cx, cy, z0, z1, step, bodies):
    out, prev = [], 'INIT'
    n = int(round(abs(z1-z0)/step)); sgn = 1.0 if z1 > z0 else -1.0
    for k in range(n+1):
        z = z0 + sgn*k*step
        p = world(mat, cx, cy, z)
        hit = None
        for onm, ob in bodies:
            if ob.pointContainment(p) == af.PointContainment.PointInsidePointContainment:
                hit = onm; break
        if hit != prev:
            out.append([z, z, hit]); prev = hit
        else:
            out[-1][1] = z
    return out

for i in range(gen.childOccurrences.count):
    wr = gen.childOccurrences.item(i)
    mod = module_of(wr)
    if mod is None: continue
    mat = mod.transform.copy(); mat.transformBy(wr.transform)
    others = []
    def walk(o, skip):
        if o is skip: return
        for b in o.bRepBodies: others.append((o.component.name, b))
        for j in range(o.childOccurrences.count): walk(o.childOccurrences.item(j), skip)
    for j in range(gen.childOccurrences.count): walk(gen.childOccurrences.item(j), mod)
    log('--- %s ---' % wr.component.name)
    for ang in (0.0, 90.0, 180.0, 270.0):
        ux, uy = math.cos(math.radians(ang)), math.sin(math.radians(ang))
        # a point 2.0 mm to the side of the hole axis, in the tangential direction
        cx, cy = 11.0*ux - 2.0*uy, 11.0*uy + 2.0*ux
        rr = runs_xy(mat, cx, cy, -2.0, 16.0, 0.1, others)
        log('  %5.1f deg, 2 mm tangential: %s' % (ang,
            ' | '.join('%s %.1f..%.1f' % (x[2] or 'air', x[0], x[1]) for x in rr)))
    # and on the axis at r 0 (the middle of the pad) and at r 16
    for r in (0.0, 16.0):
        rr = runs_xy(mat, r, 0.0, -2.0, 16.0, 0.1, others)
        log('  r %4.1f  0 deg: %s' % (r,
            ' | '.join('%s %.1f..%.1f' % (x[2] or 'air', x[0], x[1]) for x in rr)))
log('done')
