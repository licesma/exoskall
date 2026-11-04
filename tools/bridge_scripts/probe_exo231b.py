"""EXO-231 probe 2: the link pad above each rotor face, and what is below the nut pocket."""
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

def runs_along(mat, r, ang, z0, z1, step, bodies):
    cx, cy = r*math.cos(math.radians(ang)), r*math.sin(math.radians(ang))
    out = []
    n = int(round(abs(z1-z0)/step))
    sgn = 1.0 if z1 > z0 else -1.0
    prev = 'INIT'
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

log('=== what sits above the rotor face at r 11 (bodies OUTSIDE the joint module) ===')
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
    for ang in (0.0, 90.0, 180.0, 270.0):
        rr = runs_along(mat, 11.0, ang, 0.0, 14.0, 0.1, others)
        log('  %-22s %5.1f deg: %s' % (wr.component.name, ang,
            ' | '.join('%s %.1f..%.1f' % (r[2] or 'air', r[0], r[1]) for r in rr)))

log('')
log('=== what sits BELOW the nut pocket at r 11 (bodies INSIDE the joint module) ===')
for i in range(gen.childOccurrences.count):
    wr = gen.childOccurrences.item(i)
    mod = module_of(wr)
    if mod is None: continue
    mat = mod.transform.copy(); mat.transformBy(wr.transform)
    mine = []
    def walk2(o):
        for b in o.bRepBodies: mine.append((o.component.name, b))
        for j in range(o.childOccurrences.count): walk2(o.childOccurrences.item(j))
    walk2(mod)
    for ang in (0.0, 90.0, 180.0, 270.0):
        rr = runs_along(mat, 11.0, ang, -9.0, -22.0, 0.1, mine)
        log('  %-22s %5.1f deg: %s' % (wr.component.name, ang,
            ' | '.join('%s %.1f..%.1f' % (r[2] or 'air', r[0], r[1]) for r in rr)))
    break   # the module is the same at every joint

log('')
log('=== the v3 module at the root: material at r 11 / r 9.4 / r 12.6 along z, and the horn disk ===')
v3 = None
for i in range(root.occurrences.count):
    if root.occurrences.item(i).component.name == BuildArm.MODULE_V3_NAME:
        v3 = root.occurrences.item(i)
mine = []
def walk3(o):
    for b in o.bRepBodies: mine.append((o.component.name, b))
    for j in range(o.childOccurrences.count): walk3(o.childOccurrences.item(j))
walk3(v3)
ident = ac.Matrix3D.create()
for r, ang in ((11.0, 0.0), (9.4, 0.0), (12.6, 0.0), (8.0, 0.0), (7.9, 0.0), (8.25, 45.0)):
    rr = runs_along(ident, r, ang, -16.0, -8.0, 0.05, mine)
    log('  r %5.2f %5.1f deg: %s' % (r, ang,
        ' | '.join('%s %.2f..%.2f' % (x[2] or 'air', x[0], x[1]) for x in rr)))
log('done')
