"""EXO-231 probe: the M3 nut as modelled, the link pad above the rotor face, and what the
document holds. Read-only."""
import math, sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
import adsk.core as ac, adsk.fusion as af

root = design.rootComponent
log('--- root occurrences ---')
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    log('  %-44s ref=%s bulb=%s' % (o.component.name, o.isReferencedComponent, o.isLightBulbOn))

def nut_detail(occ, tag):
    b = occ.bRepBodies.item(0)
    log('  %s  component=%s  body=%s  V=%.4f mm3' % (tag, occ.component.name, b.name, b.volume*1000.0))
    r0, r1, z0, z1 = BuildArm._body_extent(occ)
    log('    extent r %.4f..%.4f  z %.4f..%.4f' % (r0, r1, z0, z1))
    for f in b.faces:
        g = f.geometry
        t = g.objectType.split('::')[-1]
        if t == 'Cylinder':
            log('    cyl  R=%.4f  axis=(%.3f,%.3f,%.3f) origin z=%.4f area=%.4f'
                % (g.radius*10.0, g.axis.x, g.axis.y, g.axis.z, g.origin.z*10.0, f.area*100.0))
        elif t == 'Plane':
            n = g.normal
            log('    plane n=(%.3f,%.3f,%.3f) origin=(%.4f,%.4f,%.4f) area=%.4f'
                % (n.x, n.y, n.z, g.origin.x*10, g.origin.y*10, g.origin.z*10, f.area*100.0))
        elif t == 'Cone':
            log('    cone halfAngle=%.4f deg R=%.4f originz=%.4f' % (math.degrees(g.halfAngle), g.radius*10.0, g.origin.z*10.0))
        else:
            log('    %s area=%.4f' % (t, f.area*100.0))

for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    n = o.component.name
    if not n.startswith('sensor module 2'):
        continue
    log('--- %s (ref=%s) ---' % (n, o.isReferencedComponent))
    try:
        screws, nuts = BuildArm.module_fastener_occs(o)
    except Exception as e:
        log('  fastener find failed: %s' % e); continue
    log('  %d screws, %d nuts' % (len(screws), len(nuts)))
    for k, s in enumerate(screws[:1]):
        log('  screw component=%s body V=%.4f' % (s.component.name, s.bRepBodies.item(0).volume*1000.0))
    for k, nt in enumerate(nuts):
        if k == 0:
            nut_detail(nt, 'nut0')
        else:
            r0, r1, z0, z1 = BuildArm._body_extent(nt)
            log('  nut%d component=%s r %.4f..%.4f z %.4f..%.4f' % (k, nt.component.name, r0, r1, z0, z1))
    # also list all child component names once
    log('  children: %s' % sorted(set(o.childOccurrences.item(j).component.name
                                      for j in range(o.childOccurrences.count))))

# --- the generated arm: what sits above each rotor face at r 11 ---
gen = None
for i in range(root.occurrences.count):
    if root.occurrences.item(i).component.name == 'generated_R_arm_general':
        gen = root.occurrences.item(i)
if gen is None:
    log('no generated_R_arm_general'); raise SystemExit
log('--- generated_R_arm_general children ---')
for i in range(gen.childOccurrences.count):
    c = gen.childOccurrences.item(i)
    log('  %-26s bodies=%d children=%d' % (c.component.name, c.bRepBodies.count, c.childOccurrences.count))

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

log('--- solid along module +z at r 11, each joint, each M3 angle (link pad) ---')
for i in range(gen.childOccurrences.count):
    wr = gen.childOccurrences.item(i)
    mod = module_of(wr)
    if mod is None:
        continue
    mat = mod.transform.copy(); mat.transformBy(wr.transform)
    # every body of the arm that is NOT inside this module
    others = []
    def walk(o, skip):
        if o is skip:
            return
        for b in o.bRepBodies:
            others.append((o.name, b))
        for j in range(o.childOccurrences.count):
            walk(o.childOccurrences.item(j), skip)
    for j in range(gen.childOccurrences.count):
        walk(gen.childOccurrences.item(j), mod)
    for ang in (0.0, 90.0, 180.0, 270.0):
        cx, cy = 11.0*math.cos(math.radians(ang)), 11.0*math.sin(math.radians(ang))
        runs = []
        prev = None
        z = 0.0
        while z <= 14.0001:
            p = world(mat, cx, cy, z)
            hit = None
            for onm, ob in others:
                if ob.pointContainment(p) == af.PointContainment.PointInsidePointContainment:
                    hit = onm.split(':')[0]; break
            if hit != prev:
                runs.append([z, z, hit])
                prev = hit
            else:
                runs[-1][1] = z
            z += 0.1
        log('  %-22s %5.1f deg: %s' % (wr.component.name, ang,
            ' | '.join('%s z %.1f..%.1f' % (r[2] or 'air', r[0], r[1]) for r in runs)))
log('done')
