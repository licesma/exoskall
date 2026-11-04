# Interference check over ALL bodies under generated_R_arm (links + modules, any depth),
# classified like BuildArm's post-build check (FAIL / sliver / module-internal). Logs each
# interference with its volume and the interference body's bbox in world and in the J2
# module frame (the J2_* wrapper's transform), so the overlap can be located.
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)

root = design.rootComponent
arm = None
for i in range(root.occurrences.count):
    occ = root.occurrences.item(i)
    if occ.component.name.startswith('generated_R_arm'):
        arm = occ; break
if arm is None:
    raise RuntimeError('generated_R_arm not found')

bodies, names = BuildArm.collect_bodies(arm)
log('bodies checked: %d' % bodies.count)
for i in range(bodies.count):
    b = bodies.item(i)
    bb = b.boundingBox
    log('  %-60s bbox mm x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f]' % (
        names[b.entityToken], bb.minPoint.x*10, bb.maxPoint.x*10, bb.minPoint.y*10, bb.maxPoint.y*10,
        bb.minPoint.z*10, bb.maxPoint.z*10))

# J2 wrapper world matrix (arm at identity, so wrapper.transform == world)
j2 = None
for i in range(arm.childOccurrences.count):
    o = arm.childOccurrences.item(i)
    if o.component.name.startswith('J2_'):
        j2 = o
m_j2 = j2.transform.copy() if j2 else None
inv = m_j2.copy(); inv.invert()

n_bodies, results, how = BuildArm.check_interference(design, arm)
log('INTERFERENCES: %d (bodies %d) - %s' % (len(results), n_bodies, how))
for kind, a, b, vol, bb in results:
    log('  [%s] %s  <->  %s   volume=%.2f mm3' % (kind, a, b, vol))
    if bb is not None:
        lo, hi = bb
        log('    bbox world mm x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]' % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
        if inv is not None:
            # corners -> J2 frame
            import itertools
            pts = []
            for cx, cy, cz in itertools.product((lo[0], hi[0]), (lo[1], hi[1]), (lo[2], hi[2])):
                p = adsk.core.Point3D.create(cx / 10.0, cy / 10.0, cz / 10.0)
                p.transformBy(inv)
                pts.append((p.x * 10, p.y * 10, p.z * 10))
            log('    bbox in J2 frame mm x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]' % (
                min(p[0] for p in pts), max(p[0] for p in pts), min(p[1] for p in pts), max(p[1] for p in pts),
                min(p[2] for p in pts), max(p[2] for p in pts)))
