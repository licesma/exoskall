import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen; importlib.reload(armgen)

root = design.rootComponent
arm = next(o for o in root.occurrences if o.component.name.startswith('generated_R_arm'))
l45 = None
for sub in arm.childOccurrences:
    if sub.component.name.startswith('link_45'):
        l45 = sub; break
if l45 is None:
    raise RuntimeError('link_45 not found')
body = l45.bRepBodies.item(0)
log('link_45 faces=%d vol=%.0f mm3' % (body.faces.count, body.volume*1000))

# the 4 rotor-pad holes: cylinders of radius HOLE_R at ROTOR_HOLE_RADIUS
hr = armgen.HOLE_R; rr = armgen.ROTOR_HOLE_RADIUS
log('holes: circle r=%.1f, hole r=%.1f -> outermost edge r=%.1f' % (rr, hr, rr+hr))

# find cylindrical faces of radius ~HOLE_R in the pad region and report their axial extent
found = []
for f in body.faces:
    g = f.geometry
    if g.objectType == 'adsk::core::Cylinder':
        r_mm = g.radius*10.0
        if abs(r_mm - hr) < 0.05:
            bb = f.boundingBox
            found.append((r_mm, bb.minPoint.z*10.0, bb.maxPoint.z*10.0))
log('hole cylinder faces found: %d' % len(found))
for r_mm, z0, z1 in found:
    log('  r=%.2f  z %.3f .. %.3f  (through-thickness = %.3f)' % (r_mm, z0, z1, z1-z0))
