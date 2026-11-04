"""EXO-341 probe b (read-only): J2 module frame in world, link_12 solid at +Y of J2 near the
stator underside, parked module stator: frame, volume, solid at +Y near its underside."""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
design = adsk.fusion.Design.cast(app.activeProduct)
root = design.rootComponent
INS = (adsk.fusion.PointContainment.PointInsidePointContainment, adsk.fusion.PointContainment.PointOnPointContainment)
def mat(m):
    a = m.asArray(); return [[a[4*i+j] for j in range(4)] for i in range(4)]
def occ_named(parent_occs, name):
    for i in range(parent_occs.count):
        if parent_occs.item(i).component.name == name: return parent_occs.item(i)
arm = occ_named(root.occurrences, 'generated_R_arm_general')
gj2 = occ_named(arm.childOccurrences, 'GJ2_right_shoulder_roll_joint')
def walk(o, d=0):
    M = mat(o.transform2 if hasattr(o, 'transform2') else o.transform)
    say('%s%r comp %r bodies %d origin %s X %s Y %s Z %s' % ('  '*d, o.name, o.component.name, o.component.bRepBodies.count,
        ['%.3f' % (M[r][3]*10) for r in range(3)], ['%.4f' % M[r][0] for r in range(3)], ['%.4f' % M[r][1] for r in range(3)], ['%.4f' % M[r][2] for r in range(3)]))
    if d < 2:
        for i in range(o.childOccurrences.count): walk(o.childOccurrences.item(i), d+1)
walk(gj2)
M = mat(gj2.transform2)
O = [M[r][3]*10 for r in range(3)]; X = [M[r][0] for r in range(3)]; Y = [M[r][1] for r in range(3)]; Z = [M[r][2] for r in range(3)]
def w(p):  # J2 module frame mm -> world cm
    return adsk.core.Point3D.create(*[(O[r] + p[0]*X[r] + p[1]*Y[r] + p[2]*Z[r])/10 for r in range(3)])
link = occ_named(arm.childOccurrences, 'g_link_12').component.bRepBodies.item(0)
for z in (-24.3, -24.6, -25.0, -28.5, -32.4, -32.7):
    row = ''.join('#' if link.pointContainment(w((0, y, z))) in INS else '.' for y in [20 + 0.25*k for k in range(61)])
    say('link J2 x=0 z %.1f  y 20..35: %s' % (z, row))
for a in (80, 85, 90, 95, 100):
    row = ''.join('#' if link.pointContainment(w((r*math.cos(math.radians(a)), r*math.sin(math.radians(a)), -24.7))) in INS else '.' for r in [20 + 0.25*k for k in range(41)])
    say('link J2 a %d z -24.7 r 20..30: %s' % (a, row))
# parked module stator
pm = occ_named(root.occurrences, 'sensor module 2 rotor')
say('parked module transform origin %s' % ['%.3f' % (mat(pm.transform2)[r][3]*10) for r in range(3)])
st = BuildArm.module_stator_occ(pm, in_context=True)
say('stator occ %r comp %r V %.3f' % (st.name, st.component.name, st.component.bRepBodies.item(0).volume*1000))
Ms = mat(st.transform2)
say('stator occ world transform origin %s Z %s' % (['%.3f' % (Ms[r][3]*10) for r in range(3)], ['%.4f' % Ms[r][2] for r in range(3)]))
sb = st.bRepBodies.item(0)
bb = sb.boundingBox
say('stator in-context bbox mm %s %s' % (['%.3f' % (v*10) for v in (bb.minPoint.x, bb.minPoint.y, bb.minPoint.z)], ['%.3f' % (v*10) for v in (bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z)]))
cb = st.component.bRepBodies.item(0).boundingBox
say('stator own-frame bbox mm %s %s' % (['%.3f' % (v*10) for v in (cb.minPoint.x, cb.minPoint.y, cb.minPoint.z)], ['%.3f' % (v*10) for v in (cb.maxPoint.x, cb.maxPoint.y, cb.maxPoint.z)]))
for z in (-24.3, -24.4, -24.6, -25.0):
    row = ''.join('#' if sb.pointContainment(adsk.core.Point3D.create(0, y/10, z/10)) in INS else '.' for y in [20 + 0.25*k for k in range(41)])
    say('stator x=0 z %.1f y 20..30: %s' % (z, row))
for a in (80, 85, 90, 95, 100):
    row = ''.join('#' if sb.pointContainment(adsk.core.Point3D.create(r*math.cos(math.radians(a))/10, r*math.sin(math.radians(a))/10, -24.3/10)) in INS else '.' for r in [20 + 0.25*k for k in range(41)])
    say('stator a %d z -24.3 r 20..30: %s' % (a, row))
