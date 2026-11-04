"""EXO-364 read-only probe: the index notch and the r 15 band of the rotor top (`inner_top` of
`sensor module 2 rotor` in `exo generated`), module frame. No write, no save.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/probe_notch_exo364.py --timeout 120"""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
des = adsk.fusion.Design.cast(app.activeProduct); root = des.rootComponent
occ = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name == BuildArm.MODULE_ROTOR_NAME][0]
up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
b = up.bRepBodies.item(0)
INS = (adsk.fusion.PointContainment.PointInsidePointContainment, adsk.fusion.PointContainment.PointOnPointContainment)
S = lambda x, y, z: b.pointContainment(adsk.core.Point3D.create(x/10, y/10, z/10)) in INS
say('upper V %.3f faces %d' % (b.volume*1000, b.faces.count))
for z in (-0.05, -1.45, -1.55):
    say('z %.2f  x -3..3 step .25' % z)
    for y in [18.2 - 0.1*k for k in range(42)]:
        say('  y %5.2f %s' % (y, ''.join('#' if S(-3 + 0.25*k, y, z) else '.' for k in range(25))))
def bis(f, a, b_, n=40):
    for _ in range(n):
        m = (a + b_)/2
        a, b_ = (m, b_) if f(m) else (a, m)
    return (a + b_)/2
say('notch floor at (0,16): z %.4f' % bis(lambda z: not S(0, 16, z), -0.01, -3))
say('notch x- at y16 z-0.5: %.4f  x+: %.4f' % (bis(lambda x: not S(x, 16, -0.5), 0, -4), bis(lambda x: not S(x, 16, -0.5), 0, 4)))
say('notch y- at x0 z-0.5: %.4f  y+: %.4f' % (bis(lambda y: not S(0, y, -0.5), 16, 12), bis(lambda y: not S(0, y, -0.5), 16, 20)))
say('notch y+ at x 1.9: %.4f' % bis(lambda y: not S(1.9, y, -0.5), 16, 20))
say('rim r at 0/45/135/225/315 deg z -0.5: %s' % ['%.4f' % bis(lambda r: S(r*math.cos(math.radians(a)), r*math.sin(math.radians(a)), -0.5), 14, 20) for a in (0, 45, 135, 225, 315)])
for a in (45, 135, 225, 315):
    say('a %d: r 12..18.5 at z -0.5: %s ; z -1.4: %s' % (a, ''.join('#' if S(r*math.cos(math.radians(a)), r*math.sin(math.radians(a)), -0.5) else '.' for r in [12 + 0.25*k for k in range(27)]),
        ''.join('#' if S(r*math.cos(math.radians(a)), r*math.sin(math.radians(a)), -1.4) else '.' for r in [12 + 0.25*k for k in range(27)])))
for i in range(b.faces.count):
    f = b.faces.item(i); bb = f.boundingBox
    if bb.maxPoint.z*10 > -0.01 and bb.minPoint.z*10 > -2 and max(abs(bb.minPoint.y), abs(bb.maxPoint.y))*10 > 13:
        say('face %d %s area %.3f bbox x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f' % (i, f.geometry.surfaceType, f.area*100, bb.minPoint.x*10, bb.maxPoint.x*10, bb.minPoint.y*10, bb.maxPoint.y*10, bb.minPoint.z*10, bb.maxPoint.z*10))
