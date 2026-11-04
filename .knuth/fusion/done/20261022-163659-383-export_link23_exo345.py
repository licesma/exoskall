"""EXO-345: export parts/link_23.3mf (High refinement) from `exo generated`, body `g_link_23` of
`generated_R_arm_general`, in the ARM frame (as EXO-341's link_12). Logs the J2 (rotor side)
and J3 (stator side) module frames and probes the alignment mark in the J3 frame.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/export_link23_exo345.py --timeout 300
"""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
LINK_PATH = '/Users/esteban/repos/exo/parts/link_23.3mf'
doc = app.activeDocument
if doc.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r' % doc.name)
design = adsk.fusion.Design.cast(app.activeProduct)
root = design.rootComponent
INS = (adsk.fusion.PointContainment.PointInsidePointContainment, adsk.fusion.PointContainment.PointOnPointContainment)
def mat(m):
    a = m.asArray(); return [[a[4*i+j] for j in range(4)] for i in range(4)]
def occ_named(occs, name):
    for i in range(occs.count):
        if occs.item(i).component.name == name: return occs.item(i)
def occ_prefix(occs, pre):
    for i in range(occs.count):
        if occs.item(i).component.name.startswith(pre): return occs.item(i)
g = armgen
arm = occ_named(root.occurrences, 'generated_R_arm_general')
say('arm occ transform origin %s' % ['%.6f' % (mat(arm.transform2)[r][3]*10) for r in range(3)])
lo = occ_named(arm.childOccurrences, 'g_link_23')
say('g_link_23 occ in arm: origin %s rot diag %s' % (['%.6f' % (mat(lo.transform)[r][3]*10) for r in range(3)], ['%.6f' % mat(lo.transform)[r][r] for r in range(3)]))
lb = lo.component.bRepBodies.item(0)
say('g_link_23 bodies %d, B-rep V %.3f, faces %d' % (lo.component.bRepBodies.count, lb.volume*1000, lb.faces.count))
frames = {}
for lab, pre in (('J2', 'GJ2_'), ('J3', 'GJ3_')):
    o = occ_prefix(arm.childOccurrences, pre)
    M = mat(o.transform)
    O = [M[r][3]*10 for r in range(3)]; X = [M[r][0] for r in range(3)]; Y = [M[r][1] for r in range(3)]; Z = [M[r][2] for r in range(3)]
    frames[lab] = (O, X, Y, Z)
    say('%s %s frame in arm coords: O %s X %s Y %s Z %s' % (lab, o.component.name, ['%.4f' % v for v in O], ['%.6f' % v for v in X], ['%.6f' % v for v in Y], ['%.6f' % v for v in Z]))
def wp(lab, p):
    O, X, Y, Z = frames[lab]
    return adsk.core.Point3D.create(*[(O[r] + p[0]*X[r] + p[1]*Y[r] + p[2]*Z[r])/10 for r in range(3)])
a = math.radians(g.ALIGN_MARK_ANGLE)
def solid(lab, r, t, z, ang=a):
    x = r*math.cos(ang) - t*math.sin(ang); y = r*math.sin(ang) + t*math.cos(ang)
    return lb.pointContainment(wp(lab, (x, y, z))) in INS
for lab in ('J3',):
    say('%s mark, z -24.7 along r 23..29 t 0  : %s' % (lab, ''.join('#' if solid(lab, 23 + 0.1*k, 0, -24.7) else '.' for k in range(61))))
    say('%s mark, z -24.7 along r 23..29 t .5 : %s' % (lab, ''.join('#' if solid(lab, 23 + 0.1*k, 0.5, -24.7) else '.' for k in range(61))))
    say('%s mark, r 26, t -0.6..0.6 z -24.7   : %s' % (lab, ''.join('#' if solid(lab, 26, -0.6 + 0.05*k, -24.7) else '.' for k in range(25))))
    say('%s mark, r 26, t 0, z -25.2..-24.4   : %s' % (lab, ''.join('#' if solid(lab, 26, 0, -25.2 + 0.05*k) else '.' for k in range(17))))
    for deg in (0, 180, 270):
        say('%s at %d deg, r 26 z -24.7 (no mark expected): %s' % (lab, deg, ''.join('#' if solid(lab, 26, -0.6 + 0.05*k, -24.7, math.radians(deg)) else '.' for k in range(25))))
opts = design.exportManager.createC3MFExportOptions(lb, LINK_PATH)
opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
say('export link -> %s %s' % (design.exportManager.execute(opts), LINK_PATH))
