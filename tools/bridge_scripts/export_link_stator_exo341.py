"""EXO-341: export parts/link_12.3mf and parts/stator.3mf (High refinement) from `exo generated`.

link_12: body `g_link_12` of `generated_R_arm_general` as built (the generator now cuts the
EXO-341 assembly mark, `BuildArmGen.build_link`). Exported in its component frame = the ARM
frame; the J1 / J2 module frames are logged so the file can be read in either.

stator: the EXPORT COPY. `exo generated` is a DIRECT design (no timeline), so the copy is a
plain body: the stator of the parked `sensor module 2 rotor` (never edited) is copied with
TemporaryBRepManager into a root component `STATOR_EXPORT_NAME` (made fresh each run, light
bulb off), in the stator's own component frame (module z = file z - 7.5, as the old
parts/stator.3mf), and the mark box is subtracted from that copy only. The module the arms
place keeps its stator unmarked. The document is saved at the end (EXO-242).
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/export_link_stator_exo341.py --timeout 300
"""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
STATOR_EXPORT_NAME = 'stator EXO-341 export'
LINK_PATH = '/Users/esteban/repos/exo/parts/link_12.3mf'
STATOR_PATH = '/Users/esteban/repos/exo/parts/stator.3mf'
doc = app.activeDocument
if doc.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r' % doc.name)
design = adsk.fusion.Design.cast(app.activeProduct)
say('designType %s (0 = direct, no timeline)' % design.designType)
root = design.rootComponent
INS = (adsk.fusion.PointContainment.PointInsidePointContainment, adsk.fusion.PointContainment.PointOnPointContainment)
def mat(m):
    a = m.asArray(); return [[a[4*i+j] for j in range(4)] for i in range(4)]
def occ_named(occs, name):
    for i in range(occs.count):
        if occs.item(i).component.name == name: return occs.item(i)
g = armgen
em = design.exportManager
# ---------- link ----------
arm = occ_named(root.occurrences, 'generated_R_arm_general')
say('arm occ transform origin %s' % ['%.6f' % (mat(arm.transform2)[r][3]*10) for r in range(3)])
lo = occ_named(arm.childOccurrences, 'g_link_12')
say('g_link_12 occ in arm: origin %s (identity expected)' % ['%.6f' % (mat(lo.transform)[r][3]*10) for r in range(3)])
lb = lo.component.bRepBodies.item(0)
say('g_link_12 B-rep V %.3f, faces %d' % (lb.volume*1000, lb.faces.count))
frames = {}
for lab, nm in (('J1', 'GJ1_right_shoulder_pitch_joint'), ('J2', 'GJ2_right_shoulder_roll_joint')):
    M = mat(occ_named(arm.childOccurrences, nm).transform)
    O = [M[r][3]*10 for r in range(3)]; X = [M[r][0] for r in range(3)]; Y = [M[r][1] for r in range(3)]; Z = [M[r][2] for r in range(3)]
    frames[lab] = (O, X, Y, Z)
    say('%s frame in arm/file coords: O %s X %s Y %s Z %s' % (lab, ['%.4f' % v for v in O], ['%.6f' % v for v in X], ['%.6f' % v for v in Y], ['%.6f' % v for v in Z]))
def wp(lab, p):
    O, X, Y, Z = frames[lab]
    return adsk.core.Point3D.create(*[(O[r] + p[0]*X[r] + p[1]*Y[r] + p[2]*Z[r])/10 for r in range(3)])
# the mark on the link, probed in the J2 frame
a = math.radians(g.ALIGN_MARK_ANGLE)
def solid_link(r, t, z):   # r radial along the mark, t tangential offset
    x = r*math.cos(a) - t*math.sin(a); y = r*math.sin(a) + t*math.cos(a)
    return lb.pointContainment(wp('J2', (x, y, z))) in INS
say('link mark, J2 frame, z -24.7 along r 23..29 at t 0 : %s' % ''.join('#' if solid_link(23 + 0.1*k, 0, -24.7) else '.' for k in range(61)))
say('link mark, J2 frame, z -24.7 along r 23..29 at t .5: %s' % ''.join('#' if solid_link(23 + 0.1*k, 0.5, -24.7) else '.' for k in range(61)))
say('link mark, J2 frame, r 26, t -0.6..0.6 at z -24.7: %s' % ''.join('#' if solid_link(26, -0.6 + 0.05*k, -24.7) else '.' for k in range(25)))
say('link mark, J2 frame, r 26, t 0, z -25.2..-24.4: %s' % ''.join('#' if solid_link(26, 0, -25.2 + 0.05*k) else '.' for k in range(17)))
opts = em.createC3MFExportOptions(lb, LINK_PATH)
opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
say('export link -> %s %s' % (em.execute(opts), LINK_PATH))
# ---------- stator export copy ----------
for i in reversed(range(root.occurrences.count)):
    if root.occurrences.item(i).component.name.startswith(STATOR_EXPORT_NAME):
        root.occurrences.item(i).deleteMe(); say('deleted the previous %r' % STATOR_EXPORT_NAME)
pm = occ_named(root.occurrences, BuildArm.MODULE_V3_NAME)
st = BuildArm.module_stator_occ(pm, in_context=True)
src = st.component.bRepBodies.item(0)
Ms = mat(st.transform2)
dz = Ms[2][3]*10
say('module stator %r comp %r V %.3f, own frame -> module: z %+.3f' % (st.name, st.component.name, src.volume*1000, dz))
tbm = adsk.fusion.TemporaryBRepManager.get()
tmp = tbm.copy(src)
zf = g.STATOR_UNDERSIDE_Z - dz          # underside in the stator's own frame
z0, z1 = zf - 0.1, zf + g.ALIGN_MARK_D  # 0.1 below the face: not coplanar
rm = (g.ALIGN_MARK_R_IN + g.ALIGN_MARK_R_OUT) / 2
c = adsk.core.Point3D.create(rm*math.cos(a)/10, rm*math.sin(a)/10, (z0 + z1)/20)
box = adsk.core.OrientedBoundingBox3D.create(c, adsk.core.Vector3D.create(math.cos(a), math.sin(a), 0),
        adsk.core.Vector3D.create(-math.sin(a), math.cos(a), 0),
        (g.ALIGN_MARK_R_OUT - g.ALIGN_MARK_R_IN)/10, g.ALIGN_MARK_W/10, (z1 - z0)/10)
tool = tbm.createBox(box)
tbm.booleanOperation(tmp, tool, adsk.fusion.BooleanTypes.DifferenceBooleanType)
newocc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
newocc.component.name = STATOR_EXPORT_NAME
sb = newocc.component.bRepBodies.add(tmp)
sb.name = 'stator'
newocc.isLightBulbOn = False
say('export copy %r: comp %r, body V %.3f (module stator %.3f, removed %.4f mm3), lumps %d' % (
    newocc.name, newocc.component.name, sb.volume*1000, src.volume*1000, (src.volume - sb.volume)*1000, sb.lumps.count))
def solid_st(r, t, z):
    x = r*math.cos(a) - t*math.sin(a); y = r*math.sin(a) + t*math.cos(a)
    return sb.pointContainment(adsk.core.Point3D.create(x/10, y/10, z/10)) in INS
say('stator mark, own frame, z %.2f along r 23..29 at t 0 : %s' % (zf + 0.2, ''.join('#' if solid_st(23 + 0.1*k, 0, zf + 0.2) else '.' for k in range(61))))
say('stator mark, own frame, z %.2f along r 23..29 at t .5: %s' % (zf + 0.2, ''.join('#' if solid_st(23 + 0.1*k, 0.5, zf + 0.2) else '.' for k in range(61))))
say('stator mark, r 26, t -0.6..0.6: %s' % ''.join('#' if solid_st(26, -0.6 + 0.05*k, zf + 0.2) else '.' for k in range(25)))
say('stator mark, r 26, t 0, z %.2f..%.2f: %s' % (zf - 0.2, zf + 0.6, ''.join('#' if solid_st(26, 0, zf - 0.2 + 0.05*k) else '.' for k in range(17))))
opts = em.createC3MFExportOptions(sb, STATOR_PATH)
opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
say('export stator -> %s %s' % (em.execute(opts), STATOR_PATH))
say('module stator after: V %.3f (untouched)' % (st.component.bRepBodies.item(0).volume*1000))
say(BuildArm.save_active_document(app))
