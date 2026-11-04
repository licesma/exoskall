"""EXO-350 / EXO-355: stator test prints, bearing seat O42.2 .. 42.6 in 0.1 steps (was O42.10, r 21.05).

Source: the plain body of root component `stator EXO-341 export` in `exo generated` (direct
design; never edited here). Frame = file frame of parts/stator.3mf (module z = file z - 7.5).
Measured seat (probe_exo350.py): cylinder r 21.05 over file z -3.5 .. 3.5 (module -11 .. -4),
shoulder face z -3.5 (r 19.25 .. 21.05) below it, top face z 3.5, stop block above (to 7.1).
Per diameter: TemporaryBRep copy, minus a cylinder r R over z -3.5 .. 3.5 only (shoulder and
stop block untouched), into a fresh root component `stator d42.x export` (light bulb off),
then the label "42.x" debossed 0.4 into the top face z 3.5 at -Y (r ~24.2, clear of the stop
block at 160-176 deg and of the rotor r <= 18). Exported High to parts/stator_d42.x.3mf.
Document saved (EXO-242).
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/build_stator_dia_exo350.py --timeout 600
"""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
SRC_NAME = 'stator EXO-341 export'
PREFIX = 'stator d42.'
# EXO-355 (2026-11-03): the set is 42.2 .. 42.6 in 0.1 steps. 42.3 / 42.5 (EXO-350) are kept as
# they are; this run builds 42.2 / 42.4 / 42.6 and deletes only 42.7 / 42.9 (and any previous
# copy of the three it builds). EXO-350 ran DIAS = 42.3/21.15, 42.5/21.25, 42.7/21.35, 42.9/21.45.
# EXO-355 ran DIAS = 42.2/21.10, 42.4/21.20, 42.6/21.30 with DROP = 42.7, 42.9.
# EXO-358 (2026-11-03): 41.8 .. 42.1 added (nothing dropped). R < 21.05 ADDS material: a ring
# r R .. 21.10 over z -3.5..3.5 (0.05 into the existing wall, so no coincident cylinder) is
# Boolean-unioned; R = 21.05 (42.1) = the source seat unchanged, label only; R > 21.05 cuts.
DIAS = (('41.8', 20.90), ('41.9', 20.95), ('42.0', 21.00), ('42.1', 21.05))
DROP = ()
SEAT_Z0, SEAT_Z1 = -3.5, 3.5        # file z, measured
R0 = 21.05
LABEL_H, LABEL_D, LABEL_YC, LABEL_FONT = 2.5, 0.4, -24.2, 'Arial Black'
OUT = '/Users/esteban/repos/exo/parts/stator_d%s.3mf'
doc = app.activeDocument
if doc.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r' % doc.name)
design = adsk.fusion.Design.cast(app.activeProduct)
root = design.rootComponent
INS = (adsk.fusion.PointContainment.PointInsidePointContainment, adsk.fusion.PointContainment.PointOnPointContainment)
P = lambda x, y, z: adsk.core.Point3D.create(x/10, y/10, z/10)
def occ_named(name):
    for i in range(root.occurrences.count):
        if root.occurrences.item(i).component.name == name: return root.occurrences.item(i)
src_occ = occ_named(SRC_NAME)
src = src_occ.component.bRepBodies.item(0)
V_src = src.volume*1000
other = {n: occ_named(n).component for n in ('sensor module 2', 'sensor module 2 rotor')}
def vols(c): return sum(c.bRepBodies.item(i).volume for i in range(c.bRepBodies.count))*1000
other_v = {n: vols(c) for n, c in other.items()}
mod_st = BuildArm.module_stator_occ(occ_named('sensor module 2 rotor'), in_context=True).component.bRepBodies.item(0).volume*1000
say('source %r V %.4f; sensor module 2 rotor stator V %.4f' % (SRC_NAME, V_src, mod_st))
for i in reversed(range(root.occurrences.count)):
    if root.occurrences.item(i).component.name in ['stator d%s export' % l for l in DROP + tuple(d[0] for d in DIAS)]:
        say('deleting previous %r' % root.occurrences.item(i).component.name); root.occurrences.item(i).deleteMe()
def radius_at(b, ang, z, lo=19.0, hi=23.0):
    """bisected inner wall radius: void at lo side, solid at hi side"""
    a = math.radians(ang)
    for _ in range(40):
        m = (lo + hi)/2
        if b.pointContainment(P(m*math.cos(a), m*math.sin(a), z)) in INS: hi = m
        else: lo = m
    return (lo + hi)/2
def row(b, ang, z):
    a = math.radians(ang)
    return ''.join('#' if b.pointContainment(P(r*math.cos(a), r*math.sin(a), z)) in INS else '.' for r in [15 + 0.25*k for k in range(53)])
say('source seat r at z 0 (0/90/168/270 deg): %s' % ['%.4f' % radius_at(src, a, 0.0) for a in (0, 90, 168, 270)])
for z in (-5.0, -3.6, -3.4, 0, 3.4, 3.6):
    say('source a 90 z %+5.2f r 15..28: %s' % (z, row(src, 90, z)))
tbm = adsk.fusion.TemporaryBRepManager.get()
em = design.exportManager
for lab, R in DIAS:
    tmp = tbm.copy(src)
    if R > R0 + 1e-9:     # enlarge: cut the cylinder r R
        cyl = tbm.createCylinderOrCone(P(0, 0, SEAT_Z0), R/10, P(0, 0, SEAT_Z1), R/10)
        ring = tbm.copy(src)
        tbm.booleanOperation(ring, tbm.copy(cyl), adsk.fusion.BooleanTypes.IntersectionBooleanType)
        v_ring = ring.volume*1000
        tbm.booleanOperation(tmp, cyl, adsk.fusion.BooleanTypes.DifferenceBooleanType)
    elif R < R0 - 1e-9:   # shrink: union a ring r R .. R0 + 0.05
        ring = tbm.createCylinderOrCone(P(0, 0, SEAT_Z0), (R0 + 0.05)/10, P(0, 0, SEAT_Z1), (R0 + 0.05)/10)
        tbm.booleanOperation(ring, tbm.createCylinderOrCone(P(0, 0, SEAT_Z0 - 0.1), R/10, P(0, 0, SEAT_Z1 + 0.1), R/10),
                             adsk.fusion.BooleanTypes.DifferenceBooleanType)
        tbm.booleanOperation(tmp, ring, adsk.fusion.BooleanTypes.UnionBooleanType)
        v_ring = -(tmp.volume - src.volume)*1000     # negative = added
        say('%s ring r %.2f..%.2f z %.1f..%.1f unioned: added %.4f (pi(R0^2-R^2)*7 = %.4f)'
            % (lab, R, R0 + 0.05, SEAT_Z0, SEAT_Z1, -v_ring, math.pi*(R0*R0 - R*R)*7))
    else:
        v_ring = 0.0
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = 'stator d%s export' % lab
    body = comp.bRepBodies.add(tmp)
    body.name = 'stator'
    v_cut = body.volume*1000
    # label
    pin = comp.constructionPlanes.createInput()
    zl = SEAT_Z1 - LABEL_D
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(zl/10))
    cp = comp.constructionPlanes.add(pin)
    sk = comp.sketches.add(cp)
    h = LABEL_H/10
    ti = sk.sketchTexts.createInput2(lab, h)
    ti.setAsMultiLine(P(-5, LABEL_YC - LABEL_H, 0), P(5, LABEL_YC + LABEL_H, 0),
                      adsk.core.HorizontalAlignments.CenterHorizontalAlignment,
                      adsk.core.VerticalAlignments.MiddleVerticalAlignment, 0)
    try: ti.fontName = LABEL_FONT
    except Exception: pass
    st = sk.sketchTexts.add(ti)
    feats = comp.features.extrudeFeatures
    ein = feats.createInput(st, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ein.setDistanceExtent(False, adsk.core.ValueInput.createByReal((LABEL_D + 0.1)/10))
    feat = feats.add(ein)
    tools = adsk.core.ObjectCollection.create()
    x0 = y0 = z0 = 1e9; x1 = y1 = z1 = -1e9
    # direct design: feat.bodies is not only the new bodies; take every body but the stator
    glyphs = [comp.bRepBodies.item(i) for i in range(comp.bRepBodies.count) if comp.bRepBodies.item(i).name != 'stator']
    for gb in glyphs:
        bb = gb.boundingBox
        x0, x1 = min(x0, bb.minPoint.x*10), max(x1, bb.maxPoint.x*10)
        y0, y1 = min(y0, bb.minPoint.y*10), max(y1, bb.maxPoint.y*10)
        z0, z1 = min(z0, bb.minPoint.z*10), max(z1, bb.maxPoint.z*10)
        tools.add(gb)
    rin = abs(y1) if x0 < 0 < x1 else min(math.hypot(x, y1) for x in (x0, x1))
    rout = max(math.hypot(x, y0) for x in (x0, x1))
    say('%s label tools %d: x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f; inner edge r %.3f (seat %.2f, wall %.3f), outer corner r %.3f (rim 27, wall %.3f), angles %.1f..%.1f deg'
        % (lab, tools.count, x0, x1, y0, y1, z0, z1, rin, R, rin - R, rout, 27 - rout,
           math.degrees(math.atan2(y0, x1)) % 360, math.degrees(math.atan2(y0, x0)) % 360))
    if tools.count == 0 or rin - R < 0.8 or 27 - rout < 0.8 or abs(z0 - zl) > 1e-3 or abs(z1 - SEAT_Z1 - 0.1) > 1e-3:
        raise RuntimeError('label placement')
    cin = comp.features.combineFeatures.createInput(body, tools)
    cin.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
    cin.isKeepToolBodies = False
    comp.features.combineFeatures.add(cin)
    if comp.bRepBodies.count != 1: raise RuntimeError('%d bodies' % comp.bRepBodies.count)
    body = comp.bRepBodies.item(0); body.name = 'stator'
    sk.deleteMe(); cp.deleteMe()
    v = body.volume*1000
    occ.isLightBulbOn = False
    say('%s comp %r: V %.4f = src %.4f - ring %.4f (cut %.4f) - label %.4f; lumps %d shells %d faces %d'
        % (lab, comp.name, v, V_src, v_ring, V_src - v_cut, v_cut - v, body.lumps.count, body.shells.count, body.faces.count))
    rs = [radius_at(body, a, z) for a in (0, 45, 90, 135, 168, 225, 270, 315) for z in (-3.4, 0.0, 3.4)]
    say('%s seat r over 8 angles x z -3.4/0/3.4: min %.5f max %.5f (asked %.5f) -> O%.4f' % (lab, min(rs), max(rs), R, 2*sum(rs)/len(rs)))
    say('%s shoulder r at z -3.6 (0/90/180/270): %s ; stop block inner r at z 5 (168 deg): %.4f' % (lab,
        ['%.4f' % radius_at(body, a, -3.6, 15.0, 23.0) for a in (0, 90, 180, 270)], radius_at(body, 168, 5.0)))
    for z in (-3.6, -3.4, 3.4, 3.6):
        say('%s a 45 z %+5.2f: %s' % (lab, z, row(body, 45, z)))
    for i in range(body.faces.count):
        f = body.faces.item(i); g = f.geometry; fb = f.boundingBox
        if g.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType and math.hypot(g.origin.x, g.origin.y) < 1e-6 and g.radius*10 > 20:
            say('%s   axis cyl r %.4f z %.4f..%.4f' % (lab, g.radius*10, fb.minPoint.z*10, fb.maxPoint.z*10))
    # label read on the solid: row at z 3.3 across the label
    for y in (LABEL_YC + 0.6, LABEL_YC, LABEL_YC - 0.6):
        say('%s label z 3.3 y %.1f x -5..5: %s' % (lab, y, ''.join('#' if body.pointContainment(P(-5 + 0.1*k, y, 3.3)) in INS else '.' for k in range(101))))
    path = OUT % lab
    opts = em.createC3MFExportOptions(body, path)
    opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    say('%s export -> %s %s' % (lab, em.execute(opts), path))
say('source after: V %.4f (before %.4f), bulb %s' % (src_occ.component.bRepBodies.item(0).volume*1000, V_src, src_occ.isLightBulbOn))
for n, c in other.items():
    say('%r V after %.4f (before %.4f)' % (n, vols(c), other_v[n]))
say(BuildArm.save_active_document(app))
d = app.activeDocument
say('isModified %s v%s' % (d.isModified, d.dataFile.versionNumber))
