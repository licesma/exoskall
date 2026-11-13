"""EXO-417: build the G1 arm with a link actuator-opening VARIANT and a debossed pad label, for
export of link_12 only. NOT saved: `exo generated` stays at v33 (margin 0.2, no label); after the
exports close_reopen_exo417.py closes it without saving and reopens the saved version.
Same builder as run_buildarm_exo415.py; after the reload `armgen.ACT_OPEN_MARGIN` is set to
OPEN_M and the functions that bound it as a default argument (actuator_rect / _volume / _cut_z /
_box_dist / in_actuator_rect) get the same value, so the whole chain opens to one number
(armgen.py itself is not edited). The label (EXO-383 machinery, `ROTOR_LABEL_ON`, on the rotor pad's
outer face, UP on the bed in the links2 orientation) reads LABEL, the TOTAL clearance Esteban
names the variant by: "0.2" = 0.1 per side, "0.4" = 0.2 per side. The EXO-383 label uses the
default font, whose strokes are about 0.3 wide (7-15 % survive a 0.4 opening): here it is drawn
in Arial Black (EXO-317 / EXO-350 / EXO-352), same height 3.0, depth 0.4, place and r 8..18 guard.
Run through the wrappers run_open0.2_exo417.py / run_open0.4_exo417.py (they set OPEN_M, LABEL):
/usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/run_open0.2_exo417.py --timeout 900"""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, sweepgen, BuildArm, BuildArmGen
importlib.reload(armgen); importlib.reload(sweepgen); importlib.reload(BuildArm)
importlib.reload(BuildArmGen)
doc = app.activeDocument
if doc.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r, not %r' % (doc.name, BuildArm.GEN_DOC_NAME))
M, LAB = float(OPEN_M), str(LABEL)
D0 = armgen.ACT_OPEN_MARGIN
armgen.ACT_OPEN_MARGIN = M
for fn in (armgen.actuator_rect, armgen.actuator_volume, armgen.actuator_cut_z,
           armgen.actuator_box_dist, armgen.in_actuator_rect):
    d = list(fn.__defaults__)
    assert d[0] == D0, (fn.__name__, d)
    d[0] = M
    fn.__defaults__ = tuple(d)
armgen.ROTOR_LABEL_ON = True
armgen.rotor_label_text = lambda: LAB
def _label_arial_black(comp, body, FA, ex, ey, name):
    """BuildArmGen._rotor_label (EXO-375/383) verbatim except `ti.fontName = 'Arial Black'`."""
    g = armgen
    text = g.rotor_label_text()
    r, a = g.ROTOR_LABEL_AT
    cx, cy = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
    z_lo, z_hi = g.ROTOR_PAD_T - g.ROTOR_LABEL_D, g.ROTOR_PAD_T + 0.1
    sk = BuildArm.PlaneSketch(comp, name + ' pad label', FA.point([0, 0, z_lo]), ex, ey)
    n = sk.sketch.xDirection.crossProduct(sk.sketch.yDirection)
    ez = ex.crossProduct(ey)
    if n.dotProduct(ez) < 0.99:
        raise RuntimeError('%s: label sketch normal is not the pad normal (%.3f)' % (name, n.dotProduct(ez)))
    h = g.ROTOR_LABEL_H
    ti = sk.sketch.sketchTexts.createInput2(text, h * BuildArm.MM_TO_CM)
    ti.setAsMultiLine(sk.pt(cx - 4.0 * h / 2, cy - h), sk.pt(cx + 4.0 * h / 2, cy + h),
                      adsk.core.HorizontalAlignments.CenterHorizontalAlignment,
                      adsk.core.VerticalAlignments.MiddleVerticalAlignment, 0)
    ti.fontName = 'Arial Black'
    st = sk.sketch.sketchTexts.add(ti)
    if st.fontName != 'Arial Black':
        raise RuntimeError('%s: label font is %r' % (name, st.fontName))
    keep = set(comp.bRepBodies.item(i).entityToken for i in range(comp.bRepBodies.count))
    feats = comp.features.extrudeFeatures
    ein = feats.createInput(st, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ein.setDistanceExtent(False, adsk.core.ValueInput.createByReal((z_hi - z_lo) * BuildArm.MM_TO_CM))
    feats.add(ein)
    tools = adsk.core.ObjectCollection.create()
    for i in range(comp.bRepBodies.count):
        b = comp.bRepBodies.item(i)
        if b.entityToken in keep:
            continue
        for k in range(b.vertices.count):
            q = FA.local(b.vertices.item(k).geometry)
            if not (8.0 <= math.hypot(q[0], q[1]) <= 18.0
                                      and z_lo - 1e-3 <= q[2] <= z_hi + 1e-3):
                raise RuntimeError('%s: label glyph vertex at r %.3f z %.3f'
                                   % (name, math.hypot(q[0], q[1]), q[2]))
        tools.add(b)
    if tools.count == 0:
        raise RuntimeError('%s: the label made no glyph bodies' % name)
    cin = comp.features.combineFeatures.createInput(body, tools)
    cin.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
    cin.isKeepToolBodies = False
    comp.features.combineFeatures.add(cin)
    if comp.bRepBodies.count != 1:
        raise RuntimeError('%s: %d bodies after the label' % (name, comp.bRepBodies.count))
    sk.sketch.isVisible = False
    sk.cplane.isLightBulbOn = False
    return comp.bRepBodies.item(0)
BuildArmGen._rotor_label = _label_arial_black
log('DOCUMENT %r v%s isModified %s' % (doc.name, doc.dataFile.versionNumber, doc.isModified))
log('VARIANT: ACT_OPEN_MARGIN %.3f (default %.3f) -> opening %s, actuator_rect() %s; label %r'
    % (M, D0, armgen.actuator_rect(M), armgen.actuator_rect(), armgen.rotor_label_text()))
log(BuildArmGen.build_general(app, ui, design, log))
log('NOT SAVED (EXO-417 test variant); isModified %s' % doc.isModified)
