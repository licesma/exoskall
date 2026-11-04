"""EXO-352: two bottoms with the four M2 horn-screw head seats counterbored O5.0 into the joint
face, 0.5 and 1.0 deep, labelled "0.5" / "1.0" (EXO-315/327 deboss), exported at High to
parts/bottom_h0.5.3mf / bottom_h1.0.3mf, plus an unmodified copy to /tmp/exo352/ref_bottom.3mf
(compare with parts/bottom.3mf).
Source: the inner_bottom body of `sensor module 2 rotor` in `exo generated` (= parts/bottom.3mf),
READ ONLY: copied with TemporaryBRepManager into a new UNSAVED scratch document; nothing in
`exo generated` is edited or saved. Scratch closed NOT saved, `exo generated` re-activated.
Component frame = file frame: module z = own z - 7.5.
Run: python3 -I tools/fusion_run.py tools/bridge_scripts/build_head_exo352.py --timeout 300
"""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
OUT_DIR = '/Users/esteban/repos/exo/parts/'
REF = '/tmp/exo352/ref_bottom.3mf'
VARIANTS = ((0.5, '0.5'), (1.0, '1.0'))
JOINT = -7.0
HEAD_R = 2.5                      # O5.0
HORN_R, HORN_ANG = BuildArm.V3_HORN_R, BuildArm.V3_HORN_ANG
LABEL_H, LABEL_DEPTH, LABEL_FONT, LABEL_WALL = 2.5, 0.4, 'Arial Black', 0.4
P = adsk.core.Point3D.create
tbm = adsk.fusion.TemporaryBRepManager.get()
gen = app.activeDocument
if gen is None or gen.name != BuildArm.GEN_DOC_NAME:
    raise RuntimeError('active document is %r' % (gen.name if gen else None))
gen_mod0 = gen.isModified
gdes = adsk.fusion.Design.cast(gen.products.itemByProductType('DesignProductType'))
groot = gdes.rootComponent
occ = [groot.occurrences.item(i) for i in range(groot.occurrences.count) if groot.occurrences.item(i).component.name == BuildArm.MODULE_V3_NAME][0]
up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
src = lo.component.bRepBodies.item(0)
bb = src.boundingBox
DZ = 7.5
if abs(bb.minPoint.z * 10 - (-13.2 + DZ)) > 1e-3 or abs(bb.maxPoint.z * 10 - (JOINT + DZ)) > 1e-3:
    raise RuntimeError('source own z %.4f..%.4f, expected -5.7..0.5' % (bb.minPoint.z * 10, bb.maxPoint.z * 10))
V_SRC = src.volume * 1000
say('source %s / %s body %r V %.4f faces %d (gen modified before: %s)' % (occ.component.name, lo.component.name, src.name, V_SRC, src.faces.count, gen_mod0))
base = tbm.copy(src)
INS = (adsk.fusion.PointContainment.PointInsidePointContainment, adsk.fusion.PointContainment.PointOnPointContainment)
def solid(b, x, y, z):            # module mm
    return b.pointContainment(P(x / 10, y / 10, (z + DZ) / 10)) in INS
def bis(f, a, b):                 # f(a) False (void), f(b) True (solid)
    for _ in range(50):
        m = (a + b) / 2
        a, b = (a, m) if f(m) else (m, b)
    return (a + b) / 2

def deboss(comp, body, text, z_face):
    z_lo, z_hi = z_face - LABEL_DEPTH, z_face + 0.1
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal((z_lo + DZ) / 10))
    cp = comp.constructionPlanes.add(pin)
    sk = comp.sketches.add(cp)
    h = LABEL_H / 10
    inp = sk.sketchTexts.createInput2(text, h)
    w = 0.6
    inp.setAsMultiLine(P(-w / 2, -h, 0), P(w / 2, h, 0), adsk.core.HorizontalAlignments.CenterHorizontalAlignment,
                       adsk.core.VerticalAlignments.MiddleVerticalAlignment, 0)
    try:
        inp.fontName = LABEL_FONT
    except Exception:
        pass
    st = sk.sketchTexts.add(inp)
    feats = comp.features.extrudeFeatures
    ein = feats.createInput(st, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ein.setDistanceExtent(False, adsk.core.ValueInput.createByReal((z_hi - z_lo) / 10))
    feat = feats.add(ein)
    sk.isVisible = False
    tools = adsk.core.ObjectCollection.create()
    x0 = y0 = 1e9; x1 = y1 = -1e9
    for i in range(feat.bodies.count):
        b = feat.bodies.item(i); fb = b.boundingBox
        x0, x1 = min(x0, fb.minPoint.x * 10), max(x1, fb.maxPoint.x * 10)
        y0, y1 = min(y0, fb.minPoint.y * 10), max(y1, fb.maxPoint.y * 10)
        if abs(fb.minPoint.z * 10 - DZ - z_lo) > 0.02 or abs(fb.maxPoint.z * 10 - DZ - z_hi) > 0.02:
            raise RuntimeError('label z %.3f..%.3f' % (fb.minPoint.z * 10 - DZ, fb.maxPoint.z * 10 - DZ))
        tools.add(b)
    if tools.count == 0:
        raise RuntimeError('label made no bodies')
    walls = []
    for a in HORN_ANG:
        hx, hy = BuildArm._polar(HORN_R, a)
        dx = max(x0 - hx, 0.0, hx - x1); dy = max(y0 - hy, 0.0, hy - y1)
        walls.append(math.hypot(dx, dy) - HEAD_R)
    if min(walls) < LABEL_WALL:
        raise RuntimeError('label box leaves %.3f to a counterbore' % min(walls))
    cin = comp.features.combineFeatures.createInput(body, tools)
    cin.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
    cin.isKeepToolBodies = False
    comp.features.combineFeatures.add(cin)
    if comp.bRepBodies.count != 1:
        raise RuntimeError('label left %d bodies' % comp.bRepBodies.count)
    return (x0, x1, y0, y1), min(walls)

doc = None
try:
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.DirectDesignType
    em = des.exportManager
    def export(comp, path):
        b = comp.bRepBodies.item(0)
        opts = em.createC3MFExportOptions(b, path)
        opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        r = em.execute(opts)
        say('  export %s -> %s %s' % (comp.name, r, path))
    # reference: the copy untouched
    rocc = des.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    rocc.component.name = 'inner_bottom'
    rb = rocc.component.bRepBodies.add(tbm.copy(base))
    say('ref copy V %.4f (source %.4f) faces %d' % (rb.volume * 1000, V_SRC, rb.faces.count))
    export(rocc.component, REF)
    for depth, text in VARIANTS:
        name = 'bottom_h' + text
        floor = JOINT - depth
        body = tbm.copy(base)
        for a in HORN_ANG:
            x, y = BuildArm._polar(HORN_R, a)
            cyl = tbm.createCylinderOrCone(P(x / 10, y / 10, (floor + DZ) / 10), HEAD_R / 10,
                                           P(x / 10, y / 10, (JOINT + 0.1 + DZ) / 10), HEAD_R / 10)
            if not tbm.booleanOperation(body, cyl, adsk.fusion.BooleanTypes.DifferenceBooleanType):
                raise RuntimeError('counterbore %g failed' % a)
        o = des.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        comp = o.component; comp.name = name
        b = comp.bRepBodies.add(body)
        v_cb = b.volume * 1000
        box, lwall = deboss(comp, b, text, JOINT)
        b = comp.bRepBodies.item(0)
        v = b.volume * 1000
        say('%s: V %.4f; counterbores %.4f (expect %.4f), label %.4f; label box x %.3f..%.3f y %.3f..%.3f, wall to counterbore %.3f; faces %d'
            % (name, v, v_cb - V_SRC, -4 * math.pi * (HEAD_R ** 2 - 1.1 ** 2) * depth, v - v_cb, box[0], box[1], box[2], box[3], lwall, b.faces.count))
        # bisection, module frame
        for a in HORN_ANG:
            cx, cy = BuildArm._polar(HORN_R, a)
            def at(rr, ang, z):
                return solid(b, cx + rr * math.cos(math.radians(ang)), cy + rr * math.sin(math.radians(ang)), z)
            fz = bis(lambda z: at(1.8, a, z), JOINT + 0.5, JOINT - 3.0)
            rcb = [bis(lambda rr: at(rr, d, floor + depth / 2), 0.0, 3.5) for d in (0, 60, 120, 180, 240, 300)]
            rh = [bis(lambda rr: at(rr, d, floor - 0.5), 0.0, 2.0) for d in (0, 90, 180, 270)]
            rh2 = [bis(lambda rr: at(rr, d, -10.0), 0.0, 2.0) for d in (0, 90, 180, 270)]
            void_axis = all(not at(0.0, 0, z) for z in [JOINT - 0.05 - 0.25 * k for k in range(14)])
            say('  %3g deg: floor z %.5f, CB D %s, hole D %s at z %.2f, %s at -10.0, axis void -7.05..-10.3 %s'
                % (a, fz, '/'.join('%.4f' % (2 * r) for r in rcb), '/'.join('%.4f' % (2 * r) for r in rh), floor - 0.5,
                   '/'.join('%.4f' % (2 * r) for r in rh2), void_axis))
        # walls: counterbore -> nearest socket (r 12.2 @ 22.5) along the centre line, z mid-counterbore
        cx, cy = BuildArm._polar(HORN_R, 45.0); sx, sy = BuildArm._polar(BuildArm.V3_PEG_AT, 22.5)
        L = math.hypot(sx - cx, sy - cy); ux, uy = (sx - cx) / L, (sy - cy) / L
        zc = floor + depth / 2
        e1 = bis(lambda t: solid(b, cx + t * ux, cy + t * uy, zc), 0.0, L / 2)
        e2 = bis(lambda t: solid(b, cx + t * ux, cy + t * uy, zc), L, L / 2)
        say('  wall CB(45) -> socket(22.5) at z %.2f: %.4f (centres %.4f apart)' % (zc, e2 - e1, L))
        mx, my = BuildArm._polar(BuildArm.V3_M3_R, 0.0)
        L = math.hypot(mx - cx, my - cy); ux, uy = (mx - cx) / L, (my - cy) / L
        e1 = bis(lambda t: solid(b, cx + t * ux, cy + t * uy, zc), 0.0, L / 2)
        e2 = bis(lambda t: solid(b, cx + t * ux, cy + t * uy, zc), L, L / 2)
        say('  wall CB(45) -> M3(0) at z %.2f: %.4f' % (zc, e2 - e1))
        # axial wall to the bore under the outer part of the counterbore (r 8.0 from axis, 45 deg)
        for rr in (7.0, 8.0):
            px, py = BuildArm._polar(rr, 45.0)
            ztop = bis(lambda z: solid(b, px, py, z), JOINT + 0.5, JOINT - 2.0)
            zbot = bis(lambda z: solid(b, px, py, z), -12.0, ztop - 0.1)
            say('  axial wall at r %.1f 45 deg: CB floor %.5f -> bore ceiling %.5f = %.4f' % (rr, ztop, zbot, ztop - zbot))
        jf = bis(lambda z: solid(b, 0.0, 9.5, z), JOINT + 0.5, JOINT - 1.0)
        sf = bis(lambda z: solid(b, *BuildArm._polar(12.2, 22.5), z), JOINT + 0.5, -9.0)
        say('  joint face z (r 9.5, 90) %.5f, socket floor %.5f' % (jf, sf))
        export(comp, OUT_DIR + name + '.3mf')
finally:
    d = app.activeDocument
    if doc is not None and d is not None and d.dataFile is None and d.name.startswith('Untitled'):
        say('closing scratch %r, NOT saved' % d.name); d.close(False)
    gen.activate()
    say('active %r; exo generated modified before %s after %s; open %s' % (app.activeDocument.name, gen_mod0, gen.isModified,
        [app.documents.item(i).name for i in range(app.documents.count)]))
