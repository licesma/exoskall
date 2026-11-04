"""EXO-304: v4 rotor pair. New UNSAVED document, linked `sensor module 2` (inserted, read-only),
local copy `sensor module 2 v4` = v3 recipe + BuildArm.rotor_v4, three parts exported at High.
Never opens or saves the original or `sensor module 2 v3`. The scratch document is closed
without saving at the end and the previously active document re-activated.
OUT_DIR below: parts/ for the deliverables, /tmp for a cross-check rebuild."""
import importlib, sys
OUT_DIR = '/Users/esteban/repos/exo/parts/'   # or '/tmp/...re_' for a cross-check rebuild
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
_log = log
def log(m):                          # the bridge writes its log as ASCII (EXO-304: lost one on a U+00D8)
    _log(str(m).encode('ascii', 'backslashreplace').decode())
SRC_ID = 'urn:adsk.wipprod:dm.lineage:CzEATRrQSnuX-w_dwKCH6A'   # sensor module 2, Estebans / G1 exeskeleton / rotor
prev = app.activeDocument
log('prev active %r; open %s' % (prev.name if prev else None, [app.documents.item(i).name for i in range(app.documents.count)]))
doc = None
try:
    df = app.data.findFileById(SRC_ID)
    log('source %s v%s' % (df.name, df.versionNumber))
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.DirectDesignType   # PlaneSketch.setByPlane needs direct
    root = des.rootComponent
    link = root.occurrences.addByInsert(df, adsk.core.Matrix3D.create(), True)
    u0, l0 = BuildArm.module_rotor_occs(link, in_context=True)
    log('original upper V %.3f, lower V %.3f (linked, read-only)' % (u0.bRepBodies.item(0).volume*1000, l0.bRepBodies.item(0).volume*1000))
    comp, occ, b_occ, how = BuildArm.build_v4_copy(des, log)
    log(how)
    up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
    em = des.exportManager
    for o, name in ((up, 'top_half'), (lo, 'bottom_3.5mm'), (b_occ, 'bottom_4mm')):
        b = o.component.bRepBodies.item(0)
        bb = o.bRepBodies.item(0).boundingBox
        path = OUT_DIR + name + '.3mf'
        opts = em.createC3MFExportOptions(b, path)
        opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        log('%s V %.3f module z %.4f..%.4f -> %s %s' % (o.component.name, b.volume*1000,
            bb.minPoint.z*10, bb.maxPoint.z*10, em.execute(opts), path))
    log('link still referenced: %s' % link.isReferencedComponent)
finally:
    d = app.activeDocument
    if doc is not None and d is not None and d.dataFile is None and d.name.startswith('Untitled'):
        log('closing scratch %r, NOT saved' % d.name); d.close(False)
    if prev is not None:
        prev.activate()
    log('open after: %s' % [app.documents.item(i).name for i in range(app.documents.count)])
