"""EXO-315 / EXO-320: four tops with race gaps GAPS (6.9..6.6 or 6.5..6.2) + the reference top_3.5mm (to /tmp only).
Same discipline as EXO-304: new UNSAVED scratch document, `sensor module 2` inserted LINKED and
read-only, local copy, export, scratch closed NOT saved, previous document re-activated."""
import importlib, sys
OUT_DIR = '/Users/esteban/repos/exo/parts/'
GAPS = 'V323_GAPS'                   # 'V315_GAPS' (EXO-315: 6.9..6.6), 'V320_GAPS' (EXO-320: 6.5..6.2), 'V323_GAPS' (EXO-323: 6.1, 6.0)
REF_DIR = '/tmp/exo323_ref_'         # top_3.5mm rebuilt here, compared with parts/top_3.5mm.3mf
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
_log = log
def log(m):
    _log(str(m).encode('ascii', 'backslashreplace').decode())
SRC_ID = 'urn:adsk.wipprod:dm.lineage:CzEATRrQSnuX-w_dwKCH6A'
prev = app.activeDocument
log('prev active %r; open %s' % (prev.name if prev else None, [app.documents.item(i).name for i in range(app.documents.count)]))
doc = None
try:
    df = app.data.findFileById(SRC_ID)
    log('source %s v%s' % (df.name, df.versionNumber))
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.DirectDesignType
    link = des.rootComponent.occurrences.addByInsert(df, adsk.core.Matrix3D.create(), True)
    u0, l0 = BuildArm.module_rotor_occs(link, in_context=True)
    log('original upper V %.3f, lower V %.3f (linked, read-only)' % (u0.bRepBodies.item(0).volume*1000, l0.bRepBodies.item(0).volume*1000))
    em = des.exportManager
    for name, o, gap in BuildArm.build_gap_tops(des, log, getattr(BuildArm, GAPS)):
        b = o.component.bRepBodies.item(0)
        bb = o.bRepBodies.item(0).boundingBox
        path = (REF_DIR if gap is None else OUT_DIR) + name + '.3mf'
        opts = em.createC3MFExportOptions(b, path)
        opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        log('%s gap %s V %.3f module z %.4f..%.4f -> %s %s' % (name, gap, b.volume*1000,
            bb.minPoint.z*10, bb.maxPoint.z*10, em.execute(opts), path))
    log('link still referenced: %s' % link.isReferencedComponent)
finally:
    d = app.activeDocument
    if doc is not None and d is not None and d.dataFile is None and d.name.startswith('Untitled'):
        log('closing scratch %r, NOT saved' % d.name); d.close(False)
    if prev is not None:
        prev.activate()
    log('open after: %s' % [app.documents.item(i).name for i in range(app.documents.count)])
