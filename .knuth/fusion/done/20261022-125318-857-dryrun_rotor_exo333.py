"""EXO-333 dry run: the final module rotor recipe (`BuildArm.build_v3_copy`) in a SCRATCH
document - `sensor module 2` inserted LINKED, read-only - measured on `V3_PROBES`, then the
scratch is closed NOT saved and the previous document re-activated. Nothing else is opened."""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
_log = log
def log(m):
    _log(str(m).encode('ascii', 'backslashreplace').decode())
SRC_ID = 'urn:adsk.wipprod:dm.lineage:CzEATRrQSnuX-w_dwKCH6A'
prev = app.activeDocument
doc = None
try:
    df = app.data.findFileById(SRC_ID)
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.DirectDesignType
    link = des.rootComponent.occurrences.addByInsert(df, adsk.core.Matrix3D.create(), True)
    comp, occ, how = BuildArm.build_v3_copy(des, 'dryrun rotor', log)
    log(how)
    ok, lines = BuildArm.measure_rotor_v3(occ)
    for l in lines:
        log(l)
    log('FINAL PROBES %s (%d)' % ('ALL OK' if ok else 'WRONG', len(BuildArm.V3_PROBES)))
    up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
    for o in (up, lo):
        log('%s V %.3f' % (o.component.name, o.bRepBodies.item(0).volume * 1000))
    nut, bolt = BuildArm.module_v3_fasteners(occ)
    log('fasteners nuts %s bolts %s' % (nut is not None, bolt is not None))
finally:
    d = app.activeDocument
    if doc is not None and d is not None and d.dataFile is None and d.name.startswith('Untitled'):
        log('closing scratch %r, NOT saved' % d.name); d.close(False)
    if prev is not None:
        prev.activate()
