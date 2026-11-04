"""EXO-364 dry run: the module rotor recipe (`BuildArm.build_v3_copy`, now with the notch fill and
the three key sockets) in a SCRATCH document - `sensor module 2` inserted LINKED, read-only -
measured on `V3_PROBES` and the key report; the scratch is closed NOT saved and the previous
document re-activated. Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/dryrun_rotor_exo364.py --timeout 600"""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
_log = log
def log(m):
    _log(str(m).encode('ascii', 'backslashreplace').decode())
exec(open('/Users/esteban/repos/exo/tools/bridge_scripts/key_measure_exo364.py').read())
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
        log('%s V %.3f faces %d' % (o.component.name, o.bRepBodies.item(0).volume * 1000, o.bRepBodies.item(0).faces.count))
    b = up.bRepBodies.item(0)
    INS = (adsk.fusion.PointContainment.PointInsidePointContainment, adsk.fusion.PointContainment.PointOnPointContainment)
    S = lambda x, y, z: b.pointContainment(adsk.core.Point3D.create(x/10, y/10, z/10)) in INS
    key_report(S, log)
finally:
    d = app.activeDocument
    if doc is not None and d is not None and d.dataFile is None and d.name.startswith('Untitled'):
        log('closing scratch %r, NOT saved' % d.name); d.close(False)
    if prev is not None:
        prev.activate()
