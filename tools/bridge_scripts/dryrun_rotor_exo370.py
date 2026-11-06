"""EXO-370 dry run: the module recipe (`BuildArm.build_v3_copy`: final bottom, upper part deleted)
in a SCRATCH document - `sensor module 2` inserted LINKED, read-only - measured on `V3_PROBES`
plus bisected bottom numbers; the scratch is closed NOT saved and the previous document
re-activated. Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/dryrun_rotor_exo370.py --timeout 600"""
import importlib, sys, math
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
    log('FINAL PROBES %s (%d)' % ('ALL OK' if ok else 'WRONG', len(lines)))
    log('rename verdict: %s' % BuildArm.v3_rename_verdict(occ))
    up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
    b = lo.bRepBodies.item(0)
    log('upper %s; lower %s V %.3f faces %d' % (up, lo.component.name, b.volume * 1000, b.faces.count))
    INS = (adsk.fusion.PointContainment.PointInsidePointContainment, adsk.fusion.PointContainment.PointOnPointContainment)
    S = lambda x, y, z: b.pointContainment(adsk.core.Point3D.create(x/10, y/10, z/10)) in INS
    def bis(f, a, c, n=45):
        for _ in range(n):
            m = (a + c) / 2
            a, c = (m, c) if f(m) else (a, m)
        return (a + c) / 2
    log('boss top z %.5f (at r 4)' % bis(lambda z: S(4, 0, z), -7.5, -6.5))
    log('boss r %.5f at z -7.5 (0 deg), %.5f (100 deg)' % (bis(lambda r: S(r, 0, -7.5), 7.0, 8.5),
        bis(lambda r: S(r*math.cos(math.radians(100)), r*math.sin(math.radians(100)), -7.5), 7.0, 8.5)))
    log('joint face z %.5f (r 10 @67.5)' % bis(lambda z: S(10*math.cos(math.radians(67.5)), 10*math.sin(math.radians(67.5)), z), -8.5, -7.5))
    log('journal r %.5f at z -9.0' % bis(lambda r: S(r*math.cos(math.radians(202.5)), r*math.sin(math.radians(202.5)), -9.0), 14.5, 15.5))
    log('flange z %.5f at r 16.5' % bis(lambda z: S(16.5, 0, z), -11.5, -10.5))
    hx, hy = 6.001*math.cos(math.radians(45)), 6.001*math.sin(math.radians(45))
    hr = bis(lambda r: not S(hx + r*math.cos(math.radians(45)), hy + r*math.sin(math.radians(45)), -7.5), 0.5, 2.0)
    log('horn hole r %.5f -> outer edge r %.5f -> boss wall %.5f' % (hr, 6.001 + hr, 7.8 - 6.001 - hr))
    px, py = 12.2*math.cos(math.radians(112.5)), 12.2*math.sin(math.radians(112.5))
    log('socket floor z %.5f' % bis(lambda z: not S(px, py, z), -8.5, -10.0))
    log('socket r %.5f' % bis(lambda r: not S(px + r, py, -8.6), 1.0, 2.5))
    log('bore ceiling z %.5f' % bis(lambda z: not S(5, 0, z), -12.0, -10.0))
finally:
    d = app.activeDocument
    if doc is not None and d is not None and d.dataFile is None and d.name.startswith('Untitled'):
        log('closing scratch %r, NOT saved' % d.name); d.close(False)
    if prev is not None:
        prev.activate()
