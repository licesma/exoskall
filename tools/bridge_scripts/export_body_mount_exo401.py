"""EXO-401: export the chest mount (`arm complete_chest mount`, Fusion `/body mount`) Body1 to
parts/body_mount.3mf at High refinement, in the doc frame (= arm frame, EXO-400). Opens the doc
read-only if it is not open, never saves, closes without saving only if this script opened it.
Logs body volume, bbox and the tilted O4.0 seat holes (r 2.0 cylinders off the X/Y/Z axes).
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/export_body_mount_exo401.py --timeout 300"""
CID = 'urn:adsk.wipprod:dm.lineage:lvB_fi9qT02ITHaBJZkTrw'
OUT = '/Users/esteban/repos/exo/parts/body_mount.3mf'
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
was_open = None
for i in range(app.documents.count):
    d = app.documents.item(i)
    try:
        if d.dataFile is not None and d.dataFile.id == CID: was_open = d
    except Exception: pass
if was_open is None:
    df = app.data.findFileById(CID)
    say('data file %r v%s latest v%s folder %r' % (df.name, df.versionNumber, df.latestVersionNumber, df.parentFolder.name))
    doc = app.documents.open(df, True)
else:
    doc = was_open
    say('doc was already open: %r v%s' % (doc.name, doc.dataFile.versionNumber))
des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
say('doc %r isModified=%s comps=%d' % (doc.name, doc.isModified, des.allComponents.count))
root = des.rootComponent
bodies = list(root.bRepBodies) + [b for o in root.allOccurrences for b in o.bRepBodies]
say('bodies: %d' % len(bodies))
if len(bodies) != 1: raise RuntimeError('expected exactly one body, got %d' % len(bodies))
b = bodies[0]
bb = b.boundingBox
say('BODY %r solid=%s V=%.3f mm3 area=%.3f mm2 faces=%d bbox (%.3f,%.3f,%.3f)..(%.3f,%.3f,%.3f)' % (
    b.name, b.isSolid, b.volume*1000, b.area*100, b.faces.count,
    bb.minPoint.x*10, bb.minPoint.y*10, bb.minPoint.z*10, bb.maxPoint.x*10, bb.maxPoint.y*10, bb.maxPoint.z*10))
for f in b.faces:
    g = f.geometry
    if g.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType and abs(g.radius*10 - 2.0) < 1e-3:
        ax = g.axis
        if max(abs(ax.x), abs(ax.y), abs(ax.z)) < 0.999:
            say('SEAT HOLE r %.4f origin (%.4f,%.4f,%.4f) axis (%.5f,%.5f,%.5f)' % (
                g.radius*10, g.origin.x*10, g.origin.y*10, g.origin.z*10, ax.x, ax.y, ax.z))
em = des.exportManager
o = em.createC3MFExportOptions(b, OUT)
o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
say('export -> %s %s' % (em.execute(o), OUT))
say('isModified before close: %s' % doc.isModified)
if was_open is None and not doc.isModified:
    doc.close(False); say('closed without saving')
else:
    say('left open (was_open=%s, modified=%s)' % (was_open is not None, doc.isModified))
