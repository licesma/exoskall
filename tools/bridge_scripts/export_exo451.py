"""EXO-451: export the camera support (neck) of `arm complete` (component `Camera support`, Body1,
identity occurrence, arm frame) and the strap clip (root Body55, same doc) at High refinement, and save
B-rep copies in a NEW document `/body mount/camera support (from arm complete)`. arm complete is opened
read only and closed without saving. Run once (2026-11-16): re-running creates and saves another new document.
The clip file was deleted after the run: Body55 / Body57 overlap the neck's receivers by 15.9 mm3 each as
modelled (Body58 / Body59 on the chest mount hooks: 0), so the clip is not offered for printing.
Saved: lineage urn:adsk.wipprod:dm.lineage:imRQgVUSRmCx9ppFVsuRNQ v1, isComplete True, in the folder listing.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/export_exo451.py --timeout 400"""
import traceback
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
AC = 'urn:adsk.wipprod:dm.lineage:Jy8czxywR4ybJedncZiLhQ'
SA = 'urn:adsk.wipprod:dm.lineage:z15GBJnDRyeQElHZvbYgwg'   # /body mount/arm complete_Camera support (folder only)
NAME = 'camera support (from arm complete)'
OUT = '/Users/esteban/repos/exo/parts/camera_support.3mf'
OUT_CLIP = '/Users/esteban/repos/exo/parts/camera_support_strap_clip.3mf'
active0 = app.activeDocument
before = [(app.documents.item(i).name, app.documents.item(i).isModified) for i in range(app.documents.count)]
say('active %r; before %s' % (active0.name, before))
def bbs(bb): return [round(v*10, 3) for v in (bb.minPoint.x, bb.minPoint.y, bb.minPoint.z, bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z)]
TB = adsk.fusion.TemporaryBRepManager.get()
ac = None
for i in range(app.documents.count):
    try:
        if app.documents.item(i).dataFile.id == AC: ac = app.documents.item(i)
    except Exception: pass
opened = ac is None
if opened:
    df = app.data.findFileById(AC); say('open %r v%s' % (df.name, df.versionNumber))
    ac = app.documents.open(df, False)
say('arm complete isModified before %s' % ac.isModified)
nd = None
try:
    acd = adsk.fusion.Design.cast(ac.products.itemByProductType('DesignProductType'))
    comp = [acd.allComponents.item(j) for j in range(acd.allComponents.count) if acd.allComponents.item(j).name == 'Camera support'][0]
    occs = acd.rootComponent.allOccurrencesByComponent(comp)
    assert occs.count == 1 and all(abs(a - b) < 1e-12 for a, b in zip(occs.item(0).transform2.asArray(), [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]))
    cs = comp.bRepBodies.item(0)
    root = acd.rootComponent
    clips = {n: root.bRepBodies.itemByName(n) for n in ('Body55', 'Body57', 'Body58', 'Body59')}
    b14 = [acd.allComponents.item(j) for j in range(acd.allComponents.count) if acd.allComponents.item(j).name == 'chest mount'][0].bRepBodies.itemByName('Body14')
    say('SRC %r V %.3f A %.3f faces %d bbox %s' % (cs.name, cs.volume*1000, cs.area*100, cs.faces.count, bbs(cs.boundingBox)))
    for n, b in clips.items():
        say('CLIP %s V %.3f A %.3f faces %d bbox %s' % (n, b.volume*1000, b.area*100, b.faces.count, bbs(b.boundingBox)))
    def inter(a, b):
        ta = TB.copy(a); TB.booleanOperation(ta, TB.copy(b), adsk.fusion.BooleanTypes.IntersectionBooleanType)
        return ta.volume*1000 if ta.lumps.count else 0.0
    for n in ('Body55', 'Body57'): say('intersection %s with camera support: %.4f mm3' % (n, inter(clips[n], cs)))
    for n in ('Body58', 'Body59'): say('intersection %s with chest mount Body14: %.4f mm3' % (n, inter(clips[n], b14)))
    # Body57 = Body55 moved? compare by intersect of aligned copies is not needed: same V/A/faces is logged
    em = acd.exportManager
    for b, path in ((cs, OUT), (clips['Body55'], OUT_CLIP)):
        o = em.createC3MFExportOptions(b, path); o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        say('export %s -> %s' % (em.execute(o), path))
    copies = [('camera support', TB.copy(cs))] + [('strap clip %s' % n, TB.copy(clips[n])) for n in ('Body55', 'Body57')]
    nd = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(nd.products.itemByProductType('DesignProductType'))
    r = des.rootComponent
    if des.designType == adsk.fusion.DesignTypes.ParametricDesignType:
        bf = r.features.baseFeatures.add(); bf.startEdit()
        for n, t in copies: r.bRepBodies.add(t, bf)
        bf.finishEdit(); bf.name = 'Camera support + 2 strap clips from arm complete v35 (EXO-451)'
    else:
        for n, t in copies: r.bRepBodies.add(t)
    for i, (n, t) in enumerate(copies):
        b = r.bRepBodies.item(i); b.name = n
        say('COPY %r V %.3f A %.3f faces %d bbox %s' % (b.name, b.volume*1000, b.area*100, b.faces.count, bbs(b.boundingBox)))
    folder = app.data.findFileById(SA).parentFolder
    names = [folder.dataFiles.item(i).name for i in range(folder.dataFiles.count)]
    say('target folder %r: %s' % (folder.name, names))
    if NAME in names: raise RuntimeError('name taken')
    ok = nd.saveAs(NAME, folder, 'EXO-451: camera support (neck) + 2 strap clips, B-rep copies from arm complete v35, arm frame', '')
    say('saveAs -> %s; name %r isSaved %s isModified %s' % (ok, nd.name, nd.isSaved, nd.isModified))
except Exception:
    say(traceback.format_exc())
say('arm complete isModified after %s' % ac.isModified)
if opened:
    ac.close(False); say('arm complete closed without saving')
active0.activate()
say('after %s; active %r' % ([(app.documents.item(i).name, app.documents.item(i).isModified) for i in range(app.documents.count)], app.activeDocument.name))
