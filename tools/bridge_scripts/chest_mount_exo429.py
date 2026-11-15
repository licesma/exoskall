"""EXO-429 (ran once, 2026-11-14; re-running creates and saves ANOTHER new document): copy Body14 (`chest mount` in `arm complete`, identity occurrence) into a new document,
widen both J1 actuator openings 20.0 x 34.0 -> 20.2 x 34.2 (Offset Faces -0.1 on the 4 walls of each),
export High to parts/chest_mount.3mf, save the new doc in /body mount. arm complete is read only.
The Offset Faces feature (-0.1 on the 8 opening walls, areas 140 / 238) removed exactly 151.760 mm3.
The doc is saved as `/body mount/chest mount hooks 20.2`.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/chest_mount_exo429.py --timeout 300"""
import math
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
AC = 'urn:adsk.wipprod:dm.lineage:Jy8czxywR4ybJedncZiLhQ'
A_DOC = 'urn:adsk.wipprod:dm.lineage:lvB_fi9qT02ITHaBJZkTrw'
NAME = 'chest mount hooks 20.2'
OUT = '/Users/esteban/repos/exo/parts/chest_mount.3mf'
active0 = app.activeDocument
docs0 = [app.documents.item(i) for i in range(app.documents.count)]
mod0 = [(d.name, d.isModified) for d in docs0]
say('active %r; modified before %s' % (active0.name, mod0))
ac = a_doc = None
for d in docs0:
    try:
        if d.dataFile.id == AC: ac = d
        if d.dataFile.id == A_DOC: a_doc = d
    except Exception: pass
acd = adsk.fusion.Design.cast(ac.products.itemByProductType('DesignProductType'))
comp = [acd.allComponents.item(j) for j in range(acd.allComponents.count) if acd.allComponents.item(j).name == 'chest mount'][0]
occs = acd.rootComponent.allOccurrencesByComponent(comp)
assert occs.count == 1 and all(abs(a - b) < 1e-12 for a, b in zip(occs.item(0).transform2.asArray(), [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]))
src = comp.bRepBodies.itemByName('Body14')
say('SRC Body14 V %.3f A %.3f faces %d' % (src.volume*1000, src.area*100, src.faces.count))
em = acd.exportManager
o = em.createC3MFExportOptions(src, '/tmp/exo429/b14_orig.3mf'); o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
say('export orig %s' % em.execute(o))
tb = adsk.fusion.TemporaryBRepManager.get().copy(src)

nd = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
des = adsk.fusion.Design.cast(nd.products.itemByProductType('DesignProductType'))
say('new doc %r designType %s' % (nd.name, des.designType))
root = des.rootComponent
if des.designType == adsk.fusion.DesignTypes.ParametricDesignType:
    bf = root.features.baseFeatures.add(); bf.startEdit()
    body = root.bRepBodies.add(tb, bf); bf.finishEdit(); bf.name = 'Body14 from arm complete (EXO-429)'
    body = root.bRepBodies.item(0)
else:
    body = root.bRepBodies.add(tb)
body.name = 'chest mount'
say('COPY V %.3f A %.3f faces %d' % (body.volume*1000, body.area*100, body.faces.count))
em2 = des.exportManager
o = em2.createC3MFExportOptions(body, '/tmp/exo429/copy_pre.3mf'); o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
say('export copy pre %s' % em2.execute(o))

V0 = body.volume*1000
walls = [f for f in body.faces if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType
         and (abs(f.area*100 - 140) < 0.01 or abs(f.area*100 - 238) < 0.01)]
say('walls %d' % len(walls))
assert len(walls) == 8
off = root.features.offsetFacesFeatures
fi = off.createInput(walls, adsk.core.ValueInput.createByString('-0.1 mm'))
ft = off.add(fi); ft.name = 'J1 opening +0.1 per side (EXO-429)'
body = root.bRepBodies.item(0)
V1 = body.volume*1000
say('AFTER V %.3f (dV %.3f, expected -%.3f) A %.3f faces %d' % (V1, V1 - V0, 2*(20.2*34.2 - 680)*7, body.area*100, body.faces.count))
bb = body.boundingBox
say('bbox %s' % [round(v*10, 4) for v in (bb.minPoint.x, bb.minPoint.y, bb.minPoint.z, bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z)])
# opening widths
seats = {'L': ((0, 0, 0), (0, 0.96126, 0.27564)), 'R': ((0, -200.44, 0), (0, 0.96126, -0.27564))}
for k, (so, sa) in seats.items():
    n = math.sqrt(sum(x*x for x in sa)); sa = [x/n for x in sa]
    fs = []
    for f in body.faces:
        g = f.geometry
        if g.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType: continue
        p = f.pointOnFace; p = [p.x*10 - so[0], p.y*10 - so[1], p.z*10 - so[2]]
        ax = sum(p[i]*sa[i] for i in range(3)); rad = math.sqrt(max(0, sum(x*x for x in p) - ax*ax))
        ok, nn = f.evaluator.getNormalAtPoint(f.pointOnFace); nn = [nn.x, nn.y, nn.z]
        if 24 < abs(ax) < 32 and rad < 26 and abs(sum(nn[i]*sa[i] for i in range(3))) < 1e-6:
            fs.append((f, [g.origin.x*10, g.origin.y*10, g.origin.z*10], nn))
    say('seat %s wall faces %d: areas %s' % (k, len(fs), [round(f.area*100, 4) for f, _, _ in fs]))
    for i in range(len(fs)):
        for j in range(i+1, len(fs)):
            ni = fs[i][2]; nj = fs[j][2]
            if sum(ni[t]*nj[t] for t in range(3)) < -0.999999:
                d = abs(sum((fs[i][1][t] - fs[j][1][t])*ni[t] for t in range(3)))
                say('  seat %s opposing walls distance %.5f' % (k, d))
em2 = des.exportManager
o = em2.createC3MFExportOptions(body, OUT); o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
say('export %s %s' % (em2.execute(o), OUT))
# save
try:
    folder = a_doc.dataFile.parentFolder
    say('target folder %r id %s' % (folder.name, folder.id))
    try:
        names = [folder.dataFiles.item(i).name for i in range(folder.dataFiles.count)]
        say('folder files: %s' % names)
        if NAME in names: raise RuntimeError('name taken')
    except Exception as e:
        say('folder listing: %s' % e)
    r = nd.saveAs(NAME, folder, 'EXO-429: Body14 chest mount (hooks) from arm complete v35, J1 openings 20.2 x 34.2', '')
    say('saveAs -> %s; name %r isSaved %s isModified %s' % (r, nd.name, nd.isSaved, nd.isModified))
except Exception as e:
    say('SAVE FAILED: %s' % e)
active0.activate()
say('active now %r' % app.activeDocument.name)
say('modified after: %s' % [(app.documents.item(i).name, app.documents.item(i).isModified) for i in range(app.documents.count)])
