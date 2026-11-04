"""EXO-304: new document, linked sensor module 2, local copy `sensor module 2 v4` = v3 recipe + rotor_v4,
export three parts at High. Never opens or saves the original or the v3 copy."""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
SRC_ID = 'urn:adsk.wipprod:dm.lineage:CzEATRrQSnuX-w_dwKCH6A'
df = app.data.findFileById(SRC_ID)
log('source %s v%s' % (df.name, df.versionNumber))
doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
des = adsk.fusion.Design.cast(app.activeProduct)
des.designType = adsk.fusion.DesignTypes.ParametricDesignType
root = des.rootComponent
link = root.occurrences.addByInsert(df, adsk.core.Matrix3D.create(), True)
log('linked occurrence %r referenced=%s' % (link.component.name, link.isReferencedComponent))
# volumes of the ORIGINAL rotor parts, read through the link (read-only)
u0, l0 = BuildArm.module_rotor_occs(link, in_context=True)
log('original upper V %.3f, lower V %.3f' % (u0.bRepBodies.item(0).volume*1000, l0.bRepBodies.item(0).volume*1000))
comp, occ, b_occ, how = BuildArm.build_v4_copy(des, log)
log(how)
link.isLightBulbOn = False
up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
em = des.exportManager
OUT = [(up, '/Users/esteban/repos/exo/parts/inner_top_v4.3mf'),
       (lo, '/Users/esteban/repos/exo/parts/inner_bottom_v4a.3mf'),
       (b_occ, '/Users/esteban/repos/exo/parts/inner_bottom_v4b.3mf')]
for o, path in OUT:
    b = o.component.bRepBodies.item(0)
    bb = o.bRepBodies.item(0).boundingBox
    opts = em.createC3MFExportOptions(b, path)
    opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    log('%s V %.3f module z %.4f..%.4f -> %s %s' % (o.component.name, b.volume*1000,
        bb.minPoint.z*10, bb.maxPoint.z*10, em.execute(opts), path))
log('link still referenced: %s; doc %r unsaved' % (link.isReferencedComponent, doc.name))
