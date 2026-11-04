"""EXO-265: re-export the ORIGINAL rotor bodies at High mesh refinement (the v3 files'
tessellation density). Read-only on the original document: no edit, no save."""
import importlib, sys
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)
OUT = {'upper': '/Users/esteban/repos/exo/parts/manual_inner_top.3mf',
       'lower': '/Users/esteban/repos/exo/parts/manual_inner_bottom.3mf'}
prev = app.activeDocument
od = [app.documents.item(i) for i in range(app.documents.count) if app.documents.item(i).name == 'sensor module 2'][0]
log('original %r v%s isModified=%s' % (od.name, od.dataFile.versionNumber, od.isModified))
if od.isModified:
    raise RuntimeError('original modified; refusing')
des = adsk.fusion.Design.cast(od.products.itemByProductType('DesignProductType'))
up, lo = BuildArm.module_rotor_occs(des.rootComponent, in_context=False)
od.activate()
em = des.exportManager
for k, o in (('upper', up), ('lower', lo)):
    b = o.component.bRepBodies.item(0)
    opts = em.createC3MFExportOptions(b, OUT[k])
    opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    log('%s: %r/%r V=%.3f refinement %s -> %s %s' % (k, o.component.name, b.name, b.volume * 1000,
        opts.meshRefinement, em.execute(opts), OUT[k]))
log('original isModified after: %s' % od.isModified)
prev.activate()
