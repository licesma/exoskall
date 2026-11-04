"""EXO-263: print file for the organizer - the box body ONLY (not the EXO-261 example
bearing / XL330 components) from "tool_organizer_exo258", as parts/tool_organizer.3mf.
Same route as parts/inner_top.3mf / stator.3mf: Fusion's native 3MF export of one body.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/export_3mf_exo263.py --timeout 300
"""
NAME = 'tool_organizer_exo258'
OUT = '/Users/esteban/repos/exo/parts/tool_organizer.3mf'
docs = [app.documents.item(i) for i in range(app.documents.count)
        if app.documents.item(i).name.split(' v')[0] == NAME]
if len(docs) != 1:
    raise RuntimeError('%d open documents named %s' % (len(docs), NAME))
docs[0].activate()
des = adsk.fusion.Design.cast(app.activeProduct)
root = des.rootComponent
if root.bRepBodies.count != 1:
    raise RuntimeError('expected one root body, found %d' % root.bRepBodies.count)
box = root.bRepBodies.item(0)
bb = box.boundingBox
log('exporting %r of %r (version %s): %.2f x %.2f x %.2f mm, volume %.0f mm3; NOT exported: %s' % (
    box.name, docs[0].name, docs[0].dataFile.versionNumber,
    10 * (bb.maxPoint.x - bb.minPoint.x), 10 * (bb.maxPoint.y - bb.minPoint.y), 10 * (bb.maxPoint.z - bb.minPoint.z),
    1000 * box.volume, [o.component.name for o in root.occurrences]))
em = des.exportManager
opts = em.createC3MFExportOptions(box, OUT)
log('mesh refinement: %s' % opts.meshRefinement)
log('3MF export: %s -> %s' % (em.execute(opts), OUT))
