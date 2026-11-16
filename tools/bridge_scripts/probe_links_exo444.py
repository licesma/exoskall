"""EXO-444: read-only probe of the six generated links in the ACTIVE `exo generated`: volume, area,
faces, bounding box and centre of mass of each g_link_* body, so the build before and after the
wrist side rule can be compared (link_12..link_45 must not change). Never edits or saves.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/probe_links_exo444.py --timeout 300
"""
import json
doc = app.activeDocument
if doc.name != 'exo generated':
    raise RuntimeError('active document is %r' % doc.name)
root = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType')).rootComponent
arm = [root.occurrences.item(i) for i in range(root.occurrences.count)
       if root.occurrences.item(i).component.name == 'generated_R_arm_general'][0]
out = {'version': doc.dataFile.versionNumber, 'isModified': doc.isModified}
for i in range(arm.childOccurrences.count):
    o = arm.childOccurrences.item(i)
    if not o.component.name.startswith('g_link_'):
        continue
    b = o.component.bRepBodies.item(0)
    bb = b.boundingBox
    pp = b.physicalProperties
    c = pp.centerOfMass
    out[o.component.name] = {'V': b.volume * 1000, 'A': b.area * 100, 'faces': b.faces.count,
                             'edges': b.edges.count,
                             'bbox': [bb.minPoint.x * 10, bb.minPoint.y * 10, bb.minPoint.z * 10,
                                      bb.maxPoint.x * 10, bb.maxPoint.y * 10, bb.maxPoint.z * 10],
                             'com': [c.x * 10, c.y * 10, c.z * 10]}
    log('%s V %.3f A %.3f faces %d edges %d bbox %s com %s' % (o.component.name, out[o.component.name]['V'],
        out[o.component.name]['A'], b.faces.count, b.edges.count, ['%.3f' % v for v in out[o.component.name]['bbox']],
        ['%.3f' % v for v in out[o.component.name]['com']]))
log('JSON ' + json.dumps(out))
