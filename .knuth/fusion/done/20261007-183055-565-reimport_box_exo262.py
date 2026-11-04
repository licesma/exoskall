"""EXO-262: swap the organizer box in "tool_organizer_exo258" for the rebuilt STEP
(one-line labels) WITHOUT a new document and without touching the EXO-261 parts.

The box is the single root body, owned by the root base feature that the STEP import
created; its geometry is replaced with BaseFeature.updateBody from a scratch import
(closed without saving). Then saved as a new version.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/reimport_box_exo262.py --timeout 300
Then: place_parts_exo261.py is NOT needed; re-measure with it only if the parts must move.
"""
import importlib
import sys

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)

NAME = 'tool_organizer_exo258'
STEP = '/Users/esteban/repos/exo/gento_luna/exo/exports/tool_organizer_exo258.step'
tbm = adsk.fusion.TemporaryBRepManager.get()

org = [app.documents.item(i) for i in range(app.documents.count)
       if app.documents.item(i).name.split(' v')[0] == NAME]
if len(org) != 1:
    raise RuntimeError('%d open documents named %s' % (len(org), NAME))
org = org[0]

im = app.importManager
tmp = im.importToNewDocument(im.createSTEPImportOptions(STEP))
troot = adsk.fusion.Design.cast(tmp.products.itemByProductType('DesignProductType')).rootComponent
src = list(troot.bRepBodies) + [b for o in troot.allOccurrences for b in o.bRepBodies]
if len(src) != 1:
    raise RuntimeError('rebuilt STEP has %d bodies' % len(src))
new = tbm.copy(src[0])
tmp.close(False)

org.activate()
des = adsk.fusion.Design.cast(app.activeProduct)
root = des.rootComponent
log('before: root bodies %d, base features %d, occurrences %s' % (
    root.bRepBodies.count, root.features.baseFeatures.count, [o.component.name for o in root.occurrences]))
if root.bRepBodies.count != 1 or root.features.baseFeatures.count != 1:
    raise RuntimeError('unexpected root structure, not touching it')
bf = root.features.baseFeatures.item(0)
bf.startEdit()
old = bf.bodies.item(0)
ok = bf.updateBody(old, new)
bf.finishEdit()
log('updateBody: %s' % ok)
box = root.bRepBodies.item(0)
bb = box.boundingBox
log('after: root bodies %d (%r), bbox mm %.2f x %.2f x %.2f, volume %.0f mm3, faces %d' % (
    root.bRepBodies.count, box.name, 10 * (bb.maxPoint.x - bb.minPoint.x), 10 * (bb.maxPoint.y - bb.minPoint.y),
    10 * (bb.maxPoint.z - bb.minPoint.z), 1000 * box.volume, box.faces.count))
log('occurrences kept: %s' % [o.component.name for o in root.occurrences])
log(BuildArm.save_active_document(app, why='EXO-262: one-line floor labels'))
