"""EXO-208 probe: what is in the document, and how `sensor module 2` is referenced."""
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)

doc = app.activeDocument
root = design.rootComponent
log('doc %r  saved=%s  designType=%s' % (doc.name, doc.isSaved, design.designType))
log('allComponents: %d' % design.allComponents.count)
for c in design.allComponents:
    log('  comp %-45r bodies=%d occs=%d feats=%d sketches=%d'
        % (c.name, c.bRepBodies.count, c.occurrences.count, c.features.count,
           c.sketches.count))
log('')
log('root occurrences: %d' % root.occurrences.count)
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    log('  %-40r comp=%-40r ref=%s bulb=%s' % (o.name, o.component.name,
        o.isReferencedComponent, o.isLightBulbOn))

def walk(occ, depth=1, maxd=3):
    for i in range(occ.childOccurrences.count):
        c = occ.childOccurrences.item(i)
        log('%s%-40r comp=%-38r ref=%s bodies=%d' % ('    ' * depth, c.name,
            c.component.name, c.isReferencedComponent, c.component.bRepBodies.count))
        if depth < maxd:
            walk(c, depth + 1, maxd)

for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name.startswith('generated_R_arm_general'):
        log('')
        log('--- under %s ---' % o.component.name)
        walk(o, 1, 3)
