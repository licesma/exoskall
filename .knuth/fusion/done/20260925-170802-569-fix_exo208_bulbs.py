"""EXO-208 / EXO-203: the placed modules read bulb=True but do not render. Force the
display state by toggling every occurrence under the arm off and back on, through the
ROOT-CONTEXT proxy (a native occurrence does not take the assignment - EXO-203)."""
import sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)

root = design.rootComponent
arm = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name.startswith('generated_R_arm_general'):
        arm = o
n = 0


def toggle(occ, parent):
    global n
    for i in range(occ.childOccurrences.count):
        c = occ.childOccurrences.item(i)
        for target in (c, BuildArm.occ_root_proxy(c, parent)):
            try:
                target.isLightBulbOn = False
                adsk.doEvents()
                target.isLightBulbOn = True
                adsk.doEvents()
                n += 1
            except Exception as e:
                log('  %s: %s' % (c.component.name, e))
        toggle(c, parent)


toggle(arm, arm)
log('toggled %d occurrence handle(s)' % n)
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if not o.component.name.startswith('generated_R_arm_general'):
        o.isLightBulbOn = True
        adsk.doEvents()
        o.isLightBulbOn = False
        adsk.doEvents()
    log('root %-34r bulb=%s vis=%s' % (o.component.name, o.isLightBulbOn, o.isVisible))
vp = app.activeViewport
vp.refresh()
adsk.doEvents()
h = int(round(1600.0 * vp.height / vp.width))
p = '/Users/esteban/repos/exo/gento_luna/exo/screenshots/generated_R_arm_exo208_arm_iso.png'
cam = vp.camera
cam.isFitView = True
vp.camera = cam
adsk.doEvents()
log('saved -> %s' % vp.saveAsImageFile(p, 1600, h))
