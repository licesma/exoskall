"""EXO-208: look at J1 with an EXPLICIT camera (the one that demonstrably moves), wide
enough to see a whole module, and report the world bbox of the module that should be there."""
import math, sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, sweepgen, BuildArm, BuildArmGen
for m in (armgen, sweepgen, BuildArm, BuildArmGen):
    importlib.reload(m)

root = design.rootComponent
vp = app.activeViewport
arm = [root.occurrences.item(i) for i in range(root.occurrences.count)
       if root.occurrences.item(i).component.name.startswith('generated_R_arm_general')][0]
wr = [arm.childOccurrences.item(i) for i in range(arm.childOccurrences.count)
      if arm.childOccurrences.item(i).component.name.startswith('GJ1_')][0]
mod = wr.childOccurrences.item(0)
log('module occ %r bulb=%s vis=%s' % (mod.component.name, mod.isLightBulbOn, mod.isVisible))
for i in range(mod.childOccurrences.count):
    c = mod.childOccurrences.item(i)
    if c.bRepBodies.count == 1:
        bb = c.bRepBodies.item(0).boundingBox
        log('  %-24r world bbox mm x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f  vis=%s'
            % (c.component.name, bb.minPoint.x * 10, bb.maxPoint.x * 10,
               bb.minPoint.y * 10, bb.maxPoint.y * 10, bb.minPoint.z * 10,
               bb.maxPoint.z * 10, c.isVisible))
        cen = [(bb.minPoint.x + bb.maxPoint.x) / 2, (bb.minPoint.y + bb.maxPoint.y) / 2,
               (bb.minPoint.z + bb.maxPoint.z) / 2]
lk = [arm.childOccurrences.item(i) for i in range(arm.childOccurrences.count)
      if arm.childOccurrences.item(i).component.name == 'g_link_12'][0]
bb = lk.bRepBodies.item(0).boundingBox
log('g_link_12 world bbox mm x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f'
    % (bb.minPoint.x * 10, bb.maxPoint.x * 10, bb.minPoint.y * 10, bb.maxPoint.y * 10,
       bb.minPoint.z * 10, bb.maxPoint.z * 10))

cam = vp.camera
cam.isFitView = False
cam.target = adsk.core.Point3D.create(*cen)
cam.eye = adsk.core.Point3D.create(cen[0] + 12.0, cen[1] - 12.0, cen[2] + 8.0)
cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
cam.viewExtents = 12.0
cam.isSmoothTransition = False
vp.camera = cam
adsk.doEvents()
vp.camera = cam
adsk.doEvents()
h = int(round(1600.0 * vp.height / vp.width))
log('saved %s' % vp.saveAsImageFile(
    '/Users/esteban/repos/exo/gento_luna/exo/screenshots/exo208_vistest_J1.png', 1600, h))
