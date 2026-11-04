import sys, importlib, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen; importlib.reload(armgen)
import adsk.core as ac

root = design.rootComponent
arm = next(o for o in root.occurrences if o.component.name.startswith('generated_R_arm'))
hr = armgen.HOLE_R
l45 = next(s for s in arm.childOccurrences if s.component.name.startswith('link_45'))
body = l45.bRepBodies.item(0)

log('link_45 stator-pad holes: what surface is at EACH end, and how flat it is')
seen = []
for f in body.faces:
    g = f.geometry
    if g.objectType != 'adsk::core::Cylinder' or abs(g.radius*10.0 - hr) > 0.05: continue
    k = (round(g.origin.x,4), round(g.origin.y,4), round(g.origin.z,4))
    if k in seen: continue
    seen.append(k)
    o, ax = g.origin, g.axis
    bb = f.boundingBox
    log('')
    log('hole at (%6.1f,%6.1f,%6.1f):' % (o.x*10, o.y*10, o.z*10))
    # the two ends of the hole = the faces that share an edge with this cylinder
    for e in f.edges:
        for nf in e.faces:
            if nf == f: continue
            gt = nf.geometry.objectType.split('::')[-1]
            # centroid-ish: area and type
            extra = ''
            if gt == 'Cylinder':
                extra = ' r=%.2f mm' % (nf.geometry.radius*10)
            elif gt in ('Torus',):
                extra = ' (fillet)'
            log('   adjoins %-10s area %7.2f mm2%s' % (gt, nf.area*100, extra))
