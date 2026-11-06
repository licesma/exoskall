"""EXO-370: read-only probe of `sensor module 2 rotor` in `exo generated`: every child body with
its r / z extent in the module frame (vertices), and the horn screw head radius. No change.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/probe_module_exo370.py"""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, BuildArm
importlib.reload(armgen); importlib.reload(BuildArm)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
design = adsk.fusion.Design.cast(app.activeProduct); root = design.rootComponent
occ = [root.occurrences.item(i) for i in range(root.occurrences.count) if root.occurrences.item(i).component.name == BuildArm.MODULE_ROTOR_NAME][0]
say('module occ transform identity? %s' % (list(occ.transform.asArray())))
def walk(o, d=0):
    for i in range(o.childOccurrences.count):
        c = o.childOccurrences.item(i)
        for j in range(c.bRepBodies.count):
            b = c.bRepBodies.item(j)
            rs, zs = [], []
            for k in range(b.vertices.count):
                p = b.vertices.item(k).geometry
                rs.append(math.hypot(p.x, p.y)*10); zs.append(p.z*10)
            bb = b.boundingBox
            say('%s%s / %s: V %.3f  vert r %.3f..%.3f z %.3f..%.3f  bbox z %.3f..%.3f' % ('  '*d, c.component.name, b.name, b.volume*1000,
                min(rs) if rs else -1, max(rs) if rs else -1, min(zs) if zs else 0, max(zs) if zs else 0, bb.minPoint.z*10, bb.maxPoint.z*10))
        walk(c, d+1)
walk(occ)
