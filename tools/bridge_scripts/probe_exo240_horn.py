"""EXO-240 follow-up: WHERE the XL330 horn disk actually is, in the module frame.

EXO-231 quotes "horn disk r 8.000, measured" as what the nut pocket's inner flat has to
stay off. `verify_exo240.py` only found something at r 5.200 inside the lower part's bore
over the nut pocket band, so this reads every body of the module that is not a rotor part
or a fastener: its own r/z extents, and then the outer radius of whatever is solid in the
z band just under the lower part's bottom face -13.200.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/probe_exo240_horn.py --timeout 300
"""
import math
import sys
import importlib

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
import BuildArm
importlib.reload(armgen)
importlib.reload(BuildArm)
import adsk.core as ac
import adsk.fusion as af

root = design.rootComponent
INSIDE = (af.PointContainment.PointInsidePointContainment,
          af.PointContainment.PointOnPointContainment)

occ = None
for i in range(root.occurrences.count):
    if root.occurrences.item(i).component.name == BuildArm.MODULE_V3_NAME:
        occ = root.occurrences.item(i)
if occ is None:
    raise RuntimeError('no "%s" occurrence at the root' % BuildArm.MODULE_V3_NAME)


def solid_xy(b, x, y, z):
    return b.pointContainment(ac.Point3D.create(x / 10.0, y / 10.0, z / 10.0)) in INSIDE


def bisect(f, a_in, a_out):
    for _ in range(60):
        m = (a_in + a_out) / 2.0
        a_in, a_out = (m, a_out) if f(m) else (a_in, m)
    return (a_in + a_out) / 2.0


up, lo = BuildArm.module_rotor_occs(occ, in_context=True)
screws, _ = BuildArm.module_fastener_occs(occ)
nut_occ, bolt_occ = BuildArm.module_v3_fasteners(occ)
skip = set([up.component.name, lo.component.name] + [s.component.name for s in screws])
for o in (nut_occ, bolt_occ):
    if o is not None:
        skip.add(o.component.name)

rows = []


def walk(o):
    if o.component.name not in skip:
        for b in o.bRepBodies:
            rows.append((o.component.name, b))
    for j in range(o.childOccurrences.count):
        walk(o.childOccurrences.item(j))


walk(occ)
log('%d bodies in the module that are neither a rotor part nor a fastener' % len(rows))
ZS = (-10.60, -11.00, -12.00, -13.00, -13.19, -13.25, -13.30, -13.40, -13.50, -13.60)
for name, b in rows:
    rs, zs = [], []
    for k in range(b.vertices.count):
        p = b.vertices.item(k).geometry
        rs.append(math.hypot(p.x, p.y) * 10.0)
        zs.append(p.z * 10.0)
    ext = ('r %7.3f..%7.3f z %8.3f..%8.3f' % (min(rs), max(rs), min(zs), max(zs))) if rs \
        else 'no vertices'
    try:
        vol = '%10.3f mm3' % (b.volume * 1000.0)
    except Exception as e:          # surface bodies of the bearing refuse getEntityVolume
        vol = '  no volume'
    log('  %-22s %-44s V %s' % (name, ext, vol))
    for z in ZS:
        hits = []
        for a in (0.0, 45.0, 90.0, 180.0, 270.0):
            x0, y0 = BuildArm._polar(1.0, a)
            if not solid_xy(b, x0, y0, z):
                # maybe it is an annulus: try further out
                found = None
                for rr in (5.0, 6.0, 7.0, 7.5, 8.0, 8.5):
                    x, y = BuildArm._polar(rr, a)
                    if solid_xy(b, x, y, z):
                        found = rr
                        break
                if found is None:
                    continue
                r_in, r_out = found, found
                r_lo = bisect(lambda r: solid_xy(b, *BuildArm._polar(r, a), z=z), found, 0.5)
                r_hi = bisect(lambda r: solid_xy(b, *BuildArm._polar(r, a), z=z), found, 20.0)
                hits.append('%.0f:%0.4f..%.4f' % (a, r_lo, r_hi))
            else:
                r_hi = bisect(lambda r: solid_xy(b, *BuildArm._polar(r, a), z=z), 1.0, 20.0)
                hits.append('%.0f:0..%.4f' % (a, r_hi))
        if hits:
            log('      z %7.3f  %s' % (z, '  '.join(hits)))
log('EXO-240 horn probe done.')
