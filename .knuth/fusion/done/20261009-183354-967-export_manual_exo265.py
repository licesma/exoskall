"""EXO-265: export the ORIGINAL `sensor module 2` rotor bodies (Component2 = rotor face
disk, Component3 = rotor inner cylinder) as parts/manual_inner_top.3mf and
parts/manual_inner_bottom.3mf. Read-only on the original: no edit, no save. Also measures
the v3 rotor bodies, the bearing, the XL330 and the fasteners for parts/README.md.
"""
import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import BuildArm
importlib.reload(BuildArm)

ORIG = {'upper': 6473.293, 'lower': 2950.793}
V3 = {'upper': 6657.964, 'lower': 2507.386}
OUT = {'upper': '/Users/esteban/repos/exo/parts/manual_inner_top.3mf',
       'lower': '/Users/esteban/repos/exo/parts/manual_inner_bottom.3mf'}
EXPORT = True

prev = app.activeDocument


def doc_named(n):
    ds = [app.documents.item(i) for i in range(app.documents.count)
          if app.documents.item(i).name.split(' v')[0] == n]
    if len(ds) != 1:
        raise RuntimeError('%d open documents named %r' % (len(ds), n))
    return ds[0]


def design_of(d):
    return adsk.fusion.Design.cast(d.products.itemByProductType('DesignProductType'))


def ext(body, T=None):
    """vertex extents in mm: r_min r_max z_min z_max, and the plain xyz bbox."""
    xs, ys, zs, rs = [], [], [], []
    for i in range(body.vertices.count):
        p = body.vertices.item(i).geometry
        if T is not None:
            p = p.copy(); p.transformBy(T)
        xs.append(p.x * 10); ys.append(p.y * 10); zs.append(p.z * 10)
        rs.append(math.hypot(p.x, p.y) * 10)
    return (min(rs), max(rs), min(zs), max(zs)), (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))


def bbox_s(b):
    bb = b.boundingBox
    return '%.3f x %.3f x %.3f min (%.3f, %.3f, %.3f)' % (
        10 * (bb.maxPoint.x - bb.minPoint.x), 10 * (bb.maxPoint.y - bb.minPoint.y),
        10 * (bb.maxPoint.z - bb.minPoint.z), 10 * bb.minPoint.x, 10 * bb.minPoint.y, 10 * bb.minPoint.z)


problems = []
# ---------- the original, in its own document ----------
od = doc_named('sensor module 2')
log('original document %r version %s isModified=%s' % (od.name, od.dataFile.versionNumber, od.isModified))
if od.isModified:
    problems.append('original document shows unsaved modifications; not exporting')
odes = design_of(od)
oroot = odes.rootComponent
log('original root occurrences: %s' % [oroot.occurrences.item(i).name for i in range(oroot.occurrences.count)])
up, lo = BuildArm.module_rotor_occs(oroot, in_context=False)
parts = {'upper': up, 'lower': lo}
for k, o in parts.items():
    if o is None:
        problems.append('original %s rotor not found' % k)
        continue
    b = o.component.bRepBodies.item(0)
    v = b.volume * 1000
    (r0, r1, z0, z1), _ = ext(o.bRepBodies.item(0))
    log('ORIGINAL %s: occ %r comp %r body %r V=%.3f (record %.3f); module frame r %.3f..%.3f z %.3f..%.3f; occ transform t=(%.3f,%.3f,%.3f)'
        % (k, o.name, o.component.name, b.name, v, ORIG[k], r0, r1, z0, z1,
           10 * o.transform.translation.x, 10 * o.transform.translation.y, 10 * o.transform.translation.z))
    log('   own-frame bbox %s; faces %d' % (bbox_s(b), b.faces.count))
    if abs(v - ORIG[k]) > 1e-3:
        problems.append('original %s volume %.3f != %.3f' % (k, v, ORIG[k]))
# everything else in the original, module frame, for the README
for i in range(oroot.occurrences.count):
    o = oroot.occurrences.item(i)
    for j in range(o.bRepBodies.count):
        (r0, r1, z0, z1), _ = ext(o.bRepBodies.item(j))
        log('   orig child %-40r body %d: r %.3f..%.3f z %.3f..%.3f' % (o.name, j, r0, r1, z0, z1))
    for c in range(o.childOccurrences.count):
        oc = o.childOccurrences.item(c)
        for j in range(oc.bRepBodies.count):
            (r0, r1, z0, z1), _ = ext(oc.bRepBodies.item(j))
            log('      sub %-36r body %d: r %.3f..%.3f z %.3f..%.3f' % (oc.name, j, r0, r1, z0, z1))

# ---------- v3, in exo generated ----------
gd = doc_named('exo generated')
gdes = design_of(gd)
groot = gdes.rootComponent
v3occ = [groot.occurrences.item(i) for i in range(groot.occurrences.count)
         if groot.occurrences.item(i).component.name == 'sensor module 2 v3'][0]
lnk = [groot.occurrences.item(i) for i in range(groot.occurrences.count)
       if groot.occurrences.item(i).component.name == 'sensor module 2'][0]
vu, vl = BuildArm.module_rotor_occs(v3occ, in_context=True)
for k, o in (('upper', vu), ('lower', vl)):
    b = o.component.bRepBodies.item(0)
    v = b.volume * 1000
    (r0, r1, z0, z1), _ = ext(o.bRepBodies.item(0), v3occ.transform.copy() if False else None)
    log('V3 %s: comp %r body %r V=%.3f (record %.3f); own-frame bbox %s; faces %d'
        % (k, o.component.name, b.name, v, V3[k], bbox_s(b), b.faces.count))
lu, ll = BuildArm.module_rotor_occs(lnk, in_context=True)
for k, o in (('upper', lu), ('lower', ll)):
    log('LINKED original in exo generated %s: V=%.3f' % (k, o.component.bRepBodies.item(0).volume * 1000))

# ---------- export ----------
if problems:
    raise RuntimeError('; '.join(problems))
if EXPORT:
    od.activate()
    em = odes.exportManager
    for k, o in parts.items():
        b = o.component.bRepBodies.item(0)
        opts = em.createC3MFExportOptions(b, OUT[k])
        log('export ORIGINAL %s body %r of %r (refinement %s): %s -> %s'
            % (k, b.name, o.component.name, opts.meshRefinement, em.execute(opts), OUT[k]))
    log('original isModified after export: %s' % od.isModified)
    prev.activate()
