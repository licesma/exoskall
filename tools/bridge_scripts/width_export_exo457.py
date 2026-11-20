"""EXO-457: export the BUILT solids of one EXO-457 width-check document as meshes, for the
joint-range interference sweep (`tools/width_sweep_exo457.py`). READ-ONLY: the document is only
activated and read (isModified is logged before and after), then the previously active document
is re-activated. Refuses any document that is not one of this issue's own.

Run through a wrapper that sets DOC (e.g. 'width52 check G1') and OUT (a /tmp directory):
    /usr/bin/python3 tools/fusion_run.py /tmp/exo457/scripts/export_g1_derived.py --timeout 1200

Writes OUT/manifest.json and one binary STL per body (world coordinates, mm). Each body is
meshed with `MeshCalculator.surfaceTolerance` = TOL_MM (chordal deviation) and its mesh bounding
box is compared with the solid's own world bounding box (logged; a mismatch raises). Module
frames (origin, X, Y, Z) are read off each placed module occurrence, world, mm.
"""
import json
import os
import struct

DOC = globals().get('DOC')
OUT = globals().get('OUT')
TOL_MM = globals().get('TOL_MM', 0.02)
if not DOC or not (DOC.startswith('width52 check') or DOC.startswith('width manual check')):
    raise RuntimeError('DOC %r is not an EXO-457 document' % DOC)
if not OUT or not OUT.startswith('/tmp/'):
    raise RuntimeError('OUT must be under /tmp (got %r)' % OUT)
os.makedirs(OUT, exist_ok=True)

_log = log


def log(m):  # noqa: F811
    _log(str(m).encode('ascii', 'backslashreplace').decode())


prev = app.activeDocument
docs = [app.documents.item(i) for i in range(app.documents.count)]
target = [d for d in docs if d.name == DOC]
if len(target) != 1:
    raise RuntimeError('%d open documents named %r' % (len(target), DOC))
doc = target[0]
doc.activate()
if app.activeDocument.name != DOC:
    raise RuntimeError('could not activate %r' % DOC)
mod0 = doc.isModified
des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
root = des.rootComponent
arm = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    if o.component.name in ('generated_R_arm_general', 'generated_R_arm_luna'):
        arm = o
if arm is None:
    raise RuntimeError('no generated arm in %r' % DOC)
log('document %r v%s isModified %s, arm %s' % (DOC, doc.dataFile.versionNumber if doc.dataFile else None,
                                               mod0, arm.component.name))


def frame_of(occ):
    o, x, y, z = occ.transform2.getAsCoordinateSystem()
    return {'o': [o.x * 10, o.y * 10, o.z * 10], 'x': [x.x, x.y, x.z], 'y': [y.x, y.y, y.z],
            'z': [z.x, z.y, z.z]}


def write_stl(path, coords, idx):
    n = len(idx) // 3
    with open(path, 'wb') as f:
        f.write(b'EXO-457'.ljust(80, b' '))
        f.write(struct.pack('<I', n))
        for t in range(n):
            f.write(struct.pack('<3f', 0.0, 0.0, 0.0))
            for k in range(3):
                j = idx[3 * t + k]
                f.write(struct.pack('<3f', coords[3 * j] * 10, coords[3 * j + 1] * 10, coords[3 * j + 2] * 10))
            f.write(b'\x00\x00')
    return n


bodies, frames = [], {}


def mesh_body(b, group, part, path_name):
    calc = b.meshManager.createMeshCalculator()
    calc.surfaceTolerance = TOL_MM / 10.0
    m = calc.calculate()
    coords = list(m.nodeCoordinatesAsDouble)
    idx = list(m.nodeIndices)
    bb = b.boundingBox
    lo = [min(coords[i::3]) * 10 for i in range(3)]
    hi = [max(coords[i::3]) * 10 for i in range(3)]
    want_lo = [bb.minPoint.x * 10, bb.minPoint.y * 10, bb.minPoint.z * 10]
    want_hi = [bb.maxPoint.x * 10, bb.maxPoint.y * 10, bb.maxPoint.z * 10]
    dev = max(max(abs(lo[i] - want_lo[i]), abs(hi[i] - want_hi[i])) for i in range(3))
    fn = '%03d.stl' % len(bodies)
    n = write_stl(os.path.join(OUT, fn), coords, idx)
    try:
        vol = b.volume * 1000.0
    except Exception:  # noqa: BLE001
        vol = None
    bodies.append({'file': fn, 'group': group, 'part': part, 'name': path_name, 'tris': n,
                   'volume': vol, 'bbox': [want_lo, want_hi], 'mesh_bbox_dev': dev})
    if dev > 0.5:
        raise RuntimeError('%s: mesh bbox %.3f mm off the solid bbox' % (path_name, dev))


def walk(occ, group, top):
    for i in range(occ.bRepBodies.count):
        b = occ.bRepBodies.item(i)
        if not b.isSolid:
            continue
        mesh_body(b, group, top or occ.component.name, occ.component.name + '/' + b.name)
    for i in range(occ.childOccurrences.count):
        c = occ.childOccurrences.item(i)
        walk(c, group, top or c.component.name)


for i in range(arm.childOccurrences.count):
    c = arm.childOccurrences.item(i)
    n = c.component.name
    if n.startswith('g_link_'):
        walk(c, n[2:], 'link')
    elif n.startswith('G') and '_' in n:
        lab = n[1:].split('_', 1)[0]
        mod = c.childOccurrences.item(0)
        frames[lab] = dict(frame_of(mod), wrapper=n)
        for k in range(mod.childOccurrences.count):
            walk(mod.childOccurrences.item(k), 'M' + lab, None)
        for k in range(mod.bRepBodies.count):
            mesh_body(mod.bRepBodies.item(k), 'M' + lab, mod.component.name,
                      mod.component.name + '/' + mod.bRepBodies.item(k).name)
    else:
        log('skipped arm child %r' % n)

man = {'doc': DOC, 'version': doc.dataFile.versionNumber if doc.dataFile else None,
       'arm': arm.component.name, 'tol_mm': TOL_MM, 'frames': frames, 'bodies': bodies}
with open(os.path.join(OUT, 'manifest.json'), 'w') as f:
    json.dump(man, f, indent=1)
log('%d bodies, %d triangles, worst mesh-vs-solid bbox %.4f mm, frames %s'
    % (len(bodies), sum(b['tris'] for b in bodies), max(b['mesh_bbox_dev'] for b in bodies),
       sorted(frames)))
log('groups: %s' % sorted(set(b['group'] for b in bodies)))
log('isModified before %s after %s' % (mod0, doc.isModified))
if prev is not None and prev.name != DOC:
    prev.activate()
log('active now %r' % (app.activeDocument.name if app.activeDocument else None))
