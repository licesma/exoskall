"""EXO-262: re-measure the EXO-261 example parts against the current box, without moving
them: min distance to each bin wall face, to the box body, and interference.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/measure_parts_exo262.py
"""
root = design.rootComponent
box = root.bRepBodies.item(0)
mm = app.measureManager
BIN_X = (-9.8, -5.05)
WALLS_Y = {'bearing_5972K159_exo261': (3.4, 9.8), 'XL330_exo261': (-9.8, -3.4)}


def faces_at(axis, coord):
    out = []
    for f in box.faces:
        pl = adsk.core.Plane.cast(f.geometry)
        if pl and abs(abs(getattr(pl.normal, axis)) - 1) < 1e-6 and abs(getattr(pl.origin, axis) - coord) < 1e-4:
            out.append(f)
    return out


coll = adsk.core.ObjectCollection.create()
coll.add(box)
for occ in root.occurrences:
    name = occ.component.name
    bodies = list(occ.bRepBodies)
    for b in bodies:
        coll.add(b)
    wy = WALLS_Y[name]
    res = []
    for tag, ax, co, sa, sp in (('-X', 'x', BIN_X[0], 'y', wy), ('+X', 'x', BIN_X[1], 'y', wy),
                                ('-Y', 'y', wy[0], 'x', BIN_X), ('+Y', 'y', wy[1], 'x', BIN_X)):
        fs = [f for f in faces_at(ax, co)
              if getattr(f.boundingBox.maxPoint, sa) >= sp[0] and getattr(f.boundingBox.minPoint, sa) <= sp[1]]
        d = min(mm.measureMinimumDistance(b, f).value for b in bodies for f in fs) if fs else None
        res.append('%s %s' % (tag, '?' if d is None else '%.3f' % (10 * d)))
    floor = 10 * min(mm.measureMinimumDistance(b, box).value for b in bodies)
    log('CLEARANCE %s (mm): %s; min distance to box %.3f' % (name, ', '.join(res), floor))
ii = design.createInterferenceInput(coll)
ii.areCoincidentFacesIncluded = False
ir = design.analyzeInterference(ii)
hits = [(r.entityOne.name, r.entityTwo.name) for r in ir if box in (r.entityOne, r.entityTwo)]
log('interference with box: %s' % (hits or 'none'))
log('document %r version %s' % (app.activeDocument.name, app.activeDocument.dataFile.versionNumber))
