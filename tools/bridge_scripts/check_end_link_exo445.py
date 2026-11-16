"""EXO-445: J7's hard stops on the end link, measured IN THE MODEL, read-only.

The end link `g_link_7h` carries J7's link stop (armgen.limited_joint at END_STOP_RANGE_DEG). A world
copy of its body (TemporaryBRepManager, the document is not touched) is turned about the J7 axis and
measured against a copy of the J7 stator (`stator/Body1` under the GJ7 wrapper, the part with the
joint stop): minimum distance on a 1 deg scan, then bisection of the first contact on each side to
0.001 deg. Also reports the minimum distance at the zero pose to every other J7 module body.
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/check_end_link_exo445.py --timeout 1800
"""
import math, sys, importlib
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
importlib.reload(armgen)
def say(m): log(str(m).encode('ascii', 'backslashreplace').decode())
doc = app.activeDocument
if doc.name != 'exo generated':
    raise RuntimeError('active document is %r' % doc.name)
mod0 = doc.isModified
root = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType')).rootComponent
arm = root.occurrences.itemByName('generated_R_arm_general:1')
tbm = adsk.fusion.TemporaryBRepManager.get()
mm = app.measureManager
link = arm.childOccurrences.itemByName('g_link_7h:1').bRepBodies.item(0)
wr = arm.childOccurrences.itemByName('GJ7_right_wrist_yaw_joint:1')
mod = wr.childOccurrences.item(0)
o, x, y, z = mod.transform2.getAsCoordinateSystem()
say('J7 axis: origin (%.4f, %.4f, %.4f) mm, Z (%.8f, %.8f, %.8f)' % (o.x * 10, o.y * 10, o.z * 10, z.x, z.y, z.z))
bodies = []
def walk(occ):
    for b in occ.bRepBodies:
        bodies.append((occ.component.name + '/' + b.name, b))
    for c in occ.childOccurrences:
        walk(c)
walk(wr)
stator = [b for n, b in bodies if n == 'stator/Body1']
if len(stator) != 1:
    raise RuntimeError('stator/Body1 under GJ7: %d found' % len(stator))
S = tbm.copy(stator[0])
L0 = tbm.copy(link)
def turned(q):
    t = tbm.copy(L0)
    m = adsk.core.Matrix3D.create()
    m.setToRotation(math.radians(q), z, o)
    tbm.transform(t, m)
    return t
def dist(q, other=S):
    t = turned(q)
    d = mm.measureMinimumDistance(t, other).value * 10.0
    if d < 1e-6:
        i = tbm.copy(t)
        tbm.booleanOperation(i, tbm.copy(other), adsk.fusion.BooleanTypes.IntersectionBooleanType)
        try:
            v = i.volume * 1000.0 if i.faces.count else 0.0
        except Exception:
            v = 0.0
        return -v
    return d
lj_lo, lj_hi = armgen.END_STOP_RANGE_DEG
rows = []
for q in range(int(lj_lo) - 3, int(lj_hi) + 4):
    rows.append((q, dist(float(q))))
say('scan (q deg: min distance to the J7 stator mm, negative = overlap mm3): ' + ', '.join('%+d: %.3f' % r for r in rows))
def contact(q):
    return dist(q) <= 1e-4
def bisect(a, b):   # a clear, b in contact
    for _ in range(18):
        m = (a + b) / 2.0
        if contact(m):
            b = m
        else:
            a = m
    return a, b
lo = bisect(lj_lo + 2.0, lj_lo - 2.0)
hi = bisect(lj_hi - 2.0, lj_hi + 2.0)
say('J7 HARD STOPS measured in the model (first contact end link vs J7 stator): lower between %.4f and %.4f deg, upper between %.4f and %.4f deg (asked %+.1f / %+.1f)'
    % (lo[0], lo[1], hi[0], hi[1], lj_lo, lj_hi))
inside = [d for q, d in rows if lj_lo + 1 <= q <= lj_hi - 1]
say('inside the range (q %+d..%+d): min distance end link vs stator %.3f mm' % (lj_lo + 1, lj_hi - 1, min(inside)))
say('at q 0, distance end link vs each J7 module body (negative = overlap mm3):')
for n, b in bodies:
    try:
        say('  %s: %.3f' % (n, dist(0.0, tbm.copy(b))))
    except Exception as e:
        say('  %s: n/a (%s)' % (n, e))
say('isModified before %s after %s' % (mod0, doc.isModified))
