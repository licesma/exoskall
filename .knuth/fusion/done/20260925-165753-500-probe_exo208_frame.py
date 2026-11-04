"""EXO-208 probe: which frame does a sketch made in a CHILD component draw in?

Two identical test extrusions - a small cylinder at (24, 0) - one made in the module
component (at root identity) and one made in the stator's own child component (offset
z -7.5 mm), both asked for module z = -1.5. Read back where the solid actually landed,
then delete both. Nothing else is touched.
"""
import math
import sys
import importlib

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
import BuildArm
importlib.reload(armgen)
importlib.reload(BuildArm)

root = design.rootComponent
comp = BuildArm.module_stop_component(design)
log('stop module component: %r' % (comp.name if comp else None))
occ = None
for i in range(root.occurrences.count):
    o = root.occurrences.item(i)
    log('root occ %-34r comp=%r' % (o.name, o.component.name))
    if comp is not None and o.component.name == comp.name:
        occ = o
if comp is None or occ is None:
    raise RuntimeError('no stop module occurrence to probe')

st_occ = BuildArm.module_stator_occ(comp)
st = st_occ.component
log('stator component %r, occ transform translation (%.4f, %.4f, %.4f) cm'
    % (st.name, st_occ.transform.translation.x, st_occ.transform.translation.y,
       st_occ.transform.translation.z))
nb = st.bRepBodies.item(0).boundingBox
log('  body bbox in the stator COMPONENT space: z %.3f .. %.3f mm'
    % (nb.minPoint.z * 10, nb.maxPoint.z * 10))
pb = BuildArm.module_stator_occ(occ, in_context=True).bRepBodies.item(0).boundingBox
log('  body bbox through the ROOT proxy:        z %.3f .. %.3f mm'
    % (pb.minPoint.z * 10, pb.maxPoint.z * 10))
log('  features already in %r: %d; sketches %d' % (comp.name, comp.features.count,
                                                   comp.sketches.count))
log('  features already in %r: %d; sketches %d' % (st.name, st.features.count,
                                                   st.sketches.count))

NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
Z_WANT = -1.5


def probe(target, label, dz):
    sk = BuildArm.PlaneSketch(target, 'exo208 frame probe %s' % label,
                              adsk.core.Point3D.create(0.0, 0.0,
                                                       (Z_WANT + dz) * BuildArm.MM_TO_CM),
                              BuildArm.V(1, 0, 0), BuildArm.V(0, 1, 0))
    sk.circle((24.0, 0.0), 2.0)
    feat = sk.extrude(1, 1.0, NEW)
    b = feat.bodies.item(0)
    bb = b.boundingBox
    log('%s: asked module z %.3f (dz %+.3f) -> body z %.3f..%.3f, x %.3f..%.3f in the '
        'component\'s own space' % (label, Z_WANT, dz, bb.minPoint.z * 10, bb.maxPoint.z * 10,
                                    bb.minPoint.x * 10, bb.maxPoint.x * 10))
    try:
        pxy = b.createForAssemblyContext(occ) if target.name != comp.name else b
        pbb = pxy.boundingBox
        log('        through the root proxy: z %.3f..%.3f, x %.3f..%.3f'
            % (pbb.minPoint.z * 10, pbb.maxPoint.z * 10, pbb.minPoint.x * 10,
               pbb.maxPoint.x * 10))
    except Exception as e:
        log('        proxy read failed: %s' % e)
    feat.deleteMe()
    sk.sketch.deleteMe()
    sk.cplane.deleteMe()


probe(comp, 'module component', 0.0)
probe(st, 'stator child component (dz = 0)', 0.0)
probe(st, 'stator child component (dz = +7.5)', -st_occ.transform.translation.z * 10.0)
