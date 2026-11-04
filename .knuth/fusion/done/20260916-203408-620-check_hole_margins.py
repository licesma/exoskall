# EXO-124: THE screw-hole gate. Every hole on every generated link, measured on the BUILT
# body, must satisfy all three of:
#
#   1. both ends of the hole open onto PLANAR faces - nothing curved may touch the hole
#      cylinder (a fillet running into a hole leaves no seat at all, and that is the second
#      defect Esteban spotted at the J5 landing);
#   2. each opening has at least `armgen.HOLE_FLAT_RING` mm of flat beyond the hole's edge,
#      measured to the nearest other edge of that face - signed, so a flat that ends INSIDE
#      the hole reads negative instead of bottoming out at zero;
#   3. the number agrees with `armgen.hole_flat_rings`, which the ROUTER solves the route
#      against, to TOL. Two independent computations - one from the route's own curves
#      before anything is built, one off the solid's face edges afterwards - and they have
#      to meet. That is what makes the router's guarantee worth anything (EXO-63).
#
# This runs over EVERY link in the arm, so a link added later cannot regress it, and it
# RAISES on any failure: it is a gate, not a report.
#
# Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/check_hole_margins.py --timeout 300
import sys
import importlib

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import armgen
import BuildArm
import hole_ring_probe
importlib.reload(armgen)
importlib.reload(BuildArm)
importlib.reload(hole_ring_probe)

import adsk.core as ac

TOL = 0.02          # mm: measured vs the route's own number

root = design.rootComponent
arm = next(o for o in root.occurrences if o.component.name.startswith('generated_R_arm'))

# the frames exactly as BuildArm builds them, so the routes are the ones that were drawn
joints, by_child = armgen.parse_urdf(BuildArm.URDF_PATH)
shift, _ = armgen.joint_world_pose(BuildArm.ORIGIN_AT_JOINT, joints, by_child)
FR = {}
for j in BuildArm.JOINTS:
    R, t = armgen.module_frame(j['name'], joints, by_child, j['axis_sign'], j['y_hint'])
    FR[j['label']] = (R, [t[i] - shift[i] for i in range(3)])

fails = []


def check(name, ok, detail=''):
    log('%s %s%s' % ('ok  ' if ok else 'FAIL', name, (' - ' + detail) if detail else ''))
    if not ok:
        fails.append(name)


log('every screw hole on every link: both ends on plane, and %.1f mm of flat ring round each'
    % armgen.HOLE_FLAT_RING)
n_holes = 0
for L in BuildArm.LINKS:
    sub = next((o for o in arm.childOccurrences
                if o.component.name.startswith(L['name'])), None)
    if sub is None or sub.bRepBodies.count == 0:
        continue
    body = sub.bRepBodies.item(0)
    spec = armgen.route(FR[L['a']], FR[L['b']], L['pad_r'], width=L['width'])
    frame = hole_ring_probe.pad_frame(
        next(o for o in arm.childOccurrences
             if o.component.name.startswith(L['a'] + '_')), spec['geometry'], ac)
    log('')
    log('%s (%s route, land fillet r %.3f, far-face round r %.3f):'
        % (L['name'], spec['pattern'], spec['route']['stator_land']['fillet_r'],
           spec['route']['stator_land']['corner_r']))
    n_holes += len(hole_ring_probe.gate(
        L['name'], body, frame, spec, armgen, check, log,
        stop=(L['stop_angle'], L['stop_half'], armgen.STOP_R_IN), tol=TOL))

log('')
log('%d hole openings checked over %d links' % (n_holes, len(BuildArm.LINKS)))
if fails:
    log('HOLE-RING CHECK FAILED: %s' % fails[:6])
    raise RuntimeError('%d screw-hole check(s) failed' % len(fails))
log('hole-ring check: all checks passed')
