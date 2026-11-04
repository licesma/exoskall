"""EXO-208: make the narrowed-joint-stop copy of the module. Nothing else is touched.

`BuildArm.ensure_stop_module` holds the whole recipe (place a copy of the LINKED
`sensor module 2`, breakLink it, rename, cut two wedges off the stator's stop block). This
script is just the trigger, so a failure cannot take the built arm with it.
"""
import sys
import importlib

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
import BuildArm
importlib.reload(armgen)
importlib.reload(BuildArm)

log('JOINT_STOP_DEG = %.4f, centre %.5f deg (measured), target %.5f .. %.5f'
    % (armgen.JOINT_STOP_DEG, 0.0 + armgen.JOINT_STOP_LO_DEG / 1.0 * 0 + 168.00225,
       armgen.JOINT_STOP_LO_DEG, armgen.JOINT_STOP_HI_DEG))
comp, how = BuildArm.ensure_stop_module(design, log)
log('component: %r' % comp.name)
log('how: %s' % how)
log('children: %s' % ', '.join('%s(%d bodies)' % (comp.occurrences.item(i).component.name,
                                                  comp.occurrences.item(i).component.bRepBodies.count)
                               for i in range(comp.occurrences.count)))
