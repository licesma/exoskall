"""Build `generated_R_arm_general` into a document of its own (EXO-196, SAVED by EXO-242).

`BuildArmGen.build_general` needs exactly two things from the document: a component called
`sensor module 2` to place beside every joint, and somewhere to put the links. It never
reads the hand-made arm (the deviation section just reports 'nothing to compare against').
So when no design is open the honest move is a document of this script's own making, with
the module inserted from the project, not somebody's saved file opened and written to.

EXO-242: that document is now SAVED BEFORE the build, not left in memory. The original
version of this script made a FRESH, UNSAVED document, and that is precisely how closing
Fusion threw away both arms and `sensor module 2 v3`. `new_doc_exo242.py` makes and saves
the document (idempotently, re-using `BuildArm.GEN_DOC_NAME` if it is already open) and
this script runs it first.

Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/new_doc_general.py --timeout 2700
"""
import importlib
import sys

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen
import sweepgen
import BuildArm
import BuildArmGen
importlib.reload(armgen)
importlib.reload(sweepgen)
importlib.reload(BuildArm)
importlib.reload(BuildArmGen)

HERE = '/Users/esteban/repos/exo/tools/bridge_scripts/new_doc_exo242.py'
with open(HERE) as f:
    src = f.read()
g = dict(globals())
g['__file__'] = HERE
exec(compile(src, HERE, 'exec'), g)

des = adsk.fusion.Design.cast(app.activeProduct)
log('DOCUMENT before the build:')
log(BuildArm.save_active_document(app))
log(BuildArmGen.build_general(app, ui, des, log))
log('DOCUMENT after the build:')
log(BuildArm.save_active_document(app))
