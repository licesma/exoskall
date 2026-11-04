"""EXO-218: enough of Fusion's `adsk` to IMPORT `BuildArm` / `BuildArmGen` outside Fusion.

`tools/luna_solve.py` needs the arm SPECS those two files hold - the joint and link tables -
and nothing else; every function it calls is in `armgen` / `sweepgen`, which are pure Python
already. Importing the two Fusion files only needs the `adsk` names their module level
touches, which is what this is. Calling anything through it does nothing useful and is not
meant to: this stub is for reading the tables, never for building.
"""


def doEvents():
    pass
