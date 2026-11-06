"""EXO-375/383: export the links of a VARIANT arm (pad 6 in EXO-375; counterbore 1.5 / 2.0 in
EXO-383, built with run_cbore_*_exo383.py), named by what the solid reads back; bottom.3mf is not touched. Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/export_p6_exo375.py"""
WITH_BOTTOM = False
exec(open('/Users/esteban/repos/exo/tools/bridge_scripts/export_exo370.py').read())
