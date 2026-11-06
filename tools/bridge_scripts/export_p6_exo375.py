"""EXO-375: export the pad-6 links (the arm must be built with run_buildarm_general_p6_exo375.py);
bottom.3mf is not touched. Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/export_p6_exo375.py"""
WITH_BOTTOM = False
exec(open('/Users/esteban/repos/exo/tools/bridge_scripts/export_exo370.py').read())
