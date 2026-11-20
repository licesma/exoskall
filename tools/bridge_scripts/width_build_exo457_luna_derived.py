"""EXO-457 wrapper: build luna at WIDTH_MODE 'derived' into its own new document (see width_build_exo457.py).
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/width_build_exo457_luna_derived.py --timeout 2700
"""
ARM, MODE = 'luna', 'derived'
exec(compile(open('/Users/esteban/repos/exo/tools/bridge_scripts/width_build_exo457.py').read(),
             'width_build_exo457.py', 'exec'))
