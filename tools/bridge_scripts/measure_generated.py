#!/usr/bin/env python3
"""Measure the generated arm STEP and compare module frames with the exo STEP.

Not a bridge script - plain python3, run from the repo root:
    /usr/bin/python3 tools/bridge_scripts/measure_generated.py
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import math
import steplib

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
GEN = os.path.join(REPO, 'gento_luna/exo/exports/generated_R_arm_only.step')
EXO = os.path.join(REPO, 'gento_luna/exo/exports/exo_skeleton_2026-08-08.step')


def frames(path, names=None, depth=None):
    s = steplib.Step(path)
    out = []
    for p, name, pd, T in s.occurrences():
        if names and name not in names:
            continue
        if depth and len(p) > depth:
            continue
        out.append((p, name, [T[0][3], T[1][3], T[2][3]],
                    [T[0][2], T[1][2], T[2][2]]))
    return out


print('--- generated arm (component export, world == arm frame) ---')
gen = {}
for p, name, t, z in frames(GEN, depth=3):
    if len(p) == 2 and 'sensor' not in name:
        continue
    if len(p) == 3 and not name.startswith('sensor module 2'):
        continue
    print('%-22s %-18s pos=(%8.3f, %8.3f, %8.3f) Zcol=(%6.3f, %6.3f, %6.3f)'
          % (p[-2] if len(p) > 1 else '', name, t[0], t[1], t[2], z[0], z[1], z[2]))
    if name.startswith('sensor module 2'):
        gen[len(gen)] = (t, z)

print()
print('--- exo STEP Module 0/1/2 R ---')
exo = []
for p, name, t, z in frames(EXO, names={'Module 0 R', 'Module 1 R', 'Module 2 R'}):
    exo.append((name, t, z))
    print('%-12s pos=(%8.3f, %8.3f, %8.3f) Zcol=(%6.3f, %6.3f, %6.3f)'
          % (name, t[0], t[1], t[2], z[0], z[1], z[2]))
exo.sort(key=lambda e: e[0])

print()
print('--- deltas (generated - exo), mm and degrees ---')
print('%-4s %-12s %-28s %8s %8s' % ('exo', 'joint', 'dpos (x, y, z)', '|dpos|', 'axis'))
labels = ['J1', 'J2', 'J3']
for k, (name, te, ze) in enumerate(exo):
    tg, zg = gen[k]
    d = [tg[i] - te[i] for i in range(3)]
    n = math.sqrt(sum(v * v for v in d))
    c = abs(sum(zg[i] * ze[i] for i in range(3)))          # axis line, sign ignored
    ang = math.degrees(math.acos(max(-1.0, min(1.0, c))))
    print('%-4s %-12s (%7.3f, %7.3f, %7.3f) %8.3f %7.3f deg'
          % (name.split()[1], labels[k], d[0], d[1], d[2], n, ang))
