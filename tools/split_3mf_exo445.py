#!/usr/bin/env python3
"""EXO-445: split the six bodies of wrist clamp 07 (parts/hand_candidates/07_wrist_clamp_v12.3mf,
EXO-434: one closed mesh object per body, the B-rep copies in `/hand/candidates/07 wrist_clamp_v12`,
arm frame) into one 3MF per printable piece, so each can go on a Bambu plate as its own object.
The mesh of each object is copied VERBATIM (vertices and triangles, same text); nothing is
re-meshed or edited. Each output has one object (id 1), one build item, no transform.

    python3 -I tools/split_3mf_exo445.py            # writes parts/clamp07_*.3mf (refuses to overwrite)
    python3 -I tools/split_3mf_exo445.py --check    # re-derives every piece and compares the bytes
"""
import os, re, sys, zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, 'parts', 'hand_candidates', '07_wrist_clamp_v12.3mf')
# body name in 07 -> output part name (EXO-438 names the pieces)
PIECES = [('side0 Body1', 'clamp07_side0_nuts'),     # side0 half with the M4 nut pockets (+Y in the J7 frame)
          ('side0 Body3', 'clamp07_side0_heads'),    # side0 half with the O8 head counterbores (-Y)
          ('side0 Body4', 'clamp07_button'),
          ('lock', 'clamp07_lock'),
          ('spring', 'clamp07_spring'),              # the V leaf spring, legs 1.5
          ('Component107', 'clamp07_hook')]
HEAD = ('<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" '
        'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">\n'
        '<metadata name="Title">%s</metadata>\n<resources>\n')


def pieces():
    z = zipfile.ZipFile(SRC)
    model = z.read('3D/3dmodel.model').decode()
    objs = dict((m.group(2), m.group(0)) for m in
                re.finditer(r'<object id="(\d+)" name="([^"]+)" type="model">.*?</object>', model, re.S))
    out = {}
    for body, part in PIECES:
        obj = re.sub(r'^<object id="\d+"', '<object id="1"', objs[body])
        out[part] = {'[Content_Types].xml': z.read('[Content_Types].xml'), '_rels/.rels': z.read('_rels/.rels'),
                     '3D/3dmodel.model': (HEAD % part + obj + '\n</resources><build>\n<item objectid="1"/>\n'
                                          '</build></model>\n').encode()}
    return out


def write(part, files):
    path = os.path.join(ROOT, 'parts', part + '.3mf')
    with zipfile.ZipFile(path, 'w', zipfile.ZIP_DEFLATED) as zf:
        for n in ('[Content_Types].xml', '_rels/.rels', '3D/3dmodel.model'):
            zf.writestr(n, files[n])
    return path


if __name__ == '__main__':
    P = pieces()
    for part, files in P.items():
        path = os.path.join(ROOT, 'parts', part + '.3mf')
        if '--check' in sys.argv:
            z = zipfile.ZipFile(path)
            same = all(z.read(n) == files[n] for n in files)
            print('%s: %s' % (part, 'identical to the split of 07' if same else 'DIFFERS'))
            assert same
        else:
            if os.path.exists(path):
                raise SystemExit('exists, not overwriting: %s' % path)
            print('wrote', write(part, files))
