"""EXO-457: interference through the JOINT RANGES, on the built solids, for one arm build.

Input: a directory written by `tools/bridge_scripts/width_export_exo457.py` (manifest.json + one
STL per body, world mm, exported from the EXO-457 width-check documents). Pure geometry, no
Fusion: python-fcl BVH meshes for contact, manifold3d for intersection volumes. Needs numpy,
trimesh, python-fcl, manifold3d; the EXO-457 run used a throwaway venv:
    /usr/bin/python3 -m venv /tmp/exo457_venv
    /tmp/exo457_venv/bin/pip install numpy trimesh python-fcl manifold3d scipy
    /tmp/exo457_venv/bin/python -I tools/width_sweep_exo457.py <export dir> <g1|luna> <manual|derived> <out.json>

Kinematics. Joint q turns everything DOWNSTREAM of it about the placed module's own Z axis (the
module frame read off the solids), by axis_sign * q, axis_sign = sign(module Z . URDF axis):
  upstream (static)  every link and module before the joint, and the joint's own module STATOR
                     side (stator, XL330 housing, inserts, bearing races);
  downstream (turns) the joint's own module ROTOR side (inner_bottom, horn screws, the M3 bolts
                     and nuts joining it to the fused rotor top), the link leaving the joint and
                     everything after it.
Pairs inside one module are never checked (the module's own mechanism). A pair already in
contact at the built pose (q = 0) is a BASELINE pair: reported, with its intersection volume
tracked on a grid, never counted as a clash unless that volume grows.

Single-joint sweeps: q from 0 outward in STEP_DEG steps to RANGE_PAD_DEG past each stop limit;
the first step with a NEW contact (not baseline) is bisected to 0.005 deg. The intended first
contact is the joint's link stop block (on the leaving link) against its own module's stator.
Two-joint sweeps (adjacent joints k, k+1): only pairs whose relative pose depends on BOTH
(upstream of k vs downstream of k+1), over both stop ranges at PAIR_STEP_DEG.
Three-joint sweeps likewise for consecutive triples at TRIPLE_STEP_DEG.
"""
import json
import math
import os
import sys
import time

import numpy as np
import trimesh
import fcl
import manifold3d

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '_adsk_stub'))
sys.path.insert(0, os.path.join(HERE, '..', 'fusion_scripts', 'BuildArm'))
import armgen  # noqa: E402
import BuildArmGen  # noqa: E402

STEP_DEG = 0.25
RANGE_PAD_DEG = 3.0
PAIR_STEP_DEG = 2.0
TRIPLE_STEP_DEG = 6.0
ROTOR_PARTS = ('inner_bottom', 'screw', 'bolt', 'nut')      # module parts that turn with the rotor


def is_rotor_part(part):
    p = part.lower()
    return any(t in p for t in ROTOR_PARTS)


def rot(axis, ang, origin):
    """4x4 rotation by `ang` rad about the line (origin, axis)."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K
    T = np.eye(4)
    T[:3, :3] = R
    o = np.asarray(origin, float)
    T[:3, 3] = o - R @ o
    return T


class Arm:
    def __init__(self, d, key, mode):
        self.dir = d
        self.man = json.load(open(os.path.join(d, 'manifest.json')))
        spec = BuildArmGen.G1_SPEC if key == 'g1' else BuildArmGen.LUNA_SPEC
        links = BuildArmGen.g1_links(mode) if key == 'g1' else BuildArmGen.luna_links(mode)
        joints, by_child = armgen.parse_urdf(spec['urdf'])
        self.order = [j['label'] for j in list(spec['fixed']) + list(spec['derived'])]
        name_of = dict((j['label'], j['name']) for j in list(spec['fixed']) + list(spec['derived']))
        self.links = [(L['name'], L['a'], L['b'], L['pad_r']) for L in links]
        self.end = spec.get('end_link')
        fr = self.man['frames']
        self.joint = {}
        for lab in self.order:
            f = fr[lab]
            _t, ax = armgen.joint_world_pose(name_of[lab], joints, by_child)
            dz = float(np.dot(f['z'], ax))
            if abs(abs(dz) - 1) > 1e-6:
                raise RuntimeError('%s: module Z . URDF axis = %.8f' % (lab, dz))
            sign = 1 if dz > 0 else -1
            jd = dict(joints[name_of[lab]], name=name_of[lab])
            info = {'axis': f['z'], 'origin': f['o'], 'sign': sign,
                    'urdf': (math.degrees(jd['lower']), math.degrees(jd['upper']))}
            pad = [L[3] for L in self.links if L[1] == lab]
            if self.end is not None and self.end['joint'] == lab:
                lj = armgen.limited_joint(armgen.end_stop_joint(jd, self.end['range_deg']), sign, armgen.END_PAD_R)
                info['stop'] = (math.degrees(lj['lower']), math.degrees(lj['upper']))
                info['stop_why'] = 'end link range override %s' % (self.end['range_deg'],)
            elif pad:
                lj = armgen.limited_joint(jd, sign, pad[0])
                info['stop'] = (math.degrees(lj['lower']), math.degrees(lj['upper']))
                info['stop_why'] = 'capped (%s)' % lj['cap_why'][:60] if lj['capped'] else 'URDF'
            else:
                info['stop'] = None
                info['stop_why'] = 'no link carries a stop (last joint): URDF range swept'
            self.joint[lab] = info
        # body -> chain position: the index of the first joint that moves it
        self.bodies = self.man['bodies']
        leave = dict((L[0], L[1]) for L in self.links)
        if self.end is not None:
            leave[self.end['name']] = self.end['joint']
        idx = dict((lab, i) for i, lab in enumerate(self.order))
        for b in self.bodies:
            g = b['group']
            if g.startswith('M'):
                lab = g[1:]
                b['module'] = lab
                # stator side moves with the joint BEFORE it; rotor side with its own joint
                b['pos'] = idx[lab] + 1 if is_rotor_part(b['part']) else idx[lab]
                b['side'] = 'rotor' if is_rotor_part(b['part']) else 'stator'
            else:
                b['module'] = None
                b['pos'] = idx[leave[g]] + 1
                b['side'] = 'link'
        self.objs, self.mesh = [], []
        for b in self.bodies:
            m = trimesh.load(os.path.join(d, b['file']), process=True)
            self.mesh.append(m)
            bvh = fcl.BVHModel()
            bvh.beginModel(len(m.vertices), len(m.faces))
            bvh.addSubModel(m.vertices, m.faces)
            bvh.endModel()
            self.objs.append(fcl.CollisionObject(bvh, fcl.Transform()))
        self.corners = []
        for m in self.mesh:
            lo, hi = m.vertices.min(axis=0), m.vertices.max(axis=0)
            self.corners.append(np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                                          for z in (lo[2], hi[2])]))
        self.label = ['%s:%s' % (b['group'], b['name']) for b in self.bodies]

    def skip(self, i, j):
        bi, bj = self.bodies[i], self.bodies[j]
        return bi['module'] is not None and bi['module'] == bj['module']

    def pose(self, qs):
        """qs: {label: q deg}. The 4x4 world transform of every body: the product of
        exponentials, each joint a rotation about its BUILT-POSE axis, base joint outermost."""
        T = [np.eye(4)]
        acc = np.eye(4)
        for lab in self.order:
            q = qs.get(lab, 0.0)
            if q:
                J = self.joint[lab]
                acc = acc @ rot(J['axis'], math.radians(J['sign'] * q), J['origin'])
            T.append(acc.copy())
        return [T[b['pos']] for b in self.bodies]

    def contacts(self, Ts, A, B):
        """Colliding pairs (i, j), i in A, j in B, at transforms Ts. Broad phase: world AABBs of
        each body's local box corners (conservative); narrow phase: fcl.collide on the BVH
        meshes (touching faces count as contact)."""
        boxes = {}
        for k in set(A) | set(B):
            T = Ts[k]
            c = self.corners[k] @ T[:3, :3].T + T[:3, 3]
            boxes[k] = (c.min(axis=0), c.max(axis=0))
        out = set()
        Bl = list(B)
        blo = np.array([boxes[j][0] for j in Bl])
        bhi = np.array([boxes[j][1] for j in Bl])
        for i in A:
            lo, hi = boxes[i]
            ok = np.all((blo <= hi + 1e-6) & (bhi >= lo - 1e-6), axis=1)
            for n in np.nonzero(ok)[0]:
                j = Bl[n]
                if i == j or self.skip(i, j):
                    continue
                for k in (i, j):
                    T = Ts[k]
                    self.objs[k].setTransform(fcl.Transform(T[:3, :3], T[:3, 3]))
                if fcl.collide(self.objs[i], self.objs[j], fcl.CollisionRequest(), fcl.CollisionResult()):
                    out.add((min(i, j), max(i, j)))
        return out

    def inter_vol(self, i, j, Ts):
        def man(k):
            m = self.mesh[k].copy()
            m.apply_transform(Ts[k])
            return manifold3d.Manifold(manifold3d.Mesh(vert_properties=np.asarray(m.vertices, np.float32),
                                                       tri_verts=np.asarray(m.faces, np.uint32)))
        return float((man(i) ^ man(j)).volume())


def chain_cross(arm, k_lo, k_hi):
    """Body index sets: upstream of joint index k_lo, downstream of joint index k_hi."""
    A = [i for i, b in enumerate(arm.bodies) if b['pos'] <= k_lo]
    C = [i for i, b in enumerate(arm.bodies) if b['pos'] >= k_hi + 1]
    return A, C


def single(arm, lab, log):
    k = arm.order.index(lab)
    A, D = chain_cross(arm, k, k)
    J = arm.joint[lab]
    rng = J['stop'] or J['urdf']
    base = arm.contacts(arm.pose({}), A, D)
    res = {'joint': lab, 'stop': J['stop'], 'stop_why': J['stop_why'], 'urdf': J['urdf'],
           'axis_sign': J['sign'], 'n_up': len(A), 'n_down': len(D),
           'baseline': [arm.label[i] + ' <-> ' + arm.label[j] for i, j in sorted(base)]}
    for direction, limit in ((+1, rng[1]), (-1, rng[0])):
        end = limit + direction * RANGE_PAD_DEG
        q, prev, hit = 0.0, 0.0, None
        n = 0
        while (q - end) * direction < 0:
            q = q + direction * STEP_DEG
            if (q - end) * direction > 0:
                q = end
            n += 1
            new = arm.contacts(arm.pose({lab: q}), A, D) - base
            if new:
                hit = (prev, q, new)
                break
            prev = q
        if hit is None:
            res['free_to_%s' % ('upper' if direction > 0 else 'lower')] = {
                'free_through': q, 'contact': None, 'steps': n}
            continue
        a, b, new = hit
        for _ in range(12):
            m = (a + b) / 2
            got = arm.contacts(arm.pose({lab: m}), A, D) - base
            if got:
                b, new = m, got
            else:
                a = m
        pairs = sorted(arm.label[i] + ' <-> ' + arm.label[j] for i, j in new)
        res['free_to_%s' % ('upper' if direction > 0 else 'lower')] = {
            'free_through': a, 'first_contact': b, 'pairs': pairs, 'steps': n,
            'vs_stop_deg': b - limit}
    # baseline pairs: intersection volume at 0 and at both ends of the stop range
    bl = []
    for i, j in sorted(base):
        row = {'pair': arm.label[i] + ' <-> ' + arm.label[j]}
        for q in (0.0, rng[0], rng[1]):
            try:
                row['vol_%+.1f' % q] = arm.inter_vol(i, j, arm.pose({lab: q}))
            except Exception as e:  # noqa: BLE001
                row['vol_%+.1f' % q] = 'n/a %s' % e
        bl.append(row)
    res['baseline_vol'] = bl
    log('%s: stop %s (%s), up %s, down %s, baseline %d'
        % (lab, J['stop'], J['stop_why'], res.get('free_to_upper'), res.get('free_to_lower'), len(base)))
    return res


def grid(arm, labs, step, log, shrink=0.5):
    """Sweep consecutive joints `labs` together; only pairs that depend on ALL of them."""
    ks = [arm.order.index(l) for l in labs]
    A, C = chain_cross(arm, ks[0], ks[-1])
    base = arm.contacts(arm.pose({}), A, C)
    axes = []
    for l in labs:
        J = arm.joint[l]
        lo, hi = J['stop'] or J['urdf']
        lo, hi = lo + shrink, hi - shrink
        n = max(1, int(math.ceil((hi - lo) / step)))
        axes.append([lo + (hi - lo) * t / n for t in range(n + 1)])
    hits, count = [], 0
    t0 = time.time()

    def rec(i, qs):
        nonlocal count
        if i == len(labs):
            count += 1
            new = arm.contacts(arm.pose(qs), A, C) - base
            if new:
                hits.append((dict(qs), sorted(arm.label[a] + ' <-> ' + arm.label[b] for a, b in new)))
            return
        for q in axes[i]:
            qs[labs[i]] = q
            rec(i + 1, qs)
    rec(0, {})
    log('%s: %d poses in %.0f s, %d with a contact, %d baseline pairs'
        % ('+'.join(labs), count, time.time() - t0, len(hits), len(base)))
    pairs = {}
    for qs, ps in hits:
        for p in ps:
            pairs.setdefault(p, []).append(qs)
    summary = {}
    for p, ql in pairs.items():
        summary[p] = {'poses': len(ql), 'example': ql[0],
                      'q_ranges': dict((l, (min(q[l] for q in ql), max(q[l] for q in ql))) for l in labs)}
    return {'joints': labs, 'step': step, 'poses': count, 'n_up': len(A), 'n_down': len(C),
            'baseline': [arm.label[i] + ' <-> ' + arm.label[j] for i, j in sorted(base)],
            'poses_in_contact': len(hits), 'pairs': summary}


def main():
    d, key, mode, out = sys.argv[1:5]
    what = sys.argv[5] if len(sys.argv) > 5 else 'all'
    lines = []

    def log(m):
        print(m, flush=True)
        lines.append(m)
    t0 = time.time()
    arm = Arm(d, key, mode)
    log('%s %s: %d bodies (%d triangles), tol %.3f mm, doc %r v%s'
        % (key, mode, len(arm.bodies), sum(b['tris'] for b in arm.bodies), arm.man['tol_mm'],
           arm.man['doc'], arm.man['version']))
    for lab in arm.order:
        J = arm.joint[lab]
        log('  %s axis_sign %+d URDF %+.3f..%+.3f stop %s (%s)' % (lab, J['sign'], J['urdf'][0], J['urdf'][1],
                                                               J['stop'], J['stop_why']))
    res = {'key': key, 'mode': mode, 'doc': arm.man['doc'], 'version': arm.man['version'],
           'step_deg': STEP_DEG, 'pad_deg': RANGE_PAD_DEG, 'single': [], 'pairs': [], 'triples': []}
    if what in ('all', 'single'):
        for lab in arm.order:
            res['single'].append(single(arm, lab, log))
    if what in ('all', 'pairs'):
        for i in range(len(arm.order) - 1):
            res['pairs'].append(grid(arm, arm.order[i:i + 2], PAIR_STEP_DEG, log))
    if what in ('all', 'triples'):
        for i in range(len(arm.order) - 2):
            res['triples'].append(grid(arm, arm.order[i:i + 3], TRIPLE_STEP_DEG, log))
    res['seconds'] = time.time() - t0
    json.dump(res, open(out, 'w'), indent=1)
    log('done in %.0f s -> %s' % (time.time() - t0, out))


if __name__ == '__main__':
    main()
