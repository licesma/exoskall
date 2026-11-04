"""Minimal STEP AP214 reader (no CAD kernel). Regex over the entity list.

What it gives you:
  Step(path)                 parse once; entities kept as raw arg strings
  s.args(eid)                parsed args: refs -> int, numbers -> float, lists -> list
  s.occurrences()            flattened assembly tree: every product occurrence with
                             its world 4x4 transform (follows NAUO / CDSR / IDT and
                             MAPPED_ITEM / REPRESENTATION_MAP)
  s.cylinders(pd)            CYLINDRICAL_SURFACE axes of one product definition, local
  s.points(pd)               CARTESIAN_POINTs of one product definition, local
  s.vertices(pd)             VERTEX_POINT positions only (bbox without control points)

Units are whatever the file uses (Fusion exports mm). Transforms are 4x4 row-major
lists; points are [x, y, z].
"""
import math
import re

_ENT = re.compile(r'#(\d+)=(\w*)\((.*?)\);\n', re.S)
_WS = re.compile(r'\s+')


class Step:
    def __init__(self, path):
        self.path = path
        with open(path, 'r', errors='replace') as f:
            txt = f.read()
        self.ents = {}      # id -> (type, argstr)
        self.by_type = {}   # type -> [id]
        for m in _ENT.finditer(txt):
            eid = int(m.group(1))
            typ = m.group(2)
            a = m.group(3)
            if not typ:  # complex entity: "(A(...)B(...)C())" -> keep first named part
                typ = '+'.join(re.findall(r'([A-Z_0-9]+)\(', a))
            self.ents[eid] = (typ, a)
            self.by_type.setdefault(typ, []).append(eid)
        del txt
        self._acache = {}

    # ---------- args ----------
    def typ(self, eid):
        return self.ents[eid][0]

    def args(self, eid):
        if eid in self._acache:
            return self._acache[eid]
        typ, a = self.ents[eid]
        a = _WS.sub('', a) if '\n' in a else a
        v = _parse_args(a)
        self._acache[eid] = v
        return v

    def complex_part(self, eid, name):
        """Args of one sub-entity of a complex entity, e.g. REPRESENTATION_RELATIONSHIP."""
        typ, a = self.ents[eid]
        a = _WS.sub('', a)
        m = re.search(re.escape(name) + r'\((.*?)\)(?=[A-Z_]|$)', a)
        return _parse_args(m.group(1)) if m else None

    # ---------- geometry primitives ----------
    def point(self, eid):
        return list(self.args(eid)[1])

    def direction(self, eid):
        return list(self.args(eid)[1])

    def placement(self, eid):
        """AXIS2_PLACEMENT_3D -> 4x4 matrix (local -> parent)."""
        a = self.args(eid)
        o = self.point(a[1])
        z = self.direction(a[2]) if a[2] != '$' else [0, 0, 1]
        x = self.direction(a[3]) if len(a) > 3 and a[3] != '$' else _any_perp(z)
        z = _unit(z)
        x = _unit(_sub(x, _scale(z, _dot(x, z))))
        y = _cross(z, x)
        return [[x[0], y[0], z[0], o[0]],
                [x[1], y[1], z[1], o[1]],
                [x[2], y[2], z[2], o[2]],
                [0, 0, 0, 1]]

    # ---------- product tree ----------
    def product_name(self, pd):
        """PRODUCT_DEFINITION -> PRODUCT name."""
        pdf = self.args(pd)[2]                 # PRODUCT_DEFINITION_FORMATION
        prod = self.args(pdf)[2]               # PRODUCT
        return self.args(prod)[0]

    def _index_tree(self):
        if hasattr(self, '_tree'):
            return
        # PD -> its SHAPE_REPRESENTATION
        self.sr_of_pd = {}
        pds_of_pd = {}
        pds_of_nauo = {}
        for e in self.by_type.get('PRODUCT_DEFINITION_SHAPE', []):
            tgt = self.args(e)[2]
            if self.typ(tgt) == 'PRODUCT_DEFINITION':
                pds_of_pd[tgt] = e
            else:
                pds_of_nauo[tgt] = e
        pds_to_pd = {v: k for k, v in pds_of_pd.items()}
        for e in self.by_type.get('SHAPE_DEFINITION_REPRESENTATION', []):
            a = self.args(e)
            if a[0] in pds_to_pd:
                self.sr_of_pd[pds_to_pd[a[0]]] = a[1]
        # NAUO -> transform (via CDSR)
        cdsr_of_pds = {}
        for e in self.by_type.get('CONTEXT_DEPENDENT_SHAPE_REPRESENTATION', []):
            a = self.args(e)
            cdsr_of_pds[a[1]] = a[0]
        self.children = {}   # parent PD -> [(child PD, T, nauo name)]
        for e in self.by_type.get('NEXT_ASSEMBLY_USAGE_OCCURRENCE', []):
            a = self.args(e)
            parent, child = a[3], a[4]
            T = _identity()
            pds = pds_of_nauo.get(e)
            rr = cdsr_of_pds.get(pds) if pds else None
            if rr is not None:
                rrwt = self.complex_part(rr, 'REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION')
                idt = self.args(rrwt[0])
                Ta = self.placement(idt[2])
                Tb = self.placement(idt[3])
                T = _mul(Tb, _inv(Ta))
            self.children.setdefault(parent, []).append((child, T, a[0]))
        all_children = {c for lst in self.children.values() for c, _, _ in lst}
        self.roots = [pd for pd in self.by_type.get('PRODUCT_DEFINITION', [])
                      if pd not in all_children]
        self._tree = True

    def occurrences(self):
        """[(path, product_name, pd, T_world)] for every occurrence, depth-first."""
        self._index_tree()
        out = []

        def walk(pd, T, path):
            name = self.product_name(pd)
            p = path + [name]
            out.append((p, name, pd, T))
            for c, Tc, _ in self.children.get(pd, []):
                walk(c, _mul(T, Tc), p)
        for r in self.roots:
            walk(r, _identity(), [])
        return out

    def find(self, name, occs=None):
        occs = occs if occs is not None else self.occurrences()
        return [o for o in occs if o[1] == name]

    # ---------- own geometry of a product definition ----------
    def _own_reps(self, pd):
        """SHAPE_REPRESENTATION-like reps holding this PD's own geometry, with transforms."""
        self._index_tree()
        sr = self.sr_of_pd.get(pd)
        if sr is None:
            return []
        reps = [(sr, _identity())]
        # ABSRs linked by SHAPE_REPRESENTATION_RELATIONSHIP
        if not hasattr(self, '_srr'):
            self._srr = {}
            for e in self.by_type.get('SHAPE_REPRESENTATION_RELATIONSHIP', []):
                a = self.args(e)
                self._srr.setdefault(a[2], []).append(a[3])
        for r in self._srr.get(sr, []):
            reps.append((r, _identity()))
        return reps

    def _walk_geom(self, pd, want):
        """Generic ref-walk from the PD's reps; MAPPED_ITEM handled as a transform node.
        Yields (eid, T) for entities whose type is in `want`."""
        seen = set()
        stack = [(r, T) for r, T in self._own_reps(pd)]
        while stack:
            eid, T = stack.pop()
            if eid in seen:
                continue
            seen.add(eid)
            typ = self.typ(eid)
            if typ in want:
                yield eid, T
            if typ == 'MAPPED_ITEM':
                a = self.args(eid)
                rmap = self.args(a[1])
                Tm = _mul(self.placement(a[2]), _inv(self.placement(rmap[0])))
                stack.append((rmap[1], _mul(T, Tm)))
                continue
            if typ in ('CARTESIAN_POINT', 'DIRECTION', 'GEOMETRIC_REPRESENTATION_CONTEXT+GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT+GLOBAL_UNIT_ASSIGNED_CONTEXT+REPRESENTATION_CONTEXT',
                       'PRESENTATION_STYLE_ASSIGNMENT'):
                continue
            if typ.startswith('GEOMETRIC_REPRESENTATION_CONTEXT'):
                continue
            for r in _refs(self.ents[eid][1]):
                if r not in seen:
                    stack.append((r, T))

    def cylinders(self, pd):
        """[(center, axis, radius)] in the PD's local frame."""
        out = []
        for eid, T in self._walk_geom(pd, {'CYLINDRICAL_SURFACE'}):
            a = self.args(eid)
            P = self.placement(a[1])
            P = _mul(T, P)
            out.append(([P[0][3], P[1][3], P[2][3]], [P[0][2], P[1][2], P[2][2]], a[2]))
        return out

    def points(self, pd):
        return [_apply(T, self.point(e)) for e, T in self._walk_geom(pd, {'CARTESIAN_POINT'})]

    def vertices(self, pd):
        return [_apply(T, self.point(self.args(e)[1]))
                for e, T in self._walk_geom(pd, {'VERTEX_POINT'})]

    def solids(self, pd):
        return [(self.args(e)[0], e) for e, T in self._walk_geom(pd, {'MANIFOLD_SOLID_BREP'})]


# ---------- arg parsing ----------
_TOK = re.compile(r"'(?:[^']|'')*'|#\d+|[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?|[()$*,]|\.[A-Z_]+\.|[A-Z_]\w*")


def _parse_args(a):
    toks = _TOK.findall(a)
    pos = 0

    def parse_list():
        nonlocal pos
        out = []
        while pos < len(toks):
            t = toks[pos]
            if t == ')':
                pos += 1
                return out
            if t == ',':
                pos += 1
                continue
            if t == '(':
                pos += 1
                out.append(parse_list())
            elif t.startswith('#'):
                out.append(int(t[1:])); pos += 1
            elif t.startswith("'"):
                out.append(t[1:-1].replace("''", "'")); pos += 1
            elif t in ('$', '*'):
                out.append(t); pos += 1
            elif t[0] in '-+.0123456789':
                try:
                    out.append(float(t))
                except ValueError:
                    out.append(t)
                pos += 1
            elif t[0].isalpha():
                # typed value e.g. LENGTH_MEASURE(1.) -> take inner
                pos += 1
                if pos < len(toks) and toks[pos] == '(':
                    pos += 1
                    inner = parse_list()
                    out.append(inner[0] if len(inner) == 1 else inner)
                else:
                    out.append(t)
            else:
                out.append(t); pos += 1
        return out
    return parse_list()


_REF = re.compile(r'#(\d+)')


def _refs(a):
    return [int(x) for x in _REF.findall(a)]


# ---------- tiny linear algebra ----------
def _identity():
    return [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def _mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _inv(T):
    R = [[T[i][j] for j in range(3)] for i in range(3)]
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]
    t = [T[0][3], T[1][3], T[2][3]]
    ti = [-sum(Rt[i][k] * t[k] for k in range(3)) for i in range(3)]
    return [[Rt[0][0], Rt[0][1], Rt[0][2], ti[0]],
            [Rt[1][0], Rt[1][1], Rt[1][2], ti[1]],
            [Rt[2][0], Rt[2][1], Rt[2][2], ti[2]],
            [0, 0, 0, 1]]


def _apply(T, p):
    return [T[i][0] * p[0] + T[i][1] * p[1] + T[i][2] * p[2] + T[i][3] for i in range(3)]


def _rot(T, v):
    return [T[i][0] * v[0] + T[i][1] * v[1] + T[i][2] * v[2] for i in range(3)]


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _scale(a, s):
    return [a[0] * s, a[1] * s, a[2] * s]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _norm(a):
    return math.sqrt(_dot(a, a))


def _unit(a):
    n = _norm(a)
    return [a[0] / n, a[1] / n, a[2] / n]


def _any_perp(z):
    return [0, 0, 1] if abs(z[2]) < 0.9 else [1, 0, 0]


# public aliases
identity, mul, inv, apply, rot, dot, sub, scale, cross, norm, unit = (
    _identity, _mul, _inv, _apply, _rot, _dot, _sub, _scale, _cross, _norm, _unit)
