"""The general link method (EXO-144): ONE body primitive and ONE landing primitive.

A link is a **ribbon** - a rectangle `width` x `thickness` - swept along a **3D centre
line** made of straight runs joined by tangent bends in ANY plane, from inside the rotor
pad disk of joint N to inside the stator landing plate of joint N+1.

There are no named shapes here. `sweep_route` sizes the same route for every pair; the
'bar', 'L' and 'ramp' of `armgen` are three answers this one route gives when the pair's
own numbers differ. Nothing branches on a pattern name, and nothing is copied from a
hand-built link: every radius, thickness and margin below is a DEFAULT, stated as such.

Frames (all of `armgen`'s, unchanged):
  A   the rotor-pad module frame of joint N. The route is computed in A's (s, w, z):
      s along `u` (the stator axis projected into the pad plane, pointing at the landing),
      w = z_A x u, z along the pad axis.
  B   the stator module frame of joint N+1. The landing plate lies on its underside.

The three things that are non-negotiable and how each is met:
  * module origin + axis = the URDF joint          - untouched, `armgen.module_frame`
  * every screw hole keeps `HOLE_FLAT_RING`        - BY CONSTRUCTION on three of the four
      seating faces (see `hole_rings`), measured on the fourth
  * the actuator cutout is cleared                 - `LAND_WEB` is solved into the landing
      height, not asserted afterwards
"""
import math

import armgen as g


# ---------------- defaults (not measured constants) ----------------
# The ribbon is the pad's own thickness, so where it leaves the rotor pad it is FLUSH with
# both pad faces and the join needs no ledge, no round and no relief: the pad disk and the
# ribbon are the same 8 mm plate. That is what makes the rotor pad's two seating faces flat
# out to the rim by construction.
RIBBON_T = g.PAD_T              # 8.0 mm
BEND_R = 15.0                   # default centre-line radius of every bend
WALL_R_MIN = 5.0                # the tightest radius any boundary of a bend may be left at
LAND_WEB = 3.0                  # slab left between the actuator cutout and the ribbon
RIM_MARGIN = 1.0                # the ribbon stays in the pad plane this far past the pad rim
PLAN_MARGIN = 1.0               # straight left between the plan bend and whatever follows
MIN_RUN = 1.0                   # shortest straight run the route may contain
BURY = g.PAD_T / 2.0            # how deep the ribbon dies inside the landing plate
AXIS_SQUARE_MAX = 1e-2          # rad; the landing plate's normal must lie in the pad plane
CLEAR_R = max(g.MODULE_R, g.STATOR_PAD_R)       # 27: module A's stator / the previous slab
CLEAR_Z = g.MODULE_STOP_Z_TOP                   # -0.4: below this the r 27 cylinder is there
# The link's own stop block sits diametrically opposite the module's stator stop block
# (158..178 deg in the module frame, exo-joint.md), which is the block it collides with:
# 168 + 180. Joint limits are deferred (EXO-41), so this is placement, not a limit.
STOP_ANGLE = 348.0
# EXO-214, kept by EXO-218: how far the EXO-181 pad chord has to stay off the link stop. The
# chord takes a straight slice off the rotor pad; the stop is what the joint stop actually
# meets, so a chord that reaches it takes the joint's own limit with it. 1.0 mm is
# MIN_CLEARANCE, the same bar every other body-to-body number in this generator is held to;
# it is a default, stated here, and there is no per-joint version of it. Only the MARGIN is
# EXO-214's - what it is measured to went back to the one solid block (EXO-218).
STOP_CHORD_MARGIN = g.MIN_CLEARANCE


# ---------------- small geometry helpers ----------------

def _rot2(p, a):
    c, s = math.cos(a), math.sin(a)
    return (p[0] * c - p[1] * s, p[0] * s + p[1] * c)


def _add(p, q):
    return tuple(p[i] + q[i] for i in range(len(p)))


def _mul(p, k):
    return tuple(v * k for v in p)


def _arc_mid(p0, p1, c, r):
    """A point on the minor arc from p0 to p1 about centre c (3D, radius r)."""
    m = tuple((p0[i] + p1[i]) / 2.0 for i in range(3))
    d = tuple(m[i] - c[i] for i in range(3))
    n = math.sqrt(sum(v * v for v in d))
    if n < 1e-12:
        raise ValueError('arc of %.3f mm is a half turn: no unique mid point' % r)
    return tuple(c[i] + d[i] * r / n for i in range(3))


# ---------------- the route ----------------
# EXO-158: the ORDER of the moves is not fixed. The level change may be taken on the final
# leg (the run between the pad and the plate), on the LATERAL departure leg, or split across
# the two; `sweep_route` sizes each placement and picks by rule. Nothing about the primitives
# changes - it is the same two tangent bends in the same vertical plane, on a different leg.
LEVEL_ORDERS = ('final', 'lateral', 'split')

# EXO-176: the two LANDING primitives, tried in this order. `square` is EXO-144's - the
# tongue is a strip parallel to the plate frame's own +Y, which `landing_numbers` pins to the
# rotor pad axis. `angled` frees the tongue's direction inside the plate plane and aims it
# from the disk centre at the point where the ribbon enters that plane; theta = 0 IS the
# square landing, so the two are one family and the square one is tried first.
LANDINGS = ('square', 'angled')
LAND_SHARE_STEP = 0.02      # the angled landing's plate share is scanned at this fraction
LAND_SHARE_TOL = 1e-4       # ... of the pair's lateral offset, then bisected to this
SOLVE_MARGIN = 1.0          # mm asked for BEYOND every clearance bar by the two solves that
                            # had none (EXO-182/EXO-186): the angled landing's plate share and
                            # the EXO-181 rotor-pad chord. A refusal is still at the true bar;
                            # this only keeps a SOLVED answer off the line, as SLIDE_MARGIN
                            # does for EXO-170's slide.
LAND_R_ROUNDS = 24          # how many times the landing disk may be grown to cover the
LAND_R_TOL = 1e-3           # actuator opening (EXO-186), and the mm it is solved to
LAND_R_GAIN = 1.6           # ... the growth overshoots by this, because moving the rim out by
                            # d buys only ~0.65 d of web: the tongue's blends follow the rim
LAND_R_STEP = 3.0           # mm; how coarsely the SEARCH walks (the answer is confirmed at
                            # `ACT_RING_STEP`, and grown again if the coarse walk flattered it)


def _level_advance(drop, r):
    """How much STRAIGHT RUN a level change of `drop` costs along the leg it is taken on.

    Two opposite tangent bends of radius `r`: when |drop| <= 2r they are two partial bends
    turning through `theta` and the run they need is `2 r sin theta`; beyond that they are
    two quarter turns with a VERTICAL riser between, which costs no run at all, so the cap is
    `2 r`. This is the number every placement is judged against."""
    if abs(drop) < 1e-9:
        return 0.0
    if abs(drop) <= 2.0 * r:
        th = math.acos(max(-1.0, min(1.0, 1.0 - abs(drop) / (2.0 * r))))
        return 2.0 * r * math.sin(th)
    return 2.0 * r


def _level_change(segs, p0, dh, drop, r):
    """Append one level change to `segs`: two opposite tangent bends of radius `r` in the
    vertical plane through the horizontal direction `dh`, from `p0` to `p0 + dh * adv` and
    `drop` higher. Returns (end point, advance, theta, riser length).

    `dh` is a unit vector in the (s, w) plane, so the same arithmetic serves the final leg
    (dh = +u) and the lateral departure leg (dh = the departure direction) - that is the
    whole of EXO-158's change to the primitive."""
    ds = 1.0 if drop >= 0 else -1.0
    if abs(drop) <= 2.0 * r:
        th = math.acos(max(-1.0, min(1.0, 1.0 - abs(drop) / (2.0 * r))))
        mid = 0.0
    else:
        th = math.pi / 2.0
        mid = abs(drop) - 2.0 * r
    adv = 2.0 * r * math.sin(th) + mid * math.cos(th)
    c1 = (p0[0], p0[1], p0[2] + ds * r)
    e1 = (p0[0] + dh[0] * r * math.sin(th), p0[1] + dh[1] * r * math.sin(th),
          p0[2] + ds * r * (1 - math.cos(th)))
    segs.append(('arc', p0, _arc_mid(p0, e1, c1, r), e1, c1, r))
    if mid > 0.0:
        e1 = (e1[0] + dh[0] * mid * math.cos(th), e1[1] + dh[1] * mid * math.cos(th),
              e1[2] + ds * mid * math.sin(th))
        segs.append(('line', segs[-1][3], e1))
    c2 = (e1[0] + dh[0] * r * math.sin(th), e1[1] + dh[1] * r * math.sin(th),
          e1[2] - ds * r * math.cos(th))
    e2 = (p0[0] + dh[0] * adv, p0[1] + dh[1] * adv, p0[2] + drop)
    segs.append(('arc', e1, _arc_mid(e1, e2, c2, r), e2, c2, r))
    return e2, adv, th, mid


def centre_line(alpha, w_off, r_plan, s_b, drop, r_side, s_end, back, half_w, half_t,
                l_b=None, drop_lat=0.0):
    """The route's centre line in the pad frame's (s, w, z), as a list of segments.

    Three moves, each of which may be absent when the pair does not need it:
      1. the PLAN turn - the ribbon leaves the pad disk along a straight through the pad
         axis at `alpha` to u and turns once, in a horizontal plane, onto the line
         w = `w_off` that the landing plate's axis stands on. One bend, so ANY lateral
         offset is reachable: alpha -> 90 deg puts the whole offset in the departure.
      2. the LEVEL CHANGE - two opposite tangent bends taking the ribbon from the pad plane
         down (or up) to the landing's height. `drop_lat` of it is taken on the LATERAL
         departure leg, starting `l_b` mm from the pad axis along that leg; the rest is
         taken on the final leg at `s_b`, which is where all of it used to go (EXO-158).
         Either part may be zero. The plan turn is drawn at whatever height the lateral
         part left the ribbon at.
      3. the straight run into the landing plate, ending `BURY` mm inside it.

    Returns (segments, numbers). Segments are ('line', p0, p1) and
    ('arc', p0, p_mid, p1, centre, r) with 3D points in (s, w, z).
    """
    sgn = 1.0 if w_off >= 0 else -1.0
    segs, num = [], {}
    z0 = g.PAD_T / 2.0
    drop_fin = drop - drop_lat
    z_lat = z0 + drop_lat
    if abs(alpha) > 1e-12:
        d0 = (math.cos(alpha), sgn * math.sin(alpha))
        t = r_plan * math.tan(alpha / 2.0)
        v1 = (abs(w_off) / math.tan(alpha) * 1.0, w_off)
        p_in = (v1[0] - d0[0] * t, v1[1] - d0[1] * t)
        p_out = (v1[0] + t, w_off)
        start = (-back * d0[0], -back * d0[1])
        run0 = math.hypot(p_in[0] - start[0], p_in[1] - start[1])
        # the bend's centre is r_plan to the ribbon's turning side of the incoming line
        nrm = (-sgn * d0[1], sgn * d0[0])       # left of travel when sgn>0 is +w... turn is
        cen = (p_in[0] - nrm[0] * r_plan, p_in[1] - nrm[1] * r_plan)
        if abs(drop_lat) > 1e-9:
            # EXO-158: the level change is taken HERE, on the lateral departure leg, which
            # runs out from the pad axis along d0 and is `run0` mm long before the turn.
            a0 = (d0[0] * l_b, d0[1] * l_b, z0)
            segs.append(('line', (start[0], start[1], z0), a0))
            e2, adv_l, th_l, mid_l = _level_change(segs, a0, d0, drop_lat, r_side)
            segs.append(('line', e2, (p_in[0], p_in[1], z_lat)))
            num.update(lat_l=(l_b, l_b + adv_l), lat_advance=adv_l, lat_drop=drop_lat,
                       lat_theta_deg=math.degrees(th_l), lat_mid=mid_l, lat_r=r_side)
        else:
            segs.append(('line', (start[0], start[1], z0), (p_in[0], p_in[1], z0)))
            num.update(lat_l=None, lat_advance=0.0, lat_drop=0.0, lat_theta_deg=0.0,
                       lat_mid=0.0, lat_r=None)
        p0 = (p_in[0], p_in[1], z_lat)
        p1 = (p_out[0], p_out[1], z_lat)
        c3 = (cen[0], cen[1], z_lat)
        segs.append(('arc', p0, _arc_mid(p0, p1, c3, r_plan), p1, c3, r_plan))
        num.update(plan_alpha_deg=math.degrees(alpha), plan_r=r_plan, plan_run=run0,
                   plan_s_in=p_in[0], plan_s_out=p_out[0], plan_start=start)
        s_here = p_out[0]
    else:
        if abs(drop_lat) > 1e-9:
            raise ValueError('there is no lateral leg to take the level change on: the '
                             'plate takes the offset and the route leaves straight along u')
        start = (-back, w_off)
        s_here = -back
        num.update(plan_alpha_deg=0.0, plan_r=None, plan_run=None, plan_s_in=None,
                   plan_s_out=None, plan_start=start, lat_l=None, lat_advance=0.0,
                   lat_drop=0.0, lat_theta_deg=0.0, lat_mid=0.0, lat_r=None)
    if abs(drop_fin) > 1e-9:
        segs.append(('line', (s_here, w_off, z_lat), (s_b, w_off, z_lat)))
        p0 = (s_b, w_off, z_lat)
        e2, adv, th, mid = _level_change(segs, p0, (1.0, 0.0), drop_fin, r_side)
        num.update(side_theta_deg=math.degrees(th), side_r=r_side, side_mid=mid,
                   side_s=(s_b, s_b + adv), side_advance=adv, side_drop=drop_fin)
        s_here = s_b + adv
        z_here = z_lat + drop_fin
    else:
        z_here = z_lat
        num.update(side_theta_deg=0.0, side_r=None, side_mid=0.0, side_s=None,
                   side_advance=0.0, side_drop=0.0)
    segs.append(('line', (s_here, w_off, z_here), (s_end, w_off, z_here)))
    num['final_run'] = s_end - s_here
    num['start'] = segs[0][1]
    return segs, num


def _plan_alpha(w_off, r_plan, s_limit):
    """The GENTLEST single plan turn that is finished by s = `s_limit`.

    `f(alpha) = |w_off| / tan(alpha) + r_plan tan(alpha/2)` is where the turn ends; it falls
    from +inf at alpha -> 0 to `r_plan` at 90 deg, so the smallest feasible alpha is one
    bisection. A pair whose offset cannot be taken even by a quarter turn is refused by
    name - that is the only lateral limit this method has, and it is `r_plan` mm of reach,
    not the S-bend's `2 r sin theta`."""
    if abs(w_off) < 1e-9:
        return 0.0
    if r_plan > s_limit:
        raise ValueError('the plan turn needs r %.2f mm of reach to take the %.2f mm lateral '
                         'offset and only %.2f mm are free before the next feature'
                         % (r_plan, w_off, s_limit))

    def f(a):
        return abs(w_off) / math.tan(a) + r_plan * math.tan(a / 2.0)

    lo, hi = 1e-6, math.pi / 2.0
    if f(hi) > s_limit:
        raise ValueError('impossible: f(90 deg) = %.3f > %.3f' % (f(hi), s_limit))
    for _ in range(200):
        mid = (lo + hi) / 2.0
        if f(mid) > s_limit:
            lo = mid
        else:
            hi = mid
    return hi


def _seg_frame(sg, p):
    """(tangent, side-plane normal) of a path segment at a sampled point."""
    if sg[0] == 'line':
        d = [sg[2][k] - sg[1][k] for k in range(3)]
    else:
        _, p0, pm, p1, c, r = sg
        a = [p0[i] - c[i] for i in range(3)]
        b = [p1[i] - c[i] for i in range(3)]
        npl = g.cross(a, b)
        d = g.cross(npl, [p[i] - c[i] for i in range(3)])
    nn = math.sqrt(sum(v * v for v in d))
    d = [v / nn for v in d] if nn > 1e-12 else [1.0, 0.0, 0.0]
    n = [-d[2], 0.0, d[0]]
    nn = math.hypot(n[0], n[2])
    n = [n[0] / nn, 0.0, n[2] / nn] if nn > 1e-12 else [0.0, 0.0, 1.0]
    return d, n


def _body_radial_clearance(segs, w_off, half_w, half_t, z_top=CLEAR_Z, step=0.25):
    """Smallest distance from axis A to the ribbon, over the part of it BELOW `z_top` - the
    plane under which module A's stator (and the previous link's landing plate) fill the
    r `CLEAR_R` cylinder. Returns (distance, point) or (None, None) when nothing is down
    there. The ribbon's width band is continuous, so the nearest w is 0 when the band
    straddles the axis and the nearer edge otherwise."""
    w_lo, w_hi = w_off - half_w, w_off + half_w
    w_near = 0.0 if w_lo <= 0.0 <= w_hi else (w_lo if w_lo > 0 else w_hi)
    best, at = None, None
    for sg in segs:
        for p in _sample(sg, step):
            _d, n = _seg_frame(sg, p)
            for k in (-1.0, 0.0, 1.0):
                q = (p[0] + n[0] * half_t * k, p[1], p[2] + n[2] * half_t * k)
                if q[2] >= z_top:
                    continue
                r = math.hypot(q[0], w_near)
                if best is None or r < best:
                    best, at = r, q
    return best, at


def _sample(sg, step):
    if sg[0] == 'line':
        p0, p1 = sg[1], sg[2]
        L = math.sqrt(sum((p1[i] - p0[i]) ** 2 for i in range(3)))
        n = max(2, int(L / step) + 1)
        return [tuple(p0[i] + (p1[i] - p0[i]) * k / (n - 1.0) for i in range(3))
                for k in range(n)]
    _, p0, pm, p1, c, r = sg
    a0 = [p0[i] - c[i] for i in range(3)]
    a1 = [p1[i] - c[i] for i in range(3)]
    dotv = sum(a0[i] * a1[i] for i in range(3)) / (r * r)
    ang = math.acos(max(-1.0, min(1.0, dotv)))
    n = max(3, int(r * ang / step) + 1)
    out = []
    for k in range(n):
        t = k / (n - 1.0)
        if ang < 1e-12:
            out.append(p0)
            continue
        s0 = math.sin((1 - t) * ang) / math.sin(ang)
        s1 = math.sin(t * ang) / math.sin(ang)
        out.append(tuple(c[i] + a0[i] * s0 + a1[i] * s1 for i in range(3)))
    return out


def _module_b_clearance(segs, num, half_w, half_t, step=0.5, collar=0.0, s_near=None):
    """Smallest gap between the ribbon and module B's own body - the r `CLEAR_R` cylinder
    about axis B over the stator's own z range. The landing plate sits BELOW that range, so
    the buried end of the ribbon is not a false positive. Returns (gap, point) or (None, None)
    when the ribbon never comes alongside module B.

    `collar` is EXO-183: the EXO-151 inside fillet puts a collar of that radius all the way
    round the ribbon where it enters the landing plate, and the collar is material like any
    other. It only exists within `collar` mm of the plate's near face, so the section is
    grown there and nowhere else - measuring the body that actually gets built, instead of
    the bare swept ribbon."""
    u, w, t_ab, z_s = num['u'], num['w'], num['t_ab'], num['z_s']
    lo, hi = g.STATOR_UNDERSIDE_Z, 0.0
    best, at = None, None
    for sg in segs:
        for p in _sample(sg, step):
            d, n = _seg_frame(sg, p)
            wd = g.cross(d, n)
            grow = (collar if (collar > 0.0 and s_near is not None
                               and abs(p[0] - s_near) <= collar) else 0.0)
            h_t, h_w = half_t + grow, half_w + grow
            for kt in (-1.0, 0.0, 1.0):
                for kw in (-1.0, 0.0, 1.0):
                    q = [p[i] + n[i] * h_t * kt + wd[i] * h_w * kw for i in range(3)]
                    pa = [u[i] * q[0] + w[i] * q[1] + (q[2] if i == 2 else 0.0)
                          for i in range(3)]
                    dv = [pa[i] - t_ab[i] for i in range(3)]
                    along = g.dot(dv, z_s)
                    if not (lo <= along <= hi):
                        continue
                    perp = math.sqrt(max(0.0, g.dot(dv, dv) - along ** 2))
                    if best is None or perp < best:
                        best, at = perp, q
    return best, at


def sweep_route(R_ab, t_ab, pad_r, width, thickness=RIBBON_T, web=LAND_WEB,
                stop_angle=None, stop_half=10.0, stop_h=g.STOP_H, stop_why=None,
                landings=LANDINGS, plate_check=True, share=None):
    """Size the one route for one joint pair. Raises ValueError naming the number that fails.

    `plate_check=False` only REPORTS an actuator opening that breaks out of the plate outline
    instead of refusing it (EXO-187). It exists so that a refused pair can still be sized and
    measured as evidence (tests); `BuildArmGen` never passes it and builds nothing refused.

    `share` FORCES the angled landing's plate share instead of solving for it, so that the
    claim 'the share is the smallest one that routes' can be checked by sizing at a hair less
    and watching it refuse (EXO-186). The builder never passes it either.

    Returns a dict with the 3D centre line (`path`, in the pad frame's (s, w, z)), the
    ribbon's cross section, the landing plate's outline and every number the report needs.
    """
    num = g.landing_numbers(R_ab, t_ab)
    if 'u' not in num:
        raise ValueError('the two joint axes are parallel: the landing plate has no normal '
                         'in the rotor pad plane (in-plane part %.2e)' % num['zb_in_plane'])
    if num['snap_axis_rad'] > AXIS_SQUARE_MAX:
        raise ValueError('the stator axis is %.4f rad out of the rotor pad plane (max %.0e): '
                         'the landing plate would not face the ribbon'
                         % (num['snap_axis_rad'], AXIS_SQUARE_MAX))
    half_w, half_t = width / 2.0, thickness / 2.0
    u, w = num['u'], num['w']
    axial, y_sign = num['axial'], num['y_sign']
    w_off = num['w_off']
    # EXO-181: the link's own rotor join, measured against module B's body and cut back by one
    # chord when it has to be. Solved once - it depends on the pair and the pad radius and on
    # nothing the route chooses - and refused by name when no chord works.
    pad_trim, pad_info = solve_pad_trim(num, pad_r)
    stop_margins = []
    if pad_trim is not None:
        # the chord takes material off the pad, so the two features that live ON the pad have
        # to survive it whole: the link's own stop, and the fused rotor top under the pad
        # (EXO-370, r 18). A chord that would cut either is a refusal, by name.
        _ang = STOP_ANGLE if stop_angle is None else stop_angle
        # EXO-208: the stop is no longer always a thin +/-5 deg sliver - the rule hands J4
        # 165 deg of arc - so the chord is checked against the WHOLE outline, not against
        # two edges and a centre line. EXO-218: that outline is the ONE solid block again,
        # and the chord has to MISS it by `STOP_CHORD_MARGIN`, not merely graze it.
        # `stop_block_points` is the same discretisation `link_samples` uses.
        _checks = [('the link\'s stop block at %.3f +/- %.3f deg' % (_ang, stop_half),
                    g.stop_block_points(_ang, stop_half, pad_r), STOP_CHORD_MARGIN)]
        _checks.append(('the fused rotor top (r %.0f)' % g.TOP_DISK_R, g.top_rim_points(), 0.0))
        for _what, _pts, _need in _checks:
            _d = min(px * pad_trim[0] + py * pad_trim[1] - pad_trim[2] for px, py in _pts)
            if _need > 0.0:
                stop_margins.append((_what, _d))
            if _d < _need - 1e-9:
                raise ValueError('the %.3f mm chord the rotor pad needs to clear module B '
                                 '(EXO-181) %s %s'
                                 % (pad_info['depth'],
                                    'would cut %.3f mm into' % -_d if _d < 0.0
                                    else 'comes %.3f mm of (need %.1f)' % (_d, _need),
                                    _what))
    # The ribbon travels along +u, so it meets the plate's NEARER face first whichever way
    # round the stator sits; the far one is the face it never reaches.
    s_near, s_far = num['s_slab']
    if half_w > pad_r:
        raise ValueError('a %.1f mm ribbon is wider than the r %.1f rotor pad it leaves '
                         '(%.2f mm per side)' % (width, pad_r, half_w - pad_r))

    # ---- 1. how high the ribbon arrives, and therefore how long the tongue is.
    # The landing sits `web` past the actuator OPENING's reach, measured in the TRUE stator
    # frame where the actuator is, so the module's rotation moves it. The opening is the
    # XL330's footprint grown by `ACT_OPEN_MARGIN` - 0 since EXO-193, the hand-made,
    # real-world-validated opening - so `cut_reach_clear` is the actuator's own edge and the
    # web is plate between the OPENING and the ribbon. The ribbon stays in the rotor pad's
    # plane when the opening lets
    # it and moves exactly as far as it must otherwise: the ONE rule that makes link_12/34
    # flat plates and link_23/45 bars.
    cut_clear = num['cut_reach_clear']
    y_min = cut_clear + web + half_t
    z_pref = g.PAD_T / 2.0
    z_arr = min(z_pref, axial - y_min) if y_sign < 0 else max(z_pref, axial + y_min)
    y_land = y_sign * (z_arr - axial)
    y_end = y_land + half_t
    cut_web = (y_land - half_t) - cut_clear
    drop = z_arr - z_pref

    # ---- 2. where the ribbon may leave the pad plane: past the rim, past the flat every
    # screw head needs, and far enough out that nothing below the stator stop block's top
    # face enters the r 27 cylinder. Solved, not assumed. `s_flat` is a DISTANCE FROM THE
    # PAD AXIS, measured along whichever leg the ribbon leaves the plane on.
    s_flat = max(pad_r + RIM_MARGIN, g.rotor_flat_needed(u))
    r_side = max(BEND_R, half_t + WALL_R_MIN)
    r_plan = max(BEND_R, half_w + WALL_R_MIN)
    s_end = s_near + BURY
    back = max(0.0, math.sqrt(max(0.0, pad_r ** 2 - half_w ** 2)) - 1.0)
    need_adv = _level_advance(drop, r_side)
    x_sign = y_sign * num['u_sign']

    # ---- 3. THE ORDER OF THE MOVES (EXO-158). The route has two moves to make - a plan
    # turn for the lateral offset and a level change for the landing's height - and used to
    # take them in ONE fixed order: turn first, then change level on the run between the pad
    # and the plate. That run is the SHORTEST leg the route has, so a pair whose plate stands
    # close to the pad was refused however much room its departure had (EXO-148). The order
    # is now free: the level change goes on the final leg, on the LATERAL departure leg, or
    # split across the two, and each placement is sized in full and checked in full.
    #
    # ---- and THE DIRECTION OF THE TONGUE (EXO-176). All of the above still assumes the
    # landing is SQUARE: the plate's tongue is a strip parallel to the plate frame's own +Y,
    # which `landing_numbers` pins to the rotor pad axis, so the only way to reach past the
    # actuator cutout is to arrive HIGHER or LOWER. The plate is a flat disk and its plane
    # carries two free directions, not one; the second landing primitive lets the tongue run
    # along any of them, AIMED FROM THE DISK CENTRE AT THE POINT WHERE THE RIBBON ENTERS THE
    # PLANE. A pair that has lateral offset to spare then spends it INSIDE the plate plane,
    # where it costs no straight run at all, instead of climbing (EXO-173's 25.45 mm).
    # theta = 0 is the square landing exactly, so the two primitives are one family and the
    # chooser tries the square one first.

    def _plan(w_route, s_limit):
        """The one plan turn that takes `w_route` of lateral before `s_limit`, or raise."""
        alpha = _plan_alpha(w_route, r_plan, s_limit)
        run0 = abs(w_route) / math.sin(alpha) - r_plan * math.tan(alpha / 2.0)
        if run0 < MIN_RUN:
            raise ValueError('the departure straight would be %.2f mm (need %.1f)'
                             % (run0, MIN_RUN))
        return alpha, run0

    def _flat_station(drop_v, s_from):
        """Push the level change's start station out along the final leg until nothing below
        the pad plane enters the r CLEAR_R cylinder. Returns (s_b, clearance, point)."""
        s_b, clear, at = s_from, None, None
        for _ in range(400):
            probe, _n = centre_line(0.0, 0.0, r_plan, s_b, drop_v, r_side, s_end,
                                    back, half_w, half_t)
            clear, at = _body_radial_clearance(probe, 0.0, half_w, half_t)
            if clear is None or clear >= CLEAR_R + g.MIN_CLEARANCE:
                return s_b, clear, at
            s_b += 0.1
        raise ValueError('no start station keeps the ribbon %.1f mm clear of the r %.1f '
                         'cylinder under the pad' % (g.MIN_CLEARANCE, CLEAR_R))

    def _finish(order, land, alpha, w_route, why_off, drop_lat, l_b, l_flat, s_b,
                clear, at, slack=0.0):
        """Every check the route has to pass, and the route dict. Shared by both landings:
        the two primitives differ ONLY in where the tongue points and how far it reaches.

        `slack` raises every clearance bar by that much (EXO-186/EXO-182). It is what the
        angled landing SOLVES its plate share against, so the share it settles on is never the
        one that clears by a micron - the same thing `SLIDE_MARGIN` does for EXO-170's slide.
        The square landing has no share to solve, so it is sized at slack 0 and the five
        square links do not move."""
        drop = land['drop']
        # EXO-186: the landing plate's own disk is not `STATOR_PAD_R` by assumption. The
        # actuator opening is fixed in the TRUE frame B and the tongue is not, so wherever
        # the spin puts the two relative to each other the plate has to COVER the opening
        # with `web` of plate all round it. The disk radius is therefore SOLVED, the same
        # shape of solve as the angled share: grow until it passes, then bisect back to the
        # smallest radius that still does. A landing whose tongue already covers the opening
        # - link_12/23/45/56 - passes at `STATOR_PAD_R` on the first draw and does not move.
        def _draw(r, coarse=True):
            eds = g.flange_edges(land['y_wall'], land['x_off'], land['half_w_t'], r,
                                 overhang='crown')
            fl, yb_, _tp = g.flange_outline(land['y_wall'], land['half_w_t'], r,
                                            x_off=land['x_off'], edges=eds,
                                            end_pts=land['end_pts'])
            # the SEARCH walks the opening and the outline coarsely - it is looking for a
            # radius, and the corners (where the web is thinnest) are walked either way; the
            # radius it settles on is then confirmed at full resolution below.
            bo, wmin, _wat = opening_in_plate(num, land['R_t'], fl,
                                              step=LAND_R_STEP if coarse else ACT_RING_STEP,
                                              poly_step=LAND_R_STEP if coarse else None)
            return eds, fl, yb_, bo, wmin

        land_r, land_r_walk = g.STATOR_PAD_R, []
        edges, flange, y_blend, _bo, _wmin = _draw(land_r)
        land_r_walk.append((land_r, _wmin))
        if _bo is not None or _wmin < web:
            lo, hi, ok = land_r, land_r, None
            for _ in range(LAND_R_ROUNDS):
                lo = hi
                hi += max(web - _wmin, LAND_R_TOL) * LAND_R_GAIN + LAND_R_TOL
                eds, fl, yb_, _bo, _wmin = _draw(hi)
                land_r_walk.append((hi, _wmin))
                if _bo is None and _wmin >= web:
                    ok = (hi, eds, fl, yb_)
                    break
            if ok is None:
                raise ValueError('the landing plate cannot be grown to cover the actuator '
                                 'opening: %d rounds from r %.1f (%s) and the thinnest plate '
                                 'round the opening is still under %.1f mm'
                                 % (LAND_R_ROUNDS, g.STATOR_PAD_R,
                                    ', '.join('r %.3f -> %.3f' % w for w in land_r_walk),
                                    web))
            # the SMALLEST radius that covers it, to LAND_R_TOL - bracketed, never assumed
            while ok[0] - lo > LAND_R_TOL:
                mid = (lo + ok[0]) / 2.0
                eds, fl, yb_, _bo, _wmin = _draw(mid)
                if _bo is None and _wmin >= web:
                    ok = (mid, eds, fl, yb_)
                else:
                    lo = mid
            land_r, edges, flange, y_blend = ok
            land_r_walk.append((land_r, web))
            # confirm at full resolution, and grow again if the coarse walk flattered it
            for _ in range(LAND_R_ROUNDS):
                eds, fl, yb_, _bo, _wmin = _draw(land_r, coarse=False)
                land_r_walk.append((land_r, _wmin))
                if _bo is None and _wmin >= web:
                    land_r, edges, flange, y_blend = land_r, eds, fl, yb_
                    break
                land_r += max(web - _wmin, LAND_R_TOL) * LAND_R_GAIN + LAND_R_TOL
            else:
                raise ValueError('the landing plate cannot be grown to cover the actuator '
                                 'opening: from r %.1f (%s) the thinnest plate round the '
                                 'opening is still under %.1f mm'
                                 % (g.STATOR_PAD_R,
                                    ', '.join('r %.3f -> %.3f' % w for w in land_r_walk),
                                    web))
        segs, nums = centre_line(alpha, w_route, r_plan, s_b, drop, r_side, s_end, back,
                                 half_w, half_t, l_b=l_b, drop_lat=drop_lat)
        if abs(drop) > 1e-9:
            clear, at = _body_radial_clearance(segs, w_route, half_w, half_t)
            if clear is not None and clear < CLEAR_R + g.MIN_CLEARANCE + slack:
                raise ValueError('the ribbon comes %.2f mm from axis A at (s %.2f, z %.2f), '
                                 'inside the r %.1f cylinder plus %.1f mm'
                                 % (clear, at[0], at[2], CLEAR_R, g.MIN_CLEARANCE))
        clear_p, at_p = _plate_radial_clearance(num, land, flange)
        if clear_p is not None and clear_p < CLEAR_R + g.MIN_CLEARANCE + slack:
            raise ValueError('the landing plate itself comes %.2f mm from axis A at '
                             '(s %.2f, w %.2f, z %.2f), below z %.1f where module A\'s own '
                             'body fills the r %.1f cylinder (need %.1f mm outside it)'
                             % (clear_p, at_p[0], at_p[1], at_p[2], CLEAR_Z, CLEAR_R,
                                g.MIN_CLEARANCE))
        clear_b, at_b = _module_b_clearance(segs, num, half_w, half_t)
        if clear_b is not None and clear_b < CLEAR_R + g.MIN_CLEARANCE + slack:
            raise ValueError('the ribbon passes %.2f mm from axis B alongside module B\'s '
                             'own body (need %.1f mm outside r %.1f)'
                             % (clear_b, g.MIN_CLEARANCE, CLEAR_R))
        # EXO-183: the EXO-151 inside fillet puts a collar of `ELBOW_R_IN` all the way round
        # the ribbon where it enters the plate, and that collar is material. It is a BLEND,
        # not body, so what it has to do is not enter module B - `CLEAR_R`, not
        # `CLEAR_R + MIN_CLEARANCE`, which is the ribbon's own bar above. The number is
        # carried and reported either way.
        clear_c, at_c = _module_b_clearance(segs, num, half_w, half_t,
                                            collar=ELBOW_R_IN, s_near=s_near)
        if clear_c is not None and clear_c < CLEAR_R + slack - 1e-9:
            raise ValueError('the EXO-151 collar the ribbon carries where it enters the '
                             'plate reaches %.3f mm from axis B, %.3f mm inside module B\'s '
                             'own r %.1f body' % (clear_c, CLEAR_R - clear_c, CLEAR_R))
        for i, sg in enumerate(segs):
            if sg[0] != 'line':
                continue
            L = math.sqrt(sum((sg[2][k] - sg[1][k]) ** 2 for k in range(3)))
            if L < MIN_RUN:
                raise ValueError('route run %d is only %.3f mm long (need %.1f): the pair has '
                                 'no room between the pad, the bend and the landing plate'
                                 % (i, L, MIN_RUN))

        route = {
            'path': segs, 'width': width, 'thickness': thickness, 'pad_r': pad_r,
            'u': u, 'w': w, 'numbers': num, 'flange': flange, 'y_end': land['y_end'],
            'y_blend': y_blend, 'edges': edges, 'y_land': land['y_land'],
            'z_arr': land['z_arr'],
            's_near': s_near, 's_far': s_far, 's_land': num['s_land'],
            'w_off': w_off, 'w_route': w_route, 'x_off': land['x_off'],
            # the plate's share of the lateral offset: 0-or-1 for the square landing (x_off),
            # any fraction for the angled one (the tongue's aim); w_route takes the rest
            'share': 0.0 if abs(w_off) < 1e-9 else (w_off - w_route) / w_off,
            'offset_taken_by': why_off,
            'drop': drop, 'cut_web': land['cut_web'], 'cut_reach': land['cut_reach'],
            'cut_reach_act': land['cut_reach_act'],
            's_b': s_b, 's_flat': s_flat, 's_end': s_end, 'back': back,
            'l_b': l_b, 'l_flat': l_flat, 'drop_lat': drop_lat, 'drop_fin': drop - drop_lat,
            'level_order': order, 'level_need': _level_advance(drop, r_side),
            'clearance_a': clear, 'clearance_at': at, 'clearance_b': clear_b,
            'clearance_b_collar': clear_c, 'clearance_b_collar_at': at_c,
            'R_b': num['R_b'], 'bury': BURY, 'blend_r': tuple(e['r'] for e in edges),
            'stop_angle': STOP_ANGLE if stop_angle is None else stop_angle,
            'stop_half': stop_half, 'stop_h': stop_h,
            # EXO-214/EXO-218: how far the EXO-181 chord - when there is one - stayed off
            # the stop block. Empty when no chord was cut.
            'stop_chord_margins': tuple(stop_margins),
            # EXO-204: where the three numbers above came from. None = the placement-only
            # default at the top of this file (diametrically opposite the module's joint
            # stop), which limits nothing; `armgen.limited_joint` fills this in with the
            # joint whose URDF limits it sized the block from.
            'stop_why': stop_why or ('placement only, %.0f deg = the module joint stop + 180 '
                                     '(no joint limit designed)' % STOP_ANGLE),
            # EXO-176: the landing primitive and where its tongue points
            'landing': land['kind'], 'tongue_deg': math.degrees(land['theta']),
            'R_t': land['R_t'], 'x_t': land['x_t'], 'y_t': land['y_t'],
            'tongue_half_w': land['half_w_t'], 'r_land': land['r_land'],
            'foot_box': land['foot_box'], 'tongue_why': land['why'],
            'tongue_end': land['end_pts'] or [(land['x_off'] + land['half_w_t'],
                                               land['y_end']),
                                              (land['x_off'] - land['half_w_t'],
                                               land['y_end'])],
            'y_round': land['y_round'], 'y_wall': land['y_wall'],
            # EXO-186: the landing disk the outline was actually drawn on, and the walk
            'land_r': land_r, 'land_r_walk': land_r_walk,
            'clearance_plate': clear_p, 'clearance_plate_at': at_p,
            # EXO-181: the rotor pad's chord, and the gap it buys
            'pad_trim': pad_trim, 'pad_outline': g.pad_outline(pad_r, pad_trim),
            'pad_trim_depth': g.pad_trim_depth(pad_r, pad_trim),
            'clearance_pad': pad_info['gap'], 'clearance_pad_0': pad_info['gap0'],
            'pad_trim_why': pad_info['why'],
        }
        route.update(nums)
        route['cut_gap'] = cut_gap(route)
        # EXO-187: nothing the builder makes for this route, other than the landing plate
        # the opening is cut through, may sit inside the actuator's exclusion volume.
        act = actuator_check(route)
        route.update({'act_clear': act['clear'], 'act_clear_at': act['at'],
                      'act_part': act['part'], 'act_gap': act['gap'],
                      'act_gap_part': act['gap_part'],
                      'act_collar_gap': act['collar_gap'],
                      'act_margin': act['m'], 'cable_exit': act['cable_exit'],
                      'cable_exit_part': act['cable_exit_part']})
        route.update(plate_opening(route))
        if plate_check and not route['act_opening_in_plate']:
            bx, by = route['act_breakout_at']
            raise ValueError('the actuator opening (the XL330 grown by %.1f mm) breaks out of '
                             'the landing plate\'s outline at module B (x %.2f, y %.2f): the '
                             'plate has no web there and the plate beyond that edge would be '
                             'cut loose (EXO-187; the outline is EXO-186\'s to grow)'
                             % (act['m'], bx, by))
        if act['clear'] < -1e-9:
            raise ValueError('the %s has material inside the actuator volume - the XL330 '
                             'grown by %.1f mm - %.3f mm deep at module B (x %.2f, y %.2f, '
                             'z %.2f); EXO-187 refuses it rather than cut it'
                             % (act['part'],
                                0.0 if act['part'] == COLLAR_PART else act['m'],
                                -act['clear'], act['at'][0], act['at'][1], act['at'][2]))
        route['rings'] = hole_rings(route)
        bad = [h for h in route['rings'] if h['ring'] < g.HOLE_FLAT_RING - 1e-9]
        if bad:
            raise ValueError('a screw head would not seat flat: %s'
                             % '; '.join('the %s pad hole at %.0f deg has %.3f mm of flat '
                                         'ring on its %s face, taken by %s'
                                         % (h['pad'], h['angle'], h['ring'], h['face'],
                                            h['bound']) for h in bad))
        if land['cut_web'] < web + slack - 1e-9:
            raise ValueError('only %.3f mm of plate between the actuator cutout and the '
                             'ribbon (need %.1f)' % (land['cut_web'], web))
        return route

    def _sized(order):
        """The SQUARE landing (EXO-144/158), unchanged: the tongue is a strip parallel to the
        plate frame's +Y and the level change is placed by `order`. Raises ValueError naming
        the number that refuses it."""
        # (a) WHO TAKES THE LATERAL OFFSET. One solved split, not two shapes: the route's
        # single plan turn takes it when the leg before the next feature is long enough for
        # the turn a ribbon this wide has to make (r_plan = width/2 + WALL_R_MIN), and the
        # plate's tongue takes it when it is not. A 52 mm ribbon needs 31 mm of reach and
        # link_12's plate is 24.5 mm from the pad axis, so there the plate takes it.
        # Where the turn has to be FINISHED depends on the order: with the level change on
        # the final leg it must be over before that starts, as it always had to be; with the
        # level change on the lateral leg the turn only has to be over before the landing.
        if order == 'final':
            s_limit = (s_flat if abs(drop) > 1e-9 else s_near) - PLAN_MARGIN
        else:
            s_limit = s_end - MIN_RUN
        alpha, w_route, why_off = 0.0, 0.0, 'no lateral offset'
        if abs(w_off) >= 1e-9:
            try:
                alpha, _run0 = _plan(w_off, s_limit)
                w_route, why_off = w_off, 'the route: one %.2f deg plan turn, r %.1f' % (
                    math.degrees(alpha), r_plan)
            except ValueError as e:
                alpha, w_route = 0.0, 0.0
                why_off = 'the plate: the route cannot turn here (%s)' % e
        if order != 'final' and abs(w_route) < 1e-9:
            raise ValueError('there is no lateral leg: the plate takes the whole %.2f mm '
                             'offset and the route leaves straight along u' % w_off)

        # (b) WHERE THE LEVEL CHANGE GOES. `need_adv` mm of straight run, on the leg this
        # order names, starting no closer to the pad axis than `s_flat` along that leg.
        drop_lat, l_b, l_flat, s_b = 0.0, None, None, s_flat
        clear, at = None, None
        if abs(drop) > 1e-9:
            if order == 'final':
                # as it always was: start at the flat limit on the final leg and push the
                # station out until nothing below the pad plane enters the r CLEAR_R cylinder
                s_b, clear, at = _flat_station(drop, s_flat)
                room = s_end - MIN_RUN - s_b
                if room < need_adv - 1e-9:
                    raise ValueError('the final leg holds %.2f mm of straight between s %.2f '
                                     '(the rotor pad flat) and the plate, and the %.2f mm '
                                     'level change needs %.2f mm at r %.1f'
                                     % (max(room, 0.0), s_b, drop, need_adv, r_side))
            else:
                sgn_w = 1.0 if w_route >= 0 else -1.0
                d0 = (math.cos(alpha), sgn_w * math.sin(alpha))
                d0_a = [d0[0] * u[i] + d0[1] * w[i] for i in range(3)]
                l_flat = max(pad_r + RIM_MARGIN, g.rotor_flat_needed(d0_a))
                run0 = abs(w_route) / math.sin(alpha) - r_plan * math.tan(alpha / 2.0)
                room = run0 - l_flat - MIN_RUN
                if order == 'lateral':
                    if room < need_adv - 1e-9:
                        raise ValueError('the lateral leg holds %.2f mm of straight between '
                                         'l %.2f (the rotor pad flat) and the plan turn, and '
                                         'the %.2f mm level change needs %.2f mm at r %.1f'
                                         % (max(room, 0.0), l_flat, drop, need_adv, r_side))
                    drop_lat, l_b = drop, l_flat
                else:
                    # split: the lateral leg takes as much as it holds, the final leg the rest
                    ds = 1.0 if drop >= 0 else -1.0
                    th = math.asin(max(0.0, min(1.0, max(room, 0.0) / (2.0 * r_side))))
                    take = min(abs(drop), 2.0 * r_side * (1.0 - math.cos(th)))
                    if take < 1e-9:
                        raise ValueError('the lateral leg holds %.2f mm of straight past '
                                         'l %.2f and can take none of the %.2f mm level '
                                         'change' % (max(room, 0.0), l_flat, drop))
                    drop_lat, l_b = ds * take, l_flat
                    rest = drop - drop_lat
                    if abs(rest) > 1e-9:
                        s_b = (abs(w_route) / math.tan(alpha)
                               + r_plan * math.tan(alpha / 2.0) + MIN_RUN)
                        room_f = s_end - MIN_RUN - s_b
                        need_f = _level_advance(rest, r_side)
                        if room_f < need_f - 1e-9:
                            raise ValueError('the lateral leg took %.2f mm of the %.2f mm '
                                             'level change and the final leg holds %.2f mm '
                                             'of straight for the remaining %.2f mm, which '
                                             'needs %.2f mm at r %.1f'
                                             % (drop_lat, drop, max(room_f, 0.0), rest,
                                                need_f, r_side))
                    else:
                        drop_lat = drop

        x_off = (w_route - w_off) * x_sign
        land = {
            'kind': 'square', 'theta': 0.0, 'R_t': num['R_b'],
            'x_t': num['x_s'], 'y_t': num['y_s'], 'x_off': x_off,
            'y_land': y_land, 'y_end': y_end, 'half_w_t': half_w, 'r_land': y_land,
            'cut_reach': cut_clear, 'cut_reach_act': num['cut_reach'],
            'cut_web': cut_web, 'z_arr': z_arr, 'drop': drop,
            'foot_box': (x_off - half_w, x_off + half_w, y_land - half_t, y_end),
            'end_pts': None, 'y_wall': y_end, 'y_round': y_end,
            'why': 'the tongue runs along the plate frame\'s own +Y, the rotor pad axis '
                   '(EXO-144); the %.2f mm of level change is what reaching past the '
                   'actuator cutout costs' % drop,
        }
        return _finish(order, land, alpha, w_route, why_off, drop_lat, l_b, l_flat, s_b,
                       clear, at)

    # ---- the ANGLED landing (EXO-176) ----------------------------------------------------
    # The plate's share of the lateral offset is no longer 0-or-all: the tongue is aimed at
    # wherever the ribbon enters the plate plane, so ANY share is a drawable landing, and the
    # share is what buys the reach past the actuator cutout. The rule takes the SMALLEST
    # share that reaches - the same rule the square landing has, where the route's plan turn
    # takes the offset whenever it can and the plate only takes what is left.
    x_full = -w_off * x_sign        # the plate's share when the route takes none of it
    y_reach0 = y_sign * (z_pref - axial)     # plate-frame +Y of the entry point at drop 0

    def _angled_land(k, y_reach):
        """The angled landing's numbers at plate share `k` of the lateral offset and entry
        `y_reach` along the plate frame's +Y."""
        x_e, y_e = k * x_full, y_reach
        theta = g.tongue_angle(x_e, y_e)
        ext_t, ext_n = g.footprint_extent(half_w, half_t, theta)
        r_e = math.hypot(x_e, y_e)
        x_t, y_t, _z = g.tongue_frame(num, theta)
        cut_t = g.cutout_reach(num, y_t, g.ACT_OPEN_MARGIN)    # the OPENING (EXO-193)
        z_arr_a = axial + y_sign * y_e
        # EXO-177: the tongue stops ON the ribbon, so its two walls end where the footprint
        # is widest and the outline follows the ribbon's own edges between them.
        end_pts, y_p, y_m, half_w_t = g.tongue_end(half_w, half_t, theta, r_e)
        return {
            'kind': 'angled', 'theta': theta, 'R_t': g.tongue_R(num, theta),
            'x_t': x_t, 'y_t': y_t, 'x_off': 0.0,
            'y_land': r_e, 'y_end': r_e + ext_t, 'half_w_t': half_w_t, 'r_land': r_e,
            'end_pts': end_pts, 'y_wall': min(y_p, y_m),
            'y_round': min(p[1] for p in end_pts),
            'cut_reach': cut_t, 'cut_reach_act': g.cutout_reach(num, y_t),
            'cut_web': (r_e - ext_t) - cut_t,
            'z_arr': z_arr_a, 'drop': z_arr_a - z_pref, 'share': k,
            'foot_box': (-ext_n, ext_n, r_e - ext_t, r_e + ext_t),
            'why': 'the tongue is aimed from the disk centre at where the ribbon enters the '
                   'plate plane, %.2f deg off the pad axis; the plate takes %.2f mm of the '
                   '%.2f mm lateral offset and the route turns the rest'
                   % (math.degrees(theta), abs(x_e), abs(w_off)),
        }

    def _angled_route(k, y_reach, order, slack=0.0):
        """The whole route at one plate share and one entry reach, or a ValueError naming the
        number that refuses it."""
        land = _angled_land(k, y_reach)
        if land['cut_web'] < web + slack - 1e-9:
            raise ValueError('the tongue aimed %.2f deg off the pad axis reaches %.3f mm past '
                             'the actuator cutout (need %.1f)'
                             % (math.degrees(land['theta']), land['cut_web'], web))
        drop_a = land['drop']
        w_route = w_off * (1.0 - k)
        alpha = 0.0
        if abs(w_route) >= 1e-9:
            s_limit = (s_flat if abs(drop_a) > 1e-9 else s_near) - PLAN_MARGIN
            alpha, _run0 = _plan(w_route, s_limit)
            why_off = ('both: the plate takes %.2f mm of the lateral offset as the tongue\'s '
                       'aim and the route turns the other %.2f mm (one %.2f deg plan turn, '
                       'r %.1f)' % (abs(k * x_full), abs(w_route), math.degrees(alpha),
                                    r_plan))
        else:
            why_off = ('the plate: the tongue is aimed %.2f deg off the pad axis and takes '
                       'the whole %.2f mm' % (math.degrees(land['theta']), abs(w_off)))
        s_b, clear, at = s_flat, None, None
        if abs(drop_a) > 1e-9:
            s_b, clear, at = _flat_station(drop_a, s_flat)
            need_a = _level_advance(drop_a, r_side)
            room = s_end - MIN_RUN - s_b
            if room < need_a - 1e-9:
                raise ValueError('the angled landing still needs %.2f mm of level change and '
                                 'the final leg holds %.2f mm of straight for the %.2f mm it '
                                 'costs at r %.1f' % (drop_a, max(room, 0.0), need_a, r_side))
        return _finish(order, land, alpha, w_route, why_off, 0.0, None, None, s_b, clear,
                       at, slack)

    def _sized_angled(order, share=share):
        """The ANGLED landing (EXO-176). The tongue points at the entry; the lateral offset is
        split continuously between the route's plan turn and the tongue's aim, and the level
        change is only spent when the plate plane has run out of lateral to spend.

        The rule is the square landing's own: the route takes what it can and the plate takes
        what is left - so the SMALLEST plate share whose whole route passes every check is the
        answer. `ok` sizes the route IN FULL at each trial share, so the share is solved
        against the actuator cutout, the screw rings, module B's body and the plan turn
        together, and nothing is asserted afterwards."""
        if abs(x_full) < 1e-9:
            raise ValueError('the pair has no lateral offset (%.3f mm), so aiming the tongue '
                             'changes nothing: this IS the square landing' % w_off)

        def ok(k, y_reach=None):
            try:
                return _angled_route(k, y_reach0 if y_reach is None else y_reach, order,
                                    SOLVE_MARGIN)
            except ValueError as e:
                ok.why = str(e)
                return None
        ok.why = ''

        if share is not None:
            r = ok(share)
            if r is None:
                raise ValueError('the plate share %.6f does not route: %s' % (share, ok.why))
            return r

        # The smallest share that works: scan, then bisect the first crossing. Nothing here is
        # monotone - the cutout turns under the tongue as the tongue turns - so the crossing is
        # bracketed rather than assumed.
        lo, found = 0.0, None
        steps = int(round(1.0 / LAND_SHARE_STEP))
        for i in range(steps + 1):
            k = min(1.0, i * LAND_SHARE_STEP)
            if ok(k) is not None:
                hi = k
                while hi - lo > LAND_SHARE_TOL:
                    mid = (lo + hi) / 2.0
                    if ok(mid) is not None:
                        hi = mid
                    else:
                        lo = mid
                found = ok(hi)
                break
            lo = k
        if found is not None:
            return found

        # The whole lateral offset is not enough on its own: spend height as well, exactly as
        # the square landing does, but measured along the tongue.
        why_flat = ok.why
        land = _angled_land(1.0, y_reach0)
        short = web - land['cut_web']
        if short > 0:
            sg = 1.0 if y_reach0 >= 0 else -1.0
            lo2 = abs(y_reach0)
            hi2 = lo2 + short + abs(x_full) + 2.0 * (half_w + half_t)
            for _ in range(200):
                mid = (lo2 + hi2) / 2.0
                if _angled_land(1.0, sg * mid)['cut_web'] >= web:
                    hi2 = mid
                else:
                    lo2 = mid
            r = ok(1.0, sg * hi2)
            if r is not None:
                return r
            raise ValueError('with the plate taking the whole %.2f mm lateral offset the '
                             'tongue still has to climb %.2f mm, and %s'
                             % (abs(x_full), sg * hi2 - y_reach0, ok.why))
        raise ValueError('no share of the %.2f mm lateral offset gives the tongue a landing: '
                         'at the largest, %s' % (abs(w_off), why_flat))

    # ---- 4. pick the landing and the order BY RULE: the FIRST placement that is feasible.
    # `LANDINGS` runs square first, so a pair the square landing can take keeps exactly the
    # route it had and no number of link_12..56 moves; `LEVEL_ORDERS` runs from the least to
    # the most disturbance within a landing. Every combination is sized IN FULL either way
    # and its path length - or its refusal, by name - is carried on the route and reported,
    # so both the choice and what it cost are visible.
    sizers = {'square': _sized, 'angled': _sized_angled}
    tried, feasible = [], []
    for landing in landings:
        for order in LEVEL_ORDERS:
            if landing == 'angled' and order != 'final':
                continue        # the angled landing spends lateral, not height: one leg
            try:
                r = sizers[landing](order)
                tried.append((landing, order, r, None))
                feasible.append((landing, order, r))
            except ValueError as e:
                tried.append((landing, order, None, str(e)))
    if not feasible:
        raise ValueError('no landing and no placement of the level change fits this pair: %s'
                         % '; '.join('%s landing, %s leg: %s' % (ln, o, e)
                                     for ln, o, _r, e in tried))
    landing, order, route = feasible[0]
    route['level_tried'] = [(o, None if r is None else path_length(r), e)
                            for ln, o, r, e in tried if ln == 'square']
    route['landing_tried'] = [(ln, o, None if r is None else path_length(r), e)
                              for ln, o, r, e in tried]
    route['level_taken_by'] = (
        'the final leg (the run between the pad and the plate)' if order == 'final' else
        'the lateral departure leg' if order == 'lateral' else
        'both legs: %.3f mm on the lateral leg, %.3f mm on the final leg'
        % (route['drop_lat'], route['drop_fin']))
    route['landing_taken_by'] = route['tongue_why']
    return route

# ---------------- the flat seating ring, three faces of four by construction ----------------

# ---------------- the rotor pad against the NEXT module's body (EXO-181) ----------------
# The ribbon has been measured against module B since EXO-144 and the landing plate since
# EXO-178. The link's own ROTOR JOIN - the pad disk and the stop block (EXO-370: the fused top
# under the pad is r 18, inside module A, and is not part of this), all of
# them inside r `pad_r` about axis A - never has been. It did not show on G1's first five
# links because module B stands 38 to 45 mm clear of every one of them. On J6->J7 it does:
# the two axes are 46.00 mm apart and perpendicular, so J6's r 27 pad reaches straight into
# J7's r 27 body. That is not the route's doing and no landing can dodge it.
ROTOR_JOIN_N_THETA = 721        # how the pad rim is sampled for the gap (0.5 deg = 0.24 mm)
PAD_TRIM_TOL = 1e-3             # mm; the bisection when the closed form is not enough


def _module_b_solid_dist(num, s_v, w_v, z_v, prof=None):
    """Distance from one point of the pad plane's (s, w, z) to module B's solid of
    revolution - `armgen.MODULE_PROFILE` about axis B, the same body `module_gap` uses."""
    prof = g.MODULE_PROFILE if prof is None else prof
    u_sign = num['u_sign']
    s_tab = num['s_land'] - (g.STATOR_UNDERSIDE_Z - g.PAD_T) * u_sign
    along = (s_v - s_tab) * u_sign
    perp = math.hypot(w_v - num['w_off'], z_v - num['axial'])
    best = None
    for z0, z1, rr in prof:
        dz = z0 - along if along < z0 else (along - z1 if along > z1 else 0.0)
        dr = perp - rr
        if dr < 0.0:
            dr = 0.0
        v = dz * dz + dr * dr
        if best is None or v < best:
            best = v
    return math.sqrt(best), along, perp


def rotor_join_clearance(num, pad_r, trim=None, z_lo=None, n_theta=ROTOR_JOIN_N_THETA):
    """Smallest gap between the link's ROTOR JOIN and module B's own body, mm, with the
    point it happens at. 0.0 means they share space.

    The join is everything inside r `pad_r` about axis A over the pad's own z band, from the
    bottom of the stop block up to the top of the pad or the ribbon over it, whichever is
    higher (max(PAD_T, ROTOR_PAD_T), EXO-375). Its distance to module B is
    attained on its BOUNDARY - the pad rim, and the chord face when there is one - and only
    the z nearest module B's axis can be the nearest, so that z is solved rather than scanned.
    """
    z_lo = -g.STOP_H if z_lo is None else z_lo
    z_star = max(z_lo, min(max(g.PAD_T, g.ROTOR_PAD_T), num['axial']))
    best, at = None, None

    def take(s_v, w_v):
        d, along, perp = _module_b_solid_dist(num, s_v, w_v, z_star)
        return d, (s_v, w_v, z_star, along, perp)

    for i in range(n_theta):
        ang = 2.0 * math.pi * i / n_theta
        s_v, w_v = pad_r * math.cos(ang), pad_r * math.sin(ang)
        if trim is not None and (s_v * trim[0] + w_v * trim[1]) < trim[2]:
            continue
        d, where = take(s_v, w_v)
        if best is None or d < best:
            best, at = d, where
    if trim is not None:
        nx, ny, c = trim
        half = math.sqrt(max(0.0, pad_r ** 2 - c * c))
        for i in range(n_theta):
            f = -half + 2.0 * half * i / (n_theta - 1.0)
            d, where = take(c * nx - f * ny, c * ny + f * nx)
            if best is None or d < best:
                best, at = d, where
    return best, at


def solve_pad_trim(num, pad_r, ring=None, margin=None):
    """EXO-181: the ONE straight chord off the rotor pad that keeps the link's rotor join
    `MIN_CLEARANCE` clear of module B's body. Returns (trim, info); `trim` is None when the
    pad already clears, which is every link on G1 but J6->J7.

    The chord is solved, not chosen. The join's nearest z to module B's axis fixes how wide
    the forbidden band of `w` is, `w_off +/- sqrt(need^2 - dz^2)`, and the chord is laid on
    whichever edge of that band keeps the pad's own centre - so the rule is a closed form,
    checked against the real profile of revolution and bisected only if the closed form is
    not enough. Raises, naming the number, when no chord works: when module B's axis passes
    over the pad itself there is nothing to cut back to, and when the chord would take a
    screw head's flat ring the pad cannot be cut that far.

    A chord is cut only when the pad does NOT already clear by `MIN_CLEARANCE` - nobody cuts
    a pad that does not need it - but when one IS cut it is cut to `MIN_CLEARANCE + margin`
    (EXO-186; EXO-182 caught link_67's opening to exactly 1.000, a solved answer on the
    line). The REFUSAL is still at `MIN_CLEARANCE`.
    """
    margin = SOLVE_MARGIN if margin is None else margin
    target = g.MIN_CLEARANCE + margin
    need = max(rr for _a, _b, rr in g.MODULE_PROFILE) + target
    gap0, at0 = rotor_join_clearance(num, pad_r)
    info = {'gap0': gap0, 'at0': at0, 'need': g.MIN_CLEARANCE, 'target': target,
            'trim': None,
            'gap': gap0, 'depth': 0.0, 'why': ''}
    if gap0 >= g.MIN_CLEARANCE - 1e-9:
        info['why'] = ('the rotor pad already clears module B by %.3f mm (need %.1f): '
                       'no chord' % (gap0, g.MIN_CLEARANCE))
        return None, info
    w_off, axial = num['w_off'], num['axial']
    z_lo = -g.STOP_H
    z_star = max(z_lo, min(max(g.PAD_T, g.ROTOR_PAD_T), axial))
    d2 = need ** 2 - (z_star - axial) ** 2
    if d2 <= 0.0:
        raise ValueError('the rotor pad fouls module B (%.3f mm, need %.1f) and no chord can '
                         'help: the pad plane never comes within %.2f mm of axis B'
                         % (gap0, target, need))
    d = math.sqrt(d2)
    if abs(w_off) <= d:
        raise ValueError('module B\'s axis passes %.3f mm from the rotor pad axis, inside the '
                         '%.2f mm the pad must keep clear: a chord would take the whole pad'
                         % (abs(w_off), d))
    side = 1.0 if w_off < 0 else -1.0
    c = side * (w_off + side * d)
    n_w = num['w']
    trim = (side * n_w[0], side * n_w[1], c)
    gap, at = rotor_join_clearance(num, pad_r, trim)
    if gap < target - 1e-9:
        lo, hi = c, pad_r - 1e-3
        for _ in range(60):
            if hi - lo <= PAD_TRIM_TOL:
                break
            mid = (lo + hi) / 2.0
            t2 = (trim[0], trim[1], mid)
            if rotor_join_clearance(num, pad_r, t2)[0] >= target:
                hi = mid
            else:
                lo = mid
        c = hi
        trim = (trim[0], trim[1], c)
        gap, at = rotor_join_clearance(num, pad_r, trim)
        if gap < g.MIN_CLEARANCE - 1e-9:
            raise ValueError('no chord off the r %.1f rotor pad opens it to %.1f mm of '
                             'module B (best %.3f mm)' % (pad_r, g.MIN_CLEARANCE, gap))
    depth = g.pad_trim_depth(pad_r, trim)
    if depth >= 2.0 * pad_r - 1e-9:
        raise ValueError('the chord would take the whole r %.1f rotor pad' % pad_r)
    info.update({'trim': trim, 'gap': gap, 'at': at, 'depth': depth,
                 'why': 'the rotor pad came %.3f mm of module B at (s %.2f, w %.2f, z %.2f) '
                        'and needs %.1f (solved to %.1f, EXO-186); one chord %.3f mm deep, '
                        '%.2f mm of pad left across it, opens it to %.3f mm'
                        % (gap0, at0[0], at0[1], at0[2], g.MIN_CLEARANCE, target, depth,
                           2.0 * pad_r - depth, gap)})
    return trim, info


def _plate_radial_clearance(num, land, flange, step=0.5):
    """Smallest distance from axis A to the LANDING PLATE, over the part of it below
    `CLEAR_Z` - the plane under which module A's own body (and the previous link's landing
    plate) fill the r `CLEAR_R` cylinder. Returns (distance, (s, w, z)) or (None, None).

    EXO-178: the ribbon has been measured against that cylinder since EXO-144
    (`_body_radial_clearance`), but the PLATE never was, because a square tongue cannot reach
    there: its tip sits at `z_arr - half_t`, which is the bottom of the ribbon, and the ribbon
    is in the pad plane. An angled tongue leans out of that plane and can, so the plate is
    measured too. The plate is the outline swept between its two faces, so both faces are
    walked; `s` is the only coordinate that differs between them."""
    x_t = land['x_t']
    y_t = land['y_t']
    sc = num['slab_centre']
    s_lo, s_hi = num['s_slab']
    best, at = None, None
    for sg in flange:
        for px, py in g._seg_points(sg, step):
            w_p = sc[1] + px * x_t[1] + py * y_t[1]
            z_p = sc[2] + px * x_t[2] + py * y_t[2]
            if z_p >= CLEAR_Z:
                continue
            for s_p in (s_lo, s_hi):
                r = math.hypot(s_p, w_p)
                if best is None or r < best:
                    best, at = r, (s_p, w_p, z_p)
    return best, at


def cut_gap(route, step=0.25, margin=None):
    """The TRUE gap, in the plate's own plane, between the actuator OPENING and the ribbon's
    footprint - measured, not solved (EXO-176 evidence). `margin` grows the XL330's footprint
    to the opening: `ACT_OPEN_MARGIN` by default, the rectangle `cut_web` is solved on.

    `cut_web` is the number the landing is SOLVED on: the cutout's own reach ALONG the tongue,
    subtracted from how far out the ribbon lands. That is the support function of the cutout
    in one direction, so it is a lower bound on the real clearance and never flatters it. This
    walks the cutout's outline in the tongue frame and measures the real planar distance, so
    the two numbers together say how much of the web is the direction and how much is real.
    """
    num = route['numbers']
    R_s = route.get('R_t') or route['R_b']
    x_t = [R_s[i][0] for i in range(3)]
    y_t = [R_s[i][1] for i in range(3)]
    z_t = [R_s[i][2] for i in range(3)]
    t_ab, z_mid = num['t_ab'], g.STATOR_UNDERSIDE_Z - g.PAD_T / 2.0
    xb, yb, zb = num['x_b'], num['y_b'], num['z_b']
    origin = [t_ab[i] + z_mid * z_t[i] for i in range(3)]
    # measured against the SAME rectangle `cut_web` is solved on, or the two numbers are
    # not comparable and the 'gap >= web' evidence check is meaningless (EXO-187).
    margin = g.ACT_OPEN_MARGIN if margin is None else margin
    ax0, ax1, ay0, ay1 = g.actuator_rect(margin)
    corners = [(ax0, ay0), (ax1, ay0), (ax1, ay1), (ax0, ay1)]
    best = None
    for k in range(4):
        a, b = corners[k], corners[(k + 1) % 4]
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step) + 1)
        for i in range(n):
            f = i / (n - 1.0)
            bx, by = a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f
            pa = [t_ab[j] + xb[j] * bx + yb[j] * by + zb[j] * z_mid for j in range(3)]
            p = [pa[j] - origin[j] for j in range(3)]
            d = g._rect_point_dist((g.dot(p, x_t), g.dot(p, y_t)), *route['foot_box'])
            if best is None or d < best:
                best = d
    return best

# ---------------- the actuator is an EXCLUSION VOLUME (EXO-187) ----------------
# The XL330 is not a body in `sensor module 2`, so `check_interference` never saw a link
# sitting in it, and the plate's opening used to be cut to the actuator's own footprint (zero
# clearance) and to the plate's own depth only. The rule: `armgen.actuator_volume` - the
# footprint, from the actuator's back up to the stator underside - is the actuator's, and
# NON-plate material owes it MIN_CLEARANCE on top (`actuator_check` passes that margin
# explicitly; the mating OPENING itself grows by `ACT_OPEN_MARGIN` = 0, EXO-193). The
# landing plate is the one part the opening is cut THROUGH; every other
# solid the builder makes (ribbon, its EXO-151 collar, rotor pad, stop block, fused top) is
# sampled here, in pure Python, before anything is built, and refused BY NAME if it is inside.
COLLAR_PART = 'fillet collar'   # the EXO-151/EXO-183 blend; a tangent to the plate, not
                                # a body, so `actuator_check` bars it against the XL330's
                                # own box and every body against the grown volume
ACT_STEP = 0.5              # mm; how the link's surfaces are sampled against the volume
ACT_RING_EPS = 0.25         # mm; the ring outside the opening that must still be plate
ACT_RING_STEP = 1.0         # mm; how that ring and the opening's edge are walked
ACT_POLY_STEP = 1.0         # mm; how the plate outline is polygonised for the inside test


def _to_b(num, pa):
    """A frame-A point (mm) in the TRUE stator frame B, where the actuator is."""
    d = [pa[i] - num['t_ab'][i] for i in range(3)]
    return (g.dot(d, num['x_b']), g.dot(d, num['y_b']), g.dot(d, num['z_b']))


def _swz_to_a(num, p):
    u, w = num['u'], num['w']
    return [u[i] * p[0] + w[i] * p[1] + (p[2] if i == 2 else 0.0) for i in range(3)]


def _inside_poly(p, poly):
    """Even-odd point-in-polygon on a closed polyline (list of (x, y))."""
    x, y = p
    inside = False
    n = len(poly)
    for i in range(n):
        (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % n]
        if (y0 > y) != (y1 > y):
            if x0 + (y - y0) * (x1 - x0) / (y1 - y0) > x:
                inside = not inside
    return inside


def link_samples(route, step=ACT_STEP, collar=None):
    """Surface samples of every solid the builder makes for this route EXCEPT the landing
    plate, as (part, point) with the point in the rotor pad frame A (mm).

    The ribbon is its W x T section walked along the centre line, grown by the EXO-151 /
    EXO-183 collar within `collar` of the plate's near face; the rotor join is the pad's
    outline (the EXO-181 chord included) over its thickness, the stop block and the index
    tab over theirs. Surfaces only: against a convex box the nearest material is on one."""
    num = route['numbers']
    half_w, half_t = route['width'] / 2.0, route['thickness'] / 2.0
    collar = ELBOW_R_IN if collar is None else collar
    s_near = route['s_near']
    ks = (-1.0, -0.5, 0.0, 0.5, 1.0)
    out = []
    for sg in route['path']:
        for p in _sample(sg, step):
            d, n = _seg_frame(sg, p)
            wd = g.cross(d, n)
            grow = collar if abs(p[0] - s_near) <= collar else 0.0
            h_t, h_w = half_t + grow, half_w + grow
            part = 'ribbon' if grow == 0.0 else COLLAR_PART
            for kt in ks:
                for kw in ks:
                    if abs(kt) < 1.0 and abs(kw) < 1.0:
                        continue
                    q = [p[i] + n[i] * h_t * kt + wd[i] * h_w * kw for i in range(3)]
                    out.append((part, _swz_to_a(num, q)))
    pad_r = route['pad_r']
    rim = []
    if route.get('pad_outline'):
        for sg in route['pad_outline']:
            rim += g._seg_points(sg, step)
    else:
        n_th = max(8, int(2.0 * math.pi * pad_r / step) + 1)
        rim = [(pad_r * math.cos(2.0 * math.pi * k / n_th),
                pad_r * math.sin(2.0 * math.pi * k / n_th)) for k in range(n_th)]
    for (x, y) in rim:
        for z in (0.0, g.ROTOR_PAD_T / 2.0, g.ROTOR_PAD_T, max(g.PAD_T, g.ROTOR_PAD_T)):
            out.append(('rotor pad', [x, y, z]))
    # EXO-218: the stop is one solid sector of arc `b`, so that is the solid the actuator
    # volume and the interference check see.
    for (x, y) in g.stop_block_points(route['stop_angle'], route['stop_half'], pad_r,
                                      step=step):
        for z in (-g.STOP_H, 0.0):
            out.append(('stop block', [x, y, z]))
    # EXO-370: the fused rotor top under the pad (disk r 18 to the shoulder, journal to the
    # joint face, pegs to z -9.2), inside module A
    for (x, y) in g.top_rim_points(step):
        for z in (g.TOP_SHOULDER_Z, 0.0):
            out.append(('fused rotor top', [x, y, z]))
    for (x, y) in g.top_rim_points(step):
        k = g.TOP_JOURNAL_R / g.TOP_DISK_R
        for z in (g.TOP_JOINT_Z, g.TOP_SHOULDER_Z):
            out.append(('fused rotor top', [x * k, y * k, z]))
    return out


def actuator_check(route, m=None, step=ACT_STEP):
    """Every non-plate solid of the route against the actuator's exclusion volume (EXO-187).

    `m` defaults to MIN_CLEARANCE and stays there for every BODY the builder makes - the
    ribbon, the rotor pad, the stop block, the fused top: none of them mates with the
    actuator, so each owes it a real 1 mm.

    The EXO-151/EXO-183 COLLAR is the exception, and its bar is the XL330's own box
    (EXO-193). The collar is not a body, it is the blend between the ribbon and the landing
    PLATE, and it is TANGENT to the plate's near face: its nearest approach to the actuator
    is the point where it meets that face, which is the plate's own opening edge. With the
    opening cut to the footprint the plate stands there at 0.000 by Esteban's decision, and
    nothing the collar does can be closer than the plate it is tangent to - measured on
    link_23, `LAND_WEB` 3.0 and `ELBOW_R_IN` 3.0: the fillet's tangent line lands exactly on
    the opening edge and the arc runs AWAY from the actuator from there (at 2.62 mm below the
    face the true fillet stands 2.976 mm off; `link_samples` models it as its bounding box,
    which reads 0.000 all the way down that band). It is the same call EXO-183 made against
    module B, where a blend's bar is `CLEAR_R` and a body's is `CLEAR_R + MIN_CLEARANCE`.

    Returns a dict: `clear` is the smallest signed distance to the volume grown by each
    part's OWN bar (negative = inside, and `sweep_route` refuses the route by `part`), `at`
    that point in the TRUE stator frame B, `gap` the smallest distance from a BODY to the
    XL330's own box (need MIN_CLEARANCE), `collar_gap` the same for the collar (need 0), and
    `cable_exit` how far along +Y from the connector face the nearest
    material stands, BELOW the plate's far face and within the opening's x band - the room a
    cable leaving the two connectors has; None when nothing is down there. No cable or
    connector dimension is recorded in the repo, so that one is measured and reported, not
    enforced (EXO-192)."""
    m = g.MIN_CLEARANCE if m is None else m
    num = route['numbers']
    x0, x1, y0, y1, z0, z1 = g.actuator_volume(m)
    z_far = g.STATOR_UNDERSIDE_Z - g.PAD_T
    best, at, part, gap = None, None, None, None
    collar_gap, gap_part = None, None
    cable, cable_part = None, None
    for name, pa in link_samples(route, step):
        q = _to_b(num, pa)
        blend = (name == COLLAR_PART)
        d = g.actuator_box_dist(q, 0.0 if blend else m)
        if best is None or d < best:
            best, at, part = d, q, name
        d0 = g.actuator_box_dist(q, 0.0)
        if blend:
            if collar_gap is None or d0 < collar_gap:
                collar_gap = d0
        elif gap is None or d0 < gap:
            gap, gap_part = d0, name
        if x0 <= q[0] <= x1 and z0 <= q[2] <= z_far and q[1] > g.CUTOUT_Y[1]:
            free = q[1] - g.CUTOUT_Y[1]
            if cable is None or free < cable:
                cable, cable_part = free, name
    return {'clear': best, 'at': at, 'part': part, 'gap': gap, 'gap_part': gap_part,
            'collar_gap': collar_gap, 'm': m,
            'cable_exit': cable, 'cable_exit_part': cable_part}


def opening_in_plate(num, R_t, flange, m=None, eps=ACT_RING_EPS, step=ACT_RING_STEP,
                     poly_step=None):
    """(break-out point or None, thinnest plate round the opening, where) for one outline.

    The opening is fixed in the TRUE frame B and the plate is drawn in the tongue frame, so
    every point is carried B -> tongue frame. Split out of `plate_opening` because the
    landing plate's own radius is SOLVED against these two numbers (EXO-186)."""
    m = g.ACT_OPEN_MARGIN if m is None else m
    x_t = [R_t[i][0] for i in range(3)]
    y_t = [R_t[i][1] for i in range(3)]
    x0, x1, y0, y1 = g.actuator_rect(m)
    poly = []
    for sg in flange:
        poly += g._seg_points(sg, ACT_POLY_STEP if poly_step is None else poly_step)

    def tongue_xy(bx, by):
        d = [num['x_b'][k] * bx + num['y_b'][k] * by for k in range(3)]
        return (g.dot(d, x_t), g.dot(d, y_t))

    def walk(ax0, ax1, ay0, ay1):
        corners = [(ax0, ay0), (ax1, ay0), (ax1, ay1), (ax0, ay1)]
        for k in range(4):
            a, b = corners[k], corners[(k + 1) % 4]
            n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step) + 1)
            for i in range(n):
                f = i / (n - 1.0)
                yield (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)

    breakout = None
    for bx, by in walk(x0 - eps, x1 + eps, y0 - eps, y1 + eps):
        if not _inside_poly(tongue_xy(bx, by), poly):
            breakout = (bx, by)
            break
    web, web_at = None, None
    for bx, by in walk(x0, x1, y0, y1):
        d = g.outline_point_dist(tongue_xy(bx, by), flange)
        if web is None or d < web:
            web, web_at = d, (bx, by)
    return breakout, web, web_at


def plate_opening(route, m=None, eps=ACT_RING_EPS, step=ACT_RING_STEP):
    """What the actuator opening does to the landing plate (EXO-187). The opening is the
    footprint grown by `m` (`ACT_OPEN_MARGIN`, 0 - EXO-193), in the TRUE frame B; the plate
    is drawn in the tongue frame, so every point is carried B -> A -> tongue frame.

    `act_opening_in_plate`: a ring `eps` outside the opening lies wholly inside the plate's
    outline - the opening is cut through plate on all four sides and cuts nothing loose. When
    it does not, the opening BREAKS OUT of the outline at `act_breakout_at` (frame B): the
    plate has no web there and whatever plate lies beyond that edge is cut off - on link_67
    the +Y edge crosses the r 27 rim at x +-8.87 and the 66.7 deg tongue does not cover the
    cap, so the cut left a loose sliver (Fusion: 2 bodies). `sweep_route` refuses it by name.
    `act_cut_mm3` is then exactly the opening's area x PAD_T. `act_web_min` is the thinnest
    plate anywhere round the opening (the tongue-side walls of a 25 mm tongue are 1.5 mm),
    measured and reported; the web ALONG the tongue is `cut_web` and is enforced."""
    m = g.ACT_OPEN_MARGIN if m is None else m
    x0, x1, y0, y1 = g.actuator_rect(m)
    breakout, web, web_at = opening_in_plate(route['numbers'], route['R_t'], route['flange'],
                                             m, eps, step)
    return {'act_cut_mm3': (x1 - x0) * (y1 - y0) * g.PAD_T if breakout is None else None,
            'act_opening_in_plate': breakout is None, 'act_breakout_at': breakout,
            'act_web_min': web, 'act_web_at': web_at}


def stop_ribbon_overlap(route, spec, step=0.25):
    """EXO-204: does the generated link stop sit under the ribbon's ROOT on the rotor pad?

    REPORT ONLY. The block's angle is fixed by the joint's URDF limits and the ribbon's
    direction is fixed by the pair, so the only free variable that could separate them is
    the module's spin about its own axis - which is solved for other reasons (EXO-150) and
    is not touched here.

    The ribbon root is the ribbon's own surface where it crosses the pad disk: the
    `link_samples` points of the ribbon (and of the EXO-183 collar) that lie inside r
    `pad_r` and within the pad's thickness. Returns the shortest arc that covers them, the
    stop's sector, and `gap_deg` - how far the nearer edge of the block is from that arc,
    0.0 when they overlap.

    EXO-213 item 3 / EXO-216: it is measured over the block's OWN ANNULUS `r_in`..`pad_r`,
    not over the whole disk. A ribbon that crosses the pad centre covers every angle seen
    from that centre, so a whole-disk version answers 0.000 deg by arithmetic on any joint
    whose ribbon reaches in that far, which says nothing about the block. The block is back
    to one solid sector of arc `b` (EXO-218), so that is the sector measured to."""
    pad_r = route['pad_r']
    lo = math.degrees(spec['centre'] - spec['half']) % 360.0
    hi = math.degrees(spec['centre'] + spec['half']) % 360.0
    r_in = spec.get('r_in', g.STOP_R_IN)
    angs = []
    for part, p in link_samples(route, step=step):
        if part not in ('ribbon', COLLAR_PART):
            continue
        if not (-1e-6 <= p[2] <= g.PAD_T + 1e-6):
            continue
        rho = math.hypot(p[0], p[1])
        # EXO-216: only ribbon that crosses the block's own annulus r_in..pad_r can be over
        # the block. Measured over the whole disk the check said nothing on any joint but
        # J1 - a ribbon that crosses the pad centre covers every angle from that centre, so
        # the answer was 0.000 deg by construction (EXO-213 item 3).
        if rho > pad_r + 1e-6 or rho < r_in - 1e-6:
            continue
        angs.append(math.degrees(math.atan2(p[1], p[0])) % 360.0)
    if not angs:
        return {'clear': True, 'gap_deg': 360.0, 'band_deg': None,
                'stop_sector_deg': (lo, hi), 'n': 0}
    angs.sort()
    # the shortest arc covering every sample = the circle less its largest empty gap
    gaps = [(angs[(k + 1) % len(angs)] - angs[k]) % 360.0 for k in range(len(angs))]
    k = max(range(len(gaps)), key=lambda i: gaps[i])
    band = (angs[(k + 1) % len(angs)], angs[k])          # (start, end) going CCW

    def sep(a):
        """Angular distance from one sample angle to the stop block's sector."""
        d = (a - lo) % 360.0
        w = (hi - lo) % 360.0
        if d <= w:
            return 0.0
        return min(d - w, 360.0 - d)

    gap = min(sep(a) for a in angs)
    return {'clear': gap > 0.0, 'gap_deg': gap, 'band_deg': band,
            'stop_sector_deg': (lo, hi), 'n': len(angs)}


def hole_rings(route, ring=None):
    """The flat ring every screw hole has, in mm beyond its edge.

    Why three of the four seating faces need no solve at all in this method:
      * rotor pad, OUTER face and ROTOR-SIDE face - the ribbon is the pad's own thickness
        and leaves the pad IN ITS PLANE, and it is not allowed to turn out of that plane
        before `s_flat` (past the rim). Both faces are therefore flat over the whole disk,
        and the only things that bound them are the rim, the other holes and the link's own
        stop block. EXO-370: the rotor pad is the fused rotor top, so its ROTOR-SIDE face at
        the M3 holes is the top's joint face z -8.0, which bears on the bottom (no head or
        nut seats there) and is bounded by the O16 centre hole (1.4 mm ring by design):
        only the OUTER face z ROTOR_PAD_T, where the M3 heads seat, is a seating face.
      * landing plate, FAR face (the one that bolts to module B) - the ribbon dies `BURY`
        mm inside the plate and never reaches it, so that face is flat over the whole
        outline; the rim, the actuator cutout and the other inserts bound it.
    The one face that is measured rather than guaranteed is the plate's NEAR face, which
    the ribbon enters through: there its own W x T footprint blocks a strip.
    """
    need = g.HOLE_FLAT_RING if ring is None else ring
    pad_r, u, w = route['pad_r'], route['u'], route['w']
    stop_ang, stop_half = route['stop_angle'], route['stop_half']
    out = []
    holes = [(a, g.ROTOR_HOLE_RADIUS * math.cos(math.radians(a)),
              g.ROTOR_HOLE_RADIUS * math.sin(math.radians(a))) for a in g.ROTOR_HOLE_ANGLES]
    for face in ('outer',):
        for a, x, y in holes:
            r_h = math.hypot(x, y)
            bounds = [('the r %.1f pad rim' % pad_r, pad_r - r_h)]
            if g.ROTOR_PAD_T < g.PAD_T:
                # EXO-375: the ribbon is cut down to the pad face only inside the head flat
                bounds.append(('the r %.0f head flat (ribbon step)' % g.TOP_DISK_R,
                               g.TOP_DISK_R - r_h))
            trim = route.get('pad_trim')
            if trim is not None:
                bounds.append(('the EXO-181 chord %.3f mm into the pad'
                               % g.pad_trim_depth(pad_r, trim),
                               x * trim[0] + y * trim[1] - trim[2]))
            for b, ox, oy in holes:
                if b != a:
                    bounds.append(('the hole at %.0f deg' % b,
                                   math.hypot(ox - x, oy - y) - g.HOLE_R))
            if face == 'rotor-side':
                bounds.append(('the stop block at %.0f +/- %.0f deg' % (stop_ang, stop_half),
                               g._sector_point_dist((x, y), g.STOP_R_IN, pad_r, stop_ang,
                                                    stop_half)))
            why, d = min(bounds, key=lambda b: b[1])
            out.append({'pad': 'rotor', 'angle': a, 'face': face, 'ring': d - g.HOLE_R,
                        'need': need, 'bound': why})

    num = route['numbers']
    # EXO-176: the plate is drawn in the TONGUE's own frame - the snapped frame turned until
    # its +Y is the tongue. For the square landing that IS the snapped frame (theta = 0), so
    # every number below is unchanged there; for the angled one the inserts, the outline and
    # the ribbon's footprint are all read in the frame the tongue is straight in.
    R_s = route.get('R_t') or route['R_b']
    xs = [R_s[i][0] for i in range(3)]
    ys = [R_s[i][1] for i in range(3)]
    zs = [R_s[i][2] for i in range(3)]
    inserts = g.insert_slab_points(num, xs, ys, zs)
    fx0, fx1, fy0, fy1 = route['foot_box']
    for ins in inserts:
        x_h, y_h = ins['slab']
        bx, by = ins['b']
        others = [('the insert at %.0f deg' % o['angle'],
                   math.hypot(o['slab'][0] - x_h, o['slab'][1] - y_h) - g.HOLE_R)
                  for o in inserts if o is not ins]
        common = others + [
            ('the plate outline', g.outline_point_dist((x_h, y_h), route['flange'])),
            ('the actuator opening',
             g._rect_point_dist((bx, by), *g.actuator_rect(g.ACT_OPEN_MARGIN)))]
        for face in ('untouched', 'entered'):
            bounds = list(common)
            if face == 'entered':
                bounds.append(('the ribbon footprint',
                               g._rect_point_dist((x_h, y_h), fx0, fx1, fy0, fy1)))
            why, d = min(bounds, key=lambda b: b[1])
            out.append({'pad': 'stator', 'angle': ins['angle'], 'face': face,
                        'ring': d - g.HOLE_R, 'need': need, 'bound': why})
    return out


# ---------------- the module's rotation about its own axis: FIXED by the link (EXO-150) ----
# EXO-144 left the spin free and SEARCHED it (largest smallest stator ring, then the shortest
# path), which put the encoders anywhere: J2 +248.5, J3 +238.5, J4 +174.5, J5 +238.5 deg. The
# encoder / connector block is nearly as wide as the r 27 pad and overhangs its rim whatever
# the spin is, so the spin is not free: it decides WHERE that overhang sticks out. EXO-150
# fixes it by rule and turns the hole / cutout solve into a CHECK.


def _stator_margin(route):
    return min(h['ring'] for h in route['rings'] if h['pad'] == 'stator')


def path_length(route):
    """How much ribbon the route sweeps: the centre line's own length, mm."""
    L = 0.0
    for sg in route['path']:
        if sg[0] == 'line':
            L += math.sqrt(sum((sg[2][i] - sg[1][i]) ** 2 for i in range(3)))
        else:
            _, p0, pm, p1, c, r = sg
            a = [p0[i] - c[i] for i in range(3)]
            b = [p1[i] - c[i] for i in range(3)]
            L += r * math.acos(max(-1.0, min(1.0, g.dot(a, b) / (r * r))))
    return L


def arrival_sign(num):
    """Which side of module B's own axis the link ARRIVES FROM, as a sign along the rotor pad
    axis z_A of frame A.

    The ribbon is the rotor pad's own thickness and leaves it IN the pad plane (z = PAD_T / 2),
    and it only ever leaves that plane because the landing forces it to. So the side of module
    B's axis the link comes in on is simply the side the rotor pad plane is on: `+1` when the
    pad plane is above B's axis, `-1` when it is below. `landing_numbers` calls the same number
    `y_sign` once the module is spun, and this is the rule that CHOOSES it instead of reading it
    off whatever pose the seed happened to carry."""
    return 1.0 if g.PAD_T / 2.0 > num['axial'] else -1.0


def encoder_spin(R_ab, t_ab, tongue_deg=0.0):
    """EXO-150, generalised by EXO-176: module B's rotation about its own axis, FIXED BY THE
    LINK, never searched.

    **The rule.** Spin the module so its **+Y** - the encoder / connector side, whose block is
    nearly as wide as the r 27 pad and overhangs the rim at any spin - faces ALONG THE TONGUE
    of the landing plate, which is the direction the link arrives from across the pad. The
    plate already wraps the actuator cutout and reaches the three inserts, so it then extends
    the pad outward exactly where the encoder overhangs it.

    EXO-150 could say `Y_B = arrival_sign * z_A` because the tongue was pinned to the rotor
    pad axis. EXO-176 frees the tongue's direction inside the plate plane, so the rule is
    stated on the tongue itself:

        Y_B = the tongue, = sin(tongue) * x_s + cos(tongue) * y_s,   y_s = arrival_sign * z_A

    and `tongue_deg` 0 gives EXO-150's own number back, exactly. `tongue_deg` is not an input:
    `spin_check` solves it as the fixpoint of 'spin the module along the tongue the route then
    draws'.

    Only solvable because both arms put every stator axis IN the rotor pad plane (EXO-88):
    that is what puts the whole plate plane in the span of X_B and Y_B, so some rotation about
    Z_B reaches any direction in it. Raises ValueError naming the number when it does not.

    Returns (spin_deg in [0, 360), info).
    """
    num = g.landing_numbers(R_ab, t_ab)
    xb = [R_ab[i][0] for i in range(3)]
    yb = [R_ab[i][1] for i in range(3)]
    zb = [R_ab[i][2] for i in range(3)]
    if abs(zb[2]) > AXIS_SQUARE_MAX:
        raise ValueError('the stator axis is %.4f out of the rotor pad plane, so no rotation '
                         'about it can point the encoder along the pad axis (max %.0e)'
                         % (abs(zb[2]), AXIS_SQUARE_MAX))
    sgn = arrival_sign(num)
    th = math.radians(tongue_deg)
    y_s = [0.0, 0.0, sgn]
    x_s = g.cross(y_s, num['z_s'])
    d = [math.sin(th) * x_s[i] + math.cos(th) * y_s[i] for i in range(3)]
    spin = math.degrees(math.atan2(-g.dot(d, xb), g.dot(d, yb))) % 360.0
    R_spun = g.spin_frame(R_ab, spin)
    y_new = [R_spun[i][1] for i in range(3)]
    got = g.dot(y_new, d)
    residual = math.degrees(math.acos(max(-1.0, min(1.0, abs(got)))))
    if got < 0.999:
        raise ValueError('the encoder rule left +Y %.4f deg off the tongue (Y . tongue '
                         '%+.6f, wanted +1)' % (residual, got))
    return spin, {
        'arrival_sign': sgn, 'residual_deg': residual, 'seed_y_axial': yb[2],
        'axial': num['axial'], 'pad_plane_z': g.PAD_T / 2.0, 'tongue_deg': tongue_deg,
        'why': 'EXO-150/176: +Y (the encoder side) runs ALONG THE TONGUE, %.3f deg off the '
               'rotor pad axis - the rotor pad plane is %s module B\'s axis (z %.2f vs '
               '%.2f), so the tongue and the encoder both lean %s z_A'
               % (tongue_deg, 'above' if sgn > 0 else 'below', g.PAD_T / 2.0, num['axial'],
                  '+' if sgn > 0 else '-'),
    }


SPIN_QUARTER = 90.0         # deg; the module's spin is a multiple of this off the pad axis
SPIN_TOL = 1e-6             # deg; below this the snapped tongue IS the pad axis


def spin_check(R_ab, t_ab, pad_r, width, quarter=SPIN_QUARTER, tol=SPIN_TOL, **kw):
    """Size the route at the spin the rule fixes and CHECK it (EXO-150, EXO-176, EXO-186).

    Nothing is searched and nothing is nudged: `sweep_route` already refuses by name when a
    screw head would not seat (`HOLE_FLAT_RING`), when the plate keeps less than `LAND_WEB` of
    web past the actuator cutout, or when the ribbon fouls either module, so calling it at the
    fixed spin IS the check.

    **The spin is AXIS-ALIGNED** (EXO-186). Spin about the module's own axis is kinematically
    free (EXO-171), so what it decides is how the encoder / connector block LOOKS and where
    the cable leaves: the module is turned to the multiple of `quarter` - +Y in or normal to
    the arriving link's pad plane - NEAREST the tongue. EXO-176/180 pointed +Y along the
    tongue itself, which on link_67's 66.736 deg landing left J7 visibly crooked; hand-made
    Link 6 R keeps it square and widens the plate instead, which is what `land_r` now does.

    So: the tongue is solved ONCE, at the EXO-150 spin, the spin is snapped ONCE to the
    nearest quarter turn, and the route is sized there. No fixpoint, nothing to fail to
    settle (EXO-192's J5 2-cycle cannot happen). A square landing draws its tongue at 0, its
    nearest quarter turn IS 0, and every EXO-150 number is unchanged - to the bit.

    Returns (route, fit); `spin_walk` is [(tongue asked, tongue drawn)] per sizing."""
    spin, info = encoder_spin(R_ab, t_ab, 0.0)
    route = sweep_route(g.spin_frame(R_ab, spin), t_ab, pad_r, width, **kw)
    th, walk = route['tongue_deg'], [(0.0, route['tongue_deg'])]
    th_snap = quarter * round(th / quarter)
    if abs(th_snap) > tol:
        spin, info = encoder_spin(R_ab, t_ab, th_snap)
        route = sweep_route(g.spin_frame(R_ab, spin), t_ab, pad_r, width, **kw)
        walk.append((th_snap, route['tongue_deg']))
    info['tongue_solved_deg'] = th
    info['spin_snap_deg'] = th_snap
    rings = route['rings']
    fit = dict(info)
    # the angle between the encoder (+Y, = the tongue by the rule) and the direction from
    # module B's axis to where the ribbon actually lands on the plate. 0 by construction once
    # the fixpoint has settled; it is reported so that the rule is visibly a CHECK.
    fit.update({
        'spin': round(spin, 6),
        'encoder_link_deg': math.degrees(math.atan2(route['x_off'], route['y_land'])),
        'margin': _stator_margin(route),
        'rotor_ring': min(h['ring'] for h in rings if h['pad'] == 'rotor'),
        'web': route['cut_web'], 'path': path_length(route),
        'drop': route['drop'], 'y_land': route['y_land'], 'x_off': route['x_off'],
        'cut_reach': route['cut_reach'], 'landing': route['landing'],
        'tongue_deg': route['tongue_deg'], 'cut_gap': route['cut_gap'],
        'act_gap': route['act_gap'], 'act_part': route['act_gap_part'],
        'act_collar_gap': route['act_collar_gap'],
        'land_r': route['land_r'], 'spin_walk': walk,
    })
    return route, fit


# ---------------- the module's own axis direction: the URDF fixes the LINE, not the way
# it points (EXO-158). `axis_sign` flips the module end for end - which side of the joint the
# stator body hangs on, and therefore which side the landing plate bolts to. It is not taste
# and it is not copied off the hand-built arm: it is derived where the pair's own geometry
# decides it, and SOLVED where the pair says nothing.
AXIS_SIGN_MIN = 1e-3            # |axis . step| below this and the step decides nothing


def axis_sign_rule(axis_w, t_prev, t_joint):
    """Which way a module's +Z (its rotor-face normal) points, from the URDF alone.

    The link arrives from the previous joint and lands on THIS module's underside, so the
    rotor face wants to point on down the arm, away from where the link came from:

        axis_sign = sign(axis . (t_joint - t_prev))

    On G1 that decides J3 (+0.1032) and J5 (-0.1000) and it agrees with the poses EXO-103 /
    EXO-114 measured in the exo STEP. It decides NOTHING when the axis is square to the step,
    which is G1's J2, J4, J6 and J7 (|axis . step| <= 1e-16): there the two signs put the
    module body on opposite sides of a joint the link reaches sideways, and the rule returns
    None so the caller SOLVES the sign the way the spin is solved - keep the ones whose link
    routes, then the shortest route.

    Returns (+1 / -1 / None, why)."""
    d = [t_joint[i] - t_prev[i] for i in range(3)]
    n = math.sqrt(g.dot(d, d))
    if n < 1e-12:
        return None, 'the two joints share an origin: the step decides nothing'
    k = g.dot(axis_w, d) / n
    if abs(k) > AXIS_SIGN_MIN:
        return (1 if k > 0 else -1), ('the rotor face points on down the arm '
                                      '(axis . step %+.4f)' % k)
    return None, ('the axis is square to the step from the previous joint '
                  '(axis . step %+.2e): the sign is free and is solved by the link' % k)


# ---------------- the module's position ALONG its own axis: `slide` (EXO-170) ----------
# A revolute joint fixes a module's AXIS LINE, not where the module sits on it. `axis_sign`
# flips the module end for end; nothing translated it, so two modules whose joints are closer
# together than the bodies are wide simply overlapped (EXO-163: J6 and J7 are 46.00 mm apart
# and 533.5 mm3 of the two stators share space). `slide` is that missing degree of freedom:
# the module's position along its own Z, in mm, measured from the URDF joint origin. It moves
# nothing kinematic.
#
# It is NEVER an input and there is no per-joint override. `solve_slides` reads the module
# BODIES - `armgen.MODULE_PROFILE`, the real solid of revolution, not EXO-163's infinite
# cylinder - and solves the smallest slide that clears every pair.
SLIDE_MARGIN = 1.0          # mm of gap asked for BEYOND `MIN_CLEARANCE`, so the solved slide
                            # does not sit exactly on the threshold. A default, stated.
SLIDE_MAX = 120.0           # mm; past this a slide is not the fix and the pair is reported
SLIDE_STEP = 2.0            # mm; the coarse scan that brackets the first clearing slide
SLIDE_TOL = 1e-3            # mm; the bisection inside that bracket
SLIDE_N_THETA = 120         # how the module surface is sampled for the gap (EXO-166 used
SLIDE_N_Z = 71              # 180 x 71 and 360 x 141; 120 x 71 reproduces it to 1e-3 mm)

_SAMPLE_CACHE = {}


def module_samples(prof=None, n_theta=SLIDE_N_THETA, n_z=SLIDE_N_Z):
    """The module's outer surface as points in its own frame: the lateral surface of every
    band of `armgen.MODULE_PROFILE`, at `n_z` axial stations and `n_theta` angles. Cached -
    every module in the arm is the same `sensor module 2`."""
    prof = g.MODULE_PROFILE if prof is None else prof
    key = (prof, n_theta, n_z)
    if key in _SAMPLE_CACHE:
        return _SAMPLE_CACHE[key]
    z_lo, z_hi = prof[0][0], prof[-1][1]
    pts = []
    for i in range(n_z):
        zi = z_lo + i * (z_hi - z_lo) / (n_z - 1.0)
        r = max(rr for a, b, rr in prof if a - 1e-9 <= zi <= b + 1e-9)
        for k in range(n_theta):
            ang = 2.0 * math.pi * k / n_theta
            pts.append((r * math.cos(ang), r * math.sin(ang), zi))
    _SAMPLE_CACHE[key] = pts
    return pts


def module_gap(R_ab, t_ab, prof=None, pts=None):
    """Smallest distance between two module bodies, mm. `R_ab`, `t_ab` place module B in
    module A's frame. 0.0 means they interfere - no negative depth is produced.

    Module A's surface is sampled (`module_samples`) and every sample is measured against
    module B's profile of revolution. This is EXO-166's `module_gap`, moved here so the
    generator owns it: it reproduces the numbers that issue reports (3.221 mm at the hand
    arm's 23.000 mm slide) to 1e-2 mm."""
    prof = g.MODULE_PROFILE if prof is None else prof
    pts = module_samples(prof) if pts is None else pts
    r0 = (R_ab[0][0], R_ab[1][0], R_ab[2][0])
    r1 = (R_ab[0][1], R_ab[1][1], R_ab[2][1])
    r2 = (R_ab[0][2], R_ab[1][2], R_ab[2][2])
    tx, ty, tz = t_ab[0], t_ab[1], t_ab[2]
    best = 1e18
    for p in pts:
        d0 = p[0] - tx
        d1 = p[1] - ty
        d2 = p[2] - tz
        qx = r0[0] * d0 + r0[1] * d1 + r0[2] * d2
        qy = r1[0] * d0 + r1[1] * d1 + r1[2] * d2
        qz = r2[0] * d0 + r2[1] * d1 + r2[2] * d2
        rho = math.sqrt(qx * qx + qy * qy)
        for z0, z1, rr in prof:
            dz = z0 - qz
            if dz < 0.0:
                dz = qz - z1
                if dz < 0.0:
                    dz = 0.0
            dr = rho - rr
            if dr < 0.0:
                dr = 0.0
            v = dz * dz + dr * dr
            if v < best:
                best = v
        if best <= 0.0:
            return 0.0
    return math.sqrt(best)


def slid_offset(R_ab, t_ab, slide_a=0.0, slide_b=0.0):
    """`t_ab` after module A is slid `slide_a` mm along its own Z and B `slide_b` along its.

    A's own Z is frame A's e_z, so sliding A subtracts from `t_ab`'s third component; B's own
    Z is the third COLUMN of `R_ab` expressed in A. `R_ab` itself never changes - a slide is
    a translation along an axis that is already a column of the rotation."""
    zb = (R_ab[0][2], R_ab[1][2], R_ab[2][2])
    return [t_ab[0] + slide_b * zb[0],
            t_ab[1] + slide_b * zb[1],
            t_ab[2] + slide_b * zb[2] - slide_a]


def pair_gap(RA, tA, RB, tB, slide_a=0.0, slide_b=0.0):
    """`module_gap` for two module frames as the chain carries them (`t` in METRES)."""
    R_ab, t_ab = g.relative_frame(RA, tA, RB, tB)
    return module_gap(R_ab, slid_offset(R_ab, t_ab, slide_a, slide_b))


def clearing_slide(R_ab, t_ab, which, sign, target, base_a=0.0, base_b=0.0, fits=None):
    """The SMALLEST extra slide of one module of a pair, in one direction, that makes the
    pair FIT. Returns (slide, gap) or (None, why).

    Fitting is two things and the second one is optional (EXO-214):

      * the two module BODIES open to `target` mm - EXO-171's rule, and all there was;
      * `fits(s)` - whatever else the caller needs the pair to satisfy at that slide. The
        builder passes the whole-pair test: the link between the two modules routes, with
        its own link stop, and any pad chord it still needs misses the stop block.

    Neither is monotone - a module leaving one neighbour can swing past a shoulder of the
    other, and a route can refuse on either side of a slide that works - so the first
    crossing is bracketed by a coarse scan at `SLIDE_STEP` and bisected to `SLIDE_TOL`. The
    FIRST crossing is the answer, by the rule: the smallest."""

    def gp(s):
        sa = base_a + (sign * s if which == 'a' else 0.0)
        sb = base_b + (sign * s if which == 'b' else 0.0)
        return module_gap(R_ab, slid_offset(R_ab, t_ab, sa, sb))

    def ok(s):
        # the body gap is the cheap half and a necessary condition, so it is asked first
        # and `fits` - which sizes routes - is only ever called where it can pass.
        if gp(s) < target:
            return False
        return True if fits is None else fits(s)

    lo = 0.0
    if ok(0.0):
        return 0.0, gp(0.0)
    s = SLIDE_STEP
    while s <= SLIDE_MAX + 1e-9:
        if ok(s):
            hi = s
            while hi - lo > SLIDE_TOL:
                mid = (lo + hi) / 2.0
                if ok(mid):
                    hi = mid
                else:
                    lo = mid
            return hi, gp(hi)
        lo = s
        s += SLIDE_STEP
    return None, ('no slide up to %.0f mm %s' % (SLIDE_MAX,
                  'opens this pair to %.2f mm' % target if fits is None
                  else 'opens this pair to %.2f mm AND makes it fit' % target))


def module_pairs(frames, order, slides=None):
    """Every module pair in the chain with its body-to-body gap, nearest first."""
    slides = {} if slides is None else slides
    out = []
    for i in range(len(order)):
        for j in range(i + 1, len(order)):
            a, b = order[i], order[j]
            RA, tA = frames[a]
            RB, tB = frames[b]
            out.append((a, b, pair_gap(RA, tA, RB, tB, slides.get(a, 0.0),
                                       slides.get(b, 0.0))))
    out.sort(key=lambda r: r[2])
    return out


def route_fits(frames, links, stops=None, target=None):
    """EXO-214: does the WHOLE PAIR fit at these frames - not just the two module bodies?

    Walks the chain exactly as `BuildArmGen` builds it - `solve_spins` fixes every module's
    rotation by the EXO-150 rule, then each link is sized at its own joint's LINK STOP - and
    returns (ok, why, pair, per_pair). `pair` is the two joint labels of the first link that
    does not fit, which is the pair a slide has to answer for; `per_pair` is
    {(a, b): (ok, why)} for EVERY link.

    EXO-218: every link is answered, not just the first that fails. The rule is about the
    PAIR - "the slide is the minimum that makes the whole pair fit" (EXO-215) - and on an
    arm where more than one pair starts out refused, an all-or-nothing answer makes the
    slide for pair 1 wait on pair 3, which no slide of pair 1's modules can do anything
    about. Luna starts with five of six links refused, so `solve_slides` asks whether THIS
    pair fits and whether any pair that already fitted stopped fitting.

    Three things have to hold for every link, and each of them is a rule that already
    existed; what is new is that the SLIDE is solved against them and not only against the
    module bodies:

      * the route sizes at all (`sweep_route` refuses by name: screw rings, the actuator
        web, either module);
      * whatever pad chord EXO-181 still needs misses the link stop block by
        `STOP_CHORD_MARGIN` - `sweep_route` raises when it does not;
      * the link's ROTOR JOIN clears module B by `target`, which defaults to the same
        `MIN_CLEARANCE + SLIDE_MARGIN` the bodies are asked for and is exactly what
        `solve_pad_trim` opens a chord to. Without it the solve would stop the instant the
        join reached `MIN_CLEARANCE`, i.e. with its answer sitting on the refusal threshold
        (EXO-182); with it, the slide buys the same margin everywhere.
    """
    target = g.MIN_CLEARANCE + SLIDE_MARGIN if target is None else target
    stops = {} if stops is None else stops
    _spun, table = solve_spins(frames, links, stops=stops)
    per, first = {}, None
    for name, a, b, pad_r, width in links:
        if table[b]['refused'] is not None:
            per[(a, b)] = (False, '%s: %s' % (name, table[b]['refused']))
        else:
            route = table[b]['route']
            gap, _at = rotor_join_clearance(route['numbers'], pad_r, route['pad_trim'])
            if gap < target - 1e-6:
                per[(a, b)] = (False, ('%s: the rotor join clears module B by %.3f mm, '
                                       'under the %.2f mm this solve asks for'
                                       % (name, gap, target)))
            else:
                per[(a, b)] = (True, '')
        if first is None and not per[(a, b)][0]:
            first = (a, b)
    if first is None:
        return True, '', None, per
    return False, per[first][1], first, per


def solve_slides(frames, order, target=None, max_rounds=8, fits=None):
    """EXO-170, generalised by EXO-214: solve every module's position along its own axis.

    **The rule.**

    1. `slide` is 0 everywhere the pair already fits. Nothing moves that does not have to.
    2. A pair is slid when its module BODIES come closer than `armgen.MIN_CLEARANCE`
       (EXO-171) or when `fits` says the pair does not fit for any other reason - on this
       arm, that the link between the two modules cannot be routed with its own link stop
       (EXO-214). Either way ONE of the two modules slides along its OWN axis. There are
       four candidates - each module, each direction - and each is solved for the SMALLEST
       slide that opens the pair to `MIN_CLEARANCE + SLIDE_MARGIN` **and** satisfies `fits`.
       A candidate that no slide up to `SLIDE_MAX` fixes, or that makes some OTHER pair
       WORSE than it already is while leaving it under `MIN_CLEARANCE`, is dropped
       (EXO-218: comparative, because several pairs can start out bad at once).
    3. The rule takes the smallest solved slide; an exact tie goes to the module nearer the
       base, so the choice is a function of the chain and nothing else. Every candidate is
       returned, so the margin the choice was made by is visible.
    4. Slides accumulate and the whole set of pairs is re-checked, until nothing is closer
       than `MIN_CLEARANCE`, `fits` passes, or nothing more can be done.

    `fits(slides) -> (ok, why, pair)` is the caller's, because it needs the URDF and the
    joint limits to re-seed the chain; `sweepgen.route_fits` is the one the builder passes.
    With `fits=None` this is EXO-170's body-only solve, unchanged to the bit.

    The axis LINE is untouched at every step, so the kinematics stay the URDF's exactly.

    Returns (slides, rows): `slides[label]` in mm along that module's own Z, and one row per
    pair the solve had to fix, carrying every candidate it sized."""
    target = g.MIN_CLEARANCE + SLIDE_MARGIN if target is None else target
    slides = dict((lab, 0.0) for lab in order)
    rows = []
    for _round in range(max_rounds):
        pairs = module_pairs(frames, order, slides)
        # EXO-218: one `fits` call per round, so the round knows which pairs fitted BEFORE
        # anything moved. That is what the guard below compares against.
        state = fits(slides) if fits is not None else None
        per_before = (state[3] if state is not None and len(state) > 3 else None) or {}
        bad = [p for p in pairs if p[2] < g.MIN_CLEARANCE - 1e-9]
        if bad:
            a, b, gap0 = bad[0]
            trigger = ('the two module bodies are %.3f mm apart, under MIN_CLEARANCE %.1f'
                       % (gap0, g.MIN_CLEARANCE))
        elif fits is None:
            break
        else:
            ok_fit, why_fit, pair = state[0], state[1], state[2]
            if ok_fit:
                break
            if pair is None or pair[0] not in slides or pair[1] not in slides:
                rows.append({'pair': pair, 'gap0': None, 'taken': None, 'candidates': [],
                             'trigger': why_fit,
                             'why': 'the pair that does not fit is not a module pair'})
                break
            a, b = pair
            if order.index(b) < order.index(a):
                a, b = b, a
            gap0 = dict(((p[0], p[1]), p[2]) for p in pairs)[(a, b)]
            trigger = ('the bodies clear by %.3f mm but the pair does not fit: %s'
                       % (gap0, why_fit))
        RA, tA = frames[a]
        RB, tB = frames[b]
        R_ab, t_ab = g.relative_frame(RA, tA, RB, tB)
        # every pair's gap BEFORE this round, so the guard below can ask whether a candidate
        # made a pair worse rather than whether it is under the bar (EXO-218).
        before = dict(((p[0], p[1]), p[2]) for p in pairs)
        cands = []
        for lab, which in ((a, 'a'), (b, 'b')):
            for sign in (+1, -1):
                def _fits_s(_lab=lab, _sign=sign, _a=a, _b=b, _before=per_before):
                    if fits is None:
                        return None

                    def f(s):
                        trial = dict(slides)
                        trial[_lab] = trial[_lab] + _sign * s
                        r = fits(trial)
                        per = r[3] if len(r) > 3 else None
                        if per is None:
                            return r[0]
                        key = (_a, _b) if (_a, _b) in per else (_b, _a)
                        if not per.get(key, (True, ''))[0]:
                            return False
                        # EXO-218, the same comparative guard the bodies get: this slide has
                        # to make THIS pair fit and must not stop any pair that already
                        # fitted from fitting. A pair that was already refused is left where
                        # it was - it gets its own round.
                        for k, st in per.items():
                            if k != key and not st[0] and _before.get(k, (False, ''))[0]:
                                return False
                        return True
                    return f
                s, got = clearing_slide(R_ab, t_ab, which, sign, target,
                                        slides[a], slides[b], fits=_fits_s())
                if s is None:
                    cands.append({'module': lab, 'sign': sign, 'slide': None,
                                  'gap': None, 'why': got})
                    continue
                trial = dict(slides)
                trial[lab] = trial[lab] + sign * s
                after = dict(((p[0], p[1]), p[2])
                             for p in module_pairs(frames, order, trial))
                worst = min(after.values())
                # EXO-218: the guard is COMPARATIVE. EXO-171 wrote it as "a candidate that
                # closes some other pair below MIN_CLEARANCE is dropped" and implemented it
                # as `min(all gaps) >= MIN_CLEARANCE`, which is the same thing only while
                # exactly one pair is bad at a time - true on G1. Luna has three coincident
                # modules, so several pairs are at 0.000 mm before anything moves, and the
                # absolute form dropped EVERY candidate for a pair it had not reached yet:
                # no slide can open R1-R2 AND R5-R6 at once, and R5-R6 has its own turn.
                # So a candidate is dropped when it makes another pair WORSE than it
                # already is - never for leaving an already-bad pair where it found it.
                spoilt = [(k, after[k], before[k]) for k in after
                          if k != (a, b)
                          and after[k] < min(g.MIN_CLEARANCE, before[k]) - 1e-9]
                cands.append({'module': lab, 'sign': sign, 'slide': s, 'gap': got,
                              'worst_other': worst, 'spoilt': spoilt,
                              'why': ('' if not spoilt else
                                      'it closes %s <-> %s to %.3f mm (it was %.3f)'
                                      % (spoilt[0][0][0], spoilt[0][0][1], spoilt[0][1],
                                         spoilt[0][2]))})
        ok = [c for c in cands if c['slide'] is not None and not c['why']]
        if not ok:
            rows.append({'pair': (a, b), 'gap0': gap0, 'taken': None,
                         'candidates': cands, 'trigger': trigger,
                         'why': 'no slide of either module makes this pair fit'})
            break
        ok.sort(key=lambda c: (round(c['slide'], 6), order.index(c['module']),
                               -c['sign']))
        take = ok[0]
        slides[take['module']] += take['sign'] * take['slide']
        rows.append({'pair': (a, b), 'gap0': gap0, 'taken': take, 'candidates': cands,
                     'trigger': trigger, 'why': ''})
    return slides, rows


def solve_spins(frames, links, stops=None, **kw):
    """Fix the whole chain's spins by the EXO-150 rule and check each link at its own.

    `stops` (EXO-214) is {rotor joint label: the `stop_angle / stop_half / stop_h / stop_why`
    that joint's own URDF limits give it}, merged into `kw` for that link alone. It changes
    no spin: the spin comes from `encoder_spin` and the tongue, and the link stop enters
    `sweep_route` only as a CHECK (the EXO-181 chord against the block, and the screw
    rings). A link the stop refuses is therefore sized a second time without it, purely to
    get the same spin the chain had before there were link stops, and the refusal is carried
    on the row. Each row also carries the `route` it was sized from, so a caller that builds
    does not size it again.

    A joint is the stator end of one link and the rotor end of the next, so the links are
    walked in order and each one's stator joint is turned before the next is sized. The first
    joint is nobody's stator end: it keeps its measured pose (EXO-150 says so in as many
    words), which is spin 0 off the `y_hint` seed.

    Returns (frames, table). `table[label]` carries the spin AND the check numbers, so a
    caller that reports can report the check without sizing the route again; `refused` is
    the ValueError `sweep_route` raised, or None."""
    frames = dict(frames)
    table, routes = {}, {}
    for name, a, b, pad_r, width in links:
        RA, tA = frames[a]
        RB, tB = frames[b]
        R_ab, t_ab = g.relative_frame(RA, tA, RB, tB)
        # The spin is fixed by `encoder_spin` alone, so a pair whose ROUTE refuses still
        # turns its module and the chain walks on; the refusal is carried on the row and
        # reported by name. A link nobody can draw is still a link nobody builds (EXO-82) -
        # `sweep_route` is the thing that raised, and the caller decides what to do about it.
        spin, info = encoder_spin(R_ab, t_ab)
        per = dict(kw)
        per.update((stops or {}).get(a, {}))
        try:
            route, fit = spin_check(R_ab, t_ab, pad_r, width, **per)
            fit['refused'] = None
            routes[name] = route
        except ValueError as e:
            routes[name] = None
            fit = None
            if per != kw:
                # refused WITH this joint's link stop: size it again without, so the module
                # still turns by the rule and the chain walks on unchanged.
                try:
                    _r2, fit = spin_check(R_ab, t_ab, pad_r, width, **kw)
                    fit['refused'] = str(e)
                except ValueError:
                    fit = None
            if fit is None:
                fit = dict(info, spin=round(spin, 6), margin=None, encoder_link_deg=None,
                           refused=str(e))
        fit['route'] = routes[name]
        frames[b] = (g.frame_spun(RB, fit['spin']), tB)
        table[b] = fit
    for name, a, b, pad_r, width in links:
        table.setdefault(a, {'spin': 0.0, 'margin': None, 'residual_deg': None,
                             'encoder_link_deg': None, 'refused': None, 'route': None,
                             'why': "nobody's stator end: the measured pose is kept (EXO-150)"})
    return frames, table


# ---------------- the chain solve (EXO-218) ----------------
# These four used to live in `BuildArmGen`, which imports Fusion and so could only ever be
# run inside Fusion. Nothing in them touches Fusion - they are the URDF, the module frames
# and the router - and the solve they do (2^n axis-sign combinations, each with its own
# slide solve) is the expensive part of a build. Moving them here makes the WHOLE answer
# reproducible and testable outside Fusion, which is what `BuildArmGen.LUNA_SPEC`'s
# precomputed `solved` block is verified against.
#
# An ARM SPEC is a dict; `BuildArmGen` holds the two (G1 and Gento Luna) and nothing here
# reads a joint or a link name.


def spec_joints(spec):
    """Every joint of an arm, base first."""
    return list(spec['fixed']) + list(spec['derived'])


def spec_links(spec):
    """The link table in the (name, a, b, pad_r, width) tuples the solvers take."""
    return [(L['name'], L['a'], L['b'], L['pad_r'], L['width']) for L in spec['links']]


def first_hint(spec, label, name, joints, by_child):
    """The +Y seed for a module that NO link arrives at - the first joint of the chain.

    EXO-150 fixes a module's spin by pointing its encoder at the direction the link arrives
    from, and `solve_spins` says in as many words that the first joint is nobody's stator
    end: it keeps the pose its `y_hint` seeds. On G1 that seed is the pose measured in the
    exo STEP. Luna has no measured pose, so the seed is taken from the URDF the same way the
    rule takes everything else: +Y points along the arm, at the first joint DOWN THE CHAIN
    whose origin is not this joint's own (R1 and R2 are coincident, so it is R3), projected
    off the axis by `module_frame`. Where even that is parallel to the axis it falls back to
    world +Z and says so. This is a DEFAULT, reported at every run, not a measurement."""
    t, ax = g.joint_world_pose(name, joints, by_child)
    for j in spec_joints(spec):
        if j['label'] == label:
            continue
        t2, _a = g.joint_world_pose(j['name'], joints, by_child)
        d = [t2[i] - t[i] for i in range(3)]
        if math.sqrt(g.dot(d, d)) > 1e-9:
            if abs(g.dot(g.unit(d), ax)) <= 0.9:
                return d, ('DEFAULT (EXO-218): no link arrives at %s, so its +Y seeds along '
                           'the arm, at %s - the first joint down the chain with an origin '
                           'of its own' % (label, j['label']))
            break
    return [0.0, 0.0, 1.0], ('DEFAULT (EXO-218): no link arrives at %s and the arm leaves '
                             'it along its own axis, so its +Y seeds on world +Z' % label)


def seed_frames(spec, joints, by_child, shift, rep=None, signs=None, slides=None):
    """Every module frame one arm needs, as a seed for the spin solve.

    A `fixed` joint keeps the (axis_sign, y_hint) its spec carries - on G1 that is J1..J5,
    the poses EXO-103 and EXO-114 confirmed against the exo STEP. A `derived` joint has no
    counterpart to confirm against: its axis_sign comes from `axis_sign_rule` where
    the pair decides it and is SOLVED where it does not, and its +Y seed is the previous
    module's axis reversed - or, for a joint with no previous module at all (Luna's R1),
    `first_hint`. `signs` overrides the free axis signs; `slides` is the EXO-170 position of
    each module along its own axis, 0 everywhere by default.
    Returns (frames, labels, table)."""
    signs = {} if signs is None else signs
    slides = {} if slides is None else slides
    frames, labels, table = {}, [], {}
    for j in spec['fixed']:
        R, t = g.module_frame(j['name'], joints, by_child, j['axis_sign'], j['y_hint'],
                                   slide=slides.get(j['label'], 0.0))
        frames[j['label']] = (R, [t[i] - shift[i] for i in range(3)])
        labels.append((j['label'], j['name'], j['axis_sign']))
        table[j['label']] = {'axis_sign': j['axis_sign'], 'solved': False,
                             'why': 'the pose measured in the exo STEP (EXO-103 / EXO-114)'}
    prev = spec['fixed'][-1]['name'] if spec['fixed'] else None
    for j in spec['derived']:
        t_j, axis = g.joint_world_pose(j['name'], joints, by_child)
        if prev is None:
            sign, why = None, ('the first joint of the chain: no previous joint, so no link '
                               'arrives and the step decides nothing - the sign is free and '
                               'is solved by the links (EXO-160)')
            hint, hint_why = first_hint(spec, j['label'], j['name'], joints, by_child)
        else:
            t_prev, _a = g.joint_world_pose(prev, joints, by_child)
            sign, why = axis_sign_rule(axis, t_prev, t_j)
            z_prev = [frames[labels[-1][0]][0][i][2] for i in range(3)]
            hint = [-v for v in z_prev]
            hint_why = 'the previous module\'s axis, reversed (EXO-160)'
            if abs(g.dot(g.unit(hint), axis)) > 0.9:
                hint = [0.0, 0.0, 1.0]
                hint_why = ('the previous module\'s axis is parallel to this one, so the '
                            'seed falls back to world +Z')
        table[j['label']] = {'axis_sign': sign, 'solved': sign is None, 'why': why,
                             'y_hint': hint, 'y_hint_why': hint_why, 'name': j['name']}
        use = signs.get(j['label'], sign or 1)
        R, t = g.module_frame(j['name'], joints, by_child, use, hint,
                                   slide=slides.get(j['label'], 0.0))
        frames[j['label']] = (R, [t[i] - shift[i] for i in range(3)])
        labels.append((j['label'], j['name'], sign))
        prev = j['name']
    return frames, labels, table


def joint_stops(spec, joints, links, signs):
    """`solve_spins`' `stops`: the link stop every LIMITED joint's own URDF limits
    give it, keyed by the joint that carries the rotor pad. `signs[label]` is that module's
    axis sign, the only other thing the rule reads."""
    by_label = dict((j['label'], j) for j in spec_joints(spec))
    out = {}
    for name, a, b, pad_r, width in links:
        if a not in spec['limited'] or a not in by_label:
            continue
        jd = by_label[a]
        lj = g.limited_joint(dict(joints[jd['name']], name=jd['name']), signs[a],
                                  pad_r)
        out[a] = {'stop_angle': lj['stop_angle'], 'stop_half': lj['stop_half'],
                  'stop_h': lj['stop_h'],
                  'stop_why': ('EXO-204/EXO-208: sized from %s\'s URDF limits %+.4f .. '
                               '%+.4f rad (range %.4f rad = %.3f deg) beside the module\'s '
                               '%.3f deg joint stop, axis_sign %+d; built as ONE block of '
                               'arc %.5f deg centred %.5f deg (EXO-218)%s'
                               % (jd['name'], lj['lower'], lj['upper'], lj['range'],
                                  math.degrees(lj['range']),
                                  math.degrees(lj['joint_stop_arc']), lj['axis_sign'],
                                  math.degrees(lj['arc']), lj['stop_angle'],
                                  '' if not lj['capped'] else ' - RANGE CAPPED, see LIMITS'))}
    return out


def axis_signs_of(spec, combo, table):
    """Every module's axis sign at one combination of the free ones."""
    out = dict((j['label'], j['axis_sign']) for j in spec['fixed'])
    for j in spec['derived']:
        lab = j['label']
        out[lab] = combo.get(lab, table[lab]['axis_sign'] or 1)
    return out


def solve_axis_signs(spec, joints, by_child, shift, links, table, rep, order,
                     only=None, scores=None):
    """Solve every axis sign `axis_sign_rule` left free AND the EXO-170 slides, together.

    Free signs are enumerated (2^n, n = 2 on G1). At EACH combination the slides are solved
    first - `solve_slides` - because which way a module faces decides which way its
    body hangs off the joint and therefore what a slide has to clear. Then the whole chain
    is spun by the EXO-150 rule and every link is sized. The rule: keep the combinations
    whose links all route; among those take the shortest total centre line; then the least
    total slide; then the lowest combination.

    EXO-214: the slide solve is no longer body-only. It is handed `route_fits` -
    the whole-pair test, which re-seeds the chain at a trial slide, spins it and sizes every
    link with that joint's own link stop - so a slide is the minimum that makes the pair FIT
    and not merely the minimum that opens the two bodies.

    EXO-218: `spec['solved']` short-circuits the ENUMERATION, not the rule. The solve is
    pure Python and deterministic, and on Gento Luna it is 2^5 combinations each of which
    slides four candidates against the router - tens of minutes per combination, hours in
    total, which is not something to spend inside a Fusion session holding a document. So
    the answer may be computed ahead of time by these same functions and handed back here,
    where it is VERIFIED rather than re-derived: every pair must fit at it, every module
    pair must clear `MIN_CLEARANCE`, and every non-zero slide must be MINIMAL - a hair less
    and its own pair stops fitting. A verification that fails raises, by name.

    Returns (frames, combo, slides, slide_rows)."""
    if spec.get('solved'):
        return verify_solved(spec, joints, by_child, shift, links, table, rep, order)
    free = [lab for lab, f in table.items() if f['solved']]
    free.sort()
    best = None
    # `only` / `scores` (EXO-218) are how the enumeration is spread over processes: `only`
    # restricts it to a subset of the 2^n combinations and `scores` collects
    # (key, combo, slides) for every one tried, so the runs can be merged and the SAME rule
    # applied to the whole set. Neither changes what any one combination computes.
    for k in (range(2 ** len(free)) if only is None else only):
        combo = dict((lab, 1 if (k >> i) & 1 == 0 else -1) for i, lab in enumerate(free))
        frames, _labels, _tbl = seed_frames(spec, joints, by_child, shift, signs=combo)
        stops = joint_stops(spec, joints, links, axis_signs_of(spec, combo, table))

        def fits(slides, _combo=combo, _stops=stops):
            trial, _l, _t = seed_frames(spec, joints, by_child, shift, signs=_combo,
                                        slides=slides)
            return route_fits(trial, links, _stops)

        slides, srows = solve_slides(frames, order, fits=fits)
        frames, _labels, _tbl = seed_frames(spec, joints, by_child, shift, signs=combo,
                                            slides=slides)
        ok, total, why = 0, 0.0, []
        spun = dict(frames)
        for name, a, b, pad_r, width in links:
            RA, tA = spun[a]
            RB, tB = spun[b]
            R_ab, t_ab = g.relative_frame(RA, tA, RB, tB)
            # EXO-186: the tongue is solved once and the spin is SNAPPED once to the
            # nearest quarter turn, so the chain is walked with `spin_check`, not with
            # `encoder_spin` alone. A pair whose ROUTE refuses still turns its module by
            # the square rule and the walk goes on.
            try:
                route, fit = spin_check(R_ab, t_ab, pad_r, width)
                spun[b] = (g.frame_spun(RB, fit['spin']), tB)
                ok += 1
                total += path_length(route)
            except ValueError as e:
                why.append('%s: %s' % (name, e))
                try:
                    spin, _info = encoder_spin(R_ab, t_ab)
                except ValueError:
                    break
                spun[b] = (g.frame_spun(RB, spin), tB)
        key = (-ok, round(total, 6), round(sum(abs(v) for v in slides.values()), 6), k)
        if scores is not None:
            scores.append({'k': k, 'key': list(key), 'combo': dict(combo),
                           'slides': dict(slides), 'ok': ok, 'total': total,
                           'why': list(why)})
        if best is None or key < best[0]:
            best = (key, combo, frames, why, slides, srows)
    if best is None:
        raise ValueError('no axis-sign combination was tried')
    key, combo, frames, why, slides, srows = best
    rep('Module axis direction, the one thing the URDF does not fix (EXO-158):')
    for lab in sorted(table):
        f = table[lab]
        if f['solved']:
            rep('  %-3s %+d - SOLVED from the link: %s. %d of %d links route at this '
                'combination, total centre line %.3f mm%s'
                % (lab, combo[lab], f['why'], -key[0], len(links), key[1],
                   '' if not why else '; refused: %s' % why[0].split(':')[0]))
        else:
            rep('  %-3s %+d - %s' % (lab, f['axis_sign'], f['why']))
    return frames, combo, slides, srows


# ---------------- default rounding (EXO-151) ----------------
# EXO-144's builder leaves every elbow sharp. These are DEFAULTS, stated as inputs with a
# default, not measured off any Link N R - although the hand-made and the old generated links
# happen to draw the same two numbers (`armgen.L_FILLET_IN` 3.0, `armgen.L_ROUND_OUT` 10.0).
ELBOW_R_IN = 3.0                # concave: the ribbon meets a plate on the inside of an elbow
ELBOW_R_OUT = 10.0              # convex: the outer corner of that elbow


def verify_solved(spec, joints, by_child, shift, links, table, rep, order):
    """Check a precomputed `spec['solved']` = {'signs': .., 'slides': .., 'why': ..} and
    report every number it was checked on (EXO-218). Same signature and same return as
    `solve_axis_signs`; raises ValueError, by name, on anything that does not hold."""
    pre = spec['solved']
    free = sorted(lab for lab, f in table.items() if f['solved'])
    combo = dict((lab, pre['signs'][lab]) for lab in free)
    slides = dict((lab, float(pre['slides'].get(lab, 0.0))) for lab in order)
    stops = joint_stops(spec, joints, links, axis_signs_of(spec, combo, table))

    def fits(sl):
        trial, _l, _t = seed_frames(spec, joints, by_child, shift, signs=combo, slides=sl)
        return route_fits(trial, links, stops)

    frames, _labels, _tbl = seed_frames(spec, joints, by_child, shift, signs=combo,
                                        slides=slides)
    rep('')
    rep('Axis signs and slides: PRECOMPUTED and VERIFIED here, not enumerated (EXO-218). '
        'The solve is `sweepgen.solve_axis_signs` - pure Python, deterministic, no Fusion - '
        'and on this arm it is 2^%d combinations x four slide candidates each, hours of '
        'router calls. It was run by that function outside Fusion; this build checks the '
        'answer. %s' % (len(free), pre.get('why', '')))
    ok, why, pair, per = fits(slides)
    for (a, b), (good, w) in sorted(per.items(), key=lambda kv: order.index(kv[0][0])):
        rep('  VERIFY %s <-> %s: %s' % (a, b, 'fits' if good else 'DOES NOT FIT - %s' % w))
    gaps = module_pairs(frames, order, {})
    rep('  VERIFY closest module pair %s <-> %s at %.3f mm (need MIN_CLEARANCE %.1f)'
        % (gaps[0][0], gaps[0][1], gaps[0][2], g.MIN_CLEARANCE))
    if gaps[0][2] < g.MIN_CLEARANCE - 1e-9:
        raise ValueError('the precomputed slides leave %s <-> %s at %.3f mm, under '
                         'MIN_CLEARANCE %.1f' % (gaps[0][0], gaps[0][1], gaps[0][2],
                                                 g.MIN_CLEARANCE))
    if not ok:
        raise ValueError('the precomputed axis signs / slides do not fit: %s' % why)
    # Tightness, REPORTED and not raised on. `solve_slides` takes one slide per ROUND and
    # each is the minimum at the round it was taken; it is not a property of the FINAL set.
    # A later round that moves a module's neighbour can leave an earlier slide with room to
    # spare - Luna's R1 is solved before R2 slides +29.501 mm, and afterwards a hair less of
    # R1 still fits. That is the rule working, not a broken answer, so the line says which
    # slides are still tight and which a later round loosened. The two things that ARE
    # invariants of the answer - every pair fits, no module pair under MIN_CLEARANCE - raise.
    for lab in order:
        v = slides[lab]
        if abs(v) < 1e-9:
            continue
        less = dict(slides)
        less[lab] = v - (SLIDE_TOL * 4.0) * (1.0 if v > 0 else -1.0)
        r = fits(less)
        rep('  VERIFY %s slide %+.4f mm: at %+.4f mm the chain %s'
            % (lab, v, less[lab],
               'STILL FITS - tight at its own round, loosened by a later one'
               if r[0] else 'stops fitting (%s) - still TIGHT' % (r[1].split(':')[0])))
    # no candidate rows: the four candidates per pair were sized by `solve_slides` in the
    # run that produced `spec['solved']`, not here. What this build can say about them is
    # the VERIFY lines above - every pair fits and every slide is minimal - so the slide
    # table is left to report the slides themselves.
    rows = []
    rep('Module axis direction, the one thing the URDF does not fix (EXO-158):')
    for lab in sorted(table):
        f = table[lab]
        if f['solved']:
            rep('  %-3s %+d - SOLVED from the link (verified here): %s'
                % (lab, combo[lab], f['why']))
        else:
            rep('  %-3s %+d - %s' % (lab, f['axis_sign'], f['why']))
    return frames, combo, slides, rows


def elbow_radii(route, r_in=None, r_out=None):
    """The two default rounds, and what each recorded guard leaves of them.

    Guard EXO-79 is the one that bites: the landing plate's FAR face is the face that bolts to
    module B, so it has to stay flat where it crosses the r `STATOR_PAD_R` rim, and a convex
    round on it may start no closer than `RIM_MARGIN` past that rim. The elbow's outer round
    starts `r_out` back from the tongue end, so `r_out <= y_end - (STATOR_PAD_R + RIM_MARGIN)`.

    Guard EXO-124/128 caps the inner round: it grows the ribbon's footprint on the plate's
    near face by `r_in`, and every insert there still has to keep `HOLE_FLAT_RING`.

    Returns both radii, both caps, and why each was reduced. A reduction is a number, never a
    silent clamp."""
    ri = ELBOW_R_IN if r_in is None else r_in
    ro = ELBOW_R_OUT if r_out is None else r_out
    # EXO-177: the round runs along the tongue's END, which is not one plane square to
    # the tongue any more; `y_round` is the nearest that end gets to the plate axis.
    cap_rim = route.get('y_round', route['y_end']) - (g.STATOR_PAD_R + RIM_MARGIN)
    num = route['numbers']
    R_s = route.get('R_t') or route['R_b']
    inserts = g.insert_slab_points(num, [R_s[i][0] for i in range(3)],
                                   [R_s[i][1] for i in range(3)],
                                   [R_s[i][2] for i in range(3)])
    d_rib = min(g._rect_point_dist(i['slab'], *route['foot_box']) for i in inserts)
    cap_seat = d_rib - g.HOLE_R - g.HOLE_FLAT_RING
    out = {'r_in_asked': ri, 'r_out_asked': ro,
           'r_in': min(ri, cap_seat), 'r_out': min(ro, cap_rim),
           'cap_rim': cap_rim, 'cap_seat': cap_seat}
    out['why_in'] = ('' if out['r_in'] >= ri - 1e-9 else
                     'reduced to %.3f by EXO-124/128: the nearest insert is %.3f mm from the '
                     'ribbon footprint and needs %.2f mm of flat ring'
                     % (out['r_in'], d_rib, g.HOLE_FLAT_RING))
    out['why_out'] = ('' if out['r_out'] >= ro - 1e-9 else
                      'reduced to %.3f by EXO-79: the tongue ends %.3f mm out and the plate\'s '
                      'far face must stay flat to r %.1f + %.1f'
                      % (out['r_out'], route['y_end'], g.STATOR_PAD_R, RIM_MARGIN))
    return out


# ---------------- report ----------------

def describe(route):
    n = route
    out = ['ribbon %.0f x %.1f mm, %d path segments' % (n['width'], n['thickness'],
                                                        len(n['path']))]
    if n['plan_alpha_deg']:
        out.append('plan turn %.2f deg, r %.1f, s %.2f -> %.2f (%+.2f mm of the pair\'s '
                   '%+.2f mm lateral offset taken here)'
                   % (n['plan_alpha_deg'], n['plan_r'], n['plan_s_in'], n['plan_s_out'],
                      n['w_route'], n['w_off']))
    else:
        out.append('no plan turn (lateral offset %.3f mm)' % n['w_off'])
    if abs(n['drop']) > 1e-9:
        out.append('level change %+.3f mm taken on %s (EXO-158; it needs %.3f mm of straight '
                   'run at r %.1f)' % (n['drop'], n['level_taken_by'], n['level_need'],
                                       max(BEND_R, n['thickness'] / 2.0 + WALL_R_MIN)))
        for o, L, e in n['level_tried']:
            out.append('  %-7s leg: %s' % (o, 'route %.3f mm long' % L if L else e))
    if n['lat_advance']:
        out.append('lateral S %+.2f mm at %.2f deg, r %.1f, riser %.2f mm, l %.2f -> %.2f'
                   % (n['lat_drop'], n['lat_theta_deg'], n['lat_r'], n['lat_mid'],
                      n['lat_l'][0], n['lat_l'][1]))
    if n['side_theta_deg']:
        out.append('side S %+.2f mm at %.2f deg, r %.1f, riser %.2f mm, s %.2f -> %.2f'
                   % (n['side_drop'], n['side_theta_deg'], n['side_r'], n['side_mid'],
                      n['side_s'][0], n['side_s'][1]))
    elif abs(n['drop']) < 1e-9:
        out.append('no side S: the landing is level with the pad plane')
    out.append('landing: the %s primitive - %s' % (n['landing'], n['tongue_why']))
    for ln, o, L, e in n.get('landing_tried', []):
        out.append('  %-6s landing, %-7s leg: %s'
                   % (ln, o, 'route %.3f mm long' % L if L else e))
    out.append('tongue %+.3f deg off the pad axis, %.2f mm wide, reaching %.2f mm from the '
               'plate axis (the ribbon lands %.2f mm out), plate blends r %s'
               % (n['tongue_deg'], 2.0 * n['tongue_half_w'], n['y_end'], n['r_land'],
                  '/'.join('%.1f' % r for r in n['blend_r'])))
    out.append('landing y %.2f (tongue to %.2f), cutout web %.3f mm along the tongue (the '
               'true planar gap is %.3f mm), buried %.1f mm'
               % (n['y_land'], n['y_end'], n['cut_web'], n['cut_gap'], n['bury']))
    out.append('landing plate disk r %.4f (EXO-186: grown from r %.1f until the opening has '
               '%.1f mm of plate all round it wherever the snapped spin puts it; %s)'
               % (n['land_r'], g.STATOR_PAD_R, LAND_WEB,
                  'it already had it' if len(n['land_r_walk']) == 1 else
                  ' -> '.join('r %.3f gives %.3f' % w for w in n['land_r_walk'])))
    out.append('actuator (EXO-187/EXO-193): the opening is the XL330 grown by %.1f mm - its '
               'own footprint - cut through the whole exclusion volume; it takes %.0f mm3 '
               'off the plate (%s); nearest BODY is the %s, %.3f mm from the XL330 (need '
               '%.1f), and the EXO-183 collar - a blend tangent to the plate - stands '
               '%.3f mm off it; cable exit along +Y below the plate: %s'
               % (g.ACT_OPEN_MARGIN, n.get('act_cut_mm3') or 0.0,
                  'all inside the plate outline, thinnest plate round it %.3f mm at module-B '
                  '(x %.2f, y %.2f)%s' % (n['act_web_min'], n['act_web_at'][0],
                                          n['act_web_at'][1],
                                          '' if n['act_web_min'] >= LAND_WEB - 1e-9
                                          else ' - under LAND_WEB %.1f, EXO-186' % LAND_WEB)
                  if n.get('act_opening_in_plate')
                  else 'BREAKS OUT of the plate outline',
                  n['act_gap_part'], n['act_gap'], g.MIN_CLEARANCE,
                  -1.0 if n['act_collar_gap'] is None else n['act_collar_gap'],
                  'nothing there' if n['cable_exit'] is None
                  else '%.3f mm to the %s' % (n['cable_exit'], n['cable_exit_part'])))
    rings = n['rings']
    out.append('smallest flat ring: rotor %.3f mm, stator %.3f mm (need %.1f)'
               % (min(h['ring'] for h in rings if h['pad'] == 'rotor'),
                  min(h['ring'] for h in rings if h['pad'] == 'stator'),
                  g.HOLE_FLAT_RING))
    if n['clearance_a'] is not None:
        out.append('nearest approach to axis A below z %.1f: %.2f mm (r %.1f + %.1f needed)'
                   % (CLEAR_Z, n['clearance_a'], CLEAR_R, g.MIN_CLEARANCE))
    return out
