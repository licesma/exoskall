"""Where each link lands on its stator pad, and what the three working links share (EXO-136).

Pure Python, no Fusion: run it directly.

    /usr/bin/python3 tools/bridge_scripts/landing_side_report.py

EXO-130 treated link_45's landing as a one-off and turned the whole J5 module +45 deg to
move an insert out from under it. EXO-136 asks the prior question: links 12, 23 and 34 all
land cleanly and were never compared with each other, so whatever convention they share is
the rule the fourth should have followed.

Everything here is measured at the SEED frames - every module at the pose measured in the
exo STEP, spin 0, nothing solved and nothing applied.

## Part 1 - the side each link arrives on

`phi_land` is the direction the slab's tongue leaves the disk centre, as an angle in the
STATOR MODULE's own frame: 0 deg is module +X (the insert at 0 deg), 90 deg is module +Y,
the connector side.

'The actuator side' is defined two ways, both reported, because the obvious one is empty:
the actuator and the pocket cut for it are the rectangle CUTOUT_X x CUTOUT_Y, which CONTAINS
the module axis, so its angular span about the centre is the whole turn.

  * `ACTUATOR_ARC` - the arc of the INSERT CIRCLE (r INSERT_RADIUS) that lies inside the
    cutout. That is where the cutout meets the ring the landing has to keep clear of, and it
    is the sector quoted as 'the actuator side'.
  * `cutout_reach(tongue)` - how far the cutout reaches along the tongue direction, between
    9.500 mm (straight away from the actuator) and 26.462 (straight at it). One number per
    link, and the sharper discriminator.

## Part 2 - why each landing is clear of the inserts

The run blocks the strip `y >= y_end - depth - r` across its band on each face of the slab
(`land_radius_fit`), so the number that decides everything is how far the landing line
`y_end` sits beyond the insert circle. Reported with the disk rim, because a landing past
the rim is on the TONGUE and cannot be near an insert at all.

## Part 3 - link_45's two rims

The ramp may land on either rim of the stator disk. Which rims exist, where they are in the
rotor-pad frame, and what each costs.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'fusion_scripts', 'BuildArm'))
sys.path.insert(0, HERE)
import armgen as g                                                    # noqa: E402
from geom_snapshot import g1_seed_frames, LINKS                       # noqa: E402

ARC_STEP = 0.01     # deg


def actuator_arc():
    """The arc of the insert circle that lies inside the actuator cutout, in the module
    frame, as (lo, hi, centre) in degrees."""
    hit = []
    for k in range(int(round(360.0 / ARC_STEP))):
        phi = k * ARC_STEP
        x = g.INSERT_RADIUS * math.cos(math.radians(phi))
        y = g.INSERT_RADIUS * math.sin(math.radians(phi))
        if g.CUTOUT_X[0] <= x <= g.CUTOUT_X[1] and g.CUTOUT_Y[0] <= y <= g.CUTOUT_Y[1]:
            hit.append(phi)
    return (min(hit), max(hit), (min(hit) + max(hit)) / 2.0) if hit else None


def cutout_peak():
    """The direction of the cutout's greatest reach from the module axis, and that reach."""
    best = None
    for k in range(int(round(360.0 / ARC_STEP))):
        phi = k * ARC_STEP
        c, s = math.cos(math.radians(phi)), math.sin(math.radians(phi))
        r = max(vx * c + vy * s for vx in g.CUTOUT_X for vy in g.CUTOUT_Y)
        if best is None or r > best[1]:
            best = (phi, r)
    return best


def slab_axes(spec, num):
    """The frame the slab is drawn in: +Y points from the disk centre out along the tongue.
    The bar and the shoulder L use the snapped frame's own +Y; the ramp turns it 180 deg
    (`_ramp_route`)."""
    y_sign = -num['y_sign'] if spec['route']['pattern'] == 'ramp' else num['y_sign']
    y_s = [0.0, 0.0, y_sign]
    z_s = num['z_s']
    return g.cross(y_s, z_s), y_s, z_s


def angle_in_module(d, num):
    """A direction given in the rotor-pad frame A, as an angle in the stator module's own
    XY frame."""
    return math.degrees(math.atan2(g.dot(num['y_b'], d), g.dot(num['x_b'], d))) % 360.0


def signed(a):
    a = a % 360.0
    return a - 360.0 if a > 180.0 else a


def measure():
    frames = g1_seed_frames()
    out = []
    for name, a, b, pad_r, width in LINKS:
        R_ab, t_ab = g.relative_frame(frames[a][0], frames[a][1], frames[b][0], frames[b][1])
        num = g.landing_numbers(R_ab, t_ab)
        spec = g.route_relative(R_ab, t_ab, pad_r, None, g.PATTERNS, width)
        geo, land = spec['geometry'], spec['route']['stator_land']
        x_s, y_s, z_s = slab_axes(spec, num)
        depth = g.route_runs(spec['route'])[-1]['thickness']
        half_w, x_off = width / 2.0, geo.get('x_off', 0.0)
        x_lo = x_off - half_w - g.TONGUE_ROUND_MARGIN
        x_hi = x_off + half_w + g.TONGUE_ROUND_MARGIN
        reach = g.HOLE_R + g.HOLE_FLAT_RING
        inserts = []
        for ins in g.insert_slab_points(num, x_s, y_s, z_s):
            xh, yh = ins['slab']
            inserts.append({
                'angle': ins['angle'], 'slab': (xh, yh),
                'in_band': (x_lo - reach) <= xh <= (x_hi + reach),
                'x_clear': max(x_lo - reach - xh, xh - x_hi - reach),
                'max_fillet': land['y_end'] - depth - yh - reach,
            })
        tight = min((i for i in inserts if i['in_band']), key=lambda i: i['max_fillet'])
        out.append({
            'link': name, 'joint': b, 'pattern': spec['route']['pattern'],
            'phi': angle_in_module(y_s, num), 'cut_reach': g.cutout_reach(num, y_s),
            'y_end': land['y_end'], 'depth': depth,
            'past_rim': land['y_end'] - g.STATOR_PAD_R,
            'under_past_rim': land['y_end'] - depth - g.STATOR_PAD_R,
            'past_insert_circle': land['y_end'] - g.INSERT_RADIUS,
            'fillet_r': land['fillet_r'], 'fillet_nom': land['fillet_r_nominal'],
            'ring': g.stator_ring_margin(spec),
            'inserts': inserts, 'tight': tight,
            'axial': num['axial'], 'y_sign': num['y_sign'], 'yb_axial': num['y_b'][2],
            'spin_off': math.degrees(num['spin_rad']),
        })
    return out


def main():
    arc, peak = actuator_arc(), cutout_peak()
    rows = measure()
    print('EXO-136 - the landing side of all four links, in the stator module frame')
    print('=' * 79)
    print('inserts at %s deg; no insert on +Y (90 deg), the connector side'
          % ', '.join('%.0f' % a for a in g.INSERT_ANGLES))
    print('actuator cutout x %.1f..%.1f, y %.1f..%.1f (module frame), disk rim r %.1f, '
          'insert circle r %.2f' % (g.CUTOUT_X + g.CUTOUT_Y + (g.STATOR_PAD_R,
                                                              g.INSERT_RADIUS)))
    print('ACTUATOR_ARC  %.2f .. %.2f deg (%.2f wide, centred %.2f) - where the cutout '
          'crosses the insert circle' % (arc[0], arc[1], arc[1] - arc[0], arc[2]))
    print('cutout reach  %.3f mm straight away from the actuator .. %.3f mm at %.2f deg'
          % (-g.CUTOUT_Y[0], peak[1], peak[0]))
    print()
    print('PART 1 - which side each link arrives on')
    print('%-9s %-5s %9s %10s %9s %9s' %
          ('link', 'route', 'phi_land', 'in arc?', 'vs actu.', 'cut reach'))
    for r in rows:
        print('%-9s %-5s %9.3f %10s %+9.3f %9.3f'
              % (r['link'], r['pattern'], r['phi'],
                 'YES' if arc[0] <= r['phi'] <= arc[1] else 'no',
                 signed(r['phi'] - arc[2]), r['cut_reach']))
    print()
    print('PART 2 - how far out each link meets the slab')
    print('%-9s %8s %7s %9s %10s %11s %9s %8s' %
          ('link', 'y_end', 'run t', 'past rim', 'under rim', 'past inserts', 'fillet r',
           'nominal'))
    for r in rows:
        print('%-9s %8.3f %7.3f %+9.3f %+10.3f %+11.3f %9.3f %8.3f'
              % (r['link'], r['y_end'], r['depth'], r['past_rim'], r['under_past_rim'],
                 r['past_insert_circle'], r['fillet_r'], r['fillet_nom']))
    print()
    print('y_end        the landing line: where the run meets the slab face, from the disk')
    print('             centre out along the tongue (slab frame)')
    print('past rim     y_end - %.1f: a landing past the rim is on the TONGUE' % g.STATOR_PAD_R)
    print('under rim    the run\'s far boundary, y_end - its thickness, past the rim.')
    print('             NEGATIVE means the run comes over the disk face itself')
    print('past inserts y_end - %.2f' % g.INSERT_RADIUS)
    print()
    print('the tightest insert under each landing (slab frame, the run\'s own band):')
    for r in rows:
        t = r['tight']
        print('  %-9s insert %3.0f deg at (%8.3f,%8.3f): largest land fillet %8.3f '
              '(nominal %.3f)' % (r['link'], t['angle'], t['slab'][0], t['slab'][1],
                                  t['max_fillet'], r['fillet_nom']))
        for i in r['inserts']:
            if not i['in_band']:
                print('  %-9s insert %3.0f deg at (%8.3f,%8.3f): outside the run\'s band by '
                      '%.3f mm in x' % ('', i['angle'], i['slab'][0], i['slab'][1],
                                        i['x_clear']))
    print()
    print('PART 3 - link_45\'s two rims (the ramp may land on either)')
    r = rows[3]
    print('the J5 disk stands edge-on in the J4 rotor-pad plane: centre z = %.4f, inside the'
          % r['axial'])
    print('plate\'s own thickness (z %.1f..%.1f). Its connector side points along Z_A %+.6f,'
          % (0.0, g.PAD_T, r['yb_axial']))
    print('i.e. %s.' % ('DOWN, away from the plate'
                        if r['yb_axial'] < 0 else 'UP, toward the plate'))
    for label, sign in (('away from the connector (built)', -1.0), ('the ACTUATOR side', +1.0)):
        # `_ramp_route`: z_run = axial - STATOR_PAD_R * y_sign at y_land = -STATOR_PAD_R,
        # i.e. z_run = axial + y_land * y_sign for either rim
        y_land = sign * g.STATOR_PAD_R
        z_run = r['axial'] + y_land * r['y_sign']
        rise = z_run - g.PAD_T / 2.0
        print('  rim %-32s Y_B %+6.1f -> run centre line at z %+9.4f, climb %+9.4f %s'
              % (label, y_land, z_run, rise,
                 '' if rise > g.PAD_T else '<- BELOW the pad: unreachable'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
