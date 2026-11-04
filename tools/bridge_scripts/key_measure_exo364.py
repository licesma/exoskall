"""EXO-364 shared helper (exec'd by dryrun/make scripts): measure the rotor key sockets and
the filled notch on an upper rotor body `b` (module frame), via `S(x, y, z)` containment."""
import math
def key_report(S, say):
    def bis(f, a, b_, n=40):
        for _ in range(n):
            m = (a + b_) / 2
            a, b_ = (m, b_) if f(m) else (a, m)
        return (a + b_) / 2
    out = {}
    for ang in (45.0, 135.0, 225.0):
        cx, cy = 15.0 * math.cos(math.radians(ang)), 15.0 * math.sin(math.radians(ang))
        rs = []
        for k in range(8):
            t = math.radians(22.5 + 45 * k)
            rs.append(bis(lambda r: not S(cx + r * math.cos(t), cy + r * math.sin(t), -0.6), 0.0, 3.0))
        floor = bis(lambda z: not S(cx, cy, z), -0.01, -3.0)
        out[ang] = (2 * sum(rs) / len(rs), min(rs), max(rs), floor)
        say('socket %3.0f deg: D %.4f (r %.4f..%.4f) floor z %.4f -> depth %.4f' % (ang, 2 * sum(rs) / len(rs), min(rs), max(rs), floor, -floor))
    c = 15.0 * math.cos(math.radians(315)), 15.0 * math.sin(math.radians(315))
    say('315 deg (empty): solid at z -0.1/-0.6/-1.25: %s' % [S(c[0], c[1], z) for z in (-0.1, -0.6, -1.25)])
    rim = [bis(lambda r: S(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), -0.5), 14.0, 20.0) for a in (45, 80, 90, 100, 135)]
    say('rim r at 45/80/90/100/135 deg z -0.5: %s' % ['%.4f' % r for r in rim])
    say('old notch region x -3..3 step .5, y 14.5..17.9, z -0.05/-1.45 all solid: %s / %s' % (
        all(S(-3 + 0.5 * i, 14.5 + 0.2 * j, -0.05) for i in range(13) for j in range(18) if math.hypot(-3 + 0.5 * i, 14.5 + 0.2 * j) < 17.95),
        all(S(-3 + 0.5 * i, 14.5 + 0.2 * j, -1.45) for i in range(13) for j in range(18) if math.hypot(-3 + 0.5 * i, 14.5 + 0.2 * j) < 17.95)))
    say('nothing above z 0 at +Y: %s' % (not any(S(x, y, 0.02) for x in (-2, 0, 2) for y in (15, 16.5, 17.8))))
    # walls on the solid: along the line socket centre -> feature centre
    def wall(ang, fr, fa, f_rad):
        sx, sy = 15.0 * math.cos(math.radians(ang)), 15.0 * math.sin(math.radians(ang))
        fx, fy = fr * math.cos(math.radians(fa)), fr * math.sin(math.radians(fa))
        L = math.hypot(fx - sx, fy - sy); ux, uy = (fx - sx) / L, (fy - sy) / L
        z = -0.6
        e1 = bis(lambda t: not S(sx + ux * t, sy + uy * t, z), 0.0, 2.5)
        e2 = bis(lambda t: S(sx + ux * t, sy + uy * t, z), e1 + 0.01, L)
        return e2 - e1
    say('wall socket 45 -> M3 hole r 11 @ 0: %.4f ; @ 90: %.4f' % (wall(45, 11, 0, 1.6), wall(45, 11, 90, 1.6)))
    say('wall socket 135 -> O5 head column r 6 @ 135: %.4f' % wall(135, 6.001, 135, 2.5))
    sx, sy = 15 * math.cos(math.radians(225)), 15 * math.sin(math.radians(225))
    e = bis(lambda t: not S(sx + t * math.cos(math.radians(225)), sy + t * math.sin(math.radians(225)), -0.6), 0, 2.5)
    o = bis(lambda t: S(sx + t * math.cos(math.radians(225)), sy + t * math.sin(math.radians(225)), -0.6), e + 0.01, 5)
    say('wall socket 225 -> rim (radial): %.4f' % (o - e))
    return out
