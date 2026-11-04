"""Pure-Python part of BuildArm: URDF FK, module frames, link geometry numbers and the
bar link's side profile / flange outline as data (lines + tangent arcs).

No Fusion imports, so it can be unit-tested with plain python3
(see tools/test_armgen.py). Lengths in this module are METERS for URDF
quantities and MILLIMETERS for pad/link geometry (as measured from the STEP
exports); the Fusion side converts to centimeters.

Frame conventions (see .knuth/docs/exo-joint.md):
  module origin = centre of the rotor top face, module Z = rotation axis out of
  the rotor face, module +Y = the actuator's connector side (the stator side
  with no heat-set insert).
"""
import math
import xml.etree.ElementTree as ET


# ---------------- URDF parsing / forward kinematics ----------------

def _rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    # URDF rpy is fixed-axis XYZ: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def matvec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def transpose(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def unit(v):
    n = math.sqrt(dot(v, v))
    if n < 1e-12:
        raise ValueError('zero-length vector')
    return [x / n for x in v]


def parse_urdf(path):
    tree = ET.parse(path)
    root = tree.getroot()
    joints = {}
    by_child = {}
    for j in root.findall('joint'):
        name = j.get('name')
        o = j.find('origin')
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if o is not None:
            if o.get('xyz'):
                xyz = [float(v) for v in o.get('xyz').split()]
            if o.get('rpy'):
                rpy = [float(v) for v in o.get('rpy').split()]
        ax = j.find('axis')
        axis = [1.0, 0.0, 0.0]
        if ax is not None and ax.get('xyz'):
            axis = [float(v) for v in ax.get('xyz').split()]
        lim = j.find('limit')
        lower = upper = None
        if lim is not None and lim.get('lower') is not None:
            lower, upper = float(lim.get('lower')), float(lim.get('upper'))
        joints[name] = {
            'xyz': xyz, 'rpy': rpy, 'axis': axis,
            'parent': j.find('parent').get('link'),
            'child': j.find('child').get('link'),
            # EXO-204: the joint's travel, in radians, straight off <limit>. None for a
            # joint the URDF leaves unlimited (continuous / fixed) - `limited_joint` refuses
            # those by name rather than inventing a range.
            'lower': lower, 'upper': upper, 'name': name,
        }
        by_child[j.find('child').get('link')] = name
    return joints, by_child


def joint_world_pose(joint_name, joints, by_child):
    """World position (m) and unit world axis of a joint at the zero configuration."""
    chain = []
    jn = joint_name
    while True:
        j = joints[jn]
        chain.append(j)
        if j['parent'] in by_child:
            jn = by_child[j['parent']]
        else:
            break
    chain.reverse()
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    t = [0.0, 0.0, 0.0]
    for j in chain:
        t = [t[i] + sum(R[i][k] * j['xyz'][k] for k in range(3)) for i in range(3)]
        R = matmul(R, _rpy_to_R(*j['rpy']))
    axis_world = unit(matvec(R, chain[-1]['axis']))
    return t, axis_world


# ---------------- module frame ----------------

def module_frame(joint_name, joints, by_child, axis_sign=1, y_hint=(0, 0, 1), spin_deg=0.0,
                 slide=0.0):
    """Full module frame for a joint.

    Returns (R, t): R is 3x3 with COLUMNS = module X, Y, Z in world; t in meters.
      Z = axis_sign * URDF joint axis          (rotor-face normal)
      Y = y_hint projected perpendicular to Z, then turned `spin_deg` about Z
      X = Y x Z
      t = the URDF joint origin + `slide` mm along Z
    Raises ValueError if y_hint is (nearly) parallel to the axis.

    `slide` (EXO-170) is the module's position ALONG ITS OWN AXIS, in mm. A revolute joint
    fixes the module's axis LINE, not where the module sits on it, so sliding moves nothing
    kinematic: the axis line, and therefore every joint angle, is untouched. It is not an
    input either - `sweepgen.solve_slides` solves it from the module bodies alone, and it is
    0 at every joint whose neighbours already clear.

    `spin_deg` is the module's rotation about its OWN axis - the one thing the URDF does not
    fix. It is not a taste: it decides where the three stator inserts, the actuator cutout,
    the rotor key and the stop block sit relative to the link that lands on them, and
    it is SOLVED by `solve_module_spins` (EXO-130). `y_hint` is the seed the solve starts
    from - the pose measured in the exo STEP - and `spin_deg` is how far the solve had to
    move off it; 0 means the measured pose already cleared everything.
    """
    if axis_sign not in (1, -1):
        raise ValueError('axis_sign must be +1 or -1, got %r' % (axis_sign,))
    t, w = joint_world_pose(joint_name, joints, by_child)
    z = [axis_sign * v for v in w]
    yh = list(y_hint)
    yp = [yh[i] - dot(yh, z) * z[i] for i in range(3)]
    if math.sqrt(dot(yp, yp)) < 0.1 * math.sqrt(dot(yh, yh)):
        raise ValueError('y_hint %r is parallel to the axis of %s' % (y_hint, joint_name))
    y = unit(yp)
    x = cross(y, z)
    if spin_deg:
        c, s = math.cos(math.radians(spin_deg)), math.sin(math.radians(spin_deg))
        x, y = ([c * x[i] + s * y[i] for i in range(3)],
                [-s * x[i] + c * y[i] for i in range(3)])
    R = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
    if slide:
        t = [t[i] + slide * z[i] / 1000.0 for i in range(3)]
    return R, t


def relative_frame(RA, tA, RB, tB):
    """Frame B expressed in frame A. Returns (R_ab, t_ab_mm)."""
    R_ab = matmul(transpose(RA), RB)
    d = [(tB[i] - tA[i]) * 1000.0 for i in range(3)]
    t_ab = matvec(transpose(RA), d)
    return R_ab, t_ab


# ---------------- link geometry (mm), measured from the exo STEP ----------------
# Source: `Link 1 R` / `Link 1` inside gento_luna/exo/exports/exo_skeleton_2026-08-08.step
# and `sensor module 2.step`. See .knuth/docs/exo-link.md for the measurement table.

PAD_T = 8.0                 # both pads
ROTOR_PAD_R = 26.0          # rotor pad disk radius (Link 1 R); Link 2 uses 27
ROTOR_HOLE_RADIUS = 11.0    # 4x M3 clearance holes, at 0/90/180/270 deg
ROTOR_HOLE_ANGLES = (0.0, 90.0, 180.0, 270.0)
HOLE_R = 1.6                # M3 clearance (dia 3.2)
# Every screw hole needs a FLAT SEATING RING around it: a screw head (or the mating
# pad) must bear on plane, never on the start of a curve. `HOLE_FLAT_RING` is how far
# beyond the hole EDGE the body must stay planar, on every seating face of both pads.
# It is a route constraint, enforced in `route_path` - see `hole_flat_rings` (EXO-124).
HOLE_FLAT_RING = 2.0
STOP_R_IN, STOP_R_OUT = 20.0, 26.0   # link stop block: ring sector
# EXO-48 item 3 called these a guess, and they were: copied off `Link 1 R` and never
# derived. `limited_joint` (EXO-204) DERIVES both, from the joint's URDF limits and the
# measured joint stop, and it is what BuildArmGen uses for every joint in LIMITED_JOINTS.
# These two stay as the conservative default for the links nobody has limited yet.
STOP_HALF_ANGLE = 5.0       # deg; block is +/-5 deg wide, centred on module +X
STOP_ANGLE = 0.0            # deg, centre of the block, measured from module +X
STOP_H = 3.6                # protrudes 3.6 below the pad face (stator top is at -4.0)
# EXO-364: ROTOR KEY. The index notch / tab pair is gone. The link's rotor pad carries three
# O3.0 x 1.2 PEGS on its rotor-side face, into three O3.3 x 1.3 SOCKETS in the rotor top
# (`BuildArm.module_rotor_final`), at r 15.0 and module angles 45 / 135 / 225; 315 is left
# empty, so the 3-of-4 pattern fits one way only (like the stator's three inserts).
# 0.15 radial and 0.1 axial clearance. The pegs sit between the M3 holes (r 11, 0/90/180/270)
# and the link stop block (r >= STOP_R_IN 20): 7.514 to an M3 hole edge, 6.364 to an O5.5
# M3 head, 3.5 radially to the stop block's inner arc.
KEY_R = 15.0                       # peg / socket circle, module frame
KEY_ANGLES = (45.0, 135.0, 225.0)  # deg; 315 is the empty position
KEY_EMPTY_ANGLE = 315.0
KEY_PEG_R, KEY_PEG_H = 1.5, 1.2    # link: O3.0, 1.2 proud of the pad's rotor-side face
KEY_SOCKET_R, KEY_SOCKET_D = 1.65, 1.3   # rotor top: O3.3, 1.3 deep off the rotor face z 0


def key_centres():
    """(x, y) of the three key pegs / sockets in the module frame."""
    return [(KEY_R * math.cos(math.radians(a)), KEY_R * math.sin(math.radians(a)))
            for a in KEY_ANGLES]


def key_peg_points(step=0.25):
    """Points on the outlines of the three pegs (module frame, pad plane)."""
    n = max(8, int(2.0 * math.pi * KEY_PEG_R / step) + 1)
    return [(cx + KEY_PEG_R * math.cos(2.0 * math.pi * k / n),
             cy + KEY_PEG_R * math.sin(2.0 * math.pi * k / n))
            for cx, cy in key_centres() for k in range(n)]


def key_peg_dist(p):
    """Distance from a point (module frame) to the nearest key peg, negative inside."""
    return min(math.hypot(p[0] - cx, p[1] - cy) for cx, cy in key_centres()) - KEY_PEG_R

STATOR_PAD_R = 27.0
STATOR_UNDERSIDE_Z = -24.5  # module z of the stator's underside
INSERT_RADIUS = 19.25       # 3x M3 heat-set inserts
INSERT_ANGLES = (0.0, 180.0, 270.0)   # no insert on +Y (connector side)
CUTOUT_X = (-10.0, 10.0)    # actuator back + connectors pass through here
CUTOUT_Y = (-9.5, 24.5)
# The XL330's own z span in the module frame (.knuth/docs/exo-joint.md): it starts inside
# the stator and sticks 12 mm out of the stator underside at -24.5. `CUTOUT_X` x `CUTOUT_Y`
# x `ACT_Z` IS the actuator, not a pocket drawn for it - see `actuator_rect` (EXO-187).
ACT_Z = (-36.5, -10.5)
MODULE_R = 27.0             # stator outer radius (for the clearance check)
# EXO-341: ASSEMBLY MARK. One radial debossed line on each of the two faces that meet at
# the stator underside: the link's stator-pad face (built here, `BuildArmGen.build_link`)
# and the stator underside (cut on the export copy, `tools/bridge_scripts/
# export_link_stator_exo341.py`). Both at module +Y (90 deg = the connector side, the side
# with no insert), same width and radii, so with the stator sitting on the pad the two
# lines continue each other and are seen at the rim before the screws go in. At 90 deg the
# actuator opening reaches y 24.5, so the line starts INSIDE the opening (R_IN 23.5 removes
# nothing there) and runs out through the stator rim r 27 to R_OUT 27.6 (0.6 of it stays
# visible on the pad outside the stator). Rotor side: none, the three key pegs (EXO-364) key it.
ALIGN_MARK_ANGLE = 90.0     # deg, module frame
ALIGN_MARK_W = 0.6          # mm, tangential width
ALIGN_MARK_D = 0.4          # mm, depth into each face
ALIGN_MARK_R_IN, ALIGN_MARK_R_OUT = 23.5, 27.6

# The three PRINTED parts of the module, and the two names each of them answers to.
# `sensor module 2` - the cloud design, the STEP in the repo, the linked component - carries
# Fusion's defaults `Component1`, `Component2`, `Component3`, the names it was drawn with,
# and keeps them: it is a reference and is never written. EXO-242 RENAMES them in every v3
# copy the generator makes (`BuildArm.rename_v3_parts`), because a print queue full of
# `Component2` tells nobody which part is which.
#
# Both names are live at once - the original and the v3 copy sit in the same document - so
# anything that looks a part up by name has to accept either. Nothing in the generator does:
# `BuildArm.module_stator_occ` and `module_rotor_occs` read the SOLID, exactly so a
# `breakLink` copy may be renamed. The measurement and probe scripts that DO name a part go
# through `module_part_names` / `is_module_part`. The mapping lives in armgen because the
# pure-Python STEP scripts need it too, with no `adsk` anywhere.
MODULE_PART_RENAME = (('stator', 'Component1'),         # the part that does NOT turn
                      ('inner_top', 'Component2'),      # rotor face disk, z -9..0
                      ('inner_bottom', 'Component3'))   # rotor inner cylinder, z -13.2..-9
MODULE_PART_ORDER = tuple(new for new, _old in MODULE_PART_RENAME)


def module_part_names(which):
    """Both names the printed module part `which` can carry: the EXO-242 name on a v3 copy
    and Fusion's default on the original. `which` is one of `MODULE_PART_ORDER` - or either
    of its own names, so a caller can pass whatever it already has."""
    for new, old in MODULE_PART_RENAME:
        if which in (new, old):
            return (new, old)
    raise ValueError('not a printed module part: %r' % (which,))


def is_module_part(name, which):
    """True when `name` names the printed module part `which` under either of its two
    names. `name` may be a bare component name, a 'component/body' interference label, or
    an occurrence name with Fusion's ' (2)' suffix."""
    if name is None:
        return False
    head = str(name).split('/')[0].strip()
    if head.endswith(')') and ' (' in head:
        head = head[:head.rindex(' (')]
    return head in module_part_names(which)

MIN_CLEARANCE = 1.0         # mm between generated body and the next module
# How much the actuator OPENING is grown beyond the XL330's own footprint (EXO-193).
# ZERO. The hand-made links cut the opening to `CUTOUT_X` x `CUTOUT_Y` exactly and that was
# checked in the real world with the real actuator: it fits. EXO-187 grew it by
# MIN_CLEARANCE on the argument that the actuator is a module body like any other; Esteban's
# call is that the mating opening is not that kind of surface and the validated part wins.
# One constant, so the whole chain - `actuator_rect` / `actuator_volume` / `actuator_cut_z` /
# `cut_reach_clear`, sweepgen's `cut_gap` / `plate_opening` / ring bounds and BuildArmGen's
# cut feature - opens to exactly one number. The pure-Python `actuator_check` on NON-plate
# material (ribbon, collar, rotor pad, stop block, tab) is a different question and keeps
# MIN_CLEARANCE: nothing but the plate is allowed to mate with the actuator.
ACT_OPEN_MARGIN = 0.0
# The module as a SOLID OF REVOLUTION about its own axis: (z_lo, z_hi, radius) in the
# module frame, measured off the leaf bodies of `sensor module 2` in the exo STEP
# (EXO-166, tools/bridge_scripts/measure_wrist_profiles.py). It is a fact about the
# reused joint, like MODULE_R - not a number off any hand-built link. The difference
# from MODULE_R matters wherever two modules pass each other: EXO-163 modelled a module
# as an INFINITE r 27 cylinder and it is 35 mm long, which is the whole of EXO-170.
MODULE_PROFILE = ((-35.000, -24.501, 26.101),    # the XL330 body
                  (-24.501, -0.400, 27.001),     # the module shell (EXO-163's r 27)
                  (-0.400, 0.000, 18.000))       # the rotor face


def fmt_frame(R, t_m):
    return 'pos(mm)=(%.2f, %.2f, %.2f) X=(%.3f, %.3f, %.3f) Y=(%.3f, %.3f, %.3f) Z=(%.3f, %.3f, %.3f)' % (
        t_m[0] * 1000, t_m[1] * 1000, t_m[2] * 1000,
        R[0][0], R[1][0], R[2][0], R[0][1], R[1][1], R[2][1], R[0][2], R[1][2], R[2][2])


# ---------------- 'bar' body (representative pattern, measured from Link 2 R = link 23) ----------------
# In the rotor-pad frame A the bar is a SIDE PROFILE (lines + tangent arcs, drawn in the
# (s, z) plane: s along u = the stator axis projected into the pad plane, z along the pad
# axis) extruded to one width BAR_W. Its root is hidden inside the rotor pad; a riser
# drops outside the module-A radius to the bar level; the bar runs along u and ends
# buried in the stator slab, whose outline (disk + flange tongue with r 15 blends) is a
# second profile in the stator frame B. All radii / thicknesses are Link 2 R's
# (exo_skeleton_2026-08-08.step, tools/bridge_scripts/measure_link2_profile.py, EXO-58).
BAR_W = 25.0                # one width for riser, bend, bar and flange tongue (y +/-12.5)
BAR_T = 7.79                # bar thickness (Link 2 R bar z -33 .. -25.21)
RISER_T = 6.44              # riser thickness (inner face 28.44, outer face 34.88)
FILLET_OUT_BOT = 12.5       # fillet: riser outer face -> bar top (centre (47.38, -12.71))
FILLET_IN_BOT = 15.0        # round: riser inner face -> bar bottom, THE bend (centre (43.44, -18))
FILLET_FLANGE = 8.5         # fillet: bar top -> stator slab face (centre (62.2, -16.71))
FLANGE_BLEND_R = 15.0       # blends: flange tongue sides -> slab disk rim (centres (+/-27.5, 31.74) in B)
PROFILE_ROOT_S = 15.0       # the profile starts this far from axis A, inside the rotor pad
PROFILE_BURY = 0.05         # flange fillet ends this far inside the slab face (no tangent boolean)
PROFILE_BURY_UP = 1.0       # the buried bar end follows the slab face this far up before turning in
TONGUE_END_R = 7.9          # round on the tongue's end edge, stator side (Link 2 R centre (70.8, -25.1))
TONGUE_ROUND_MARGIN = 1.0   # the round's cut is this much wider than the tongue per side (no coplanar faces)
BAR_CLEAR = 6.85            # cutout edge -> bar face (Link 2: bar at Y_B 31.45..39.24, centre 35.35)
BAR_SQUARE_MIN = 0.9        # |Z_B in the pad plane| must exceed this: the stator axis lies in the plane
# EXO-130 removed `BAR_LAND_AXIAL_MIN` (|Y_B . Z_A| >= 0.99). It read the module's own
# rotation about its axis as if it were a squareness condition, and so froze the one free
# parameter this generator has. The bar and the ramp both draw the slab in the SNAPPED frame
# and cut the inserts and the cutout in the TRUE frame B; what they actually need is that
# the stator AXIS is square (`snap_axis_rad`), that the cutout is clear of the landing
# (`cutout_reach`) and that the inserts keep their flat rings (`land_radius_fit`) - three
# named geometric checks instead of one proxy.
# The slab (disk + tongue + blends) is built in a frame B' SNAPPED to the bar: Z' = the stator
# axis projected into the pad plane, Y' = the pad axis. Otherwise the URDF's rounding (G1: J3
# is 1.3e-4 rad off J2's X) puts the tongue walls 0.01 mm off the bar walls: two coplanar
# faces, sliver faces, a seam. The snap moves the slab by snap_rad * r at the rim; above
# this angle the frames are not square enough for the bar pattern and it raises.
BAR_SNAP_MAX = 1e-3         # rad (0.027 mm at r 27; the interference check tolerates 0.02 mm slivers)
# Riser inner face -> module A's stator / the previous link's stator slab (both r 27).
# Exo Link 2 R: riser inner face at 28.44 from the J2 axis, i.e. 1.44 mm clear (EXO-2/3).
# The bar level straddles the previous link's slab in z (exo too: bar -33..-25.2 vs slab
# -32.5..-24.5), so the riser/bar must clear the slab RADIALLY - hence this constant.
RISER_CLEAR = 1.44
# The stator's r 27 cylinder proper stops at MODULE_Z_TOP, but its STOP BLOCK rises to
# z = -0.4 over r 21.05..27 (angles 158-178 deg; measured in the module STEP, EXO-13 and
# tools/bridge_scripts/measure_j2_module_radii.py). The link turns past it, so ANY link
# material below this plane must keep RISER_CLEAR to r 27 - not just the material below
# the stator's top face. Link 2 R's r 5 join fillet did not: it sat at s 25.40 at z = -0.4
# and only crossed r 27 at z = -1.489, i.e. inside the stop block's own annulus (EXO-74).
MODULE_STOP_Z_TOP = -0.4

# ---------------- rotor-pad join: cover the rim (EXO-74) ----------------
# Link 2 R leaves its rotor pad through two fillets: an r 12.5 round that leaves the pad's
# OUTER face 4.62 mm inside the rim and crosses r 27 0.885 mm below that face, and an r 5
# fillet from the pad's rotor-side face to the riser inner face. Together they scoop a
# 25 mm wide pocket across the rim, which is what Esteban flagged. EXO-74 fills it:
#   * over the pad's own thickness the link keeps material outside the rim (it hugs r pad_r
#     across the whole BAR_W) and its outside surface stays the pad's outer-face plane,
#     level with it, as the J3 tongue is - no cove, one smooth strip;
#   * only past the pad's rotor-side face does the inside step out to RISER_CLEAR: that
#     face runs on out to the riser inner face as a RISER_CLEAR-wide ledge which faces the
#     joint, under the pad, and is invisible (and stays clear of the stop block, above);
#   * the outer face turns down onto the riser outer face through JOIN_ROUND_R, whose
#     tangent point must stay JOIN_RIM_MARGIN OUTSIDE the rim, so the outer face crosses
#     the rim flat. That caps the radius at RISER_T + RISER_CLEAR - JOIN_RIM_MARGIN = 6.88
#     for the exo numbers; JOIN_ROUND_R reuses the smallest radius already in the profile.
JOIN_ROUND_R = 5.0          # convex round: pad outer face -> riser outer face (was r 12.5)
JOIN_RIM_MARGIN = 1.0       # the round may start no closer than this to the pad rim
# EXO-83: the fill left the riser inner face meeting the ledge plane in a SHARP re-entrant
# corner at (s_in, 0) - the most loaded corner of the link, and the one place the hand-built
# exo link had a relief. One concave arc goes back in, tangent to both: centre
# (s_in - R, -R), tangent points (s_in, -R) on the riser inner face and (s_in - R, 0) on the
# ledge. It ADDS material into the quadrant s < s_in, z < 0 - exactly the space the stop
# block turns through - so the radius is capped twice over:
#   * flat ledge: the relief must not eat the whole ledge, i.e. R < RISER_CLEAR = 1.44, or
#     the ledge plane and the r 27 rim cylinder would meet tangentially in a zero-width
#     edge instead of crossing in a real planar strip (checked in bar_side_profile);
#   * stop block: the arc is at s = s_in - R + sqrt(R^2 - (R + MODULE_STOP_Z_TOP)^2) where
#     it crosses z = MODULE_STOP_Z_TOP, so keeping MIN_CLEARANCE to r 27 there caps it at
#     R <= 1.4333 for the exo numbers (checked by the guard in link_bar_geometry).
# R = 1.2 leaves 0.240 mm of flat ledge outside the rim, never comes inside r 27 below z = 0
# (its innermost point is s = 27.240 AT z = 0), clears the stop block by 1.134 mm (13% over
# MIN_CLEARANCE) and is cut by a stock 2.4 mm diameter tool. Nothing above z = 0 moves, so
# the rim stays covered and RISER_CLEAR is untouched. (EXO-79 proposed tangent to the rim,
# R = RISER_CLEAR = 1.44: that leaves no flat ledge AND measures 0.996 mm at the stop block,
# below MIN_CLEARANCE - link_bar_geometry raises on it.)
JOIN_RELIEF_R = 1.2         # concave relief: riser inner face -> ledge (was FILLET_IN_TOP 5)

# ---------------- riser S-curve (EXO-64) ----------------
# A URDF can put stator B sideways of the rotor axis: the G1 puts J3 6.24 mm off J2 along
# X_B (skew axes). EXO-58 absorbed that by shifting the WHOLE bar, so the bar left J2's
# rotor pad off-centre (walls at -18.74 / +6.26 instead of +/-12.5). Instead the bar now
# leaves the pad CENTRED and the offset is absorbed inside the riser as an S in PLAN view:
# the band runs straight at w = 0 out to s_start, turns through two equal tangent arcs of
# radius S_RADIUS (opposite senses), and is straight again at w = w_off from s_end on.
# The band keeps its full BAR_W width because the two walls are CONCENTRIC offsets of the
# centre arcs - wall radii S_RADIUS -/+ BAR_W/2 - so the width across the band is 25 mm
# everywhere. (Walls made from the same arc merely shifted in w would pinch the band to
# 25*cos(theta) = 20.5 mm in the middle.)
S_WALL_R_MIN = 5.0                       # tightest wall arc = JOIN_ROUND_R, the smallest
                                         # fillet already in the side profile
S_RADIUS = BAR_W / 2.0 + S_WALL_R_MIN    # 17.5 mm centre-line radius (>= FILLET_IN_BOT 15)
S_MIN_OFFSET = 0.05                      # below this the bar is centred anyway: no S
S_PAD_MARGIN = 1.0                       # the S must start this far outside the point where
                                         # the 25-wide band leaves the rotor pad rim, so the
                                         # walls at the pad are real planes at +/-BAR_W/2

# ---------------- where the S sits along the bar (EXO-69) ----------------
# The S itself is the same in both options - two tangent arcs of radius S_RADIUS whose walls
# are concentric offsets - only its s-range differs:
#   'riser' (EXO-64): it ends where the r 12.5 bend fillet ends, so the whole shift happens
#           across the riser and the bend. The J3 join is then flush and square, but the
#           wall bends away from the J2 pad rim after only S_PAD_MARGIN..4 mm of straight
#           wall, and the riser itself is no longer flush with the pad.
#   'bar'   (EXO-69): riser and bend stay straight, full width and centred on the J2 pad;
#           the S sits in the STRAIGHT BAR between the end of the bend and the stator slab,
#           with straight bar on both sides of it (S_BAR_MARGIN each at least).
# Everything downstream (the plan band, the wide extrude + intersect, the clearance sampler,
# the measured-body check) is written against `s_curve` and does not care which was chosen.
S_PLACEMENT = 'bar'                      # default: Esteban's choice (EXO-83);
                                         # 'riser' still reproduces EXO-64 exactly
S_PLACEMENTS = ('riser', 'bar')
S_BAR_MARGIN = 1.0                       # straight bar kept on EACH side of a bar-placed S


def _arc(segs, c, r, a0, a1):
    """Append a tangent arc about centre c from angle a0 to a1 (deg, through the mid angle)."""
    def p(a):
        return (c[0] + r * math.cos(math.radians(a)), c[1] + r * math.sin(math.radians(a)))
    segs.append(('arc', p(a0), p((a0 + a1) / 2.0), p(a1), (c[0], c[1]), r))


def _line(segs, p, q):
    segs.append(('line', (p[0], p[1]), (q[0], q[1])))


def _rev_segs(segs):
    """The same chain of segments walked the other way (endpoints swapped, arcs keep
    their centre, radius and mid point, so they stay the same arc)."""
    out = []
    for sg in reversed(segs):
        out.append(('line', sg[2], sg[1]) if sg[0] == 'line'
                   else ('arc', sg[3], sg[2], sg[1], sg[4], sg[5]))
    return out


def bar_side_profile(s_in, s_out, z_top, z_bot, s_face, s_end, root_s=PROFILE_ROOT_S,
                     pad_t=PAD_T, pad_r=None):
    """Link 2 R's side profile, as a ROUTE (EXO-96): a ledge join, a riser run of thickness
    s_out - s_in down the pad axis, one 90 degree bend (walls r 12.5 / r 15) and a bar run of
    thickness z_top - z_bot out to the stator slab, which it dies half way inside. Same
    arguments, same segments and same loud failures as the hand-written profile it replaced;
    `route_profile` draws it, and draws the shoulder plate too."""
    s_c, z_c = (s_in + s_out) / 2.0, (z_top + z_bot) / 2.0
    return route_profile(make_route(
        BAR_W, (s_c, pad_t / 2.0), (0.0, -1.0),
        [Run(pad_t / 2.0 - z_c, s_out - s_in, 'riser'),
         Bend(90.0, 'side', FILLET_OUT_BOT, FILLET_IN_BOT, 'bend'),
         Run(s_face - s_c, z_top - z_bot, 'bar')],
        (rotor_join('ledge', root_s, pad_r, pad_t),
         stator_land(s_face, FILLET_FLANGE, PROFILE_BURY, PROFILE_BURY_UP, s_end - s_face,
                     TONGUE_END_R, BAR_W + 2 * TONGUE_ROUND_MARGIN,
                     s_far=s_face + pad_t)),
        pattern='bar'))




def riser_s_curve(w_off, s_end, cap_lo, cap_hi, pad_r, half_w=BAR_W / 2.0, r=S_RADIUS):
    """Plan-view (s, w) band of the bar link: BAR_W wide, centred on w = 0 where it leaves
    the rotor pad, an S of two tangent arcs inside the riser/bend, then centred on w_off
    from `s_end` on (where the bar runs straight). `cap_lo`/`cap_hi` close the band well
    short of / past the body so extruding it and intersecting cannot trim the real ends.

    Returns None when |w_off| < S_MIN_OFFSET (a bar already centred needs no S), else a dict:
      s_start, s_end, length, theta_deg, r, wall_r (tight, wide), sign
      centres      : the two arc centres (s, w)
      inflection   : where the two centre arcs meet (on the line joining the centres)
      centre_line  : line + arc + arc + line, the band's centre line
      wall_lo/hi   : the w = centre -/+ half_w walls, each line + arc + arc + line
      band         : the closed outline (walls + end caps) to sketch and extrude
      w_range      : (min, max) lateral extent of the band
    Raises ValueError when the S cannot fit (loud failure)."""
    if abs(w_off) < S_MIN_OFFSET:
        return None
    if r <= half_w:
        raise ValueError('riser S radius %.2f must exceed half the bar width %.2f '
                         '(the inner wall would turn inside out)' % (r, half_w))
    if abs(w_off) > 2.0 * r:
        raise ValueError('riser S of radius %.2f cannot reach a %.2f mm offset (max %.2f)'
                         % (r, w_off, 2.0 * r))
    sg = 1.0 if w_off > 0 else -1.0
    th = math.acos(1.0 - abs(w_off) / (2.0 * r))
    th_deg = math.degrees(th)
    length = 2.0 * r * math.sin(th)
    s_start = s_end - length
    pad_exit = math.sqrt(max(0.0, pad_r ** 2 - half_w ** 2))
    if s_start < pad_exit + S_PAD_MARGIN:
        raise ValueError('riser S would start at s=%.2f, %.2f mm outside the r %.1f pad rim '
                         '(need %.1f): a %.2f mm shift needs %.2f mm of run at radius %.1f'
                         % (s_start, s_start - pad_exit, pad_r, S_PAD_MARGIN, w_off, length, r))
    if s_start <= cap_lo or s_end >= cap_hi:
        raise ValueError('riser S (s %.2f..%.2f) does not fit inside the band caps %.2f..%.2f'
                         % (s_start, s_end, cap_lo, cap_hi))
    c1 = (s_start, sg * r)                 # centre of the first arc (it curves toward w_off)
    c2 = (s_end, w_off - sg * r)           # centre of the second, on the other side
    a0, a1 = -sg * 90.0, sg * (th_deg - 90.0)          # first arc, start -> inflection
    b0, b1 = sg * (90.0 + th_deg), sg * 90.0           # second arc, inflection -> end

    def offset_curve(d):
        """The curve at lateral offset d from the centre line: concentric with both arcs,
        so it is exactly d away from the centre line across the band everywhere."""
        segs = []
        _arc(segs, c1, r - sg * d, a0, a1)
        _arc(segs, c2, r + sg * d, b0, b1)
        return segs

    centre_line = offset_curve(0.0)
    wall_lo, wall_hi = offset_curve(-half_w), offset_curve(half_w)
    band = []
    _line(band, (cap_lo, -half_w), (s_start, -half_w))
    band += wall_lo
    _line(band, (s_end, w_off - half_w), (cap_hi, w_off - half_w))
    _line(band, (cap_hi, w_off - half_w), (cap_hi, w_off + half_w))
    _line(band, (cap_hi, w_off + half_w), (s_end, w_off + half_w))
    band += _rev_segs(wall_hi)
    _line(band, (s_start, half_w), (cap_lo, half_w))
    _line(band, (cap_lo, half_w), (cap_lo, -half_w))
    if not profile_closed(band):
        raise ValueError('riser S band outline does not close')
    inflection = centre_line[0][3]        # end of the first centre arc
    return {
        'r': r, 'wall_r': (r - half_w, r + half_w), 'sign': sg,
        's_start': s_start, 's_end': s_end, 'length': length, 'theta_deg': th_deg,
        'w_off': w_off, 'half_w': half_w, 'centres': (c1, c2), 'inflection': inflection,
        'centre_line': [('line', (cap_lo, 0.0), (s_start, 0.0))] + centre_line
                       + [('line', (s_end, w_off), (cap_hi, w_off))],
        'wall_lo': [('line', (cap_lo, -half_w), (s_start, -half_w))] + wall_lo
                   + [('line', (s_end, w_off - half_w), (cap_hi, w_off - half_w))],
        'wall_hi': [('line', (cap_lo, half_w), (s_start, half_w))] + wall_hi
                   + [('line', (s_end, w_off + half_w), (cap_hi, w_off + half_w))],
        'band': band,
        'w_range': (min(-half_w, w_off - half_w), max(half_w, w_off + half_w)),
        'cap': (cap_lo, cap_hi),
    }


def s_curve_run(w_off, r=S_RADIUS):
    """(theta_deg, run) of the two-arc S that shifts the band by `w_off` at centre-line
    radius `r`: 2R(1 - cos th) = |w_off|, run = 2R sin th. The run GROWS with r, so the
    smallest radius the walls allow (S_RADIUS, wall radius S_WALL_R_MIN) is also the
    shortest S. Used to place the S before drawing it; riser_s_curve derives the same
    numbers itself and is the single source of the geometry."""
    if r <= 0.0:
        raise ValueError('S radius %.2f must be positive' % r)
    if abs(w_off) > 2.0 * r:
        raise ValueError('an S of radius %.2f cannot reach a %.2f mm offset (max %.2f)'
                         % (r, w_off, 2.0 * r))
    th = math.acos(1.0 - abs(w_off) / (2.0 * r))
    return math.degrees(th), 2.0 * r * math.sin(th)


def bar_s_curve(w_off, s_from, s_to, cap_lo, cap_hi, pad_r, half_w=BAR_W / 2.0, r=S_RADIUS,
                margin=S_BAR_MARGIN):
    """EXO-69 option 'bar': the SAME S as riser_s_curve, placed inside the straight bar.
    `s_from`..`s_to` is the straight-bar window (end of the bend .. the slab's bar-side
    face); the S is CENTRED in it, so the straight bar left on each side is equal and as
    long as it can be. Returns riser_s_curve's dict plus `margin` and `window`.
    Raises ValueError when the window is too short for the run plus `margin` per side."""
    if abs(w_off) < S_MIN_OFFSET:
        return None
    th_deg, length = s_curve_run(w_off, r)
    slack = (s_to - s_from) - length
    if slack < 2.0 * margin - 1e-12:
        raise ValueError('bar S: a %.2f mm shift needs %.2f mm of run at radius %.1f, but the '
                         'straight bar is only %.2f mm long (s %.2f..%.2f) - %.2f mm of straight '
                         'bar per side, need %.2f. A shorter run needs a radius below %.1f, i.e. '
                         'a wall radius below %.1f'
                         % (w_off, length, r, s_to - s_from, s_from, s_to, slack / 2.0, margin,
                            r, r - half_w))
    m = slack / 2.0
    sc = riser_s_curve(w_off, s_to - m, cap_lo, cap_hi, pad_r, half_w, r)
    sc['margin'] = (m, m)
    sc['window'] = (s_from, s_to)
    return sc


def s_curve_w(sc, s):
    """(w_lo, w_hi) of the band at station s, or None outside the band's caps."""
    if sc is None:
        return None
    lo, hi = sc['cap']
    if s < lo or s > hi:
        return None
    return (_chain_w(sc['wall_lo'], s), _chain_w(sc['wall_hi'], s))


def _chain_w(segs, s):
    """w of a monotone (in s) line/arc chain at station s."""
    for sg in segs:
        if sg[0] == 'line':
            (s0, w0), (s1, w1) = sg[1], sg[2]
            if min(s0, s1) - 1e-9 <= s <= max(s0, s1) + 1e-9:
                if abs(s1 - s0) < 1e-12:
                    return w0
                return w0 + (w1 - w0) * (s - s0) / (s1 - s0)
        else:
            _, p0, _pm, p1, c, r = sg
            if min(p0[0], p1[0]) - 1e-9 <= s <= max(p0[0], p1[0]) + 1e-9:
                dy = math.sqrt(max(0.0, r ** 2 - (s - c[0]) ** 2))
                cands = [c[1] + dy, c[1] - dy]
                lo, hi = min(p0[1], p1[1]) - 1e-6, max(p0[1], p1[1]) + 1e-6
                inside = [v for v in cands if lo <= v <= hi]
                return inside[0] if inside else min(cands, key=lambda v: abs(v - p0[1]))
    raise ValueError('station s=%.3f is outside the chain' % s)


def tongue_round_outline(s_far, z_bot, r=TONGUE_END_R, side=1):
    """Corner region cut away by the round on the tongue's end edge, in the (s, z) profile
    plane: the corner (s_far = slab stator-side face, z_bot = the run boundary the round sits
    on) minus the quarter disc of radius r tangent to both faces. Link 2 R: centre
    (70.8, -25.1). `side` is +1 when the body is on the +z side of that boundary (the bar and
    the shoulder L, whose round is under the run) and -1 when it is below it (the ramp, whose
    land fillet is under the run and whose round is therefore on top - Link 4 R, EXO-114)."""
    c = (s_far - r, z_bot + side * r)
    segs = []
    _line(segs, (c[0], z_bot), (s_far, z_bot))
    _line(segs, (s_far, z_bot), (s_far, c[1]))
    _arc(segs, c, r, 0.0, -90.0 * side)
    return segs


# Radii already measured in the exo links, largest first: when the standard r 15 tongue
# blend does not fit (an OFFSET tongue - the shoulder L - runs out of room on the side that
# points at the disk centre), `flange_edges` falls back through this list rather than
# inventing a radius (same argument as EXO-84's JOIN_ROUND_R).
BLEND_RADII = (FLANGE_BLEND_R, FILLET_OUT_BOT, FILLET_FLANGE, JOIN_ROUND_R)   # 15, 12.5, 8.5, 5
FLANGE_TONGUE_MARGIN = 1.0   # straight tongue wall left past the blend's tangent point
# A tongue wider than the stator disk cannot blend into the rim at all on the side that
# overhangs it: the wall never reaches the rim. The hand-built Link 1 R - a 52 mm plate on a
# r 27 disk offset 13.8 mm - answers that with one huge arc, r 500, EXTERNALLY tangent to the
# rim (measured: centre 527.00 = 500 + 27 from the disk axis, tangent point exactly
# 27 C/|C|, sweep 4.25 deg, sagitta 0.34 mm). It reads as a straight taper from the plate's
# corner that lands on the rim without an edge. `flange_toe` reproduces it.
FLANGE_TOE_R = 500.0
FLANGE_TOE_MARGIN = 1.0      # straight wall left between the tongue end and the toe corner
# The SECOND hand-built answer to the same problem, measured on `Link 3 R` (EXO-105). Where
# Link 1 R's tongue is 26 mm long and its toe reads as a straight taper (sagitta 0.34 mm),
# Link 3 R's is 80 mm - a real column - and a toe over that length would bow 2.7 mm and neck
# the column from 52 to 43 mm on one side. Link 3 R instead rounds the overhanging wall into
# the disk's own far TANGENT LINE (y = -pad_r) with a big fillet and then runs flat into the
# rim's tangent point: wall, fillet, flat, rim, tangent at every joint and with no corner
# vertex at all, and the column's two walls stay parallel over its whole length. Measured:
# fillet r 35.000 centred (-6.783, 9.000) on a r 26 disk, leaving 6.783 mm of flat.
# The radius is the largest fillet that still leaves FLANGE_CROWN_FLAT of real flat between
# itself and the rim's tangent point, so nothing is invented: r = |x_wall| - FLANGE_CROWN_FLAT.
FLANGE_CROWN_FLAT = FLANGE_TONGUE_MARGIN


def _flange_blend(x_wall, side, y_end, pad_r, blend_r):
    """One tongue-to-rim blend, for a tongue wall that lies INSIDE the disk. `x_wall` = the
    wall's x in the slab frame, `side` = +1 when the material is at smaller x (so the blend
    centre sits at larger x) and -1 the other way. Returns (centre, y_c, rim tangent point)."""
    cx = x_wall + side * blend_r
    rr = pad_r + blend_r
    if abs(x_wall) >= pad_r:
        raise ValueError('flange blend: the tongue wall at x=%.2f is outside the r %.1f disk, '
                         'so it never reaches the rim' % (x_wall, pad_r))
    if abs(cx) >= rr:
        raise ValueError('flange blend r %.1f at x=%.2f cannot reach the r %.1f rim'
                         % (blend_r, x_wall, pad_r))
    y_c = math.sqrt(rr ** 2 - cx ** 2)
    if y_end <= y_c + FLANGE_TONGUE_MARGIN:
        raise ValueError('flange to y=%.2f is too short: the r %.1f blend at x=%.2f ends at '
                         'y=%.2f (need %.1f mm of straight wall past it)'
                         % (y_end, blend_r, x_wall, y_c, FLANGE_TONGUE_MARGIN))
    k = pad_r / rr
    return (cx, y_c), y_c, (cx * k, y_c * k)


def flange_toe(x_wall, side, y_t, pad_r, toe_r=FLANGE_TOE_R, x_off=0.0):
    """Link 1 R's taper (above): the arc of radius `toe_r` through the tongue wall's corner
    (x_wall, y_t) that is EXTERNALLY tangent to the r `pad_r` rim, on the side away from the
    tongue. Returns (centre, rim tangent point). Raises when no such arc exists."""
    pl = math.hypot(x_wall, y_t)
    if pl <= pad_r:
        raise ValueError('flange toe: the corner (%.2f, %.2f) is inside the r %.1f disk'
                         % (x_wall, y_t, pad_r))
    r_min = (pl - pad_r) / 2.0
    if toe_r < r_min:
        raise ValueError('flange toe: r %.1f is below the %.2f mm needed to reach the r %.1f '
                         'rim from the corner (%.2f, %.2f)' % (toe_r, r_min, pad_r, x_wall, y_t))
    # a circle of radius toe_r tangent to the rim touches it at pad_r * u and is centred at
    # (pad_r + toe_r) * u; |C - P| = toe_r fixes P . u, and the tongue side picks the root
    q = (pad_r ** 2 + pl ** 2 + 2 * pad_r * toe_r) / (2.0 * (pad_r + toe_r))
    d = max(-1.0, min(1.0, q / pl))
    th = math.atan2(y_t, x_wall) - side * math.acos(d)
    u = (math.cos(th), math.sin(th))
    c = ((pad_r + toe_r) * u[0], (pad_r + toe_r) * u[1])
    t = (pad_r * u[0], pad_r * u[1])
    # Two arcs of this radius touch the rim; only the one that leads AWAY from the tongue is
    # the edge. The other lands on the rim on the far side of the tongue's centre line and
    # would sweep the arc straight across the tongue.
    if (t[0] - x_off) * side < 0.0:
        raise ValueError('flange toe: the r %.1f arc from the corner (%.2f, %.2f) touches the '
                         'rim at (%.2f, %.2f), on the far side of the tongue centre line '
                         'x=%.2f - it would sweep straight across the tongue'
                         % (toe_r, x_wall, y_t, t[0], t[1], x_off))
    return c, t


def flange_crown(x_wall, side, pad_r, flat=FLANGE_CROWN_FLAT):
    """Link 3 R's crown (above): for a tongue wall OUTSIDE the r `pad_r` disk, the fillet
    tangent to that wall and to the disk's far tangent line y = -pad_r, followed by `flat`
    mm of that line into the rim's own tangent point (0, -pad_r).

    `side` is +1 for the wall at larger x (material at smaller x) and -1 the other way, as
    in `_flange_blend`. Returns (radius, centre, wall tangent y, rim tangent point).
    Raises when the wall is inside the disk (an ordinary blend belongs there) or when the
    fillet would be too small to be a round."""
    if abs(x_wall) <= pad_r:
        raise ValueError('flange crown: the tongue wall at x=%.2f is inside the r %.1f disk, '
                         'where a tangent blend reaches the rim' % (x_wall, pad_r))
    r = abs(x_wall) - flat
    if r <= pad_r:
        raise ValueError('flange crown: a wall at x=%.2f leaves only r %.2f once %.1f mm of '
                         'flat is kept, which does not clear the r %.1f rim'
                         % (x_wall, r, flat, pad_r))
    c = (side * flat, r - pad_r)
    return r, c, r - pad_r, (0.0, -pad_r)


# A tongue wall that sits EXACTLY on the disk's own tangent line (|x| = pad_r) needs no edge
# at all: wall and rim already touch, tangentially, at one point. `Link 4 R`'s +x' wall is
# that case by construction - the plan taper draws each wall tangent to the rim it has to
# stay inside (EXO-114) - and neither a blend (which wants the wall INSIDE the disk) nor a
# toe (which wants its corner OUTSIDE) is defined there. The tolerance is only for the
# arithmetic: the sizer puts the wall on the rim exactly.
FLANGE_TANGENT_TOL = 1e-6

FLANGE_OVERHANG = ('toe', 'crown')   # what an overhanging tongue wall does; see above


def flange_edges(y_end, x_off=0.0, half_w=BAR_W / 2.0, pad_r=STATOR_PAD_R,
                 candidates=BLEND_RADII, toe_y=None, toe_r=FLANGE_TOE_R,
                 overhang='toe'):
    """How each tongue wall meets the disk rim. Per side, either the largest blend radius in
    `candidates` that fits, or - when that wall lies OUTSIDE the disk, where no blend of any
    radius reaches the rim - whichever `overhang` construction the route asked for: Link 1 R's
    toe arc ('toe', which needs the corner `toe_y` where the straight wall stops) or Link 3 R's
    crown ('crown'). Returns two dicts with kind ('blend' / 'toe' / 'crown'), r, centre,
    y_wall (where the straight wall ends) and the rim tangent point `t`; a crown also carries
    `flat`, the point where the fillet meets the disk's tangent line. Raises naming the side
    when neither works."""
    if overhang not in FLANGE_OVERHANG:
        raise ValueError('unknown flange overhang %r (expected one of %s)'
                         % (overhang, ', '.join(FLANGE_OVERHANG)))
    out = []
    for x_wall, side, label in ((x_off + half_w, +1, '+x'), (x_off - half_w, -1, '-x')):
        edge = None
        if abs(abs(x_wall) - pad_r) <= FLANGE_TANGENT_TOL:
            if y_end <= FLANGE_TONGUE_MARGIN:
                raise ValueError('flange %s side: the wall on the rim at x=%.2f leaves only '
                                 '%.2f mm of tongue (need %.1f)'
                                 % (label, x_wall, y_end, FLANGE_TONGUE_MARGIN))
            out.append({'kind': 'tangent', 'r': 0.0, 'c': (x_wall, 0.0), 'y_wall': 0.0,
                        't': (x_wall, 0.0), 'x': x_wall, 'side': side})
            continue
        for r in candidates:
            try:
                c, y_c, t = _flange_blend(x_wall, side, y_end, pad_r, r)
            except ValueError as e:
                last = e
                continue
            edge = {'kind': 'blend', 'r': r, 'c': c, 'y_wall': y_c, 't': t, 'x': x_wall,
                    'side': side}
            break
        if edge is None and abs(x_wall) >= pad_r and overhang == 'crown':
            r, c, y_wall, t = flange_crown(x_wall, side, pad_r)
            if y_end <= y_wall + FLANGE_TONGUE_MARGIN:
                raise ValueError('flange %s side: the r %.1f crown meets the wall at y=%.2f, '
                                 'past the tongue end at y=%.2f (need %.1f mm of straight '
                                 'wall)' % (label, r, y_wall, y_end, FLANGE_TONGUE_MARGIN))
            edge = {'kind': 'crown', 'r': r, 'c': c, 'y_wall': y_wall, 't': t, 'x': x_wall,
                    'side': side, 'flat': (c[0], -pad_r)}
        if edge is None and abs(x_wall) >= pad_r:
            if toe_y is None:
                raise ValueError('flange %s side: the tongue wall at x=%.2f is outside the '
                                 'r %.1f disk and no toe corner was given' % (label, x_wall, pad_r))
            if y_end <= toe_y + FLANGE_TOE_MARGIN:
                raise ValueError('flange %s side: the toe corner at y=%.2f leaves no straight '
                                 'wall before the tongue end at y=%.2f (need %.1f)'
                                 % (label, toe_y, y_end, FLANGE_TOE_MARGIN))
            c, t = flange_toe(x_wall, side, toe_y, pad_r, toe_r, x_off)
            edge = {'kind': 'toe', 'r': toe_r, 'c': c, 'y_wall': toe_y, 't': t, 'x': x_wall,
                    'side': side}
        if edge is None:
            raise ValueError('flange %s side: no blend radius of %s fits a tongue wall at '
                             'x=%.2f reaching y=%.2f on the r %.1f disk (%s)'
                             % (label, ', '.join('%g' % r for r in candidates), x_wall,
                                y_end, pad_r, last))
        out.append(edge)
    return tuple(out)


def flange_blend_radii(y_end, x_off=0.0, half_w=BAR_W / 2.0, pad_r=STATOR_PAD_R,
                       candidates=BLEND_RADII, toe_y=None):
    """(r_plus, r_minus) of the two tongue edges (see flange_edges)."""
    return tuple(e['r'] for e in flange_edges(y_end, x_off, half_w, pad_r, candidates, toe_y))


def flange_outline(y_end, half_w=BAR_W / 2.0, pad_r=STATOR_PAD_R, blend_r=FLANGE_BLEND_R,
                   x_off=0.0, blend_r_minus=None, edges=None, toe_y=None, end_pts=None):
    """Closed outline of the stator slab in the slab frame's (x, y): r pad_r disk plus the
    flange tongue x_off -/+ half_w out to y_end, each side carried into the rim by
    `flange_edges` (a tangent blend, or Link 1 R's toe arc when the wall overhangs the disk).
    Pass `edges` to reuse an already-chosen pair; otherwise `blend_r` / `blend_r_minus` name
    the +x / -x blend radii directly and `toe_y` the toe corner.
    `x_off` offsets the tongue from the disk centre - 0 for the bar, where the plan-view S
    brings the band onto the stator's centre line, and -w_off for the shoulder L, where there
    is no room for an S and the offset is taken here instead (EXO-81).
    Returns (segments, y_blend, tangent_point) of the +x side (the bar's only side)."""
    if edges is None:
        r_m = blend_r if blend_r_minus is None else blend_r_minus
        ep, _, tp = _flange_blend(x_off + half_w, +1, y_end, pad_r, blend_r)
        em, _, tm = _flange_blend(x_off - half_w, -1, y_end, pad_r, r_m)
        edges = ({'kind': 'blend', 'r': blend_r, 'c': ep, 'y_wall': ep[1], 't': tp,
                  'x': x_off + half_w, 'side': +1},
                 {'kind': 'blend', 'r': r_m, 'c': em, 'y_wall': em[1], 't': tm,
                  'x': x_off - half_w, 'side': -1})
    ed_p, ed_m = edges
    if x_off - half_w <= -pad_r - FLANGE_TOE_R or x_off + half_w >= pad_r + FLANGE_TOE_R:
        raise ValueError('flange tongue x %.2f..%.2f is nowhere near the r %.1f disk'
                         % (x_off - half_w, x_off + half_w, pad_r))

    def arc_ends(ed):
        """(start angle, end angle) about the edge's centre: from the wall to where the arc
        ends - the rim tangent point for a blend or a toe, the disk's tangent line for a
        crown, whose last stretch to the rim is a straight flat and not part of the arc."""
        end = ed['flat'] if ed['kind'] == 'crown' else ed['t']
        a_w = math.degrees(math.atan2(ed['y_wall'] - ed['c'][1], ed['x'] - ed['c'][0]))
        a_t = math.degrees(math.atan2(end[1] - ed['c'][1], end[0] - ed['c'][0]))
        if ed['kind'] == 'blend':
            # a blend's wall tangent point is at -180 deg (+x side) or 0 deg (-x side) about
            # its centre and its rim tangent point below it: always the SHORT way (EXO-61)
            return (-180.0 if ed['side'] > 0 else 0.0), (a_t if a_t < 0 else a_t - 360.0)
        while a_t - a_w > 180.0:
            a_t -= 360.0
        while a_t - a_w < -180.0:
            a_t += 360.0
        return a_w, a_t

    def edge_to_rim(ed):
        """The edge walked from its straight wall to its rim tangent point: one arc, plus -
        for a crown - the flat along the disk's tangent line that follows it (EXO-105). A
        wall already ON the rim's tangent line touches it at a point and contributes
        nothing (EXO-114)."""
        if ed['kind'] == 'tangent':
            return []
        out = []
        _arc(out, ed['c'], ed['r'], *arc_ends(ed))
        if ed['kind'] == 'crown':
            _line(out, ed['flat'], ed['t'])
        return out

    a_rim_m = math.degrees(math.atan2(ed_m['t'][1], ed_m['t'][0]))
    a_rim_p = math.degrees(math.atan2(ed_p['t'][1], ed_p['t'][0]))
    while a_rim_p <= a_rim_m:
        a_rim_p += 360.0
    # EXO-177: `end_pts` is where the tongue stops - the default is the one flat line square
    # to the tongue that every square landing draws, and an angled landing hands in the
    # ribbon's own outline instead (`tongue_end`).
    ends = ([(x_off + half_w, y_end), (x_off - half_w, y_end)] if end_pts is None
            else list(end_pts))
    segs = []
    _line(segs, (x_off + half_w, ed_p['y_wall']), ends[0])
    for k in range(len(ends) - 1):
        _line(segs, ends[k], ends[k + 1])
    _line(segs, ends[-1], (x_off - half_w, ed_m['y_wall']))
    segs += edge_to_rim(ed_m)                                        # -x edge into the rim
    _arc(segs, (0.0, 0.0), pad_r, a_rim_m, a_rim_p)                  # the rim, the long way round
    segs += _rev_segs(edge_to_rim(ed_p))                             # +x edge back to the tongue
    if not profile_closed(segs, 1e-4):
        raise ValueError('flange outline does not close')
    return segs, ed_p['y_wall'], ed_p['t']


MODULE_Z_TOP = -4.0     # module A's stator top face (the rotor sits above it); the r 27
                        # cylinder to clear runs from here down past the previous link's slab


def _seg_points(seg, step):
    """Points along one profile segment, at most `step` mm apart."""
    if seg[0] == 'line':
        (x0, y0), (x1, y1) = seg[1], seg[2]
        n = max(1, int(math.ceil(math.hypot(x1 - x0, y1 - y0) / step)))
        return [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(n + 1)]
    _, p0, pm, p1, c, r = seg
    a0, am, a1 = (math.atan2(q[1] - c[1], q[0] - c[0]) for q in (p0, pm, p1))
    d1, dm = (a1 - a0) % (2 * math.pi), (am - a0) % (2 * math.pi)
    if dm > d1:
        d1 -= 2 * math.pi
    n = max(1, int(math.ceil(abs(d1) * r / step)))
    return [(c[0] + r * math.cos(a0 + d1 * k / n), c[1] + r * math.sin(a0 + d1 * k / n))
            for k in range(n + 1)]


def riser_radial_clearance(bar, r_keep=None, z_top=MODULE_Z_TOP, s_from=None, step=0.02):
    """Smallest radial clearance (mm) between the link body and the r `r_keep` cylinder
    about axis A (module A's stator and the previous link's stator slab) over the z range
    where that cylinder exists. Every body point is (s, w, z); its distance from axis A is
    sqrt(s^2 + w^2), so the plan-view S - which only moves w - can never bring the body
    closer than the side profile already does. Samples the side profile's boundary (which
    contains the inner boundary s_min(z)) and takes, at each station, the band's w nearest
    the u-z plane. `s_from` restricts the sampling to stations at or beyond s_from (used to
    report the clearance ALONG the S). Returns (clearance, (s, w, z)) at the minimum."""
    if r_keep is None:
        r_keep = max(MODULE_R, STATOR_PAD_R)
    sc = bar.get('s_curve')
    half = bar.get('width', BAR_W) / 2.0
    best, where = float('inf'), None
    for seg in bar['profile']:
        for (s, z) in _seg_points(seg, step):
            if z > z_top or (s_from is not None and s < s_from - 1e-9):
                continue
            span = s_curve_w(sc, s)
            if span is None:
                span = (bar['w_off'] - half, bar['w_off'] + half)
            lo, hi = min(span), max(span)
            w = 0.0 if lo <= 0.0 <= hi else (lo if lo > 0.0 else hi)
            d = math.hypot(s, w) - r_keep
            if d < best:
                best, where = d, (s, w, z)
    if where is None:
        raise ValueError('riser_radial_clearance: no body point below z=%.1f' % z_top)
    return best, where


def pad_outline(pad_r, trim):
    """The rotor pad's outline in the pad plane (x, y): the r `pad_r` disk with ONE straight
    chord taken off it, where the next module's body is in the way (EXO-181).

    `trim` is `(nx, ny, c)` - the material kept is `dot(p, n) >= c`, with `n` a unit vector
    in the pad plane. `None` means no chord at all and the caller draws the plain circle it
    always drew, so a link whose pad clears is untouched.

    Returned as the usual closed outline: the chord, then the MAJOR arc back round the far
    side of the disk."""
    if trim is None:
        return None
    nx, ny, c = trim
    n = math.hypot(nx, ny)
    if n < 1e-12:
        raise ValueError('pad trim: the chord has no direction')
    nx, ny = nx / n, ny / n
    if c >= pad_r - 1e-9:
        raise ValueError('pad trim: a chord at %.3f mm takes the whole r %.1f pad'
                         % (c, pad_r))
    if c <= -pad_r + 1e-9:
        return None                          # the chord misses the disk: nothing to cut
    half = math.sqrt(max(0.0, pad_r ** 2 - c * c))
    tx, ty = -ny, nx                         # the chord's own direction
    p1 = (c * nx - half * tx, c * ny - half * ty)
    p2 = (c * nx + half * tx, c * ny + half * ty)
    th = math.degrees(math.atan2(ny, nx))
    phi = math.degrees(math.atan2(half, c))
    segs = []
    _line(segs, p2, p1)
    _arc(segs, (0.0, 0.0), pad_r, th - phi, th + phi)
    if not profile_closed(segs, 1e-6):
        raise ValueError('pad outline does not close')
    return segs


def pad_trim_depth(pad_r, trim):
    """How deep into the r `pad_r` disk the chord cuts, mm. 0 when there is no chord.

    The disk spans `dot(p, n)` from -pad_r to +pad_r and the material kept is `>= c`, so what
    comes off is the segment of height `pad_r + c`."""
    return 0.0 if trim is None else max(0.0, pad_r + trim[2])


def stop_block_outline(ang, half, r_in=STOP_R_IN, r_out=ROTOR_PAD_R):
    """Ring sector in the pad plane (x, y): r_in..r_out, angles ang +/- half (deg)."""
    segs = []
    _arc(segs, (0.0, 0.0), r_out, ang - half, ang + half)
    p_out1 = segs[-1][3]
    p_in1 = (r_in * math.cos(math.radians(ang + half)), r_in * math.sin(math.radians(ang + half)))
    _line(segs, p_out1, p_in1)
    _arc(segs, (0.0, 0.0), r_in, ang + half, ang - half)
    p_in0 = segs[-1][3]
    _line(segs, p_in0, segs[0][1])
    return segs


# ---------------- EXO-204: the limited joint ----------------
# Two blocks make the mechanical limit and only ONE of them is generated.
#
#   JOINT stop - the fixed block on the stator rim of `sensor module 2`. One module design,
#   the same at every joint. Measured off the module STEP in the module frame (these are the
#   numbers, not EXO-77's rounding to 158/178): r 21.05..27.000, z -4.000..-0.400, angles
#   158.0026 .. 178.0019 deg. EXO-208 narrows that block to `JOINT_STOP_DEG` about the same
#   measured centre, in a COPY of the module (`sensor module 2 stop15`, BuildArm.
#   MODULE_STOP_NAME) - the original design is not touched. Radii and z are unchanged.
#
#   LINK stop - the block the generator puts under the link's rotor pad (r STOP_R_IN..pad_r,
#   z -STOP_H..0, `stop_block_outline`). It turns with the rotor.
#
# The rotor turns freely until the two arcs touch, so the travel the joint gets is the whole
# circle less the two arcs: 2*pi - a - b. Making that EQUAL the URDF's own range fixes the
# link stop's ARC; making the first touch happen at `lower` fixes its ANGLE. Neither is a
# choice, so there is nothing per joint to hand-set - the inputs are the URDF's two limits,
# the measured module, and the joint's axis sign.
#
# ZERO POSE. URDF q = 0 is the pose every module frame in this file is built at
# (`joint_world_pose` walks the chain with no joint angles applied), so the link AS DRAWN in
# the module frame is the link at q = 0. That drawing is realisable on the real rotor in
# exactly one orientation: the rotor's four M3 holes are 90 deg symmetric and the ROTOR
# KEY breaks the tie (EXO-364; before it, the index notch at +Y did): three pegs on the
# link's rotor pad into three sockets in the rotor top at KEY_ANGLES 45 / 135 / 225, with
# 315 empty. So the convention this rule is written to, and the one the whole generator
# assumes, is
#
#     URDF q = 0  <->  the empty key position at module 315 deg  <->  the link as drawn.
#
# AXIS SIGN. The module frame's Z is `axis_sign` * the URDF axis (`module_frame`), so a URDF
# rotation of q is a rotation of axis_sign * q about module +Z. J1 is axis_sign = -1: its
# rotor turns CLOCKWISE in the module frame as q grows, so its link stop has to sit on the
# other side of the joint stop from an axis_sign = +1 joint. This flip is the whole
# difference between the two placements and it is not optional.
#
# EXO-208. The joint stop's WIDTH is the one hyperparameter of this whole rule, and it is
# `JOINT_STOP_DEG`. Everything else on both blocks is measured or derived:
#
#   * its CENTRE, its two radii and its z span are the module's, measured, and they do not
#     move. `JOINT_STOP_MID` stays the measured 168.00225 deg; the block is re-cut
#     symmetrically about it, so the only thing that changes is how much of the circle it
#     eats.
#   * the LINK stop's arc is then 2*pi - JOINT_STOP_ARC - range, as before.
#
# Why the joint stop is the MINIMUM and the link stop takes the rest (Esteban, EXO-208):
# the two blocks share the circle, so one of them has to give way to a wide joint. The
# module's is the one that is the same at every joint of every arm, so it is the one that
# is sized once, as small as the arm can live with; the link's is generated per joint and
# absorbs whatever is left. The rule below therefore reads
#
#     link stop b = 2*pi - a - range,   and   b >= a.
#
# WHY 15 AND NOT LESS. 15.0 deg is the widest single value that still limits every joint of
# the G1 arm: J1 (right_shoulder_pitch, range 330.00077 deg) leaves b = 14.99923 deg, so
# 15 is J1's ceiling to within the measurement tolerance below, and J1 comes out at 15/15 -
# the two blocks the same width, the circle split exactly. Gento Luna's 346 deg joints
# (J1/J3/J5) would need 7 deg, which is DEFERRED: a 7 deg block on this module is a thinner
# standing feature than anything that has been printed, and thinning it is not something to
# do while the rest of the arm is still moving. One number, one place to change it.
JOINT_STOP_DEG = 15.0
# The module's block AS MEASURED, before EXO-208 re-cut it (`tools/bridge_scripts/
# measure_exo204.py` over `sensor module 2.step`, module frame): a nominal 20 deg block at
# 158.0026 .. 178.0019, i.e. 19.9993 deg wide, centred on 168.00225. The centre is the only
# thing carried forward; the width is now JOINT_STOP_DEG.
JOINT_STOP_MEASURED_LO_DEG = 158.0026
JOINT_STOP_MEASURED_HI_DEG = 178.0019
JOINT_STOP_MEASURED_ARC = math.radians(JOINT_STOP_MEASURED_HI_DEG
                                       - JOINT_STOP_MEASURED_LO_DEG)        # 0.349055 rad
JOINT_STOP_MID = math.radians((JOINT_STOP_MEASURED_LO_DEG
                               + JOINT_STOP_MEASURED_HI_DEG) / 2.0)         # 168.00225 deg
JOINT_STOP_ARC = math.radians(JOINT_STOP_DEG)
JOINT_STOP_LO_DEG = math.degrees(JOINT_STOP_MID) - JOINT_STOP_DEG / 2.0     # 160.50225
JOINT_STOP_HI_DEG = math.degrees(JOINT_STOP_MID) + JOINT_STOP_DEG / 2.0     # 175.50225
JOINT_STOP_R_IN = 21.05
JOINT_STOP_Z = (-4.0, -0.4)
TWO_PI = 2.0 * math.pi

# The floor on the link stop is now the joint stop itself: `b >= a`. It is not a taste and
# it is not a second hyperparameter - it is what "the joint stop is the minimum" means. Two
# blocks meet face to face and carry the same load, so the generated one may never be the
# weaker of the pair; and since a + b + range = 2*pi, `b >= a` is exactly the statement that
# this module can limit a range of at most 2*pi - 2*a = 330 deg. A joint wider than that is
# refused by name with the numbers - never silently clipped.
#
# (EXO-204's LINK_STOP_B_MIN = 10 deg is gone. It was the arc of the one hand-built block
# that existed, `Link 1 R`'s +/-5 deg, used as a floor for want of a rule; the +/-5 deg
# block is still what the rule hands J1, but now because 330 deg of range beside a 15 deg
# joint stop leaves 15 deg - not because a printed part said so. Section is no longer the
# binding argument either: at the block's root radius STOP_R_IN = 20 a 15 deg arc is a
# 5.22 mm chord over the 3.6 mm it stands proud, 18.8 mm2 in shear, comfortably past the
# old index tab's 3.6 mm and LAND_WEB's 3.0.)
#
# B_TOL is the resolution the comparison is worth. `a` was measured off a STEP whose two
# edges of ONE nominal 20.000 deg block come out at 158.0026 and 178.0019 - 0.0026 deg of
# export scatter - and the URDF states its limits to 1e-4 rad = 0.0057 deg. Comparing b to
# a any finer than that is comparing noise. 0.003 deg is 1.4 um of chord at r 26.
#
# It matters exactly once, on the joint EXO-204 was opened for: G1's shoulder pitch has
# range 330.00077 deg, so b = 14.99923 deg - 0.00077 deg (0.13 um of chord at r 20) under
# the 15 deg joint stop. The design intent is unmistakable: 330 deg of range beside a 15 deg
# joint stop leaves 15 deg. J1 passes on B_TOL, and the deficit is reported so it is never
# silently absorbed.
LINK_STOP_B_TOL = math.radians(0.003)


def _wrap_pi(a):
    """Angle into (-pi, pi]."""
    a = math.fmod(a, TWO_PI)
    if a > math.pi:
        a -= TWO_PI
    elif a <= -math.pi:
        a += TWO_PI
    return a


# EXO-218: the link stop is ONE BLOCK of arc `b` (Esteban, 2026-09-26).
#
# EXO-214 drew it as two tabs of `JOINT_STOP_DEG` on the two ends of `b` and left the rim
# between them bare, on the argument that only the two ENDS ever touch the joint stop. They
# bought nothing. The one refusal they were meant to answer - the EXO-181 pad chord on
# link_67 - was answered by the slide instead (EXO-214/EXO-217: at the solved slide there is
# no chord at all, and J6's chord aimed straight at a tab anyway, EXO-216 item 2). The other
# thing they changed, the EXO-213 ribbon-root overlap, is non-physical: the stop is UNDER the
# pad and the ribbon root is ON it. So the stop goes back to what EXO-204 drew - one solid
# sector `centre +/- b/2`.
#
# Nothing about the LIMIT changes either way: the contact faces are the two ends of `b`, so
# `stop_gap`, `stop_sweep`, the travel and both contact poses are the same numbers to the
# bit, and J1 - whose whole arc is 15 deg - is unchanged in what is BUILT as well.


def stop_block_points(ang, half, pad_r, r_in=STOP_R_IN, step=0.5):
    """The link stop AS BUILT, polygonised: [(x, y), ...] round the one sector `ang +/-
    half`, r `r_in`..`pad_r`. One place discretises the built stop, so the EXO-181 chord
    check, the `link_samples` sample set and the reports all measure the same outline."""
    return [pt for sg in stop_block_outline(ang, half, r_in, pad_r)
            for pt in _seg_points(sg, step)]


# EXO-218: the RANGE CAP (Esteban, 2026-09-26).
#
# `a + b + range = 2*pi` and `b >= a`, so this module can limit at most `2*pi - 2*a` =
# 330.0000 deg (EXO-209). A joint whose URDF range is wider than that used to be REFUSED by
# name. Gento Luna's R1 / R3 / R5 are 345.998 deg, and refusing three joints of a seven-joint
# arm builds no arm, so the rule is: cap the range at what the module can limit, SYMMETRICALLY
# about the URDF midpoint, and carry the fact on the spec so every report says which joints
# were capped and by how much. The alternative - a narrower `JOINT_STOP_DEG` (Luna would need
# 7.0011) - changes the MODULE, which is a fixed design; that is Esteban's call, not this
# rule's.
#
# The threshold is the same one the refusal used, `b < JOINT_STOP_ARC - LINK_STOP_B_TOL`, so a
# joint that only misses by measurement noise is NOT capped: G1's J1 is 330.00077 deg and
# comes through untouched, exactly as EXO-208 left it.
MAX_LIMITED_RANGE = TWO_PI - 2.0 * JOINT_STOP_ARC


def capped_joint(joint):
    """The limits this module can actually enforce for one URDF joint.

    Returns (joint, capped, why). `joint` is the same dict when the range fits; otherwise a
    copy whose lower/upper are the URDF midpoint +/- `MAX_LIMITED_RANGE / 2`. Pure - no
    Fusion, no side effects, and it never touches `JOINT_STOP_DEG`."""
    lower, upper = joint.get('lower'), joint.get('upper')
    if lower is None or upper is None:
        return joint, False, ''
    rng = upper - lower
    if rng <= MAX_LIMITED_RANGE + LINK_STOP_B_TOL:
        return joint, False, ''
    mid = 0.5 * (lower + upper)
    out = dict(joint)
    out['lower'] = mid - MAX_LIMITED_RANGE / 2.0
    out['upper'] = mid + MAX_LIMITED_RANGE / 2.0
    why = ('%s: the URDF asks for %.5f deg of range and this module can limit at most '
           '%.5f deg (2*pi - 2 * JOINT_STOP_DEG %.1f), so the range is CAPPED to %.5f deg '
           'symmetrically about the midpoint %+.5f deg: %+.5f .. %+.5f deg instead of '
           '%+.5f .. %+.5f. %.5f deg of travel is given up at each end. JOINT_STOP_DEG is '
           'untouched; this joint would need %.4f deg to keep its whole range (EXO-218).'
           % (joint.get('name', '?'), math.degrees(rng), math.degrees(MAX_LIMITED_RANGE),
              JOINT_STOP_DEG, math.degrees(MAX_LIMITED_RANGE), math.degrees(mid),
              math.degrees(out['lower']), math.degrees(out['upper']),
              math.degrees(lower), math.degrees(upper),
              math.degrees(rng - MAX_LIMITED_RANGE) / 2.0,
              (360.0 - math.degrees(rng)) / 2.0))
    return out, True, why


def limited_joint(joint, axis_sign=1, pad_r=ROTOR_PAD_R, stop_h=STOP_H):
    """The link stop that gives one URDF joint exactly its own travel. Radians in, radians
    (and the degrees the sketch wants) out. Pure geometry - no Fusion, no route, no link.

    `joint` is a `parse_urdf` entry (needs 'lower' / 'upper'); `axis_sign` is the joint's
    module-frame axis sign. Returns a dict:
        arc            b, the link stop's full arc (rad)     half     b/2 (rad)
        centre         its centre in the module frame at q = 0 (rad, 0..2pi)
        stop_angle / stop_half / stop_h   the same thing in the units `stop_block_outline`
                       and BuildArmGen's LINKS take (deg, deg, mm)
        r_in / r_out   the block's radii
        travel         2*pi - a - b, which is exactly `upper - lower`
        contact_lower / contact_upper   the q at which each arc edge meets the joint stop
        zero_gap       the angular clearance between the two blocks at q = 0 (rad)
    Raises ValueError, by name and with numbers, for a joint with no <limit> and for one
    whose range leaves b < JOINT_STOP_ARC (EXO-208: the joint stop is the minimum)."""
    name = joint.get('name', '?')
    urdf_lower, urdf_upper = joint.get('lower'), joint.get('upper')
    if urdf_lower is None or urdf_upper is None:
        raise ValueError('%s has no <limit lower= upper=>: a joint with no range in the '
                         'URDF has no link stop to generate' % name)
    # EXO-218: a range wider than this module can limit is capped, symmetrically about the
    # midpoint, and the fact is carried on the spec so no report can lose it.
    joint, capped, cap_why = capped_joint(joint)
    lower, upper = joint['lower'], joint['upper']
    rng = upper - lower
    if rng <= 0.0:
        raise ValueError('%s has upper %.4f <= lower %.4f' % (name, upper, lower))
    b = TWO_PI - JOINT_STOP_ARC - rng
    # EXO-208: the joint stop is the MINIMUM and the link stop takes the rest, so the one
    # thing that can go wrong is the link stop coming out NARROWER than the joint stop.
    if b < JOINT_STOP_ARC - LINK_STOP_B_TOL:
        raise ValueError(
            '%s: range %.4f rad (%.3f deg) leaves a link stop of only %.4f rad (%.4f deg) '
            'beside the module\'s %.4f rad (%.4f deg) joint stop (JOINT_STOP_DEG = %.1f), '
            'and the joint stop is the minimum: b >= a. The widest range this module can '
            'limit is %.4f rad (%.4f deg); this joint would need JOINT_STOP_DEG <= %.4f.'
            % (name, rng, math.degrees(rng), b, math.degrees(b), JOINT_STOP_ARC,
               math.degrees(JOINT_STOP_ARC), JOINT_STOP_DEG,
               TWO_PI - 2.0 * JOINT_STOP_ARC,
               math.degrees(TWO_PI - 2.0 * JOINT_STOP_ARC),
               (360.0 - math.degrees(rng)) / 2.0))
    half_sum = (JOINT_STOP_ARC + b) / 2.0
    # delta = (link stop centre) - (joint stop centre), CCW in the module frame. The blocks
    # touch at |delta| = half_sum; the free region is delta in [half_sum, 2pi - half_sum],
    # which is exactly `rng` wide. delta = (centre - JOINT_STOP_MID) + axis_sign * q, so it
    # runs WITH q when axis_sign = +1 and AGAINST it when axis_sign = -1: the end of the
    # free region that q = lower sits at is the low one in the first case and the high one
    # in the second.
    delta_lower = half_sum if axis_sign > 0 else TWO_PI - half_sum
    centre = (JOINT_STOP_MID + delta_lower - axis_sign * lower) % TWO_PI
    spec = {
        'joint': name, 'axis_sign': 1 if axis_sign > 0 else -1,
        'lower': lower, 'upper': upper, 'range': rng,
        # EXO-218: what the URDF asked for, and whether this module could give it.
        'urdf_lower': urdf_lower, 'urdf_upper': urdf_upper,
        'urdf_range': urdf_upper - urdf_lower, 'capped': capped, 'cap_why': cap_why,
        'max_range': MAX_LIMITED_RANGE,
        'joint_stop_arc': JOINT_STOP_ARC, 'joint_stop_mid': JOINT_STOP_MID,
        'arc': b, 'half': b / 2.0, 'centre': centre, 'travel': TWO_PI - JOINT_STOP_ARC - b,
        # EXO-208: the floor IS the joint stop. b_margin < 0 means the link stop came out
        # narrower than the block it meets - inside B_TOL it is measurement noise (J1),
        # outside it the joint is refused above.
        'b_min': JOINT_STOP_ARC, 'b_margin': b - JOINT_STOP_ARC,
        'joint_stop_deg': JOINT_STOP_DEG,
        'stop_angle': math.degrees(centre), 'stop_half': math.degrees(b / 2.0),
        'stop_h': stop_h, 'r_in': STOP_R_IN, 'r_out': pad_r,
    }
    spec['zero_gap'] = stop_gap(spec, 0.0)
    spec['contact_lower'], spec['contact_upper'] = lower, upper
    return spec


def stop_gap(spec, q):
    """Signed angular gap (rad) between the link stop and the joint stop at URDF angle `q`:
    > 0 clear, 0 touching, < 0 the two blocks overlap. This is the ONE function the sweep
    and every test go through, so the rule is checked, not restated."""
    d = _wrap_pi(spec['centre'] + spec['axis_sign'] * q - spec['joint_stop_mid'])
    return abs(d) - (spec['joint_stop_arc'] + spec['arc']) / 2.0


def stop_sweep(spec, pad=math.radians(5.0), step=math.radians(0.002)):
    """Sweep q through the whole travel and find where the rotor FIRST touches the joint
    stop on each side, by bisection on `stop_gap` - never by re-deriving the placement.

    Starts at the middle of the nominal range (which must be clear) and walks out both ways
    in `step` until the gap goes negative, `pad` past each limit. Returns
    (q_lo, q_hi, samples) in radians; raises if the mid pose is already in contact."""
    mid = (spec['lower'] + spec['upper']) / 2.0
    if stop_gap(spec, mid) <= 0.0:
        raise ValueError('%s: the blocks already overlap at the middle of the range'
                         % spec['joint'])
    out, n = [], 0
    for direction in (-1.0, +1.0):
        far = (spec['lower'] - pad) if direction < 0 else (spec['upper'] + pad)
        q, hit = mid, None
        while (far - q) * direction > 0.0:
            q_next = q + direction * step
            n += 1
            if stop_gap(spec, q_next) <= 0.0:
                hit = (q, q_next)
                break
            q = q_next
        if hit is None:
            raise ValueError('%s: swept %.4f rad past the %s limit without touching the '
                             'joint stop' % (spec['joint'], pad,
                                             'lower' if direction < 0 else 'upper'))
        lo, hi = hit
        for _ in range(60):                       # bisect to well under 1e-9 rad
            m = (lo + hi) / 2.0
            if stop_gap(spec, m) > 0.0:
                lo = m
            else:
                hi = m
        out.append((lo + hi) / 2.0)
    return min(out), max(out), n


def rect_outline(x0, y0, x1, y1):
    segs = []
    _line(segs, (x0, y0), (x1, y0))
    _line(segs, (x1, y0), (x1, y1))
    _line(segs, (x1, y1), (x0, y1))
    _line(segs, (x0, y1), (x0, y0))
    return segs


def profile_closed(segs, tol=1e-6):
    """True if consecutive segments share endpoints and the loop closes."""
    for a, b in zip(segs, segs[1:] + segs[:1]):
        pa, pb = (a[2] if a[0] == 'line' else a[3]), b[1]
        if math.hypot(pa[0] - pb[0], pa[1] - pb[1]) > tol:
            return False
    return True


# ---------------- pair classification (EXO-81) ----------------
# Every link the router builds joins the ROTOR pad of joint N to the STATOR pad of joint
# N+1, and both G1 and Gento Luna put those two axes exactly perpendicular, with the
# stator axis lying IN the rotor pad plane (measured: tools/bridge_scripts/classify_pairs.py).
# `landing_numbers` reduces such a pair to the three numbers every pattern is chosen on:
#   s_land  reach   - how far, in the pad plane, the stator slab's free face is from axis A
#   zL      level   - how high above the rotor pad's rotor-side plane the nominal landing is
#   w_off   offset  - how far that landing sits off the u-z plane through axis A
# It assumes NOTHING and checks NOTHING: route() and the pattern builders do the checking.
PAIR_IN_PLANE_MIN = 1e-6    # below this the stator axis has no in-plane part at all and
                            # there is no bar direction to speak of (parallel axes)
# `y_sign` snaps the stator's connector side Y_B onto the rotor pad axis. That is only a
# SIGN when Y_B actually leans along the axis; G1's J3->J4 puts Y_B exactly IN the pad plane
# (|Y_B . Z_A| measures 1e-4 on the URDF frames and 0 on the exo's own `Module 3 R` pose),
# and there the sign was being read off rounding noise: 1e-4 the other way moves the nominal
# landing zL from 45.17 to 115.87 and the pair stops being the one EXO-88/EXO-97 tabulated.
# Below this threshold the connector side says nothing, so the sign comes from the pair's own
# geometry instead: the stator slab's tongue runs back TOWARD the rotor pad, i.e. against the
# pair's axial offset. Both built links are far above the threshold (0.96 and 1.00), so
# nothing they compute moves. `y_sign_from` records which rule decided (EXO-103).
YB_AXIAL_MIN = 0.1


def actuator_rect(m=ACT_OPEN_MARGIN):
    """The XL330's footprint in the module frame, grown by `m` mm all round (EXO-187).

    `CUTOUT_X` x `CUTOUT_Y` is the ACTUATOR, measured off `sensor module 2`. The default
    growth is `ACT_OPEN_MARGIN` = 0 (EXO-193): the hand-made links cut the opening to that
    rectangle exactly and the real XL330 was put through it - it fits, and a validated part
    outranks the argument (EXO-187) that the actuator is a module body owed MIN_CLEARANCE
    like any other. So the opening IS the footprint, and every number the landing is solved
    on - `cut_reach_clear`, `cut_web`, `cut_gap`, the ring bounds, the cut feature itself -
    is measured off this same rectangle at this same margin.

    What EXO-187 measured stands and is kept: the plate used to leave a 1 mm picture frame
    of material inside the actuator's box grown by MIN_CLEARANCE (896 mm3 on every link,
    `tools/bridge_scripts/measure_exo187.py`) because the cut only went to plate depth. The
    fix that survives is `actuator_cut_z` - the opening is cut THROUGH every solid in the
    footprint - not the growth.

    The cable leaves the two connectors on +Y. It leaves through this opening, which
    `actuator_cut_z` takes right THROUGH the plate 1 mm proud of the connector face at
    y = CUTOUT_Y[1]; no separate cable allowance is invented here, because no XL330 cable
    or connector-shell dimension is recorded anywhere in the repo. `sweepgen.actuator_check`
    MEASURES the room along +Y below the plate and reports it (EXO-192).
    """
    return (CUTOUT_X[0] - m, CUTOUT_X[1] + m, CUTOUT_Y[0] - m, CUTOUT_Y[1] + m)


def actuator_volume(m=ACT_OPEN_MARGIN):
    """`actuator_rect` plus the z span it is excluded over: (x0, x1, y0, y1, z0, z1).

    Down to the actuator's own back at `ACT_Z[0]`, grown by `m`; capped ABOVE at the stator
    underside, because above that plane the volume is the stator's own solid - the module's
    business, not the link's, and `check_interference` already sees it."""
    x0, x1, y0, y1 = actuator_rect(m)
    return (x0, x1, y0, y1, ACT_Z[0] - m, STATOR_UNDERSIDE_Z)


def actuator_cut_z(m=ACT_OPEN_MARGIN, eps=0.5):
    """(centre, length) in module z of the cut that clears the volume, for a SYMMETRIC
    extrude. It spans the whole exclusion volume, not just the plate: EXO-187's rule is
    that the opening is cut THROUGH all link material in the footprint - ribbon, fillet
    collar, anything - not to a fixed plate depth."""
    z0, z1 = ACT_Z[0] - m, STATOR_UNDERSIDE_Z
    return ((z0 + z1) / 2.0, (z1 - z0) + 2.0 * eps)


def actuator_box_dist(p, m=ACT_OPEN_MARGIN):
    """Signed distance (mm) from a point in the MODULE frame to the exclusion volume:
    >= 0 outside (the true Euclidean distance), negative inside (minus the depth of the
    shallowest face it is inside of)."""
    x0, x1, y0, y1, z0, z1 = actuator_volume(m)
    lo, hi = (x0, y0, z0), (x1, y1, z1)
    out = [max(lo[i] - p[i], 0.0, p[i] - hi[i]) for i in range(3)]
    d = math.sqrt(sum(v * v for v in out))
    if d > 0.0:
        return d
    return -min(min(p[i] - lo[i], hi[i] - p[i]) for i in range(3))


def in_actuator_rect(p, m=ACT_OPEN_MARGIN):
    """Is a MODULE-frame point inside the footprint the opening is cut to, over the z span
    the cut spans? That material is not there: the cut removed it."""
    x0, x1, y0, y1, z0, z1 = actuator_volume(m)
    return x0 <= p[0] <= x1 and y0 <= p[1] <= y1 and p[2] <= z1


def _cutout_reach(xb, yb, y_s, m=0.0):
    """How far the actuator footprint reaches along the slab-frame direction `y_s`.

    The actuator is fixed in the TRUE frame B, so it turns with the module about its own
    axis while the slab body stays put. Every route that lands on the slab measures this
    instead of assuming `CUTOUT_Y` (EXO-130).

    `m` grows the rectangle (`actuator_rect`). It DEFAULTS TO 0 - the XL330's own edge -
    because that is the datum every measured constant in this module was read off
    (`BAR_CLEAR` 6.85 and `L_CUT_CLEAR` are webs from the raw cutout edge on the hand-built
    links, and EXO-130 read `CUTOUT_Y[1]` off `Link 2 R`). Since EXO-193 the OPENING is that
    same rectangle (`ACT_OPEN_MARGIN` 0), so `cut_reach` and `cut_reach_clear` coincide;
    the two names are kept because the second is the one the landing solve reads and the
    margin lives in exactly one constant."""
    x0, x1, y0, y1 = actuator_rect(m)
    cx, cy = dot(xb, y_s), dot(yb, y_s)
    return max(vx * cx + vy * cy for vx in (x0, x1) for vy in (y0, y1))


def cutout_reach(num, y_s, m=0.0):
    """`_cutout_reach` for a pair already reduced by `landing_numbers`, along any slab-frame
    direction (the ramp draws its slab turned 180 deg from the snapped frame, so it asks for
    its own). `m` as in `_cutout_reach`: 0 is the actuator's own edge."""
    return _cutout_reach(num['x_b'], num['y_b'], y_s, m)


def landing_numbers(R_ab, t_ab):
    """Classification numbers of one joint pair, in frame A (mm). `R_ab`, `t_ab` = the
    stator frame B in the rotor pad frame A (relative_frame). Returns a dict with
    axis_angle_deg, dist, axial, lateral, zb_in_plane, yb_axial and - when the stator
    axis has an in-plane part - the link frame (u, w), the snapped stator frame
    (R_b, snap_rad, y_sign, u_sign), the nominal landing L and s_land / w_off / zL.

    The snapped frame B' is B turned about its own axis until its connector side Y' runs
    along the pad axis and its axis Z' lies in the pad plane: that is the frame in which
    the slab BODY (disk, tongue, blends) lines up with the link, while the insert holes
    and the actuator cutout stay in the true B where they mate (EXO-61). For the bar the
    turn is rounding noise (BAR_SNAP_MAX); for the shoulder L it is the real tilt of the
    pad axis (16 deg on the G1), which is why the L checks the cutout in the true frame."""
    xb = [R_ab[i][0] for i in range(3)]
    yb = [R_ab[i][1] for i in range(3)]
    zb = [R_ab[i][2] for i in range(3)]
    zb_ip = [zb[0], zb[1], 0.0]
    n_ip = math.sqrt(dot(zb_ip, zb_ip))
    out = {
        'axis_angle_deg': math.degrees(math.acos(max(-1.0, min(1.0, abs(zb[2]))))),
        'dist': math.sqrt(dot(t_ab, t_ab)),
        'axial': t_ab[2],
        'lateral': math.hypot(t_ab[0], t_ab[1]),
        'zb_in_plane': n_ip,
        'yb_axial': abs(yb[2]),
        'x_b': xb, 'y_b': yb, 'z_b': zb,
        't_ab': list(t_ab),          # the pair's own offset, carried so that a route sized
                                     # from `num` alone can check module B (EXO-96)
    }
    if n_ip < PAIR_IN_PLANE_MIN:
        return out
    z_s = unit(zb_ip)
    if abs(yb[2]) >= YB_AXIAL_MIN:
        y_sign, y_sign_from = (1.0 if yb[2] > 0 else -1.0), 'connector side Y_B'
    else:
        # Y_B lies in the pad plane and cannot be snapped onto the axis by a sign; the
        # tongue runs from the slab back down to the rotor pad (EXO-103)
        y_sign, y_sign_from = (-1.0 if t_ab[2] > 0 else 1.0), 'the pair axial offset'
    y_s = [0.0, 0.0, y_sign]
    x_s = cross(y_s, z_s)
    # Two different things used to be one number. `snap_axis_rad` is how far the stator AXIS
    # is off square to the pad - real geometry, and what a route that draws the tongue walls
    # and the band walls as the same planes cannot tolerate. `spin_rad` is how far the module
    # is turned about its OWN axis away from the pad axis - the free parameter EXO-130 solves,
    # which moves the inserts and the cutout under the landing and nothing else. `snap_rad`
    # is kept as their max, which is what it always was.
    snap_axis_rad = math.acos(max(-1.0, min(1.0, dot(zb, z_s))))
    spin_rad = math.acos(max(-1.0, min(1.0, dot(yb, y_s))))
    snap_rad = max(snap_axis_rad, spin_rad)
    # How far the actuator reaches along the snapped frame's +Y is what a landing has to
    # stay clear of, and it turns with the module: at `spin_rad` 0 it is CUTOUT_Y[1]
    # exactly, which is what EXO-130 read off `Link 2 R`. `cut_reach` is the XL330's OWN
    # edge; `cut_reach_clear` is the edge of the OPENING cut for it, `ACT_OPEN_MARGIN`
    # proud all round - 0 since EXO-193, so the two are equal. Old routes read the first,
    # anything sizing a landing reads the second, and the margin moves in one place.
    cut_reach = _cutout_reach(xb, yb, y_s)
    cut_reach_clear = _cutout_reach(xb, yb, y_s, ACT_OPEN_MARGIN)
    # `y_land` is NOT sized off the grown rectangle. `BAR_CLEAR` was measured on `Link 2 R`
    # as raw-cutout-edge -> bar face (CUTOUT_Y[1] 24.5 + 6.85 + PAD_T/2 = 35.35, the bar
    # centre the hand-built link really has), so growing the rect here would silently move
    # every landing 1 mm further out and drop zL below L_LEVEL_TOL (EXO-187: link_12 went
    # to zL 1.65 and route_path refused it). The MIN_CLEARANCE the opening owes the
    # actuator is enforced where it belongs - in `cut_reach` / `LAND_WEB`, which are
    # measured off `actuator_rect` and so bite 1 mm EARLIER, the conservative direction.
    y_land = CUTOUT_Y[1] + BAR_CLEAR + PAD_T / 2   # nominal landing along +Y_B (bar)
    z_free = STATOR_UNDERSIDE_Z - PAD_T              # slab face away from the stator module
    L = [t_ab[i] + y_land * y_s[i] + z_free * z_s[i] for i in range(3)]
    L_ip = [L[0], L[1], 0.0]
    u_sign = 1.0 if dot(z_s, L_ip) >= 0 else -1.0
    u = [u_sign * v for v in z_s]
    w = unit(cross([0.0, 0.0, 1.0], u))
    out.update({
        'z_s': z_s, 'y_s': y_s, 'x_s': x_s, 'y_sign': y_sign, 'u_sign': u_sign,
        'y_sign_from': y_sign_from, 'cut_reach': cut_reach,
        'cut_reach_clear': cut_reach_clear,
        'snap_axis_rad': snap_axis_rad, 'spin_rad': spin_rad,
        'R_b': [[x_s[i], y_s[i], z_s[i]] for i in range(3)], 'snap_rad': snap_rad,
        'u': u, 'w': w, 'landing': L, 'y_land': y_land, 'z_free': z_free,
        's_land': dot(L_ip, u), 'w_off': dot(L_ip, w), 'zL': L[2],
        # the two faces of the stator slab along u: `s_near` faces axis A, `s_far_face`
        # faces module B. (For the bar u runs with Z_B and the near face is the landing;
        # for the shoulder L u runs against it and the landing is the far one.)
        's_slab': (min(dot(L_ip, u), dot(L_ip, u) + u_sign * PAD_T),
                   max(dot(L_ip, u), dot(L_ip, u) + u_sign * PAD_T)),
        # centre of the stator slab disk in (s, w, z)
        'slab_centre': (dot([t_ab[0], t_ab[1], 0.0], u) + (STATOR_UNDERSIDE_Z - PAD_T / 2) * u_sign,
                        dot([t_ab[0], t_ab[1], 0.0], w), t_ab[2]),
    })
    return out


def _axial_route(num, pad_r, width, placement=None):
    """Size the route that leaves the rotor pad ALONG THE PAD AXIS (the 'bar', link_23):
    ledge join, a riser run down to the bar's level, one 90 degree bend in the side plane and
    a long run out to the stator slab, with the pair's lateral offset taken as an S inside a
    straight run. Every check and every number is EXO-58 / EXO-74 / EXO-83's, unchanged; only
    the shape it produces is now data. Raises ValueError naming the number that failed."""
    placement = S_PLACEMENT if placement is None else placement
    if placement not in S_PLACEMENTS:
        raise ValueError('unknown S placement %r (expected one of %s)'
                         % (placement, ', '.join(S_PLACEMENTS)))
    if num['zb_in_plane'] < BAR_SQUARE_MIN:
        raise ValueError('stator axis B is not in the rotor-pad plane '
                         '(in-plane part %.2f, need > %.2f)' % (num['zb_in_plane'], BAR_SQUARE_MIN))
    if num['snap_axis_rad'] > BAR_SNAP_MAX:
        raise ValueError('stator AXIS is %.2e rad off square to the pad (max %.0e); this '
                         'route draws the tongue walls and the band walls as the same '
                         'planes' % (num['snap_axis_rad'], BAR_SNAP_MAX))
    u, w = num['u'], num['w']
    s_land, w_off, zL = num['s_land'], num['w_off'], num['zL']
    half_w = width / 2.0
    if s_land < pad_r + 2 * PAD_T:
        raise ValueError('landing point only %.1f mm from axis A along the run (need > %.1f)'
                         % (s_land, pad_r + 2 * PAD_T))
    # the run's inner boundary: outside module A's stator AND the previous link's slab
    r_keep = max(MODULE_R, STATOR_PAD_R)
    s_in = max(pad_r, r_keep) + RISER_CLEAR
    clearance_a = s_in - r_keep
    if clearance_a < MIN_CLEARANCE:
        raise ValueError('riser inner face at s=%.2f is only %.2f mm outside the module/slab '
                         'radius %.1f (need >= %.1f)'
                         % (s_in, clearance_a, r_keep, MIN_CLEARANCE))
    s_out = s_in + RISER_T
    z_top, z_bot = zL + BAR_T / 2, zL - BAR_T / 2
    # the profile root must be hidden inside the rotor pad across the whole band. The band
    # leaves the pad CENTRED on axis A (EXO-64: the lateral offset is absorbed further out),
    # so the root only has to cover `width`.
    if half_w >= pad_r:
        raise ValueError('the %.0f mm wide root would leave the r %.1f pad' % (width, pad_r))
    root_s = min(PROFILE_ROOT_S, math.sqrt(pad_r ** 2 - half_w ** 2) - 1.0)
    if root_s < 5.0:
        raise ValueError('profile root at s=%.1f is too close to axis A' % root_s)
    s_end = s_land + LAND_BURY_HALF
    # the S goes in a straight run: it must be finished where the bar top becomes straight
    # (the end of the bend's left fillet) at the 'riser' placement, or sit inside the
    # straight bar between there and the slab at the 'bar' placement (EXO-69).
    s_bend_end = s_out + FILLET_OUT_BOT
    if s_bend_end > s_land - FILLET_FLANGE:
        raise ValueError('the bend ends at s=%.2f, past the flange fillet at %.2f'
                         % (s_bend_end, s_land - FILLET_FLANGE))
    r_s = s_radius(width)
    cap = (root_s - 2.0, s_end + 2.0)
    if placement == 'riser':
        s_curve = riser_s_curve(w_off, s_bend_end, cap[0], cap[1], pad_r, half_w, r_s)
    else:
        s_curve = bar_s_curve(w_off, s_bend_end, s_land, cap[0], cap[1], pad_r, half_w, r_s)
    if s_curve is not None:
        s_curve['placement'] = placement
        s_curve['wide_w'] = 2.0 * (abs(w_off) + half_w) + 2.0    # side-profile extrude,
        s_curve['z_span'] = (PAD_T + 2.0) - (z_bot - 2.0)        # symmetric about w = 0
        s_curve['z_mid'] = ((PAD_T + 2.0) + (z_bot - 2.0)) / 2.0 # plan extrude, about z
    # the stator slab: the tongue is centred on the stator (the S brought the band there)
    y_land = num['y_land']
    y_end = y_land + BAR_T / 2
    # EXO-130. `y_land` is `Link 2 R`'s measured landing, which is taken past the actuator
    # cutout on the CONNECTOR side. The cutout turns with the module about its own axis, so
    # where it actually reaches is a number to CHECK, not a constant to assume: this route is
    # the one that lands on the connector side and it is therefore the one whose module the
    # rotation solve cannot turn far. `BAR_CLEAR` is the web measured on the hand-built part.
    cut_web = (y_end - BAR_T) - num['cut_reach']
    if cut_web < BAR_CLEAR:
        raise ValueError('only %.2f mm of slab between the actuator cutout (reaching y=%.2f '
                         'in the slab frame, the module turned %.1f deg about its own axis) '
                         'and the bar\'s near face (y=%.2f); need >= %.2f'
                         % (cut_web, num['cut_reach'], math.degrees(num['spin_rad']),
                            y_end - BAR_T, BAR_CLEAR))
    edges = flange_edges(y_end, 0.0, half_w, STATOR_PAD_R)
    flange, y_blend, _t = flange_outline(y_end, half_w, STATOR_PAD_R, edges=edges)
    # EXO-124: the same solve as the other two routes. The bar dies HALF way into the slab,
    # so only its near face carries the run; the tongue's end edge is rounded at the far one
    # either way, and both radii are measured against the insert holes.
    inserts = insert_slab_points(num, num['x_s'], num['y_s'], num['z_s'])
    x_band = (-half_w - TONGUE_ROUND_MARGIN, half_w + TONGUE_ROUND_MARGIN)
    r_flange, land_tight = land_radius_fit(FILLET_FLANGE, y_end, z_top - z_bot, inserts,
                                           x_band[0], x_band[1], 'land fillet')
    r_tongue, corner_tight = land_radius_fit(TONGUE_END_R, y_end, 0.0, inserts,
                                             x_band[0], x_band[1], 'tongue end round')
    # the corner round's cut is a little wider than the tongue; the blends next to the tongue
    # must already have receded below the round there
    y_blend_edge = y_blend - math.sqrt(max(0.0, FLANGE_BLEND_R ** 2
                                           - (FLANGE_BLEND_R - TONGUE_ROUND_MARGIN) ** 2))
    if y_blend_edge >= y_end - r_tongue:
        raise ValueError('the corner round cut (%.1f wide) would clip the slab blends (blend '
                         'at y=%.2f, round from y=%.2f)'
                         % (2 * (half_w + TONGUE_ROUND_MARGIN), y_blend_edge, y_end - r_tongue))
    # the riser must stay clear of module B's stator cylinder (axis through t_ab along zb)
    t_ab, zb = num['t_ab'], num['z_s']
    riser_c = [u[i] * (s_in + RISER_T / 2) + w[i] * w_off for i in range(3)]
    riser_c[2] = (min(z_bot, 0.0) + max(z_top, PAD_T)) / 2
    d = [riser_c[i] - t_ab[i] for i in range(3)]
    along = dot(d, zb)
    perp = math.sqrt(max(0.0, dot(d, d) - along ** 2))
    if STATOR_UNDERSIDE_Z - PAD_T <= along <= 0.0 and perp < MODULE_R + half_w:
        raise ValueError('the riser would intersect module B (perp %.1f mm)' % perp)
    s_c, z_c = (s_in + s_out) / 2.0, (z_top + z_bot) / 2.0
    route = make_route(
        width, (s_c, PAD_T / 2.0), (0.0, -1.0),
        [Run(PAD_T / 2.0 - z_c, s_out - s_in, 'riser'),
         Bend(90.0, 'side', FILLET_OUT_BOT, FILLET_IN_BOT, 'bend'),
         Run(s_land - s_c, z_top - z_bot, 'bar')],
        (rotor_join('ledge', root_s, pad_r),
         stator_land(s_land, r_flange, PROFILE_BURY, PROFILE_BURY_UP, LAND_BURY_HALF,
                     r_tongue, 2 * (half_w + TONGUE_ROUND_MARGIN),
                     fillet_r_nominal=FILLET_FLANGE, corner_r_nominal=TONGUE_END_R,
                     flange=flange, y_end=y_end, y_blend=y_blend, edges=edges)),
        pattern='bar', departure='axial', s_curve=s_curve, placement=placement,
        numbers=num, pad_r=pad_r, u=u, w=w, w_off=w_off, zL=zL,
        extra={'s_in': s_in, 's_out': s_out, 's_end': s_end, 's_bend_end': s_bend_end,
               'clearance_a': clearance_a, 'y_land': y_land, 'flange_y': (0.0, y_end),
               'land_tight': None if land_tight is None else land_tight['angle'],
               'corner_tight': None if corner_tight is None else corner_tight['angle'],
               'tongue_round_w': 2 * (half_w + TONGUE_ROUND_MARGIN),
               'ext_s': (root_s, s_out), 'riser_s': (s_in, s_out),
               'riser_z': (min(z_bot, 0.0), max(z_top, PAD_T)),
               'bar_s': (s_in, s_end), 'bar_z': (z_bot, z_top), 'blend_r': tuple(
                   e['r'] for e in edges)})
    return route


def link_bar_geometry(R_ab, t_ab, pad_r, placement=None, width=None):
    """Thin wrapper: the route that leaves the rotor pad along its axis, drawn. Kept because
    the measured-body checks and the tests name it; `route`/`route_path` is the entry point
    that CHOOSES (EXO-96). Raises ValueError when that route does not size."""
    return render_route(_axial_route(landing_numbers(R_ab, t_ab), pad_r,
                                    BAR_W if width is None else width, placement))




# ---------------- 'L' body: the shoulder link (measured from Link 1 R = link 12, EXO-89) ----------------
# When the stator slab's nominal landing falls inside the rotor pad's OWN thickness there is
# no room for a riser, a bend or a bar: the link is a plate COPLANAR with the rotor pad,
# running straight out to the slab, which stands on it at right angles. That is exactly what
# the hand-built Link 1 R is - an 8 mm plate at z 0..8, an r 3 concave fillet from its top
# face into the slab's near face and an r 10 convex round from its rotor-side face onto the
# slab's far face, the only two arcs in its whole side elevation.
#
# The rotor-pad join is then the EXO-74 standard for free: the plate IS the pad continued
# outward, so both pad faces run on flat, the rim is covered over the pad's whole thickness
# across the band, and NOTHING goes below z = 0 - no pocket, no ledge, no relief, and
# nothing that can reach the stator stop block's annulus (MODULE_STOP_Z_TOP).
#
# The lateral offset cannot be taken by a plan-view S here: on the G1 shoulder w_off is
# 13.83 mm and `s_curve_run` wants 27.9 mm of run at S_RADIUS, while the plate is only about
# 24.5 mm long (the fit is computed and reported in `s_fit`). It is taken in the SLAB
# instead - the tongue sits off-centre on the stator disk (flange_outline's x_off) - which
# keeps the rotor pad join centred and symmetric, the one Esteban inspects.
# Width. The bar necks down to BAR_W = 25; the shoulder L does NOT (Esteban, 2026-09-15):
# it is the most loaded and the shortest link, and the hand-built Link 1 R's plate is the
# full rotor-pad diameter, 52 mm (EXO-90). L_W is therefore its own constant. At exactly
# 2 x pad_r the plate's two side walls are TANGENT to the pad rim, which is what Link 1 R
# does (its plate measures w -26..26 on a r 26 pad), and the profile root then sits on the
# pad's own diameter, s = 0. A plate that wide overhangs the stator disk on the far side,
# so that side of the tongue cannot blend into the rim - it uses Link 1 R's toe arc instead
# (flange_toe).
L_W = 52.0
L_FILLET_IN = 3.0       # concave fillet: plate top face -> slab near face (Link 1 R, exact)
L_ROUND_OUT = 10.0      # convex round: plate rotor-side face -> slab far face (Link 1 R, exact).
                        # Capped so its tangent point stays JOIN_RIM_MARGIN outside the pad rim
                        # and the rotor-side face crosses the rim FLAT (EXO-74): on the G1
                        # shoulder that cap is 32.5 - 26 - 1 = 5.5. The hand-built r 10 has its
                        # tangent point at s 22.5, 3.5 mm INSIDE the rim, and leaves a 0.632 mm
                        # uncovered strip of rim under the plate. The rim rule wins.
L_ROUND_MIN = 2.0       # below this the elbow's outer corner is effectively sharp
L_BURY_UP = 1.0         # the side profile is buried this far past the fillet, inside the tongue
L_LEVEL_TOL = 2.0       # |zL - PAD_T/2| below this is 'the landing is in the pad's own plane'
                        # (G1 J1->J2 and J5->J6 both measure 1.35)
L_CUT_CLEAR = 2.0       # min slab web between the actuator cutout and the plate's outer face,
                        # measured in the snapped frame the tongue is drawn in. The G1 shoulder
                        # measures 3.69; the bar's equivalent is BAR_CLEAR = 6.85.


# ---------------- the 'ramp': the in-plane departure that climbs (measured from Link 4 R,
# EXO-114) ----------------
# The J4->J5 pair puts the stator slab 67.5 mm out and only 1.89 mm off the rotor pad's own
# level, edge on, with its actuator cutout straddling the pad plane (the cutout reaches 24.5
# mm one way and 9.5 the other from the disk centre). A plate straight out would run into the
# cutout; the bar route's S does not fit (EXO-95). `Link 4 R` - measured before anything was
# written - answers with neither: it leaves the pad COPLANAR, as Link 1 R and Link 3 R do,
# CLIMBS over the cutout through an S in the SIDE plane, and lands with the run's centre line
# exactly ON the stator disk's rim, on the side away from the connector (Y_B = -27, where the
# cutout only reaches -9.5, so the run clears it by 13.5 mm).
#
# Measured on Link 4 R in the J4 rotor-pad frame (tools/bridge_scripts/measure_link4_profile.py):
#   plate z 0..6 coplanar with the pad; both S boundaries r 20.000 (lower arcs centred
#   (16, 20) and (53.428, 5.889), upper (12.573, 26) and (50.0, 11.889)), turning 69.3396 deg
#   and back, rising 25.887 mm over 37.428 mm of reach; bar z 25.889..31.889; land fillet
#   r 5.000 at (64.5, 20.889) into the slab's near face s 69.5 and corner round r 5.000 at
#   (70.5, 26.889) on its far face s 75.5, i.e. the run passes THROUGH the slab as the
#   shoulder plate does. The land fillet is on the run's UNDERSIDE and the corner round on
#   top - the mirror of the shoulder L, because here the slab hangs BELOW the landing.
RAMP_RISE_R = 20.0       # both boundary radii of the side-plane S (Link 4 R, measured)
RAMP_FILLET = JOIN_ROUND_R   # 5.0: Link 4 R's land fillet AND its far-face corner round are
                             # both r 5, the smallest radius already in the profile (EXO-84)
RAMP_LAND_MARGIN = 1.0   # straight run kept each side of the S inside its window
RAMP_W = 44.0            # Link 4 R's measured width, and the widest band that stays inside
                         # both rims of its pair (27 + 27 - 10). Width is still Esteban's
                         # input per link (EXO-101); this is only the survey fallback.


# The lateral offset, the third time. A ledge join takes it in an S-bend and a coplanar join
# whose tongue runs along the pad AXIS takes it in the slab (EXO-92/EXO-101). `Link 4 R` does
# NEITHER: its band is TAPERED in plan so that each wall is drawn tangent to the rim it has
# to stay inside - the far wall straight on the rotor pad's own tangent line w = -pad_r, the
# near wall on the stator disk's tangent line w = w_off + STATOR_PAD_R, led off the rotor rim
# by one big arc EXTERNALLY tangent to it (measured r 50.000, centre (37.945, 67.001),
# |centre| = 77.000 = 27 + 50, rim tangent (13.306, 23.494) - the same construction as Link
# 1 R's r 500 toe, in the plan plane instead of the slab plane). The offset is therefore
# SPLIT: the band sits w_off/2 off centre on BOTH pads (measured -5.000 against a -10.000
# pair) instead of all of it at one end, and the band is |w_off| narrower than the pads.
PLAN_TAPER_R = 50.0      # Link 4 R, measured; free as FLANGE_TOE_R is, and kept as measured
PLAN_TAPER_MARGIN = 1.0  # straight wall left between the taper's tangent point and the slab


def plan_taper_limits(w_off, pad_r, stator_pad_r=STATOR_PAD_R):
    """The band a tapered plan outline may occupy: (w_lo, w_hi, centre, max width). Each wall
    must stay inside BOTH rims - the rotor pad's (radius `pad_r` about w = 0) and the stator
    disk's (radius `stator_pad_r` about w = w_off) - so the band is the two disks' tangent
    strips intersected. At equal radii that makes the widest band `pad_r + stator_pad_r -
    |w_off|` wide and centred on w_off / 2, which is exactly Link 4 R's 44.000 at -5.000."""
    w_lo = max(-pad_r, w_off - stator_pad_r)
    w_hi = min(pad_r, w_off + stator_pad_r)
    return w_lo, w_hi, (w_lo + w_hi) / 2.0, w_hi - w_lo


def plan_taper(w_off, width, pad_r, cap, stator_pad_r=STATOR_PAD_R, taper_r=PLAN_TAPER_R,
               s_wall_end=None):
    """The plan-view (s, w) band of a TAPERED route: `width` across, placed by
    `plan_taper_limits`, each wall either straight on the rotor rim's own tangent line or led
    off that rim by an arc of radius `taper_r` externally tangent to it (EXO-114).

    `cap` = (cap_lo, cap_hi) closes the band short of / past the body, exactly as
    `riser_s_curve`'s does; `s_wall_end` (the slab's near face) is where each wall must
    already be straight, with PLAN_TAPER_MARGIN to spare. Returns a dict with w_lo / w_hi /
    centre / walls / band / w_range, or raises ValueError naming the wall that does not fit."""
    cap_lo, cap_hi = cap
    lo_lim, hi_lim, centre, w_max = plan_taper_limits(w_off, pad_r, stator_pad_r)
    if width > w_max + 1e-9:
        raise ValueError('a %.2f mm band cannot stay inside both rims: the r %.1f rotor pad '
                         'and the r %.1f stator disk %.2f mm off it leave w %.2f..%.2f, '
                         '%.2f mm across' % (width, pad_r, stator_pad_r, w_off, lo_lim,
                                             hi_lim, w_max))
    if abs(cap_lo) >= pad_r:
        raise ValueError('the plan band is capped at s=%.2f, outside the r %.1f rotor pad '
                         'rim it is drawn tangent to' % (cap_lo, pad_r))
    half = width / 2.0
    w_lo, w_hi = centre - half, centre + half
    walls = {}
    for sign, w_wall in ((+1, w_hi), (-1, w_lo)):
        if abs(abs(w_wall) - pad_r) <= FLANGE_TANGENT_TOL:
            walls[sign] = {'w': w_wall, 'kind': 'tangent'}
            continue
        if abs(w_wall) > pad_r:
            raise ValueError('the %+d wall at w=%.2f is outside the r %.1f rotor pad rim'
                             % (sign, w_wall, pad_r))
        cw = w_wall + sign * taper_r
        rr = pad_r + taper_r
        if abs(cw) >= rr:
            raise ValueError('a r %.1f plan taper at w=%.2f cannot reach the r %.1f rotor rim'
                             % (taper_r, w_wall, pad_r))
        s_tan = math.sqrt(rr ** 2 - cw ** 2)
        t = (pad_r * s_tan / rr, pad_r * cw / rr)
        if s_wall_end is not None and s_tan > s_wall_end - PLAN_TAPER_MARGIN:
            raise ValueError('the r %.1f plan taper meets the %+d wall at s=%.2f, past the '
                             'slab face at s=%.2f (need %.1f mm of straight wall)'
                             % (taper_r, sign, s_tan, s_wall_end, PLAN_TAPER_MARGIN))
        if t[0] <= cap_lo:
            raise ValueError('the r %.1f plan taper touches the rotor rim at s=%.2f, behind '
                             'the band cap at s=%.2f' % (taper_r, t[0], cap_lo))
        walls[sign] = {'w': w_wall, 'kind': 'taper', 'r': taper_r, 'c': (s_tan, cw),
                       't': t, 's_tan': s_tan}

    def wall_out(sign):
        """The wall walked from the rotor pad outward, cap_lo -> cap_hi."""
        segs, ww = [], walls[sign]
        if ww['kind'] == 'tangent':
            _line(segs, (cap_lo, ww['w']), (cap_hi, ww['w']))
            return segs
        _arc(segs, (0.0, 0.0), pad_r, _ang((0.0, 0.0), (cap_lo, sign * math.sqrt(
            max(0.0, pad_r ** 2 - cap_lo ** 2)))), _ang((0.0, 0.0), ww['t']))
        _arc(segs, ww['c'], ww['r'], _ang(ww['c'], ww['t']), _ang(ww['c'], (ww['s_tan'],
                                                                           ww['w'])))
        _line(segs, (ww['s_tan'], ww['w']), (cap_hi, ww['w']))
        return segs

    def wall_start(sign):
        ww = walls[sign]
        return (cap_lo, ww['w']) if ww['kind'] == 'tangent' else (
            cap_lo, sign * math.sqrt(max(0.0, pad_r ** 2 - cap_lo ** 2)))

    wall_lo, wall_hi = wall_out(-1), wall_out(+1)
    band = list(wall_lo)
    _line(band, (cap_hi, w_lo), (cap_hi, w_hi))
    band += _rev_segs(wall_hi)
    _line(band, wall_start(+1), wall_start(-1))
    if not profile_closed(band):
        raise ValueError('plan taper band outline does not close')
    ws = [q[1] for seg in band for q in _seg_points(seg, 0.5)]
    return {'w_lo': w_lo, 'w_hi': w_hi, 'centre': centre, 'width': width,
            'w_max': w_max, 'limits': (lo_lim, hi_lim), 'walls': walls, 'band': band,
            'wall_lo': wall_lo, 'wall_hi': wall_hi,
            'w_off': w_off, 'pad_r': pad_r, 'taper_r': taper_r, 'cap': (cap_lo, cap_hi),
            'w_range': (min(ws), max(ws))}


def taper_w(tp, s):
    """(w_lo, w_hi) of a tapered band at station s, or None outside its caps - `s_curve_w`
    for the other plan-view device."""
    if tp is None:
        return None
    lo, hi = tp['cap']
    if s < lo or s > hi:
        return None
    return (_chain_w(tp['wall_lo'], s), _chain_w(tp['wall_hi'], s))


def l_side_profile(s_lo, s_hi, root_s, pad_t=PAD_T, r_in=L_FILLET_IN, r_out=L_ROUND_OUT,
                   bury_up=L_BURY_UP, pad_r=None):
    """The shoulder plate's side profile, as a ROUTE (EXO-96): a coplanar join and one run of
    thickness pad_t straight out to the stator slab, which it runs THROUGH (its far boundary
    is the pad's own rotor-side plane, and the elbow's outer round is cut on it - a join
    fills that corner whatever the profile draws). Same arguments and same loud failures as
    the hand-written profile it replaced, drawn by the one renderer."""
    return route_profile(make_route(
        L_W, (root_s, pad_t / 2.0), (1.0, 0.0),
        [Run(s_lo - root_s, pad_t, 'plate')],
        (rotor_join('coplanar', root_s, pad_r, pad_t),
         stator_land(s_lo, r_in, 0.0, bury_up, s_hi - s_lo, r_out,
                     2.0 * (STATOR_PAD_R + TONGUE_ROUND_MARGIN), s_far=s_hi)),
        pattern='L'))


def _inplane_route(num, pad_r, width, placement=None):
    """Size the route that leaves the rotor pad IN ITS OWN PLANE (the shoulder 'L', link_12):
    coplanar join, one run straight out to the stator slab, which stands on it at right
    angles. EXO-92's geometry and judgment calls, unchanged; the lateral offset is taken in
    the slab because a coplanar join cannot carry an S (JOIN_TAKES_OFFSET)."""
    if num['zb_in_plane'] < BAR_SQUARE_MIN:
        raise ValueError('stator axis B is not in the rotor-pad plane '
                         '(in-plane part %.2f, need > %.2f)' % (num['zb_in_plane'], BAR_SQUARE_MIN))
    zL, axial, y_sign = num['zL'], num['axial'], num['y_sign']
    level = zL - PAD_T / 2.0
    if level < -L_LEVEL_TOL:
        raise ValueError('the landing is %.2f mm BELOW the rotor pad plate (zL %.2f vs the '
                         'plate mid-plane %.2f, max %.1f down): the slab would stand under '
                         'the plate, which no exo link does'
                         % (level, zL, PAD_T / 2.0, L_LEVEL_TOL))
    # EXO-103. A landing inside the pad's own thickness (Link 1 R, |level| <= L_LEVEL_TOL) is
    # an elbow: the tongue barely leaves the disk. A landing well above it (Link 3 R, level
    # 41.17) is the SAME route with a COLUMN - the tongue runs on down the slab's own plane to
    # the plate, so the link is the pad plane turning once into the slab plane. Nothing in the
    # side profile changes; what changes is how long the tongue is, and therefore how its
    # overhanging wall reaches the disk (`overhang`).
    column = level > L_LEVEL_TOL
    overhang = 'crown' if column else 'toe'
    if y_sign > 0:
        raise ValueError('the stator connector side points away from the rotor pad '
                         '(Y_B . Z_A = +1), so the slab would stand below the plate; only the '
                         'slab-above shoulder has been measured (Link 1 R, EXO-89)')
    u, w, w_off = num['u'], num['w'], num['w_off']
    s_lo, s_hi = num['s_slab']
    half_w = width / 2.0
    # the stator slab disk and module B both stand clear of the plate's top face
    over = axial - max(STATOR_PAD_R, MODULE_R) - PAD_T
    if over < MIN_CLEARANCE:
        raise ValueError('module B reaches z=%.2f, only %.2f mm over the plate top z=%.1f '
                         '(need >= %.1f)'
                         % (axial - max(STATOR_PAD_R, MODULE_R), over, PAD_T, MIN_CLEARANCE))
    # the profile root must not stick out of the rotor pad anywhere across the band; at
    # width = 2 pad_r (Link 1 R) that puts it on the pad's own diameter, s = 0
    if half_w > pad_r:
        raise ValueError('the %.0f mm wide plate is wider than the r %.1f pad it leaves '
                         '(%.2f mm per side)' % (width, pad_r, half_w - pad_r))
    root_s = max(0.0, min(PROFILE_ROOT_S, math.sqrt(pad_r ** 2 - half_w ** 2) - 1.0))
    r_out = r_out_nominal = min(L_ROUND_OUT, s_hi - pad_r - JOIN_RIM_MARGIN)
    if r_out < L_ROUND_MIN:
        raise ValueError('the outer round would be r %.2f (the slab far face is at s=%.2f, '
                         'only %.2f mm outside the r %.1f pad rim); need >= %.1f'
                         % (r_out, s_hi, s_hi - pad_r, pad_r, L_ROUND_MIN))
    # The tongue runs from the disk straight down the LINK frame (the snapped frame B'),
    # ending flush with the plate's rotor-side plane z = 0, and is centred on the plate - so
    # on the disk it sits off-centre by the pair's whole lateral offset (EXO-92 judgment 2).
    # In B' a link-frame station (w, z) is (x, y) = (x_sign (w - w_off), y_sign (z - axial)).
    x_sign = y_sign * num['u_sign']
    y_end = y_sign * (0.0 - axial)
    x_off = -w_off * x_sign
    # EXO-124: the plate passes THROUGH the slab, so its two land radii are solved against
    # the insert holes exactly as the ramp's are. Both G1 shoulder pairs have 18 mm of room
    # here, so this returns the measured radii untouched - it is the same rule, not an
    # exception, and a Gento Luna pair that runs closer will get a smaller radius instead of
    # a hole with a fillet in it.
    inserts = insert_slab_points(num, num['x_s'], num['y_s'], num['z_s'])
    x_band = (x_off - half_w - TONGUE_ROUND_MARGIN, x_off + half_w + TONGUE_ROUND_MARGIN)
    r_in, land_tight = land_radius_fit(L_FILLET_IN, y_end, PAD_T, inserts,
                                       x_band[0], x_band[1], 'land fillet')
    r_out, corner_tight = land_radius_fit(r_out, y_end, 0.0, inserts,
                                          x_band[0], x_band[1], 'elbow outer round')
    if r_out < L_ROUND_MIN:
        raise ValueError('the outer round is r %.2f after the insert holes have their %.1f '
                         'mm of flat ring; need >= %.1f' % (r_out, HOLE_FLAT_RING, L_ROUND_MIN))
    z_up = PAD_T + r_in + L_BURY_UP
    y_up = y_sign * (z_up - axial)      # the toe corner: where the profile's material stops
    edges = flange_edges(y_end, x_off, half_w, STATOR_PAD_R, toe_y=y_up, overhang=overhang)
    flange, y_blend, _t = flange_outline(y_end, half_w, STATOR_PAD_R, x_off=x_off, edges=edges)
    # The profile's buried top must lie INSIDE the slab over the whole band, so that joining
    # the two leaves the slab's own outline in charge there.
    for e in edges:
        lim = (e['y_wall'] if e['kind'] in ('toe', 'crown')
               else -math.sqrt(max(0.0, STATOR_PAD_R ** 2 - e['x'] ** 2)))
        if not (lim <= y_up <= y_end - FLANGE_TONGUE_MARGIN):
            raise ValueError('the side profile is buried to z=%.2f (y=%.2f in the slab) but '
                             'its wall at x=%.2f only covers the %.0f wide band over y '
                             '%.2f..%.2f' % (z_up, y_up, e['x'], width, lim,
                                             y_end - FLANGE_TONGUE_MARGIN))
    # the plate's top face must leave a web of slab between itself and the actuator cutout,
    # measured in the frame the tongue is drawn in but against the TRUE cutout, which is cut
    # in frame B where it mates
    y_plate_top = y_sign * (PAD_T - axial)
    y_s = num['y_s']
    cx_y, cy_y = dot(num['x_b'], y_s), dot(num['y_b'], y_s)
    cut_reach = cutout_reach(num, y_s)
    cut_web = y_plate_top - cut_reach
    if cut_web < L_CUT_CLEAR:
        raise ValueError('only %.2f mm of slab between the actuator cutout (reaching y=%.2f) '
                         'and the plate top face (y=%.2f); need >= %.1f'
                         % (cut_web, cut_reach, y_plate_top, L_CUT_CLEAR))
    ins_reach = max(INSERT_RADIUS * (math.cos(math.radians(a)) * cx_y
                                     + math.sin(math.radians(a)) * cy_y) + HOLE_R
                    for a in INSERT_ANGLES)
    if ins_reach >= y_plate_top:
        raise ValueError('an insert hole reaches y=%.2f, at or past the plate top face y=%.2f'
                         % (ins_reach, y_plate_top))
    # Why there is no S here: the same fit test the axial route uses, reported not hidden.
    # The radius is the one THIS band could actually turn through (s_radius(width)): at 52 mm
    # across, the 17.5 mm the 25-wide bar uses would turn the inner wall inside out.
    s_fit = None
    if abs(w_off) >= S_MIN_OFFSET:
        r_s = s_radius(width)
        try:
            th, run = s_curve_run(w_off, r_s)
        except ValueError as e:
            s_fit = {'run': None, 'window': (root_s, s_lo), 'r': r_s, 'why': str(e)}
        else:
            s_fit = {'theta_deg': th, 'run': run, 'window': (root_s, s_lo), 'r': r_s,
                     'window_len': s_lo - root_s,
                     'fits': run + 2 * S_BAR_MARGIN <= s_lo - root_s}
            if s_fit['fits']:
                s_fit['why'] = ('an S would fit; a coplanar join still takes the offset in '
                                'the slab so the plate stays flush with the pad over its '
                                'full width')
            else:
                s_fit['why'] = ('a %.2f mm shift needs %.2f mm of run at radius %.1f and the '
                                'plate is only %.2f mm long (s %.2f..%.2f)'
                                % (w_off, run, r_s, s_lo - root_s, root_s, s_lo))
    route = make_route(
        width, (root_s, PAD_T / 2.0), (1.0, 0.0),
        [Run(s_lo - root_s, PAD_T, 'plate')],
        (rotor_join('coplanar', root_s, pad_r),
         stator_land(s_lo, r_in, 0.0, L_BURY_UP, LAND_BURY_THROUGH, r_out,
                     2.0 * (max(STATOR_PAD_R, half_w) + TONGUE_ROUND_MARGIN),
                     fillet_r_nominal=L_FILLET_IN, corner_r_nominal=r_out_nominal,
                     flange=flange, y_end=y_end, y_blend=y_blend, edges=edges)),
        pattern='L', departure='in-plane', s_curve=None, placement=None,
        numbers=num, pad_r=pad_r, u=u, w=w, w_off=w_off, zL=zL,
        extra={'s_lo': s_lo, 's_hi': s_hi, 'plate_z': (0.0, PAD_T), 'z_up': z_up,
               'r_in': r_in, 'r_out': r_out, 'r_out_measured': L_ROUND_OUT,
               'land_tight': None if land_tight is None else land_tight['angle'],
               'corner_tight': None if corner_tight is None else corner_tight['angle'],
               'x_off': x_off, 'x_sign': x_sign, 'y_sign': y_sign, 'axial': axial,
               'half_w': half_w, 'band_w': (-half_w, half_w), 'y_up': y_up,
               'level': level, 'column': column, 'overhang': overhang,
               'column_z': (z_up, axial - STATOR_PAD_R) if column else None,
               'blend_r': tuple(e['r'] for e in edges), 'round_cut_w':
                   2.0 * (max(STATOR_PAD_R, half_w) + TONGUE_ROUND_MARGIN),
               'cut_web': cut_web, 'cut_reach': cut_reach, 'y_plate_top': y_plate_top,
               'clearance_b': over, 's_fit': s_fit})
    return route


def link_l_geometry(R_ab, t_ab, pad_r, width=None):
    """Thin wrapper: the route that leaves the rotor pad in its own plane, drawn."""
    return render_route(_inplane_route(landing_numbers(R_ab, t_ab), pad_r,
                                      L_W if width is None else width))


def _ramp_route(num, pad_r, width, placement=None):
    """Size `Link 4 R`'s route (EXO-114): leave the rotor pad IN ITS OWN PLANE as the
    shoulder L does, CLIMB over the stator's actuator cutout through an S in the SIDE plane,
    and land with the run's centre line exactly ON the stator disk's rim, on the side away
    from the connector - where the cutout only reaches |CUTOUT_Y[0]| instead of CUTOUT_Y[1].
    The lateral offset goes in the PLAN TAPER, not in an S and not in the slab: each wall is
    drawn tangent to the rim it has to stay inside, so the band sits w_off/2 off centre at
    BOTH ends. Every number is measured from Link 4 R; raises ValueError naming what failed."""
    if num['zb_in_plane'] < BAR_SQUARE_MIN:
        raise ValueError('stator axis B is not in the rotor-pad plane '
                         '(in-plane part %.2f, need > %.2f)' % (num['zb_in_plane'], BAR_SQUARE_MIN))
    if num['snap_axis_rad'] > BAR_SNAP_MAX:
        raise ValueError('stator AXIS is %.2e rad off square to the pad (max %.0e); this '
                         'route lands across the disk and needs the tongue walls and the '
                         'band walls to be the same planes'
                         % (num['snap_axis_rad'], BAR_SNAP_MAX))
    u, w, w_off = num['u'], num['w'], num['w_off']
    axial, y_sign, u_sign = num['axial'], num['y_sign'], num['u_sign']
    half_w = width / 2.0
    s_lo, s_hi = num['s_slab']
    if s_lo < pad_r + 2 * PAD_T:
        raise ValueError('the stator slab is only %.1f mm from axis A (need > %.1f): a ramp '
                         'needs room to climb' % (s_lo, pad_r + 2 * PAD_T))
    # the landing: centre line on the rim, on the side of the disk away from the connector
    y_land = -STATOR_PAD_R
    z_run = axial - STATOR_PAD_R * y_sign
    rise = z_run - PAD_T / 2.0
    if rise <= PAD_T:
        raise ValueError('the disk rim away from the connector sits at z=%.2f, only %.2f mm '
                         'over the pad plate mid-plane %.1f: there is nothing to climb and a '
                         'coplanar join cannot go below the pad' % (z_run, rise, PAD_T / 2.0))
    # the band: tangent to the rotor rim on one side and to the stator disk on the other, so
    # the profile root sits on the pad's own diameter, s = 0, where both walls are on the rim
    root_s = 0.0
    th, horiz = rise_numbers(rise, RAMP_RISE_R)
    cap = (root_s - 2.0, s_hi + 2.0)
    taper = plan_taper(w_off, width, pad_r, cap, STATOR_PAD_R, PLAN_TAPER_R, s_wall_end=s_lo)
    centre_w = taper['centre']
    # the slab body frame: the tongue leaves the disk on the side the run climbs to, which is
    # the snapped frame turned 180 deg about its own axis. The disk is symmetric about it and
    # the holes and the cutout are still cut in the TRUE frame B, where they mate (EXO-61).
    y_slab = -y_sign
    y_s = [0.0, 0.0, y_slab]
    z_s = num['z_s']
    x_s = cross(y_s, z_s)
    x_sign = y_slab * u_sign
    y_end = y_slab * (z_run + PAD_T / 2.0 - axial)
    x_off = x_sign * (centre_w - w_off)
    # EXO-130. The run crosses the whole slab, so the actuator cutout has to be clear of it -
    # and the cutout turns with the module about its own axis while the slab stays put, so
    # this is measured in the slab's own frame against the TRUE cutout, exactly as the
    # in-plane route measures its web. It was `CUTOUT_Y[0]` against a landing at -Y_B, which
    # only reads the right number while the connector side points along the pad axis.
    cut_reach = cutout_reach(num, y_s)
    cut_web = (y_end - PAD_T) - cut_reach
    if cut_web < BAR_CLEAR:
        raise ValueError('only %.2f mm of slab between the actuator cutout (reaching y=%.2f '
                         'in the slab frame, the module turned %.1f deg about its own axis) '
                         'and the run crossing it (y=%.2f); need >= %.2f'
                         % (cut_web, cut_reach, math.degrees(num['spin_rad']),
                            y_end - PAD_T, BAR_CLEAR))
    # EXO-124, the LANDING end. The run crosses the slab 44 mm wide and the tongue always
    # passes over the insert on the side away from the connector (INSERT_ANGLES has none on
    # +Y, and this is the route that leaves the disk on -Y), so the two land radii are the
    # only free parameters left between the run and that hole. SOLVE them - the largest
    # radius that still leaves every insert its flat ring - instead of drawing r 5 and
    # discovering afterwards that the fillet has eaten the hole, which is what `Link 4 R`
    # itself does (its own insert circle is cut 1.849 mm deep).
    inserts = insert_slab_points(num, x_s, y_s, z_s)
    x_band = (x_off - half_w - TONGUE_ROUND_MARGIN, x_off + half_w + TONGUE_ROUND_MARGIN)
    r_land, land_tight = land_radius_fit(RAMP_FILLET, y_end, PAD_T, inserts,
                                         x_band[0], x_band[1], 'land fillet')
    r_corner, corner_tight = land_radius_fit(RAMP_FILLET, y_end, 0.0, inserts,
                                             x_band[0], x_band[1], 'far-face corner round')
    # EXO-124, the ROTOR end. The pad's OUTER face is where the four M3 heads seat, and the
    # climb's upper boundary leaves that face `stagger` BEFORE the centre line does - that is
    # exactly what put link_45's outermost hole on the start of the r 20 curve. So the window
    # does not open at the root: it opens where the plate has to stay flat to, and the S is
    # still CENTRED in whatever window is left (EXO-69's rule, on a shorter window).
    stagger = (PAD_T / 2.0) * math.tan(math.radians(th) / 2.0)
    s_flat = rotor_flat_needed(u)
    run_window = (root_s, s_lo - r_land)
    # Where the climb's CENTRE LINE may start. The two boundaries are staggered `stagger`
    # about it - the rise-side one leaves the plate first, the other resumes on the bar last
    # - so RAMP_LAND_MARGIN is measured on the BOUNDARIES, which is what `route_profile`
    # actually checks. (Before EXO-124 it was measured on the centre line; it never bound,
    # and the difference is the whole stagger.)
    a_min = max(root_s + RAMP_LAND_MARGIN + stagger,
                s_flat + stagger)              # EXO-124: the screw heads' flat ring
    a_max = run_window[1] - horiz - stagger - RAMP_LAND_MARGIN
    # The climb must not GRAZE the rim. Where its lower boundary leaves the pad plane decides
    # how much of the pad's rim cylinder shows under it (EXO-123): leave it well inside and
    # the rim is a real face, leave it outside and there is none at all - but leave it AT the
    # rim and what is left is a sliver, which is EXO-61's other defect signature. Pushed to
    # s = 26.489 by an earlier draft of this window, link_45 grew a 0.0365 mm2 face there.
    # JOIN_RIM_MARGIN is this module's existing 'no feature closer than this to the rim'.
    #
    # OUTSIDE the rim would be EXO-74 met literally, and the window is now long enough to
    # reach it (`rim_rule` records that, and it is TRUE for this pair - EXO-123 said a ramp
    # at r 20 could not). It is not taken: with the plate covering the whole disk over the
    # pad's own thickness, the rotor pad disk's JOIN has nothing left to add inside the band
    # and its rim cylinder is exactly coincident with the band wall, and Fusion refuses the
    # feature outright ('extrude (JOIN, 8.00 mm) failed: !extrudes.empty()'). That is a
    # BUILD-ORDER limit, not a geometry one, and it belongs to EXO-41 with EXO-123.
    rim_rule = pad_r + JOIN_RIM_MARGIN - stagger <= a_max + 1e-12
    a_max = min(a_max, pad_r - JOIN_RIM_MARGIN - stagger)
    if a_max < a_min - 1e-12:
        raise ValueError('the ramp needs %.2f mm of run to climb %.2f mm at r %.1f and its '
                         'boundaries stagger %.2f: the climb may start no earlier than '
                         's %.3f (%.1f mm of flat ring past the screw holes%s) and no later '
                         'than s %.3f (%.1f mm of straight bar before the r %.2f land '
                         'fillet at s %.2f, and %.1f mm clear of the r %.1f rim)'
                         % (horiz, rise, RAMP_RISE_R, stagger, a_min, HOLE_FLAT_RING, '',
                            a_max, RAMP_LAND_MARGIN, r_land, run_window[1],
                            JOIN_RIM_MARGIN, pad_r))
    window = (a_min, a_max)
    slack = a_max - a_min
    a_c = (a_min + a_max) / 2.0                  # the S is CENTRED in its window (EXO-69)
    s_bar = a_c + horiz
    edges = flange_edges(y_end, x_off, half_w, STATOR_PAD_R)
    flange, y_blend, _t = flange_outline(y_end, half_w, STATOR_PAD_R, x_off=x_off, edges=edges)
    # the buried top must lie inside the slab outline over the whole band
    y_up = y_slab * (z_run - PAD_T / 2.0 - r_land - L_BURY_UP - axial)
    for e in edges:
        lim = (e['y_wall'] if e['kind'] in ('toe', 'crown')
               else -math.sqrt(max(0.0, STATOR_PAD_R ** 2 - min(e['x'] ** 2,
                                                               STATOR_PAD_R ** 2))))
        if not (lim <= y_up <= y_end - FLANGE_TONGUE_MARGIN):
            raise ValueError('the run is buried to y=%.2f in the slab but its wall at x=%.2f '
                             'only covers the %.0f wide band over y %.2f..%.2f'
                             % (y_up, e['x'], width, lim, y_end - FLANGE_TONGUE_MARGIN))
    # the corner round at the slab's far face is cut across the TONGUE - its two walls and
    # whatever each edge does to reach the rim - and not across the disk, which is why the
    # rim arc of the outline takes no part in the width
    tongue_w = max([abs(taper['w_lo']), abs(taper['w_hi'])]
                   + [abs(x_sign * v + w_off) for e in edges for v in (e['x'], e['t'][0])])
    cut_w = 2.0 * (tongue_w + TONGUE_ROUND_MARGIN)
    # why there is no plan-view S here, reported and not hidden
    s_fit = None
    if abs(w_off) >= S_MIN_OFFSET:
        r_s = s_radius(width)
        try:
            th_s, run_s = s_curve_run(w_off, r_s)
        except ValueError as e:
            s_fit = {'run': None, 'window': run_window, 'r': r_s, 'why': str(e)}
        else:
            s_fit = {'theta_deg': th_s, 'run': run_s, 'window': run_window, 'r': r_s,
                     'window_len': run_window[1] - run_window[0],
                     'fits': run_s + 2 * S_BAR_MARGIN <= run_window[1] - run_window[0],
                     'why': ('a %.2f mm shift needs %.2f mm of run at radius %.1f; a coplanar '
                             'join cannot carry one anyway - the walls would leave the rotor '
                             'rim curved - so the offset goes in the plan taper'
                             % (w_off, run_s, r_s))}
    route = make_route(
        width, (root_s, PAD_T / 2.0), (1.0, 0.0),
        [Run(a_c - root_s, PAD_T, 'plate'),
         Rise(rise, RAMP_RISE_R, 'climb'),
         Run(s_lo - s_bar, PAD_T, 'bar')],
        (rotor_join('coplanar', root_s, pad_r),
         stator_land(s_lo, r_land, 0.0, L_BURY_UP, LAND_BURY_THROUGH, r_corner,
                     cut_w, s_far=s_hi, side=-1,
                     fillet_r_nominal=RAMP_FILLET, corner_r_nominal=RAMP_FILLET,
                     flange=flange, y_end=y_end, y_blend=y_blend, edges=edges)),
        pattern='ramp', departure='ramp', s_curve=None, placement=None,
        numbers=num, pad_r=pad_r, u=u, w=w, w_off=w_off, zL=z_run,
        taper=taper, offset_device='taper',
        slab_axes=(x_s, y_s, z_s),
        extra={'s_lo': s_lo, 's_hi': s_hi, 'plate_z': (0.0, PAD_T), 'z_up': z_run,
               'r_in': r_land, 'r_out': r_corner, 'r_land_nominal': RAMP_FILLET,
               'land_tight': None if land_tight is None else land_tight['angle'],
               'corner_tight': None if corner_tight is None else corner_tight['angle'],
               'rise': rise,
               'rise_r': RAMP_RISE_R, 'rise_theta_deg': th, 'rise_run': horiz,
               'rise_window': window, 'rise_run_window': run_window,
               'rise_margin': (a_c - a_min, a_max - a_c),
               'rise_stagger': stagger, 'rotor_flat_needed': s_flat, 'rim_rule': rim_rule,
               'rise_lift_off': (a_c - stagger, a_c + stagger),
               'rise_s': (a_c, s_bar), 'bar_z': (z_run - PAD_T / 2.0, z_run + PAD_T / 2.0),
               'x_off': x_off, 'x_sign': x_sign, 'y_sign': y_slab, 'axial': axial,
               'half_w': half_w, 'band_w': (taper['w_lo'], taper['w_hi']), 'y_up': y_up,
               'y_land': y_land, 'centre_w': centre_w, 'level': rise,
               'column': False, 'overhang': None,
               'R_b': [[x_s[i], y_s[i], z_s[i]] for i in range(3)],
               'blend_r': tuple(e['r'] for e in edges), 'round_cut_w': cut_w,
               'clearance_b': cut_web, 'clear_cut': cut_web, 'cut_reach': cut_reach,
               'clearance_note': 'the run lands on the disk rim at Y_B=%.1f and clears the '
                                 'actuator cutout, which reaches y=%.2f in the slab frame, '
                                 'by %.2f mm' % (y_land, cut_reach, cut_web),
               's_fit': s_fit})
    return route


# ---------------- the route: one ribbon, any pair (EXO-96) ----------------
# EXO-81's router decided generally (reach / level / offset) but drew two named shapes, so 3
# of G1's 6 pairs were refused (EXO-95). A link is really ONE thing: a ribbon of a given
# width swept along a ROUTE - straight runs joined by tangent bends - from the rotor-pad join
# to the stator-pad join. `route_path` computes that path; `render_route` draws it. The two
# shapes that used to be patterns are now two routes, and there is one renderer:
#
#   bar (link_23)  ledge join -> Run('riser', down the pad axis) -> Bend(90 deg, side plane)
#                  -> Run('bar', out along u, carrying the plan-view S) -> slab land
#   L   (link_12)  coplanar join -> Run('plate', out along u in the pad's own plane) -> land
#
# A route is data in the rotor-pad frame's SIDE plane (s along u, z along the pad axis):
#   Run(length, thickness, label)           a straight run of the ribbon's centre line
#   Bend(angle, plane, r_left, r_right)     a tangent corner, carrying the two BOUNDARY radii
#                                           (Link 2 R's bend is not a constant-thickness
#                                           offset: walls r 12.5 / r 15 across a 6.44 -> 7.79
#                                           step, so neither is derivable from the other)
#   SBend(offset, radius, placement)        the lateral shift, taken in the PLAN plane (s, w)
#                                           by `s_curve_run` / `riser_s_curve` (EXO-64/69)
# plus the two join descriptors, which stay exactly as approved and draw themselves:
#   rotor 'ledge'     EXO-74 + EXO-83 - pad outer face flat past the rim, JOIN_ROUND_R down
#                     onto the run, the RISER_CLEAR ledge, the JOIN_RELIEF_R relief
#   rotor 'coplanar'  EXO-92 - the run IS the pad continued outward, both faces flat, no
#                     ledge, no relief, nothing below the pad's rotor-side plane
#   stator 'slab'     the run dies inside the stator slab through a tangent fillet, buried;
#                     the slab outline is `flange_outline` with the r 500 toe rule (EXO-93)
#
# LIMIT OF THIS RENDERER, stated rather than hidden. The side-profile + plan-band machinery
# (EXO-58 / EXO-64) draws bends in the SIDE plane only; the plan plane carries an S and
# nothing else. A route that needs a real bend in the plan plane - G1's J6->J7, 46 mm of
# lateral in 32.5 mm of reach, which no S can take (EXO-95) - is refused by `route_runs`
# with the plane named. Drawing it needs a different renderer: a 3D sketch path through the
# route's centre line plus a rectangular profile, swept with Fusion `sweepFeatures`. EXO-96
# does not switch renderers, because the regression has to hold bit for bit.
ROUTE_PLANES = ('side', 'plan')
ROTOR_JOINS = ('ledge', 'coplanar')
DEPARTURES = ('axial', 'in-plane', 'ramp')     # how the ribbon leaves the rotor pad
# The rotor join each departure uses, and the label the route carries. The label is a NAME
# FOR A ROUTE (so BuildArm's LINKS table, EXO-88's classification and every measurement
# script keep their words) - it is not a builder: there is one builder.
DEPARTURE_JOIN = {'axial': 'ledge', 'in-plane': 'coplanar', 'ramp': 'coplanar'}
DEPARTURE_PATTERN = {'axial': 'bar', 'in-plane': 'L', 'ramp': 'ramp'}
PATTERN_DEPARTURE = dict((v, k) for k, v in DEPARTURE_PATTERN.items())
# Where a pair's lateral offset goes. It follows from the ROTOR JOIN, not from a shape:
#  'ledge'    the ribbon has already left the pad and dropped to the run's level, so the
#             shift is an S-bend inside a straight run (EXO-64: the bar leaves the pad
#             centred). No run long enough -> the route is REFUSED, naming the run it would
#             need; putting the tongue off-centre instead is the answer EXO-64 rejected.
#  'coplanar' the run IS the pad continued: an S in it would curve the walls at the rim and
#             break the join Esteban inspects, so the shift is taken in the slab (EXO-92).
JOIN_TAKES_OFFSET = {'ledge': 's-bend', 'coplanar': 'slab'}
# ...and every device each join ALLOWS. A coplanar join has a third, measured on `Link 4 R`
# (EXO-114): the plan TAPER, where each band wall is drawn tangent to the rim it has to stay
# inside, so the offset is split w_off/2 between the two pads. It is only open to a coplanar
# join for the same reason the S is not: a tapered wall still leaves the rotor rim TANGENT to
# it, which is the join Esteban inspects, while a ledge join's band left the pad long ago and
# has nothing to be tangent to. A route names the device it uses; anything else is refused.
JOIN_OFFSET_DEVICES = {'ledge': ('s-bend',), 'coplanar': ('slab', 'taper')}
# Width is Esteban's input per link and is never derived (EXO-96): BuildArm's LINKS table
# passes it. These are only the fallback for callers that survey pairs without building -
# the two approved values, 25 for the bar (BAR_W) and 52 for the shoulder (L_W, EXO-92).
ROUTE_WIDTH_DEFAULT = {'axial': BAR_W, 'in-plane': L_W, 'ramp': RAMP_W}
# How deep the run dies inside the stator slab. The bar stops half way; the shoulder plate
# runs THROUGH, because its far boundary is the pad's own rotor-side plane and the elbow
# round is cut on it. A run that merely stopped ON the slab's far face would leave two
# coplanar faces (EXO-61).
LAND_BURY_HALF = PAD_T / 2.0
LAND_BURY_THROUGH = PAD_T
# A straight part of the profile shorter than this counts as zero length. It is not a
# tolerance on the design - the shortest real straight part in either link is the 0.240 mm of
# flat ledge outside the rim (EXO-84) - it is the floor under which a "face" cannot be
# manufactured and is in any case below the rounding of the route's own arithmetic: walking a
# centre line and offsetting it moves a nominal 0.000 to 3.6e-15, which would otherwise make
# a guard that must be LOUD exactly at its cap (JOIN_RELIEF_R = RISER_CLEAR) silently pass.
ROUTE_MIN_STRAIGHT = 1e-9


def Run(length, thickness, label=''):
    """A straight run of the ribbon's centre line: `length` mm long, `thickness` mm across in
    the side plane. The ribbon's WIDTH (across the plan plane) belongs to the route: one
    number for the whole link."""
    return {'kind': 'run', 'length': length, 'thickness': thickness, 'label': label}


def Bend(angle, plane, r_left, r_right, label=''):
    """A tangent corner of `angle` degrees (positive turns toward the ribbon's left) in
    `plane`. `r_left` / `r_right` are the fillet radii of the two BOUNDARIES; see the section
    comment for why both are carried instead of one centre-line radius."""
    if plane not in ROUTE_PLANES:
        raise ValueError('unknown bend plane %r (expected one of %s)'
                         % (plane, ', '.join(ROUTE_PLANES)))
    return {'kind': 'bend', 'angle': angle, 'plane': plane, 'r_left': r_left,
            'r_right': r_right, 'label': label}


def bend_centre_radius(bend, thickness):
    """The centre-line radius a bend of these two boundary radii implies, and how far the two
    disagree. Reported, never used to draw: for Link 2 R's measured bend (12.5 / 15 across
    6.44 -> 7.79) they disagree by 3.94 mm, which is the number to look at when a bend is
    designed rather than copied. Returns (radius, disagreement)."""
    return (bend['r_left'] + bend['r_right']) / 2.0, \
           (bend['r_left'] + thickness) - bend['r_right']


def SBend(offset, radius, placement, label=''):
    """The lateral shift, taken in the PLAN plane (s, w) inside one run: two tangent arcs of
    `radius` whose walls are concentric offsets, so the band keeps its full width
    (riser_s_curve / bar_s_curve, EXO-64 / EXO-69)."""
    return {'kind': 'S', 'offset': offset, 'radius': radius, 'placement': placement,
            'label': label}


def Rise(rise, radius, label=''):
    """An S in the SIDE plane: the ribbon shifts `rise` mm perpendicular to its travel
    (positive toward its LEFT) through two tangent arcs of radius `radius` on EACH boundary,
    and comes out parallel to where it went in. `Link 4 R` is exactly this - four r 20.000
    arcs, two per boundary, no straight between them (EXO-114) - and it is to the side plane
    what `SBend` is to the plan plane.

    The two boundaries' S-curves are STAGGERED along the travel by `thickness * tan(theta/2)`,
    the rise-side one first, which is what keeps the ribbon its full thickness measured
    across the ribbon at the inflection, where the turn is steepest. (Link 4 R staggers 3.427
    where this rule asks 4.150 and is 5.325 thick at the inflection instead of 6.000; drawing
    the two boundaries at the same station would neck it to 6 cos(theta) = 2.1 mm.)"""
    return {'kind': 'rise', 'rise': rise, 'radius': radius, 'plane': 'side', 'label': label}


def rise_numbers(rise, radius):
    """(theta_deg, run) of the S that shifts a ribbon by `rise` at boundary radius `radius`:
    2R(1 - cos th) = |rise|, run = 2R sin th - the same arithmetic as `s_curve_run`, in the
    other plane. Raises when the shift is beyond 2R."""
    if radius <= 0.0:
        raise ValueError('rise radius %.2f must be positive' % radius)
    if abs(rise) > 2.0 * radius:
        raise ValueError('a side-plane S of radius %.2f cannot reach a %.2f mm rise (max %.2f)'
                         % (radius, rise, 2.0 * radius))
    th = math.acos(1.0 - abs(rise) / (2.0 * radius))
    return math.degrees(th), 2.0 * radius * math.sin(th)


def rotor_join(kind, root_s, pad_r, pad_t=None, round_r=None, relief_r=None,
               rim_margin=None):
    """Descriptor of the rotor-pad join: 'ledge' (EXO-74 / EXO-83) or 'coplanar' (EXO-92).
    None means the module constant, read HERE and not at import time, so that a test (or a
    sabotage) that moves JOIN_ROUND_R / JOIN_RELIEF_R still moves the join."""
    if kind not in ROTOR_JOINS:
        raise ValueError('unknown rotor join %r (expected one of %s)'
                         % (kind, ', '.join(ROTOR_JOINS)))
    return {'kind': kind, 'root_s': root_s, 'pad_r': pad_r,
            'pad_t': PAD_T if pad_t is None else pad_t,
            'round_r': JOIN_ROUND_R if round_r is None else round_r,
            'relief_r': JOIN_RELIEF_R if relief_r is None else relief_r,
            'rim_margin': JOIN_RIM_MARGIN if rim_margin is None else rim_margin}


def s_radius(width):
    """The tightest centre-line radius an S may use for a band `width` across: its inner wall
    is S_WALL_R_MIN, the smallest fillet already in the profile. The run of an S GROWS with
    the radius, so this is also the SHORTEST S that band can make. S_RADIUS (17.5) is this
    number for BAR_W; a 52 mm band cannot use it at all - its inner wall would turn inside
    out - which is why the width has to reach the fit test (EXO-96)."""
    return width / 2.0 + S_WALL_R_MIN


def stator_land(s_face, fillet_r, bury_s, bury_up, bury_depth, corner_r, corner_cut_w,
                s_far=None, **slab):
    """Descriptor of the stator-pad join as the RUN meets it: the run dies `bury_depth` mm
    inside the slab whose near face is at s = `s_face`, entering through a tangent fillet
    r `fillet_r` from the run's slab-side boundary, `bury_s` past that face and `bury_up`
    past the fillet. The corner round r `corner_r` at the slab's FAR face is a CUT
    `corner_cut_w` wide - a join fills that corner whatever the profile draws, so only a cut
    can shape it. `slab` carries the slab outline the sizer drew (flange, y_end, edges)."""
    d = {'kind': 'slab', 's_face': s_face, 'fillet_r': fillet_r, 'bury_s': bury_s,
         'bury_up': bury_up, 'bury_depth': bury_depth, 'corner_r': corner_r,
         'corner_cut_w': corner_cut_w, 'side': 1,
         # EXO-130: the radii the sizer MEANT to draw - the ones measured on the hand-built
         # part - before `land_radius_fit` cut them back for an insert hole. `fillet_r` equal
         # to `fillet_r_nominal` is the statement that nothing had to be given up here, and
         # it is what the rotation solve searches for.
         'fillet_r_nominal': fillet_r, 'corner_r_nominal': corner_r,
         's_far': s_face + PAD_T if s_far is None else s_far}
    d.update(slab)
    return d


def make_route(width, start, start_dir, segments, joins, **extra):
    """A route: the ribbon `width` mm across, swept from `start` along `start_dir` through
    `segments` (Runs and Bends in order), between `joins` = (rotor join, stator land)."""
    r = {'width': width, 'start': tuple(start), 'start_dir': tuple(start_dir),
         'segments': list(segments), 'joins': tuple(joins),
         'rotor_join': joins[0], 'stator_land': joins[1], 's_curve': None,
         'taper': None, 'slab_axes': None, 'offset_device': None}
    r.update(extra)
    return r


def _rot90(d, left=True):
    return (-d[1], d[0]) if left else (d[1], -d[0])


def _snap_dir(d, tol=1e-12):
    """A turned direction with its rounding noise removed: cos(pi/2) is 6.1e-17, not 0, and
    without this every number past a bend carries that error into the built body."""
    out = []
    for v in d:
        for k in (-1.0, 0.0, 1.0):
            if abs(v - k) < tol:
                v = k
                break
        out.append(v)
    return tuple(out)


def route_runs(route):
    """The route's centre line as runs, in order. Each is a dict with
      p0, p1   start / end vertex in the side plane (s, z)
      d, n     unit direction and its LEFT normal
      half     half the run's thickness (the ribbon is symmetric about its centre line)
      bend_in / bend_out   the Bend at each end, or None; length; label
    Raises ValueError, naming the plane, for a bend this renderer cannot draw."""
    runs = []
    p, d = tuple(route['start']), tuple(route['start_dir'])
    pending = None
    pending_rise = None
    for seg in route['segments']:
        if seg['kind'] == 'run':
            q = (p[0] + d[0] * seg['length'], p[1] + d[1] * seg['length'])
            runs.append({'p0': p, 'p1': q, 'd': d, 'n': _rot90(d), 'length': seg['length'],
                         'half': seg['thickness'] / 2.0, 'thickness': seg['thickness'],
                         'label': seg['label'], 'bend_in': pending, 'bend_out': None,
                         'rise_in': pending_rise, 'rise_out': None})
            p, pending, pending_rise = q, None, None
        elif seg['kind'] == 'rise':
            if not runs:
                raise ValueError('a route cannot start with a rise: a rise joins two runs')
            if runs[-1]['bend_out'] is not None or runs[-1]['rise_out'] is not None:
                raise ValueError('run %r already turns; a rise needs a run of its own'
                                 % (runs[-1]['label'] or 'unnamed',))
            th, horiz = rise_numbers(seg['rise'], seg['radius'])
            n = runs[-1]['n']
            p = (p[0] + d[0] * horiz + n[0] * seg['rise'],
                 p[1] + d[1] * horiz + n[1] * seg['rise'])
            runs[-1]['rise_out'] = dict(seg, theta_deg=th, horiz=horiz,
                                        sign=(1.0 if seg['rise'] >= 0 else -1.0))
            pending_rise = runs[-1]['rise_out']
        elif seg['kind'] == 'bend':
            if seg['plane'] != 'side':
                raise ValueError(
                    'route bend %r turns %.1f deg in the %r plane: this renderer (side '
                    'profile + plan band) draws bends in the side plane only; a plan-plane '
                    'bend needs a 3D sweep along the route (EXO-96)'
                    % (seg['label'] or 'unnamed', seg['angle'], seg['plane']))
            if not runs:
                raise ValueError('a route cannot start with a bend: a bend joins two runs')
            a = math.radians(seg['angle'])
            d = _snap_dir((d[0] * math.cos(a) - d[1] * math.sin(a),
                           d[0] * math.sin(a) + d[1] * math.cos(a)))
            runs[-1]['bend_out'] = seg
            pending = seg
        else:
            raise ValueError('unknown route segment %r' % (seg['kind'],))
    if pending is not None:
        raise ValueError('a route cannot end with a bend: a bend joins two runs')
    if pending_rise is not None:
        raise ValueError('a route cannot end with a rise: a rise joins two runs')
    if not runs:
        raise ValueError('a route needs at least one run')
    return runs


def _wall(run, side, a=0.0):
    """The point `a` mm along the run on one boundary (side +1 = left of travel, -1 right)."""
    o = side * run['half']
    return (run['p0'][0] + run['n'][0] * o + run['d'][0] * a,
            run['p0'][1] + run['n'][1] * o + run['d'][1] * a)


def _along(run, p):
    """Where p sits along the run's direction, measured from its start vertex."""
    return (p[0] - run['p0'][0]) * run['d'][0] + (p[1] - run['p0'][1]) * run['d'][1]


def _ang(c, p):
    """Angle of p about c, in [0, 360) - the range the hand-written outlines used, so an arc
    drawn here has the same numbers as the one it replaces."""
    return math.degrees(math.atan2(p[1] - c[1], p[0] - c[0])) % 360.0


def _sweep_arc(segs, c, r, p_start, sweep):
    """A tangent arc of radius r about c starting at p_start and turning `sweep` degrees."""
    a0 = _ang(c, p_start)
    _arc(segs, c, r, a0, a0 + sweep)


def _bend_fillet(run_a, run_b, side, bend):
    """One boundary's corner at a bend: the two boundary lines meet at a corner and the
    fillet is tangent to both, its centre r toward the turn. Returns
    (tangent in, tangent out, centre, radius). Only the tangent DISTANCE depends on the
    angle, so this is the general construction; the exo links exercise it at 90 degrees."""
    r = bend['r_left'] if side > 0 else bend['r_right']
    pa, da = _wall(run_a, side), run_a['d']
    pb, db = _wall(run_b, side), run_b['d']
    den = da[0] * db[1] - da[1] * db[0]
    if abs(den) < 1e-12:
        raise ValueError('route bend %r does not turn' % (bend['label'] or 'unnamed',))
    k = ((pb[0] - pa[0]) * db[1] - (pb[1] - pa[1]) * db[0]) / den
    corner = (pa[0] + da[0] * k, pa[1] + da[1] * k)
    t = r * math.tan(math.radians(abs(bend['angle'])) / 2.0)
    a_in = (corner[0] - da[0] * t, corner[1] - da[1] * t)
    a_out = (corner[0] + db[0] * t, corner[1] + db[1] * t)
    m = _rot90(da, left=bend['angle'] > 0)
    return a_in, a_out, (a_in[0] + m[0] * r, a_in[1] + m[1] * r), r


def _rise_arcs(run_a, run_b, side, rise):
    """One boundary's S at a rise: the two tangent arcs of radius `rise['radius']`, the
    station on `run_a` where its straight ends, and the station on `run_b` where it resumes.
    Both arcs sweep `theta` - the first toward the rise, the second back - so the boundary
    leaves parallel to how it arrived, `rise['rise']` mm across (EXO-114)."""
    if abs(run_a['d'][0] - run_b['d'][0]) > 1e-9 or abs(run_a['d'][1] - run_b['d'][1]) > 1e-9:
        raise ValueError('a rise does not turn the route: the runs it joins must be parallel')
    if abs(run_a['thickness'] - run_b['thickness']) > 1e-9:
        raise ValueError('a rise keeps the ribbon its thickness: %.3f -> %.3f'
                         % (run_a['thickness'], run_b['thickness']))
    r, th, sg = rise['radius'], rise['theta_deg'], rise['sign']
    d, n = run_a['d'], run_a['n']
    stagger = run_a['half'] * math.tan(math.radians(th) / 2.0)
    a_start = run_a['length'] - side * sg * stagger
    P = _wall(run_a, side, a_start)
    c1 = (P[0] + sg * n[0] * r, P[1] + sg * n[1] * r)
    E2 = (P[0] + d[0] * rise['horiz'] + n[0] * rise['rise'],
          P[1] + d[1] * rise['horiz'] + n[1] * rise['rise'])
    c2 = (E2[0] - sg * n[0] * r, E2[1] - sg * n[1] * r)
    E1 = ((c1[0] + c2[0]) / 2.0, (c1[1] + c2[1]) / 2.0)
    segs = []
    _sweep_arc(segs, c1, r, P, sg * th)
    _sweep_arc(segs, c2, r, E1, -sg * th)
    return a_start, segs, _along(run_b, E2)


def _rotor_join_boundaries(route, run0):
    """What the rotor join contributes: (left prefix segments, where the left boundary of
    run 0 starts, right suffix segments, where its right boundary starts, root line, fit
    lengths). The left chain leaves the root and the right chain comes back to it, so the
    profile closes on the root line hidden inside the pad disk."""
    j, land = route['rotor_join'], route['stator_land']
    root_s, pad_t, pad_r = j['root_s'], j['pad_t'], j['pad_r']
    lengths = {}
    if j['kind'] == 'coplanar':
        # EXO-92: the run IS the pad continued outward - both boundaries start on the root
        # line, both pad faces run on flat, the rim is covered over the pad's whole thickness
        # and nothing goes below z = 0. The one thing to check is that the elbow's outer
        # round starts outside the rim, so the rotor-side face crosses it FLAT (EXO-74).
        if pad_r is not None:
            flat = (land['s_far'] - land['corner_r']) - pad_r
            if flat < j['rim_margin'] - 1e-9:
                raise ValueError(
                    'the r %.2f corner round would start %.2f mm outside the r %.1f rim '
                    '(need %.1f), so the plate would not cross it flat'
                    % (land['corner_r'], flat, pad_r, j['rim_margin']))
        return [], 0.0, [], 0.0, ((root_s, 0.0), (root_s, pad_t)), lengths
    # EXO-74 / EXO-83 'ledge'. The pad's OUTER face runs flat from the root out past the rim,
    # turns down onto the run's left boundary through one convex round, and on the way back
    # the run's right boundary rises to the relief, which turns onto the pad's rotor-side
    # plane - the RISER_CLEAR ledge - and runs back over the rim to the root.
    R_jt, R_rl = j['round_r'], j['relief_r']
    s_left, s_right = _wall(run0, +1)[0], _wall(run0, -1)[0]
    lengths['pad outer face to the join round'] = (s_left - R_jt) - root_s
    lengths['pad rotor-side face to the relief'] = (s_right - R_rl) - root_s
    if pad_r is not None:
        lengths['outer face flat past the rim'] = (s_left - R_jt) - (pad_r + j['rim_margin'])
        lengths['flat ledge outside the rim'] = (s_right - R_rl) - pad_r
    left = []
    _line(left, (root_s, pad_t), (s_left - R_jt, pad_t))
    _arc(left, (s_left - R_jt, pad_t - R_jt), R_jt, 90.0, 0.0)
    right = []
    _arc(right, (s_right - R_rl, -R_rl), R_rl, 0.0, 90.0)
    _line(right, (s_right - R_rl, 0.0), (root_s, 0.0))
    return (left, _along(run0, (s_left, pad_t - R_jt)),
            right, _along(run0, (s_right, -R_rl)), ((root_s, 0.0), (root_s, pad_t)), lengths)


def _stator_land_boundaries(route, run):
    """What the stator land contributes: the tangent fillet from the run's slab-side (left)
    boundary into the slab's near face, up past it, across the buried top and down to the
    run's other boundary, where the walk back begins. Returns (segments, left end, right end,
    fit lengths). The run must arrive along +u, since the slab's faces are the planes
    perpendicular to u that `landing_numbers` measured."""
    land = route['stator_land']
    if abs(run['d'][0] - 1.0) > 1e-9:
        raise ValueError('the run reaching the stator slab travels (%.3f, %.3f), not along u: '
                         'the slab faces are perpendicular to u' % run['d'])
    # which boundary the land fillet sits on: the one the slab's own body continues past.
    # +1 (left) for the bar and the shoulder L, whose slab stands above the run; -1 for the
    # ramp, which lands on the disk's rim from above so the slab hangs below (Link 4 R).
    sd = land.get('side', 1)
    d, n, h = run['d'], run['n'], run['half']
    a_face = (land['s_face'] + land['bury_s']) - run['p0'][0]
    a_end = (land['s_face'] + land['bury_depth']) - run['p0'][0]
    R_f, b_up = land['fillet_r'], h + land['fillet_r'] + land['bury_up']

    def pt(a, b):
        b = sd * b
        return (run['p0'][0] + d[0] * a + n[0] * b, run['p0'][1] + d[1] * a + n[1] * b)

    segs = []
    _sweep_arc(segs, pt(a_face - R_f, h + R_f), R_f, pt(a_face - R_f, h), 90.0 * sd)
    _line(segs, pt(a_face, h + R_f), pt(a_face, b_up))       # buried: on up the slab face
    _line(segs, pt(a_face, b_up), pt(a_end, b_up))           # buried: across, inside the slab
    _line(segs, pt(a_end, b_up), pt(a_end, -h))              # the buried end
    if sd < 0:
        segs = _rev_segs(segs)
        return segs, a_end, a_face - R_f, {
            'run buried past the slab near face': a_end - a_face,
            'slab near face past the land fillet': land['bury_up'],
            'buried end down to the run': b_up + h}
    return segs, a_face - R_f, a_end, {
        'run buried past the slab near face': a_end - a_face,
        'slab near face past the land fillet': land['bury_up'],
        'buried end down to the run': b_up + h}


def route_profile(route):
    """The route drawn as ONE closed side profile in the (s, z) plane: the rotor join, the
    ribbon's LEFT boundary out along every run and bend, the stator land, the RIGHT boundary
    back, and the root line hidden inside the rotor pad. Segments as everywhere else in this
    module - ('line', p0, p1) / ('arc', p0, p_mid, p1, centre, r) - consecutive ones sharing
    endpoints, every arc tangent to its neighbours.

    This is the only side-profile renderer there is: `bar_side_profile` and `l_side_profile`
    are now the two routes that used to be patterns, expressed as data (EXO-96).
    Raises ValueError, naming every part that failed, when a straight part would have zero
    or negative length."""
    runs = route_runs(route)
    left_pre, a_left, right_suf, a_right, root, lengths = _rotor_join_boundaries(route, runs[0])
    land_segs, a_land_l, a_land_r, land_lengths = _stator_land_boundaries(route, runs[-1])
    lengths.update(land_lengths)
    chains = {+1: [], -1: []}
    starts = {+1: a_left, -1: a_right}
    for i, run in enumerate(runs):
        for side in (+1, -1):
            a0, bend, rise = starts[side], run['bend_out'], run['rise_out']
            arcs, resume = None, None
            if bend is not None:
                a_in, a_out, c, r = _bend_fillet(run, runs[i + 1], side, bend)
                a1 = _along(run, a_in)
            elif rise is not None:
                a1, arcs, resume = _rise_arcs(run, runs[i + 1], side, rise)
            else:
                a1 = a_land_l if side > 0 else a_land_r
            lengths['%s %s boundary' % (run['label'] or 'run %d' % i,
                                        'left' if side > 0 else 'right')] = a1 - a0
            _line(chains[side], _wall(run, side, a0), _wall(run, side, a1))
            if bend is not None:
                _sweep_arc(chains[side], c, r, a_in, bend['angle'])
                starts[side] = _along(runs[i + 1], a_out)
            elif rise is not None:
                chains[side] += arcs
                starts[side] = resume
    bad = ['%s %.2f mm' % (k, v) for k, v in sorted(lengths.items())
           if v <= ROUTE_MIN_STRAIGHT]
    if bad:
        raise ValueError('%s route does not fit: %s'
                         % (route.get('pattern', 'link'), ', '.join(bad)))
    segs = left_pre + chains[+1] + land_segs + _rev_segs(chains[-1]) + right_suf
    _line(segs, root[0], root[1])
    if not profile_closed(segs):
        raise ValueError('route side profile does not close')
    return segs


def route_corner_cut(route, runs=None):
    """The corner round at the stator slab's FAR face, as a cut outline in the side-profile
    plane: the bar's r 7.9 tongue-end round and the shoulder L's r 5.5 elbow round are the
    same feature in the same place, on the boundary of the last run that faces away from the
    slab. Returns (outline, s_corner, b_far)."""
    runs = route_runs(route) if runs is None else runs
    land, run = route['stator_land'], runs[-1]
    sd = land.get('side', 1)
    s_corner = land['s_far']
    b_far = _wall(run, -sd)[1]
    return (tongue_round_outline(s_corner, b_far, land['corner_r'], sd), s_corner, b_far)


HULL_SLICES = 24            # strips per disk / slab; see route_hulls
HULL_STEP = 0.5             # mm between sampled points of the slab outline
STOP_BLOCK_Z = -STOP_H      # deepest the link stop block and the key pegs reach below the
                            # rotor pad's rotor-side plane (BuildArm builds both from LINKS;
                            # every exo link's block is at most this deep)


def _disk_hulls(pad_r, z_lo, z_hi, n=HULL_SLICES):
    """A disk of radius `pad_r` about axis A over z `z_lo`..`z_hi`, as `n` boxes sliced along
    s. Each strip's half width is the disk's at the station NEAREST axis A, so the strip
    contains the disk over its own s range and the union is a hull of the whole disk."""
    out = []
    for k in range(n):
        s0 = -pad_r + 2.0 * pad_r * k / n
        s1 = -pad_r + 2.0 * pad_r * (k + 1) / n
        s_near = 0.0 if s0 <= 0.0 <= s1 else min(abs(s0), abs(s1))
        out.append(((s0, s1), (z_lo, z_hi), (0.0, math.sqrt(max(0.0, pad_r ** 2 - s_near ** 2)))))
    return out


def _slab_hulls(route, n=HULL_SLICES):
    """The stator slab (disk + tongue + blends, `flange` swept over the slab's thickness) as
    boxes in the LINK frame, sliced along z.

    The snapped slab frame B' always lines up with the link frame - its y' is the pad axis by
    construction and its z' is the stator axis, which lies in the pad plane, so x' is the
    lateral - which is why one slice per z band is tight rather than a bounding box around a
    tilted disk. `snap_rad` measures B' against the TRUE B (where the holes are cut), not
    against the link."""
    num, land = route['numbers'], route['stator_land']
    u, w_hat, t_ab = route['u'], route['w'], num['t_ab']
    # the frame the slab BODY is drawn in: the snapped frame, or the route's own when it
    # turns it (the ramp's tongue leaves the disk away from the connector - EXO-114)
    x_s, y_s, z_s = route.get('slab_axes') or (num['x_s'], num['y_s'], num['z_s'])
    pts, bulge = [], 0.0
    for seg in land['flange']:
        if seg[0] == 'arc':
            # a chord cuts INSIDE its arc: the sagitta is how far the real outline bulges
            # past the sampled points, and every box is grown by the largest one so that it
            # stays a hull of the curve and not of the polygon (EXO-103)
            bulge = max(bulge, seg[5] - math.sqrt(max(0.0, seg[5] ** 2 - (HULL_STEP / 2) ** 2)))
        for (xp, yp) in _seg_points(seg, HULL_STEP):
            for zp in (STATOR_UNDERSIDE_Z - PAD_T, STATOR_UNDERSIDE_Z):
                p = [t_ab[i] + xp * x_s[i] + yp * y_s[i] + zp * z_s[i] for i in range(3)]
                pts.append((dot(p, u), dot(p, w_hat), p[2]))
    z_lo = min(p[2] for p in pts) - bulge
    z_hi = max(p[2] for p in pts) + bulge
    pad = (z_hi - z_lo) / n
    out = []
    for k in range(n):
        a, b = z_lo + (z_hi - z_lo) * k / n, z_lo + (z_hi - z_lo) * (k + 1) / n
        band = [p for p in pts if a - pad <= p[2] <= b + pad]
        if not band:
            continue
        ss = [p[0] for p in band]
        ws = [p[1] for p in band]
        out.append(((min(ss) - bulge, max(ss) + bulge), (a, b),
                    ((min(ws) + max(ws)) / 2.0, (max(ws) - min(ws)) / 2.0 + bulge)))
    return out


def _taper_hulls(taper, profile, n=HULL_SLICES):
    """The ribbon of a TAPERED route, as boxes sliced along s: per slice the profile's own
    z envelope and the plan band's own w envelope. The run boxes of `route_hulls` are drawn
    across the straight band only, and a taper leaves material outside it - on Link 4 R's
    pair the plate follows the rotor rim out to w 23.89 where the straight wall is at 17 -
    so this covers the ribbon exactly rather than growing every other box (EXO-114)."""
    bulge = 0.0
    for seg in list(profile) + list(taper['band']):
        if seg[0] == 'arc':
            bulge = max(bulge, seg[5] - math.sqrt(max(0.0, seg[5] ** 2 - (HULL_STEP / 2) ** 2)))
    prof = [q for seg in profile for q in _seg_points(seg, HULL_STEP)]
    band = [q for seg in taper['band'] for q in _seg_points(seg, HULL_STEP)]
    s_lo = max(min(q[0] for q in prof), min(q[0] for q in band))
    s_hi = min(max(q[0] for q in prof), max(q[0] for q in band))
    if s_hi <= s_lo:
        raise ValueError('the plan band (s %.2f..%.2f) does not overlap the side profile '
                         '(s %.2f..%.2f)' % (min(q[0] for q in band), max(q[0] for q in band),
                                             min(q[0] for q in prof), max(q[0] for q in prof)))
    out, pad = [], (s_hi - s_lo) / n
    for k in range(n):
        a, b = s_lo + (s_hi - s_lo) * k / n, s_lo + (s_hi - s_lo) * (k + 1) / n
        zs = [q[1] for q in prof if a - pad <= q[0] <= b + pad]
        ws = [q[1] for q in band if a - pad <= q[0] <= b + pad]
        if not zs or not ws:
            continue
        out.append(((a, b), (min(zs) - bulge, max(zs) + bulge),
                    ((min(ws) + max(ws)) / 2.0, (max(ws) - min(ws)) / 2.0 + bulge)))
    return out


def route_hulls(route, runs, w_env):
    """Over-large boxes containing the WHOLE rendered body, for `link_clearance`'s
    pure-Python sampler. Fillets only remove material, so the boxes stay a hull and the
    sampled clearance stays a LOWER bound. Each box is (s range, z range, (w centre, w half))
    in the link frame.

    In order: the rotor pad disk (with the stop block and key pegs under it), the rotor
    join's own material, one box per run - the run's own boundaries across, and along it from
    whatever bounds it at each end - and the run's buried end inside the slab. EXO-102 found
    the pad disk, the buried end and the stator slab all missing, so the sampler stopped being
    a lower bound the moment a third link could face one of them from the other side.

    The SLAB is `_slab_hulls`, kept apart under `slab_hulls` because `link_clearance` already
    sweeps its outline EXACTLY when the body is the source of the comparison; strip boxes are
    only needed when it is the target, and sampling their surfaces as if they were the body
    would move the source's own points off it (link_12's would drift 1.5 mm into link_23's
    riser and report a collision that is not there). Together the two lists cover the whole
    rendered body: `tools/bridge_scripts/hull_coverage.py` checks that point by point."""
    j, land = route['rotor_join'], route['stator_land']
    half_w = route['width'] / 2.0
    parts = _disk_hulls(j['pad_r'], STOP_BLOCK_Z, j['pad_t'])
    if j['kind'] == 'ledge':
        # the pad plate, centred on axis A across the band, and the relief - the one piece of
        # body INSIDE the run's right boundary, which needs its own (over-large) box
        s_left, s_right = _wall(runs[0], +1)[0], _wall(runs[0], -1)[0]
        parts.append(((j['root_s'], s_left), (0.0, j['pad_t']), (0.0, half_w)))
        parts.append(((s_right - j['relief_r'], s_right), (-j['relief_r'], 0.0), w_env))
    for i, run in enumerate(runs):
        axis = 0 if abs(run['d'][0]) > 0.5 else 1          # the run travels along s or z
        lo = [_wall(run, +1)[1 - axis], _wall(run, -1)[1 - axis]]
        ends = []
        if i == 0:
            ends += [j['root_s']] if axis == 0 else [0.0, j['pad_t']]
        else:
            prev = runs[i - 1]
            ends += [_wall(prev, +1)[axis], _wall(prev, -1)[axis]]
        if i == len(runs) - 1:
            ends += [land['s_face'] + land['bury_depth']]
        else:
            nxt = runs[i + 1]
            ends += [_wall(nxt, +1)[axis], _wall(nxt, -1)[axis]]
        span = (min(ends), max(ends))
        cr = (min(lo), max(lo))
        parts.append((span, cr, w_env) if axis == 0 else (cr, span, w_env))
        # the bend's own corner: neither run's box reaches the material the two fillets leave
        # outside the corner (on link_23 that is 12.5 mm of the r 12.5 outer fillet, 3.6 mm
        # deep into free space). Each fillet's tangent points and its centre bound it.
        if run['bend_out'] is not None:
            bs, bz = [], []
            for side in (+1, -1):
                a_in, a_out, c, _r = _bend_fillet(run, runs[i + 1], side, run['bend_out'])
                bs += [a_in[0], a_out[0], c[0]]
                bz += [a_in[1], a_out[1], c[1]]
            parts.append(((min(bs), max(bs)), (min(bz), max(bz)), w_env))
        # a rise's four arcs: each sweeps less than 90 deg, so its two endpoints and its
        # centre bound it, exactly as a bend fillet's do
        if run['rise_out'] is not None:
            bs, bz = [], []
            for side in (+1, -1):
                _a, arcs, _res = _rise_arcs(run, runs[i + 1], side, run['rise_out'])
                for sg in arcs:
                    bs += [sg[1][0], sg[3][0], sg[4][0]]
                    bz += [sg[1][1], sg[3][1], sg[4][1]]
            parts.append(((min(bs), max(bs)), (min(bz), max(bz)), w_env))
    # the buried end, for BOTH bury depths (EXO-102: the bar's half-depth case used to drop
    # out, leaving the 9.5 mm of profile that climbs the slab's near face outside every box)
    # It starts at the land FILLET's tangent point, not at the slab face: the fillet climbs
    # from the run's boundary to `fillet_r` above it over the `fillet_r` before that face, and
    # the run's own box stops at the boundary.
    run = runs[-1]
    b_up = run['half'] + land['fillet_r'] + land['bury_up']
    # across the band it takes the same envelope the runs do: an S is still turning inside
    # the land fillet's own s range (link_23's reaches w +7.22 where the landed band is at
    # w_off -6.24 +/- 12.5), and without an S the envelope IS the ribbon
    parts.append(((land['s_face'] - land['fillet_r'], land['s_face'] + land['bury_depth']),
                  (_wall(run, -1)[1], run['p0'][1] + b_up), w_env))
    return parts


def route_describe(route):
    """The route as one line per segment, for reports and the pair survey."""
    out = ['%-9s width %.0f, %s rotor join at root s=%.2f (pad r %.1f)'
           % ('route', route['width'], route['rotor_join']['kind'],
              route['rotor_join']['root_s'], route['rotor_join']['pad_r'])]
    runs = route_runs(route)
    out.append('%-9s start (s, z) = (%.3f, %.3f) heading (%.0f, %.0f)'
               % ('', route['start'][0], route['start'][1],
                  route['start_dir'][0], route['start_dir'][1]))
    i = 0
    for seg in route['segments']:
        if seg['kind'] == 'run':
            r = runs[i]
            out.append('%-9s %-6s length %8.3f, thickness %5.3f, (%.3f, %.3f) -> (%.3f, %.3f)'
                       % ('Run', seg['label'], seg['length'], seg['thickness'],
                          r['p0'][0], r['p0'][1], r['p1'][0], r['p1'][1]))
            i += 1
        elif seg['kind'] == 'rise':
            th, horiz = rise_numbers(seg['rise'], seg['radius'])
            out.append('%-9s %-6s %+.3f mm across the side plane through two tangent arcs '
                       'r %.1f, %.4f deg each, %.3f mm of run'
                       % ('Rise', seg['label'], seg['rise'], seg['radius'], th, horiz))
        else:
            rc, dis = bend_centre_radius(seg, runs[i - 1]['thickness'])
            out.append('%-9s %-6s %+.1f deg in the %s plane, walls r %.1f (left) / %.1f '
                       '(right), centre-line r %.2f (walls disagree by %.2f)'
                       % ('Bend', seg['label'], seg['angle'], seg['plane'], seg['r_left'],
                          seg['r_right'], rc, dis))
    sc = route.get('s_curve')
    if sc:
        out.append('%-9s %-6s offset %+.3f mm, r %.1f, %.3f deg, run %.3f mm, s %.3f..%.3f'
                   % ('S', sc['placement'], sc['w_off'], sc['r'], sc['theta_deg'],
                      sc['length'], sc['s_start'], sc['s_end']))
    land = route['stator_land']
    out.append('%-9s slab near face s=%.2f, fillet r %.1f, buried %.2f mm in (%.1f up), '
               'corner round r %.2f cut %.0f wide'
               % ('Land', land['s_face'], land['fillet_r'], land['bury_depth'],
                  land['bury_up'], land['corner_r'], land['corner_cut_w']))
    return out


DEPARTURE_SIZER = {'axial': _axial_route, 'in-plane': _inplane_route,
                   'ramp': _ramp_route}


def render_route(route):
    """Draw a route: the geometry dict BuildArm builds from, for ANY route. Nothing here
    knows about bars or shoulders - it renders runs, bends, the two joins and the plan band.

    Returns the same keys the pattern builders returned (so every measurement script and the
    build are unchanged), plus the route's own: `offset_taken`, `profile_w` (the w the side
    profile is drawn on), `corner_cut` / `corner_cut_w` / `corner_cut_w_centre` (the round at
    the slab's far face) and `route_segments`.
    Raises ValueError, naming the part, when the route cannot be drawn."""
    runs = route_runs(route)
    land, j = route['stator_land'], route['rotor_join']
    sc, w_off = route['s_curve'], route['w_off']
    taper = route.get('taper')
    takes = route.get('offset_device') or JOIN_TAKES_OFFSET[j['kind']]
    if takes not in JOIN_OFFSET_DEVICES[j['kind']]:
        raise ValueError('a %r rotor join cannot take the offset in a %r: it allows %s'
                         % (j['kind'], takes, ' / '.join(JOIN_OFFSET_DEVICES[j['kind']])))
    half_w = route['width'] / 2.0
    profile = route_profile(route)
    corner_cut, s_corner, _b_far = route_corner_cut(route, runs)
    # the S envelope is a conservative (over-wide) hull, so the sampled clearance stays a
    # lower bound; without an S the band is the ribbon itself, and a taper's straight band is
    # its own (what the taper leaves OUTSIDE that band gets `_taper_hulls`)
    w_env = ((w_off / 2.0, half_w + abs(w_off) / 2.0) if sc else
             ((taper['centre'], half_w) if taper else (0.0, half_w)))
    num = route['numbers']
    info = {
        'pattern': route['pattern'], 'departure': route['departure'],
        'width': route['width'], 'u': route['u'], 'w': route['w'], 'w_off': w_off,
        'zL': route['zL'], 'landing': num['landing'], 's_land': num['s_land'],
        'root_s': j['root_s'], 's_face': land['s_face'], 's_far': s_corner,
        'profile': profile, 'flange': land['flange'], 'y_end': land['y_end'],
        'y_blend': land['y_blend'], 'edges': land['edges'],
        'corner_cut': corner_cut, 'corner_cut_w': land['corner_cut_w'],
        'corner_cut_w_centre': w_off if takes == 's-bend' else 0.0,
        'profile_w': 0.0 if (sc or taper or takes == 'slab') else w_off,
        'offset_taken': takes, 'route_segments': route['segments'],
        'R_b': num['R_b'], 'snap_rad': num['snap_rad'],
        's_curve': sc, 'taper': taper, 'placement': route['placement'],
        'hull_parts': route_hulls(route, runs, w_env)
                      + (_taper_hulls(taper, profile) if taper else []),
        # the slab's own strip boxes, for when this body is the TARGET of a clearance
        # comparison; as the source its outline is swept exactly (EXO-102 / EXO-103)
        'slab_hulls': _slab_hulls(route),
        # the slab as built, for the pure-Python clearance sampler: `flange` swept over the
        # slab's own thickness in B
        'slab_faces': (STATOR_UNDERSIDE_Z - PAD_T, STATOR_UNDERSIDE_Z),
    }
    info.update(route['extra'])
    # what the side profile is extruded against: the S's band, the taper's band, or nothing.
    # One key, so `BuildArm.build_link_body` has no branch on which device a route uses.
    band = sc or taper
    if band is not None:
        zs = [q[1] for seg in profile for q in _seg_points(seg, 1.0)]
        ws = [abs(q) for q in band['w_range']]
        band.setdefault('wide_w', 2.0 * max(ws) + 2.0)
        band.setdefault('z_span', (max(zs) + 2.0) - (min(zs) - 2.0))
        band.setdefault('z_mid', ((max(zs) + 2.0) + (min(zs) - 2.0)) / 2.0)
    info['plan_band'] = band
    # EXO-74/EXO-77: everything the link carries BELOW the stator stop block's top face turns
    # past that block's r 27 annulus, so it must keep MIN_CLEARANCE to r 27 there. Measured on
    # the profile that was just drawn (the plan S only moves w, never s). A route that keeps
    # everything above that plane - the coplanar one does - has nothing to check.
    r_keep = max(MODULE_R, STATOR_PAD_R)
    if min(q[1] for seg in profile for q in _seg_points(seg, 1.0)) < MODULE_STOP_Z_TOP:
        clr_stop, at_stop = riser_radial_clearance(info, r_keep=r_keep,
                                                   z_top=MODULE_STOP_Z_TOP)
        if clr_stop < MIN_CLEARANCE:
            raise ValueError('the body reaches %.2f mm from r %.1f at s=%.2f z=%.2f, inside '
                             'the stator stop block (top face z=%.1f); need >= %.1f'
                             % (clr_stop, r_keep, at_stop[0], at_stop[2], MODULE_STOP_Z_TOP,
                                MIN_CLEARANCE))
        info['clearance_stop'] = clr_stop
    # Both names the pattern builders used for that one corner round, so the measured-body
    # checks and the tests read the key they always did. They are ALIASES of `corner_cut`,
    # set unconditionally: this renderer has no branch on the route's name anywhere.
    info['tongue_round'] = info['round_cut'] = corner_cut
    return info


# ---------------- the flat seating ring every screw hole needs (EXO-124) ----------------
# Esteban found it by eye at the J4 join: the ramp's lift-off tangent landed on the rotor
# pad's outermost screw hole, so a screw head would have seated on the start of the r 20
# curve instead of on plane. The rule that makes explicit is a ROUTE CONSTRAINT, not a
# drawing detail - on every SEATING face of both pads the body must stay planar out to each
# hole's edge plus HOLE_FLAT_RING - and `route_path` refuses a route that breaks it by name.
# Nothing is reshaped quietly.
#
# The seating faces are the ones something BEARS on:
#   rotor pad, outer face z = pad_t     the four M3 heads seat here;
#   rotor pad, rotor-side face z = 0    this is what sits on the module's rotor;
#   stator slab, FAR face               this is what bolts to module B (EXO-121).
# The slab's NEAR face is the one the link's OWN run enters through - `Link 4 R` runs
# straight through it, and link_45's land fillet crosses an insert hole there by 2.85 mm -
# so nothing bears on it: it is measured and reported, and not enforced.
#
# What bounds the flat region on a face is always a curve this module already draws, so the
# ring is a DISTANCE and not a sampled walk:
#   * the pad outline - the rotor disk's rim, the stator slab's flange outline;
#   * the line of constant s where the ribbon LEAVES that face's own plane (the ramp's
#     lift-off tangent, the shoulder L's elbow fillet, the ledge join's round), across the
#     plan band and no wider;
#   * on the rotor-side face, the link's own stop block and key pegs, which stand proud of
#     it (EXO-364: the pegs are 7.514 off every hole edge; the old index tab held links to 2.35 mm);
#   * on a slab face, the run's footprint with the fillet or the corner round beside it;
#   * the actuator cutout, in the TRUE stator frame B where it is cut.
# `tools/bridge_scripts/check_hole_margins.py` measures the same rings on the BUILT body,
# off each seating face's own edges, and fails if the two disagree by more than 0.02 mm.


def _seg_point_dist(p, sg):
    """Distance from p to one profile segment - ('line', a, b) or ('arc', p0, pm, p1, c, r)."""
    if sg[0] == 'line':
        (ax, ay), (bx, by) = sg[1], sg[2]
        dx, dy = bx - ax, by - ay
        den = dx * dx + dy * dy
        t = 0.0 if den <= 0.0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / den))
        return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))
    _, p0, pm, p1, c, r = sg
    a0, am, a1 = _ang(c, p0), _ang(c, pm), _ang(c, p1)
    ap = _ang(c, p)
    ccw = (am - a0) % 360.0 <= (a1 - a0) % 360.0
    span = ((a1 - a0) % 360.0) if ccw else ((a0 - a1) % 360.0)
    off = ((ap - a0) % 360.0) if ccw else ((a0 - ap) % 360.0)
    if off <= span + 1e-9:
        return abs(math.hypot(p[0] - c[0], p[1] - c[1]) - r)
    return min(math.hypot(p[0] - p0[0], p[1] - p0[1]), math.hypot(p[0] - p1[0], p[1] - p1[1]))


def outline_point_dist(p, loop):
    """Distance from p to the nearest point of a closed outline (the outlines this module
    draws: lines and arcs). Unsigned - every hole this is used on is inside its own pad."""
    return min(_seg_point_dist(p, sg) for sg in loop)


def _rect_point_dist(p, x0, x1, y0, y1):
    """Distance from p to an axis-aligned rectangle, NEGATIVE (the depth inside) when p is
    in it: a hole whose centre is already in the blocked region has no ring at all."""
    dx, dy = max(x0 - p[0], 0.0, p[0] - x1), max(y0 - p[1], 0.0, p[1] - y1)
    if dx == 0.0 and dy == 0.0:
        return -min(p[0] - x0, x1 - p[0], p[1] - y0, y1 - p[1])
    return math.hypot(dx, dy)


def _sector_point_dist(p, r_in, r_out, ang, half):
    """Distance from a point to an annulus sector (the link's stop block), all in the module
    frame. Zero inside it. The block only ever reaches the hole nearest its own angle - a
    radius-only test would have every link reporting the block as its tightest feature."""
    r = math.hypot(p[0], p[1])
    d_ang = ((math.degrees(math.atan2(p[1], p[0])) - ang + 180.0) % 360.0) - 180.0
    if abs(d_ang) <= half:
        if r_in <= r <= r_out:
            return 0.0
        return r_in - r if r < r_in else r - r_out
    best = None
    for side in (-1.0, +1.0):
        a = math.radians(ang + side * half)
        q0 = (r_in * math.cos(a), r_in * math.sin(a))
        q1 = (r_out * math.cos(a), r_out * math.sin(a))
        d = _seg_point_dist(p, ('line', q0, q1))
        if best is None or d < best:
            best = d
    return best


def plane_flat_reach(profile, z0, root_s, tol=1e-6):
    """How far out along s the side profile stays ON the plane z = z0, starting from the
    root line inside the rotor pad: the station where the ribbon leaves that plane. None
    when the profile never runs along it. This is the ONE number that makes the ramp
    different - its plate leaves the pad's outer plane `thickness/2 * tan(theta/2)` before
    its centre line does, and that is where EXO-124's hole was."""
    spans = []
    for sg in profile:
        if sg[0] != 'line':
            continue
        (s1, z1), (s2, z2) = sg[1], sg[2]
        if abs(z1 - z0) <= tol and abs(z2 - z0) <= tol:
            spans.append((min(s1, s2), max(s1, s2)))
    spans.sort()
    merged = []
    for a, b in spans:
        if merged and a <= merged[-1][1] + 1e-6:
            merged[-1] = (merged[-1][0], max(merged[-1][1], b))
        else:
            merged.append((a, b))
    for a, b in merged:
        if a <= root_s + 1e-6:
            return b
    return None


def band_w_at(route, s):
    """(w_lo, w_hi) of the ribbon's plan band at station s, whichever device carries the
    lateral offset - the tapered band, the S, or neither."""
    geo = route.get('geometry') or {}
    if geo.get('taper'):
        w = taper_w(geo['taper'], s)
        if w is not None:
            return w
    if geo.get('s_curve'):
        w = s_curve_w(geo['s_curve'], s)
        if w is not None:
            return w
    if 'band_w' in geo:
        return tuple(geo['band_w'])
    half = route['width'] / 2.0
    return (-half, half)


# ---------------- the tongue's direction in the landing plate's plane (EXO-176) ----------
# `landing_numbers` pins the landing plate's drawing frame to the rotor pad axis: y_s is
# (0, 0, y_sign) and the tongue is a strip PARALLEL to it. That is one choice out of a whole
# circle, and it is the choice that refuses G1's J6->J7 (EXO-173): the ribbon's arrival
# HEIGHT is then the only way to reach past the actuator cutout, so a pair whose landing sits
# level with the pad has to climb and then come back down.
#
# The plate is a flat disk, so its plane carries TWO free directions, and the tongue may run
# along any of them. `tongue_frame` is that rotation and nothing else: the same snapped frame
# turned `theta` about its own axis Z' = z_s, so that +Y' is the tongue. theta = 0 IS the
# square landing, exactly - every number below reduces to `landing_numbers`' own.
TONGUE_SQUARE = 0.0


def tongue_frame(num, theta_rad):
    """The landing plate's drawing frame turned `theta_rad` about the plate axis.

    Returns (x_t, y_t, z_s) as three vectors in the rotor pad frame A, where +Y' = y_t is the
    tongue's direction and a point of the plate at plate coordinates (x', y') sits at
    x' * x_t + y' * y_t from the plate axis. At theta = 0 this is `num`'s own (x_s, y_s, z_s).
    """
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    x_s, y_s, z_s = num['x_s'], num['y_s'], num['z_s']
    x_t = [c * x_s[i] - s * y_s[i] for i in range(3)]
    y_t = [s * x_s[i] + c * y_s[i] for i in range(3)]
    return x_t, y_t, list(z_s)


def tongue_R(num, theta_rad):
    """`tongue_frame` as the 3x3 the builder sketches the plate on (columns x_t, y_t, z_s)."""
    x_t, y_t, z_s = tongue_frame(num, theta_rad)
    return [[x_t[i], y_t[i], z_s[i]] for i in range(3)]


def tongue_angle(x_plate, y_plate):
    """The tongue's own direction: FROM THE DISK CENTRE AT the point (x', y') where the
    ribbon enters the plate plane (EXO-176). Measured from the snapped frame's +Y, so 0 is
    the square landing and the sign follows +X'."""
    return math.atan2(x_plate, y_plate)


def footprint_extent(half_w, half_t, theta_rad):
    """How far the ribbon's W x T footprint reaches from its own centre, ALONG the tongue and
    ACROSS it, when the tongue runs at `theta_rad` to the plate frame's +Y.

    The ribbon is `RIBBON_T` thick along the pad axis and `width` wide across it, because it
    leaves the rotor pad IN the pad plane and arrives square to the plate; the tongue is what
    turns, so the footprint meets it as a rotated rectangle. At theta = 0 these are half_t
    (along) and half_w (across) - the square landing's own two numbers."""
    c, s = abs(math.cos(theta_rad)), abs(math.sin(theta_rad))
    return half_w * s + half_t * c, half_w * c + half_t * s


TONGUE_END_TOL = 1e-9    # mm; two footprint corners share a wall within this


def tongue_end(half_w, half_t, theta_rad, r_land, x_off=0.0):
    """Where the tongue STOPS: ON THE RIBBON, not on a plane square to itself (EXO-177).

    The ribbon arrives square to the plate, so its footprint is a `width` x `thickness`
    rectangle turned `theta` in the plate's plane. A tongue that ends on a straight line
    square to its own direction has to cover that rectangle's BOUNDING BOX, and the two
    corners of the box the ribbon does not occupy are pure overhang - on G1's J6->J7 one of
    them swings 22.6 mm past the ribbon and down into module J6's own body.

    So the tongue ends on the footprint itself: its two walls run out to the corners where
    the footprint is widest, and between them the outline follows the ribbon's own two outer
    edges. At `theta` = 0 the two wall corners are the footprint's own top corners and the
    walk between them is the single flat line the square tongue always had, so this changes
    NOTHING where the tongue is square.

    Returns (points, y_plus, y_minus, half_w_t): the outline from the +x wall's corner to the
    -x wall's corner, the y where each wall stops, and the tongue's half width."""
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    corners = [(a * c - b * s + x_off, a * s + b * c + r_land)
               for a, b in ((half_w, half_t), (half_w, -half_t),
                            (-half_w, -half_t), (-half_w, half_t))]
    # the extreme-x corners, to a tolerance: at theta = +/- 90 deg two corners share an x to
    # within rounding and the OUTER of the two (larger y) is the one the wall stops at.
    x_hi = max(p[0] for p in corners)
    x_lo = min(p[0] for p in corners)
    i_max = max((i for i in range(4) if corners[i][0] >= x_hi - TONGUE_END_TOL),
                key=lambda i: corners[i][1])
    i_min = max((i for i in range(4) if corners[i][0] <= x_lo + TONGUE_END_TOL),
                key=lambda i: corners[i][1])

    def walk(step):
        out, i = [], i_max
        while True:
            out.append(i)
            if i == i_min:
                return out
            i = (i + step) % 4

    a, b = walk(1), walk(-1)
    best = min((a, b), key=lambda wk: (-max(corners[i][1] for i in wk), len(wk)))
    pts = [corners[i] for i in best]
    return pts, pts[0][1], pts[-1][1], max(p[0] for p in corners) - x_off


def insert_slab_points(num, x_s, y_s, z_s):
    """Every stator insert hole in the SLAB'S OWN drawing frame - `angle`, `slab` = (x, y),
    `b` = (x, y) in the true stator frame B where it is cut, `pa` = the hole axis in the pad
    frame A. The sizers need this BEFORE the route exists, to solve the land radii."""
    t_ab, z_mid = num['t_ab'], STATOR_UNDERSIDE_Z - PAD_T / 2.0
    xb, yb, zb = num['x_b'], num['y_b'], num['z_b']
    origin = [t_ab[i] + z_mid * z_s[i] for i in range(3)]
    out = []
    for a in INSERT_ANGLES:
        bx = INSERT_RADIUS * math.cos(math.radians(a))
        by = INSERT_RADIUS * math.sin(math.radians(a))
        pa = [t_ab[i] + xb[i] * bx + yb[i] * by + zb[i] * z_mid for i in range(3)]
        p = [pa[i] - origin[i] for i in range(3)]
        out.append({'angle': a, 'slab': (dot(p, x_s), dot(p, y_s)), 'b': (bx, by), 'pa': pa})
    return out


def land_radius_fit(nominal, y_end, depth, inserts, x_lo, x_hi, what, ring=None):
    """SOLVE one of the two land radii instead of asserting it afterwards (EXO-124).

    The run enters the slab through its near face and leaves through the far one; on each
    face it blocks the strip `y_end - depth - r .. y_end` across the tongue, where `depth` is
    the run's own thickness at the near face and 0 at the far one (the tongue's end edge is
    rounded there whether the run reaches it or not). The radius is therefore the LAST free
    parameter between the run and an insert hole, and this returns the largest value not
    greater than `nominal` that still leaves every hole its flat seating ring.

    Raises ValueError naming the hole when even a radius of 0 is not enough - that is the
    one case where the route genuinely cannot be drawn, and it says so by name."""
    reach = HOLE_R + (HOLE_FLAT_RING if ring is None else ring)
    r, tight = nominal, None
    for ins in inserts:
        x_h, y_h = ins['slab']
        if x_h < x_lo - reach or x_h > x_hi + reach:
            continue                      # the run's strip never comes near this hole
        lim = y_end - depth - y_h - reach
        if lim < r:
            r, tight = lim, ins
    if r < 0.0:
        raise ValueError(
            'the %s cannot be drawn at any radius: the insert hole at %.0f deg (slab x %.2f, '
            'y %.2f) leaves the run only %.3f mm of flat and %.2f mm are needed (hole edge '
            '%.1f + HOLE_FLAT_RING %.1f), so the %s would have to be r %.3f'
            % (what, tight['angle'], tight['slab'][0], tight['slab'][1], r + reach, reach,
               HOLE_R, HOLE_FLAT_RING if ring is None else ring, what, r))
    return r, tight


def rotor_hole_stations(route):
    """Every rotor-pad screw hole, as a dict: `angle`, `module` = (x, y) in the MODULE frame
    (where the stop block and the key pegs are), `sw` = (s, w) in the route's own frame,
    `at` = a point on the hole's axis and `axis` = its direction, both in the pad frame's
    (s, w, z) - which is what a measurement of the built body matches on."""
    u, w = route['u'], route['w']
    out = []
    for a in ROTOR_HOLE_ANGLES:
        x = ROTOR_HOLE_RADIUS * math.cos(math.radians(a))
        y = ROTOR_HOLE_RADIUS * math.sin(math.radians(a))
        s_h, w_h = x * u[0] + y * u[1], x * w[0] + y * w[1]
        out.append({'angle': a, 'module': (x, y), 'sw': (s_h, w_h),
                    'at': (s_h, w_h, PAD_T / 2.0), 'axis': (0.0, 0.0, 1.0)})
    return out


def stator_hole_stations(route):
    """Every stator-pad insert hole: (angle, (x, y) in the SLAB's own drawing frame - where
    the flange outline and the tongue are - and (x_b, y_b) in the TRUE stator frame B, where
    the holes and the actuator cutout are cut (EXO-61))."""
    num, geo = route['numbers'], route['geometry']
    # the slab's own drawing frame, which is the one BuildArm sketches the flange in - NOT
    # `landing_numbers`' snapped frame: the ramp turns it 180 deg about its axis so the
    # tongue leaves the disk away from the connector (EXO-114).
    R_s = geo['R_b']
    xs = [R_s[i][0] for i in range(3)]
    ys = [R_s[i][1] for i in range(3)]
    zs = [R_s[i][2] for i in range(3)]
    u, w = route['u'], route['w']
    out = []
    for ins in insert_slab_points(num, xs, ys, zs):
        pa = ins['pa']
        out.append({'angle': ins['angle'], 'slab': ins['slab'], 'b': ins['b'],
                    'at': (dot(pa, u), dot(pa, w), pa[2]),
                    'axis': (dot(zs, u), dot(zs, w), zs[2])})
    return out


def hole_flat_rings(route, ring=None, stop=None):
    """The flat seating ring every screw hole on this route actually has, in mm beyond the
    hole edge, measured to the nearest thing that bounds the flat region of its face.

    Returns one dict per (hole, face): pad ('rotor' / 'stator'), angle, face
    ('outer' / 'rotor-side' / 'far' / 'near'), seating (whether anything bears on that face),
    ring, need and bound (what the ring is measured to). Negative means the flat ends inside
    the hole's own edge - the defect EXO-124 was opened for.

    `stop` = (angle, half angle, inner radius) of the link's own stop block, which stands
    proud of the rotor-side face; None uses this module's constants, which are the most
    conservative placement (BuildArm gives each link its own).

    `route['geometry']` must already be drawn (`render_route`)."""
    need = HOLE_FLAT_RING if ring is None else ring
    geo = route['geometry']
    land, joins = route['stator_land'], route['rotor_join']
    pad_r, root_s = joins['pad_r'], joins['root_s']
    out = []

    # ---- the rotor pad: the two faces of the plate the four M3 screws go through ----
    stop_ang, stop_half, stop_r = (STOP_ANGLE, STOP_HALF_ANGLE, STOP_R_IN) if stop is None \
        else stop
    zs_prof = [q[1] for sg in geo['profile'] for q in (sg[1:4] if sg[0] == 'arc' else sg[1:3])]
    rotor_holes = rotor_hole_stations(route)
    for face, z0, out_n in (('outer', joins['pad_t'], +1.0), ('rotor-side', 0.0, -1.0)):
        reach = plane_flat_reach(geo['profile'], z0, root_s)
        # the pad DISK carries this plane out to the rim whatever the ribbon does, so where
        # the ribbon leaves the plane only interrupts the face it leaves TOWARD: the ramp
        # climbs, so it takes the outer face and never the rotor-side one (nothing of a
        # coplanar join goes below z = 0 at all).
        leaves = any(out_n * (z - z0) > 1e-9 for z in zs_prof)
        for hole in rotor_holes:
            a, (mx, my), (s_h, w_h) = hole['angle'], hole['module'], hole['sw']
            r_h = math.hypot(mx, my)
            bounds = [('the r %.1f pad rim' % pad_r, pad_r - r_h)]
            if reach is not None and leaves:
                w_lo, w_hi = band_w_at(route, reach)
                bounds.append(('the ribbon leaving the plane at s = %.3f' % reach,
                               _rect_point_dist((s_h, w_h), reach, reach + 1e6, w_lo, w_hi)))
            for other in rotor_holes:
                if other is hole:
                    continue
                bounds.append(('the hole at %.0f deg' % other['angle'],
                               math.hypot(other['module'][0] - mx, other['module'][1] - my)
                               - HOLE_R))
            if face == 'rotor-side':
                # the link's own stop block and key pegs (EXO-364) stand proud of this face
                bounds.append(('the key pegs at r %.1f' % KEY_R, key_peg_dist((mx, my))))
                # EXO-218: the stop is one solid sector again, so that is what a screw
                # head is measured to.
                bounds.append(('the stop block at r %.1f..%.1f, %.2f +/- %.2f deg'
                               % (stop_r, pad_r, stop_ang, stop_half),
                               _sector_point_dist((mx, my), stop_r, pad_r, stop_ang,
                                                  stop_half)))
            d, why = min(bounds, key=lambda b: b[1])[1], min(bounds, key=lambda b: b[1])[0]
            out.append({'pad': 'rotor', 'angle': a, 'face': face, 'seating': True,
                        's': s_h, 'w': w_h, 'at': hole['at'], 'axis': hole['axis'],
                        'plane': ('z', z0), 'ring': d - HOLE_R, 'need': need, 'bound': why})

    # ---- the stator slab: the face that bolts to module B, and the one the run enters ----
    runs = route_runs(route)
    run = runs[-1]
    sd = land.get('side', 1)
    y_end, x_walls = land['y_end'], [e['x'] for e in land['edges']]
    x_lo = min(x_walls) - TONGUE_ROUND_MARGIN
    x_hi = max(x_walls) + TONGUE_ROUND_MARGIN
    # the run reaches the far face only when it is buried the whole way through; either way
    # the tongue's end edge is rounded there, and the flat starts corner_r inside it
    faces = (('far', y_end - land['corner_r'], True),
             ('near', y_end - run['thickness'] - land['fillet_r'], True))
    stator_holes = stator_hole_stations(route)
    for hole in stator_holes:
        a, (x_h, y_h), (bx, by) = hole['angle'], hole['slab'], hole['b']
        others = [('the insert at %.0f deg' % o['angle'],
                   math.hypot(o['slab'][0] - x_h, o['slab'][1] - y_h) - HOLE_R)
                  for o in stator_holes if o is not hole]
        for face, y_flat, seating in faces:
            bounds = others + [
                     ('the slab outline', outline_point_dist((x_h, y_h), land['flange'])),
                      # the XL330's own edge (m=0): this ring table is the OLD router's and
                      # every number in it was read against that datum. EXO-187's grown
                      # opening is checked in sweepgen, on the general router (EXO-187).
                      ('the actuator cutout',
                       _rect_point_dist((bx, by), *actuator_rect(0.0))),
                      ('the run and its r %.1f %s'
                       % (land['corner_r'] if face == 'far' else land['fillet_r'],
                          'corner round' if face == 'far' else 'land fillet'),
                       _rect_point_dist((x_h, y_h), x_lo, x_hi, y_flat, y_end))]
            d, why = min(bounds, key=lambda b: b[1])[1], min(bounds, key=lambda b: b[1])[0]
            out.append({'pad': 'stator', 'angle': a, 'face': face, 'seating': seating,
                        'x': x_h, 'y': y_h, 'at': hole['at'], 'axis': hole['axis'],
                        'plane': ('s', land['s_far'] if face == 'far' else land['s_face']),
                        'ring': d - HOLE_R, 'need': need, 'bound': why})
    return out


def check_hole_flat_rings(route, ring=None, stop=None):
    """Refuse a route whose screw holes do not all have their flat seating ring (EXO-124).
    Raises ValueError naming every hole that is short, the face, the number it has, the
    number it needs and what takes it away. Returns the full table on success."""
    rings = hole_flat_rings(route, ring, stop)
    bad = [h for h in rings if h['seating'] and h['ring'] < h['need'] - 1e-9]
    if bad:
        raise ValueError(
            'a screw head would not seat flat: %s (HOLE_FLAT_RING = %.2f mm beyond the '
            'hole edge is the rule)'
            % ('; '.join('the %s pad hole at %.0f deg has %.3f mm of flat ring on its %s '
                         'face, taken by %s' % (h['pad'], h['angle'], h['ring'], h['face'],
                                                h['bound']) for h in bad),
               bad[0]['need']))
    return rings


def rotor_flat_needed(u, ring=None):
    """How far out along s the rotor pad must stay flat for the screw heads: the OUTERMOST
    hole's edge plus HOLE_FLAT_RING, in the route's own frame. A route that climbs out of
    the pad plane sizes its climb from this, so the constraint shapes the route instead of
    only judging it afterwards (EXO-124)."""
    need = HOLE_FLAT_RING if ring is None else ring
    s = max(ROTOR_HOLE_RADIUS * math.cos(math.radians(a)) * u[0]
            + ROTOR_HOLE_RADIUS * math.sin(math.radians(a)) * u[1]
            for a in ROTOR_HOLE_ANGLES)
    return s + HOLE_R + need


# ---------------- the router (EXO-81, generalised by EXO-96) ----------------
# Both G1 and Gento Luna put every consecutive pair of joint axes exactly perpendicular with
# the stator axis in the rotor pad plane (EXO-88), so a pair is fully described by
# landing_numbers' three numbers, and what follows from them is not a shape but a DEPARTURE -
# how the ribbon leaves the rotor pad:
#   'axial'     the landing is well below the pad: leave along the pad axis, bend, run out
#               (link_23, J2->J3 and J4->J5's shape)
#   'in-plane'  the landing is inside the pad's own thickness, or straight up the slab's own
#               plane from it: leave in the pad plane, no bend, no riser (link_12, J1->J2 and
#               J5->J6; link_34's column, EXO-103)
#   'ramp'      the slab stands edge on at the pad's OWN level, far out, with its actuator
#               cutout across the pad plane: leave in the pad plane, climb over the cutout
#               through a side-plane S and land on the disk's rim (link_45, J4->J5; measured
#               from `Link 4 R`, EXO-114). Tried last, so a pair the bar can take is still a
#               bar - which is what `Link 2 R` is.
# Anything else RAISES, listing what each departure objected to. There is no fallback: a pair
# the router cannot size is a pair nobody builds until its route exists (EXO-82).
PATTERNS = ('bar', 'L', 'ramp')   # the names of the routes, in the order they are tried


def route_path(R_ab, t_ab, pad_r=ROTOR_PAD_R, width=None, placement=None,
               departures=DEPARTURES):
    """Compute the ROUTE for one joint pair: the path a ribbon of `width` mm is swept along,
    from the rotor-pad join of joint N to the stator-pad join of joint N+1 (EXO-96).

    `R_ab`, `t_ab` = the stator frame B in the rotor pad frame A (mm), as `relative_frame`
    returns them. `width` is Esteban's per-link input; None falls back to the approved value
    for the departure (ROUTE_WIDTH_DEFAULT) so that pairs can be surveyed without building.

    Returns the route (see `make_route`), with the drawn geometry under route['geometry'].
    Raises ValueError carrying the pair's three numbers AND every departure's own objection -
    it never falls back to anything, least of all a box."""
    num = landing_numbers(R_ab, t_ab)
    reasons = []
    for dep in departures:
        if dep not in DEPARTURES:
            raise ValueError('unknown departure %r (expected one of %s)'
                             % (dep, ', '.join(DEPARTURES)))
        try:
            route = DEPARTURE_SIZER[dep](
                num, pad_r, ROUTE_WIDTH_DEFAULT[dep] if width is None else width, placement)
            route['geometry'] = render_route(route)
            # EXO-124: every screw hole on either pad needs its flat seating ring. This is a
            # route constraint like the reach or the rim rule - it refuses by name here and
            # never reshapes anything quietly.
            route['geometry']['hole_rings'] = check_hole_flat_rings(route)
        except ValueError as e:
            reasons.append('%s: %s' % (DEPARTURE_PATTERN[dep], e))
            continue
        return route
    raise ValueError(
        'no route fits this joint pair (axis angle %.2f deg, reach s_land %s, level zL %s, '
        'lateral w_off %s). %s'
        % (num['axis_angle_deg'],
           '%.2f' % num['s_land'] if 's_land' in num else 'n/a',
           '%.2f' % num['zL'] if 'zL' in num else 'n/a',
           '%.2f' % num['w_off'] if 'w_off' in num else 'n/a',
           ' || '.join(reasons)))


def _departures_for(patterns):
    for name in patterns:
        if name not in PATTERN_DEPARTURE:
            raise ValueError('unknown pattern %r (expected one of %s)'
                             % (name, ', '.join(PATTERNS)))
    return tuple(PATTERN_DEPARTURE[n] for n in patterns)


def route_relative(R_ab, t_ab, pad_r=ROTOR_PAD_R, placement=None, patterns=PATTERNS,
                   width=None):
    """route() for a pair whose relative frame is already known. Returns a LinkSpec:
    pattern (the route's name), geometry (what BuildArm draws from), numbers
    (landing_numbers), route (the path itself) and the relative frame it came from."""
    route = route_path(R_ab, t_ab, pad_r, width, placement, _departures_for(patterns))
    return {'pattern': route['pattern'], 'geometry': route['geometry'],
            'numbers': route['numbers'], 'route': route,
            'R_ab': R_ab, 't_ab': t_ab, 'pad_r': pad_r}


def route(frame_a, frame_b, pad_r=ROTOR_PAD_R, placement=None, patterns=PATTERNS, width=None):
    """Compute and draw the link body for one joint pair.

    `frame_a` / `frame_b` are (R, t) module frames as `module_frame` returns them (R's
    columns are the module axes in world, t in METERS): frame_a is the rotor-pad side
    (joint N), frame_b the stator-pad side (joint N+1). `pad_r` is the rotor pad radius,
    `width` the ribbon's width (Esteban's per-link input).

    Returns a LinkSpec (see route_relative). Raises ValueError naming EVERY departure's
    objection when no route fits - the router never falls back to a box."""
    R_ab, t_ab = relative_frame(frame_a[0], frame_a[1], frame_b[0], frame_b[1])
    return route_relative(R_ab, t_ab, pad_r, placement, patterns, width)


# ---------------- the module's rotation about its own axis, SOLVED (EXO-130) ----------------
# The URDF fixes a joint's position and its axis. It does not fix the module's rotation ABOUT
# that axis, and that rotation is not cosmetic: it carries the three stator inserts
# (INSERT_ANGLES, a 90 deg gap between 180 and 270 and another between 270 and 0), the
# actuator cutout, the rotor key and the stator's own stop block. Until now it was
# a hand-set `y_hint` copied from the exo STEP, and at J5 that put the 270 deg insert exactly
# under the middle of link_45's landing - which is why EXO-124 had to solve the land fillet
# down to r 0.150 to keep the seat.
#
# So it is solved, on the same footing as the land radii: sweep the rotation, ask the ROUTE
# whether it is feasible, and keep the measured pose only when it is.
#
# THE RULE (one sentence): a module's rotation is the one that lets its link land with BOTH
# land radii at the radius measured on the hand-built part and every screw hole on both pads
# keeping HOLE_FLAT_RING of flat - the measured `y_hint` when that already holds, and
# otherwise the safest point (largest smallest ring) of the feasible window nearest it.
# The coarse sweep's resolution. A feasible window NARROWER than this could be stepped over
# entirely, so it is the claim 'no rotation works' that this number qualifies, not the
# rotation it returns - every window it does find has its edges bisected to SPIN_REFINE. On
# the G1 arm the narrowest window is 1.22 deg (J3, pinned by the bar's web to the actuator
# cutout), so 0.5 has a factor of two in hand; `spin_windows_resolved` is the check.
SPIN_STEP = 0.5
SPIN_REFINE = 1e-4          # deg, bisection tolerance on a window edge
SPIN_ROUND = 6              # deg reported to this many decimals


def spin_frame(R_ab, spin_deg):
    """`R_ab` with the STATOR module B turned `spin_deg` about its own axis."""
    c, s = math.cos(math.radians(spin_deg)), math.sin(math.radians(spin_deg))
    return matmul(R_ab, [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def frame_spun(R, spin_deg):
    """A module frame turned `spin_deg` about its OWN axis (its Z, the third column). The
    same rotation `module_frame(..., spin_deg=)` applies, for callers that already hold a
    seed frame instead of building one from the URDF."""
    return spin_frame(R, spin_deg)


def spin_feasible(R_ab, t_ab, pad_r, width, spin_deg, placement=None, patterns=PATTERNS):
    """Is this rotation of module B allowed? Returns (spec, None) or (None, reason).

    Allowed means three things, all of them the route's own: it routes at all; neither land
    radius had to be cut back from the measured one to keep an insert its ring
    (`land_radius_fit` reports which hole did it); and `check_hole_flat_rings` passes, which
    `route_path` has already run."""
    try:
        spec = route_relative(spin_frame(R_ab, spin_deg), t_ab, pad_r, placement, patterns,
                              width)
    except ValueError as e:
        return None, str(e)
    geo = spec['geometry']
    for key, what in (('land_tight', 'land fillet'), ('corner_tight', 'corner round')):
        if geo.get(key) is not None:
            return None, ('the %s had to be cut back from the measured radius to leave the '
                          'insert at %.0f deg its %.1f mm of flat ring'
                          % (what, geo[key], HOLE_FLAT_RING))
    return spec, None


def stator_ring_margin(spec):
    """The smallest flat seating ring on the STATOR pad of this link, mm beyond the hole
    edge. This is what the stator module's rotation actually moves - the rotor pad's four
    holes are 90 deg symmetric and its key pegs and stop block turn WITH them - so it is the
    number the solve maximises inside a feasible window."""
    return min(h['ring'] for h in spec['geometry']['hole_rings']
               if h['pad'] == 'stator' and h['seating'])


def _spin_windows(ok, step):
    """Contiguous circular runs of True in `ok` (sampled every `step` deg), as (lo, hi) in
    sample indices; a fully feasible circle comes back as one full turn."""
    n = len(ok)
    if all(ok):
        return [(0.0, 360.0 - step)]
    out = []
    for i in range(n):
        if ok[i] and not ok[(i - 1) % n]:
            j = i
            while ok[j % n]:
                j += 1
            out.append((i * step, (j - 1) * step))
    return out


def _spin_edge(inside, outside, test, tol=SPIN_REFINE):
    """Bisect the feasibility boundary between a feasible and an infeasible rotation."""
    lo, hi = inside, outside
    while abs(hi - lo) > tol:
        mid = (lo + hi) / 2.0
        if test(mid) is not None:
            lo = mid
        else:
            hi = mid
    return lo


def _ang_dist(a, b):
    d = (a - b) % 360.0
    return min(d, 360.0 - d)


def module_spin_fit(R_ab, t_ab, pad_r, width, placement=None, patterns=PATTERNS,
                    step=SPIN_STEP, seed=0.0):
    """Solve one module's rotation about its own axis, from the stator end of ONE link.

    Sweeps the whole turn, asks `spin_feasible` at every sample, refines each window's edges
    by bisection and returns:

      windows  - the feasible ranges in degrees, `lo` in [0, 360) and `hi` > `lo` (a window
                 that straddles 0 comes back as e.g. 248.26 .. 439.74), edges bisected
      seed_ok  - whether the measured pose (`seed`) is already one of them
      spin     - the rotation chosen, by the rule at the top of this section
      margin   - the smallest stator flat ring at `spin`
      best     - the maximin rotation over the WHOLE turn, and its margin, whether or not it
                 was taken: the solve says what it gave up.

    Raises ValueError, naming the pair and every sampled objection it saw, when no rotation
    works at all."""
    n = int(round(360.0 / step))
    ok, reasons = [], {}
    for i in range(n):
        spec, why = spin_feasible(R_ab, t_ab, pad_r, width, i * step, placement, patterns)
        ok.append(spec is not None)
        if why is not None:
            key = why.split(';')[0][:160]
            reasons[key] = reasons.get(key, 0) + 1
    if not any(ok):
        raise ValueError('no rotation of this module clears the stator inserts: over %d '
                         'samples of the full turn the route objected - %s'
                         % (n, ' || '.join(sorted(reasons))))

    def test(d):
        return spin_feasible(R_ab, t_ab, pad_r, width, d % 360.0, placement, patterns)[0]

    windows = []
    for lo_i, hi_i in _spin_windows(ok, step):
        lo = lo_i if hi_i - lo_i >= 360.0 - step else _spin_edge(lo_i, lo_i - step, test)
        hi = hi_i if hi_i - lo_i >= 360.0 - step else _spin_edge(hi_i, hi_i + step, test)
        windows.append((round(lo, SPIN_ROUND), round(hi, SPIN_ROUND)))

    def maximin(lo, hi, n_grid=40):
        """The safest rotation inside a feasible window: the one whose smallest stator flat
        ring is largest. Sampled on a grid across the window; where the objective is FLAT -
        the landing is nowhere near an insert over a whole plateau, which is the normal case
        for a link with room to spare - the MIDDLE of the plateau is taken, so the answer is
        the window's own centre rather than whichever sample came first."""
        grid = [lo + (hi - lo) * k / float(n_grid) for k in range(n_grid + 1)]
        vals = [(d, stator_ring_margin(s)) for d, s in ((d, test(d)) for d in grid)
                if s is not None]
        top = max(m for _, m in vals)
        flat = [d for d, m in vals if m >= top - 1e-9]
        return (flat[0] + flat[-1]) / 2.0, top

    seed_spec = test(seed)
    # one candidate per window: its safest rotation, its ring there, and how far off the seed
    # it is. The order below IS the rule - nearest to the measured pose first, then the
    # largest ring, then the widest window, then the lower angle so the answer is never
    # arbitrary.
    cand = []
    for lo, hi in windows:
        d, m = maximin(lo, hi)
        cand.append({'lo': lo, 'hi': hi, 'spin': d, 'margin': m,
                     'off_seed': min(_ang_dist(lo, seed), _ang_dist(hi, seed)),
                     'width': hi - lo})
    all_best = max(cand, key=lambda c: (c['margin'], -abs(c['spin'])))
    all_best = (all_best['spin'], all_best['margin'])
    if seed_spec is not None:
        spin, margin, why = seed % 360.0, stator_ring_margin(seed_spec), \
            'the measured pose already clears every insert'
    else:
        c = min(cand, key=lambda c: (c['off_seed'], -c['margin'], -c['width'], c['lo']))
        spin, margin = c['spin'], c['margin']
        why = ('the measured pose puts a stator insert under the landing; this is the safest '
               'point of the nearest feasible window %.3f..%.3f deg' % (c['lo'], c['hi']))
    return {'spin': round(spin, SPIN_ROUND), 'margin': margin, 'seed': seed,
            'seed_ok': seed_spec is not None, 'windows': windows, 'why': why,
            'best': {'spin': round(all_best[0], SPIN_ROUND), 'margin': all_best[1]},
            'step': step}


def solve_module_spins(frames, links, placement=None, patterns=PATTERNS, step=SPIN_STEP):
    """Solve EVERY module's rotation, as a chain.

    `frames` = {label: (R, t)} at the seed rotation (the pose measured in the exo STEP);
    `links` = [(name, a, b, pad_r, width)] from the base outward. Returns
    {label: fit-dict}, with `spin` 0 and `why` 'no link lands on this module' for a joint
    that is nobody's stator end (the base joint).

    A joint is the STATOR end of one link and the ROTOR end of the next, and ONE rotation has
    to satisfy both, so they are solved together and not per link. The coupling is one-way,
    and measurably so:

    - turning the STATOR module moves the three inserts and the actuator cutout under the
      landing. That is the constraint being solved here.
    - turning the ROTOR module turns the pad frame with it, so `landing_numbers` returns the
      same pair (u, w, s_land, zL, w_off are all unchanged) and the LANDING does not move at
      all. It is not a no-op on the body, though: the four M3 holes turn under the route, so
      `rotor_flat_needed` - how far out the pad must stay flat for the screw heads - changes
      by up to 1.83 mm over a turn, which moves a ramp's climb window. A rotor-end rotation
      therefore reshapes its link without changing where it lands.

    So one forward pass is exact for the constraint this solves, and `spin_chain_check`
    re-routes every link on the SOLVED frames afterwards rather than assuming it."""
    out = {}
    for label in frames:
        out[label] = {'spin': 0.0, 'margin': None, 'seed': 0.0, 'seed_ok': True,
                      'windows': [], 'why': 'no link lands on this module',
                      'best': None, 'step': step}
    for name, a, b, pad_r, width in links:
        R_ab, t_ab = relative_frame(frames[a][0], frames[a][1], frames[b][0], frames[b][1])
        fit = module_spin_fit(R_ab, t_ab, pad_r, width, placement, patterns, step)
        fit['from_link'] = name
        out[b] = fit
    return out


def spin_windows_resolved(spins, factor=2.0):
    """Is the coarse sweep fine enough to trust? Returns the narrowest feasible window the
    solve found, over every joint, and whether it is at least `factor` steps wide. A window
    narrower than the step could have been stepped over, which would make a 'no rotation
    works' answer wrong - so this is the number that qualifies the search, and it is reported
    rather than assumed."""
    widths = [(hi - lo, label) for label, f in spins.items() for lo, hi in f['windows']]
    if not widths:
        return None
    w, label = min(widths)
    step = max(f['step'] for f in spins.values())
    return {'narrowest': w, 'at': label, 'step': step, 'ok': w >= factor * step}


def spun_frames(frames, links, placement=None, patterns=PATTERNS, step=SPIN_STEP):
    """Solve the chain and apply it: returns (frames with every module at its solved
    rotation, the spin table). One call, so the snapshot, the hull sampler, the reports and
    BuildArm cannot drift apart on what the arm actually is."""
    spins = solve_module_spins(frames, links, placement, patterns, step)
    out = dict((label, (frame_spun(R, spins[label]['spin']), t))
               for label, (R, t) in frames.items())
    return out, spins


def spin_chain_check(frames, links, spins, placement=None, patterns=PATTERNS):
    """Prove the chain closed: with every solved rotation applied, re-route every link and
    check that the stator end still clears (the land radii are the measured ones and every
    hole keeps its ring) AND that the rotor end is unmoved - i.e. that the link's geometry is
    the same one the solve looked at. Returns one dict per link; raises naming the link that
    does not close."""
    out = []
    for name, a, b, pad_r, width in links:
        R_ab, t_ab = relative_frame(frames[a][0], frames[a][1], frames[b][0], frames[b][1])
        spec, why = spin_feasible(R_ab, t_ab, pad_r, width, 0.0, placement, patterns)
        if spec is None:
            raise ValueError('%s does not close with the solved rotations (%s %+.3f deg, '
                             '%s %+.3f deg): %s'
                             % (name, a, spins[a]['spin'], b, spins[b]['spin'], why))
        out.append({'link': name, 'a': a, 'b': b, 'pattern': spec['pattern'],
                    'stator_margin': stator_ring_margin(spec),
                    'fillet_r': spec['route']['stator_land']['fillet_r'],
                    'corner_r': spec['route']['stator_land']['corner_r'],
                    'rings': spec['geometry']['hole_rings']})
    return out


def route_report(spec):
    """One line describing what was built, for BuildArm's report: the same sentence for any
    route, so the build has no per-pattern branch left."""
    g, r = spec['geometry'], spec['route']
    j, land = r['rotor_join'], r['stator_land']
    arcs = [sg for sg in g['profile'] if sg[0] == 'arc']
    bits = ['%s route, ribbon %.0f mm wide' % (r['pattern'], r['width'])]
    bits.append('%s rotor-pad join at root s=%.2f (pad r %.1f)' % (j['kind'], j['root_s'],
                                                                  j['pad_r']))
    if j['kind'] == 'ledge':
        s_in, s_out = g['s_in'], g['s_out']
        bits.append('outer face flat out to s=%.2f (rim r %.1f + %.2f), round r %.1f down to '
                    'z=%.2f, ledge %.2f mm of which %.2f mm flat (relief r %.2f), clearance '
                    'to r %.1f below the stop block (z=%.1f) %.3f mm'
                    % (s_out - j['round_r'], j['pad_r'], s_out - j['round_r'] - j['pad_r'],
                       j['round_r'], j['pad_t'] - j['round_r'], s_in - j['pad_r'],
                       s_in - j['relief_r'] - j['pad_r'], j['relief_r'],
                       max(MODULE_R, STATOR_PAD_R), MODULE_STOP_Z_TOP, g['clearance_stop']))
    else:
        bits.append('plate z %.1f..%.1f COPLANAR with the rotor pad (both faces flat, rim '
                    'covered, nothing below z=0), %s'
                    % (g['plate_z'][0], g['plate_z'][1],
                       g.get('clearance_note',
                             'module B clears it by %.2f mm' % g['clearance_b'])))
    for line in route_describe(r)[2:]:
        bits.append(line.strip())
    bits.append('slab tongue %s the disk centre (x_off %.3f), edges %s'
                % ('on' if abs(g.get('x_off', 0.0)) < 1e-9 else 'off',
                   g.get('x_off', 0.0),
                   ' / '.join('%s r %.1f' % (e['kind'], e['r']) for e in g['edges'])))
    sc, fit = g['s_curve'], g.get('s_fit')
    if sc is not None:
        clr, at = riser_radial_clearance(g)
        bits.append('lateral %.3f mm taken by the S (%s placement, window s=%.2f..%.2f, '
                    '%.2f mm straight each side): two tangent arcs r %.1f through %.2f deg, '
                    'walls r %.1f / %.1f, inflection s=%.2f w=%.3f; radial clearance along '
                    'it %.3f mm (at s=%.2f z=%.2f)'
                    % (abs(sc['w_off']), sc['placement'], sc['window'][0], sc['window'][1],
                       sc['margin'][0], sc['r'], sc['theta_deg'], sc['wall_r'][0],
                       sc['wall_r'][1], sc['inflection'][0], sc['inflection'][1], clr,
                       at[0], at[2]) if 'window' in sc else
                    'lateral %.3f mm taken by the S (%s placement) s=%.2f..%.2f'
                    % (abs(sc['w_off']), sc['placement'], sc['s_start'], sc['s_end']))
    elif g['offset_taken'] == 'taper':
        tp = g['taper']
        bits.append('lateral %.3f mm taken by the PLAN TAPER: band %.3f mm across on '
                    'w %.3f..%.3f (centre %.3f = w_off/2), %s wall on the rotor rim and %s '
                    'wall on the stator disk, led off the rim by r %.1f (%s)'
                    % (abs(g['w_off']), tp['width'], tp['w_lo'], tp['w_hi'], tp['centre'],
                       tp['walls'][-1]['kind'], tp['walls'][+1]['kind'], tp['taper_r'],
                       fit['why'] if fit else 'no S was possible'))
    elif fit is not None:
        bits.append('lateral %.3f mm taken in the SLAB (%s)' % (abs(g['w_off']), fit['why']))
    else:
        bits.append('no lateral offset')
    bits.append('side profile %d segments, radii %s; slab frame snapped %.1e rad to the route'
                % (len(g['profile']), '/'.join('%.1f' % sg[5] for sg in arcs), g['snap_rad']))
    return '; '.join(bits)


# ---------------- clearance between two generated links (pure-Python proxy for the
# Fusion interference check; the real check runs in BuildArm after the build) ----------------

def _obox_dist(p, center, u, w, half):
    """Distance from point p to an oriented box (axes u, w, u x w; half extents)."""
    n = cross(u, w)
    d = [p[i] - center[i] for i in range(3)]
    q = [abs(dot(d, u)) - half[0], abs(dot(d, w)) - half[1], abs(dot(d, n)) - half[2]]
    out = math.sqrt(sum(max(v, 0.0) ** 2 for v in q))
    return out if out > 0 else max(q)       # negative = inside (penetration depth)


def _box_surface_points(lo, hi, step):
    """Points on the six faces of an axis-aligned box, `step` mm apart."""
    def rng(a, b):
        n = max(1, int(math.ceil((b - a) / step)))
        return [a + (b - a) * k / n for k in range(n + 1)]
    xs, ys, zs = rng(lo[0], hi[0]), rng(lo[1], hi[1]), rng(lo[2], hi[2])
    pts = []
    for x in xs:
        for y in ys:
            pts.append([x, y, lo[2]]); pts.append([x, y, hi[2]])
    for x in xs:
        for z in zs:
            pts.append([x, lo[1], z]); pts.append([x, hi[1], z])
    for y in ys:
        for z in zs:
            pts.append([lo[0], y, z]); pts.append([hi[0], y, z])
    return pts


def _hull_boxes(geo, parts=None):
    """A geometry dict's hull boxes as oriented boxes (centre, u, w, half extents) in its own
    link frame. `parts` defaults to everything the body is made of - `hull_parts` AND the
    slab's strip boxes - which is what a TARGET must be measured against (EXO-102); pass
    `geo['hull_parts']` alone to sample the body's own surface."""
    u, w = geo['u'], geo['w']
    if parts is None:
        parts = list(geo['hull_parts']) + list(geo.get('slab_hulls') or [])
    out = []
    for (s0, s1), (z0, z1), (wc, wh) in parts:
        sc, zc = (s0 + s1) / 2.0, (z0 + z1) / 2.0
        out.append(([u[0] * sc + w[0] * wc, u[1] * sc + w[1] * wc, zc], u, w,
                    [(s1 - s0) / 2.0, wh, (z1 - z0) / 2.0]))
    return out


def _hull_points(geo, step):
    """Points `step` mm apart on the surface of a geometry dict's hull boxes, in its own
    link frame."""
    n = cross(geo['u'], geo['w'])
    pts = []
    for c, u, w, half in _hull_boxes(geo, geo['hull_parts']):
        for q in _box_surface_points([-half[0], -half[1], -half[2]], half, step):
            pts.append([c[i] + q[0] * u[i] + q[1] * w[i] + q[2] * n[i] for i in range(3)])
    return pts


def link_clearance(R_a1a2, t_a1a2, boxes12, R_23, t_23, bar23, step=0.5):
    """Min distance (mm) from link 12's body (frame A1) to link 23's HULL boxes (frame A2).
    R_a1a2/t_a1a2 = frame A2 in A1; `boxes12` and `bar23` are geometry dicts from route().
    The hulls contain the filleted profile (fillets only remove material), so the value
    is a lower bound of the real clearance; at the riser inner face (a plane at s_in over
    the slab's z range) hull and body coincide. Negative = penetration. Sampled at `step`
    mm, so accurate to about step/2."""
    pts = _hull_points(boxes12, step)
    # the stator slab of link 12, in frame A1. Its disk and its tongue blends reach outside
    # the hull boxes, so the flange OUTLINE itself is swept over the slab's own thickness:
    # boundary points at `step`, mapped through the frame the slab was drawn in (R_b, the
    # snapped frame - 16 deg for the shoulder L) and placed at A2's origin, which for link 12
    # IS joint B. `R_b` defaults to A2's own frame for geometries that have none.
    R_slab = boxes12.get('R_b') or R_a1a2
    z_lo, z_hi = boxes12.get('slab_faces', (STATOR_UNDERSIDE_Z - PAD_T, STATOR_UNDERSIDE_Z))
    ring = []
    for seg in boxes12['flange']:
        ring += _seg_points(seg, step)
    for (x2, y2) in ring:
        for z2 in (z_lo, (z_lo + z_hi) / 2.0, z_hi):
            p2 = [x2, y2, z2]
            pts.append([t_a1a2[i] + sum(R_slab[i][j] * p2[j] for j in range(3)) for i in range(3)])
    RT = transpose(R_a1a2)
    parts = _hull_boxes(bar23)
    best = float('inf')
    for p in pts:
        d = [p[i] - t_a1a2[i] for i in range(3)]
        p2 = matvec(RT, d)
        for c, uu, ww, half in parts:
            best = min(best, _obox_dist(p2, c, uu, ww, half))
    return best


# ---------------- interference classification (used by BuildArm's post-build check) ----------------
INTERFERENCE_DEPTH_TOL = 0.02   # mm: thinner overlaps are modelling slivers (the module's
                                # stator underside / heat-set inserts sit 0.008-0.013 mm
                                # below the STATOR_UNDERSIDE_Z plane the pads are built to)


def classify_interference(name_a, name_b, bbox, module_internal=False):
    """'module-internal' when BuildArm found the same hit inside one J* module on its own
    (press-fit inserts and screws in `sensor module 2`, not generator-made); 'sliver' when
    the overlap is at most INTERFERENCE_DEPTH_TOL deep (smallest bbox extent - a
    conservative measure for tilted overlaps); otherwise 'FAIL'. No bbox -> 'FAIL'."""
    if module_internal:
        return 'module-internal'
    if bbox is not None:
        depth = min(bbox[1][i] - bbox[0][i] for i in range(3))
        if depth <= INTERFERENCE_DEPTH_TOL:
            return 'sliver'
    return 'FAIL'
