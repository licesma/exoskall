import importlib, sys, math
sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, sweepgen, BuildArm, BuildArmGen
for m in (armgen, sweepgen, BuildArm, BuildArmGen):
    importlib.reload(m)

joints, by_child = armgen.parse_urdf(BuildArm.URDF_PATH)
shift, _ = armgen.joint_world_pose(BuildArm.ORIGIN_AT_JOINT, joints, by_child)
seed = {}
for j in BuildArm.JOINTS:
    R, t = armgen.module_frame(j['name'], joints, by_child, j['axis_sign'], j['y_hint'])
    seed[j['label']] = (R, [t[i] - shift[i] for i in range(3)])
LINKS = [(L['name'], L['a'], L['b'], L['pad_r'], L['width']) for L in BuildArm.LINKS]
frames, spins = sweepgen.solve_spins(seed, LINKS)

def find_body(name):
    for c in design.allComponents:
        if c.name == name:
            for i in range(c.bRepBodies.count):
                return c.bRepBodies.item(i)
    return None

TYPES = {0:'plane',1:'cyl',2:'cone',3:'sphere',4:'torus',5:'ellcyl',6:'ellcone',7:'nurbs'}
for name, a, b, pad_r, width in LINKS:
    if name not in ('link_12','link_45'):
        continue
    RA, tA = frames[a]; RB, tB = frames[b]
    R_ab, t_ab = armgen.relative_frame(RA, tA, RB, tB)
    route = sweepgen.sweep_route(R_ab, t_ab, pad_r, width)
    u, w = route['u'], route['w']
    F = BuildArm.Frame(RA, [v*1000.0 for v in tA])
    Rt = armgen.transpose(RA)
    def to_swz(p_cm):
        p = [p_cm.x*10.0, p_cm.y*10.0, p_cm.z*10.0]
        d = [p[i] - F.t[i] for i in range(3)]
        loc = armgen.matvec(Rt, d)
        return (armgen.dot(loc, u), armgen.dot(loc, w), loc[2])
    body = find_body('g_'+name)
    log('=== %s: %d faces %d edges, s_near %.2f s_far %.2f y_end %.2f pad_r %.1f half_w %.1f'
        % (name, body.faces.count, body.edges.count, route['s_near'], route['s_far'],
           route['y_end'], pad_r, width/2.0))
    rows = []
    for i in range(body.edges.count):
        e = body.edges.item(i)
        g0 = e.geometry
        kind = g0.objectType.split('::')[-1]
        L = e.length * 10.0
        p = e.pointOnEdge
        f1, f2 = e.faces.item(0), e.faces.item(1)
        t1 = TYPES.get(f1.geometry.surfaceType, str(f1.geometry.surfaceType))
        t2 = TYPES.get(f2.geometry.surfaceType, str(f2.geometry.surfaceType))
        try:
            _, n1 = f1.evaluator.getNormalAtPoint(p)
            _, n2 = f2.evaluator.getNormalAtPoint(p)
            nx = n1.x+n2.x; ny = n1.y+n2.y; nz = n1.z+n2.z
            nn = math.sqrt(nx*nx+ny*ny+nz*nz)
            if nn < 1e-6:
                conv = 'smooth'
            else:
                eps = 0.005
                q = adsk.core.Point3D.create(p.x - eps*nx/nn, p.y - eps*ny/nn, p.z - eps*nz/nn)
                pc = body.pointContainment(q)
                conv = 'convex' if pc == adsk.fusion.PointContainment.PointInsidePointContainment else 'concave'
        except Exception as ex:
            conv = 'err'
        s, ww, z = to_swz(p)
        rows.append('  e%-3d %-14s L%8.2f %-8s %-6s/%-6s at (s %8.2f w %8.2f z %8.2f)'
                    % (i, kind, L, conv, t1, t2, s, ww, z))
    for r in rows:
        log(r)
