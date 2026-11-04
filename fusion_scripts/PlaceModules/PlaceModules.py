"""Place sensor modules at URDF joint poses (test: 2 joints).

Prompts for a URDF file, computes each joint's world position + axis at the
zero configuration, and inserts one copy of the module component per joint,
rotated so the module's local Z (the horn axis, w) aligns with the joint axis.
The rotation about the axis (u) is arbitrary — tweak it manually afterwards;
any rotation about the module's local Z keeps the axis constraint valid.

Each module is wrapped in a new component named after the joint, whose origin
is exactly the joint frame.
"""

import adsk.core
import adsk.fusion
import traceback
import math
import xml.etree.ElementTree as ET

# Name of the saved Fusion document (or existing in-design component) to place.
MODULE_DOC_NAME = 'sensor module 2'

# Joints to place. R1/R2 share the same origin (shoulder cluster), so the
# default test pair is R2/R3 which are visibly apart.
TEST_JOINTS = ['Arm_R2_Joint', 'Arm_R3_Joint']

M_TO_CM = 100.0  # URDF uses meters, the Fusion API uses centimeters.


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


def _matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _matvec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


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
        joints[name] = {
            'xyz': xyz, 'rpy': rpy, 'axis': axis,
            'parent': j.find('parent').get('link'),
            'child': j.find('child').get('link'),
        }
        by_child[j.find('child').get('link')] = name
    return joints, by_child


def joint_world_pose(joint_name, joints, by_child):
    """World position (m) and world axis of a joint at the zero configuration."""
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
        R = _matmul(R, _rpy_to_R(*j['rpy']))
    axis_world = _matvec(R, chain[-1]['axis'])
    n = math.sqrt(sum(v * v for v in axis_world))
    axis_world = [v / n for v in axis_world]
    return t, axis_world


# ---------------- Fusion placement ----------------

def frame_matrix(t_m, w):
    """Matrix3D: local Z rotated onto world axis w, origin at t (meters -> cm)."""
    m = adsk.core.Matrix3D.create()
    z = adsk.core.Vector3D.create(0, 0, 1)
    target = adsk.core.Vector3D.create(w[0], w[1], w[2])
    target.normalize()
    if z.dotProduct(target) < -0.99999:
        # Antiparallel: 180 deg about X (any in-plane axis works).
        m.setToRotation(math.pi, adsk.core.Vector3D.create(1, 0, 0),
                        adsk.core.Point3D.create(0, 0, 0))
    else:
        m.setToRotateTo(z, target)
    m.translation = adsk.core.Vector3D.create(
        t_m[0] * M_TO_CM, t_m[1] * M_TO_CM, t_m[2] * M_TO_CM)
    return m


def find_module_component(design):
    # Linked components get the version appended ("sensor module 2 v12"),
    # so match on prefix.
    for comp in design.allComponents:
        if comp.name == MODULE_DOC_NAME or comp.name.startswith(MODULE_DOC_NAME + ' v'):
            return comp
    return None


def find_data_file(app, name):
    """Recursively search the active project for a document named `name`."""
    project = app.data.activeProject
    if project is None:
        return None
    stack = [project.rootFolder]
    while stack:
        folder = stack.pop()
        for i in range(folder.dataFiles.count):
            f = folder.dataFiles.item(i)
            if f.name == name:
                return f
        for i in range(folder.dataFolders.count):
            stack.append(folder.dataFolders.item(i))
    return None


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('Open a Fusion design first.')
            return

        dlg = ui.createFileDialog()
        dlg.title = 'Select URDF file'
        dlg.filter = 'URDF files (*.urdf);;All files (*.*)'
        dlg.isMultiSelectEnabled = False
        if dlg.showOpen() != adsk.core.DialogResults.DialogOK:
            return
        joints, by_child = parse_urdf(dlg.filename)

        missing = [n for n in TEST_JOINTS if n not in joints]
        if missing:
            ui.messageBox('Joints not found in URDF: ' + ', '.join(missing))
            return

        # Module source: reuse a component already in the design, otherwise
        # insert (as link) from the active project's cloud data.
        module_comp = find_module_component(design)
        data_file = None
        if module_comp is None:
            data_file = find_data_file(app, MODULE_DOC_NAME)
            if data_file is None:
                ui.messageBox(
                    'Could not find "{}" in this design or the active project. '
                    'Save the module document in the same project.'
                    .format(MODULE_DOC_NAME))
                return

        root = design.rootComponent
        report = []
        for jn in TEST_JOINTS:
            t, w = joint_world_pose(jn, joints, by_child)
            m = frame_matrix(t, w)

            wrapper = root.occurrences.addNewComponent(m)
            wrapper.component.name = jn

            identity = adsk.core.Matrix3D.create()
            if module_comp is not None:
                inner = wrapper.component.occurrences.addExistingComponent(
                    module_comp, identity)
            else:
                inner = wrapper.component.occurrences.addByInsert(
                    data_file, identity, True)
                module_comp = inner.component  # reuse for subsequent joints
            inner.isLightBulbOn = True

            report.append('{}: pos(mm)=({:.1f}, {:.1f}, {:.1f}) axis=({:.3f}, {:.3f}, {:.3f})'
                          .format(jn, t[0] * 1000, t[1] * 1000, t[2] * 1000,
                                  w[0], w[1], w[2]))

        ui.messageBox('Placed {} modules:\n\n{}'.format(
            len(report), '\n'.join(report)))

    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
