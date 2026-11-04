#!/usr/bin/env python3
"""Build parts/rotors_compare.bambu.3mf, a Bambu Studio project, without the Bambu CLI (EXO-267).

    python3 -I tools/bambu_project_exo267.py

Why not the CLI: BambuStudio 02.04.00.70 CLI segfaults on macOS in every mode tried
(--export-3mf, --info), in PartPlateList::set_shapes -> GUI_App::plater() (null in CLI).

Structure copied from parts/inner_bottom_v2.3mf (BambuStudio 02.04.00.70 project):
3D/3dmodel.model with one component object per part -> 3D/Objects/object_N.model,
Metadata/project_settings.config copied verbatim (printer/filament/process), Metadata/
model_settings.config naming each object, one plate. No thumbnails, no slice data.

Orientation (mesh pre-transformed, build transform is a pure translation):
  *_top:    rotated 180 deg about X ((x,y,z)->(x,-y,-z)): rotor face (file z 7.5, O36,
            the largest flat face) on the bed, spigot/pegs up.
  *_bottom: as exported: O35 plate (file z -5.7) on the bed, r15 boss up.
"""
import os, re, sys, zipfile, uuid
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PARTS = os.path.join(ROOT, 'parts')
# EXO-333: the settings source moved to the archive with the other October test files
SRC_PROJECT = os.path.join(PARTS, 'archive', '2026-10-tests', 'inner_bottom_v2.3mf')
OUT = os.path.join(PARTS, 'rotors_compare.bambu.3mf')
NS = {'m': 'http://schemas.microsoft.com/3dmanufacturing/core/2015/02'}
# name, flip about X, plate centre (x, y) on the 256x256 P1S bed; 46 mm pitch, ~10 mm gaps
PARTS_LAYOUT = [
    ('manual_inner_top', True, (105.0, 151.0)),
    ('manual_inner_bottom', False, (151.0, 151.0)),
    ('inner_top', True, (105.0, 105.0)),
    ('inner_bottom', False, (151.0, 105.0)),
]
HDR = ('<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" '
       'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
       'xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" '
       'xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" requiredextensions="p">\n')


def read_mesh(path):
    r = ET.fromstring(zipfile.ZipFile(path).read('3D/3dmodel.model'))
    objs = [o for o in r.findall('m:resources/m:object', NS) if o.find('m:mesh', NS) is not None]
    assert len(objs) == 1, path
    assert r.find('m:build/m:item', NS).get('transform') is None, path
    m = objs[0].find('m:mesh', NS)
    V = [(float(v.get('x')), float(v.get('y')), float(v.get('z'))) for v in m.findall('m:vertices/m:vertex', NS)]
    T = [(t.get('v1'), t.get('v2'), t.get('v3')) for t in m.findall('m:triangles/m:triangle', NS)]
    return V, T


def f(x):
    return ('%.6f' % x).rstrip('0').rstrip('.').replace('-0', '-0') if abs(x) > 5e-7 else '0'


def main():
    src = zipfile.ZipFile(SRC_PROJECT)
    settings = src.read('Metadata/project_settings.config')
    objects, build, rels, ms_objs, ms_inst, assemble = [], [], [], [], [], []
    files = {}
    for i, (name, flip, (px, py)) in enumerate(PARTS_LAYOUT):
        V, T = read_mesh(os.path.join(PARTS, name + '.3mf'))
        if flip:
            V = [(x, -y, -z) for x, y, z in V]
        lo = [min(v[k] for v in V) for k in range(3)]
        hi = [max(v[k] for v in V) for k in range(3)]
        c = [(lo[k] + hi[k]) / 2 for k in range(3)]
        V = [(x - c[0], y - c[1], z - c[2]) for x, y, z in V]  # mesh centred, like v2
        tz = (hi[2] - lo[2]) / 2  # lifts min z to 0
        # EXO-290: ids must be unique across ALL model files, Bambu pattern (Component1.3mf,
        # writst_mount_e_v10.3mf): mesh in object_k.model has id 2k-1, wrapper id 2k.
        # Every sub-model used id 1 before; Bambu Studio resolved all four to the first mesh.
        mid, oid, sub = 2 * i + 1, 2 * i + 2, 'object_%d.model' % (i + 1)
        body = [HDR, ' <metadata name="BambuStudio:3mfVersion">1</metadata>\n <resources>\n',
                '  <object id="%d" name="%s" p:UUID="%s" type="model">\n   <mesh>\n    <vertices>\n' % (mid, name, uuid.uuid4())]
        body += ['     <vertex x="%s" y="%s" z="%s"/>\n' % (f(x), f(y), f(z)) for x, y, z in V]
        body.append('    </vertices>\n    <triangles>\n')
        body += ['     <triangle v1="%s" v2="%s" v3="%s"/>\n' % t for t in T]
        body.append('    </triangles>\n   </mesh>\n  </object>\n </resources>\n <build/>\n</model>\n')
        files['3D/Objects/' + sub] = ''.join(body)
        objects.append('  <object id="%d" name="%s" p:UUID="%s" type="model">\n   <components>\n'
                       '    <component p:path="/3D/Objects/%s" objectid="%d" p:UUID="%s" transform="1 0 0 0 1 0 0 0 1 0 0 0"/>\n'
                       '   </components>\n  </object>\n' % (oid, name, uuid.uuid4(), sub, mid, uuid.uuid4()))
        tr = '1 0 0 0 1 0 0 0 1 %s %s %s' % (f(px), f(py), f(tz))
        build.append('  <item objectid="%d" p:UUID="%s" transform="%s" printable="1"/>\n' % (oid, uuid.uuid4(), tr))
        rels.append(' <Relationship Target="/3D/Objects/%s" Id="rel-%d" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>\n' % (sub, i + 1))
        ms_objs.append('  <object id="%d">\n    <metadata key="name" value="%s"/>\n    <metadata key="extruder" value="1"/>\n'
                       '    <metadata face_count="%d"/>\n    <part id="1" subtype="normal_part">\n'
                       '      <metadata key="name" value="%s"/>\n      <metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/>\n'
                       '      <metadata key="source_file" value="%s.3mf"/>\n      <metadata key="source_object_id" value="0"/>\n'
                       '      <metadata key="source_volume_id" value="0"/>\n'
                       '      <metadata key="source_offset_x" value="%s"/>\n      <metadata key="source_offset_y" value="%s"/>\n'
                       '      <metadata key="source_offset_z" value="%s"/>\n'
                       '      <mesh_stat face_count="%d" edges_fixed="0" degenerate_facets="0" facets_removed="0" facets_reversed="0" backwards_edges="0"/>\n'
                       '    </part>\n  </object>\n' % (oid, name, len(T), name, name, f(c[0]), f(c[1]), f(c[2]), len(T)))
        ms_inst.append('    <model_instance>\n      <metadata key="object_id" value="%d"/>\n'
                       '      <metadata key="instance_id" value="0"/>\n      <metadata key="identify_id" value="%d"/>\n'
                       '    </model_instance>\n' % (oid, 100 + i))
        assemble.append('   <assemble_item object_id="%d" instance_id="0" transform="%s" offset="0 0 0" />\n' % (oid, tr))
    meta = ''.join(' <metadata name="%s">%s</metadata>\n' % kv for kv in [
        ('Application', 'BambuStudio-02.04.00.70'), ('BambuStudio:3mfVersion', '1'),
        ('CreationDate', '2026-10-09'), ('ModificationDate', '2026-10-15'), ('Title', 'rotors_compare')])
    files['3D/3dmodel.model'] = (HDR + meta + ' <resources>\n' + ''.join(objects) + ' </resources>\n <build p:UUID="%s">\n' % uuid.uuid4()
                                 + ''.join(build) + ' </build>\n</model>\n')
    files['3D/_rels/3dmodel.model.rels'] = ('<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">\n'
                                            + ''.join(rels) + '</Relationships>')
    files['_rels/.rels'] = ('<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">\n'
                            ' <Relationship Target="/3D/3dmodel.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>\n</Relationships>')
    files['[Content_Types].xml'] = src.read('[Content_Types].xml')
    files['Metadata/project_settings.config'] = settings
    files['Metadata/slice_info.config'] = src.read('Metadata/slice_info.config')
    files['Metadata/filament_sequence.json'] = src.read('Metadata/filament_sequence.json')
    files['Metadata/model_settings.config'] = ('<?xml version="1.0" encoding="UTF-8"?>\n<config>\n' + ''.join(ms_objs)
        + '  <plate>\n    <metadata key="plater_id" value="1"/>\n    <metadata key="plater_name" value=""/>\n'
          '    <metadata key="locked" value="false"/>\n    <metadata key="filament_map_mode" value="Auto For Flush"/>\n'
          '    <metadata key="filament_maps" value="1"/>\n    <metadata key="filament_volume_maps" value="0"/>\n'
        + ''.join(ms_inst) + '  </plate>\n  <assemble>\n' + ''.join(assemble) + '  </assemble>\n</config>\n')
    order = ['[Content_Types].xml', '_rels/.rels', '3D/3dmodel.model', '3D/_rels/3dmodel.model.rels']
    with zipfile.ZipFile(OUT, 'w', zipfile.ZIP_DEFLATED) as z:
        for n in order + sorted(k for k in files if k not in order):
            d = files[n]
            z.writestr(n, d if isinstance(d, bytes) else d.encode())
    print('wrote', OUT)


if __name__ == '__main__':
    main()
