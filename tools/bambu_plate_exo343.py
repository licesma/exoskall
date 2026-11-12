#!/usr/bin/env python3
"""Build and read back Bambu plates of the exported parts on one P1S plate (EXO-343, EXO-345).

    python3 -I tools/bambu_plate_exo343.py                      # link.bambu.3mf: 2x stator + link_12
    python3 -I tools/bambu_plate_exo343.py --plate joints4      # joints4.bambu.3mf (EXO-345)
    python3 -I tools/bambu_plate_exo343.py --plate all          # all.bambu.3mf (EXO-348): joints4 + link_12
    python3 -I tools/bambu_plate_exo343.py --plate joints2      # joints2.bambu.3mf (EXO-394): link_12, link_23, 2x stator, 2x bottom
    python3 -I tools/bambu_plate_exo343.py --plate links2       # links2.bambu.3mf (EXO-394): link_12, link_23
    python3 -I tools/bambu_plate_exo343.py --plate stator_dia   # stator_dia.bambu.3mf (EXO-355): O42.2..42.6 stators
    python3 -I tools/bambu_plate_exo343.py --plate stator_dia_low  # stator_dia_low.bambu.3mf (EXO-358): O41.8..42.4
    python3 -I tools/bambu_plate_exo343.py --plate rotor        # rotor.bambu.3mf (EXO-338 layout, rebuilt EXO-364)
    python3 -I tools/bambu_plate_exo343.py --plate rotor_head   # rotor_head.bambu.3mf (EXO-352): bottom_h0.5 + bottom_h1.0
    python3 -I tools/bambu_plate_exo343.py --plate body_mount   # body_mount.bambu.3mf (EXO-401): the chest mount
    python3 -I tools/bambu_plate_exo343.py --plate links34      # links34.bambu.3mf (EXO-402, EXO-405/406): link_34, link_45
    python3 -I tools/bambu_plate_exo343.py --plate joints34     # joints34.bambu.3mf (EXO-402, EXO-405/406): + 2x stator, 2x bottom
    python3 -I tools/bambu_plate_exo343.py [--plate P] --check [file]   # read back only

Plates are specs in PLATES (output, title, layout); add one there instead of copying the script.

Derived from tools/bambu_project_exo267.py (no Bambu CLI: it segfaults, EXO-268).
Settings, slice_info, filament_sequence and [Content_Types] are copied verbatim from
parts/rotor.bambu.3mf (EXO-338: P1S 0.4, PLA Basic, 0.20mm Standard, supports tree(auto),
threshold 30, not build-plate-only). Ids per EXO-294: mesh in object_k.model id 2k-1,
wrapper id 2k, names on both and in model_settings.config.
EXO-405/406: project_settings.config = that reference with the declared per-plate overrides
(settings_overrides(): PLA_CF_MATCHA for every plate not in LEGACY_PLA_BASIC, plus
PLATE_SETTINGS), re-serialised exactly as Bambu writes it; per-object settings (OBJECT_SETTINGS)
go into model_settings.config. --check rebuilds the expected bytes and lists every changed key.

Orientation is BAKED INTO THE MESH; build items are pure translations. Per layout entry:
  None            as exported;
  'flipX'         180 deg about X, (x,y,z)->(x,-y,-z) (top.3mf rotor face down, = rotor.bambu.3mf);
  ('bed', axis)   the flat face at the extreme of the mesh along `axis` (source coords) goes on
                  the bed: R maps its area-weighted normal exactly onto -Z.
  stator x2: as exported, underside (file z -17.0 = module z -24.5, inserts, "0") on the bed.
  link_12 (EXO-394): ROTOR SIDE DOWN - the fused top's joint face -8.0 (410.2 mm2) on the bed
             (axis -Z_J1); the sockets and counterbores print clean, the 45 deg chamfer and the
             body take supports. Before EXO-394 it was rotated so its largest flat face, the outer
             (back) face of the J2 stator pad (normal -X, x = -32.5, 2339 mm2), is on the
             bed: R maps that face's area-weighted normal exactly onto -Z (about -Y, ~90 deg;
             (x,y,z) -> (-z, y, x) to 1e-4). The pad's mating face with the mark faces up;
             tab and stop block (J1 side) end >= 30 mm above the bed.
  joints4 (EXO-345): stator x4 as exported; top x4 'flipX'; bottom x4 as exported (O35 plate
             down), all exactly as rotor.bambu.3mf / link.bambu.3mf. link_23 (arm coords):
             its only flat support planes are the J3 stator-pad face (1649 mm2, carries the
             assembly mark, rejected) and the end face of the J3 flange (+X, x = 35.5,
             333 mm2); the latter goes down: mark on a vertical face 8-11 above the bed,
             J2 rotor face up, tab/stop block 36-39 above the bed, height 62.5. The J2 rotor
             pad's outer face (2328 mm2) hangs 27.5 above the bed on supports.
  all (EXO-348): the 14 objects of joints4 + link_12, print orientations unchanged (no XY
             rotation needed). Positions from a bottom-left greedy packer over bbox footprints
             (gap 11, margin 9, exclude zone 18x28 blocked), then shifted +0.88 / +2.75 to centre.
  joints2 (EXO-369): link_12, link_23, stator x2, top x2, bottom x2, orientations as `all`;
             rows link_23 + link_12 / 2 stators + 2 tops / 2 bottoms, gaps 12, centred, margins >= 10.
  stator_dia (EXO-350, EXO-355): stator_d42.2 .. 42.6 as exported (underside on the bed, bore and
             the "42.x" label on the top face up), rows of 3 + 2 at 64.5 pitch (gap 10.5).
"""
import math, os, sys, uuid, zipfile
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PARTS = os.path.join(ROOT, 'parts')
# EXO-375: rotor.bambu.3mf (top + bottom) is archived with the top; its settings stay the source
SETTINGS_SRC = os.path.join(PARTS, 'archive', '2026-10-tests', 'rotor_exo364.bambu.3mf')
NS = {'m': 'http://schemas.microsoft.com/3dmanufacturing/core/2015/02'}
PNS = '{http://schemas.microsoft.com/3dmanufacturing/production/2015/06}path'
# name, source, rotate?, plate centre (x, y). Stators 54x54 in a column at x 91 (y pitch
# 64.5 -> 10.5 gap), link (63.5 x 74.5 footprint) at x 160.25 (10.5 gap). Clear of the
# P1S exclude zone (0..18 x 0..28).
LINK_LAYOUT = [
    ('stator_1', 'stator', None, (91.0, 160.25)),
    ('stator_2', 'stator', None, (91.0, 95.75)),
    ('link_12', 'link_12', ('bed', (-0.000199314, 0.961245874, -0.275692454)), (160.25, 128.0)),
]
# joints4 (EXO-345): rows of 60.2 / 54 / 36 / 35 with 10.5 gaps, centred on the bed (y 19.6..236.4).
# Row 1: link_23 (105.7 x 60.2 after rotation), stator, top; row 2: 3 stators, top;
# row 3: 2 tops, 2 bottoms; row 4: 2 bottoms. Row 1 starts at x 19.65 (P1S exclude zone 18x28).
J4_LAYOUT = [
    ('link_23', 'link_23', ('bed', (1, 0, 0)), (72.5, 49.75)),
    ('stator_1', 'stator', None, (162.85, 49.75)),
    ('stator_2', 'stator', None, (40.25, 117.375)),
    ('stator_3', 'stator', None, (104.75, 117.375)),
    ('stator_4', 'stator', None, (169.25, 117.375)),
    ('bottom_1', 'bottom', None, (151.75, 172.875)),
    ('bottom_2', 'bottom', None, (197.25, 172.875)),
    ('bottom_3', 'bottom', None, (105.25, 218.875)),
    ('bottom_4', 'bottom', None, (150.75, 218.875)),
]
# all (EXO-348): one plate, 14 objects; footprints link_23 105.7 x 60.2, link_12 63.5 x 74.5,
# stator 54, top 36, bottom 35; nominal gaps >= 11, layout x 9.88..246.12, y 11.75..244.26.
ALL_LAYOUT = [
    ('link_23', 'link_23', ('bed', (1, 0, 0)), (71.73, 41.88)),
    ('link_12', 'link_12', ('bed', (-0.000199314, 0.961245874, -0.275692454)), (167.35, 49.01)),
    ('stator_1', 'stator', None, (36.88, 110.0)),
    ('stator_2', 'stator', None, (101.88, 126.86)),   # EXO-375: +2.6, the fused link_12 is 79.7 deep
    ('stator_3', 'stator', None, (166.88, 126.86)),
    ('stator_4', 'stator', None, (36.88, 175.0)),
    ('bottom_1', 'bottom', None, (222.38, 123.25)),
    ('bottom_2', 'bottom', None, (222.38, 169.25)),
    ('bottom_3', 'bottom', None, (186.38, 215.25)),
    ('bottom_4', 'bottom', None, (92.38, 226.76)),
]
# joints2 / links2 (EXO-394): link_12 ROTOR SIDE DOWN (the joint face -8.0 on the bed: R maps
# -Z_J1 onto -Z), link_23 on its J3 flange end face as before - it cannot go rotor side down,
# its bar hangs to module z -27.0, 19 mm past the joint face. joints2: link_12, link_23, 2
# stators, 2 bottoms (pegs up); links2: the two links. Footprints link_23 105.7 x 60.26, link_12
# 58.5 x 66.84 (73 tall, 410.2 mm2 on the bed); rows gap 12, centred.
J2_LAYOUT = [
    ('link_23', 'link_23', ('bed', (1, 0, 0)), (92.75, 95.0)),
    ('link_12', 'link_12', ('bed', (-0.000199314, 0.961245874, -0.275692454)), (186.85, 95.0)),
    ('stator_1', 'stator', None, (48.0, 167.42)),
    ('stator_2', 'stator', None, (114.0, 167.42)),
    ('bottom_1', 'bottom', None, (170.5, 167.42)),
    ('bottom_2', 'bottom', None, (217.5, 167.42)),
]
LINKS2_LAYOUT = [
    ('link_23', 'link_23', ('bed', (1, 0, 0)), (92.75, 128.0)),
    ('link_12', 'link_12', ('bed', (-0.000199314, 0.961245874, -0.275692454)), (186.85, 128.0)),
]
# stator_dia (EXO-355): five stators 54 x 54, rows of 3 + 2 at 64.5 pitch (gap 10.5), centred on 128/128.
SDIA_LAYOUT = [
    ('stator_d42.2', 'stator_d42.2', None, (63.5, 160.25)),
    ('stator_d42.3', 'stator_d42.3', None, (128.0, 160.25)),
    ('stator_d42.4', 'stator_d42.4', None, (192.5, 160.25)),
    ('stator_d42.5', 'stator_d42.5', None, (95.75, 95.75)),
    ('stator_d42.6', 'stator_d42.6', None, (160.25, 95.75)),
]
# rotor_head (EXO-352): bottom_h0.5 / bottom_h1.0 as exported (O35 plate down, as rotor.bambu.3mf),
# 35 x 35, pitch 45.5 -> gap 10.5, centred on 128/128.
HEAD_LAYOUT = [
    ('bottom_h0.5', 'bottom_h0.5', None, (105.25, 128.0)),
    ('bottom_h1.0', 'bottom_h1.0', None, (150.75, 128.0)),
]
# stator_dia_low (EXO-358): stator_d41.8 / 41.9 / 42.0 / 42.1 + the existing stator_d42.2 / 42.3 / 42.4,
# as exported (underside on the bed), rows of 3 + 3 + 1 at 64.5 pitch (gap 10.5), centred on 128/128
# (4 across would be 247.5 wide, margins 4.25 < MARGIN).
SDIA_LOW_LAYOUT = [
    ('stator_d41.8', 'stator_d41.8', None, (63.5, 192.5)),
    ('stator_d41.9', 'stator_d41.9', None, (128.0, 192.5)),
    ('stator_d42.0', 'stator_d42.0', None, (192.5, 192.5)),
    ('stator_d42.1', 'stator_d42.1', None, (63.5, 128.0)),
    ('stator_d42.2', 'stator_d42.2', None, (128.0, 128.0)),
    ('stator_d42.3', 'stator_d42.3', None, (192.5, 128.0)),
    ('stator_d42.4', 'stator_d42.4', None, (128.0, 63.5)),
]
# rotor (EXO-338, layout read off the file it built; rebuilt by EXO-364 with the keyed top):
# top rotor face down ('flipX', = the key sockets on the bed), bottom O35 plate down, 46 apart.
# NOTE rotor.bambu.3mf is also SETTINGS_SRC: its settings are read before it is rewritten.
ROTOR_LAYOUT = [
    ('top', 'top', 'flipX', (105.0, 128.0)),
    ('bottom', 'bottom', None, (151.0, 128.0)),
]
# body_mount (EXO-401): the hand-made chest mount (Fusion `arm complete_chest mount`, doc frame =
# arm frame) on its x -45 robot bolt face (15655 mm2, 8x O3.5 counterbores open on the bed):
# R maps -X onto -Z, the J1 seat pads end up vertical, their O4.0 holes horizontal. Footprint
# 114.15 x 176.43 (long side along Y), 75 tall, centred on 128/128. Ceilings at 3 / 5 / 7 / 55
# above the bed take supports.
BODY_MOUNT_LAYOUT = [
    ('body_mount', 'body_mount', ('bed', (-1, 0, 0)), (128.0, 128.0)),
]
# links34 / joints34 (EXO-402): link_34 ROTOR SIDE DOWN (joint face -8.0 of J3, 410.2 mm2, on the
# bed: R maps -Z_J3 = arm +Z onto -Z; 115.9 tall; the pad underside ring 8 above the bed and the
# body take supports; CoM 18.9 from the J3 axis, outside the r 15.05 foot, inside the r 27 pad).
# link_45 cannot go rotor side down (its J5 stator pad reaches module z -25.1, 17.1 past the J4
# joint face) nor pad face down (the ramp reaches 29.4 past the pad face: 440 mm2 foot, 3450 mm2
# of supports, counterbores facing down on them): it stands on the MATING face of its J5 stator
# pad (arm x 91.3, normal +X, 1780.1 mm2; the 0.6 x 0.4 assembly mark is a groove in the bed
# face), 102.5 tall, CoM over the foot, J4 axis horizontal (journal, sockets, counterbores print
# sideways). Its only two support planes are these. To lay link_34 flat instead (J4 stator pad outer face, arm +Y, 4669.7 mm2, 58.5
# tall, J3 axis horizontal) use ('bed', (0, 1, 0)) and its footprint 69.2 x 115.9.
# EXO-405 (Esteban): link_45 ROTOR SIDE UP instead: J4 axis (arm Y) vertical, joint face y -239.0,
# journal and sockets up. Its pad outer face (y -255.0, one plane to 0.012, 2304.3 mm2, the M3
# counterbores) cannot touch the bed: the J5 end (arm x 53..91) reaches y -284.4, 29.4 past it, so
# the bed face is that end face (normal -Y, 440.3 mm2, x 73.7..83.7 of the part), 62.5 tall, CoM
# 23 mm off the foot; the pad face sits 29.4 above the bed on supports (normal, dense interface).
# link_34 unchanged, with an outer-only 6 mm brim (OBJECT_SETTINGS).
# Footprints link_34 60.66 x 71.89, link_45 102.50 x 64.02; row 175.16 wide, gap 12, centred;
# joints34 row 2 = joints2 row 2.
LINKS34_LAYOUT = [
    ('link_34', 'link_34', ('bed', (0, 0, 1)), (70.75, 128.0)),
    ('link_45', 'link_45', ('bed', (0, -1, 0)), (164.33, 128.0)),
]
J34_LAYOUT = [
    ('link_34', 'link_34', ('bed', (0, 0, 1)), (70.75, 95.0)),
    ('link_45', 'link_45', ('bed', (0, -1, 0)), (164.33, 95.0)),
    ('stator_1', 'stator', None, (48.0, 169.94)),
    ('stator_2', 'stator', None, (114.0, 169.94)),
    ('bottom_1', 'bottom', None, (170.5, 169.94)),
    ('bottom_2', 'bottom', None, (217.5, 169.94)),
]
PLATES = {
    'body_mount': (os.path.join(PARTS, 'body_mount.bambu.3mf'), 'body_mount', BODY_MOUNT_LAYOUT),
    'link': (os.path.join(PARTS, 'link.bambu.3mf'), 'link', LINK_LAYOUT),
    'joints4': (os.path.join(PARTS, 'joints4.bambu.3mf'), 'joints4', J4_LAYOUT),
    'all': (os.path.join(PARTS, 'all.bambu.3mf'), 'all', ALL_LAYOUT),
    'joints2': (os.path.join(PARTS, 'joints2.bambu.3mf'), 'joints2', J2_LAYOUT),
    'links2': (os.path.join(PARTS, 'links2.bambu.3mf'), 'links2', LINKS2_LAYOUT),
    'stator_dia': (os.path.join(PARTS, 'stator_dia.bambu.3mf'), 'stator_dia', SDIA_LAYOUT),
    'rotor_head': (os.path.join(PARTS, 'rotor_head.bambu.3mf'), 'rotor_head', HEAD_LAYOUT),
    'stator_dia_low': (os.path.join(PARTS, 'stator_dia_low.bambu.3mf'), 'stator_dia_low', SDIA_LOW_LAYOUT),
    'links34': (os.path.join(PARTS, 'links34.bambu.3mf'), 'links34', LINKS34_LAYOUT),
    'joints34': (os.path.join(PARTS, 'joints34.bambu.3mf'), 'joints34', J34_LAYOUT),
}
# EXO-364: top / link_12 / link_23 re-exported with the rotor key (was 5786.178 / 40392.766 / 45818.953)
SOURCE_V = {'stator': 25680.834, 'link_12': 40406.203, 'top': 5772.270, 'bottom': 3865.634,
            'link_23': 45832.391, 'stator_d42.2': 25630.734, 'stator_d42.3': 25584.431,
            'stator_d42.4': 25537.812, 'stator_d42.5': 25491.064, 'stator_d42.6': 25443.948}
SOURCE_V.update({'stator_d41.8': 25815.537, 'stator_d41.9': 25769.600, 'stator_d42.0': 25723.026,
                 'stator_d42.1': 25677.170})   # EXO-358
SOURCE_V.update({'bottom_h0.5': 3831.022, 'bottom_h1.0': 3799.806})   # EXO-352
# EXO-370/375: fused links (pad 7 default, pad 6 test) and the new bottom (boss r 7.8)
SOURCE_V.update({'bottom': 3578.998})   # EXO-394: O3.0 x 2.0 pegs on the bottom
# EXO-383: pad 8 + O6.2 head counterbore 1.0 / 1.5 / 2.0 (the EXO-375 p7 / p6 links are archived)
SOURCE_V.update({'link_12': 44324.563, 'link_23': 49751.082})   # EXO-394 (c2.0, sockets, chamfer)
SOURCE_V.update({'body_mount': 200380.178})   # EXO-401 (Fusion Body1 200387.392)
SOURCE_V.update({'link_34': 63225.829, 'link_45': 62060.991})   # EXO-402 (Fusion 63232.554 / 62072.567)
MARGIN = 8.0   # min distance of every object to the bed edge (brim)
PLATE_MARGIN = {'joints2': 10.0, 'links2': 10.0}   # stricter per-plate margins (EXO-369)
PLATE_MARGIN.update({'links34': 10.0, 'joints34': 10.0})   # EXO-402
# EXO-406: default filament for plates built from now on: Bambu PLA-CF, Matcha Green. Values =
# preset "Bambu PLA-CF @BBL X1C" (GFSA50_01; lists P1S 0.4 nozzle as compatible) with inheritance
# resolved (-> @base -> fdm_filament_pla -> fdm_filament_common, include
# fdm_filament_template_direct_dual) from the installed system profiles (BBL 02.04.00.10): only the
# keys that differ from the reference's PLA Basic @BBL P1S 0.4 nozzle (whose 98 filament keys the
# same resolution reproduces exactly). Colour: filaments_color_codes.json, GFA50 code 14500 "Matcha
# Green" #5c9748FF. All 28 equal the GUI-saved FILAMENT_XCHECK, which --check asserts.
PLA_CF_MATCHA = {
    'additional_cooling_fan_speed': ['0'],
    'cool_plate_temp': ['45'],
    'cool_plate_temp_initial_layer': ['45'],
    'filament_change_length': ['10'],
    'filament_colour': ['#5C9748'],
    'filament_cost': ['34.99'],
    'filament_density': ['1.22'],
    'filament_deretraction_speed': ['nil', 'nil'],
    'filament_flow_ratio': ['0.98', '0.975'],
    'filament_ids': ['GFA50'],
    'filament_max_volumetric_speed': ['15', '25'],
    'filament_multi_colour': ['#5C9748'],
    'filament_prime_volume': ['45'],
    'filament_retraction_length': ['nil', '0.8'],
    'filament_retraction_speed': ['nil', 'nil'],
    'filament_scarf_gap': ['15%'],
    'filament_settings_id': ['Bambu PLA-CF @BBL X1C'],
    'filament_type': ['PLA-CF'],
    'filament_z_hop': ['nil', '0.4'],
    'impact_strength_z': ['7.8'],
    'nozzle_temperature': ['230', '230'],
    'nozzle_temperature_initial_layer': ['230', '230'],
    'nozzle_temperature_range_high': ['250'],
    'nozzle_temperature_range_low': ['210'],
    'required_nozzle_HRC': ['40'],
    'slow_down_layer_time': ['8'],
    'supertack_plate_temp': ['50'],
    'supertack_plate_temp_initial_layer': ['50'],
}
FILAMENT_XCHECK = os.path.join(PARTS, 'archive', '2026-10-tests', 'rotor_dia.bambu.3mf')   # GUI-saved, same preset
# Plates built before EXO-406 keep PLA Basic (their files are not rebuilt); drop a name here to
# rebuild that plate in PLA-CF.
LEGACY_PLA_BASIC = {'body_mount', 'link', 'joints4', 'all', 'joints2', 'links2', 'stator_dia',
                    'rotor_head', 'stator_dia_low'}
# EXO-405: normal supports with a dense top interface; brim only where OBJECT_SETTINGS asks
# (auto_brim would pick its own brims). Key names as in the reference and in the 02.04 binary.
SUPPORT_EXO405 = {'support_type': 'normal(auto)', 'support_top_z_distance': '0.15',
                  'support_interface_top_layers': '3', 'support_interface_spacing': '0.2',
                  'brim_type': 'no_brim'}
PLATE_SETTINGS = {'links34': SUPPORT_EXO405, 'joints34': SUPPORT_EXO405}
# per-object settings (model_settings.config <object> metadata), by plate and object name
OBJECT_SETTINGS = {('links34', 'link_34'): [('brim_type', 'outer_only'), ('brim_width', '6')],
                   ('joints34', 'link_34'): [('brim_type', 'outer_only'), ('brim_width', '6')]}


def settings_overrides(plate):
    o = {} if plate in LEGACY_PLA_BASIC else dict(PLA_CF_MATCHA)
    o.update(PLATE_SETTINGS.get(plate, {}))
    return o


def project_settings(plate):
    """Reference project_settings.config with the plate's overrides, serialised as Bambu does
    (sorted keys, indent 4, trailing newline: reproduces the reference byte for byte)."""
    import json
    raw = zipfile.ZipFile(SETTINGS_SRC).read('Metadata/project_settings.config')
    o = settings_overrides(plate)
    if not o:
        return raw
    ps = json.loads(raw)
    assert (json.dumps(ps, indent=4, ensure_ascii=False) + '\n').encode() == raw
    for k, v in o.items():
        assert k in ps and type(ps[k]) is type(v) and ps[k] != v, k   # only existing keys, real changes
        ps[k] = v
    return (json.dumps(ps, indent=4, ensure_ascii=False) + '\n').encode()
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
    T = [(int(t.get('v1')), int(t.get('v2')), int(t.get('v3'))) for t in m.findall('m:triangles/m:triangle', NS)]
    return V, T


def tri_n(V, t):
    a, b, c = (V[i] for i in t)
    u = [b[k] - a[k] for k in range(3)]
    w = [c[k] - a[k] for k in range(3)]
    return [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]]


def volume(V, T):
    s = 0.0
    for a, b, c in T:
        (x1, y1, z1), (x2, y2, z2), (x3, y3, z3) = V[a], V[b], V[c]
        s += x1 * (y2 * z3 - y3 * z2) - x2 * (y1 * z3 - y3 * z1) + x3 * (y1 * z2 - y2 * z1)
    return s / 6.0


def bed_face_rotation(V, T, axis=(-1, 0, 0)):
    """Rotation taking the area-weighted normal of the flat face at the extreme of the mesh
    along `axis` (unit, source coords) onto -Z."""
    proj = lambda p: sum(axis[k] * p[k] for k in range(3))
    pmax = max(proj(v) for v in V)
    n = [0.0, 0.0, 0.0]
    for t in T:
        if all(proj(V[i]) > pmax - 0.2 for i in t):
            nn = tri_n(V, t)
            a = math.sqrt(sum(x * x for x in nn))
            if a > 1e-9 and proj(nn) / a > 0.9999:  # coplanar face triangles only
                n = [n[k] + nn[k] for k in range(3)]
    L = math.sqrt(sum(c * c for c in n))
    n = [c / L for c in n]
    assert proj(n) > 0.9999, n
    tgt = [0.0, 0.0, -1.0]
    k = [n[1] * tgt[2] - n[2] * tgt[1], n[2] * tgt[0] - n[0] * tgt[2], n[0] * tgt[1] - n[1] * tgt[0]]
    s = math.sqrt(sum(c * c for c in k))
    c = sum(n[i] * tgt[i] for i in range(3))
    k = [x / s for x in k]
    K = [[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]]
    K2 = [[sum(K[i][m] * K[m][j] for m in range(3)) for j in range(3)] for i in range(3)]
    R = [[(1 if i == j else 0) + s * K[i][j] + (1 - c) * K2[i][j] for j in range(3)] for i in range(3)]
    return R, n


def f(x):
    return ('%.6f' % x).rstrip('0').rstrip('.') if abs(x) > 5e-7 else '0'


def oriented(srcname, rot, name=None):
    """Source mesh in print orientation, bbox-centred; returns V, T, centre, half height."""
    V, T = read_mesh(os.path.join(PARTS, srcname + '.3mf'))
    if rot == 'flipX':
        V = [(x, -y, -z) for x, y, z in V]
    elif rot:
        R, n = bed_face_rotation(V, T, rot[1])
        if name:
            print('%s bed-face normal (source coords): %.6f %.6f %.6f' % ((name,) + tuple(n)))
            print('R rows: ' + ' | '.join(' '.join('%.6f' % x for x in r) for r in R))
        V = [tuple(sum(R[r][k] * v[k] for k in range(3)) for r in range(3)) for v in V]
    lo = [min(v[k] for v in V) for k in range(3)]
    hi = [max(v[k] for v in V) for k in range(3)]
    c = [(lo[k] + hi[k]) / 2 for k in range(3)]
    V = [(x - c[0], y - c[1], z - c[2]) for x, y, z in V]
    return V, T, c, (hi[2] - lo[2]) / 2


def mesh_text(V, T):
    return (''.join('     <vertex x="%s" y="%s" z="%s"/>\n' % (f(x), f(y), f(z)) for x, y, z in V) + '    </vertices>\n    <triangles>\n'
            + ''.join('     <triangle v1="%d" v2="%d" v3="%d"/>\n' % t for t in T))


def build(plate='link'):
    OUT, title, LAYOUT = PLATES[plate]
    src = zipfile.ZipFile(SETTINGS_SRC)
    objects, items, rels, ms_objs, ms_inst, assemble, files = [], [], [], [], [], [], {}
    for i, (name, srcname, rot, (px, py)) in enumerate(LAYOUT):
        V, T, c, tz = oriented(srcname, rot, name)
        mid, oid, sub = 2 * i + 1, 2 * i + 2, 'object_%d.model' % (i + 1)
        body = [HDR, ' <metadata name="BambuStudio:3mfVersion">1</metadata>\n <resources>\n',
                '  <object id="%d" name="%s" p:UUID="%s" type="model">\n   <mesh>\n    <vertices>\n' % (mid, name, uuid.uuid4())]
        body.append(mesh_text(V, T))
        body.append('    </triangles>\n   </mesh>\n  </object>\n </resources>\n <build/>\n</model>\n')
        files['3D/Objects/' + sub] = ''.join(body)
        objects.append('  <object id="%d" name="%s" p:UUID="%s" type="model">\n   <components>\n'
                       '    <component p:path="/3D/Objects/%s" objectid="%d" p:UUID="%s" transform="1 0 0 0 1 0 0 0 1 0 0 0"/>\n'
                       '   </components>\n  </object>\n' % (oid, name, uuid.uuid4(), sub, mid, uuid.uuid4()))
        tr = '1 0 0 0 1 0 0 0 1 %s %s %s' % (f(px), f(py), f(tz))
        items.append('  <item objectid="%d" p:UUID="%s" transform="%s" printable="1"/>\n' % (oid, uuid.uuid4(), tr))
        rels.append(' <Relationship Target="/3D/Objects/%s" Id="rel-%d" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>\n' % (sub, i + 1))
        objset = ''.join('    <metadata key="%s" value="%s"/>\n' % kv for kv in OBJECT_SETTINGS.get((plate, name), []))
        ms_objs.append(('  <object id="%d">\n    <metadata key="name" value="%s"/>\n' + objset.replace('%', '%%') + '    <metadata key="extruder" value="1"/>\n'
                       '    <metadata face_count="%d"/>\n    <part id="%d" subtype="normal_part">\n'
                       '      <metadata key="name" value="%s"/>\n      <metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/>\n'
                       '      <metadata key="source_file" value="%s.3mf"/>\n      <metadata key="source_object_id" value="0"/>\n'
                       '      <metadata key="source_volume_id" value="0"/>\n'
                       '      <metadata key="source_offset_x" value="%s"/>\n      <metadata key="source_offset_y" value="%s"/>\n'
                       '      <metadata key="source_offset_z" value="%s"/>\n'
                       '      <mesh_stat face_count="%d" edges_fixed="0" degenerate_facets="0" facets_removed="0" facets_reversed="0" backwards_edges="0"/>\n'
                       '    </part>\n  </object>\n') % (oid, name, len(T), mid, name, srcname, f(c[0]), f(c[1]), f(c[2]), len(T)))
        ms_inst.append('    <model_instance>\n      <metadata key="object_id" value="%d"/>\n'
                       '      <metadata key="instance_id" value="0"/>\n      <metadata key="identify_id" value="%d"/>\n'
                       '    </model_instance>\n' % (oid, 100 + i))
        assemble.append('   <assemble_item object_id="%d" instance_id="0" transform="%s" offset="0 0 0" />\n' % (oid, tr))
    meta = ''.join(' <metadata name="%s">%s</metadata>\n' % kv for kv in [
        ('Application', 'BambuStudio-02.04.00.70'), ('BambuStudio:3mfVersion', '1'),
        ('CreationDate', '2026-10-22'), ('ModificationDate', '2026-10-22'), ('Title', title)])
    files['3D/3dmodel.model'] = (HDR + meta + ' <resources>\n' + ''.join(objects) + ' </resources>\n <build p:UUID="%s">\n' % uuid.uuid4()
                                 + ''.join(items) + ' </build>\n</model>\n')
    files['3D/_rels/3dmodel.model.rels'] = ('<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">\n'
                                            + ''.join(rels) + '</Relationships>')
    files['_rels/.rels'] = ('<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">\n'
                            ' <Relationship Target="/3D/3dmodel.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>\n</Relationships>')
    for k in ('[Content_Types].xml', 'Metadata/slice_info.config', 'Metadata/filament_sequence.json'):
        files[k] = src.read(k)
    files['Metadata/project_settings.config'] = project_settings(plate)
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


def check(path):
    """Independent read-back: walks build item -> wrapper -> component p:path/objectid."""
    import hashlib, json, re
    z = zipfile.ZipFile(path)
    assert z.testzip() is None
    root = ET.fromstring(z.read('3D/3dmodel.model'))
    ids = []
    wrappers = {o.get('id'): o for o in root.findall('m:resources/m:object', NS)}
    ms = ET.fromstring(z.read('Metadata/model_settings.config'))
    ms_names = {o.get('id'): o.find("metadata[@key='name']").get('value') for o in ms.findall('object')}
    ms_parts = {o.get('id'): o.find('part').get('id') for o in ms.findall('object')}
    ms_src = {o.get('id'): o.find("part/metadata[@key='source_file']").get('value')[:-4] for o in ms.findall('object')}
    plate = [mi.find("metadata[@key='object_id']").get('value') for mi in ms.findall('plate/model_instance')]
    rows, boxes, md5 = [], [], set()
    key = [k for k, v in PLATES.items() if os.path.abspath(v[0]) == os.path.abspath(path)]
    margin = PLATE_MARGIN.get(key[0], MARGIN) if key else MARGIN
    layout = {e[0]: e for e in PLATES[key[0]][2]} if key else {}
    mesh_md5 = []
    for it in root.findall('m:build/m:item', NS):
        oid = it.get('objectid')
        t = [float(x) for x in it.get('transform').split()]
        assert t[:9] == [1, 0, 0, 0, 1, 0, 0, 0, 1], 'non-translation build transform'
        w = wrappers[oid]
        comp = w.find('m:components/m:component', NS)
        sub, mid = comp.get(PNS).lstrip('/'), comp.get('objectid')
        raw = z.read(sub)
        md5.add(hashlib.md5(raw).hexdigest())
        sroot = ET.fromstring(raw)
        so = sroot.findall('m:resources/m:object', NS)
        assert len(so) == 1 and so[0].get('id') == mid
        k = int(re.search(r'object_(\d+)\.model', sub).group(1))
        assert int(mid) == 2 * k - 1 and int(oid) == 2 * k, (sub, mid, oid)
        assert ms_names[oid] == w.get('name') == so[0].get('name') and ms_parts[oid] == mid and oid in plate
        ids += [oid, mid]
        if w.get('name') in layout:   # mesh block vs the same block regenerated from parts/<source>.3mf
            e = layout[w.get('name')]
            Vs, Ts = oriented(e[1], e[2])[:2]
            a = raw.index(b'<vertices>\n') + len(b'<vertices>\n')
            got = hashlib.md5(raw[a:raw.index(b'    </triangles>')]).hexdigest()
            want = hashlib.md5(mesh_text(Vs, Ts).encode()).hexdigest()
            srcmd5 = hashlib.md5(open(os.path.join(PARTS, e[1] + '.3mf'), 'rb').read()).hexdigest()
            mesh_md5.append((w.get('name'), e[1], srcmd5, got, want))
            assert got == want, (w.get('name'), got, want)
        m = so[0].find('m:mesh', NS)
        V = [(float(v.get('x')) + t[9], float(v.get('y')) + t[10], float(v.get('z')) + t[11]) for v in m.findall('m:vertices/m:vertex', NS)]
        T = [(int(q.get('v1')), int(q.get('v2')), int(q.get('v3'))) for q in m.findall('m:triangles/m:triangle', NS)]
        edges = {}
        for a, b, c in T:
            for e in ((a, b), (b, c), (c, a)):
                edges[e] = edges.get(e, 0) + 1
        open_e = sum(1 for (a, b) in edges if (b, a) not in edges)
        lo = [min(v[q] for v in V) for q in range(3)]
        hi = [max(v[q] for v in V) for q in range(3)]
        bed = 0.0
        for tri in T:
            if all(V[i][2] < lo[2] + 0.01 for i in tri):
                bed += math.sqrt(sum(x * x for x in tri_n(V, tri))) / 2
        vol = volume(V, T)
        srcv = SOURCE_V[ms_src[oid]]
        rows.append((oid, mid, sub, w.get('name'), vol, srcv, lo, hi, bed, open_e))
        boxes.append((w.get('name'), lo, hi))
    names = [r[3] for r in rows]
    assert len(ids) == len(set(ids)) == 2 * len(rows) and len(set(names)) == len(names)
    print('objects %d, ids %s, names distinct' % (len(rows), ','.join(sorted(ids, key=int))))
    print('| obj/mesh | file | name | V (mm3) | source | min z | bbox x | bbox y | height | bed mm2 | open edges |')
    print('|---|---|---|---|---|---|---|---|---|---|---|')
    for oid, mid, sub, name, vol, srcv, lo, hi, bed, oe in rows:
        print('| %s/%s | %s | %s | %.3f | %.3f | %.3f | %.2f..%.2f | %.2f..%.2f | %.3f | %.1f | %d |'
              % (oid, mid, sub.split('/')[-1], name, vol, srcv, lo[2], lo[0], hi[0], lo[1], hi[1], hi[2] - lo[2], bed, oe))
        assert abs(vol - srcv) < 0.002 and abs(lo[2]) < 1e-5 and oe == 0
        assert min(lo[0], lo[1], 256 - hi[0], 256 - hi[1]) >= margin and hi[2] <= 250
        assert not (lo[0] < 18 and lo[1] < 28)
    print('distinct sub-model md5:', len(md5))
    for name, src, sm, got, want in mesh_md5:
        print('mesh md5 %s: plate %s = from %s.3mf (file md5 %s) %s: %s' % (name, got, src, sm, want, got == want))
    gaps = []
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            a, b = boxes[i], boxes[j]
            gx = max(b[1][0] - a[2][0], a[1][0] - b[2][0])
            gy = max(b[1][1] - a[2][1], a[1][1] - b[2][1])
            gaps.append((a[0], b[0], max(gx, gy)))
            print('gap %s - %s: %.2f mm' % (a[0], b[0], max(gx, gy)))
    assert not gaps or min(g[2] for g in gaps) >= 10   # EXO-401: one-object plates have no gaps
    LO = [min(b[1][k] for b in boxes) for k in range(2)]
    HI = [max(b[2][k] for b in boxes) for k in range(2)]
    print('margin required >= %.1f' % margin)
    print('layout bbox x %.2f..%.2f, y %.2f..%.2f (%.2f x %.2f); margins left %.2f right %.2f front %.2f back %.2f'
          % (LO[0], HI[0], LO[1], HI[1], HI[0] - LO[0], HI[1] - LO[1], LO[0], 256 - HI[0], LO[1], 256 - HI[1]))
    if gaps:
        g = min(gaps, key=lambda q: q[2])
        print('smallest gap %.2f mm (%s - %s)' % (g[2], g[0], g[1]))
    txt = z.read('Metadata/project_settings.config')
    ps = json.loads(txt)
    ref = json.loads(zipfile.ZipFile(SETTINGS_SRC).read('Metadata/project_settings.config'))
    keys = ['printer_settings_id', 'filament_settings_id', 'filament_colour', 'print_settings_id', 'layer_height',
            'enable_support', 'support_type', 'support_threshold_angle', 'support_on_build_plate_only',
            'support_top_z_distance', 'support_interface_top_layers', 'support_interface_spacing', 'brim_type', 'brim_width']
    for k in keys:
        print('%s = %s' % (k, ps[k]))
    over = settings_overrides(key[0]) if key else {}
    changed = sorted(k for k in set(ps) | set(ref) if ps.get(k) != ref.get(k))
    print('keys changed vs the reference %s: %d (declared overrides %d)'
          % (os.path.basename(SETTINGS_SRC), len(changed), len(over)))
    for k in changed:
        print('  %s: %s -> %s' % (k, json.dumps(ref.get(k)), json.dumps(ps.get(k))))
    assert set(ps) == set(ref) and changed == sorted(over) and all(ps[k] == v for k, v in over.items())
    same = txt == (project_settings(key[0]) if key else zipfile.ZipFile(SETTINGS_SRC).read('Metadata/project_settings.config'))
    print('project_settings.config byte-identical to the reference + declared overrides:', same)
    assert same and ps['enable_support'] == '1' and ps['support_type'] == over.get('support_type', 'tree(auto)')
    assert ps['support_threshold_angle'] == '30' and ps['support_on_build_plate_only'] == '0'
    fil = [k for k in over if k in PLA_CF_MATCHA]
    if fil:   # the filament keys as Bambu Studio itself saved this preset (GUI project)
        xc = json.loads(zipfile.ZipFile(FILAMENT_XCHECK).read('Metadata/project_settings.config'))
        bad = [k for k in fil if xc[k] != ps[k]]
        print('filament keys = %s (GUI-saved, %s): %d of %d' % (os.path.basename(FILAMENT_XCHECK),
              xc['filament_settings_id'][0], len(fil) - len(bad), len(fil)))
        assert not bad, bad
    objset = {}
    for o in ms.findall('object'):
        extra = [(m.get('key'), m.get('value')) for m in o.findall('metadata')
                 if m.get('key') not in (None, 'name', 'extruder')]
        if extra:
            objset[ms_names[o.get('id')]] = extra
    want = {n: v for (pl, n), v in OBJECT_SETTINGS.items() if key and pl == key[0]}
    print('per-object settings:', objset or 'none')
    assert objset == want, (objset, want)
    print('READBACK OK')


if __name__ == '__main__':
    a = sys.argv
    plate = a[a.index('--plate') + 1] if '--plate' in a else 'link'
    if '--check' not in a:
        build(plate)
    check(a[a.index('--check') + 1] if '--check' in a and len(a) > a.index('--check') + 1 else PLATES[plate][0])
