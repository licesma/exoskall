"""EXO-457: build ONE arm at ONE width mode into its OWN new, separately named document, for the
width-52 check. Never touches `exo generated`, `arm complete` or any other existing document,
never writes the exports reports, parts or plates, and never edits the WIDTH_MODE default.

Run through a wrapper that sets ARM ('g1' | 'luna') and MODE ('manual' | 'derived'):
    /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/width_build_exo457_g1_derived.py --timeout 2700
(the four wrappers: width_build_exo457_{g1,luna}_{manual,derived}.py).

What it does:
  1. refuses if a document of the target name is already open or already in the target folder
     (so it can never rebuild anything that exists);
  2. adds a NEW document, direct design, inserts `sensor module 2` LINKED (read-only), saves it
     AS `width52 check G1` / `width manual check G1` / ... into the cloud folder
     `exo457 width check` (made under the folder of `exo generated` if missing);
  3. builds the arm with `BuildArmGen.build_arm` on a COPY of the arm spec whose 'links' come
     from `g1_links(MODE)` / `luna_links(MODE)` (the runtime override; the module default is
     not touched) and whose 'report' is /tmp/exo457/build_<arm>_<mode>.txt;
  4. saves the new document again, then re-activates whatever was active before.
"""
import importlib
import os
import sys
import time

sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
import armgen, sweepgen, BuildArm, BuildArmGen  # noqa: E401
importlib.reload(armgen); importlib.reload(sweepgen); importlib.reload(BuildArm)
importlib.reload(BuildArmGen)

ARM = globals().get('ARM')
MODE = globals().get('MODE')
if ARM not in ('g1', 'luna') or MODE not in BuildArmGen.WIDTH_MODES:
    raise RuntimeError('run through a wrapper that sets ARM and MODE (got %r %r)' % (ARM, MODE))
DOC = '%s check %s' % ('width52' if MODE == 'derived' else 'width manual',
                       'G1' if ARM == 'g1' else 'Luna')
OUT = '/tmp/exo457'
SUBFOLDER = 'exo457 width check'
GEN_ID = 'urn:adsk.wipprod:dm.lineage:I6f3yGrzR8S8xg8G0BP0aw'   # exo generated (folder anchor only)
SRC_ID = 'urn:adsk.wipprod:dm.lineage:CzEATRrQSnuX-w_dwKCH6A'   # sensor module 2 (inserted LINKED)
PROTECTED = ('exo generated', 'arm complete')

_log = log


def log(m):   # noqa: F811 - the bridge log is ASCII
    _log(str(m).encode('ascii', 'backslashreplace').decode())


def open_docs():
    return [(app.documents.item(i).name, app.documents.item(i).isModified)
            for i in range(app.documents.count)]


assert BuildArmGen.WIDTH_MODE == 'manual', 'the committed default must stay manual'
os.makedirs(OUT, exist_ok=True)
prev = app.activeDocument
log('EXO-457 build %s %s -> document %r; WIDTH_MODE default %r (not changed)'
    % (ARM, MODE, DOC, BuildArmGen.WIDTH_MODE))
log('before: active %r; open %s' % (prev.name if prev else None, open_docs()))
if any(n == DOC for n, _m in open_docs()):
    raise RuntimeError('a document %r is already open - refusing (never rebuild)' % DOC)

gen_df = app.data.findFileById(GEN_ID)
parent = gen_df.parentFolder
folder = None
for i in range(parent.dataFolders.count):
    if parent.dataFolders.item(i).name == SUBFOLDER:
        folder = parent.dataFolders.item(i)
if folder is None:
    folder = parent.dataFolders.add(SUBFOLDER)
    log('made cloud folder %r under %r' % (SUBFOLDER, parent.name))
for i in range(folder.dataFiles.count):
    if folder.dataFiles.item(i).name == DOC:
        raise RuntimeError('%r already exists in %r - refusing (never rebuild)' % (DOC, SUBFOLDER))

doc = None
t0 = time.time()
try:
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.DirectDesignType
    src = app.data.findFileById(SRC_ID)
    occ = des.rootComponent.occurrences.addByInsert(src, adsk.core.Matrix3D.create(), True)
    log('inserted %r v%s LINKED (referenced %s)' % (src.name, src.versionNumber,
                                                    occ.isReferencedComponent))
    if not doc.saveAs(DOC, folder, 'EXO-457 width check, empty + linked module', ''):
        raise RuntimeError('saveAs %r failed' % DOC)
    log('SAVED new document %r into %r' % (doc.name, folder.name))
    base = BuildArmGen.G1_SPEC if ARM == 'g1' else BuildArmGen.LUNA_SPEC
    links = BuildArmGen.g1_links(MODE) if ARM == 'g1' else BuildArmGen.luna_links(MODE)
    spec = dict(base, links=links, report=os.path.join(OUT, 'build_%s_%s.txt' % (ARM, MODE)))
    log('widths: %s' % ', '.join('%s %.1f' % (L['name'], L['width']) for L in links))
    text = BuildArmGen.build_arm(spec, app, ui, des, log)
    log('BUILD RETURNED in %.1f s; head: %s' % (time.time() - t0, text.split('\n', 1)[0]))
    if app.activeDocument is None or app.activeDocument.name != DOC:
        raise RuntimeError('active document changed under the build (%r) - NOT saving'
                           % (app.activeDocument.name if app.activeDocument else None))
    log('save: %s' % doc.save('EXO-457 %s %s built' % (ARM, MODE)))
    log('document %r v%s isModified %s' % (doc.name, doc.dataFile.versionNumber, doc.isModified))
finally:
    if prev is not None:
        try:
            prev.activate()
        except Exception as e:  # noqa: BLE001
            log('could not re-activate %r: %s' % (prev.name, e))
    log('after: active %r; open %s' % (app.activeDocument.name if app.activeDocument else None,
                                      open_docs()))
