"""EXO-444: open `exo generated` (latest version) and make it the active document, so the rebuild
script can run on it. Opens and activates only: nothing is edited or saved. Every other open
document is left as it is (their isModified is logged before and after).
Run: /usr/bin/python3 tools/fusion_run.py tools/bridge_scripts/open_gen_exo444.py --timeout 300
"""
GEN_ID = 'urn:adsk.wipprod:dm.lineage:I6f3yGrzR8S8xg8G0BP0aw'   # exo generated (EXO-283)
def docs():
    return dict((app.documents.item(i).name, app.documents.item(i).isModified) for i in range(app.documents.count))
log('before: active %r; open %s' % (app.activeDocument.name if app.activeDocument else None, docs()))
gen = None
for i in range(app.documents.count):
    d = app.documents.item(i)
    try:
        if d.dataFile is not None and d.dataFile.id == GEN_ID: gen = d
    except Exception: pass
if gen is None:
    df = app.data.findFileById(GEN_ID)
    log('data file %r v%s latest v%s' % (df.name, df.versionNumber, df.latestVersionNumber))
    gen = app.documents.open(df, True)
gen.activate()
log('after: active %r v%s isModified %s; open %s' % (app.activeDocument.name, gen.dataFile.versionNumber,
                                                     gen.isModified, docs()))
