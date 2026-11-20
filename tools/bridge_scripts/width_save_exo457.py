"""EXO-457: save one of this issue's OWN width-check documents (name set by the wrapper variable
DOC, must start with 'width52 check' or 'width manual check'), if it is open. Never saves any
other document. Re-activates whatever was active before.
Run: write a wrapper that sets DOC, or edit DOC below for a one-off.
"""
DOC = globals().get('DOC')
if not DOC or not (DOC.startswith('width52 check') or DOC.startswith('width manual check')):
    raise RuntimeError('DOC %r is not an EXO-457 document' % DOC)
prev = app.activeDocument
target = [app.documents.item(i) for i in range(app.documents.count) if app.documents.item(i).name == DOC]
if len(target) != 1:
    raise RuntimeError('%d open documents named %r' % (len(target), DOC))
d = target[0]
d.activate()
if app.activeDocument.name != DOC:
    raise RuntimeError('could not activate %r' % DOC)
log('save %r: %s; v%s isModified %s' % (DOC, d.save('EXO-457 built'), d.dataFile.versionNumber, d.isModified))
if prev is not None and prev.name != DOC:
    prev.activate()
log('active now %r' % (app.activeDocument.name if app.activeDocument else None))
