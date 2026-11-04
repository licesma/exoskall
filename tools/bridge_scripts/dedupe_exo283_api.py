"""EXO-283 read-only: probe API for occurrence replace routes."""
O = adsk.fusion.Occurrence; Cm = adsk.fusion.Component; Os = adsk.fusion.Occurrences
for cls in (O, Cm, Os):
    log('%s: %s' % (cls.__name__, [a for a in dir(cls) if any(k in a.lower() for k in ('replace','reference','link','break','datafile','insert','source'))]))
for n in ('replace',):
    if hasattr(O, n): log('Occurrence.replace doc: %s' % (O.replace.__doc__ or '')[:1500])
cds = ui.commandDefinitions
hits = []
for i in range(cds.count):
    try:
        c = cds.item(i)
        if any(k in (c.id + c.name).lower() for k in ('replace', 'relink', 'swap')): hits.append((c.id, c.name))
    except Exception as e: pass
log('cmds: %s' % hits)
log('version %s' % app.version)
