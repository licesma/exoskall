"""EXO-208: find Fusion's built-in 'show all components' command id."""
ui_ = app.userInterface
hits = []
for i in range(ui_.commandDefinitions.count):
    c = ui_.commandDefinitions.item(i)
    cid = c.id
    low = cid.lower()
    if 'showall' in low or ('visib' in low and 'all' in low) or 'unhide' in low:
        hits.append('%s | %s' % (cid, c.name))
for h in hits:
    log(h)
log('%d candidate(s)' % len(hits))
