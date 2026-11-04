"""EXO-282: READ-ONLY. Which hub/project/lineage each open doc belongs to."""
g = lambda fn: (lambda: fn())
def get(fn):
    try: return fn()
    except Exception as e: return 'ERR: %s' % e
for i in range(app.documents.count):
    d = app.documents.item(i)
    log('%s | %s | %s | %s | v%s | modified=%s' % (get(lambda: d.name), get(lambda: d.dataFile.parentProject.parentHub.name),
        get(lambda: d.dataFile.parentProject.name), get(lambda: d.dataFile.id), get(lambda: d.dataFile.versionNumber), get(lambda: d.isModified)))
