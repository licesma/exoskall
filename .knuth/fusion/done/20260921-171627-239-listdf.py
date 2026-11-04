dfs = app.data.activeProject.rootFolder.dataFiles
for i in range(dfs.count):
    d = dfs.item(i)
    if d.name in ('arm complete', 'arm complete luna', 'arm', 'right'):
        try:
            log('%2d %-20s v%-4s modified %s  id=%s' % (i, d.name, d.versionNumber,
                d.dateModified, d.id))
        except Exception as e:
            log('%2d %-20s (%s)' % (i, d.name, e))
