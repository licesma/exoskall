"""EXO-278: READ-ONLY. Compare current latestVersionNumber/dateModified of all 44 source docs with the EXO-271 inventory."""
import json, time
inv = json.load(open('/Users/esteban/repos/exo/gento_luna/exo/exports/fusion_data_inventory_exo271.json'))
ch = []
for f in inv['files']:
    try:
        df = app.data.findFileById(f['id'])
        v = df.latestVersionNumber
        if v != f['latestVersionNumber'] or df.name != f['name']:
            ch.append((f['name'], f['latestVersionNumber'], df.name, v, time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(df.dateModified)), df.lastUpdatedBy.displayName))
    except Exception as e:
        ch.append((f['name'], 'ERR', str(e)[:100]))
log('checked %d; changed: %s' % (len(inv['files']), ch))
