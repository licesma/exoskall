"""EXO-278 (3): READ-ONLY counts for queued assemblies (state['exo278']['count_queue']); <=120 s per run.
Opens each doc (source or target) read-only, counts components/bodies/occurrences/linked occurrences and where the
links resolve (hub/project), closes only docs this script opened, never saves."""
import json, os, time, collections
ST = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274/state.json'
state = json.load(open(ST))
X = state['exo278']; T0 = time.time()
def get(fn):
    try:
        return fn()
    except Exception as e:
        return 'ERR: %s' % (str(e).splitlines()[0] if str(e) else type(e).__name__)
def save():
    json.dump(state, open(ST + '.tmp', 'w'), indent=1, default=str); os.replace(ST + '.tmp', ST)
for label, fid in X['count_queue']:
    if label in X['counts']:
        continue
    if time.time() - T0 > 120:
        break
    t0 = time.time(); r = {}
    pre = [app.documents.item(i) for i in range(app.documents.count) if get(lambda i=i: app.documents.item(i).dataFile.id) == fid]
    before = app.documents.count
    try:
        df = app.data.findFileById(fid)
        r['df'] = '%s v%s @ %s/%s' % (df.name, df.versionNumber, get(lambda: df.parentProject.parentHub.name), get(lambda: df.parentProject.name))
        doc = pre[0] if pre else app.documents.open(df, True)
        r['reused_open'] = bool(pre)
        des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        root = des.rootComponent
        comps = [des.allComponents.item(i) for i in range(des.allComponents.count)]
        occs = [root.allOccurrences.item(i) for i in range(root.allOccurrences.count)]
        r['components'] = len(comps); r['bodies_total'] = sum(c.bRepBodies.count for c in comps); r['occurrences'] = len(occs)
        linked = [o for o in occs if get(lambda o=o: o.isReferencedComponent) is True]
        r['linked_occ'] = len(linked)
        where = collections.Counter(); ood = 0; unread = 0
        for o in linked:
            dr = get(lambda o=o: o.documentReference)
            h = get(lambda: '%s/%s' % (dr.dataFile.parentProject.parentHub.name, dr.dataFile.parentProject.name))
            if str(h).startswith('ERR'):
                unread += 1
            else:
                where[h] += 1
            if get(lambda: dr.isOutOfDate) is True:
                ood += 1
        r['linked_resolve_to'] = dict(where); r['linked_unreadable'] = unread; r['linked_outOfDate'] = ood
        r['doc_refs'] = [{'name': get(lambda i=i: doc.documentReferences.item(i).dataFile.name),
                          'id': get(lambda i=i: doc.documentReferences.item(i).dataFile.id.split(':')[-1]),
                          'ver': get(lambda i=i: doc.documentReferences.item(i).version),
                          'where': get(lambda i=i: doc.documentReferences.item(i).dataFile.parentProject.parentHub.name)}
                         for i in range(doc.documentReferences.count)]
        r['modified'] = doc.isModified
    except Exception as e:
        r['error'] = str(e)[:300]
    finally:
        closed = []
        if not pre:
            for d in reversed([app.documents.item(i) for i in range(app.documents.count)][before:]):
                try:
                    n = d.name; d.close(False); closed.append(n)
                except Exception as e:
                    log('close err %s' % e)
        r['closed'] = closed; r['s'] = round(time.time() - t0, 1)
    X['counts'][label] = r; save()
    log('%s: %s' % (label, json.dumps({k: r.get(k) for k in ('df', 'components', 'bodies_total', 'occurrences', 'linked_occ', 'linked_resolve_to', 'linked_unreadable', 'linked_outOfDate', 'closed', 'reused_open', 's', 'error')}, default=str)))
left = [l for l, _ in X['count_queue'] if l not in X['counts']]
log('left: %d; open docs now %d' % (len(left), app.documents.count))
