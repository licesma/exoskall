"""EXO-281 reviewer: READ-ONLY spot check. Opens target Estebans/G1 exeskeleton/'arm complete' (exact name),
counts components / bodies / occurrences / linked occurrences and where links resolve, then close(False)
only if this script opened it. Never saves. Merges into review_exo281.json."""
import json, os, time, collections
OUT = '/Users/esteban/repos/exo/gento_luna/exo/exports/review_exo281.json'
R = json.load(open(OUT))
T0 = time.time()
def get(fn):
    try:
        return fn()
    except Exception as e:
        return 'ERR: %s' % (str(e).splitlines()[0] if str(e) else type(e).__name__)
def flush():
    with open(OUT + '.tmp', 'w') as f:
        json.dump(R, f, indent=1, default=str)
    os.replace(OUT + '.tmp', OUT)
hubs = {get(lambda h=h: h.name): h for h in [app.data.dataHubs.item(i) for i in range(app.data.dataHubs.count)]}
h = hubs['Estebans']
tp = [p for p in [h.dataProjects.item(i) for i in range(h.dataProjects.count)] if get(lambda p=p: p.name) == 'G1 exeskeleton'][0]
root = tp.rootFolder
cands = []
for i in range(root.dataFiles.count):
    df = get(lambda: root.dataFiles.item(i))
    if not isinstance(df, str) and get(lambda: df.name) == 'arm complete':
        cands.append(df)
r = {'candidates': [(get(lambda d=d: d.name), get(lambda d=d: d.id)) for d in cands], 'time': time.strftime('%Y-%m-%d %H:%M:%S')}
log('candidates: %s' % r['candidates'])
assert len(cands) == 1, r['candidates']
df = cands[0]
fid = df.id
r['df'] = '%s v%s @ %s/%s id=%s' % (df.name, df.versionNumber, get(lambda: df.parentProject.parentHub.name), get(lambda: df.parentProject.name), fid)
pre = [app.documents.item(i) for i in range(app.documents.count) if get(lambda i=i: app.documents.item(i).dataFile.id) == fid]
before = app.documents.count
r['open_docs_before'] = [get(lambda i=i: app.documents.item(i).name) for i in range(app.documents.count)]
try:
    doc = pre[0] if pre else app.documents.open(df, True)
    r['reused_open'] = bool(pre)
    r['doc_name'] = doc.name
    r['doc_dataFile_id'] = get(lambda: doc.dataFile.id)
    des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    rootc = des.rootComponent
    comps = [des.allComponents.item(i) for i in range(des.allComponents.count)]
    occs = [rootc.allOccurrences.item(i) for i in range(rootc.allOccurrences.count)]
    r['components'] = len(comps)
    r['bodies_total'] = sum(c.bRepBodies.count for c in comps)
    r['occurrences'] = len(occs)
    linked = [o for o in occs if get(lambda o=o: o.isReferencedComponent) is True]
    r['linked_occ'] = len(linked)
    where = collections.Counter(); unread = 0; ood = 0; names = collections.Counter()
    for o in linked:
        dr = get(lambda o=o: o.documentReference)
        w = get(lambda: '%s/%s' % (dr.dataFile.parentProject.parentHub.name, dr.dataFile.parentProject.name))
        if str(w).startswith('ERR'):
            unread += 1
        else:
            where[w] += 1
        names[get(lambda o=o: o.component.name)] += 1
        if get(lambda: dr.isOutOfDate) is True:
            ood += 1
    r['linked_resolve_to'] = dict(where); r['linked_unreadable'] = unread; r['linked_outOfDate'] = ood
    r['linked_component_names'] = dict(names)
    r['doc_refs'] = [{'name': get(lambda i=i: doc.documentReferences.item(i).dataFile.name),
                      'id': get(lambda i=i: doc.documentReferences.item(i).dataFile.id),
                      'ver': get(lambda i=i: doc.documentReferences.item(i).version),
                      'hub': get(lambda i=i: doc.documentReferences.item(i).dataFile.parentProject.parentHub.name),
                      'project': get(lambda i=i: doc.documentReferences.item(i).dataFile.parentProject.name)}
                     for i in range(doc.documentReferences.count)]
    r['isModified_before_close'] = get(lambda: doc.isModified)
except Exception as e:
    r['error'] = str(e)[:400]
finally:
    closed = []
    if not pre:
        for d in reversed([app.documents.item(i) for i in range(app.documents.count)][before:]):
            try:
                n = d.name; d.close(False); closed.append(n)
            except Exception as e:
                log('close err %s' % e)
    r['closed'] = closed
    r['open_docs_after'] = [get(lambda i=i: app.documents.item(i).name) for i in range(app.documents.count)]
    r['active_hub_after'] = get(lambda: app.data.activeHub.name)
    r['active_project_after'] = get(lambda: app.data.activeProject.name)
    r['s'] = round(time.time() - T0, 1)
R['arm_complete_spot_check'] = r
flush()
log(json.dumps(r, default=str))
