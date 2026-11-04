"""EXO-278 (2): READ-ONLY. Open the uploaded target 'arm complete' and the source 'arm complete' v35 + Aug-8 v1,
count components/bodies/occurrences and external references (resolved or not). Close only docs opened here; never save."""
import json, os, time
BASE = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274'
ST = BASE + '/state.json'
state = json.load(open(ST))
def get(fn):
    try:
        return fn()
    except Exception as e:
        return 'ERR: %s' % (str(e).splitlines()[0] if str(e) else type(e).__name__)
IDS = [('target_upload', state['exo278']['df_id']),
       ('source_v35', 'urn:adsk.wipprod:dm.lineage:EXtWGgfjQ1GnGLG53zkBsQ'),
       ('source_aug8_v1', 'urn:adsk.wipprod:dm.lineage:eip2VaCiSPOzOXokAfwGWw')]
out = {}
for label, fid in IDS:
    t0 = time.time()
    before = app.documents.count
    pre = [app.documents.item(i) for i in range(app.documents.count) if get(lambda i=i: app.documents.item(i).dataFile.id) == fid]
    opened = []
    r = {}
    try:
        df = app.data.findFileById(fid)
        r['df'] = {'name': df.name, 'ver': df.versionNumber, 'project': get(lambda: df.parentProject.name), 'hub': get(lambda: df.parentProject.parentHub.name)}
        if pre:
            doc = pre[0]; r['reused_open'] = True
        else:
            names_before = set(get(lambda i=i: app.documents.item(i).name) + '|' + str(i) for i in range(app.documents.count))
            doc = app.documents.open(df, True)
            opened = [doc]
        des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        root = des.rootComponent
        comps = [des.allComponents.item(i) for i in range(des.allComponents.count)]
        occs = [root.allOccurrences.item(i) for i in range(root.allOccurrences.count)]
        r['root_name'] = root.name
        r['components'] = len(comps)
        r['bodies_total'] = sum(c.bRepBodies.count for c in comps)
        r['root_bodies'] = [b.name for b in root.bRepBodies][:20]
        r['occurrences'] = len(occs)
        r['top_occurrences'] = [o.name for o in root.occurrences][:40]
        refs = []
        for o in occs:
            if get(lambda o=o: o.isReferencedComponent) is True:
                dr = get(lambda o=o: o.documentReference)
                refs.append({'occ': o.name, 'ref_df': get(lambda: dr.dataFile.name), 'ref_ver': get(lambda: dr.version),
                             'ref_project': get(lambda: dr.dataFile.parentProject.name), 'outOfDate': get(lambda: dr.isOutOfDate),
                             'bodies': get(lambda o=o: o.component.bRepBodies.count)})
        r['referenced_occurrences'] = refs
        r['doc_refs'] = [{'name': get(lambda i=i: doc.documentReferences.item(i).dataFile.name),
                          'ver': get(lambda i=i: doc.documentReferences.item(i).version)} for i in range(get(lambda: doc.documentReferences.count) or 0)] if hasattr(doc, 'documentReferences') else 'n/a'
        r['modified_after_open'] = doc.isModified
    except Exception as e:
        r['error'] = get(lambda: (_ for _ in ()).throw(e))
    finally:
        # close only docs this script opened (main + any new ones), never save
        closed = []
        for d in reversed([app.documents.item(i) for i in range(app.documents.count)][before:] if not pre else []):
            try:
                n = d.name; d.close(False); closed.append(n)
            except Exception as e:
                log('close err %s' % e)
        r['closed'] = closed
        r['s'] = round(time.time() - t0, 1)
    out[label] = r
    log('%s: %s' % (label, json.dumps({k: v for k, v in r.items() if k not in ('top_occurrences',)}, default=str)[:1500]))
    log('  top occurrences: %s' % r.get('top_occurrences'))
state['exo278']['compare'] = out
json.dump(state, open(ST + '.tmp', 'w'), indent=1, default=str); os.replace(ST + '.tmp', ST)
log('open docs now: %d' % app.documents.count)
