"""EXO-278b: READ-ONLY fingerprint of the 2 source wrist docs (already open: reused) and the 2 target uploads A/B.
Opens target docs read-only if needed; closes only what it opened with close(False). Never saves."""
import json, os, time
ST = '/Users/esteban/repos/exo/gento_luna/exo/exports/migration_exo274/state.json'
state = json.load(open(ST)); r = state.setdefault('exo278b', {})
def get(fn):
    try: return fn()
    except Exception as e: return 'ERR: %s' % (str(e).splitlines()[0] if str(e) else type(e).__name__)
B_CHILD = 'urn:adsk.wipprod:dm.lineage:eKtchCMKQ1as7fljsFaAEw'
ch = app.data.findFileById(B_CHILD)
pars = [(ch.parentReferences.item(i).name, ch.parentReferences.item(i).id) for i in range(ch.parentReferences.count)]
log('B child parents: %s' % pars)
b_id = [p[1] for p in pars if p[0].startswith('wrist_sketch3_with_straps')][0]
DOCS = [('S_v1_MQGUPD', 'urn:adsk.wipprod:dm.lineage:MQGUPDCWQMeCHnTDtaIOjA'),
        ('S_v4now5_FrHFTv', 'urn:adsk.wipprod:dm.lineage:FrHFTvTiRtmuwsPlVZ90ew'),
        ('T_A_jIKLpT', 'urn:adsk.wipprod:dm.lineage:jIKLpTb3SCeDiKx5Latkcw'),
        ('T_B', b_id)]
out = {}
for label, fid in DOCS:
    rr = {'id': fid}; opened = None
    try:
        df = app.data.findFileById(fid)
        rr['df'] = '%s v%s @ %s' % (df.name, df.versionNumber, df.parentProject.parentHub.name)
        rr['latest'] = df.latestVersionNumber
        pre = [app.documents.item(i) for i in range(app.documents.count) if get(lambda i=i: app.documents.item(i).dataFile.id) == fid]
        if pre:
            doc = pre[0]; rr['reused_open'] = True
            rr['open_ver'] = get(lambda: doc.dataFile.versionNumber); rr['open_modified'] = doc.isModified
        else:
            doc = app.documents.open(df, False); opened = doc
        des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        root = des.rootComponent
        comps = [des.allComponents.item(i) for i in range(des.allComponents.count)]
        rr['comps'] = sorted(c.name for c in comps)
        rr['bodies'] = sorted('%s/%s v=%.3f a=%.3f' % (c.name, b.name, b.volume, b.area) for c in comps for b in c.bRepBodies)
        rr['sketches'] = sorted('%s/%s n=%d' % (c.name, s.name, s.sketchCurves.count) for c in comps for s in c.sketches)
        rr['features'] = sum(c.features.count for c in comps)
        rr['timeline'] = get(lambda: des.timeline.count)
        rr['params'] = sorted('%s=%s' % (p.name, p.expression) for p in des.userParameters)
        rr['n_model_params'] = get(lambda: des.allParameters.count)
        rr['occ'] = [(o.name, get(lambda o=o: o.isReferencedComponent), get(lambda o=o: o.documentReference.dataFile.name),
                      get(lambda o=o: o.documentReference.version), get(lambda o=o: o.documentReference.dataFile.parentProject.parentHub.name))
                     for o in root.allOccurrences]
        rr['desc'] = get(lambda: df.description)
    except Exception as e:
        rr['error'] = get(lambda: (_ for _ in ()).throw(e))
    finally:
        if opened is not None:
            try: opened.close(False); rr['closed'] = True
            except Exception as e: rr['close_err'] = str(e)
    out[label] = rr
    log('%s: %s' % (label, json.dumps(rr, default=str)))
r['wrist_compare'] = out; r['wrist_B_id'] = b_id
json.dump(state, open(ST + '.tmp', 'w'), indent=1, default=str); os.replace(ST + '.tmp', ST)
log('open docs now: %s' % [app.documents.item(i).name for i in range(app.documents.count)])
