"""EXO-283 shared helpers (imported by dedupe_exo283_*.py bridge scripts)."""
import json, os, time
D = '/Users/esteban/repos/exo/gento_luna/exo/exports/dedupe_exo283/'
ST = D + 'state.json'
PARAM = D + 'param.json'
PROJECT_ID = 'a.YnVzaW5lc3M6Z21haWw2MDk1MjQ0IzIwMjYxMDE1MTE1NTM4MzE1MQ'
P = 'urn:adsk.wipprod:dm.lineage:'
KEPT = {'u2fX': P+'u2fXVhGnTnWSL4pORlPOTw', 'Bmc3': P+'Bmc3JQj_TxiHKtTGBQDIBQ', 'yDln': P+'yDlnR5NwS16jA0o2fXON7A',
        'UmT3': P+'UmT3rfbjTOqW97ZqcdqOgA', 'khSi': P+'khSiwew-RLGCubfLBKzzKw', 'zU49': P+'zU49JHZLRTmJLNjYSZeSRA',
        'eKtc': P+'eKtchCMKQ1as7fljsFaAEw'}
# extra -> kept short id
EXTRA = {P+'_vds__rqTwSDoxaVW9ch6g': 'u2fX', P+'Grel7U5zRIKetApTsuNIAw': 'u2fX',
         P+'3GOuJvUxRySz5kPInpNDog': 'Bmc3', P+'06avThYqTgy4dwOO_7VdOw': 'Bmc3',
         P+'dW5WClmISqWaUGbJ8iBm6g': 'yDln', P+'l1Z5i7nDSdCE7Iq-qgLEeQ': 'yDln',
         P+'i1pB70ZESvaIAZxFwU7wSQ': 'UmT3', P+'FYgk4wLCRVabVtGEiVbFhg': 'UmT3',
         P+'_mtPirY5TZ6HAGO9Oa0ULQ': 'khSi', P+'z82oImUIQxq1BOI-jrPxBg': 'khSi', P+'bYhycXTtRDCF7RLAR_EaWQ': 'khSi',
         P+'VtO-MkfaRleYqpo3jRa1XA': 'zU49', P+'zs1gGokvTayE8cgJrH4veg': 'eKtc'}
BACKUP_ID = P+'xWFsuHDcTy64WIBsxPfAiQ'
BACKUP_COPIES = {P+'b28KoYRTTCOU1lsWTZr0ag': 'u2fX', P+'eMNdbWDpTq20Uclv6skCVA': 'Bmc3',
                 P+'7XBtxEtKQKecKXWSNQMdlQ': 'yDln', P+'cjURVua5QTO85UwocGuaFQ': 'UmT3'}
ASM = {'arm complete': P+'Jy8czxywR4ybJedncZiLhQ', 'arm test': P+'fZK9GF8GRG2BcpTRYFhfXg',
       'arm complete luna': P+'S38u3qgCTH-_S5ut0JUksA', 'exo generated': P+'I6f3yGrzR8S8xg8G0BP0aw',
       'sensor module 2': P+'CzEATRrQSnuX-w_dwKCH6A', 'wrist_sketch3_with_straps': P+'s_bSGtj1SL-TvkFV7MHp3Q',
       'wrist_sketch3_with_straps (copy v1)': P+'jIKLpTb3SCeDiKx5Latkcw', 'arm': P+'cXGOAcs-Q6unIh91MhgbSQ',
       'arm 2': P+'3E7UqMBkQdKxu7gpyS2oRA', 'sensor module pGmH': P+'pGmHk0RESMyeqHdQEMWmdw',
       'sensor module ocXT': P+'ocXTpTytQRyIxEl55toCfw'}

def load():
    return json.load(open(ST)) if os.path.exists(ST) else {}

def save(state):
    with open(ST + '.tmp', 'w') as f:
        json.dump(state, f, indent=1, default=str)
    os.replace(ST + '.tmp', ST)

def param():
    return json.load(open(PARAM))

def get(fn):
    try:
        return fn()
    except Exception as e:
        return 'ERR: %s' % (str(e).splitlines()[0] if str(e) else type(e).__name__)

def retry(fn, tries=5):
    last = None
    for t in range(tries):
        try:
            return fn()
        except Exception as e:
            last = e; time.sleep(0.5)
    raise last

def items(c, tries=5):
    if not hasattr(c, 'count'):
        return list(c)
    return [retry(lambda i=i: c.item(i), tries) for i in range(c.count)]

def short(i):
    return str(i).split(':')[-1][:4]

def find_file(app, fid):
    df = retry(lambda: app.data.findFileById(fid))
    assert df is not None, fid
    assert df.parentProject.id == PROJECT_ID, (fid, df.parentProject.id)
    assert df.parentProject.parentHub.name == 'Estebans' and df.parentProject.name == 'G1 exeskeleton'
    return df

def classify(fid):
    if fid in EXTRA: return 'extra->' + EXTRA[fid]
    if fid in BACKUP_COPIES: return 'backup-copy'
    if fid in KEPT.values(): return 'kept'
    return 'other'

def inspect(adsk, doc):
    """Read-only snapshot of a design document: counts and every linked occurrence."""
    g = get
    des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    root = des.rootComponent
    comps = items(des.allComponents)
    occs = items(root.allOccurrences)
    joints = [('joint', j) for j in items(root.allJoints)] + [('asBuilt', j) for j in items(root.allAsBuiltJoints)]
    rgs = items(root.allRigidGroups)
    def jpaths(j):
        return [g(lambda: j.occurrenceOne.fullPathName), g(lambda: j.occurrenceTwo.fullPathName)]
    jinfo = [(k, g(lambda j=j: j.name), jpaths(j), g(lambda j=j: str(j.healthState)), g(lambda j=j: j.isSuppressed)) for k, j in joints]
    rginfo = [(g(lambda r=r: r.name), [g(lambda o=o: o.fullPathName) for o in items(r.occurrences)]) for r in rgs]
    def under(p, base):
        return isinstance(p, str) and (p == base or p.startswith(base + '+'))
    linked = []
    for o in occs:
        if g(lambda: o.isReferencedComponent) is not True:
            continue
        fp = g(lambda: o.fullPathName)
        # owner: nearest referenced ancestor (nested links belong to that child doc)
        owner = None; a = g(lambda: o.assemblyContext)
        while a is not None and not isinstance(a, str):
            if g(lambda a=a: a.isReferencedComponent) is True:
                owner = g(lambda a=a: a.fullPathName); break
            a = g(lambda a=a: a.assemblyContext)
        dr = g(lambda: o.documentReference)
        fid = g(lambda: dr.dataFile.id)
        js = [(k, n, h, s) for k, n, ps, h, s in jinfo if any(under(p, fp) for p in ps)]
        rg = [n for n, ps in rginfo if any(under(p, fp) for p in ps)]
        linked.append({'occ': fp, 'name': g(lambda: o.name), 'owner_link': owner, 'component': g(lambda: o.component.name), 'file_id': fid,
                       'file': g(lambda: dr.dataFile.name), 'ver': g(lambda: dr.version), 'outOfDate': g(lambda: dr.isOutOfDate),
                       'class': classify(fid), 'joints': js, 'rigidGroups': rg})
    bad = [x for x in jinfo if 'Healthy' not in x[3] and 'ERR' not in x[3] and x[3] not in ('0',)]
    return {'components': len(comps), 'bodies': sum(c.bRepBodies.count for c in comps), 'occurrences': len(occs),
            'linked': len(linked), 'joints': sum(1 for k in jinfo if k[0] == 'joint'),
            'asBuilt': sum(1 for k in jinfo if k[0] == 'asBuilt'), 'rigidGroups': len(rgs),
            'joint_health': sorted(set(x[3] for x in jinfo)), 'linked_occ': linked,
            'doc_refs': [{'id': g(lambda r=r: r.dataFile.id), 'name': g(lambda r=r: r.dataFile.name), 'ver': g(lambda r=r: r.version),
                          'outOfDate': g(lambda r=r: r.isOutOfDate)} for r in items(doc.documentReferences)]}
