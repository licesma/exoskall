try:
    exec(open('/tmp/exo304/p1_build.py').read())
except Exception:
    d = app.activeDocument
    if d is not None and d.dataFile is None and d.name.startswith('Untitled'):
        d.close(False); log('scratch doc closed after failure')
    raise
