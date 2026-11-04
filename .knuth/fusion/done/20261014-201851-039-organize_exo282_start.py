"""EXO-282: READ-ONLY. Record active hub/project/doc, probe DataFolder.move."""
import sys, importlib, time, json
sys.path.insert(0, '/Users/esteban/repos/exo/tools/bridge_scripts')
import organize_exo282_common as C; importlib.reload(C)
s = C.load(); g = C.get
cur = {'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'active_hub': g(lambda: app.data.activeHub.name),
       'active_hub_id': g(lambda: app.data.activeHub.id), 'active_project': g(lambda: app.data.activeProject.name),
       'active_project_id': g(lambda: app.data.activeProject.id), 'active_doc': g(lambda: app.activeDocument.name),
       'open_docs': [g(lambda i=i: app.documents.item(i).name) for i in range(app.documents.count)]}
s.setdefault('start', cur)
tp = C.target(app); root = tp.rootFolder
s['project_id'] = tp.id
s['root_folders'] = [(f.name, f.id) for f in C.items(root.dataFolders)]
s['api'] = {'DataFolder.move': hasattr(adsk.core.DataFolder, 'move'), 'DataFile.move': hasattr(adsk.core.DataFile, 'move'),
            'DataFolders.add': hasattr(adsk.core.DataFolders, 'add'), 'DataFolder.deleteMe': hasattr(adsk.core.DataFolder, 'deleteMe'),
            'DataFile.deleteMe': hasattr(adsk.core.DataFile, 'deleteMe')}
s['root_file_count_start'] = root.dataFiles.count
C.save(s)
log(json.dumps({k: s[k] for k in ('start', 'root_folders', 'api', 'root_file_count_start')}, default=str))
