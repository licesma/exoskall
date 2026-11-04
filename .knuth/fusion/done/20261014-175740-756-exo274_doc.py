"""EXO-274: READ-ONLY API doc lookup."""
for n in ('createFusionArchiveExportOptions',):
    log((getattr(adsk.fusion.ExportManager, n).__doc__ or '')[:1500])
log([a for a in dir(adsk.fusion.FusionArchiveExportOptions) if not a.startswith('_')])
log((adsk.core.DataFile.copy.__doc__ or '')[:1200])
log((adsk.core.DataFolder.uploadAssembly.__doc__ or '')[:1200])
log((adsk.core.DataFolder.uploadFile.__doc__ or '')[:1200])
log([a for a in dir(adsk.core.DataUpload) if not a.startswith('_')])
