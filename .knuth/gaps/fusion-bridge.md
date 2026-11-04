# Gaps — fusion-bridge

- accepted: add-in observed live 2026-09-14 — ping OK, FAIL path with traceback, survival on a second ping. Custom-event plumbing and manifest keys therefore work.
- open: `export_step` and `screenshot` are written from the API docs and never executed —
  `createSTEPExportOptions` + `exportManager.execute`, `activeViewport.saveAsImageFile`
  at 1600x1000. First real call is the check.
- open: the atomic log write (temp file + `os.replace`) is only exercised through the
  stub, and the reader side of it only through the fake bridge in `tools/test_fusion_run.py`.
  No test proves a reader can never see a partial log under real concurrency.
- open: `design` is re-resolved per script and may be `None`; the bridge logs a line and
  runs the script anyway. Whether scripts should instead be refused when no design is
  open is a call nobody has had to make yet.
- open: no cancellation and no watchdog. A script that blocks (modal dialog, long loop)
  hangs the Fusion UI and the bridge until Esteban stops the add-in.
- open: `.knuth/fusion/{inbox,running,outbox,done}` grow without bound; no pruning policy.
- open: `BuildArm.build_arm(...)` was factored out of `run()` but has not been called
  through the bridge — its geometry code is untouched and its known `occ.transform2`
  failure on direct designs is unrelated and still there.
- open: `interactive=False` is what keeps `build_arm` from opening the missing-URDF file
  dialog. Any new dialog added to BuildArm must respect that flag or it will hang the bridge.
