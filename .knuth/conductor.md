# Conductor addendum — exo

## Verification commands

No test suite; this is a CAD/hardware project. An agent proves a change by:

- **Geometry claims** → a Python check that parses the URDF (`xml.etree`) and/or a
  STEP export and prints the measured numbers next to the target. Run it, paste the output.
- **Fusion scripts** (`fusion_scripts/*/`) → run them through the bridge:
  `/usr/bin/python3 tools/fusion_run.py <script.py> [--timeout 120]` executes the script
  inside Esteban's open Fusion session (AgentBridge add-in) and prints its log; exit 0 =
  `OK`, 1 = `FAIL` + traceback, 2 = timeout. Health check first:
  `tools/fusion_run.py --ping` prints the active document name. Scripts get `app`, `ui`,
  `design`, `adsk`, `log`, `export_step`, `screenshot` — and must never open a dialog or
  a message box. See `.knuth/docs/fusion-bridge.md`.
  Fallback, only when Fusion is closed or the ping fails: agent writes the script,
  Esteban runs it by hand (Scripts and Add-Ins → Run) and drops a STEP into
  `gento_luna/exo/exports/` or a screenshot into `gento_luna/exo/screenshots/`.
  Either way, a geometry claim is proven by measuring the export, not by the log.
- Tier 1 = docs/notes/copy. Tier 2 = scripts and displacement computations. Tier 3 =
  anything that executes code inside the running Fusion session (the bridge add-in and
  `tools/fusion_run.py` themselves); no credentials, no spawning.

## Batch check

None defined. No VPT.

## Environment

- Fusion 360 on Esteban's Mac, driven by hand. `.f3d`/cloud designs are the editable
  source; STEP files are measure-only.
- Python 3 (`/usr/bin/python3`) for URDF/STEP parsing. No CAD kernel installed — treat
  a request for OCC/cadquery as a judgment call to report, not install silently.
- Original G1 exo STEP: `gento_luna/exo/exports/exo_skeleton_2026-08-08.step` (112 MB).

## Repo facts

- **Not a git repository.** Never commit; never assume a clean tree. Don't move or
  rename the reference assets.
- `gento_luna/` = vendor drop (PDFs, DriveMaster tool, URDF + meshes) + `exo/` (our notes,
  exports, screenshots). `fusion_scripts/` = Fusion add-in scripts.
- Exo joint doc: `.knuth/docs/exo-joint.md`. Change list: `gento_luna/exo/NOTES.md`.
- Paths contain spaces (`Gento Luna_URDF 20260428`) — quote everything.
