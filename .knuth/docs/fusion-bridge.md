# Fusion bridge

Fusion has no headless mode, but an add-in running inside an open Fusion session can
execute arbitrary Python through the API. `fusion_scripts/AgentBridge/` is that add-in:
it watches a folder in the repo, runs whatever Python lands there, and writes the result
back as a log file. `tools/fusion_run.py` is the other end — an agent drops a script and
reads the log, no human click and no screenshot in the loop.

## Install (once, by hand)

Fusion → **Utilities → Scripts and Add-Ins → Add-Ins** tab → **+** →
**Script or add-in from device** → pick the folder `fusion_scripts/AgentBridge` →
tick **Run on Startup** → **Run**.

It then starts with Fusion. `.knuth/fusion/bridge.log` gets a `started; polling …` line
each time; that file is the first place to look if nothing happens.

## Use

    /usr/bin/python3 tools/fusion_run.py --ping                      # health check
    /usr/bin/python3 tools/fusion_run.py path/to/script.py --timeout 120

Exit codes: **0** = the script finished (`OK`), **1** = it raised (`FAIL`, traceback in
the log) or the log was malformed, **2** = timeout. The log is printed to stdout, status
line first. `--ping` runs a built-in one-liner that logs the active document name — it
proves Fusion is open, the add-in is alive and the round trip works.

## Exec globals

The script is `exec`'d with these names already bound — no imports needed:

| Name | What |
|---|---|
| `app` | `adsk.core.Application.get()` |
| `ui` | `app.userInterface` |
| `design` | `adsk.fusion.Design.cast(app.activeProduct)`, re-resolved per script. **May be `None`** when no design is open (the bridge notes that in the log and runs the script anyway — the script decides whether it cares). |
| `adsk` | the `adsk` package, with `adsk.core` and `adsk.fusion` imported |
| `log(msg)` | append a line to this script's log |
| `export_step(path)` | STEP export of the root component via `design.exportManager`; returns the path |
| `screenshot(path)` | `app.activeViewport.saveAsImageFile(path, 1600, 1000)`; returns the path |

`print()` works too: stdout and stderr are captured into the same log, in order.
Relative paths given to `export_step` / `screenshot` are repo-relative; missing parent
directories are created.

**Imports must be reloaded.** Fusion caches modules between runs, so a script that pulls
in repo code has to force a reload or it will silently run the previous version:

    import importlib, sys
    sys.path.insert(0, '/Users/esteban/repos/exo/fusion_scripts/BuildArm')
    import BuildArm
    importlib.reload(BuildArm)
    log(BuildArm.build_arm(app, ui, design, log))

(`BuildArm.build_arm(app, ui, design, log=None, interactive=False)` is the old `run()`
body, factored out so it can be driven from here; `Utilities → Scripts and Add-Ins →
BuildArm → Run` still works and is unchanged in behaviour.)

## Folder protocol

Everything lives under `.knuth/fusion/`, resolved from the add-in's own location
(`<repo>/fusion_scripts/AgentBridge/` → `<repo>/.knuth/fusion/`). Both sides create the
directories on start.

    inbox/<stamp>-<name>.py      fusion_run.py drops it here (temp file + rename, so the
                                 add-in never sees a half-written script)
    running/<stamp>-<name>.py    while it executes
    done/<stamp>-<name>.py       after it executes - the exact source that ran
    outbox/<stamp>-<name>.py.log first line "OK" or "FAIL", then the log
    bridge.log                   the add-in's own errors, never the script's

The log is written to a temp name and renamed, so a reader never sees a partial file,
and it is written *last*, so its existence means the run is over. The name is the full
script filename plus `.log`. Nothing prunes these folders; they are scratch, not tracked
(the repo is not a git repo — just leave them out of any archive).

## How it works

A background `threading.Thread` polls the inbox every second — it only does `os.listdir`,
never a Fusion API call — and when it sees work it calls
`app.fireCustomEvent('AgentBridgeInboxEvent')`. A `CustomEventHandler` registered with
`app.registerCustomEvent` then runs on Fusion's main thread, where the API is legal, and
drains the inbox in filename order (so the timestamp prefix is also the run order).
Handler and thread are module globals so Fusion does not garbage-collect them.
`stop(context)` sets the thread's stop event, joins it, removes the handler and
unregisters the event.

Every failure mode of an agent script — exception, syntax error, `sys.exit(1)` — becomes
a `FAIL` log with the traceback; nothing propagates out of the handler, so the add-in
keeps serving the next script. The add-in's own failures go to `bridge.log`.

## Limitations

- **Never call `ui.messageBox` or any modal dialog from a bridge script.** It runs on the
  UI thread: the box would sit there until someone clicks it, blocking Fusion *and* the
  bridge. `fusion_run.py`'s timeout message says "still in running/" when this happens.
- Same reason: a long script freezes the Fusion UI while it runs. Seconds are fine,
  minutes are not.
- One script at a time, in filename order. No cancellation — a hung script needs Fusion's
  add-in Stop/Run, or a restart.
- The bridge only reports what the script logs. Geometry claims still need an
  `export_step` plus a measurement pass over the STEP, or a `screenshot` for Esteban.
- If Fusion (or the add-in) dies mid-run the script stays in `running/` with no log and
  the caller times out with exit 2. Nothing is retried automatically.
- The add-in is installed per-machine, outside the repo's control. Editing
  `AgentBridge.py` needs a Stop/Run in Scripts and Add-Ins to take effect.
- No sandbox at all: anything dropped in `inbox/` runs with Fusion's full privileges.
  Only `tools/fusion_run.py` should write there.
