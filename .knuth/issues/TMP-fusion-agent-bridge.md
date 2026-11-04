---
id: TMP-fusion-agent-bridge
title: Fusion agent bridge add-in
description: Fusion add-in that runs agent scripts from a watched folder and writes logs back, no human click.
status: done
mode: subagent
parent: TMP-exoskeleton-from-urdf
---
## Context

Every generator iteration currently costs Esteban a click in Fusion plus a screenshot,
and the agent is blind to the outcome. That loop is the bottleneck for the whole parent.
Fusion has no headless mode, but an add-in running inside the open Fusion session can
execute arbitrary Python via the API.

## Task

`fusion_scripts/AgentBridge/` — a Fusion **add-in** (not script), "Run on Startup":

- Polls `.knuth/fusion/inbox/` (repo-relative, resolved from the add-in's path) every
  ~1 s via a custom event / timer thread — never block the UI thread.
- For each `*.py` found: move it to `.knuth/fusion/running/`, `exec` it with
  `app`, `ui`, `design` pre-bound and a `log(msg)` function, capture stdout/stderr and
  any traceback, then write `.knuth/fusion/outbox/<name>.log` (status line first:
  `OK` / `FAIL`), and move the script to `.knuth/fusion/done/`.
- Scripts can request an export by calling a provided `export_step(path)` /
  `screenshot(path)` helper (`app.activeViewport.saveAsImageFile`).
- Errors inside the agent script never kill the add-in; the add-in's own errors go to
  `.knuth/fusion/bridge.log`.
- A `tools/fusion_run.py <script.py> [--timeout 120]` CLI drops the script, waits for
  the log, prints it, exits non-zero on `FAIL` or timeout. This is what agents call.

## Done when

- Esteban installs the add-in once and enables Run on Startup.
- `tools/fusion_run.py` with a 3-line script (`log(design.rootComponent.name)`) returns
  the name; with a script that raises, returns `FAIL` + traceback and the add-in survives.
- `.knuth/conductor.md` verification section updated: agents verify Fusion scripts by
  running them through the bridge; the manual click path is the fallback only.

## Judgment calls to report, not ask

Polling mechanism (thread + custom event vs. `adsk.core.Application.fireCustomEvent`);
how `design` is resolved when no design is active; whether `BuildArm` is refactored so
its `run()` body is callable through the bridge.

## Hand-off — install the add-in (Esteban, once)

1. In Fusion, **Utilities → Scripts and Add-Ins** (or `Shift+S`).
2. **Add-Ins** tab → the **+** next to "My Add-Ins" → **Script or add-in from device**.
3. Pick the folder `fusion_scripts/AgentBridge` (the folder itself, not a file inside it).
4. Select **AgentBridge** in the list, tick **Run on Startup**, click **Run**.

It now starts with Fusion. Check from a terminal:

    /usr/bin/python3 tools/fusion_run.py --ping

Expect `OK` and `active document: <your document name>`. If it times out saying
"still in inbox/", the add-in is not running — Stop/Run it in that same dialog and look
at `.knuth/fusion/bridge.log`.

After editing `AgentBridge.py`, Stop then Run it again for the change to take effect.

## Close-out (2026-09-14, conductor)

Installed by Esteban, verified live from the CLI:
- `fusion_run.py --ping` → `OK`, "active document: arm complete" (0.0 s)
- script raising `ValueError` → `FAIL` + traceback, exit 1
- second `--ping` after the failure → `OK` — add-in survives
`export_step` / `screenshot` still unobserved (gap).
