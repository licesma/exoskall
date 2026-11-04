"""AgentBridge: run agent-authored Python inside the open Fusion session.

Fusion has no headless mode, but this add-in — loaded once, "Run on Startup" —
watches a folder in the repo and executes whatever Python lands there, then writes
the result back as a log file. That closes the agent loop: no click, no screenshot.

Folder protocol (all under <repo>/.knuth/fusion/, resolved from this file's path):

    inbox/<name>.py     dropped by tools/fusion_run.py (moved in atomically)
    running/<name>.py   moved here while it executes
    done/<name>.py      moved here when it finishes
    outbox/<name>.py.log  first line "OK" or "FAIL", then stdout/stderr/log()/traceback
    bridge.log          the add-in's own errors (never the agent script's)

The outbox log is written to a temp name and renamed, so a reader never sees a
partial file; it is written last, so its existence means the run is over.

Threading: a background thread only does os.listdir and calls fireCustomEvent; all
Fusion API work happens on the main thread in the custom event handler. Handler and
thread references are module globals so Fusion does not garbage-collect them.

Globals given to each script: app, ui, design (may be None), adsk, log(msg),
export_step(path), screenshot(path). See .knuth/docs/fusion-bridge.md.
"""

import glob
import io
import os
import sys
import threading
import time
import traceback

import adsk.core
import adsk.fusion

# ============================== CONFIG ==============================

EVENT_ID = 'AgentBridgeInboxEvent'
POLL_SECONDS = 1.0

ADDIN_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.normpath(os.path.join(ADDIN_DIR, '..', '..'))
FUSION_DIR = os.path.join(REPO_DIR, '.knuth', 'fusion')

INBOX = os.path.join(FUSION_DIR, 'inbox')
RUNNING = os.path.join(FUSION_DIR, 'running')
OUTBOX = os.path.join(FUSION_DIR, 'outbox')
DONE = os.path.join(FUSION_DIR, 'done')
BRIDGE_LOG = os.path.join(FUSION_DIR, 'bridge.log')

SCREENSHOT_SIZE = (1600, 1000)

# ====================================================================

# Kept global so Fusion does not garbage-collect the handler / thread.
_app = None
_ui = None
_custom_event = None
_event_handler = None
_watcher = None
_stop_event = None
_busy = False


# ---------------- infrastructure ----------------

def _ensure_dirs():
    for d in (FUSION_DIR, INBOX, RUNNING, OUTBOX, DONE):
        if not os.path.isdir(d):
            os.makedirs(d)


def _bridge_log(msg):
    """The add-in's own diagnostics. Best effort; never raises."""
    try:
        _ensure_dirs()
        stamp = time.strftime('%Y-%m-%d %H:%M:%S')
        with open(BRIDGE_LOG, 'a') as f:
            f.write('%s  %s\n' % (stamp, msg))
    except Exception:
        pass


def _pending_scripts():
    """Inbox *.py, sorted by name. No Fusion API — safe on the watcher thread."""
    try:
        return sorted(glob.glob(os.path.join(INBOX, '*.py')), key=os.path.basename)
    except Exception:
        return []


def _resolve_path(path):
    """Absolute path (relative ones are repo-relative), parent directory created."""
    path = os.path.normpath(path if os.path.isabs(path) else os.path.join(REPO_DIR, path))
    parent = os.path.dirname(path)
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)
    return path


def _write_log(name, status, body):
    """Write outbox/<name>.log atomically: temp file, fsync, rename."""
    final = os.path.join(OUTBOX, name + '.log')
    tmp = os.path.join(OUTBOX, '.' + name + '.log.tmp')
    text = status + '\n' + body
    with open(tmp, 'w') as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, final)
    return final


# ---------------- running one agent script ----------------

def _run_one(src_path):
    """Move, exec, log, move to done. Never raises."""
    name = os.path.basename(src_path)
    running_path = os.path.join(RUNNING, name)
    try:
        os.replace(src_path, running_path)
    except OSError:
        _bridge_log('could not move %s into running/: %s' % (name, traceback.format_exc()))
        return

    buf = io.StringIO()

    def emit(msg):
        buf.write('%s\n' % (msg,))

    def export_step(path):
        """STEP export of the active design's root component. Returns the path."""
        design = adsk.fusion.Design.cast(_app.activeProduct)
        if design is None:
            raise RuntimeError('export_step: no active Fusion design')
        out = _resolve_path(path)
        manager = design.exportManager
        options = manager.createSTEPExportOptions(out, design.rootComponent)
        if not manager.execute(options):
            raise RuntimeError('export_step: exportManager.execute failed for ' + out)
        emit('bridge: exported STEP -> ' + out)
        return out

    def screenshot(path):
        """Save the active viewport as an image. Returns the path."""
        viewport = _app.activeViewport
        if viewport is None:
            raise RuntimeError('screenshot: no active viewport')
        out = _resolve_path(path)
        if not viewport.saveAsImageFile(out, SCREENSHOT_SIZE[0], SCREENSHOT_SIZE[1]):
            raise RuntimeError('screenshot: saveAsImageFile failed for ' + out)
        emit('bridge: saved screenshot -> ' + out)
        return out

    design = adsk.fusion.Design.cast(_app.activeProduct)
    if design is None:
        emit('bridge: no active Fusion design - design is None')

    script_globals = {
        '__name__': '__agent_script__',
        '__file__': running_path,
        '__builtins__': __builtins__,
        'app': _app,
        'ui': _ui,
        'design': design,
        'adsk': adsk,
        'log': emit,
        'export_step': export_step,
        'screenshot': screenshot,
    }

    status = 'OK'
    started = time.time()
    old_stdout, old_stderr = sys.stdout, sys.stderr
    sys.stdout, sys.stderr = buf, buf
    try:
        with open(running_path) as f:
            source = f.read()
        exec(compile(source, running_path, 'exec'), script_globals)
    except SystemExit as exc:                  # sys.exit() is an early return, not a crash
        if exc.code not in (None, 0):
            status = 'FAIL'
            traceback.print_exc(file=buf)
        else:
            buf.write('bridge: script called sys.exit(%r)\n' % (exc.code,))
    except BaseException:                      # nothing an agent script does kills the add-in
        status = 'FAIL'
        traceback.print_exc(file=buf)
    finally:
        sys.stdout, sys.stderr = old_stdout, old_stderr

    emit('bridge: %s in %.1fs' % (status, time.time() - started))

    try:
        os.replace(running_path, os.path.join(DONE, name))
    except OSError:
        emit('bridge: could not move the script into done/\n' + traceback.format_exc())

    try:
        _write_log(name, status, buf.getvalue())
    except Exception:
        _bridge_log('could not write the log for %s: %s' % (name, traceback.format_exc()))


def _process_inbox():
    """Main thread. Drains the inbox; one bad script never stops the rest."""
    global _busy
    if _busy:
        return
    _busy = True
    try:
        for path in _pending_scripts():
            try:
                _run_one(path)
            except Exception:
                _bridge_log('_run_one(%s): %s' % (path, traceback.format_exc()))
    finally:
        _busy = False


# ---------------- event plumbing ----------------

class _InboxEventHandler(adsk.core.CustomEventHandler):
    """Runs on Fusion's main thread, where the API is legal."""

    def notify(self, args):
        try:
            _process_inbox()
        except Exception:
            _bridge_log('notify(): ' + traceback.format_exc())


class _Watcher(threading.Thread):
    """Background poller. Touches no Fusion API except fireCustomEvent."""

    def __init__(self, stop_event):
        threading.Thread.__init__(self, name='AgentBridgeWatcher')
        self.daemon = True
        self._stop = stop_event

    def run(self):
        while not self._stop.wait(POLL_SECONDS):
            try:
                if _pending_scripts() and _app is not None:
                    _app.fireCustomEvent(EVENT_ID, '')
            except Exception:
                _bridge_log('watcher: ' + traceback.format_exc())


# ---------------- add-in entry points ----------------

def run(context):
    global _app, _ui, _custom_event, _event_handler, _watcher, _stop_event
    try:
        _app = adsk.core.Application.get()
        _ui = _app.userInterface
        _ensure_dirs()

        try:                                    # clear a stale registration from a reload
            _app.unregisterCustomEvent(EVENT_ID)
        except Exception:
            pass
        _custom_event = _app.registerCustomEvent(EVENT_ID)
        _event_handler = _InboxEventHandler()
        _custom_event.add(_event_handler)

        _stop_event = threading.Event()
        _watcher = _Watcher(_stop_event)
        _watcher.start()
        _bridge_log('started; polling %s every %.1fs' % (INBOX, POLL_SECONDS))
    except Exception:
        _bridge_log('run(): ' + traceback.format_exc())
        if _ui:
            _ui.messageBox('AgentBridge failed to start:\n' + traceback.format_exc())


def stop(context):
    global _app, _ui, _custom_event, _event_handler, _watcher, _stop_event
    try:
        if _stop_event is not None:
            _stop_event.set()
        if _watcher is not None:
            _watcher.join(POLL_SECONDS * 3)
        if _custom_event is not None and _event_handler is not None:
            _custom_event.remove(_event_handler)
        if _app is not None:
            _app.unregisterCustomEvent(EVENT_ID)
        _bridge_log('stopped')
    except Exception:
        _bridge_log('stop(): ' + traceback.format_exc())
    finally:
        _watcher = None
        _stop_event = None
        _event_handler = None
        _custom_event = None
        _ui = None
        _app = None
