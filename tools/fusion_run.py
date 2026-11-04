#!/usr/bin/env python3
"""Run a Python script inside the open Fusion session, through the AgentBridge add-in.

    /usr/bin/python3 tools/fusion_run.py path/to/script.py [--timeout 120]
    /usr/bin/python3 tools/fusion_run.py --ping            # health check

Copies the script into <repo>/.knuth/fusion/inbox/ under a unique name, waits for the
matching log in outbox/, prints it, and exits 0 on OK, 1 on FAIL (or a malformed log),
2 on timeout. The script runs with app / ui / design / adsk / log / export_step /
screenshot pre-bound; see .knuth/docs/fusion-bridge.md.

Fusion must be open with the AgentBridge add-in running. A timeout almost always means
it is not; the message says where the submitted script ended up, which tells you which.
"""

import argparse
import datetime
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.normpath(os.path.join(HERE, '..'))
DEFAULT_FUSION_DIR = os.path.join(REPO_DIR, '.knuth', 'fusion')
FUSION_DIR_ENV = 'EXO_FUSION_DIR'        # override, used by tools/test_fusion_run.py

SUBDIRS = ('inbox', 'running', 'outbox', 'done')

PING_SOURCE = """\
doc = app.activeDocument
log('active document: %s' % (doc.name if doc else '<none>'))
"""


def fusion_dir():
    return os.environ.get(FUSION_DIR_ENV) or DEFAULT_FUSION_DIR


def ensure_dirs(base):
    for sub in SUBDIRS:
        os.makedirs(os.path.join(base, sub), exist_ok=True)
    return base


def unique_name(basename, now=None, seq=0):
    """Timestamp + basename. `seq` breaks a tie inside the same millisecond."""
    stamp = (now or datetime.datetime.now()).strftime('%Y%m%d-%H%M%S-%f')[:-3]
    if seq:
        stamp = '%s-%d' % (stamp, seq)
    safe = os.path.basename(basename).replace(os.sep, '_') or 'script.py'
    if not safe.endswith('.py'):
        safe += '.py'
    return '%s-%s' % (stamp, safe)


def name_taken(base, name):
    """True if any stage folder already knows this name - never reuse it."""
    return (any(os.path.exists(os.path.join(base, sub, name))
                for sub in ('inbox', 'running', 'done'))
            or os.path.exists(log_path(base, name)))


def submit(base, source, basename, now=None):
    """Drop the source into the inbox atomically (temp file + rename). Returns the name."""
    ensure_dirs(base)
    for seq in range(1000):
        name = unique_name(basename, now, seq)
        if not name_taken(base, name):
            break
    else:
        raise RuntimeError('could not find a free name for %s in %s' % (basename, base))
    inbox = os.path.join(base, 'inbox')
    tmp = os.path.join(inbox, '.%s.tmp' % name)     # not *.py: the add-in ignores it
    with open(tmp, 'w') as f:
        f.write(source)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, os.path.join(inbox, name))
    return name


def log_path(base, name):
    return os.path.join(base, 'outbox', name + '.log')


def wait_for_log(base, name, timeout, poll=0.2):
    """Contents of the outbox log, or None if it did not appear within `timeout`."""
    path = log_path(base, name)
    deadline = time.time() + timeout
    while True:
        if os.path.exists(path):
            with open(path) as f:
                return f.read()
        left = deadline - time.time()
        if left <= 0:
            return None
        time.sleep(min(poll, left))


def status_of(text):
    """'OK', 'FAIL', or None when the first line is neither."""
    first = (text or '').split('\n', 1)[0].strip()
    return first if first in ('OK', 'FAIL') else None


def where_is(base, name):
    """Which stage folder still holds the submitted script - the timeout diagnosis."""
    for sub in ('inbox', 'running', 'done'):
        if os.path.exists(os.path.join(base, sub, name)):
            return sub
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Run a script inside Fusion through the AgentBridge add-in.')
    parser.add_argument('script', nargs='?', help='Python file to run inside Fusion')
    parser.add_argument('--ping', action='store_true',
                        help='run a built-in one-liner that logs the active document name')
    parser.add_argument('--timeout', type=float, default=120.0,
                        help='seconds to wait for the log (default: 120)')
    args = parser.parse_args(argv)

    if args.ping == bool(args.script):
        parser.error('give either a script or --ping, not both')

    if args.ping:
        source, basename = PING_SOURCE, 'ping.py'
    else:
        with open(args.script) as f:
            source = f.read()
        basename = os.path.basename(args.script)

    base = fusion_dir()
    name = submit(base, source, basename)
    sys.stderr.write('submitted %s\n' % name)

    text = wait_for_log(base, name, args.timeout)
    if text is None:
        stage = where_is(base, name)
        if stage == 'inbox':
            hint = 'still in inbox/ - Fusion is not open or the AgentBridge add-in is not running'
            try:    # withdraw it, so it does not run by surprise when Fusion next starts
                os.remove(os.path.join(base, 'inbox', name))
                hint += '; withdrawn'
            except OSError:
                pass                            # the add-in took it in the meantime
        elif stage == 'running':
            hint = 'still in running/ - the script is executing (or blocked on a dialog/message box)'
        elif stage == 'done':
            hint = 'in done/ but no log - check .knuth/fusion/bridge.log'
        else:
            hint = 'script not found in any stage folder - check .knuth/fusion/bridge.log'
        sys.stderr.write('TIMEOUT after %gs waiting for %s\n  %s\n'
                         % (args.timeout, log_path(base, name), hint))
        return 2

    sys.stdout.write(text if text.endswith('\n') else text + '\n')
    status = status_of(text)
    if status is None:
        sys.stderr.write('malformed log: first line is neither OK nor FAIL\n')
        return 1
    return 0 if status == 'OK' else 1


if __name__ == '__main__':
    sys.exit(main())
