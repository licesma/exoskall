"""Checks the fusion_run <-> AgentBridge folder protocol without Fusion: a fake bridge
thread plays the add-in's part (take the inbox script, write an outbox log) while
fusion_run.main() runs for real against a temp .knuth/fusion tree.

Run: /usr/bin/python3 tools/test_fusion_run.py
"""
import contextlib
import datetime
import io
import os
import shutil
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fusion_run  # noqa: E402

fails = []


def check(label, got, want):
    if got == want:
        print('ok  %s' % label)
    else:
        fails.append('%s: got %r, want %r' % (label, got, want))
        print('FAIL %s: got %r, want %r' % (label, got, want))


def check_that(label, cond):
    check(label, bool(cond), True)


def fake_bridge(base, log_text, delay=0.0, deadline=5.0):
    """Stand-in for the add-in: first *.py in the inbox -> done/ + an outbox log."""
    def work():
        time.sleep(delay)
        stop = time.time() + deadline
        while time.time() < stop:
            names = sorted(n for n in os.listdir(os.path.join(base, 'inbox'))
                           if n.endswith('.py'))
            if names:
                name = names[0]
                os.replace(os.path.join(base, 'inbox', name),
                           os.path.join(base, 'done', name))
                tmp = os.path.join(base, 'outbox', '.%s.log.tmp' % name)
                with open(tmp, 'w') as f:
                    f.write(log_text)
                os.replace(tmp, os.path.join(base, 'outbox', name + '.log'))
                return
            time.sleep(0.02)
    thread = threading.Thread(target=work)
    thread.daemon = True
    thread.start()
    return thread


def run_main(argv):
    """fusion_run.main(argv) -> (exit code, stdout)."""
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        code = fusion_run.main(argv)
    return code, out.getvalue()


base = tempfile.mkdtemp(prefix='fusion_run_test_')
os.environ[fusion_run.FUSION_DIR_ENV] = base
try:
    # ---- directories and naming ----
    fusion_run.ensure_dirs(base)
    for sub in fusion_run.SUBDIRS:
        check_that('ensure_dirs created %s/' % sub, os.path.isdir(os.path.join(base, sub)))
    check_that('ensure_dirs is idempotent', fusion_run.ensure_dirs(base) == base)

    early = datetime.datetime(2026, 9, 14, 10, 0, 0)
    late = datetime.datetime(2026, 9, 14, 10, 0, 1)
    check_that('unique_name ends with the basename',
               fusion_run.unique_name('build.py', early).endswith('-build.py'))
    check_that('unique_name sorts by time',
               fusion_run.unique_name('b.py', early) < fusion_run.unique_name('b.py', late))
    check_that('unique_name keeps a .py suffix', fusion_run.unique_name('x').endswith('.py'))
    check_that('unique_name strips directories',
               os.sep not in fusion_run.unique_name(os.path.join('a', 'b.py')))

    # ---- submit ----
    same_ms = [fusion_run.submit(base, 'log(%d)\n' % i, 'clash.py', early) for i in range(3)]
    check('three submits in one millisecond get three names', len(set(same_ms)), 3)
    check('all three are in the inbox',
          sorted(n for n in os.listdir(os.path.join(base, 'inbox')) if 'clash' in n),
          sorted(same_ms))
    for n in same_ms:
        os.remove(os.path.join(base, 'inbox', n))

    name = fusion_run.submit(base, 'log("hi")\n', 'hi.py')
    dropped = os.path.join(base, 'inbox', name)
    check_that('submit writes the script into inbox/', os.path.isfile(dropped))
    check('submit writes the source verbatim', open(dropped).read(), 'log("hi")\n')
    check('submit leaves no temp file behind',
          [f for f in os.listdir(os.path.join(base, 'inbox')) if f.endswith('.tmp')], [])
    os.remove(dropped)

    # ---- status parsing ----
    check('status_of OK', fusion_run.status_of('OK\nbody\n'), 'OK')
    check('status_of FAIL', fusion_run.status_of('FAIL\nTraceback\n'), 'FAIL')
    check('status_of missing status line', fusion_run.status_of('body only\n'), None)
    check('status_of empty log', fusion_run.status_of(''), None)

    # ---- wait_for_log ----
    t0 = time.time()
    check('wait_for_log gives up', fusion_run.wait_for_log(base, 'nope.py', 0.3), None)
    check_that('wait_for_log waited about the timeout', 0.25 <= time.time() - t0 < 2.0)

    # ---- end to end: OK ----
    fake_bridge(base, 'OK\nactive document: arm v7\n', delay=0.3)
    code, out = run_main(['--timeout', '10', os.path.join(HERE, 'fusion_run.py')])
    check('OK log -> exit 0', code, 0)
    check_that('OK log is printed', 'active document: arm v7' in out)

    # ---- end to end: FAIL, with the traceback passed through ----
    fake_bridge(base, 'FAIL\nTraceback (most recent call last):\nRuntimeError: boom\n')
    code, out = run_main(['--timeout', '10', os.path.join(HERE, 'fusion_run.py')])
    check('FAIL log -> exit 1', code, 1)
    check_that('FAIL traceback is printed', 'RuntimeError: boom' in out)

    # ---- sabotage: a log with no status line must not read as success ----
    fake_bridge(base, 'active document: arm v7\n')
    code, out = run_main(['--timeout', '10', os.path.join(HERE, 'fusion_run.py')])
    check('log without a status line -> exit 1', code, 1)

    # ---- sabotage: a log whose body says OK but whose first line does not ----
    fake_bridge(base, 'done\nOK\n')
    code, _ = run_main(['--timeout', '10', os.path.join(HERE, 'fusion_run.py')])
    check('OK only in the body -> exit 1', code, 1)

    # ---- timeout: nothing answers, and the script is withdrawn from the inbox ----
    before = set(os.listdir(os.path.join(base, 'inbox')))
    err = io.StringIO()
    with contextlib.redirect_stderr(err):
        code, out = run_main(['--timeout', '0.4', os.path.join(HERE, 'fusion_run.py')])
    check('no answer -> exit 2', code, 2)
    check('timed-out script is withdrawn from inbox/',
          set(os.listdir(os.path.join(base, 'inbox'))) - before, set())
    check_that('timeout says where the script is', 'still in inbox/' in err.getvalue())

    # ---- --ping submits the built-in one-liner ----
    fake_bridge(base, 'OK\nactive document: arm v7\n', delay=0.2)
    code, out = run_main(['--ping', '--timeout', '10'])
    check('--ping -> exit 0', code, 0)
    pinged = sorted(n for n in os.listdir(os.path.join(base, 'done')) if n.endswith('-ping.py'))
    check_that('--ping submitted a ping.py', len(pinged) == 1)
    check('--ping source is the built-in one-liner',
          open(os.path.join(base, 'done', pinged[0])).read(), fusion_run.PING_SOURCE)
    check_that('--ping one-liner logs the active document name',
               'activeDocument' in fusion_run.PING_SOURCE and 'log(' in fusion_run.PING_SOURCE)

    # ---- usage errors ----
    for argv, label in (([], 'no script and no --ping'),
                        (['--ping', 'a.py'], 'both a script and --ping')):
        try:
            with contextlib.redirect_stderr(io.StringIO()):
                run_main(argv)
            fails.append('%s should be a usage error' % label)
            print('FAIL %s should be a usage error' % label)
        except SystemExit:
            print('ok  %s is a usage error' % label)
finally:
    shutil.rmtree(base, ignore_errors=True)

print()
if fails:
    print('FAILED:', fails)
    sys.exit(1)
print('all checks passed')
