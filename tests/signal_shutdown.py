"""Send signals only to test subprocesses, after their Action has been accepted."""

import os
import selectors
import signal
import subprocess
import sys
import time


def check_process(executable, signals):
    args = [executable] + ([] if signals else ["--normal"])
    process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        with selectors.DefaultSelector() as selector:
            selector.register(process.stdout, selectors.EVENT_READ)
            deadline = time.monotonic() + 5
            output = b""
            while b"READY\n" not in output:
                if time.monotonic() >= deadline:
                    raise RuntimeError("test Action was not accepted")
                if selector.select(0.1):
                    chunk = os.read(process.stdout.fileno(), 4096)
                    if not chunk:
                        raise RuntimeError("fixture exited before READY")
                    output += chunk
        started = time.monotonic()
        for current in signals:
            os.kill(process.pid, current)
            time.sleep(0.01)
        tail, errors = process.communicate(timeout=5)
        output += tail
        if process.returncode != 0 or b"DRAINED\n" not in output:
            raise RuntimeError(
                f"fixture failed: {process.returncode}\n{errors.decode()}"
            )
        if signals and time.monotonic() - started < 0.09:
            raise RuntimeError("signal exit skipped asynchronous cleanup")
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()


for sequence in (
    [],
    [signal.SIGINT],
    [signal.SIGTERM],
    [signal.SIGINT, signal.SIGTERM],
):
    check_process(sys.argv[1], sequence)
print("normal, SIGINT, SIGTERM and repeated-signal exits drained")
