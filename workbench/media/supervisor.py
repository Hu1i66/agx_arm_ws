"""Keep the media worker alive without coupling it to the workbench task state."""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import time


def main(argv=None):
    stopping = False
    child = None

    def stop(signum, frame):
        nonlocal stopping
        stopping = True
        if child is not None and child.poll() is None:
            try:
                child.send_signal(signum)
            except ProcessLookupError:
                pass

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    delay = 1.0
    while not stopping:
        started = time.monotonic()
        child = subprocess.Popen(
            [sys.executable, "-m", "workbench.media.worker"],
            cwd=os.environ.get("AGX_ARM_WS") or os.getcwd(),
            env=os.environ.copy(),
        )
        returncode = child.wait()
        if stopping:
            break
        print(
            f"[tablet-media-supervisor] worker exited code={returncode}; "
            f"restarting in {delay:.1f}s",
            flush=True,
        )
        if time.monotonic() - started > 30:
            delay = 1.0
        else:
            delay = min(10.0, delay * 1.5)
        deadline = time.monotonic() + delay
        while not stopping and time.monotonic() < deadline:
            time.sleep(0.1)
    if child is not None and child.poll() is None:
        child.terminate()
        try:
            child.wait(timeout=3)
        except subprocess.TimeoutExpired:
            child.kill()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
