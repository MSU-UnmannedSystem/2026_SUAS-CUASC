import sys
import threading
from pathlib import Path

import pytest

# Imports use the package path competition.drone_simulator..., so the
# "2025 code" folder must be on sys.path.
CODE_ROOT = Path(__file__).resolve().parents[3]
# The tests share helpers (link_fakes.py) that live next to this file.
for path in (CODE_ROOT, Path(__file__).resolve().parent):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))


@pytest.fixture
def background():
    """Runs ``fn`` every ``period_s`` in a thread until the test ends."""
    stop = threading.Event()
    threads = []

    def start(fn, period_s=0.1):
        def loop():
            while not stop.is_set():
                fn()
                stop.wait(period_s)
        t = threading.Thread(target=loop, daemon=True)
        threads.append(t)
        t.start()

    yield start
    stop.set()
    for t in threads:
        t.join(timeout=1)
