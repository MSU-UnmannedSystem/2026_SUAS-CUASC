import sys
from pathlib import Path

# Imports use the package path competition.drone_simulator..., so the
# "2025 code" folder must be on sys.path.
CODE_ROOT = Path(__file__).resolve().parents[3]
if str(CODE_ROOT) not in sys.path:
    sys.path.insert(0, str(CODE_ROOT))
