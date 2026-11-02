"""scripts/ から piper_pp を import できるようにする（pip install -e . していなくても動く）"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
