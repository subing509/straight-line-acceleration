#!/usr/bin/env python3
"""Selected geometry and guard regressions; no ROS nodes or vehicle sockets."""
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
tests = root / 'src/acceleration_event/tests'
result = subprocess.run([
    sys.executable, '-B', '-m', 'unittest', '-v',
    'test_track_frame_nodes', 'test_track_tolerance', 'test_legacy_run_guard',
], cwd=str(tests))
sys.exit(result.returncode)
