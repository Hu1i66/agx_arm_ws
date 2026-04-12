#!/usr/bin/env python3
from pathlib import Path
import os
import sys


def ensure_workspace_sdk_on_path() -> str:
    """Prepend workspace vendored SDK path to sys.path if available."""
    candidates = []

    env_ws = os.environ.get("AGX_ARM_WS")
    if env_ws:
        candidates.append(Path(env_ws) / "third_party" / "python")

    here = Path(__file__).resolve()
    for parent in [here.parent] + list(here.parents):
        candidates.append(parent / "third_party" / "python")

    for candidate in candidates:
        if candidate.is_dir():
            candidate_str = str(candidate)
            if candidate_str not in sys.path:
                sys.path.insert(0, candidate_str)
            return candidate_str

    return ""
