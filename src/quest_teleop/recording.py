"""Compatibility import; implementation lives in episode_io.recording."""
import sys
from episode_io import recording as _implementation
sys.modules[__name__] = _implementation
