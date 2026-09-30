"""Compatibility import; implementation lives in episode_io.video."""
import sys
from episode_io import video as _implementation
sys.modules[__name__] = _implementation
