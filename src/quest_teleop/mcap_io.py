"""Compatibility import; implementation lives in episode_io.mcap_io."""
import sys
from episode_io import mcap_io as _implementation
sys.modules[__name__] = _implementation
