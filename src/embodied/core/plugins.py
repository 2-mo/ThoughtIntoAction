"""Compatibility module identity for the extracted registry, including old hooks."""
import sys
from embodied.contracts import registry as _registry

if __name__ == "__main__":
    _registry.main()
else:
    sys.modules[__name__] = _registry
