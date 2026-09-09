"""Explicit, lazy plugin loading. Listing or validating a selection opens no device."""

import argparse
from dataclasses import dataclass
import importlib
import json
from pathlib import Path
import tomllib

from .project import PROJECT_ROOT


@dataclass(frozen=True)
class Plugin:
    kind: str
    name: str
    factory: str
    hardware: bool = False
    simulation_only: bool = False
    capabilities: tuple[str, ...] = ()


class Registry:
    def __init__(self, path=None):
        self.path = Path(path) if path else PROJECT_ROOT / "configs/plugins.toml"
        with self.path.open("rb") as stream:
            self.entries = tomllib.load(stream)

    def spec(self, kind, name, *, allow_hardware=True):
        entries = self.entries.get(kind, {})
        if name not in entries:
            raise ValueError(f"Unknown {kind} plugin {name!r}; available: {', '.join(sorted(entries))}")
        entry = entries[name]
        spec = Plugin(kind, name, entry["factory"], entry.get("hardware", False),
                      entry.get("simulation_only", False), tuple(entry.get("capabilities", ())))
        if not allow_hardware and spec.hardware:
            raise ValueError(f"Simulation cannot load hardware plugin {kind}:{name}")
        return spec

    def load(self, kind, name, *, allow_hardware=True):
        spec = self.spec(kind, name, allow_hardware=allow_hardware)
        module, separator, attribute = spec.factory.partition(":")
        if not separator or not module or not attribute:
            raise ValueError(f"Invalid plugin factory: {spec.factory!r}; expected module:factory")
        factory = getattr(importlib.import_module(module), attribute)
        if not callable(factory):
            raise ValueError(f"Plugin factory is not callable: {spec.factory}")
        return factory

    def create(self, kind, name, *, allow_hardware=True, **options):
        return self.load(kind, name, allow_hardware=allow_hardware)(**options)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", help="Plugin registry TOML (default: configs/plugins.toml)")
    args = parser.parse_args()
    print(json.dumps(Registry(args.registry).entries, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
