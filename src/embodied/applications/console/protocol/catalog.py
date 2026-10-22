from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True, slots=True)
class CommandSpec:
    name: str
    kernel_action: str
    description: str
    mutates_hardware: bool = True


@dataclass(frozen=True, slots=True)
class FeatureSpec:
    name: str
    description: str
    commands: tuple[CommandSpec, ...]
    streams: tuple[str, ...] = ()


class CommandCatalog:
    def __init__(self, features: Iterable[FeatureSpec]) -> None:
        self.features = tuple(features)
        self._commands = {
            (feature.name, command.name): command
            for feature in self.features
            for command in feature.commands
        }
        self._kernel_actions = {
            command.kernel_action: (feature.name, command.name)
            for feature in self.features
            for command in feature.commands
        }

    def resolve(self, feature: str, command: str) -> CommandSpec:
        try:
            return self._commands[(feature, command)]
        except KeyError as exc:
            raise ValueError(f"Unsupported command: {feature}.{command}") from exc

    def public_name(self, kernel_action: str | None) -> tuple[str, str]:
        if kernel_action is None:
            return "system", "unknown"
        return self._kernel_actions.get(kernel_action, ("system", kernel_action))

    def describe(self) -> list[dict[str, object]]:
        return [
            {
                "name": feature.name,
                "description": feature.description,
                "streams": list(feature.streams),
                "commands": [
                    {
                        "name": command.name,
                        "description": command.description,
                        "mutates_hardware": command.mutates_hardware,
                    }
                    for command in feature.commands
                ],
            }
            for feature in self.features
        ]
