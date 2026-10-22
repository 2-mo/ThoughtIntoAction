"""Explicit terminal status colors; plain output retains the same words."""
from dataclasses import dataclass
from typing import Literal


@dataclass(frozen=True)
class Status:
    text: str
    level: Literal['ready', 'waiting', 'error', 'disabled']

    @property
    def style(self):
        return {'ready': 'bold green', 'waiting': 'bold yellow',
                'error': 'bold red', 'disabled': 'dim'}[self.level]

    def __str__(self):
        return self.text


def status_text(value):
    from rich.text import Text

    if isinstance(value, Status):
        return Text('● ' + value.text, style=value.style)
    return Text(str(value))
