from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field


RequestId = str | int


class CommandRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    type: Literal["command"]
    request_id: RequestId
    feature: str = Field(min_length=1, max_length=64)
    command: str = Field(min_length=1, max_length=64)
    payload: dict[str, Any] = Field(default_factory=dict)


class StreamRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    type: Literal["stream"]
    feature: Literal["arm", "gripper", "teleoperation"]
    stream: str = Field(min_length=1, max_length=64)
    command: Literal["start", "update", "stop"]
    sequence: int
    payload: dict[str, Any] = Field(default_factory=dict)
