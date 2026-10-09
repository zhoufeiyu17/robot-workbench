"""Public physical14 contract used by the rule based collection workbench."""

from __future__ import annotations

import math
from typing import Any


SCHEMA_VERSION = "physical14.v1"
ARM_KEYS = ("left_arm_joints", "right_arm_joints")
GRIPPER_KEYS = ("left_gripper", "right_gripper")
TOP_LEVEL_KEYS = {"schema_version", "encoding", "units", *ARM_KEYS, *GRIPPER_KEYS}


def _finite_number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{path} must be a finite number")
    return float(value)


def validate_physical14(payload: Any, *, kind: str = "state", gripper_max_mm: float | None = None, gripper_min_mm: float = 0.) -> dict[str, Any]:
    """Validate a state or absolute physical target without exposing model-side shapes."""
    if not isinstance(payload, dict):
        raise ValueError(f"{kind} must be an object")
    unknown = set(payload) - TOP_LEVEL_KEYS
    if unknown:
        raise ValueError(f"unknown physical14 fields: {', '.join(sorted(unknown))}")
    if payload.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"schema_version must be {SCHEMA_VERSION}")
    if payload.get("encoding") != "absolute":
        raise ValueError("state/action encoding must be absolute")
    if payload.get("units") != {"joints": "rad", "grippers": "mm"}:
        raise ValueError('units must be {"joints":"rad","grippers":"mm"}')
    for key in ARM_KEYS:
        values = payload.get(key)
        if not isinstance(values, list) or len(values) != 6:
            raise ValueError(f"{key} must contain exactly 6 joint targets")
        for index, value in enumerate(values):
            _finite_number(value, f"{key}[{index}]")
    for key in GRIPPER_KEYS:
        value = _finite_number(payload.get(key), key)
        if value < gripper_min_mm or gripper_max_mm is not None and value > gripper_max_mm:
            limit = f"[{gripper_min_mm:g}, infinity]" if gripper_max_mm is None else f"[{gripper_min_mm:g}, {gripper_max_mm:g}]"
            raise ValueError(f"{key} must be in {limit} mm")
    return payload



def validate_physical_feedback(payload, profile=None, *, enforce_range=True):
    """Measured jaw envelope: -1..84 mm for the 83 mm ARX command range.

    Keep raw measurements; this allowance never expands command targets.
    """
    if not enforce_range:
        return validate_physical14(payload,kind='feedback',gripper_min_mm=-math.inf)
    maximum=profile['gripper']['max_opening_mm'] if profile is not None else 83.
    return validate_physical14(payload,kind='feedback',gripper_min_mm=-1.,gripper_max_mm=maximum+1.)


def validate_command(payload: Any, *, gripper_max_mm: float | None = None) -> dict[str, Any]:
    if not isinstance(payload, dict) or set(payload) != {"action", "dry_run"}:
        raise ValueError("command must contain exactly action and dry_run")
    if not isinstance(payload["dry_run"], bool):
        raise ValueError("dry_run must be boolean")
    validate_physical14(payload["action"], kind="action", gripper_max_mm=gripper_max_mm)
    return payload


def validate_boundary(name: str, payload: Any) -> dict[str, Any]:
    """Keep external adapters intentionally small until real services are connected."""
    if not isinstance(payload, dict):
        raise ValueError(f"{name} input must be an object")
    if name == "calibration":
        required = {"result_id", "source", "status"}
        if not required <= payload.keys():
            raise ValueError("calibration input requires result_id, source, status")
        if payload["status"] not in {"ready", "unavailable"}:
            raise ValueError("calibration status must be ready or unavailable")
    elif name == "online_detection":
        if "timestamp_s" not in payload or "objects" not in payload:
            raise ValueError("online_detection input requires timestamp_s and objects")
        _finite_number(payload["timestamp_s"], "timestamp_s")
        if not isinstance(payload["objects"], list):
            raise ValueError("online_detection objects must be a list")
    return payload


def demo_state() -> dict[str, Any]:
    zeros = [0.0] * 6
    return {
        "schema_version": SCHEMA_VERSION,
        "encoding": "absolute",
        "units": {"joints": "rad", "grippers": "mm"},
        "left_arm_joints": zeros,
        "right_arm_joints": zeros,
        "left_gripper": 38.0,
        "right_gripper": 38.0,
    }
