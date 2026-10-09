"""CPU-only checks for preserving the native terminal partial control tick."""
from contextlib import redirect_stdout
from copy import deepcopy
import io
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

from export_plan import export
from test_app import fixture_messages


def main():
    source, plan, _ = fixture_messages()
    arm = SimpleNamespace(joint_positions_rad=[0.0] * 6, gripper_open=True)
    module = ModuleType("robot_sim.specs.action")
    module.ac_one_dual_arm_zero_home_action_spec = lambda: SimpleNamespace(
        decode=lambda action: SimpleNamespace(left=arm, right=arm))
    report = {"outcome": "success", "requested_scene": source["scene"]["episode_scene"],
              "identity": {"planner_id": plan["planner_id"], "motion_profile": "continuous-baseline",
                           "acceptance": {"open_position_m": .0415},
                           "effective_gripper_opening_mm": 83.0,
                           "control": {"physics_hz": 240, "substeps": 8}}}
    steps = [{"stage": "home", "info": {"execution_report": {
        "action_applied": True, "actual_substeps": n, "applied_action": [0] * 14,
        "terminated": i == 1, "truncated": False,
        "event": {"reason": "success"} if i == 1 else None}}} for i, n in enumerate((8, 4))]
    with TemporaryDirectory() as folder, patch.dict(sys.modules, {module.__name__: module}):
        root = Path(folder)
        (root / "source.json").write_text(json.dumps(source))
        (root / "report.json").write_text(json.dumps(report))

        def attempt(rows, path):
            (root / "steps.jsonl").write_text("\n".join(json.dumps(x) for x in rows))
            with redirect_stdout(io.StringIO()):
                export(root / "source.json", root, root / path)

        attempt(steps, "plan.json")
        result = json.loads((root / "plan.json").read_text())
        assert result["native_substeps"] == [8, 4] and result["duration_s"] == 12 / 240
        assert result["gripper_opening_mm"] == 83 and result["ticks"][0]["target"]["left_gripper"] == 83
        assert result["ticks"][1]["time_s"] == 8 / 240
        for index, change in [(0, {"actual_substeps": 4}), (1, {"action_applied": False}),
                              (1, {"event": {"reason": "collision"}}), (1, {"truncated": True}),
                              (0, {"terminated": True})]:
            invalid = deepcopy(steps)
            invalid[index]["info"]["execution_report"].update(change)
            try:
                attempt(invalid, "invalid.json")
            except ValueError:
                pass
            else:
                raise AssertionError(f"accepted invalid native execution: {change}")
        assert not (root / "invalid.json").exists()
    print("Native export terminal/substep boundary: PASS (CPU fixtures only)")


if __name__ == "__main__":
    main()
