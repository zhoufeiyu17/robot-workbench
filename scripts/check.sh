#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
ROOT=$(pwd)
export PATH="$ROOT/.venv/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1
sh calibration/scripts/check.sh
python tests/test_owner_clock.py
python tests/test_physx_boundary.py
python tests/test_unified_runtime.py
for name in test_playground_motiongen_entry test_real_pipeline test_playground_pipeline test_real_acquisition test_physical_motiongen test_initial_state test_real_preparation_app test_frozen_executor; do
    python "rule_based_workbench/$name.py"
done
python rule_based_workbench/test_real_control.py "$ROOT/calibration"
if command -v node >/dev/null 2>&1; then
    node calibration/tests/test_joint_head_workbench_ui.js
fi
