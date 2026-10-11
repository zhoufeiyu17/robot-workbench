#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
ROOT=$(pwd)
export PATH="$ROOT/.venv/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
sh calibration/scripts/check.sh
python tests/test_owner_clock.py
python tests/test_joint_bounds.py
python tests/test_recording_preparation.py
python tests/test_recording_io.py
python tests/test_physx_boundary.py
python tests/test_unified_runtime.py
python tests/test_http_service.py
python tests/test_device_release.py
python tests/test_release_races.py
python tests/test_shared_bridges.py
python tests/test_workflow_adapter.py
python tests/test_physx_camera.py
python tests/test_physx_task.py
python tests/test_resident_collection.py
python tests/test_warm_diagnostic.py
python tests/test_collection_preparation_http.py
python tests/test_detector_adapter.py
python tests/test_detector_preflight.py
python tests/test_target_preview.py
python tests/test_preview_http.py
python tests/test_preview_service.py
python tests/test_preview_subscription.py
if [ -n "${AC_ONE_ASSET_ROOT:-}" ]; then
    python tests/test_collection_setup.py --a1 "$AC_ONE_ASSET_ROOT" --roles 4
    python tests/test_collection_setup.py --a1 "$AC_ONE_ASSET_ROOT" --roles 5
else
    echo 'SKIP collection setup 4/5-role asset checks: set AC_ONE_ASSET_ROOT to sealed A1 assets'
fi
python tests/test_serve_physx_device.py
python tests/test_dev_calibration_resume.py
python tests/test_collection_history.py
python tests/test_atomic_evidence_save.py
python tests/test_setup_resident_collection.py
python tests/test_collection_physx_protocol.py
python rule_based_workbench/test_fake_device_pipeline.py
python rule_based_workbench/test_grasp_contact_geometry.py
python rule_based_workbench/test_grasp_contact_certificate.py
python rule_based_workbench/test_motiongen_pipeline.py
python rule_based_workbench/test_gripper_release.py
python rule_based_workbench/test_sample_clock.py
python rule_based_workbench/test_recording_wall_clock.py
python rule_based_workbench/test_a1_asset_derivation.py
python rule_based_workbench/test_local_execution.py
python rule_based_workbench/test_launch_environment.py
python rule_based_workbench/test_owned_process.py
python rule_based_workbench/test_native_failure.py
python rule_based_workbench/test_playground_interrupt.py
python rule_based_workbench/test_resident_task_receipt.py
python rule_based_workbench/test_joint_space.py
python rule_based_workbench/test_playground_plan.py
python rule_based_workbench/test_playground_sim.py
for name in test_playground_motiongen_entry test_real_pipeline test_playground_pipeline test_real_acquisition test_physical_motiongen test_initial_state test_real_preparation_app test_real_bridge test_frozen_executor; do
    python "rule_based_workbench/$name.py"
done
python rule_based_workbench/test_real_control.py "$ROOT/calibration"
if command -v node >/dev/null 2>&1; then
    node calibration/tests/test_joint_head_workbench_ui.js
    node tests/test_web_ui.mjs
fi
