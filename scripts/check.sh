#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
ROOT=$(pwd)
export PATH="$ROOT/.venv/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$ROOT:$ROOT/calibration:$ROOT/calibration/infra:$ROOT/calibration/examples/scripts/debug:$ROOT/rule_based_workbench${PYTHONPATH:+:$PYTHONPATH}"
sh calibration/scripts/check.sh
python tests/test_owner_clock.py
python tests/test_joint_bounds.py
python tests/test_manual_self_collision.py
python tests/test_recording_preparation.py
python tests/test_recording_io.py
python tests/test_recording_event_budget.py
python tests/test_recording_admission.py
python tests/test_recording_process.py
python tests/test_recording_memory.py
python tests/test_recording_segments.py
python tests/test_recording_quality.py
python tests/test_recording_history.py
python rule_based_workbench/test_rgbd_segments.py
python rule_based_workbench/test_segmented_playback.py
python tests/test_physx_boundary.py
python tests/test_fake_actuation_bias.py
python tests/test_execution_settings.py
python tests/test_inference.py
python tests/test_inference_owner.py
python tests/test_inference_transport.py
python tests/test_inference_workflow.py
python tests/test_inference_timing.py
python tests/test_inference_reference.py
python tests/test_inference_catalog.py
python tests/test_inference_memory.py
python tests/test_gc_timing.py
python tests/test_motion_reference.py
python tests/test_target_interpreter.py
python tests/test_shared_target_routes.py
python tests/test_prepared_reference_backend.py
python tests/test_control_owner_process.py
python tests/test_sdk_process.py
python tests/test_operations.py
python tests/test_information_flow.py
python tests/test_physical_cup_return_service.py
python tests/test_managed_services.py
python tests/test_backend_launcher.py
python tests/test_control_plan.py
python tests/test_fake_pipeline_spawn.py
python rule_based_workbench/test_shared_model_gpu.py
python tests/test_management.py
python tests/test_environment_manager.py
python tests/test_environment_state.py
python tests/test_environment_presentation.py
python tests/test_control_operations.py
python tests/test_machine_profile.py
python tests/test_maintenance.py
python tests/test_device_bootstrap.py
python tests/test_arx_output_guard.py
python tests/test_release_trace.py
python tests/test_arx_guarded_backend.py
python tests/test_sdk_quiesce.py
python tests/test_device_profile.py
python tests/test_remote_collection_setup.py
python tests/test_physical_endpoint.py
python tests/test_rgbd_service.py
python tests/test_camera_rebind.py
python tests/test_fake_camera_operations.py
python tests/test_streaming_recording.py
python tests/test_unified_runtime.py
python tests/test_http_service.py
python tests/test_device_release.py
python tests/test_release_races.py
python tests/test_shared_bridges.py
python tests/test_workflow_adapter.py
python tests/test_physx_camera.py
python tests/test_physx_task.py
python tests/test_resident_collection.py
python tests/test_collection_service_recovery.py
python tests/test_warm_diagnostic.py
python tests/test_collection_preparation_http.py
python tests/test_detector_adapter.py
python tests/test_continuous_observation.py
python tests/test_detector_preflight.py
python tests/test_target_preview.py
python tests/test_preview_http.py
python tests/test_preview_service.py
python tests/test_preview_subscription.py
python tests/test_cpu_preview.py
python tests/test_physical_preview.py
python tests/test_target_model.py
if [ -n "${AC_ONE_ASSET_ROOT:-}" ]; then
    python tests/test_collection_setup.py --a1 "$AC_ONE_ASSET_ROOT" --roles 4
    python tests/test_collection_setup.py --a1 "$AC_ONE_ASSET_ROOT" --roles 5
else
    echo 'SKIP collection setup 4/5-role asset checks: set AC_ONE_ASSET_ROOT to sealed A1 assets'
fi
python tests/test_serve_physx_device.py
python tests/test_dev_calibration_resume.py
python tests/test_collection_history.py
python tests/test_fake_session.py
python tests/test_lan_gateway.py
python tests/test_site_cameras.py
python tests/test_camera_sources.py
python tests/test_capture_diagnostics.py
python tests/test_common_camera_preview.py
python tests/test_camera_owner.py
python tests/test_ssh_tcp_transport.py
python tests/test_fake_scene.py
python tests/test_cup_return_service.py
python rule_based_workbench/test_cup_return_offset.py
python tests/test_fake_session_profile.py
python tests/test_atomic_evidence_save.py
python tests/test_setup_resident_collection.py
python tests/test_collection_physx_protocol.py
python rule_based_workbench/test_fake_device_pipeline.py
python rule_based_workbench/test_cup_return_replay.py
python rule_based_workbench/test_grasp_contact_geometry.py
python rule_based_workbench/test_grasp_contact_certificate.py
python rule_based_workbench/test_motiongen_pipeline.py
python rule_based_workbench/test_motiongen_cache.py
python rule_based_workbench/test_left3_refinement.py
python rule_based_workbench/test_scene_service.py
python rule_based_workbench/test_gripper_release.py
python rule_based_workbench/test_sample_clock.py
python rule_based_workbench/test_recording_wall_clock.py
python rule_based_workbench/test_a1_asset_derivation.py
python rule_based_workbench/test_local_execution.py
python rule_based_workbench/test_launch_environment.py
python rule_based_workbench/test_owned_process.py
python rule_based_workbench/test_native_failure.py
python rule_based_workbench/test_playground_interrupt.py
python rule_based_workbench/test_playground_execution_process.py
python rule_based_workbench/test_resident_task_receipt.py
python rule_based_workbench/test_joint_space.py
python rule_based_workbench/test_playground_plan.py
python rule_based_workbench/test_playground_sim.py
python rule_based_workbench/test_playground_failure.py
python rule_based_workbench/test_playground_camera.py
python rule_based_workbench/test_restore_prepared.py
python rule_based_workbench/test_operator_assisted_source.py
python rule_based_workbench/test_assisted_gripper.py
python rule_based_workbench/test_fixed_right_policy.py
python rule_based_workbench/test_playground_assisted.py
python rule_based_workbench/test_playground_assisted_ui.py
for name in test_playground_motiongen_entry test_real_pipeline test_playground_pipeline test_real_acquisition test_physical_motiongen test_initial_state test_real_preparation_app test_real_bridge test_frozen_executor test_disabled_grippers; do
    python "rule_based_workbench/$name.py"
done
python rule_based_workbench/test_real_control.py "$ROOT/calibration"
if command -v node >/dev/null 2>&1; then
    node calibration/tests/test_joint_head_workbench_ui.js
    node tests/test_web_ui.mjs
    node tests/test_operations_ui.mjs
    node tests/test_session_ui.mjs
    node tests/test_physical_endpoint_ui.mjs
    node tests/test_target_viewer_fk.mjs tests/fixtures/target-model.json tests/fixtures/target-native-fk.json
fi
