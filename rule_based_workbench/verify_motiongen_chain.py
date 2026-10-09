"""Actual-GPU offline chain regression from frozen geometry; never dual-DSW E2E evidence.

This explicitly synthesizes a simulation source around historical geometry. Its
plans receive an offline-only schema and must be rejected by the publish bridge.
No camera, robot control, SSH, or native action execution occurs here.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time

from integration import DualGpuPipeline, _digest
from motiongen_plan import plan_loaded, write_new


def synthetic_source(parent, provenance):
    from test_motiongen_pipeline import motiongen_fixture
    source, _ = motiongen_fixture()
    source.update(run_id='offline-motiongen-chain-'+provenance['planning_input_sha256'][:12],
                  test_fixture=True, synthetic_fixture=True, fixture_provenance=provenance)
    profile = source['calibration_profile']
    profile.update(profile_id='offline-motiongen-chain-synthetic-profile',
                   source=dict(kind='simulation-fixture', hardware_calibrated=False, synthetic_fixture=True))
    scene = deepcopy(parent['scene'])
    # Preserve the frozen asset-local poses, but not any claim that this is a new
    # observation from the simulated-real DSW or a fresh physical acquisition.
    scene.update(episode_id=source['run_id'], seed=0)
    scene['source'] = dict(source['scene']['source'], synthetic_fixture=True,
        observation_mode='offline_frozen_geometry_fixture', parent_scene_digest=parent['scene']['scene_digest'],
        evidence_scope='synthetic validator window; no new detection/camera acquisition')
    keys = ('episode_id', 'seed', 'machine_asset', 'environment_from_machine_root', 'environment_from_cup')
    scene['episode_scene'] = {key: scene[key] for key in keys}
    scene['scene_digest'] = _digest({key: value for key, value in scene.items() if key != 'scene_digest'})
    source['scene'] = scene
    DualGpuPipeline().ingest_source(source)
    return source


def seal_offline(plan, provenance):
    result = deepcopy(plan)
    result.update(offline_candidate=True, synthetic_fixture=True, fixture_provenance=provenance)
    result['planning_validation'].update(schema_version='offline_curobo_motiongen_path_validation.v1',
        offline_candidate=True, synthetic_fixture=True,
        fixture_provenance_sha256=_digest(provenance), dual_dsw_e2e_completed=False)
    return result


def main(args):
    args.output.mkdir(parents=True, exist_ok=False)
    began, updates = time.monotonic(), 0
    result = dict(status='failed', offline_candidate=True, synthetic_fixture=True,
                  dual_dsw_e2e_completed=False, camera_acquisition=False, hardware_access=False)
    try:
        parent = json.loads(args.source.read_text())
        provenance = dict(planning_input_path=str(args.input.resolve()), parent_source_path=str(args.source.resolve()),
            planning_input_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
            parent_source_sha256=hashlib.sha256(args.source.read_bytes()).hexdigest(),
            parent_source_schema=parent['schema_version'],
            scope='historical geometry regression with a synthetic simulation source; not observed dual-DSW evidence')
        from playground_geometry import load
        planner, observation, data = load(args.input, parent, args.native_code, args.assets)
        from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
        source = synthetic_source(parent, provenance)
        fixture_data = deepcopy(data)
        fixture_data['fixture_provenance'] = provenance
        write_new(args.output/'synthetic-source.json', source)
        write_new(args.output/'fixture-provenance.json', provenance)

        def diagnostic(name, value):
            nonlocal updates
            if name == 'motiongen-receipts-latest':
                name = f'motiongen-receipts-{updates:03d}.json'; updates += 1
            write_new(args.output/name, value)

        base, plan, evidence = plan_loaded(planner, observation, fixture_data, source,
            ac_one_asset_spec(args.assets).urdf_path, speed_scale=args.speed_scale, diagnostic=diagnostic)
        base, plan = seal_offline(base, provenance), seal_offline(plan, provenance)
        bridge = DualGpuPipeline(); bridge.ingest_source(source)
        for artifact in (base, plan):
            try:
                bridge.accept_plan(artifact)
            except ValueError:
                pass
            else:
                raise AssertionError('offline synthetic plan was accepted for publishing')
        write_new(args.output/'base-plan-offline.json', base)
        write_new(args.output/'plan-offline.json', plan)
        result.update(status='offline-motiongen-chain-validated', timeline_digest=plan['timeline_digest'],
                      publish_rejected=True, fixture_provenance=provenance, evidence=evidence,
                      scope='actual MotionGen core chain and native final geometry only; no action execution')
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
    result['elapsed_seconds'] = time.monotonic()-began
    write_new(args.output/'result.json', result)
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return 0 if result['status'] == 'offline-motiongen-chain-validated' else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('input', 'source', 'native-code', 'assets', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--speed-scale', type=float, default=1.)
    raise SystemExit(main(parser.parse_args()))
