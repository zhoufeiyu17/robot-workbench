#!/usr/bin/env python3
"""Read-only closed-release checks in the declared detector interpreter.

This checks the runtime's real startup/catalog contract, not model execution or
GPU readiness. No application/backend factory, cache or output is created.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def read_release(node_config, runtime_root):
    node_config, runtime_root = Path(node_config), Path(runtime_root)
    if not node_config.is_absolute() or not node_config.is_file():
        raise ValueError('absolute existing detection_node_config required')
    if not runtime_root.is_absolute() or not all(
            (runtime_root / name).is_dir() for name in ('scripts', 'ros2/online_detection_ros')):
        raise ValueError('absolute existing detection_runtime_root required')
    sys.path.insert(0, str(runtime_root / 'ros2/online_detection_ros'))
    from online_detection_ros_node.runtime.runtime_exec import _startup_from_node_config
    from online_detection_ros_node.configuration.startup import load_startup_catalog
    from online_detection_ros_node.pose.configuration import compile_foundationpose_assets

    startup = _startup_from_node_config(node_config)
    if startup is None or startup.runtime_mode.value != 'DEVELOPMENT':
        raise ValueError('pinned DEVELOPMENT release required')
    catalog = load_startup_catalog(replace(startup, verify_development_optional_artifacts=True))
    assets = compile_foundationpose_assets(catalog)
    if not assets:
        raise ValueError('FoundationPose-enabled release with bound assets required')
    if catalog.unverified_artifacts:
        raise ValueError('all detector release artifacts must be verified')
    return dict(status='passed', scope='closed_release_only',
        detection_runtime_root=str(runtime_root.resolve(strict=True)),
        node_config=str(node_config.resolve(strict=True)),
        node_config_sha256=hashlib.sha256(node_config.read_bytes()).hexdigest(),
        release_root=str(catalog.release_root.resolve(strict=True)),
        release_manifest_digest=catalog.release_manifest_digest,
        targets_digest=catalog.targets_digest,
        runtime_config_id=catalog.runtime_config.runtime_config_id,
        verified_artifact_count=len(catalog.verified_artifact_digests),
        pose_asset_count=len(assets))


def preflight_release(pipeline_config, runtime_root, *, timeout=180):
    """Run before GPU, C collection or setup output; never inherit Isaac Python."""
    from rule_based_workbench.launch_environment import clean_environment
    python = Path(pipeline_config['detection_python'])
    if not python.is_absolute() or not python.is_file():
        raise ValueError('absolute existing detection_python required')
    command = [str(python), '-I', '-B', str(Path(__file__).resolve()),
        '--node-config', str(pipeline_config['detection_node_config']),
        '--runtime-root', str(runtime_root)]
    try:
        result = subprocess.run(command, check=False, capture_output=True, text=True, timeout=timeout,
            env=clean_environment({'CUDA_VISIBLE_DEVICES': '', 'PYTHONDONTWRITEBYTECODE': '1'}))
    except subprocess.TimeoutExpired as error:
        raise RuntimeError('detector release CPU preflight timed out before GPU startup') from error
    if result.returncode:
        raise RuntimeError('detector release CPU preflight failed before GPU startup: ' +
                           (result.stderr or result.stdout)[-8000:].strip())
    receipt = json.loads(result.stdout)
    if receipt.get('status') != 'passed' or receipt.get('scope') != 'closed_release_only':
        raise RuntimeError('invalid detector release CPU preflight receipt')
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node-config', type=Path, required=True)
    parser.add_argument('--runtime-root', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(read_release(args.node_config, args.runtime_root), allow_nan=False))
