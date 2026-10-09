#!/usr/bin/env python3
"""Validate collection probe inputs; execution requires the resident PhysX service.

Requires a real fake prepare/plan/binding set; never fabricates an initial anchor
or promotes the historical base rollout into new L2 task success.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'calibration'), str(ROOT/'calibration/infra'),
               str(ROOT/'rule_based_workbench')]
from probe_owner_physx import preflight
from robot_workbench.collection_physx import load_inputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('base', 'source', 'plan', 'binding', 'jaw-mapping', 'plant-fixture', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--gpu', type=int, required=True)
    parser.add_argument('--source-ticks', type=int, default=31)
    parser.add_argument('--max-sim-seconds', type=float, default=30.)
    parser.add_argument('--max-wall-seconds', type=float, default=600.)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.source_ticks <= 90 or not 0 < args.max_sim_seconds <= 60 or not 0 < args.max_wall_seconds <= 900:
        raise ValueError('bounded S0 prefix and time limits required')
    base = json.loads(args.base.read_text())
    source, plan, binding, checked = load_inputs(base,args.source,args.plan,args.binding,args.jaw_mapping,
        plant_fixture=json.loads(args.plant_fixture.read_text()))
    if args.source_ticks >= len(plan['ticks']):
        raise ValueError('source prefix requires a following target for stop revocation')
    checked.update(preflight(base))
    if args.output.exists():
        raise FileExistsError(args.output)
    if args.dry_run:
        print(json.dumps(dict(status='dry-run-passed', **checked)))
        return
    raise ValueError('sealed observations require the same resident plant: use serve_physx_device.py and its shared gateway; this process must not rebuild the source plant')


if __name__ == '__main__':
    main()
