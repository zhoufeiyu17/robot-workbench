"""Benchmark frozen Playground inputs offline; never connect to robot services."""
import argparse
import cProfile
import hashlib
import json
from pathlib import Path
import pstats
import time


def timed(stages, name, function):
    row = dict(stage=name, passed=False)
    stages.append(row)
    wall, cpu = time.perf_counter(), time.process_time()
    try:
        value = function()
        row['passed'] = True
        return value
    except Exception as error:
        row['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        row.update(wall_seconds=time.perf_counter()-wall, cpu_seconds=time.process_time()-cpu)


def profile_summary(profile):
    if not profile.getstats():
        return []
    stats = pstats.Stats(profile)
    return [dict(file=file, line=line, function=name, primitive_calls=cc, calls=nc,
                 self_seconds=tt, cumulative_seconds=ct)
            for (file, line, name), (cc, nc, tt, ct, _) in
            sorted(stats.stats.items(), key=lambda item: item[1][3], reverse=True)[:40]]


def write_new(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write('\n')


def run(args):
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(schema_version='offline_planning_benchmark.v1', passed=False, offline_candidate=True,
                  until=args.until, speed_scale=args.speed_scale, stages=[],
                  inputs={name: str(getattr(args, name).resolve())
                          for name in ('input', 'source', 'native_code', 'assets')},
                  profiling_enabled=args.profile, geometry_backend=args.geometry_backend,
                  fast_cup_audit=args.fast_cup_audit,
                  timing_note='Stage wall times are sequential; nested function cumulative times must not be added.')
    profiler = cProfile.Profile() if args.profile else None
    started = time.perf_counter()
    try:
        source = json.loads(args.source.read_text())
        report['sha256'] = {name: hashlib.sha256(getattr(args, name).read_bytes()).hexdigest()
                            for name in ('input', 'source')}
        # Importing this module does not start its CLI, Isaac, or robot services.
        from playground_geometry import load
        from playground_plan import build, retime
        from curobo_evidence import backend_identity, stamp_plan
        from frozen_executor import validate_plan_evidence
        if profiler:
            profiler.enable()
        planner, observation, data = timed(report['stages'], 'load', lambda: load(
            args.input, source, args.native_code, args.assets))
        if args.geometry_backend != 'native':
            from curobo_whole_geometry import CuroboWholeGeometry
            callback = None
            if args.geometry_backend == 'curobo':
                from curobo_collision import CuroboWorldBroadphase
                engine = timed(report['stages'], 'gpu_world_init', lambda: CuroboWorldBroadphase(data))
                if engine.object_ids != tuple(item[0] for item in planner.geo.whole.world):
                    raise ValueError('GPU/native world ordering differs')
                def callback(spheres, clearance):
                    return engine.query(spheres[None, None], clearance=clearance)[0, 0].cpu().numpy()
            planner.geo.whole = CuroboWholeGeometry(planner.geo.whole, callback)
        if args.fast_cup_audit:
            from curobo_relative_geometry import install, initial as accelerated_initial
            install(planner.geo)
        identity = backend_identity(args.geometry_backend, relative_geometry=args.fast_cup_audit)
        report['backend_identity'] = identity
        if args.until != 'load':
            if args.initial_plan:
                initial = json.loads(args.initial_plan.read_text())
                if initial.get('scene_sha256') != observation.scene.digest():
                    raise ValueError('frozen route belongs to a different scene')
                report['frozen_route_sha256'] = hashlib.sha256(args.initial_plan.read_bytes()).hexdigest()
            else:
                solve = accelerated_initial if args.fast_cup_audit else lambda p,o,e: p.initial(o,e)
                initial = timed(report['stages'], 'initial', lambda: solve(
                    planner, observation, data['native_identity']['acceptance']['edge_margin_m']))
            write_new(args.output/'initial-plan.json', initial)
            report['route_passed'] = initial.get('passed') is True
            if not report['route_passed']:
                report['stages'][-1].update(passed=False, error=initial.get('error', 'no collision-free complete route'))
                raise ValueError(report['stages'][-1]['error'])
            if args.until == 'retime':
                base = timed(report['stages'], 'build_and_audit', lambda: build(
                    planner, observation, initial, source, data['native_identity'], speed_scale=None))
                plan = timed(report['stages'], 'retime', lambda: retime(base, source, args.speed_scale))
                validate_plan_evidence(base, source)
                validate_plan_evidence(plan, source)
                offline_base = stamp_plan(base, identity, offline=True)
                offline_plan = stamp_plan(plan, identity, offline=True)
                write_new(args.output/'base-plan.json', offline_base)
                write_new(args.output/'plan.json', offline_plan)
                report.update(input_ticks=len(base['ticks']), output_ticks=len(plan['ticks']),
                              timeline_digest=plan['timeline_digest'])
        report['passed'] = True
        if args.fast_cup_audit:
            report['cup_audit_stats'] = planner.geo._candidate_audit_stats
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        if profiler:
            profiler.disable()
            profiler.dump_stats(str(args.output/'planning.pstats'))
            report['top_cumulative_functions'] = profile_summary(profiler)
        report['total_wall_seconds'] = time.perf_counter()-started
        write_new(args.output/'benchmark.json', report)
    print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
    return 0 if report['passed'] else 1


def self_check():
    """Check timing/error accounting without importing any planning runtime."""
    rows, profiler = [], cProfile.Profile()
    profiler.enable()
    assert timed(rows, 'ok', lambda: 7) == 7
    try:
        timed(rows, 'failed', lambda: 1/0)
    except ZeroDivisionError:
        pass
    else:
        raise AssertionError('timed must preserve failures')
    profiler.disable()
    assert rows[0]['passed'] and not rows[1]['passed']
    assert 'ZeroDivisionError' in rows[1]['error']
    assert all(r['wall_seconds'] >= 0 and r['cpu_seconds'] >= 0 for r in rows)
    assert any(r['function'] == 'timed' and r['calls'] == 2 for r in profile_summary(profiler))
    print('offline benchmark self-check passed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-check', action='store_true')
    for name in ('input', 'source', 'native-code', 'assets', 'output'):
        parser.add_argument('--'+name, type=Path)
    parser.add_argument('--until', choices=('load', 'initial', 'retime'), default='retime')
    parser.add_argument('--profile', action='store_true', help='Save cProfile data and top 40 cumulative functions.')
    parser.add_argument('--geometry-backend', choices=('native', 'batch-cpu', 'curobo'), default='native')
    parser.add_argument('--initial-plan', type=Path, help='Benchmark final trajectory generation on a frozen route in the same scene.')
    parser.add_argument('--fast-cup-audit', action='store_true', help='Reuse same-call geometry and batch cup-pair broadphase.')
    parser.add_argument('--speed-scale', type=float, default=.3)
    args = parser.parse_args()
    if args.self_check:
        self_check()
        return 0
    if any(getattr(args, name) is None for name in ('input', 'source', 'native_code', 'assets', 'output')):
        parser.error('--input, --source, --native-code, --assets and --output are required')
    if not .3 <= args.speed_scale <= 1.7:
        parser.error('--speed-scale must be finite and between 0.3 and 1.7')
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
