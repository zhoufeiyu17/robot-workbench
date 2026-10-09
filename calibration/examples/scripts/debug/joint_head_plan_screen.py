"""Screen a planned queue for swept-path problems the plan itself never checked.

The head half replays the frozen standard seed. `_plan_from_seed` re-validates that
every pose keeps the marker detector-visible under the current geometry, but unlike
the wrist planner it never runs `path_check` along the segments, and
`queue_is_plan_checked` then exempts the whole queue from the execution-time check
because it is the queue the planner produced. So a head batch moves on the strength
of a seed that was reviewed once, by eye, against whatever mesh existed then -
re-planning after a mesh change is exactly when that assumption has to be re-tested.

The two `intersection solver uncertain` segments the standard seed has always had are
reported separately: they come from the collision LP failing to decide on a degenerate
hull, not from a geometric overlap. Treating them as collisions would refuse the
standard trajectory, which is why the planner-produced queue is exempt in the first
place; a *definite* pair is the thing worth stopping for.
"""
import argparse
import json
import sys
from pathlib import Path

UNCERTAIN = 'intersection solver uncertain'


def classify(pairs):
    """Split a path_check collision list into definite overlaps and undecided pairs."""
    definite = [list(pair) for pair in pairs if UNCERTAIN not in pair]
    uncertain = [list(pair) for pair in pairs if UNCERTAIN in pair]
    return definite, uncertain


def segment_detail(result):
    if result.get('ok'):
        return '通过'
    pairs = list(result.get('collisions') or [])
    if not pairs:
        return '超出关节限位'
    definite, uncertain = classify(pairs)
    parts = ['/'.join(str(part) for part in pair if part != UNCERTAIN) for pair in definite + uncertain]
    if definite:
        return '模型判定干涉/自碰撞：' + '；'.join(parts)
    return '求解器不确定（保守判定，非实测干涉）：' + '；'.join(parts)


def screen(geometry, rows, start, gaps=(83., 83.), names=None):
    """Run the planner's own path_check over `rows`, starting from `start`.

    Returns one verdict per waypoint; `blocking` marks the verdicts that should stop
    a re-run (definite overlaps or limit violations), `uncertain` the conservative
    ones that the standard seed is expected to carry.
    """
    checked = geometry.with_gaps(*gaps)
    previous = geometry._q(start)
    verdicts = []
    for index, row in enumerate(rows):
        target = geometry._q(row['q'])
        result = checked.path_check(previous, target)
        pairs = list(result.get('collisions') or [])
        definite, uncertain = classify(pairs)
        ok = bool(result.get('ok'))
        verdicts.append({'index': index,
                         'name': (names or {}).get(index) or row.get('name') or row.get('kind') or '?',
                         'ok': ok,
                         'fraction': result.get('fraction'),
                         'detail': segment_detail(result),
                         'definite': definite,
                         'uncertain': uncertain,
                         # A failed segment blocks unless the only thing the collision
                         # LP said was "uncertain"; a failure with no pairs at all is a
                         # joint-limit violation and blocks too.
                         'blocking': (not ok) and (bool(definite) or not pairs)})
        previous = target
    return verdicts


def summarise(verdicts):
    blocking = [row for row in verdicts if row['blocking']]
    uncertain = [row for row in verdicts if row['uncertain'] and not row['blocking']]
    return {'segments': len(verdicts), 'blocking': len(blocking), 'uncertain': len(uncertain),
            'blocking_segments': [row['index'] for row in blocking],
            'uncertain_segments': [row['index'] for row in uncertain]}


def main(argv=None):
    parser = argparse.ArgumentParser(description='Screen a workbench queue with path_check (read-only)')
    parser.add_argument('--config', required=True, type=Path, help='workbench config.json used by the run')
    parser.add_argument('--state', type=Path, help='workbench-state.json holding the queue (default: <run>/data/workbench-state.json)')
    parser.add_argument('--start-q', help='JSON list of 12 joint angles to start from; default: the first waypoint')
    parser.add_argument('--json', action='store_true', help='print the verdicts as JSON')
    args = parser.parse_args(argv)

    run = args.config.resolve().parent
    state_path = args.state or run / 'data' / 'workbench-state.json'
    # The geometry module lives in the repo's `infra/`, but a deployed run keeps its
    # config outside the checkout (`<root>/<run>/config.json` next to `<root>/code/`),
    # so try the checkout first and then the layouts a deployment uses.
    repo = Path(__file__).resolve().parents[3]
    for candidate in (repo / 'infra', args.config.resolve().parent.parent / 'infra',
                      args.config.resolve().parent.parent / 'code' / 'infra'):
        if candidate.is_dir():
            sys.path.insert(0, str(candidate))
    from arx_env.joint_head_workbench_geometry import Geometry

    geometry = Geometry(json.loads(args.config.read_text()))
    state = json.loads(state_path.read_text())
    rows = state.get('queue') or []
    if not rows:
        print('queue is empty; nothing to screen')
        return 0
    start = json.loads(args.start_q) if args.start_q else rows[0]['q']
    verdicts = screen(geometry, rows, start)
    summary = summarise(verdicts)
    if args.json:
        print(json.dumps({'summary': summary, 'verdicts': verdicts}, ensure_ascii=False, indent=2))
    else:
        for row in verdicts:
            if row['ok']:
                continue
            print('seg %2d (%s): %s at fraction %s' % (row['index'], row['name'], row['detail'],
                                                       'n/a' if row['fraction'] is None else '%.2f' % row['fraction']))
        print('segments=%d blocking=%d uncertain=%d' % (summary['segments'], summary['blocking'], summary['uncertain']))
        if not args.start_q:
            print('note: started from the first waypoint; pass --start-q with the robot current pose for a real screen')
    return 1 if summary['blocking'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
