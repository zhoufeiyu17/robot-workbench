"""CPU-only receipt/timing check: parallel, failure, history, overrun."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from runtime_progress import snapshot, durations, estimates


def main():
    with TemporaryDirectory() as temp:
        output = Path(temp) / 'jobs/new/result'
        output.mkdir(parents=True)
        run = {'status': 'running', 'started_unix_s': 100, 'config': {'record_raw': True},
               'stages': [{'name': 'observe', 'group': None},
                          {'name': 'replay-source', 'group': 'replay'},
                          {'name': 'replay-target', 'group': 'replay'}],
               'receipts': [{'stage': 'observe', 'started_unix_s': 100, 'finished_unix_s': 170, 'exit_code': 0}]}
        def save(name, value):
            (output / name).write_text(json.dumps(value))
        save('run.json', run)
        save('progress.json', {'stage': 'replay', 'updated_unix_s': 170})
        view = snapshot(output, 'running', 600)
        assert view['stages'][0]['elapsed_s'] == 70
        assert view['stages'][1]['elapsed_s'] == 430
        assert view['stages'][1]['status'] == 'running'  # overrun is not completion
        run['receipts'] += [dict(stage='replay-source', started_unix_s=170, finished_unix_s=550, exit_code=0),
                            dict(stage='replay-target', started_unix_s=171, finished_unix_s=560, exit_code=0)]
        assert durations(run)['replay'] == 390  # union wall time, not sum
        old = output.parent.parent / 'old/result/run.json'
        old.parent.mkdir(parents=True)
        old.write_text(json.dumps({**run, 'started_unix_s': 50}))
        assert estimates(output, run)['replay']['seconds'] == 390
        run['receipts'][-1]['exit_code'] = 1
        run['status'] = 'failed'
        save('run.json', run)
        save('final.json', {'finished_unix_s': 561, 'error': 'replay failed'})
        failed = snapshot(output, 'failed', 1000)
        assert failed['elapsed_s'] == 461 and failed['stages'][1]['status'] == 'failed'
        assert failed['stages'][-1]['status'] == 'pending'
        assert snapshot(output, 'failed', 2000)['elapsed_s'] == failed['elapsed_s']
        # Legacy failures omit the finish timestamp; freeze at the final receipt.
        del run['receipts'][-1]['finished_unix_s']
        save('run.json', run)
        assert snapshot(output, 'failed', 2000)['stages'][1]['status'] == 'failed'
        # Legacy completed runs have no explicit local-verification receipt.
        run['receipts'][-1].update(exit_code=0, finished_unix_s=560)
        run['status'] = 'replay-complete'
        save('run.json', run)
        save('final.json', {'status': 'replay-complete', 'finished_unix_s': 561})
        complete = snapshot(output, 'completed', 2000)
        assert all(row['status'] == 'completed' for row in complete['stages'])
        assert complete['stages'][-1]['elapsed_s'] == 1
        assert complete['elapsed_s'] == 461
    print('runtime progress checks passed')


if __name__ == '__main__':
    main()
