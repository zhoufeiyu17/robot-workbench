"""Wall-clock estimates; receipts alone establish completed stages."""
import json
from statistics import median

# Measured 2026-09-23: resident detector + close-hold raw run (stitched baseline).
BASELINE = {'preflight-simulated-real': 3, 'preflight-dsw-scene': 4,
            'initialize': 1, 'shared-storage': 1, 'observe': 73, 'observe-verify': 1,
            'detect': 17, 'bridge': 1, 'geometry': 47, 'geometry-verify': 1,
            'place': 166, 'export': 2, 'replay': 383, 'export-playback': 8,
            'verify-results': 3}
INITIAL_ESTIMATES = {'motiongen-scene': 60, 'motiongen-plan': 30, 'motiongen-publish': 1}
LABELS = {'preflight-simulated-real': '源端环境预检', 'preflight-dsw-scene': '仿真环境预检',
          'physical-capture-import': '载入已封存的真机观测', 'physical-detection-bind': '核验检测输入来源',
          'initialize': '准备运行目录', 'shared-storage': '检查共享存储',
          'observe': '采集三路相机观测', 'observe-verify': '校验观测', 'detect': '检测杯子与咖啡机',
          'bridge': '构造检测场景', 'geometry': '生成场景几何', 'geometry-verify': '校验几何',
          'place': '规划抓取与放置轨迹', 'export': '导出冻结轨迹', 'replay': '双侧执行与录制（并行）',
          'motiongen-scene': '构建 MotionGen 场景与碰撞模型',
          'motiongen-plan': 'MotionGen GPU 路径搜索与轨迹生成',
          'motiongen-publish': '校验并冻结同一份仿真轨迹',
          'export-playback': '生成回放视频', 'verify-results': '校验结果与视频',
          'diagnostic-input': '准备诊断轨迹', 'reuse-frozen-detection': '读取冻结检测场景'}


def groups(run):
    result = []
    for stage in run.get('stages', []):
        key = stage.get('group') if stage.get('group') == 'replay' else stage['name']
        if result and key == 'replay' and result[-1][0] == key:
            result[-1][1].append(stage['name'])
        else:
            result.append((key, [stage['name']]))
    return result + [('verify-results', ['verify-results'])]


def read(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def durations(run):
    receipts = {r['stage']: r for r in run.get('receipts', [])}
    values = {}
    for key, names in groups(run):
        rows = [receipts[n] for n in names if n in receipts]
        if len(rows) == len(names) and all(r.get('exit_code') == 0 and 'finished_unix_s' in r for r in rows):
            values[key] = max(r['finished_unix_s'] for r in rows) - min(r['started_unix_s'] for r in rows)
    return values


def estimates(output, run):
    samples = {}
    # ponytail: last 10 sibling jobs, sufficient for this single-user workbench.
    paths = sorted(output.parent.parent.glob('*/result/run.json'), key=lambda p: p.stat().st_mtime, reverse=True)
    for path in [p for p in paths if p != output / 'run.json'][:10]:
        old = read(path)
        if old.get('started_unix_s', float('inf')) >= run.get('started_unix_s', 0):
            continue
        config = run.get('config', {})
        if any(old.get('config', {}).get(k) != config.get(k) for k in ('record_raw', 'motion_profile', 'force_diagnostic', 'planning_backend')):
            continue
        # Cold-start detector history is not representative of the resident service.
        for key, value in durations(old).items():
            if key == 'detect' and bool(old.get('config', {}).get('detector_service')) != bool(config.get('detector_service')):
                continue
            if value > 0:
                samples.setdefault(key, []).append(value)
    return {key: {'seconds': median(samples[key]) if key in samples else BASELINE.get(key, INITIAL_ESTIMATES.get(key, 1)),
                  'basis': f'近期 {len(samples[key])} 次中位数' if key in samples else
                  ('历史分段实测' if key in BASELINE else '初次粗估，待本后端实测' if key in INITIAL_ESTIMATES else '粗估')}
            for key, _ in groups(run)}


def snapshot(output, job_status, now):
    run, progress = read(output / 'run.json'), read(output / 'progress.json')
    if not run:
        return {}
    expected = run.get('progress_estimates') or estimates(output, run)
    receipts = {r['stage']: r for r in run.get('receipts', [])}
    active = progress.get('stage')
    terminal = job_status != 'running' or run.get('status') != 'running'
    final = read(output / 'final.json') if terminal else {}
    end = final.get('finished_unix_s', progress.get('updated_unix_s', now)) if terminal else now
    rows = []
    for key, names in groups(run):
        found = [receipts[n] for n in names if n in receipts]
        complete = len(found) == len(names) and all('finished_unix_s' in r for r in found)
        status = ('failed' if any(r.get('exit_code') != 0 for r in found) else 'completed') if complete else 'pending'
        started = min((r['started_unix_s'] for r in found), default=None)
        finished = max((r.get('finished_unix_s', end) for r in found), default=None) if complete else None
        if not complete and (found or key == active or active in names):
            started = started or progress.get('updated_unix_s', now)
            status = ('failed' if found and any(r.get('exit_code') != 0 for r in found) else 'stopped') if terminal else 'running'
        # Older successful runs did not write a separate local-verification receipt.
        if key == 'verify-results' and not found and final.get('status') in ('replay-complete', 'diagnostic-complete') and not final.get('error'):
            started = max((r.get('finished_unix_s', end) for r in receipts.values()), default=end)
            finished, status = end, 'completed'
        actual = max(0, (finished or end) - started) if started is not None else None
        rows.append({'key': key, 'label': LABELS.get(key, '取回 '+key[6:] if key.startswith('fetch-') else key),
                     'status': status, 'elapsed_s': actual, 'estimate_s': expected[key]['seconds'],
                     'basis': expected[key]['basis']})
    return {'stages': rows, 'elapsed_s': max(0, end-run['started_unix_s']),
            'status': 'failed' if final.get('error') or job_status == 'failed' else job_status,
            'pipeline_finished': run.get('status') != 'running', 'sampled_unix_s': now}
