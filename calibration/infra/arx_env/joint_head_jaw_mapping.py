"""Jaw mappings: legacy +2 mm default, opt-in A1 +3 mm reinterpretation.

Command and feedback knots are distinct. SDK zero equals actual zero by explicit
user convention; zero-to-first-measurement is interpolated, not ruler-validated.
"""
import math
import hashlib
import json
from pathlib import Path

COMMAND = {'left': (0., .0197, .0394, .0591, .0788), 'right': (0., .019325, .03865, .057975, .0773)}
FEEDBACK = {'left': (0., .020079090438953413, .03919807246630678, .05892929753873843, .07841933595704834),
            'right': (0., .019533262101218883, .03842884221425934, .057821904729424806, .07709738049499709)}
GAP_MM = {'left': (0., 23., 43., 63., 83.), 'right': (0., 22., 43., 63., 83.)}


_LEGACY = (COMMAND, FEEDBACK, GAP_MM, {'version': 'legacy-pad-correction-2mm'})
_ACTIVE = _LEGACY
FEEDBACK_MAX_MM = 89.


def configure_mapping(path):
    """Select a validated mapping at startup; invalid input leaves it unchanged."""
    global _ACTIVE
    if path is None:
        _ACTIVE = _LEGACY
        return mapping_identity()
    raw = Path(path).read_bytes()
    data = json.loads(raw)
    if isinstance(data,dict) and data.get('schema_version')=='native_gripper_mapping.v1':
        from robot_workbench.sdk_quiesce import load_gripper_calibration
        if data.get('robot_id')not in ('arx6','arx13'):raise ValueError('native jaw mapping requires its physical device identity')
        value=load_gripper_calibration(data.get('native_gripper_calibration'),robot_id=data.get('robot_id'))
        if value is None or set(data)!={'schema_version','robot_id','native_gripper_calibration'}:
            raise ValueError('exact native gripper mapping binding required')
        commands={side:(0.,value['sdk_width_m']) for side in ('left','right')}
        gaps={side:(0.,row['width_m']*1000.) for side,row in value['sides'].items()}
        identity=dict(version='native-gripper-endpoints-v1',robot_id=value['robot_id'],
            source_sha256=hashlib.sha256(raw).hexdigest(),native_calibration_id=value['identity'],
            feedback_margin_mm=3.,
            command_max_mm={side:row[-1] for side,row in gaps.items()})
        _ACTIVE=(commands,commands,gaps,identity)
        return mapping_identity()
    if not isinstance(data, dict) or data.get('schema_version') != 'jaw_mapping_audit.v1':
        raise ValueError('unsupported jaw mapping schema')
    rows = data.get('reinterpreted_mapping')
    if not isinstance(rows, dict) or set(rows) != {'left', 'right'}:
        raise ValueError('jaw mapping requires left and right')
    tables = [{}, {}, {}]
    for side, row in rows.items():
        if not isinstance(row, dict) or row.get('candidate_use_allowed') is not True:
            raise ValueError('jaw mapping candidate must be explicitly allowed')
        for table, name in zip(tables, ('command_sdk_knots', 'feedback_sdk_knots', 'metal_knots_mm')):
            values = row.get(name)
            if (not isinstance(values, list) or len(values) < 2
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)
                    or values[0] != 0 or any(a >= b for a, b in zip(values, values[1:]))):
                raise ValueError(f'{side} {name} requires finite increasing knots starting at zero')
            table[side] = tuple(values)
        if len({len(table[side]) for table in tables}) != 1 or tables[2][side][-1] < 83:
            raise ValueError(f'{side} mapping columns must align and cover 0..83 mm')
    identity = {'version': 'a1-pad-correction-3mm', 'source_sha256': hashlib.sha256(raw).hexdigest()}
    _ACTIVE = (*tables, identity)
    return mapping_identity()


def mapping_identity():
    return dict(_ACTIVE[3])


def command_max_mm(side):
    if side not in ('left','right'):raise ValueError('left or right jaw required')
    return _ACTIVE[3].get('command_max_mm',{}).get(side,83.)


def feedback_max_mm(side):
    return command_max_mm(side)+_ACTIVE[3].get('feedback_margin_mm',6.)


def feedback_min_mm(side):
    if side not in ('left','right'):raise ValueError('left or right jaw required')
    return -_ACTIVE[3].get('feedback_margin_mm',2.)


def interpolate(value, xs, ys, *, field='jaw mapping input'):
    name=type(value).__module__+'.'+type(value).__name__
    if type(value) not in (float, int):
        raise ValueError(f'{field}: unsupported type {name}; expected Python float/int')
    if not math.isfinite(value):
        raise ValueError(f'{field}: non-finite {name} value {value!r}')
    i = next((i for i in range(len(xs)-1) if value <= xs[i+1]), len(xs)-2)
    return ys[i] + (value-xs[i]) * (ys[i+1]-ys[i]) / (xs[i+1]-xs[i])


def command_sdk(side, gap_mm):
    commands, _, gaps, _ = _ACTIVE
    if side not in gaps or type(gap_mm) not in (float, int) or not math.isfinite(gap_mm):
        raise ValueError('finite left/right actual jaw gap required')
    if not 0 <= gap_mm <= command_max_mm(side):
        raise ValueError(f'{side} jaw command range is 0–{command_max_mm(side):g} mm')
    return interpolate(gap_mm, gaps[side], commands[side])


def feedback_gaps(raw):
    _, feedback, gaps, _ = _ACTIVE
    return [interpolate(float(raw[i]), feedback[side], gaps[side]) for side, i in [('left', 6), ('right', 13)]]



def command_gap_mm(side, sdk):
    commands, _, gaps, _ = _ACTIVE
    return interpolate(sdk, commands[side], gaps[side],field=f'{side} jaw command SDK value')


def feedback_sdk(side, gap_mm):
    _, feedback, gaps, _ = _ACTIVE
    return interpolate(gap_mm, gaps[side], feedback[side],field=f'{side} jaw feedback gap_mm')


def command_gap_slope(side):
    commands, _, gaps, _ = _ACTIVE
    return max((b-a)/(d-c) for a,b,c,d in zip(gaps[side], gaps[side][1:], commands[side], commands[side][1:]))


def hold_command_sdk(side, gap_mm):
    """Accept measured endpoint error, without commanding beyond the calibrated range."""
    if type(gap_mm) not in (float, int) or not math.isfinite(gap_mm) or not feedback_min_mm(side)-1e-9 <= gap_mm <= feedback_max_mm(side)+1e-9:
        raise ValueError(f'{side} measured jaw gap {gap_mm} mm outside feedback tolerance {feedback_min_mm(side):g}–{feedback_max_mm(side):g} mm')
    return command_sdk(side, min(command_max_mm(side), max(0., gap_mm)))


def feedback_state(raw):
    _, feedback, gaps, identity = _ACTIVE
    sdk = [float(raw[6]), float(raw[13])]
    return dict(gap_mm=[interpolate(v, feedback[s], gaps[s]) for s,v in zip(('left','right'),sdk)], sdk=sdk,
                measured_range_mm=[[gaps[s][1], gaps[s][-1]] for s in ('left','right')],
                feedback_range_mm=[[feedback_min_mm(side),feedback_max_mm(side)] for side in ('left','right')],
                control_range_mm=[[0.,command_max_mm(side)] for side in ('left','right')],zero_basis='user convention: SDK 0 = actual 0 mm',
                within_measured_range=[feedback[s][1] <= v <= feedback[s][-1] for s,v in zip(('left','right'),sdk)],
                mapping=identity['version'], mapping_identity=dict(identity))


def measured_jaw_gaps(value):
    """Measured metal gaps: retain endpoint error; never use for commands."""
    import numpy as np
    gaps = np.asarray(value, float)
    if (gaps.shape != (2,) or not np.isfinite(gaps).all()
            or np.any(gaps < np.asarray([feedback_min_mm(s)for s in ('left','right')])-1e-9)
            or np.any(gaps > np.asarray([feedback_max_mm(s)for s in ('left','right')])+1e-9)):
        raise ValueError('left/right measured metal gaps exceed calibrated feedback tolerance')
    return gaps
