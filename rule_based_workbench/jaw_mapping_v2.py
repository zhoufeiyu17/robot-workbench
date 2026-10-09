"""Candidate mapping: operator-confirmed old rubber-gap readings + 3 mm metal.

Each rubber layer is 1.5 mm in the current baseline. The old four-point SDK
command/feedback knots are unchanged; this is arithmetic reinterpretation, not
new ruler measurement or hardware acceptance. Not deployed to calibration.
SDK zero = metal zero remains the user's convention, not a +3 mm data point.
"""
import math

COMMAND = {'left': (0., .0197, .0394, .0591, .0788), 'right': (0., .019325, .03865, .057975, .0773)}
FEEDBACK = {'left': (0., .020079090438953413, .03919807246630678, .05892929753873843, .07841933595704834),
            'right': (0., .019533262101218883, .03842884221425934, .057821904729424806, .07709738049499709)}
GAP_MM = {'left': (0., 24., 44., 64., 84.), 'right': (0., 23., 44., 64., 84.)}
CONTROL_MAX_MM = 83.
MAPPING_ID = 'operator-pad-basis-1p5mm-per-side-v2'


def interpolate(value, xs, ys):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('finite jaw value required')
    index=next((i for i in range(len(xs)-1) if value <= xs[i+1]),len(xs)-2)
    return ys[index]+(value-xs[index])*(ys[index+1]-ys[index])/(xs[index+1]-xs[index])


def command_sdk(side, gap_mm):
    if (side not in GAP_MM or type(gap_mm) not in (int, float)
            or not math.isfinite(gap_mm) or not 0 <= gap_mm <= CONTROL_MAX_MM):
        raise ValueError('finite left/right metal command gap in 0..83 mm required')
    return interpolate(gap_mm,GAP_MM[side],COMMAND[side])


def feedback_gaps(raw):
    if len(raw)!=14:
        raise ValueError('14 SDK feedback values required')
    values=[interpolate(float(raw[index]),FEEDBACK[side],GAP_MM[side])
            for side,index in (('left',6),('right',13))]
    if any(not -1 <= value <= 84 for value in values):
        raise ValueError('measured metal gaps outside -1..84 mm feedback tolerance')
    return values


def hold_command_sdk(side, gap_mm):
    if type(gap_mm) not in (int,float) or not math.isfinite(gap_mm) or not -1 <= gap_mm <= 84:
        raise ValueError('measured metal gap outside -1..84 mm feedback tolerance')
    return command_sdk(side,min(CONTROL_MAX_MM,max(0.,gap_mm)))


def feedback_state(raw):
    sdk=[float(raw[6]),float(raw[13])]
    return dict(gap_mm=feedback_gaps(raw),sdk=sdk,
                measured_range_mm=[[GAP_MM[s][1],84.] for s in ('left','right')],
                control_range_mm=[[0.,83.],[0.,83.]],
                zero_basis='user convention: SDK 0 = metal 0 mm; low-opening interpolation not ruler-validated',
                within_measured_range=[FEEDBACK[s][1] <= value <= FEEDBACK[s][-1]
                                       for s,value in zip(('left','right'),sdk)],
                mapping=MAPPING_ID,physical_revalidation_passed=False,
                mapping_basis='operator-confirmed reinterpretation; +3 mm total rubber correction')


def measured_jaw_gaps(value):
    import numpy as np
    gaps=np.asarray(value,float)
    if gaps.shape!=(2,) or not np.isfinite(gaps).all() or np.any(gaps < -1) or np.any(gaps > 84):
        raise ValueError('left/right measured metal gaps must be finite, -1..84 mm')
    return gaps
