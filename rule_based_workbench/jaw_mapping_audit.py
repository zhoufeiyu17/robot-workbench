"""Offline audit of legacy jaw knots against the 83 mm metal / 80 mm pad baseline.

This does not change a controller mapping or manufacture ruler measurements.
An explicit operator confirmation can reinterpret the old pad-surface readings;
this remains arithmetic reinterpretation, never a new physical measurement.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path


def read_mapping(path):
    """Read literal knots without importing or executing calibration code."""
    values = {}
    for node in ast.parse(Path(path).read_text()).body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id in ('COMMAND', 'FEEDBACK', 'GAP_MM'):
                    values[target.id] = ast.literal_eval(node.value)
    if set(values) != {'COMMAND', 'FEEDBACK', 'GAP_MM'}:
        raise ValueError('legacy mapping constants missing')
    return values


def interpolate(value, xs, ys):
    if not xs[0] <= value <= xs[-1]:
        raise ValueError('interpolation would extrapolate')
    for i in range(len(xs)-1):
        if value <= xs[i+1]:
            return ys[i] + (value-xs[i])/(xs[i+1]-xs[i])*(ys[i+1]-ys[i])
    raise ValueError('empty interpolation')


def audit(mapping, home_measurement, *, operator_pad_basis_confirmation=None):
    import math
    for group in ('COMMAND', 'FEEDBACK', 'GAP_MM'):
        for side in ('left', 'right'):
            knots = mapping[group][side]
            if (len(knots) != 5 or knots[0] != 0
                    or any(type(x) not in (int, float) or not math.isfinite(x) for x in knots)
                    or any(b <= a for a,b in zip(knots, knots[1:]))):
                raise ValueError('five increasing finite knots including conventional zero required')
    if any(mapping['GAP_MM'][side][-1] != 83 for side in ('left', 'right')):
        raise ValueError('unexpected legacy endpoint; repeat evidence audit')
    result = dict(schema_version='jaw_mapping_audit.v1',
        status='requires-four-point-ruler-basis-confirmation',
        controller_mapping_changed=False, physical_revalidation_passed=False,
        baseline=dict(metal_inner_gap_max_mm=83., pad_thickness_each_mm=1.5,
                      pad_inner_gap_at_metal_max_mm=80., source='current user baseline; not a new measurement'),
        legacy=dict(mapping=mapping, correction_mm=2.,
                    four_point_raw_measurements_available=False,
                    ruler_basis='unknown: pad-vs-metal basis not present in four-point source'),
        home_measurement=home_measurement,
        recommendation='Keep the old mapping marked pending audit; do not silently apply +3 mm to hardware.',
        limitations=[
            'The 2026-09-17 home reading reports 83 mm bare / 81 mm padded and 1 mm per pad; it does not establish the 2026-09-20 four-point ruler basis.',
            'SDK zero = metal zero is a user convention, not a ruler measurement. Correction is not added to this synthetic zero.',
            'Zero to first measured knot is interpolation, not ruler-validated. Compressed-pad contact near closure is not calibrated.',
            'Inferred pre-correction numbers are legacy GAP_MM minus 2, not recovered original ruler logs.',
            'A conditional 84 mm highest corrected knot does not require 84 mm operation: cap at 83 mm and interpolate SDK below the old maximum.',
        ], conditional_pad_basis_candidate={})
    for side in ('left', 'right'):
        old = list(mapping['GAP_MM'][side])
        inferred_raw = [g-2 for g in old[1:]]
        metal = [0.] + [g+3 for g in inferred_raw]
        clipped = [g for g in metal if g < 83.] + [83.]
        command = list(mapping['COMMAND'][side]); feedback = list(mapping['FEEDBACK'][side])
        result['conditional_pad_basis_candidate'][side] = dict(
            activation_allowed=False, condition='Only if original four-point ruler readings measured pad-to-pad clearance with the applicable pad geometry.',
            inferred_uncorrected_knots_mm=inferred_raw, inferred_numbers_are_original_measurements=False,
            metal_knots_mm=metal, command_sdk_knots=command, feedback_sdk_knots=feedback,
            control_metal_knots_mm=clipped,
            control_command_sdk_knots=[interpolate(g,metal,command) for g in clipped],
            control_feedback_sdk_knots=[interpolate(g,metal,feedback) for g in clipped],
            command_sdk_at_83mm=interpolate(83.,metal,command),
            feedback_sdk_at_83mm=interpolate(83.,metal,feedback),
            cap_is_interpolated_not_measured=True)
    if operator_pad_basis_confirmation is not None:
        if not isinstance(operator_pad_basis_confirmation, str) or not operator_pad_basis_confirmation.strip():
            raise ValueError('explicit operator confirmation text required')
        result.update(status='operator-confirmed-reinterpretation',
                      operator_pad_basis_confirmation=operator_pad_basis_confirmation,
                      deployment_status='not-deployed',
                      recommendation='Use the reinterpreted +3 mm mapping in a new candidate; cap metal commands at 83 mm. Deployment and physical validation remain separate.')
        result['legacy']['ruler_basis']='operator confirmed original four-point readings were rubber-surface clearance'
        rows=result.pop('conditional_pad_basis_candidate')
        result['reinterpreted_mapping']=rows
        for row in rows.values():
            row.pop('activation_allowed')
            row.pop('condition')
            row['candidate_use_allowed']=True
            row['basis']='operator-confirmed old rubber-surface readings, plus current 1.5 mm per side baseline'
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mapping',required=True)
    parser.add_argument('--home-measurement',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--operator-pad-basis-confirmation')
    args=parser.parse_args()
    result=audit(read_mapping(args.mapping),json.loads(Path(args.home_measurement).read_text()),
                 operator_pad_basis_confirmation=args.operator_pad_basis_confirmation)
    result['evidence']=[dict(path=str(Path(path).resolve()),sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())
                        for path in (args.mapping,args.home_measurement)]
    output=Path(args.output); output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')
    print(output)


if __name__ == '__main__':
    main()
