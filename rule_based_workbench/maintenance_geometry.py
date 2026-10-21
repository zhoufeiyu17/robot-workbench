"""Explicit maintenance-only geometry scope and initial support audit; no task facts."""
import hashlib
import json
import math
import os
from pathlib import Path

from integration import _digest

PURPOSE = 'physical_cup_maintenance.v1'
STATUS = 'maintenance-geometry-inputs-ready'


def validate_request(path, source, environment):
    """Check sealed inputs before importing Kit or allocating a GPU context."""
    from physical_cup_maintenance import validate_parent_execution
    value = json.loads(Path(path).read_text())
    if (value.get('schema') != 'maintenance_geometry_request.v1' or value.get('purpose') != PURPOSE
            or value.get('environment') != environment or value.get('source_digest') != _digest(source)
            or value.get('session') != os.environ.get('ROBOT_WORKBENCH_SESSION_ID')
            or not value.get('session') or value.get('support') not in ('table', 'tray')):
        raise ValueError('explicit sealed maintenance geometry request required')
    acceptance = value.get('error_acceptance')
    expected_acceptance = {'schema': 'maintenance_orientation_acceptance.v1', 'session': '01a13e4e-70dc-7513-9467-d34b9a7a99a3', 'source_digest': 'af0d293cd6c22d10a56ccdcf6a81bf57d71fe7514755b7b3c48cbf354995e57a', 'original_limit_rad': 0.01, 'observed_error_rad': 0.012156242583612913, 'max_error_rad': 0.012157, 'basis_sha256': 'ce9c53553ae4dd52a68f29a5eb35f0983d2a733b9815e26772fa71577c9f0047', 'human_instruction': '我们接受现在的误差，你来帮忙继续完成两轮真机数采、infer 链路测试吧；infer 链路可以用 2s infer 结果调试', 'scope': 'maintenance initialization requested-vs-realized orientation only; no contact/collision/control exception', 'recorded_at_utc': '2026-10-18T13:51:12.783812+00:00', 'owner_pid': 3120071, 'runtime_nonce': '1505687f8740925f52766715be00472c530fee98c269e869', 'command_epoch': 1, 'applies_to': {'drift': False, 'speed': False, 'closing': False, 'path': False, 'control': False}}
    if acceptance is not None and (environment != 'physical' or value['support'] != 'tray'
            or _digest(source) != expected_acceptance['source_digest'] or acceptance != expected_acceptance):
        raise ValueError('maintenance orientation acceptance differs from the human-approved frozen case')
    files = value.get('files', {})
    required = {'parent_source', 'parent_plan', 'parent_execution', 'asset_pins', 'asset_receipt', 'object_mesh'}
    if set(files) != required:
        raise ValueError('complete maintenance parent and asset pins required')
    for row in files.values():
        p = Path(row['path'])
        if not p.is_absolute() or p.is_symlink() or hashlib.sha256(p.read_bytes()).hexdigest() != row['sha256']:
            raise ValueError('maintenance sealed file changed')
    parent, plan, execution = [json.loads(Path(files[k]['path']).read_text())
                               for k in ('parent_source', 'parent_plan', 'parent_execution')]
    validate_parent_execution(plan, parent, execution)
    if environment == 'physical':
        if (source.get('schema_version') != 'physical_collection_source.v1'
                or source.get('simulation_only') is not False or source.get('training_observation') is not False
                or any(source.get(k) != parent.get(k) for k in ('robot_id', 'camera_serials', 'calibration_profile', 'grasp_command'))):
            raise ValueError('physical maintenance current identity differs from its parent')
    elif source.get('simulation_only') is not True:
        raise ValueError('fake maintenance must retain its simulation identity')
    pins = json.loads(Path(files['asset_pins']['path']).read_text())
    if not pins or any(hashlib.sha256(Path(p).read_bytes()).hexdigest() != sha for p, sha in pins.items()):
        raise ValueError('maintenance asset bytes differ from pins')
    return dict(purpose=PURPOSE, session=value['session'], environment=environment,
                source_digest=_digest(source), parent_source_digest=_digest(parent),
                parent_timeline_digest=plan['timeline_digest'], parent_execution_digest=_digest(execution),
                asset_pins_sha256=files['asset_pins']['sha256'],
                asset_receipt_sha256=files['asset_receipt']['sha256'],
                object_mesh_sha256=files['object_mesh']['sha256'], support=value['support'],
                task_characterization='not_run', **({'error_acceptance': acceptance, 'error_acceptance_digest': _digest(acceptance)} if acceptance else {}))


def validate_export(path, source, maintenance_source=None):
    path = Path(path)
    result = json.loads(path.with_name('result.json').read_text())
    data = json.loads(path.read_text())
    scope = data.get('export_scope')
    if maintenance_source is None:
        if result.get('status') != 'geometry-inputs-ready' or scope is not None:
            raise ValueError('ordinary planning requires ordinary task geometry; maintenance scope rejected')
        return data
    from physical_cup_maintenance import current_geometry_binding, validate_current_geometry_source
    binding = current_geometry_binding(maintenance_source)
    validate_current_geometry_source(maintenance_source, source)
    context = maintenance_source['maintenance_context']
    expected = dict(purpose=PURPOSE, environment='physical', source_digest=_digest(source),
                    parent_source_digest=maintenance_source['parent_source_digest'],
                    parent_timeline_digest=context['parent_timeline_digest'],
                    parent_execution_digest=context['parent_execution_digest'],
                    task_characterization='not_run', support='tray',
                    **{k:binding[k] for k in ('asset_pins_sha256', 'asset_receipt_sha256', 'object_mesh_sha256')})
    if (result.get('status') != STATUS or not isinstance(scope, dict)
            or any(scope.get(k) != v for k, v in expected.items())
            or data.get('export_scope_digest') != _digest(scope)
            or hashlib.sha256(path.read_bytes()).hexdigest() != binding['export_sha256']
            or scope.get('initial_support_audit', {}).get('passed') is not True
            or not scope.get('native_runtime_sha256') or not scope.get('session')):
        raise ValueError('maintenance export qualification or bound scope differs')
    return data


def audit_initial_support(runtime, sample, scene, contacts, *, support, require_support=False):
    """Original support/contact limits, without grasp history or evaluator.observe."""
    evaluator = runtime.evaluator
    a = runtime.spec.acceptance
    pose = sample.world_from_cup
    speed = math.sqrt(sum(v*v for v in sample.cup_velocity))
    runtime.maintenance_initial_audit = dict(passed=False, physics_step=sample.physics_step,
        require_support=require_support, cup_velocity_m_s=list(sample.cup_velocity),cup_speed_m_s=speed,
        cup_pose=dict(position=list(pose.position),orientation_wxyz=list(pose.orientation_wxyz)),
        speed_limit_m_s=a.landing_speed_m_s,speed_limit_source='landing_speed_m_s',
        speed_check_additional=True,speed_check_stage='final_snapshot',task_characterization='not_run')
    if not hasattr(runtime,'maintenance_audit_trace'):runtime.maintenance_audit_trace=[]
    runtime.maintenance_audit_trace.append(runtime.maintenance_initial_audit)
    if (scene.instance.episode_id != sample.episode_id or scene.observation_id != str(sample.physics_step)
            or scene.profile_sha256 != evaluator.machine_profile_sha256
            or (not require_support and sample.physics_step != getattr(runtime, 'maintenance_physics_step', 0) + 1)):
        raise ValueError('maintenance stale physics or geometry')
    if not require_support: runtime.maintenance_physics_step = sample.physics_step
    if support == 'tray':
        support_pose = scene.world_from_support
        polygon_world = scene.support_polygon_world_m
        support_shape = scene.support_shape_path
    elif support == 'table':
        from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
        layout = runtime.layout
        position = list(layout.worktable.position)
        position[2] += layout.worktable_size_m[2] / 2
        support_pose = RigidTransform(tuple(position))
        x, y = [v / 2 for v in layout.worktable_size_m[:2]]
        polygon_world = [support_pose.transform_point(p) for p in ((-x,-y,0), (x,-y,0), (x,y,0), (-x,y,0))]
        support_shape = evaluator.table_shape
    else:
        raise ValueError('explicit maintenance support required')
    inverse = support_pose.inverse()
    bottom = inverse.transform_point(pose.position)
    up = inverse.rotate(pose.rotate((0, 0, 1)))
    polygon = [inverse.transform_point(p) for p in polygon_world]
    radius = evaluator.cup.bottom_radius_m + a.edge_margin_m
    inside = len(polygon) >= 3 and all(
        (q[0]-p[0])*(bottom[1]-p[1])-(q[1]-p[1])*(bottom[0]-p[0]) >= radius*math.hypot(q[0]-p[0],q[1]-p[1])
        for p,q in zip(polygon, polygon[1:]+polygon[:1]))
    clearance = evaluator._lowest_z(inverse.compose(pose))
    if not all(math.isfinite(v) for v in [*bottom, *up, clearance, *sample.cup_velocity, *(v for p in polygon for v in p)]):
        raise ValueError('maintenance support geometry incomplete')
    errors = []
    if not inside: errors.append('maintenance_support_edge')
    if up[2] < math.cos(a.tilt_rad): errors.append('maintenance_support_tilt')
    if abs(clearance) > a.surface_tolerance_m: errors.append('maintenance_support_clearance')
    if require_support and speed > a.landing_speed_m_s: errors.append('maintenance_support_speed')
    count = 0
    force = 0.
    allowed = 0
    machine_shapes = {s.shape_path for s in scene.shapes}
    for contact in contacts:
        numbers = [contact.separation_m, *contact.position, *contact.normal, *contact.impulse]
        if (len(contact.actors) != 2 or len(contact.shapes) != 2
                or any(len(v) != 3 for v in (contact.position, contact.normal, contact.impulse))
                or not all(math.isfinite(v) for v in numbers)):
            raise ValueError('maintenance contact report incomplete')
        if contact.separation_m > 0 and not any(contact.impulse): continue
        if evaluator.cup_root not in contact.actors:
            if any(p.startswith(evaluator.robot_root+'/') for p in contact.actors): errors.append('robot_forbidden_contact')
            continue
        i = contact.actors.index(evaluator.cup_root)
        other = contact.shapes[1-i]
        local = pose.inverse().transform_point(contact.position)
        normal = pose.inverse().rotate(contact.normal)
        if other == support_shape:
            point = inverse.transform_point(contact.position)
            n = inverse.rotate(contact.normal)
            if (abs(local[2]) > a.surface_tolerance_m or abs(normal[2]) < math.cos(a.tilt_rad)
                    or abs(point[2]) > a.surface_tolerance_m or abs(n[2]) < math.cos(a.tilt_rad)):
                errors.append('maintenance_invalid_bottom_support')
            count += 1
            force += abs(sum(x*y for x,y in zip(contact.impulse, support_pose.rotate((0,0,1))))) / runtime.env.physics_dt
        elif (other in machine_shapes and other != scene.support_shape_path
              and getattr(a, 'cup_machine_contact_allowance_m', 0.) > 0
              and max(0., -contact.separation_m) <= a.cup_machine_contact_allowance_m):
            allowed += 1
        else: errors.append('maintenance_forbidden_contact')
    if force > a.support_force_max_n: errors.append('maintenance_support_force')
    if require_support and not count: errors.append('maintenance_support_not_observed')
    audit = dict(passed=not errors, support=support, require_support=require_support, physics_step=sample.physics_step,
                 support_contacts=count, support_force_n=force, support_clearance_m=clearance,
                 inside_edge_margin=inside, allowed_non_support_contacts=allowed, errors=errors,
                 contact_report_complete=True, task_characterization='not_run')
    runtime.maintenance_initial_audit.update(audit)
    audit=runtime.maintenance_initial_audit
    if errors: raise ValueError('maintenance initial support rejected: '+','.join(errors))
    return audit
