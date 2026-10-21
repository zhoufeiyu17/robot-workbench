"""CPU scope misuse and support failures; these fixtures do not qualify native physics."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from maintenance_geometry import PURPOSE, STATUS, audit_initial_support, validate_export


class Pose:
    def __init__(self, position=(0.,0.,0.), up=1.):
        self.position=position;self.up=up;self.orientation_wxyz=(1.,0.,0.,0.)
    def inverse(self): return Pose(tuple(-x for x in self.position))
    def transform_point(self, p): return tuple(a+b for a,b in zip(self.position,p))
    def rotate(self, p): return (p[0],p[1],p[2]*self.up)
    def compose(self, p): return Pose(self.transform_point(p.position))


def fixture():
    a=NS(edge_margin_m=.005,tilt_rad=.1,surface_tolerance_m=.003,landing_speed_m_s=.02,
         support_force_max_n=10.,cup_machine_contact_allowance_m=.003)
    evaluator=NS(cup=NS(bottom_radius_m=.03),cup_root='/Cup',robot_root='/Robot',table_shape='/table',
                 machine_profile_sha256='profile',_lowest_z=lambda pose:pose.position[2])
    runtime=NS(evaluator=evaluator,spec=NS(acceptance=a),env=NS(physics_dt=.01))
    sample=NS(world_from_cup=Pose(),cup_velocity=(0.,0.,0.),physics_step=1,episode_id='episode')
    scene=NS(world_from_support=Pose(),support_shape_path='/tray',
             instance=NS(episode_id='episode'),observation_id='1',profile_sha256='profile',
             support_polygon_world_m=[(-.1,-.1,0),(.1,-.1,0),(.1,.1,0),(-.1,.1,0)],
             shapes=[NS(shape_path='/tray'),NS(shape_path='/machine')])
    contact=NS(actors=('/Cup','/machineBody'),shapes=('/cupShape','/tray'),position=(0.,0.,0.),
               normal=(0.,0.,1.),impulse=(0.,0.,.01),separation_m=.001)
    return runtime,sample,scene,contact


class MaintenanceTest(unittest.TestCase):
    def test_same_speed_threshold_applies_only_to_final_snapshot_and_all_samples_survive(self):
        runtime,sample,scene,contact=fixture();sample.cup_velocity=(0.,0.,.04)
        audit_initial_support(runtime,sample,scene,[contact],support='tray')
        with self.assertRaisesRegex(ValueError,'maintenance_support_speed'):
            audit_initial_support(runtime,sample,scene,[contact],support='tray',require_support=True)
        self.assertEqual(len(runtime.maintenance_audit_trace),2)
        self.assertTrue(runtime.maintenance_audit_trace[0]['passed'])
        self.assertFalse(runtime.maintenance_audit_trace[1]['passed'])
        self.assertEqual(runtime.maintenance_audit_trace[1]['cup_speed_m_s'],.04)
        self.assertEqual(runtime.maintenance_audit_trace[1]['speed_limit_source'],'landing_speed_m_s')
        self.assertTrue(runtime.maintenance_audit_trace[1]['speed_check_additional'])

    def test_ordinary_load_rejects_maintenance_even_with_forged_success_before_native_import(self):
        from playground_geometry import load
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'planning-input.json'
            p.write_text(json.dumps(dict(export_scope=dict(purpose=PURPOSE))))
            result=p.with_name('result.json')
            for status in (STATUS,'geometry-inputs-ready'):
                result.write_text(json.dumps(dict(status=status)))
                with patch('playground_geometry.configure_native') as native:
                    with self.assertRaisesRegex(ValueError,'maintenance scope rejected'):load(p,{},'unused','unused')
                    native.assert_not_called()

    def test_maintenance_without_current_binding_never_loads(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'planning-input.json';p.write_text('{}')
            p.with_name('result.json').write_text(json.dumps(dict(status=STATUS)))
            with self.assertRaises(ValueError): validate_export(p,{}, {})

    def test_support_positive_gap_impulse_is_active_and_no_task_evaluator_is_needed(self):
        runtime,sample,scene,contact=fixture()
        result=audit_initial_support(runtime,sample,scene,[contact],support='tray',require_support=True)
        self.assertTrue(result['passed']);self.assertEqual(result['support_contacts'],1)
        self.assertEqual(result['task_characterization'],'not_run')
        contact.impulse=(0.,0.,0.)
        with self.assertRaisesRegex(ValueError,'not_observed'):
            audit_initial_support(runtime,sample,scene,[contact],support='tray',require_support=True)

    def test_original_support_edge_tilt_clearance_force_and_forbidden_contacts_still_reject(self):
        for kind in ('edge','tilt','clearance','force','robot','cup_robot','unknown'):
            runtime,sample,scene,contact=fixture()
            if kind=='edge':sample.world_from_cup.position=(.09,0.,0.)
            elif kind=='tilt':sample.world_from_cup.up=.9
            elif kind=='clearance':sample.world_from_cup.position=(0.,0.,-.0031)
            elif kind=='force':contact.impulse=(0.,0.,.11)
            elif kind=='robot':contact.actors=('/Robot/link','/machineBody')
            elif kind=='cup_robot':contact.actors=('/Cup','/Robot/link');contact.shapes=('/cupShape','/Robot/link')
            else:contact.separation_m=float('nan')
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                audit_initial_support(runtime,sample,scene,[contact],support='tray',require_support=True)

    def test_three_mm_allowance_never_applies_to_support_shape(self):
        runtime,sample,scene,contact=fixture();other=deepcopy(contact)
        other.shapes=('/cupShape','/machine');other.separation_m=-.003
        self.assertTrue(audit_initial_support(runtime,sample,scene,[contact,other],support='tray',require_support=True)['passed'])
        other.separation_m=-.00301
        with self.assertRaises(ValueError):audit_initial_support(runtime,sample,scene,[contact,other],support='tray',require_support=True)
        other.shapes=('/cupShape','/tray');other.position=(0.,0.,.01);other.separation_m=-.001
        with self.assertRaisesRegex(ValueError,'invalid_bottom_support'):
            audit_initial_support(runtime,sample,scene,[contact,other],support='tray',require_support=True)


if __name__=='__main__':unittest.main()
