"""CPU certificate contract fixtures; these records are never geometry evidence."""
from copy import deepcopy
from dataclasses import asdict,dataclass
from types import SimpleNamespace as NS
from unittest.mock import patch
from integration import _digest
from grasp_contact_certificate import GRASP_CONTACT_POLICY,validate_contact_certificate,inverse_pose


def certificate_fixture(source_digest,urdf_sha256,*,source_parts_sha256='b'*64,
        cup_profile_sha256='c'*64,cup_geometry_sha256='d'*64,exported_geometry_sha256='e'*64,
        candidate_id='cpu-fixture',actual_cup_from_tool=None,grasp_q_model=None):
    pose=deepcopy(actual_cup_from_tool or dict(position=[-.12,0.,.13],orientation_wxyz=[1.,0.,0.,0.]))
    regions=[dict(link=link,band='/Cup/Collisions/MiddleBody',jaw_interval_m=[.032496,.038],
        height_interval_m=[.04,.06],lower_witness_cup_m=[0.,0.,.04],upper_witness_cup_m=[0.,0.,.06],
        extremum_jaws_m=[.038,.038]) for link in ('left_link7','left_link8')]
    region=dict(schema_version='grasp_contact_geometry.v1',passed=True,reason='passed',input_digest='f'*64,
        urdf_sha256=urdf_sha256,source_parts_sha256=source_parts_sha256,cup_profile_sha256=cup_profile_sha256,
        original_cup_from_tool=pose,commanded_closed_jaw_m=.032496,
        independent_contact_stops_lp_m=dict(left_joint7=.038,left_joint8=.038),native_closing_checked=True,
        table_normal_cup=[0.,0.,1.],centering_axis_cup=[0.,1.,0.],
        contact_geometry_stops_m=dict(left_joint7=.038,left_joint8=.038),terminal_jaw_position_m=.038,
        centering_displacement_cup_m=[0.,0.,0.],original_cup_from_held_cup=dict(position=[0.,0.,0.],orientation_wxyz=[1.,0.,0.,0.]),
        jaw_breakpoints_m=[.041496,.038,.032496],nominal_height_band_m=[.035,.1],reserve_m=.002,
        permitted_height_band_m=[.037000000000000005,.098],contact_height_interval_m=[.04,.06],regions=regions,
        numerical_bound_padding_m=1e-9,native_acceptance_tolerance_consumed_m=0.,
        method='4D linear programming over original convex pad/cup solid intersections',
        scope='CPU synthetic contract fixture; not geometric or execution evidence')
    region['geometry_digest']=_digest(region)
    value=dict(schema_version='grasp_contact_certificate.v1',passed=True,policy=deepcopy(GRASP_CONTACT_POLICY),
        grasp_q_model=list(grasp_q_model or [0.]*6),
        source_digest=source_digest,urdf_sha256=urdf_sha256,source_parts_sha256=source_parts_sha256,
        cup_profile_sha256=cup_profile_sha256,cup_geometry_sha256=cup_geometry_sha256,
        exported_geometry_sha256=exported_geometry_sha256,candidate_id=candidate_id,original_cup_from_tool=pose,
        tool_from_held_cup=inverse_pose(pose),original_cup_from_held_cup=region['original_cup_from_held_cup'],
        contact_geometry_stops_m=region['contact_geometry_stops_m'],
        grasp_contact_region=region,native_closing_digest='a'*64,claw_cup_allowance_m=.003,
        linked_jaws=True,terminal_jaw_position_m=.038,maximum_claw_penetration_m=.00001,
        cup_radius_from_origin_m=.14,numerical_contact_reserve_m=1e-5,
        wrist_source_separation_proven=True,minimum_wrist_clearance_m=.005,
        jaw_sweep_source_samples=[dict(jaw_position_m=gap,tool_from_cup=inverse_pose(pose),maximum_claw_penetration_m=.00001)
                                  for gap in (.041496,.038)])
    value['certificate_digest']=_digest(value)
    return value


def main():
    proof=certificate_fixture('source','robot')
    assert validate_contact_certificate(proof,source_digest='source',urdf_sha256='robot')==proof
    for key,value in [('source_digest','other'),('certificate_digest','bad'),('linked_jaws',False),('grasp_contact_region',{})]:
        changed=deepcopy(proof);changed[key]=value
        if key!='certificate_digest':changed['certificate_digest']=_digest({k:v for k,v in changed.items() if k!='certificate_digest'})
        try:validate_contact_certificate(changed,source_digest='source',urdf_sha256='robot')
        except ValueError:pass
        else:raise AssertionError('invalid certificate accepted: '+key)
    def reseal(value):
        region=value['grasp_contact_region']
        region['geometry_digest']=_digest({k:v for k,v in region.items() if k!='geometry_digest'})
        value['certificate_digest']=_digest({k:v for k,v in value.items() if k!='certificate_digest'})
    changes=(
        lambda p:p['grasp_contact_region'].__setitem__('native_closing_checked',False),
        lambda p:p['grasp_contact_region']['regions'][0]['height_interval_m'].__setitem__(0,.0369),
        lambda p:p['grasp_contact_region']['regions'][0]['height_interval_m'].__setitem__(1,.0981),
        lambda p:p['grasp_contact_region'].__setitem__('centering_displacement_cup_m',[]),
        lambda p:p['original_cup_from_held_cup'].__setitem__('orientation_wxyz',[0.,1.,0.,0.]),
        lambda p:p['tool_from_held_cup']['position'].__setitem__(0,1.),
        lambda p:p['jaw_sweep_source_samples'][0].__setitem__('jaw_position_m',1.),
        lambda p:p['jaw_sweep_source_samples'][0].__setitem__('maximum_claw_penetration_m',-.000005),
    )
    for mutate in changes:
        bad=deepcopy(proof);mutate(bad);reseal(bad)
        try:validate_contact_certificate(bad,source_digest='source',urdf_sha256='robot')
        except ValueError:pass
        else:raise AssertionError('self-resealed invalid continuous/native proof accepted')
    @dataclass
    class Pose:
        position:tuple=(0.,0.,0.)
        orientation_wxyz:tuple=(1.,0.,0.,0.)
        def compose(self,other):return Pose(tuple(a+b for a,b in zip(self.position,other.position)))
        def inverse(self):return Pose(tuple(-v for v in self.position))
        def rotate(self,values):return tuple(values)
    from grasp_contact_certificate import build_contact_certificate
    planner=NS(geo=NS(mount=Pose()),table=NS(world_from_shape=Pose()),cup=NS(profile_sha256='c'*64),
        fk=NS(forward=lambda state:{'left_link6':Pose()},pose=lambda value:value))
    data=dict(parts=[],urdf_sha256='robot');source_parts=_digest(data['parts'])
    fresh=certificate_fixture('source','robot',source_parts_sha256=source_parts,
        exported_geometry_sha256=_digest(data),actual_cup_from_tool=asdict(Pose()))
    native=dict(passed=True,contact_geometry_stops_m=dict(left_joint7=.038,left_joint8=.038),
        terminal_state=dict(left_joint7=.038,left_joint8=.038),centering_displacement_world_m=[0.,0.,0.],samples=[])
    with patch('grasp_contact_geometry.analyze_contact_pose',return_value=fresh['grasp_contact_region']) as analyze,\
            patch('grasp_contact_certificate.source_closing',return_value=native),\
            patch('grasp_contact_certificate.source_contact_certificate',return_value={k:v for k,v in fresh.items() if k!='certificate_digest'}),\
            patch('grasp_contact_certificate.enrich_contact_sweep',side_effect=lambda planner,proof,closing:deepcopy(proof)):
        closing,built=build_contact_certificate(planner,data,{},Pose(),source_digest='source',candidate_id='cpu-fixture',closed_jaw_m=.032496)
        assert closing==native and analyze.call_count==2
        assert 'closing_geometry' not in analyze.call_args_list[0].kwargs
        assert analyze.call_args_list[1].kwargs['closing_geometry']['contact_geometry_stops_m']==native['contact_geometry_stops_m']
        assert built['grasp_contact_region']['native_closing_checked'] is True
    with patch('grasp_contact_geometry.analyze_contact_pose',side_effect=[fresh['grasp_contact_region'],{'passed':False,'reason':'native centering differs'}]),\
            patch('grasp_contact_certificate.source_closing',return_value=native),\
            patch('grasp_contact_certificate.source_contact_certificate') as create:
        try:build_contact_certificate(planner,data,{},Pose(),source_digest='source',candidate_id='cpu-fixture',closed_jaw_m=.032496)
        except ValueError:pass
        else:raise AssertionError('native closing path escaped continuous contact recheck')
        assert not create.called
    print('CPU grasp contact certificate binding/tamper checks PASS; geometry not asserted')


if __name__=='__main__':main()
