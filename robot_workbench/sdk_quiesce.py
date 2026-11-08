"""Pinned, opt-in SDK candidate contract. Never accepts browser configuration."""
import hashlib
import json
from pathlib import Path
import re


REPORT_FIELDS=('library_build_id','controller_instance_id','caller_generation','send_thread_joined',
               'last_send_attempt_monotonic_ns','quiesce_returned_monotonic_ns','receiver_state','transport_basis','verification_scope','idempotent_hit')


def dm_joint_velocity_wire_scale(contract, *, can_interfaces=None, robot_id=None):
    """Opt in only for the six measured A1 wrists; never infer ranges from model names."""
    if contract is None:return None
    if (not isinstance(contract,dict)or set(contract)!={'vendor_vmax_rad_s','register_receipt'}
            or type(contract['vendor_vmax_rad_s'])not in(int,float)or contract['vendor_vmax_rad_s']!=45.):
        raise ValueError('pinned DM vendor velocity range and register receipt required')
    pin=contract['register_receipt']
    if (not isinstance(pin,dict)or set(pin)!={'path','sha256'}or not isinstance(pin['path'],str)
            or not Path(pin['path']).is_absolute()or '..'in Path(pin['path']).parts
            or not isinstance(pin['sha256'],str)or not re.fullmatch('[0-9a-f]{64}',pin['sha256'])):
        raise ValueError('absolute hash-pinned DM register receipt required')
    from .control_plan import _read_file
    raw=_read_file(pin['path'],128*1024)
    if hashlib.sha256(raw).hexdigest()!=pin['sha256']:raise ValueError('DM register receipt hash differs')
    receipt=json.loads(raw);rows=receipt.get('queries',[])
    buses=can_interfaces or {'left':'can3','right':'can1'}
    if buses!={'left':'can3','right':'can1'} and (
            robot_id!='arx13' or buses!={'left':'can13l','right':'can13r'}
            or receipt.get('robot_id')!=robot_id or receipt.get('can_interfaces')!=buses):
        raise ValueError('DM register receipt belongs to another physical device')
    if (receipt.get('complete')is not True or any(receipt.get(k)!=0 for k in
            ('parameter_writes','enable_commands','motion_commands'))or not isinstance(rows,list)or len(rows)!=18):
        raise ValueError('complete non-enabling six-motor register receipt required')
    seen=set()
    for row in rows:
        if not isinstance(row,dict):raise ValueError('DM register row must be an object')
        side,motor,rid=(row.get(k)for k in('side','motor_id','register'));key=(side,motor,rid)
        if (side not in('left','right')or type(motor)is not int or motor not in(5,6,7)
                or type(rid)is not int or rid not in(21,22,23)or key in seen
                or row.get('bus')!=buses[side]or row.get('query_count')!=1
                or row.get('request_can_id')!=0x7ff):raise ValueError('DM register identity differs')
        payload=bytes([motor,0,0x33,rid,0,0,0,0]);replies=row.get('responses')
        if row.get('request_data')!=payload.hex()or not isinstance(replies,list)or len(replies)!=1:
            raise ValueError('one matching DM register response required')
        import struct
        reply=replies[0]
        if not isinstance(reply,dict)or not isinstance(reply.get('data'),str):raise ValueError('DM register reply bytes required')
        data=bytes.fromhex(reply['data'])
        expected={21:12.5,22:30.,23:10.}[rid]
        if (len(data)!=8 or data[:4]!=payload[:4]or reply.get('can_id')!=0
                or struct.unpack('<f',data[4:])[0]!=expected or reply.get('value')!=expected):
            raise ValueError('DM register ranges differ from this qualified deployment')
        seen.add(key)
    return 45./30.


DM_EQUIVALENT_UNITS = 'arx13_dm_protocol_equivalent_v1'
DM_LEGACY_TO_PROTOCOL = 10. / 18. / .424


def dm_torque_units(contract, *, can_interfaces=None, robot_id=None):
    """Six measured ARX13 wrists; explicit units with the former wire output retained."""
    if contract is None:return None
    if (not isinstance(contract,dict)or set(contract)!={'mode','register_receipt'}
            or contract['mode']!=DM_EQUIVALENT_UNITS or robot_id!='arx13'
            or can_interfaces!={'left':'can13l','right':'can13r'}):
        raise ValueError('reviewed ARX13 equivalent DM torque contract required')
    dm_joint_velocity_wire_scale(dict(vendor_vmax_rad_s=45.,register_receipt=contract['register_receipt']),
                                 can_interfaces=can_interfaces,robot_id=robot_id)
    return DM_EQUIVALENT_UNITS


def load_gripper_calibration(pin, *, robot_id=None):
    """Pinned existing machine endpoints; the research SDK retains its 82 mm coordinate."""
    if pin is None:return None
    from .gravity_calibration import _read
    import math
    value=json.loads(_read(pin))
    if (not isinstance(value,dict) or set(value)!={'schema','robot_id','source','sdk_width_m','sides'}
            or value['schema']!='native_gripper_endpoints.v1' or value['robot_id']not in ('arx6','arx13')
            or robot_id is not None and value['robot_id']!=robot_id
            or value['sdk_width_m']!=.082 or not isinstance(value['sides'],dict) or set(value['sides'])!={'left','right'}):
        raise ValueError('exact machine-bound native gripper endpoint schema required')
    source=json.loads(_read(value['source']))
    buses={'left':'can3','right':'can1'}if value['robot_id']=='arx6'else{'left':'can13l','right':'can13r'}
    if (source.get('id')!=value['robot_id'] or any(source.get(side+'_can')!=bus for side,bus in buses.items())
            or source.get('left_model')!='X5' or source.get('right_model')!='X5'):
        raise ValueError('gripper source hardware identity differs')
    for side,row in value['sides'].items():
        expected={name:source.get(side+'_gripper_'+name) for name in ('open_readout','closed_readout','width_m')}
        if expected['width_m']is None:expected['width_m']=source.get('gripper_width_m')
        if (row!=expected or any(type(v)not in(int,float) or not math.isfinite(v) for v in row.values())
                or not 0<row['width_m']<=.1 or max(abs(row['open_readout']),abs(row['closed_readout']))>12.5
                or abs(row['open_readout']-row['closed_readout'])<.05):
            raise ValueError('gripper endpoints differ from the pinned machine configuration')
    return dict(value,identity=pin['sha256'])


EC_STATIC_UNITS = 'ec_protocol_torque_left_j2_j3_static_v1'
EC_J2_EQUIVALENT_UNITS = 'ec_protocol_torque_left_j2_equivalent_static_v1'
EC_J2_HALF_UNITS = 'ec_protocol_torque_left_j2_half_static_v1'
EC_J2_FULL_UNITS = 'ec_protocol_torque_left_j2_full_static_v1'
EC_J2_GRAVITY_SCALES = {EC_J2_EQUIVALENT_UNITS:1/1.4,EC_J2_HALF_UNITS:6/7,EC_J2_FULL_UNITS:1.}


def ec_torque_units(contract):
    """One reviewed static candidate, never a free-form motor calibration knob."""
    if contract is None:return None
    expected = 'bfac89fa11b94a12e56887d071813685a166cb66e79e86ee56f4d1d9098f1a5d'
    if (not isinstance(contract,dict) or set(contract)!={'mode','register_receipt'}
            or contract['mode']not in(EC_STATIC_UNITS,*EC_J2_GRAVITY_SCALES)):
        raise ValueError('reviewed left J2/J3 static EC torque contract required')
    pin=contract['register_receipt']
    if (not isinstance(pin,dict) or set(pin)!={'path','sha256'} or pin['sha256']!=expected
            or not isinstance(pin['path'],str) or not Path(pin['path']).is_absolute()
            or '..' in Path(pin['path']).parts):
        raise ValueError('exact reviewed EC parameter receipt required')
    from .control_plan import _read_file
    if hashlib.sha256(_read_file(pin['path'],128*1024)).hexdigest()!=expected:
        raise ValueError('EC parameter receipt hash differs')
    return contract['mode']


def validate_contract(value):
    if (not isinstance(value,dict) or set(value)!={'library_build_id','module','vendor_libraries','vendor_trust'}
            or not isinstance(value['library_build_id'],str)
            or not re.fullmatch('[0-9a-f]{64}',value['library_build_id'])
            or value['vendor_trust']!='human-approved-unmodified-binary'
            or not isinstance(value['vendor_libraries'],list) or not value['vendor_libraries']):
        raise ValueError('explicit pinned SDK/vendor trust contract required')
    for row in [value['module'],*value['vendor_libraries']]:
        if (not isinstance(row,dict) or set(row)!={'path','sha256'}
                or not isinstance(row['path'],str) or not Path(row['path']).is_absolute()
                or '..' in Path(row['path']).parts or not isinstance(row['sha256'],str)
                or not re.fullmatch('[0-9a-f]{64}',row['sha256'])):
            raise ValueError('invalid SDK artifact identity')
        if hashlib.sha256(Path(row['path']).read_bytes()).hexdigest()!=row['sha256']:
            raise ValueError('SDK artifact hash differs: '+row['path'])
    return value


def verify_module(sdk, contract):
    if (Path(sdk.__file__).resolve()!=Path(contract['module']['path']).resolve()
            or getattr(sdk,'workbench_build_id',None)!=contract['library_build_id']):
        raise ValueError('loaded SDK differs from pinned candidate')


def quiesce_report(controller, generation):
    report=controller.quiesce(generation)
    return {key:getattr(report,key) for key in REPORT_FIELDS}
