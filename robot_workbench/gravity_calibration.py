"""Pinned joint-zero calibration for the SDK solver, independent of command q."""
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def _read(pin):
    if (not isinstance(pin,dict) or set(pin)!={'path','sha256'}
            or not isinstance(pin['path'],str) or not Path(pin['path']).is_absolute()
            or '..' in Path(pin['path']).parts or not isinstance(pin['sha256'],str)
            or not re.fullmatch('[0-9a-f]{64}',pin['sha256'])):
        raise ValueError('absolute hash-pinned gravity calibration input required')
    from .control_plan import _read_file
    raw=_read_file(pin['path'],256*1024)
    if hashlib.sha256(raw).hexdigest()!=pin['sha256']:
        raise ValueError('gravity calibration input changed: '+pin['path'])
    return raw


def validate_sdk_model(config,calibration):
    """Validate nominal model identity across installation-directory changes."""
    if (config.robot_model!='X5' or list(config.gravity_vector)!=calibration['gravity_vector']):
        raise ValueError('SDK gravity model differs from the calibrated source')
    try:
        _read(dict(calibration['source_pins']['sdk_model'],path=config.urdf_path))
    except (OSError,ValueError) as error:
        raise ValueError('SDK gravity model differs from the calibrated source') from error


def check_sdk_solver(profile):
    """Probe the selected native configuration and solvers without controllers."""
    import importlib.util
    import numpy as np
    from .sdk_quiesce import verify_module
    pin=profile['sdk_quiesce']
    spec=importlib.util.spec_from_file_location('arx5_interface',pin['module']['path'])
    sdk=importlib.util.module_from_spec(spec);spec.loader.exec_module(sdk)
    verify_module(sdk,pin)
    config=sdk.RobotConfigFactory.get_instance().get_config('X5')
    calibration=load_gravity_calibration(profile.get('sdk_gravity_calibration'))
    models=[config.urdf_path]
    if calibration is not None:
        validate_sdk_model(config,calibration)
        models.extend(calibration['models'][side]['path']for side in('left','right'))
    q=np.zeros(config.joint_dof)
    for model in models:
        solver=sdk.Arx5Solver(model,config.joint_dof,config.joint_pos_min,config.joint_pos_max,
            config.base_link_name,config.eef_link_name,config.gravity_vector)
        if (not np.isfinite(solver.forward_kinematics(q)).all()
                or not np.isfinite(solver.inverse_dynamics(q,q,q)).all()):
            raise ValueError('SDK solver produced nonfinite FK or gravity')
    return dict(passed=True,models_checked=len(models),hardware_connected=False,
        scope='selected native SDK configuration and calibrated solvers; no controllers')


def bake_joint_offsets(raw,offsets):
    """T_origin*R_axis(offset)*R_axis(q) equals the original FK at q+offset."""
    import numpy as np
    from scipy.spatial.transform import Rotation
    tree=ET.fromstring(raw)
    if len(offsets)!=6 or any(type(x) not in(int,float) or not math.isfinite(x) for x in offsets):
        raise ValueError('six finite gravity joint offsets required')
    for i,delta in enumerate(offsets,1):
        joint=tree.find("joint[@name='joint"+str(i)+"']")
        if joint is None or joint.get('type')!='revolute':
            raise ValueError('gravity model must contain the six X5 revolute joints')
        origin=joint.find('origin');axis=joint.find('axis')
        if origin is None or axis is None:raise ValueError('explicit X5 joint origin and axis required')
        vector=np.fromstring(axis.get('xyz',''),sep=' ')
        rpy=np.fromstring(origin.get('rpy','0 0 0'),sep=' ')
        if vector.shape!=(3,) or rpy.shape!=(3,) or not np.isfinite(vector).all() or not np.isfinite(rpy).all() or np.linalg.norm(vector)<1e-12:
            raise ValueError('finite gravity model axis and origin required')
        rotation=Rotation.from_euler('xyz',rpy)*Rotation.from_rotvec(vector/np.linalg.norm(vector)*delta)
        origin.set('rpy',' '.join(format(x,'.17g') for x in rotation.as_euler('xyz')))
    return ET.tostring(tree,encoding='utf-8',xml_declaration=True)


def load_gravity_calibration(pin):
    if pin is None:return None
    value=json.loads(_read(pin))
    if (not isinstance(value,dict) or set(value)!={'schema','robot_id','source_pins','joint_offsets_rad','models','convention','gravity_vector'}
            or value['schema']!='sdk_gravity_calibration.v1'
            or not isinstance(value['robot_id'],str) or not value['robot_id']
            or value['convention']!='q_gravity_model=q_sdk_feedback+joint_offsets_rad'
            or value['gravity_vector']!=[0.,0.,-9.807]
            or not isinstance(value['source_pins'],dict)
            or set(value['source_pins'])!={'head','wrist','manifest','sdk_model','calibration_model'}
            or not isinstance(value['models'],dict) or set(value['models'])!={'left','right'}
            or not isinstance(value['joint_offsets_rad'],dict) or set(value['joint_offsets_rad'])!={'left','right'}):
        raise ValueError('exact SDK gravity calibration schema and convention required')
    sources={name:_read(source) for name,source in value['source_pins'].items()}
    head,wrist,manifest=(json.loads(sources[name])for name in('head','wrist','manifest'))
    if (not all(isinstance(v,dict)for v in(head,wrist,manifest))
            or not all(isinstance(manifest.get(k),dict)for k in('head','wrist'))
            or head.get('kind')!='head' or wrist.get('kind')!='wrist'
            or not head.get('id') or not wrist.get('id')
            or head.get('id')!=manifest['head'].get('id') or wrist.get('id')!=manifest['wrist'].get('id')
            or any(v.get('robot_id')!=value['robot_id'] for v in(head,wrist,manifest))):
        raise ValueError('gravity calibration group or robot identity differs')
    try:installed=head['payload']['row']['parameters']['joint_offsets_rad']
    except (KeyError,TypeError):raise ValueError('installed head lacks joint offset parameters')from None
    if value['joint_offsets_rad']!=installed:
        raise ValueError('gravity offsets differ from installed head calibration')
    for side,offsets in value['joint_offsets_rad'].items():
        if (not isinstance(offsets,list) or len(offsets)!=6
                or any(type(x)not in(int,float) or not math.isfinite(x) or abs(x)>math.radians(5) for x in offsets)):
            raise ValueError('gravity offsets exceed the calibration solver domain')
        raw=_read(value['models'][side])
        if raw!=bake_joint_offsets(sources['sdk_model'],offsets):
            raise ValueError('gravity model is not the exact single application of installed offsets')
    return dict(value,id=pin['sha256'])
