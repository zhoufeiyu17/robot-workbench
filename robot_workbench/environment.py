"""Read-only presentation of manager facts; no lifecycle decisions or device IO."""
from copy import deepcopy
import ipaddress
from urllib.parse import urlsplit


def validate_profiles(rows):
    if not isinstance(rows, list) or len(rows) > 32:
        raise ValueError('最多登记32个明确的环境档案')
    result = []
    for row in rows:
        if (not isinstance(row, dict) or set(row)-{'id','label','environment','device_id','robot_address',
                'compute_address','manager_url','topology','capabilities','configured'}
                or not all(isinstance(row.get(key), str) and row[key] for key in ('id','label','device_id'))
                or row.get('environment') not in ('fake','physical')):
            raise ValueError('环境档案需要明确身份、标签和模式')
        for field in ('robot_address','compute_address'):
            if row.get(field) and str(ipaddress.IPv4Address(row[field])) != row[field]:
                raise ValueError('环境档案地址必须是规范IPv4')
        url = row.get('manager_url', '/')
        if url != '/':
            parsed = urlsplit(url)
            if (parsed.scheme not in ('http','https') or not parsed.hostname or parsed.username or parsed.password
                    or parsed.path not in ('','/') or parsed.query or parsed.fragment):
                raise ValueError('管理入口必须是已登记的HTTP(S)根地址，不得含凭据')
            ipaddress.IPv4Address(parsed.hostname)
            if parsed.port is not None and not 1024 <= parsed.port <= 65535:
                raise ValueError('管理入口端口无效')
        result.append(deepcopy(dict(row, manager_url=url)))
    if len({row['id'] for row in result}) != len(result):
        raise ValueError('环境档案ID重复')
    return result


def describe(config, view, profiles):
    """Present observed dependencies; readiness never grants execution permission."""
    result = deepcopy(view)
    health = result.get('health') or {}
    available = health.get('backend_reachable') is True
    runtime = (health.get('runtime') or {}) if available else {}
    activity = (health.get('activity') or {}) if available else {}
    businesses = activity.get('businesses')
    model = (health.get('model') or {}) if available else {}
    collection = (health.get('collection') or {}) if available else {}
    stamp, identity = health.get('checked_wall_ns'), result.get('target')
    resources = health.get('resources') or dict(status='unknown',reason='未提供当前资源观测，不能据此判定空闲')
    selected_id = config.get('selected_profile_id', profiles[0]['id'] if profiles else None)
    selected = next((row for row in profiles if row['id']==selected_id), {})
    declared = (result.get('machine_profile') or {}).get('declared') or {}

    def boolean(value):return value if type(value) is bool else None
    def together(*values):return False if False in values else True if all(value is True for value in values) else None
    def service_ready(name,row,*,model=True):
        status=row.get('status',row.get('state'))
        ready=True if status in ('ready','busy') else False if status in ('error','failed','stopped','closed','not_loaded') else None
        return boolean(row.get('model_initialized')) if model and name in ('planner','scene') and ready is True else ready
    def supported(name):
        if not available:return None
        if isinstance(businesses,dict) and name in businesses:return True
        if name=='collection' and collection.get('status') not in (None,'unknown','unconfigured'):return True
        if isinstance(businesses,dict) or name=='collection' and collection.get('status')=='unconfigured':return False
        return None
    def capability(ready, reason, *, support=True):
        if support is not True:ready=False if support is False else None
        return dict(status='unsupported' if support is False else 'unknown' if ready is None else 'ready' if ready else 'blocked',
            ready=ready,reason=None if ready is True else reason,checked_wall_ns=stamp,identity=identity,
            support='supported' if support is True else 'unsupported' if support is False else 'unknown')
    def business_capability(name,ready,reason):
        support=supported(name)
        if support is False:reason='当前环境未配置此业务'
        elif support is None:reason='尚未取得此业务的中立注册事实'
        return capability(ready,reason,support=support)

    observed = boolean((result.get('current_capabilities') or {}).get('camera_frames')) if available else None
    connected = boolean(runtime.get('connected'))
    state = str(runtime.get('state') or '').upper()
    release = runtime.get('release_receipt') or {}
    guards = [row for row in (runtime.get('output_guard'),health.get('output_guard')) if isinstance(row,dict)]
    guard_fault=next((row for row in guards if row.get('latched') is True or row.get('state') in ('latched','fault','failed')),None)
    fault = (any(word in state for word in ('FAULT','EMERGENCY','LATCHED')) or bool(runtime.get('recoverable_fault'))
        or bool(runtime.get('initialization_error')) or bool(runtime.get('service_error'))
        or release.get('status') in ('requested','unconfirmed')
        or guard_fault is not None)
    holding = runtime.get('hold') or {}
    hold_ready = all(holding.get(side) in ('position','damping','manual_damping') for side in ('left','right')) if holding else None
    control = False if fault else together(connected,boolean(runtime.get('feedback_healthy')),hold_ready,
        True if state in ('READY','STOPPED_DAMPING_HOLD','MANUAL_TAKEOVER') else None)
    if runtime.get('control_coherence',{}).get('consistent') is False:control=False
    control_reason = (release.get('message') or runtime.get('service_error') or runtime.get('initialization_error')
        or (guard_fault or {}).get('reason') or runtime.get('recoverable_fault')
        or ('控制故障或释放仍锁定' if fault else '请接入设备控制' if connected is False else '控制状态、反馈与保持尚未核验'))
    from .state_contract import model_readiness
    model_gate=model_readiness(model,health.get('model_service'),required='model' in health.get('services',{}))
    model_ready=model_gate['ready']
    if available:model=dict(model,readiness=model_gate)
    collection_ready = boolean(collection.get('ready'))
    if collection.get('status') in (None,'unknown'):collection_ready=None
    elif collection.get('status')=='unconfigured':collection_ready=False
    collection_services={name:row for name,row in (collection.get('services') or {}).items()
        if name in ('detector','planner','scene') and isinstance(row,dict)}
    collection_ready=together(collection_ready,*(service_ready(name,row) for name,row in collection_services.items()))
    collection_reason = collection.get('error') or collection.get('message') or ('数采服务已就绪，请在数采页初始化与预热'
        if collection.get('services_ready')is True else '请在运维准备数采资产与常驻服务；业务初始化与预热在数采页执行')
    cameras = health.get('cameras')
    observe_support = (bool(cameras) if isinstance(cameras,list) else True if observed is not None else None) if available else None
    capabilities = dict(
        observe=capability(observed,'尚未确认当前三路新鲜画面',support=observe_support),
        control=capability(control,control_reason,support=True if available and runtime else None),
        calibration=business_capability('calibration',together(control,observed),'标定需要控制和新鲜相机；首次标定不要求已有标定结果或推理模型'),
        collection=business_capability('collection',together(control,observed,collection_ready),collection_reason),
        inference=business_capability('inference',together(control,observed,model_ready),model_gate['reason'] or model.get('error') or '推理需要控制、新鲜相机及所选模型加载与预热'))

    maintenance = health.get('maintenance')
    busy = boolean(activity.get('busy',health.get('busy'))) if available else None
    admission_reason = ('维护窗口仍冻结业务准入' if maintenance or runtime.get('maintenance_active') is True else
        '当前业务、录制或控制操作尚未结束' if busy is True else control_reason if control is False else
        '尚未取得完整的当前执行准入事实' if not available or busy is None or control is None else
        '依赖就绪不代表执行授权；进入业务页核验本轮占用、审阅和确认')
    admission = dict(status='blocked' if maintenance or runtime.get('maintenance_active') is True or busy is True or control is False else
        'unknown' if not available or busy is None or control is None else 'requires_review',
        reason=admission_reason,authorization_evaluated=False,checked_wall_ns=stamp)

    dependencies = [
        dict(id='deployment',label='受支持的服务与身份',category='preinstalled',
            expected='管理服务、固定启动配置和设备档案已部署',actual='业务接口可核验' if available else health.get('error','业务后端尚未核验'),
            status='ready' if available else 'blocked' if health.get('backend_reachable') is False else 'unknown',recommended_action='prepare_environment'),
        dict(id='device_adapter',label='设备适配与驱动',category='preinstalled',
            expected='Physical：已匹配的SDK、CAN驱动、相机序列号和内参' if config['environment']=='physical' else 'Fake：Isaac/PhysX设备适配和相机角色映射',
            actual=declared.get('hardware') or '未提供部署声明',status='declared' if declared.get('hardware') else 'unknown',
            reason='部署声明不是设备或版本核验；后端在线不能代替驱动与反馈证据'),
        dict(id='network',label='网络与计算拓扑',category='preinstalled',
            expected='使用已登记的机器人/计算地址和管理入口；分机部署分别核验链路与身份',
            actual={key:selected[key] for key in ('id','robot_address','compute_address','manager_url','topology','configured') if key in selected},
            status='declared' if selected.get('configured') is True else 'unknown',
            reason='档案声明不等于网络可达；此展示不扫描端口、不连接机器人'),
        dict(id='compute_resources',label='计算资源与使用授权',category='preinstalled',
            expected='Fake仿真需要兼容GPU；模型与数采按部署选择计算设备，并遵守已有资源预留',
            actual=resources,status='declared' if (resources.get('reservation') or {}).get('status')=='declared' else 'unknown',
            reason='GPU利用率或显存观测不代表空闲或独占授权；远端计算以对应档案和资源记录核验'),
        dict(id='observation',label='相机观测',category='preparable',expected='声明的RGB-D配置与三路新鲜帧',
            actual='三路画面已核验' if observed is True else '画面未核验',status=capabilities['observe']['status'],recommended_action='start_cameras'),
        dict(id='model',label='所选推理模型',category='preparable',expected='仅推理需要所选bundle加载与预热；不阻断首次标定',
            actual=model.get('selected_model_id') or model.get('label') or model.get('id') or '尚未选择',
            status=capability(model_ready,'所选模型尚未核验')['status'],recommended_action='prepare_model'),
        dict(id='collection',label='数采资产与常驻服务',category='preparable',expected='有效标定/派生资产与已声明的常驻服务；观测、场景与模型初始化由业务页执行',
            actual=dict(services_ready=collection.get('services_ready'),business_ready=collection_ready),
            status=capability(collection.get('services_ready',collection_ready),collection_reason,support=supported('collection'))['status'],
            reason=None if collection.get('services_ready',collection_ready)is True else collection_reason,recommended_action='prepare_collection'),
        dict(id='onsite',label='现场条件',category='onsite',
            expected='控制器与电机供电、CAN/USB连线、双臂可靠支撑、工作区及急停/踏板条件',
            actual='Fake无物理供电和线缆条件' if config['environment']=='fake' else '由操作者在具体接入/恢复时确认',
            status='not_applicable' if config['environment']=='fake' else 'operator_confirmation',
            reason='踏板、软件Stop、guard与硬件急停分别按适配器证据显示，不相互替代')]
    if config['environment']=='fake':
        dependencies.extend([
            dict(id='fake_runtime',label='仿真运行时',category='preinstalled',
                expected='匹配的NVIDIA驱动、GPU运行环境、Isaac Sim与PhysX版本',actual='当前事实未提供版本兼容性核验',
                status='unknown',reason='GPU遥测和后端可达不能代替仿真运行时兼容性核验'),
            dict(id='fake_assets',label='仿真机器人与场景资产',category='preinstalled',
                expected='部署所需机器人USD/网格、关节映射、场景和相机资产；首次标定不要求既有标定产物',
                actual=dict(robot_model=(declared.get('hardware') or {}).get('robot_model'),topology=selected.get('topology')),
                status='declared' if (declared.get('hardware') or {}).get('robot_model') else 'unknown',reason='资产身份和完整性仍需对应部署证据')])
    services = deepcopy(health.get('services') or {})
    for name,label in (('detector','物体检测服务'),('planner','轨迹规划服务'),('scene','场景服务')):
        row=collection_services.get(name)
        if not isinstance(row,dict):continue
        services[name]=deepcopy(row)
        ready=service_ready(name,row,model=False)
        dependencies.append(dict(id='collection_'+name,label=label,category='preparable',expected='已归属当前数采环境的常驻服务可用；模型初始化在数采页核验',
            actual=deepcopy(row),status=capability(ready,'当前服务未就绪')['status'],reason=row.get('error') or row.get('reason'),recommended_action='prepare_collection'))
    for row in dependencies:
        row.setdefault('reason',None);row['checked_wall_ns']=stamp
    result.update(schema='robot_workbench.environment.v1',environment=config['environment'],
        device=dict(id=config['device_id'],environment=config['environment']),runtime=runtime,
        activity=activity or dict(busy=busy),maintenance=maintenance,execution_admission=admission,
        capabilities=capabilities,dependencies=dependencies,services=services,resources=resources,
        model=model,collection=collection,profiles=deepcopy(profiles),selected_profile_id=selected_id,
        safety_stop_path='/api/operations/stop')
    return result
