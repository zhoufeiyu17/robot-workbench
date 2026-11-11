# C 工作台消费者集成：唯一接口源

## 2026-11-09 数采独立相机交接（I10／UC10）

部署存在独立数采相机Owner时，`collection_handoff.camera_owner`登记其`unit`、`files`、
`state_path`及`cameras`。unit/files沿用既有服务登记；state_path提供新鲜心跳、实例与PID，
cameras核对本设备相机身份。只操作显式登记的用户服务，不按机器名或进程名推断归属。

共同交接先确认数采任务正常结束和Web退出，再核对独立相机Owner的PID/相机配置，正常
停止并等待其cgroup清空。Web已经停止也必须处理独立相机；身份不符、心跳失联或退出
未确认均不继续打开工作台相机。动作回执保留`camera_owner`的退出证明。
网页`connect_control`复用此交接，但不停止活动数采任务；显式`reclaim_control`沿用
既有正常停采授权。桌面接管启动先打开管理页，工作台相机延迟到交接完成后由管理器
启动，避免先抢占USB再尝试交接。其他健康服务、数据和数采配置保留。

## 2026-11-09 已确认 HOLD 的原生保持

正常模式完成 HOLD 后，沿用已确认的原生零速度保持输出，不再周期性重发相同的短期
参考窗口。每个控制 tick 仍检查新鲜反馈、执行代次、设备限位和夹爪映射；看门狗继续
由真实反馈请求维护。MODEL、TEACH、HOME 或结束请求仍走原有显式模式切换与确认，
移动参考、初始化和过渡阶段的未来窗口及时限检查保持不变。

桌面入口从部署登记读取合法 systemd 服务名，不要求某个产品前缀。WAM 普通加载复用
大权重的登记摘要，检查受控路径、大小与 manifest，再严格加载；小型资产和运行时代码
继续做内容校验。完整性异常需要另行定位核验，不能把未执行的大权重全量哈希记为通过。

## 2026-11-09 公共退出与数采组合入口（I10／UC09）

主页面与运维页提供 `exit_workbench`。它复用现有停止、低速回零、SDK正常释放、
录制封存及自有服务退出流程，不启动数采。`exit_environment` 是组合入口：先调用同一
公共退出，再启动或复用配置登记的正式数采网页；核验成功才返回 `next_url` 供页面打开。
共享部署只释放本消费者绑定，保留共享Owner、SDK及其他消费者服务。

记录器确认 `can_stop` 后才停止其写入进程和相机。失败或未知结果保留已完成步骤的
回执和管理入口，不强杀、不继续释放、不重放旧动作。两种操作成功且后置核验通过后，
持久回执中的 `result.manager_exit_pending=true` 同样进入轮询摘要，供页面禁用新操作并
显示正常退出。管理HTTP服务等待4秒供正常轮询读取结果，然后正常退出；浏览器关闭
不阻止退出，历史请求、失败请求和未知请求不会触发管理器自退。

退出范围来自部署服务登记和实际进程归属，不按机器名猜测服务，不更改GPU资源台账或
推断转交。单独退出没有数采入口依赖；需要再次使用时运行已登记的启动入口。

## 2026-11-09 桌面冷启动与自动接管（I10）

ARX6 桌面启动配置 `connect_control=true` 表达操作者已在现场确认可以接入。入口先启动
已登记软件，再通过既有 `reclaim_control` 正常停止旧工作台动作和已登记数采、核验真实
进程退出，并接入当前位置保持；无需第二次网页确认，不执行模型轨迹。健康静止连接复用。
未配置该布尔值的消费者继续只启动软件。配置须有 managed Owner 与登记的数采交接入口。

启动操作复用 I10 的 target、request_id、job_id 和成功核验，不重放未知结果。相机来源
重新登记后，失效订阅通过一次 `rebind_cameras` 恢复；健康订阅不重绑。启动完成须同时
满足实际三路帧、目标资产、人工规划、常驻记录器、所选模型与配置要求的控制就绪。

数采 `safe_ok` 是 episode 的复位结果。仅 schema v1 的 stopped、明确 null、空 error、
完整五类组件终态允许使用无 episode 的正常退出证据；manager/orphan/teleop 与 Web
进程及 cgroup 退出仍须核验，false 或缺字段不降级为成功。桌面失败显示该 job 的实际原因。
已确认的 `startup_failed` 可在 episode_index 和 recording_started_at 均明确为空、robot
stopped、其余组件仅 waiting/stopped/disabled 时正常退役；它仍须通过上述真实进程退出
核验，回执保留 previous_collection_phase/error，不把失败数采改写成成功。
验收须从七个登记服务全部停止、网页不可访问开始，验证原桌面入口自动打开网页并达到
上述条件；CPU 用例或热启动不能代替该项。实际回执见本轮 ARX6 startup-rebind 进度。

用户随后明确网页打开目标为 5 秒，不能等待模型准备结束。桌面入口在准备锁之前调用
`--open-page-only`，只启动显式 `ui_unit` 并打开已响应的管理网页，再独立执行原准备链。
模型/控制/输入准备通过既有状态与 job 回执呈现；启动成功仍要求完整就绪。

同日用户要求去掉源码、配置的重复文件扫描。Deployment 在管理器启动时登记一次源码
与配置摘要，投影 `source_verification=captured_at_startup`，删除 `files_unchanged`、
`changed_files` 等运行时漂移投影及前后端/网关中的对应阻塞条件。页面与管理器的部署
身份握手保留。机器配置索引也只在启动时采集；unit/drop-in 摘要保留为历史发布证据，
生命周期按登记 unit、实际 PID/cgroup、已加载状态及真实退出核验。模型/SDK 首次制品
校验保留。此变更不把“未反复校验”表述为“文件持续未变”。

## 2026-11-07 踏板会话与边界门槛

### 2026-11-08 WAM可选静止恢复

用户授权在参考源码未提供时采用合适假设。本轮将夹爪0.05定义为各侧标定行程的5%，
ARX13对应左4.0mm、右4.45mm。窗口为3秒模型源轨迹，即30Hz下90个实际执行步。
Infer订阅既有控制ROS事件，按完整段的起点、重定时时长和步数选择每个模型步之后的首条实测反馈。
重复采样只计一次，控制事件缺口、跳步及超出绑定反馈时限会清空窗口。完整窗口内所有关节
极差小于0.02rad、两侧夹爪极差均小于各自阈值时判为停滞。

`inference_run.wam_stall_recovery`为布尔配置，UI复选框默认关闭，准备会话时冻结，踏板启动复用。
开启后，下一次规划按2块顶部黑屏、1块真实画面循环，每个块对应一次模型请求，执行前缀沿用
ControlPolicy。顶部RGB在预处理前用同形零数组替换，腕部RGB、state及任务文本继续使用原输入。
每个停滞块生成新的uint63种子，排除配置种子及上一次随机值。种子只作用于该请求，
模型错误或正常返回均不会修改配置种子。完整窗口检测到运动后恢复真实输入与配置种子。

ModelInput复用现有metadata_json保存干预决定和实际种子，黑屏输入使用可重建的
`wam_zero_head_bicubic256_wan_bf16`转换。常驻记录器保留原始ROS图像、验证实际输入摘要，
封存时从原ModelInput日志生成`stall_recovery.jsonl`，包含失败模型请求的干预记录。
记录服务在线协议现为`workbench.recording-service.v7`，同时提供下文的取流关闭证明，旧组合在绑定时拒绝。v7要求录制服务加载支持2 rad/s的当前策略校验。服务明确拒绝bind/start后，客户端核验writer仍处于prepared/bound及同一绑定，发送正常结束请求并等待封存与真实退出，保留失败原因及已有结束请求。
Owner和SDK继续消费原动作契约。方案、用例及发布状态见
`.evidence/infer-pedal2-start-20261107-v1/wam-trick-design.json`和本目录统一进度记录。
维护触发为检测单位、步数语义、转换格式、Run配置或录制协议变化。

本节记录本聊天已批准的当前契约。验收状态和实际部署身份见 .evidence/infer-pedal2-start-20261107-v1/progress.json，架构追溯见 web/architecture/catalog.json。

2026-11-08 已确认的踏板规则使用一次网页准备。完成位置 HOLD 后，双踩踏板 1 通过现有 inference_run 启动模型，使用服务端已准备的模型、seed 和运行时长。该手势直接表达启动意图，并显式使用 `memory_start=continue`。WAM-MEM 页面只保留带记忆启动与 HOLD 下清空记忆，不再提供无状态 Infer 按钮。单踩 1 立即请求保持。MODEL 及其进入过程内，踏板 1 和 2 请求保持，踏板 3 无动作。第一脚发生在 Infer 中的双踩，第一脚停止，第二脚忽略。TEACH 中先完成 HOLD，再用一组独立双踩启动。其余时刻，踏板 2 切换 TEACH/HOLD，踏板 3 从 HOLD 启动原生回零。

2026-11-11 用户将双踩识别窗口扩大至 600 ms，发布状态以本轮部署回执为准。来源使用 50 ms 去抖并保留按住锁定。业务侧在既有 `inference.operator_controls` 中一次绑定 `double_press_window_s=0.6`，按 evdev 原始 `occurred_at` 的微秒精度判断双踩间隔，600 ms 边界包含在内。手势同时绑定已有踏板来源实例、前台客户端、Owner 与控制会话。重复来源序号、失焦、会话更换以及夹杂其他踏板均不能把旧第一脚用于启动。每次手势资格由第一脚时的模式确定，执行前再次核对当前 HOLD 与准入。

2026-11-09 用户确认推理录像按 MODEL／TEACH 模式分段：准备和 HOLD 等待、HOME 回零不计入有效录像；TEACH 内短暂停手继续录制，不依赖速度判定。业务在既有 `execution_context` 冻结 `recording_activity=model_teach_compact_v1`，记录器消费同一 Owner 的模式接纳事件，以请求进入 MODEL／TEACH 为开始，退出请求或终止事件为结束。原始源时间和序列不改；仅为模型输入追溯保留策略规定范围内的前置原始样本，前置样本不进入有效回放。

封存的 `recording_activity` 保存原始控制／wall 时间区间及各区间 `effective_start_ns`。每流的 `effective_timestamps_ref` 指向原记录序号与连续有效纳秒时间的映射；每相机把全部有效区间连接成一个 MP4，去掉等待时间，保留 `effective_origin_ns` 关联其首帧。没有有效区间则没有视频。迟到的活动尾帧按原始源时间接纳，不按回调到达时刻误删；缺失或无效活动映射明确标记不完整，不导出未裁切录像。I15 原始记录器健康发布 `recording_activity`（当前有效模式、累计有效秒数、区间数和首次开始 wall 时间），页面据此显示等待／录制。历史任务名以实际首次录制时间的 `MM-DD HH:mm` 显示，内部 Run 身份保持；旧记录只做有界只读投影，没有时间证据则显示未知，不以文件修改时间猜测。

2026-11-09 用户进一步确认：直接点击 Run 也先建立相同的 HOLD 控制会话，再进入本轮 MODEL。正常中止或时长结束后保留该会话、冻结参数和 SDK 连接；双踩 1、踏板 2／3 恢复准入，MODEL 内的首脚停止和第二脚忽略规则保持。只有“结束控制会话并封存”或独立全局 Stop 结束整轮控制会话；故障恢复不自动续跑。

- “准备控制与踏板”使用现有 inference_run 的 prepare_only=true，冻结本次模型、seed、运行时长和 ControlPolicy，并建立已准备录制的 HOLD 会话。随后 MODEL、TEACH、HOLD 和原生回零共用该会话及 Run 身份，各次操作使用既有 request_id。保持状态应用参数时结束旧 Run 的取流、应用设置并准备新 Run；SDK 保持连接。旧 Run 参数不变，踏板只使用新准备成功的值。用户显式结束会话后封存录制。
- 踏板 1 请求停止推理并保持。踏板 2 在 TEACH 重力补偿拖动和 HOLD 之间切换。下一次显式 Run 使用新鲜观测与当前保持位置。踏板 3 依次执行双臂 SDK 原生回零，按原生逻辑张开夹爪，正常踏板操作等待其完成。独立 Stop 继续执行撤权与停止确认。
- 原生回零的插值阶段和增益由 SDK 后台反馈／输出线程推进。Owner 持续读取反馈、维护监督心跳并记录过程。SDK 方法返回、目标命令到位和实测残差分别保留证据。
- 用户于本日最终批准关节到位误差、审阅／启动／准备漂移以及静止位置跨度统一为 2°。静止窗口为 0.3 秒。物理关节反馈越界容差为 5°。角度在代码和接口中使用 rad，夹爪使用其已登记的毫米／SDK单位。各业务到位设置及 Infer 准备漂移通过唯一 ControlPolicy 提交并回读。
- 人工拖动收尾使用 set_damping_hold(manual=false)，以实测关节位置和夹爪开口建立合法停止保持目标。取得新鲜静止证据后，Owner 使用现有 begin 和 TargetInterpreter 恢复正常增益并建立固定 HOLD。原始反馈、修正值和实际 SDK 调用进入同一录制。
- 在线 Owner 协议为 workbench.control_owner.v10，连接时精确核验，包含 TEACH 夹爪被动开合及退出后的实测锚定语义；录制生命周期使用显式 ingress_closed。preserve_jaws 是停止保持调用的可选布尔字段，默认 false；需要保留已有夹爪命令的维护调用仍可使用 true，人工拖动结束使用 false 取得新开口，再恢复正常位置保持。其消费者为 SDK 适配器。

WAM 静止恢复 trick 由用户安排在踏板开发、上线和验收之后。其目标入口为默认未勾选的复选框，具体实现与验收另行记录。

文档版本 `c-workbench-interface/2.7-development`。更新日期 2026-11-09，Asia/Shanghai。

当前实测状态（2026-11-07）：ARX13上的E04_2 SO/Torch各5秒总控制诊断均已通过模型输出、SDK参考窗口、实测运动及完整ROS录制核验。任意有限正数秒数输入已发布。证据与实际进程身份见 `.evidence/infer-observation-timeout-20261107-v1/progress.json`；较早带日期章节的“仅CPU/暂停”描述只作当时记录，不代表当前硬件状态。模型精度、长期稳定性和回零残差是独立待办。

## 在线协议统一前置拒绝（2026-11-06 用户确认）

在线服务在连接、配置或依赖绑定时核验当前协议；版本缺失或不匹配立即拒绝，不进入Run，不通过缺字段默认值、旧握手或运行时降级继续执行。当前Owner为`workbench.control_owner.v5`，模型HTTP为`workbench.model-http.v2`，常驻记录服务为`workbench.recording-service.v3`。部署必须同步生产者和消费者；本节不表示候选已发布到真机。

模型HTTP在POST准备前先只读核验服务协议；准备请求必须包含完整bundle摘要，所有模型（含Base379）的worker身份均按完整IO、统计量和运行模式核验。ModelSession沿用其未改变的`prometheus.model-session.v1`，但在获取pin前核对动作契约。公开模型准备/卸载与pin租约操作是两种明确操作，不是新旧协议的回退。

Owner v2保证统一策略等代码契约，客户端不再逐项尝试旧Owner能力。是否配置了常驻录制工厂、是否提供目标数据面属于实际部署能力，连接或配置时另行核验；不等到confirm才发现缺失。独立Stop不依赖模型或记录器的接入核验。SDK已资格的原生API选型保留其显式设备配置，不把设备能力差异冒充在线协议兼容。

ROS数据面保持已批准schema；创建任何节点或设备后端前校验已安装`workbench_interfaces`全部字段、类型及上限，旧消息包直接拒绝。I11绑定原记录器健康来源，Owner、业务/UI直接订阅同一I15 DataEvent；RPC仅回传生命周期、真实进程退出与最终封存证明，不再轮询转发实时覆盖率。无需新增`recording_health_ros_v1`能力标记。

历史录制、checkpoint与持久化设置按各自文件schema读取和迁移，不作为在线服务的旧协议后门。验收优先执行真实ROS、独立Owner、常驻记录器的完整Run与失败用例；删除依赖私有历史样本且持续跳过的冗余测试，不以单元测试数量代替实际用例覆盖。维护触发：在线协议、部署能力或UC/SC契约变化时同步更新本节、目录与开发Map。

本候选的ROS迁移与非真机验收已完成，当前统一收据为[final-nonhardware-receipt.json](../.evidence/ros-migration-20261106-v1/final-nonhardware-receipt.json)。完整用例覆盖四业务、独立Owner两种时钟、直接ROS健康/结果索引、Stop、模型/写盘失败、writer崩溃恢复及断连重连；60秒三路640×480 RGB-D逐帧对账与深度像素核验通过。631个源码/测试文件与隔离CPU快照摘要一致。下文较早检查点的“待装配/待回归”说明仅描述其当时状态，以该收据和当前开发Map为准。真机发布、真实模型执行与G34设备性能预算仍按用户暂停要求保留，不从CPU通过推断真机验收完成。
Owner：C 工作台会话 `01a16327-7545-7332-a361-0123ddad811d`。
父入口：[共享 infer/control 跨会话交接](/Users/wujie/Developer/work/anyverse/_context/notes/shared-infer-control-handoff-20261103.md)。

本文件是 **C 所有的消费者装配、工作台动作入口、共享生命周期配置** 的唯一可维护接口规范。
版本号标识本文，不新增 HTTP/NPZ wire 版本。历史 receipt、candidate.patch 与配置方案保留为冻结证据，不作为第二份可编辑接口源。
接口字段、组件边界或验收状态变化时，本聊天在同一工作轮更新本文并同步当前18930架构站；图引用本源，不维护第二份可编辑规范。历史中控引用不作为本轮发布入口。

当前架构开发由本聊天独立推进，状态发布到 `http://10.17.0.27:18930/#progress`。下文A/B/D分工和18920中控操作为2026-11-03至04的历史续接说明，本轮不向旧协作方或中控派工、发消息。当前仅做CPU开发和验收，不据历史段落推断GPU或真机服务的实时归属。

### 2026-11-07 踏板2人工设置Infer起点

用户要求通过踏板2切换重力补偿拖动与位置保持，作为下一轮Infer的起点。新增同义网页动作 `inference_teach`，请求沿用I08的环境、业务、request_id、runtime_nonce和epoch。原始踏板命令22沿既有单一ROS来源进入同一操作者入口。

当前实现复用已准备模型的正常模式会话，要求Infer持权及normal_modes已配置。首踩先完成活动Infer的HOLD/END和封存，然后创建定位Run，初始HOLD确认后请求TEACH。TEACH使用既有normal_X5零关节kp和0.1比例kd，SDK重力补偿持续工作。2026-11-09按用户确认修订：夹爪kp/kd随同一次增益过渡降为零，允许手动开合；再次请求HOLD时，先使用正常SDK停止保持取得实测开口，再通过begin恢复正常增益和共享保持参考，不拉回进入TEACH前的开口。关节和夹爪增益仍不得超过已绑定原生normal配置。定位期间模型调用数为0。下一次显式Run获取新位置的新鲜相机/state，重新建立执行上下文。

踏板1可结束人工定位到HOLD。全局Stop撤销定位流程，原请求不得自动复活动作。重复request_id保持幂等，模式转换期间额外踩踏只能请求结束，不能叠加新TEACH会话。页面显示进入重力补偿、人工拖动、锚定起点及封存状态。Run入口在位置保持与封存完成后开放。

本次复用现有Owner TEACH/HOLD/END接口及录制流程，发布范围为业务编排与UI。进度与CPU/现场证据位于 `.evidence/infer-pedal2-start-20261107-v1/progress.json`。现场拖动属于人工操作，实际验证与CPU替身记录分别注明。

### 2026-11-09 踏板重连与实时观测供给

同一已配置踏板的USB断开、设备节点重建和重连属于输入连接变化，不重新创建逻辑ROS来源或要求重新准备控制。读取器关闭失效fd，等待原配置路径恢复后重新打开；部署应使用稳定的udev路径。读取器、发布者和递增序列持续存在，source_tick的connected布尔值供操作者输入状态使用；断连清除未完成手势，重连不合成动作、不重放旧输入。既有250ms去抖、0.5秒双踩窗口、指令映射和当前控制状态检查保留。权限或配置错误仍明确报告，不作为USB瞬断无限重试。

实时ROS观测选择直接使用绑定的canonical state中的runtime_nonce、command_epoch和原始采样时间，不为每份观测同步查询Owner控制RPC。首次设备/模型绑定与策略核验仍沿已有生命周期完成；原始来源、代次、夹爪有效性、100ms状态新鲜度及500ms供给检查不变。业务的相机/state接收线程只解析实际消费的控制事件；原始窗口、反馈与所有审计事件仍由独立记录器完整保存，Trick开启时继续逐序号记账。

模型输入的来源引用与变换摘要先按原样落盘。取流关闭后的封存流程以已保存的原始样本核对引用，并按涉及的视频分段逐帧重建、校验实际模型输入摘要；内存仅保留当前解码帧，不能用最新帧替代原引用。实时写入不因短缓存淘汰而从头扫描不断增长的状态日志，也不在取流阶段重复进行模型图像预处理。缺原始帧或输入摘要不匹配仍使最终封存不完整；只有验证结束才能报告完整，ingress_closed不等于封存完成。

维护触发：用户改变人工夹爪行为、踏板重连规则，或输入选择/封存验证实现变化时同步更新本节与开发Map。实际候选与发布证据分别记录，不以CPU通过替代真机验收。

### 2026-11-07 观测接纳修复与任意时长（用户5秒诊断目标）

正常工作台Run的既有 `duration_s` 接受有限正数秒数，网页改为数字输入、支持小数；默认120秒保留。该值在Run审阅和确认时冻结，是MODEL控制阶段中观测、模型调用、准备、等待和运动的总预算，不能解释为累计模型action的源时长。下一整段无法完整放入预算时正常保持并结束。正常工作台的0.5秒也遵守这个墙钟预算，不继续借用0.5作为隐式单段编码；既有显式Raw录制门流程的历史单段约定保留。

Owner在线协议同步为 `workbench.control_owner.v5`，防止旧端在Run时才拒绝新秒数。模型协议及SDK窗口字段不变，记录服务保持v3。实际发布身份、两后端各5秒的模型输出/真实控机/封存证据统一在 `.evidence/infer-observation-timeout-20261107-v1/progress.json`，未通过真实控机不得记作目标完成。

观测与原始state来自独立ROS topic，不能依赖跨topic到达顺序。Owner只保留一个最新待解析引用，原始state到达时再解析该引用；原始样本不存在、身份变化、超龄或已被缓存淘汰时仍拒绝，不替换为最新state。state订阅队列复用已有有界缓存容量，图像队列策略不变。解析与诊断日志均在ROS回调，控制tick不等待DDS或解析。500ms等待、100ms state年龄及其他Guard门槛不变。故意阻塞state回调、观测先到的原生DDS用例证明修复前失败、修复后能调用模型；旧失败Run没有记录解析拒绝原因，不能据此宣称历史唯一根因已被完全证明。

### 2026-11-07 Infer 操作者控制（已批准，发布状态见开发Map）

按钮“停止推理并保持”与踏板1走相同的 `inference_hold` 意图：本次请求只携带已有 `request_id/runtime_nonce/epoch` 及显式环境、业务；检查当前推理控制权后，正常Run走HOLD与END，保留SDK和迟到的模型输出。准备中停止阻止后续confirm，清除本轮审阅；不是失败Run，也不自动续跑。全局Stop仍为独立撤权出口。

按钮“回零并保持”与踏板3走 `inference_home`：按钮额外确认本次场景净空；踏板是前台Infer中的直接操作。等待当前Run保持、结束和封存，再用已有目标编译、Guard和confirm执行双臂低速回零，保持SDK实际夹爪命令，不从测量值推断目标。Home与停止Home后的定点保持均为独立录制Run。完成须同时证明本次计划、新Run身份、到位/静止、同一Run的writer封存退出，不能用旧Run的完成事实。故障不自动恢复。Home途中停止先走独立撤权，再经共享执行链锚定当前实测关节并保持。

部署配置 `inference.operator_controls` 一次绑定 `home_velocity_rad_s/timeout_s/focus_timeout_s`；启用踏板时必须同时提供绝对 `pedal_source_descriptor` 与 `event_max_age_s`。速度仍受唯一ControlPolicy约束，当前部署使用0.1rad/s。新增薄入口 `scripts/serve_pedal_source.py`、受管角色 `pedal`，复用已固定来源/摘要的evdev读取和去抖，不导入Replay的SDK Owner。一个常驻进程发布一份原始DataEvent与SourceDescriptor；原evdev时间在occurred_at中保留，source_tick仅给健康及封存尾边界。Infer和记录器订阅同一来源，不复制topic。退出环境时停止踏板源后才交还数采，避免两个evdev消费者。

网页 `inference_focus` 只绑定现有客户端请求号、Owner身份与显式client_id/active；前台租约有界，失焦撤销，过期事件或错误控制归属不触发动作。回零确认绑定打开时的Owner/epoch，轮询变化不能替换原确认上下文。Owner在线协议为 `workbench.control_owner.v5`，常驻记录服务为 `workbench.recording-service.v3`；生产者与消费者统一升级，旧在线组合在接入前拒绝。

完整CPU用例和实际发布分别记录在 `.evidence/infer-operator-controls-20261107-v1/progress.json`。CPU已覆盖阻塞模型停止、回零、回零中停止、独立录制封存、单一ROS踏板、失焦/过期拒绝及原事件保存；实际现场踩踏不由CPU替代。

### 2026-11-07 Infer 参数配置（已批准并接入）

正常工作台使用既有 `normal_modes` / `normal_initial_mode=HOLD` Run 流程，不使用固定三段的 `finite_chunk_count` 验收配置。WAM 执行长度范围由已绑定动作契约给出，当前为1–32步，默认15；仍通过唯一 `ControlPolicy.execution.execute_steps` 提交，并在下一Run冻结。单段源时长为N/30秒，最长执行时间为N/30×6秒。运行时长沿用既有单段诊断、1/2/40/120秒选项；正常整轮预算包含观测、模型调用、规划、等待和运动，只接纳能完整放入预算的片段，不截断运动尾部。

WAM模型描述升级为 `workbench.wam.v2`，显式携带 `inference_options.denoise_steps` 及模型包声明的 `inference_capabilities.denoise_steps.{minimum,maximum,default}`。本批E03/E04_2模型包声明1–10整数、默认2；公共代码不按checkpoint名字推断该值。checkpoint、统计量、文本及基础配置的制品身份保持，既有 `bundle_digest` 同时冻结所选推理配置。缺少字段、旧在线描述或越界值在准备模型前拒绝。

模型目录提供受包能力约束的配置身份；网页按去噪草稿过滤同一checkpoint，不把所有配置重复列入checkpoint菜单。应用继续使用既有 `prepare_model(model_id, expected_bundle_digest)` 操作，不新增控制RPC。普通 WAM／Torch 在模型、权重、文本、统计量、运行时身份均未改变且仅去噪步数变化时，通过既有 worker IPC 更新采样参数并短预热，保留原进程和权重。新参数经 worker 身份回读后绑定；存在活动模型租约或推理请求时仍拒绝更新。SO 从基础 YAML 生成本次配置并重新加载，热更新适配暂缓。参数在 Run 之间生效，草稿未应用时不能以旧配置执行 Run。Owner 与 SDK 不接收去噪参数，录制保存实际步数及配置摘要。WAM 观测/结果新鲜度2.5秒、实时 state 年龄及其余保护不变。

验证入口为 `.evidence/infer-parameters-20261107-v1/progress.json`：包括实际CPU ROS/Owner/常驻记录器的1/15/32步完整用例、两后端实际GPU1/10步去噪、线上策略提交及模型配置应用回执。候选、实际部署与真机运动验收分开记录；本轮参数发布后不自动执行机器人动作。

### 2026-11-10 推理链路耗时诊断（用户已要求，实际发布按回执）

高频状态与完整历史分开：普通 Runtime 快照不携带模式历史；业务的 `inference_state`
轮询显式使用 `receipt_view=status`，只保留标量处理状态以及已有消费者需要的 native_output／hold_target。
默认显式读取仍可取完整内容，`receipt_view=full/status/none` 在 Owner 校验；实际模式账本、
重复请求的完整返回和录制原始事件均保留。生产者与消费者统一升级 Owner v10，连接时拒绝旧端。
`/api/state` 只返回页面所需当前模式和切换阶段，删除三条展示路径中重复的 mode_receipts，
不修改内部状态或扩大原状态响应上限。短暂忙导致的执行前拒绝记为已知失败、可重试；后端拒绝维护准入时，
须核实原实例已无活动维护再清理票据。失联、丢失回复、释放不明仍保持未知，不自动重放。

夹爪停止适配的实测上限由用户指定为 `Arx5BackendConfig.gripper_feedback_upper_m=0.085`；
只替换该实测上限检查，SDK命令尺度、标定、原到位误差及原始读数不变，不新增投影处理。

只读 `GET /api/inference-diagnostics?environment=physical&limit=128` 返回
`workbench.inference-diagnostics.v1`：`context` 关联当前 Run／模型；`business` 为业务进程测量；
`owner` 通过独立只读 `pipeline_timing` RPC 查询控制进程；`recording` 读取当前和近期录制的
有界 `pipeline-timing.json`。该入口不构造完整页面状态，不调用模型、连接设备或修改策略。
`after`／`owner_after` 分别分页两个进程的事件序号；`limit` 为 0–512，0 只返回统计。
未知参数、重复参数、负游标和越界数量拒绝。生产进程未加载该诊断能力时明确返回 unavailable，
不能把缺测当成耗时为零。

各进程返回 `workbench.pipeline-timing.v1`、独立 `instance_id`、PID、boot_id、单调时钟和采样墙钟。
事件以原始帧 `source_time_ns`／source instance、Observation ID、request ID、Run ID 和 epoch 关联。
`sequence` 是诊断游标，`source_sequence` 是原始数据序号，不混用。每进程最多8192条事件、128类阶段，
每阶段分位数只统计最近256次；次数／均值／最大值自进程启动累计。环形覆盖由 `cursor_gap` 明示。
真实供给失败保留最近4个故障窗口，每个至多2048条先前事件；可按故障的 oldest_sequence 分页读取。
进程重启会清空内存窗口，录制文件中的测量保留；需要跨重启保留时应先导出该只读响应。

| 阶段 | 测量边界与限制 |
| --- | --- |
| 相机采集与发布 | 使用源声明的 timing_topic 和同机 boot_id，关联原始 frame clock；分别测量 wait_for_frames、接收后至整组RGB/IR发布返回。不是传感器曝光到主机的硬件延迟。 |
| 传输与回调 | 发布返回至同帧回调包含DDS传输、排队及线程调度；发布和消费可能重叠，signed_gap_ns保留负值，不能解释为负传输耗时。运行时提供RMW接收时间时另测中间件到回调；未提供时不编造。 |
| 接收与配对 | 分别记录publisher查询、解码、缓存写入、观测锁等待、配对、引用解析、G02/G03/G04实际裁决及发布；源时间和配对规则不变。 |
| Owner | 分别记录观测到达、引用解析、G02/G03裁决、tick间隔及执行耗时；与原始Observation ID关联。 |
| 模型 | 分别测量回调派发、源引用解析、预处理及输入证明、NPZ编码、HTTP往返、解码和结果发布；WAM返回同机worker的准备／GPU同步执行／回传／反归一化时间点。旧duration-only数据明确标记为回包锚定，不能当精确阶段时间点。 |
| 轨迹与SDK | 转换校验、参考轨迹及制动余量准备分别记录；SDK调用保留排队等待、backend耗时和返回总时长。SDK软件返回不代表电机物理到位。 |
| 录制 | 队列等待、编码写入、journal刷新、逐流校验、模型输入证明和回放导出分别记录；封存阶段写入原Run的有界诊断文件，不读取大权重。 |

统计只用于诊断，不参与新鲜度裁决、不修改原时间戳或用新帧替换引用帧。事件只含有界标量与身份，
不含图像、动作数组、命令行或凭据；模型会话能力串在传输测量中还原为原请求ID。诊断文件写入失败
只计诊断错误，不改变录制完整性裁决。实际原生ROS验证与部署入口见开发Map的告警与测量诊断职责域。

```mermaid
sequenceDiagram
    participant C as Camera publisher
    participant D as DDS and executor
    participant B as Business inputs / pairing
    participant O as Owner
    participant M as Model worker
    participant S as SDK
    participant R as Recorder
    C->>C: wait_for_frames → receive → RGB/IR publish return
    C->>D: Original image + frame clock (same source stamp)
    D->>B: Callback entry → decode → cache
    B->>B: Latest common 30 Hz selection → original-state age guard
    B->>O: Observation ID + original frame/state references
    O->>O: Resolve references → G02/G03 → zero-speed supply deadline
    O->>M: Request ID + Observation ID + Run/epoch
    M->>M: Preprocess/proof → HTTP → worker/GPU → decode
    M->>O: Same request ID, action and stage timings
    O->>O: Validate/convert → reference + brake reserve
    O->>S: Submit registered window; measure SDK software return
    C-->>R: Independent original-frame subscriber
    O-->>R: Original control events and mode boundaries
    M-->>R: Model input/result references
    R->>R: Queue → encode/write → verify → playback export
```

### 2026-11-08 Checkpoint 目录发现（已发布并验收）

Infer 启动 registry 使用唯一绝对路径 `catalog_root`。每个包位于
`<catalog_root>/<checkpoint>/<backend>/manifest.json`，复用现有 registry 的模型身份、资产根、
文件大小和制品身份、统计量、任务文本、模型配置、运行时及去噪能力字段。目录后端与manifest声明须一致。
目录模式排除同时配置 `additional_registries`。manifest须为当前用户所有的私有普通文件，
包目录链接及越界资产明确拒绝。

目录模式的启动选择只需 `catalog_root`、`model_id`、`backend` 和可选的
`selected_inference_options`。资产位置及默认推理配置从所选manifest读取。已配置的初始模型
缺失、不兼容或身份重复时，服务在启动阶段拒绝该配置。

扫描读取有界元数据并检查资产存在及大小。WAM同族checkpoint复用通用描述与既有Torch/SO加载器，
支持现有已核验Pi加载器的包继续使用Torch/Triton。未支持的模型族、后端、缺失文件、重复身份及无效
manifest显示具体原因。加载操作复核所选描述、文件大小与必要的小制品内容，旧目录摘要失效时拒绝该准备请求；大型权重不做例行全量哈希。

2026-11-10 用户批准取消权重 SHA256 的硬性依赖。已有摘要继续使用 `workbench.wam.v2`；
无摘要的包使用 `workbench.wam.v3`，明确令 `artifacts.checkpoint.sha256=null`，由
`checkpoint_source.{receipt_sha256,bytes}` 绑定真实的小型来源回执与权重大小。
部署 registry 的 `checkpoint_path` 必须与 `workbench.model-source.v1` 回执中的模型ID、step、
路径及文件大小一致。仅大型权重文件允许无摘要；统计量、文本、配置、来源回执与运行代码仍校验真实摘要。
目录负责可见性与元数据检查，加载器负责来源一致性和实际加载，worker身份原样报告来源信息，
业务模型绑定及录制保存该描述；Owner只消费动作契约和既有绑定身份。元数据校验不声称证明权重字节完整性，
Torch严格加载与SO固定引擎加载的证据分别保留，禁止伪造摘要。

业务侧每5秒自动刷新目录，现有“刷新 checkpoint”入口保留。目录轮询独立于模型健康读取。
新增、删除或替换包只更新候选列表，当前已加载模型继续使用绑定时的冻结描述。UI按模型家族、
后端和去噪配置展示候选，并展示不可选条目的原因。

首次准备和显式卸载后的重新加载都按当前manifest解析资产位置。注册内容相同时复用原客户端，
资产位置改变时重建客户端。已加载模型保留冻结描述。

本轮标准CPU检查、目录与模型会话检查，真实资产40项HTTP发现及线上页面核验见
`.evidence/infer-pedal2-start-20261107-v1/checkpoint-discovery-acceptance.json`。
受控发布与目录动态增删回读已完成，过程分别保存在同目录的 `catalog-publication.json` 和
`catalog-directory-acceptance.json`。维护触发为目录布局、加载器支持范围、刷新行为或
加载验证契约变化，同步更新I03、开发Map和验收回执。

### 2026-11-06 已批准的 ROS 数据面迁移（实现中）

用户已批准将观测、模型输入输出、控制事实与录制数据迁移到 ROS，并要求完成全部不依赖真机的验收。专用消息包为 `ros/workbench_interfaces`；共享收发与源缓存目前定位于 `robot_workbench/ros_data.py`、`ros_inputs.py`。进度入口为 `.evidence/ros-migration-20261106-v1/progress.json`，字段和源共享约定为同目录 `contract.json`。旧章节描述迁移前实现，不能据旧 CPU 回执宣布 ROS 迁移已经完成。

每个原始图像 topic 仅由原相机源发布，使用原 `sensor_msgs/Image`；每份实测 state 仅由唯一 SDK 适配器发布。输入管理、模型消费者和常驻记录器分别直接订阅同一源。`Observation` 与 `ModelInput` 引用 `StreamRef`，不复制原始图像到第二条 topic。`StreamRef` 复用 stream、source instance、source clock、原始时间戳和源序列。对于原 Image 没有独立序列的来源，原 header stamp 作为帧标识，不能伪造订阅者局部序列或宣称已知上游完整性。

用户已批准保留 fake RGB-D：每路RGB及深度各自使用一个原始标准Image topic，来源描述通过可选`metadata.depth_stream_id`关联；深度声明`pixel_format=depth16le`、既有`depth_scale_m`和`depth_aligned_to`。同一仿真帧的两路数据使用同一个原始header时间戳，不新增配对身份。模型仅消费RGB，预览和记录器按原时间戳取得RGB与深度；记录器继续使用既有无损RGB-D分段格式。该契约已批准，来源启动及各消费者接线仍在实现，未据此声明深度路径已通过验收。

fake原始发布源由既有PhysX进程直接承载，不再通过旧相机IPC转发原图。用户已确认其相机启停只管理ROS发布源，保留仿真及控制进程；沿用`SourceDescriptor.available`表达源活动状态，重启更新`source_instance_id`并要求消费者重新绑定。PID继续如实表示PhysX进程存活，不能把发布源停止写成进程已退出。常驻记录服务仍保持独立CPU进程和原有用途故障策略。此变更仅用于fake源生命周期，不能据此扩大真机操作授权。

用户进一步批准默认fake启动统一读取冻结契约顶层`data_plane`，复用`workbench.ros-data-plane.v1`的namespace/domain_id。缺失时在构造Isaac/设备前拒绝，不选隐式ROS域。原PhysX进程内的Runtime由既有`ControlOwnerService`暴露，业务使用既有`ControlRuntimeClient`；现有CPU编码子进程照常受管，不新增GPU plant或模型。`serve_physx_device.py`已接入此路径并通过CPU启动检查；规则数采连接仍需去掉旧目录依赖，不能据此声称完整fake业务已完成。

fake业务读取已发布的仿真控制/采样时钟，不以host monotonic代替或外推；来源时钟一致时保留整数`source_time_ns`，反馈引用使用同一时间戳。相机源停止回执明确`source_available=false/process_preserved=true/exit_verified=false`；运维核验源停止与进程退出分别处理。

用户已批准通过I17的`metadata.metadata_topic`声明同帧元信息通道，复用`DataEvent(event=frame_metadata)`及原RGB的`StreamRef`，不增加配对身份。载荷仅含原始`host_receive_ns/physics_step/frame_number`及采样时`owner_available/owner_pid/command_epoch/runtime_nonce`，不重复像素或关节数组。订阅者严格按原帧引用合并，不能把元信息接收时间写成采样时间；缺失元信息按用途报告缺口，身份变化撤销绑定。

规则数采连接配置一次绑定已有`data_plane/control_owner_descriptor`；ROS模式不再要求`recording_directory`。`RealBridge`通过共享ROS输入读图像，通过既有Owner客户端解析原始state引用；授权控制及浏览器预览仍保留现有入口。`real_bridge.py/real_observation.py`的实际职责与文件定位已补入开发Map，完整装配验收进行中。

2026-11-06用户批准将新录制策略的`recording.resource_budget.event_queue_bytes`默认值和上限调整为128 MiB，消息上限仍为512 KiB。已有持久化64 MiB策略保持原值，只有新策略或显式提交后才采用128 MiB；控制门槛及按用途的失败动作不变。依据是三路RGB-D各16帧已约70.3 MiB，超过此前RGB阶段的64 MiB预算；此前段落中的64 MiB是历史默认值。

用户已批准补齐通用物理RGB-D来源：`serve_camera_sources.py`仍管理原来的三个逐相机采集进程，`real_rgbd.py`在ROS模式直接发布原RGB与对齐深度，不再将原始像素送入CommonCameraService。复用冻结profile中的`data_plane/robot_id/cameras`，每次源进程启动生成原有source实例身份。I17元信息仍引用相同RGB StreamRef；物理载荷仅包含`host_receive_ns/frame_number/depth_frame_number/device_timestamp_ms/depth_device_timestamp_ms/timestamp_domain`，不附加fake的physics或Owner字段。内参、深度比例及对齐关系在来源描述中绑定，运行中改变须重绑；退出发布unavailable。运维读取已登记物理profile对应的同一ROS来源，录制活动/封存期间仍禁止相机复位。ARX6外部Prometheus发布器不在本次源码修改范围；本轮只以CPU替身验证，不部署或接入真机。

已批准将现有`serve_control_owner.py`、`serve_physical_device.py`及`workbench_deploy.py`补入开发Map的“服务启动、归属与部署”职责域，并关联既有启动／发布追溯。此项只补齐实现定位，不新增组件、接口字段或控机权限。

模型专属预处理产生的 state、变换描述或不可重建的派生输入单独声明，不冒充原始样本。记录器按源引用重建模型实际输入并核验；缺帧、源替换或引用已淘汰时报明确缺口，不能改取最新帧。源 topic、消息类型、设备与实例在绑定时确认；ROS 发布者改变要求显式重新绑定，无文件、旧缓存、其他设备或 fake 自动回退。

模型请求与结果复用已有 request、Run、epoch、session 与 bundle 身份。控制事实、公共 Guard 裁决和录制健康只发布一份，消费者分别订阅。常驻记录服务是生产产物的唯一写入者，结果索引引用其封存清单；业务进程不再先写模型 IO 后要求记录器复制。

SDK 命令授权与接纳、独立 Stop、恢复和释放回执保留原控制接口；公共 Guard 仍在控制本地评估。录制 prepare/bind/finish 与最终序列屏障保留，不用普通广播替代操作完成证明。所有现有运动阈值和按用途固定的录制失败策略保持不变。独立 CPU 验收使用 3090 的 ROS 2 Humble、localhost-only 域，不接入 ARX6、不启停真机、不调用 GPU 模型。

当前接线遵循上述分工：`InferenceWorkflow` 从原始来源的有界历史进行同一30Hz最近时间配对，发布不含像素的 `Observation`；`OwnerInference` 仅订阅state和观测引用，复用原夹爪换算、G02/G03/G04及ActionChunk身份/新鲜度校验，直接通过ROS发出模型请求。模型侧使用同一观测引用重建原始帧，适配器仅调用一次，已适配输入直接交给既有模型服务传输层。模型种类不再选择不同采样通道。

源声明采用每条流独立的 `sources/<stream_id>` retained topic，depth=1，防止晚加入者先看到旧的不可用声明。原始图像订阅在绑定时核对唯一发布者及其实际QoS：可靠源采用可靠接收，best-effort源保留其语义，不以一律best-effort订阅放弃可靠源的传输保证；state要求可靠源。DDS接收队列与选帧历史容量独立，输入默认接收深度4、记录器16、预览1。采样时间与订阅接收时间分别保存；SDK读取期间发生撤权时保留读取开始的epoch，不能给旧样本换上新epoch。仿真使用原始仿真源时钟。

传输可靠不等于硬件采集从未丢帧。完整性验证需声明源端序列的可观测范围；全分辨率合成负载会将每个实际发布的图像时间戳与落盘索引逐个对账，不能仅凭manifest封存成功宣布帧完整。健康中的 `data_queue` 表示ROS源数据待落盘队列，区别于既有生命周期IPC的 `parent_queue`。

已配置数据面以 `workbench.ros-data-plane.v1` 的namespace/domain绑定，Owner通过既有能力列表声明 `ros_data_plane_v1`；业务端未见能力则在配置阶段拒绝。ROS路径的 `control_inputs(include_feedback=False)` 只取限位、健康和运行身份等准备元信息，不携带另一份反馈。未声明ROS模式的历史协议测试仍显式走旧路径，不作为新版生产资格。

`CaptureRun` 继续承担准备、绑定和结束屏障的业务适配，ROS模式不接受数据事件注入；常驻记录器直接订阅原始state/图像、模型输入引用、模型输出及控制事件。模型生产者和控制生产者分别提交最终watermark，连续原始流按自身时钟提交结束点。ROS路径不创建 `InferenceEvidence` 或 `RunArtifacts` 写入者，结果列表从记录服务生成的既有Run receipt建索引。该实际Run接线正在验证；尚未完成所有业务的默认启动切换、旧在线路径清退及完整故障/持续负载验收，不能据组件测试声明迁移完成。

四业务录制适配现已接到同一 `CaptureControlSession/CaptureRun`：标定、规则数采和研究通过 `capture_control(..., ros_runtime=runtime)` 绑定原始来源，Infer使用同一Run适配器。业务端只提交生命周期和最终watermark，控制事件由唯一Owner发布；记录服务生成Run回执，业务只读取并更新结果索引。该分支由明确的 `data_plane` 配置选择，默认启动与旧入口清退仍待完成。

ROS模式的推理事件只经一条发布路径；UI指标和记录器分别订阅这份事实。控制事件中的反馈使用原state引用，预测事件中的原始动作和历史分别由ModelResult与ModelInput持有，避免再复制一份。源发布入口把SDK/NumPy数值标量规范化为消息定义的数值，保留原数值、采样时刻和epoch；不以序列缺口代替序列化错误。

ROS模式的Owner查询回执携带 `feedback_reference` 和 `control_owner.data_plane`，不再传输另一份 `calibration_feedback` 数组。业务端的 `RosStateView` 按该引用解析已绑定的原始state，并在本地恢复既有UI/标定读取格式；样本缺失、淘汰或Owner不符时报告 `feedback_source_error`，不改取最新样本。此处恢复的是兼容字段，不是第二条反馈数据通道。原始时钟身份保留在 `ros_reference`；物理旧读取格式仍用其原有host-wall词汇。

`serve_physical_device.py` 和 `serve_control_owner.py` 的生产入口使用 `read_profile(..., require_ros=True)`：要求显式数据面、统一常驻录制，拒绝同时声明旧 `native_rgb` 文件源，并在构造设备后端前核验ROS消息依赖。发布封存还要求Owner/业务配置的数据面相同。旧解析默认值仅供既有库调用与明确的协议CPU测试；不能据此绕过新版生产入口。此约束尚未部署ARX6，fake原始来源启动与最终故障/负载验收仍待完成。

录制进程启动与来源就绪分别判断：`data_required`等待所有必需原始流的首样本后才返回准备完成，异常仍按G35/G38处理；`infer_debug`保留已批准的缺口策略。若结束屏障无法确认，允许保存已接收的数据前缀，但回执明确不完整；不伪造最终序列，不重放动作，writer完成封存后退出。当前已增加完整CPU Run、计算中Stop、三类非Infer实际控制入口，以及缺失最终屏障的验证；范围与失败原件见本轮进度目录。

### 2026-11-06已批准的跨业务录制接线（开发中）

用户已批准将同一CaptureSession接入标定、规则数采和研究Run，并新增专用能力标记。`recording_backend=capture_session_v1`复用既有配置语义；配置此模式的非Infer客户端在确认执行前要求`control_capture_v1`，不能把旧Owner的`capture_session_v1`（Infer事件能力）当作证明。显式null保留旧实现／协议CPU测试兼容，不能声明通用录制资格。能力沿原snapshot/协商通道发布，无新增RPC或SDK窗口字段；能力存在仍不能代替实际来源就绪。

| 数据 | 生产者与频率 | 消费者及用途 |
| --- | --- | --- |
| `recording_preparation.business_kind/control_policy` | Owner在既有Run建立后绑定一次 | 现有录制工厂和SC05，选择既定用途并冻结同一I01策略 |
| 既有`execution_instance_id/execution_owner` | 复用Run及PID/nonce/epoch | 准备撤销、相机租约和结束回执关联，不另造身份 |
| `record_event(name, payload)` | 共用MotionRunner／TargetInterpreter的可选有界回调 | 保存计划绑定、窗口摘要与裁决；SDK实际参数、返回及原始反馈由唯一出口保存 |
| 既有`plan_ref`／`execution_timeline_ref` | 已核验的I04制品引用 | 记录子进程复制和校验原始计划、来源、绑定与执行时间线，不在Owner读取大文件 |
| I15 `guard_decisions/coverage_failed_conditions` | 封存时汇总原有失败、缺口、屏障事实 | G33/G36/G37回执和结果读取；原始issues、数据字节及控制终态分别保留 |

非Infer Run在Owner接纳原资格后异步准备记录资源。准备期间保持原反馈与独立Stop路径，新的增益配置和轨迹输出等待准备完成；不会把记录器编码、写盘、散列或IPC等待放入控制循环。保留原已接纳审阅年龄，不能因资源准备跨过旧TTL而伪造新的审阅时间。规则数采复用同一记录会话，后续路点不另建writer。完整性用途的资源失败由同一Guard裁决并排队撤销输出；调试用途可继续并标记缺口，不能放宽运动保护。

控制事件`source_time`表示原控制时钟下的发布时刻；原反馈`sample_time_s`、SDK时间、host读取区间和wall时间仍在数据体中原样保存，不能将发布时刻当作新的传感器采样时间。相机继续使用原source clock和帧时间。已绑定记录租约可独立完成Stop后的尾帧封存，执行撤权不授予新控制权，也不允许接管其他记录租约。

G33/G36/G37在seal阶段只报告留存事实，不反向触发已结束运动；必需数据的运行中故障仍由G35/G38及生命周期执行拒绝／停止。writer真实退出后，结束流程只允许恢复数据前缀与封存，不重放动作。`manifest-rNNNNNN.json`和`receipt-rNNNNNN.json`保留不可变版本，`manifest.json/receipt.json`提供最新回读；旧封存版本不覆盖。配置、健康、控制终态和录制完整性分别表达。

I15补齐`producer_publish_cost`及队列`oldest_age_s`观测，不新增性能拦截阈值。前序2,000条CPU反馈突发测量的发布p99约68µs，不代表三路真实视频负载或真机控制预算。公共门槛基线的标准入口已分段覆盖2,268次单测执行（2,256通过、12环境跳过）；通用录制核心57项与控制适配器11项Linux CPU检查另有匹配源码回执。此后实际Run接线仍在验证，不据上述增量宣布P1—P4全部完成或真机已发布。

## P1开发边界与已批准归并（2026-11-05）

用户已确认当前架构设计工作台符合预期，并授权开始开发。开发期间，Python文件以上层级的架构、接口、用例变更必须先提交具体差异并取得用户同意。文件内部实现由本聊天决定。开发Map的状态、证据和下一步按实际进展同步更新。冻结设计、源码对照及续接状态统一位于 `.evidence/architecture-p1-20261105-v1/progress.json`。

P1只读核对了ARX6已部署的92个Python源码文件，均与既有部署快照一致。本地原有16个差异、2个缺失文件。用户分两次批准恢复 `raw_state.py` 和 `raw_replay.py`、补入SDK反馈与轨迹准备文件映射，以及按原部署契约补齐I07/I04接线。其他部署差异按具体接口与功能范围继续核对。

I07增加可选 `raw_state`，由可信profile的 `raw_state_enabled` 布尔开关启用。字段包含每臂原始q/dq、effort、同一q样本的原生FK、SDK时间、host getter时间区间、单位、有效性及不可用原因。每臂getter串行读取，各自SDK时钟独立，host时间区间标识接收调用范围。可选力矩或FK缺失明确标记，q/dq和原始反馈时间仍执行现有有限数校验。实际启动接线为 `serve_sdk_control.py` → `hardware_backend` → `raw_state.read_snapshot`。

I04恢复原 `preview_workbench.sampled_trajectory` 入口，schema为 `workbench.raw-replay.v1`。输入绑定device、source摘要/时钟、原始毫秒时间戳、14轴目标、夹爪映射、倍率及预算。关节单位rad、夹爪输入为device mm，准备后转换为SDK单位。现有部署约束包括前2源秒、2至513个样本、单调时间、源间隔上限不超过0.1秒、倍率0至1、执行预算上限30秒、速度不超过1 rad/s及当前实测保护、准备加速度5 rad/s²。原样保留时间、末样本和初始化保持语义。已准备参考经 `control_reference.preview(..., prepared_reference=...)` 进入原审阅、确认和唯一执行出口。推理规划策略与本项Raw回放的既有约束分别登记。

`runtime.py` 内部实时速度检查改为直接读取当前已提交业务的速度策略，默认策略在Owner创建时从既有defaults取得。状态快照和准备路径仍使用原设置接口。设置更新、恢复、业务交接、速度保护、Stop以及Raw反馈/回放相关CPU回归共121项通过。策略读取与关节速度检查的CPU局部基准由约14µs降至3–4µs，限本机微基准范围。运行时实际发布和真机资格由后续相应回执确定。

### I06按消费者和变化频率传递

用户已确认I06字段分工，并要求将接口审查三条规则写入本仓库 `AGENTS.md`。当前候选使用 `workbench.sdk-execution-context.v1` 能力协商。Owner保存Run身份、逻辑epoch、完整规划策略及其业务关联。SDK worker通过既有RPC授权代号核验当前命令资格。

`bind_execution_context` 在执行资格建立时确认 `controller_profile`、`jaw_calibration_id` 及声明的 `limits`。limits仅包含关节速度、关节加速度、夹爪速度三个实际执行边界，来源为公共策略投影。既有未声明新规划策略的路径沿已登记适配器默认检查。原有SDK位置插值对照模式的 `velocity_source` 同样放在绑定配置中。worker核对已安装配置、夹爪单位转换和原生速度能力后返回 `{bound:true}`。

同一generation的绑定内容固定，重复请求幂等返回。SDK代理复用已确认绑定，配置变化先更换generation。每次新审阅执行也撤销先前SDK输出授权。Stop、worker替换及其他撤权之后，必须重新绑定。绑定请求携带捕获时的外层generation，绑定期间撤权会拒绝后续窗口。

逐窗口RPC外层继续使用 `id/generation/deadline`。窗口体仅携带 `reference_id/active_arm/host_times_s/q_sdk14/dq_sdk14`，实际存在额外力矩时携带 `torque_bias_sdk14`。省略力矩字段表示全零额外力矩。worker从本次已绑定配置解释窗口，SDK轨迹帧继续使用原单位和原生API。`reference_id`保留一份，用于原生接纳回执及末端保持来源关联。逐窗口传输中的完整policy、policy摘要、Run实例和重复generation均已移到各自需要的层级。

Owner对模型参考与当前Run策略进行核验，worker就地执行形状、时限、位置、速度、加速度、夹爪步进及当前输出衔接检查。策略绑定仅使用已安装原生能力元数据，窗口接纳仍读取真实当前输出。业务调用方在Owner能力声明中要求 `sdk_execution_context_v1`，旧worker在完整规划路径的准备阶段明确拒绝。传统调用的兼容范围由相应版本与模式检查控制。

I06候选验证覆盖真实本地子进程IPC、重复绑定复用、缺绑定拒绝、绑定中Stop、旧generation拒绝、配置更新、worker重建、冗余字段拒绝、原生边界和夹爪单位。完整片段实现同步归并既有默认15步、1至60步配置、源时长乘4的单段预算及段间零速供给流程。保留当前录制回调和独立夹爪接口的既有范围。相关过程证据位于 `.evidence/architecture-p1-20261105-v1/`，最终资格以该目录的当前进度及验证回执为准。

### I01统一策略提交与运行快照

用户已批准由Owner维护唯一策略状态。`revision`用于编辑并发检查，`policy_hash`为`workbench.control-policy.v1`及规范化参数的SHA256；不再增加`policy_id`。设备、环境、模型、标定和授权身份复用既有运行上下文，不逐窗口复制完整策略。

读取入口为`GET /api/control-policy?environment=...&business=...`，返回`revision/policy_hash/parameters/origins/editable_paths/update_allowed/update_reason`。提交入口为`POST /api/actions/control_policy`，复用HTTP层`environment/business/request_id`，策略负载仅为`expected_revision`与`changes`。`changes`是非空的点分字段映射，例如`{"planning.velocity_limit_rad_s":0.5,"execution.execute_steps":30}`；只有读取结果声明的`editable_paths`可修改。Owner RPC分别为`control_policy_snapshot`和`control_policy`，后者的负载严格限定为上述两个字段。

当前参数包含实测速度保护`measured`、跟踪`tracking`、到位`arrival`、`execution.fake_error`及下述`recording`策略。推理业务增加`execution.execute_steps`，默认15、范围1至60；已绑定的模型配置可提供`planning`、`observation.maximum_age_s`和`model.reference_supply_timeout_s`。未提供的能力不作为可编辑字段。规划参数支持关节速度至2 rad/s、加速度至40 rad/s²、夹爪速度至40 mm/s；实际配置由ControlPolicy提供，physical规划速度不得高于启用的实测速度保护。普通在线Infer使用`max_execution_duration_s=null`，按速度和加速度约束重定时。完整片段及制动预留受本轮总时长约束。观测有效期与供给期限使用各自的ControlPolicy字段。通用生命周期其余策略继续按对应契约接入。

更新先认证业务、核验持权/空闲和`expected_revision`，完整校验后原子替换存储，再提交内存状态并撤销旧预览；写盘失败保留旧版本和摘要。写盘期间禁止新动作进入，但不占用Owner控制锁或HTTP Stop路径。新Run冻结`revision/policy_hash/parameters`到已有执行上下文，既有Run不得在执行中修改策略。唯一兼容例外是其他业务空闲持权时修改下一次infer执行步数，该操作不改变当前持权业务的有效参数。

持久化继续使用原execution-settings文件路径，首次统一提交升级为`workbench.control-policy-store.v1`，保存设备/环境、版本、业务参数及逐字段来源。旧`execution_settings.v1`可直接读取；旧参数写入入口迁移后投影到同一存储。推理步数旧文件只用于首次迁移，迁移后以Owner为准，重新绑定模型不会用旧默认步数覆盖已提交值。`origins`记录默认、当前推理绑定、旧持久化设置或API更新的直接来源，不替代模型/部署本身的来源回执。旧代码回滚前须导出兼容的`execution_settings.v1`；本轮尚未迁移真实设备存储。

CPU证据覆盖版本冲突、字段校验、写盘失败、恢复、旧入口兼容、Run冻结、较低规划上限和不阻塞Stop的写盘过程，索引见`.evidence/architecture-p1-20261105-v1/progress.json`。CPU通过与ARX6发布及真机验收分别记录。

### I05公共裁决第一批接线

用户已批准将`capture-guard`文件定位补到`execution_settings.py`，将准备期间关节/夹爪漂移纳入I01策略，并在I05裁决中增加`checked_time`。新增策略字段为`execution.prepare_drift_joint_rad`和`execution.prepare_drift_jaw_mm`，默认分别为0.01 rad和2 mm；两者必须为有限正数，沿原始模型配置迁移，下一Run生效。录制用途裁决在同一模块接入，不由记录进程自行决定控制动作。

第一批`Guard.evaluate(signal, phase, policy)`覆盖G02观测年龄、G11关节漂移、G12夹爪漂移和G14实测速度。`signal`只提供`rule_id/signal_id/observed/source_time/checked_time`及已有`run_id/epoch/request_id`上下文；阈值从已冻结的`policy.parameters`读取。调用方负责提取原始反馈、单位转换与传入真实检查时刻。`policy`复用I01的`revision/policy_hash/parameters`，不在实时检查中重算摘要或复制整份策略。

裁决返回既定规则、阶段、信号、观测值、阈值、单位、比较式、策略身份及时间。`verdict`为`pass/reject/observe`，`severity`为`info/warning/error`；关闭的实测保护只记录`observe`，不会更改其阈值。G02要求年龄介于0与上限之间，其余三项按绝对值不超过阈值判断，等于上限继续允许。拒绝建议分别为`REJECT_INPUT/REJECT_PLAN/FAULT_STOP`，通过为`CONTINUE`；原调用方沿原状态机执行保持或停止，Guard本身不下发设备命令。

Owner按业务和策略修订缓存输入策略，实时速度检查不写盘、不重新散列策略；超速回执携带同一裁决。推理结果接纳记录G02/G11/G12裁决，`source_time`为对应信号采样时间，`checked_time`为实际检查时间。有限段未来下发时刻的年龄预算与真实检查时刻分别保留，不能把计划下发时刻写成已发生的检查。其余门槛及录制G35/G38仍在后续迁移范围，不将第一批接线称为全生命周期归并完成。

G35/G38的用途裁决纯函数已具备：输入为已核验失败条件数，`policy.parameters.recording.purpose_profile`仅接受`data_required/infer_debug`。无失败返回通过；存在失败时，完整性用途分别要求拒绝准备或受控保持，infer调试返回warning与CONTINUE。该用途策略只覆盖记录链，不能使G14超速转为允许。实际录制健康生产、调用方处置和全流封存仍等待CaptureSession接入。

### I10共同操作回执

沿用`robot_workbench.operation.v1`及`job_id`作为操作身份，不另造`operation_id`。请求仍为既有`action/request_id/expected_target/arms_supported/parameters`，客户端不能注入用例身份或任意命令。可信操作注册可声明`use_case_id`，该身份表示本操作参与的用例，不代表整个用例已完成；历史或尚未精确映射的操作保持缺省，不从动作名称猜测旧回执的用例。

回执保留原`status`和读写入口，增加归一化`outcome`、`unknown_reason`与`allowed_next_actions`。`timeout_unknown/interrupted_unknown`统一投影为unknown；preflight和verify分别呈现checking/verifying。允许的后续动作目前仅为只读`inspect`，以及仍在执行且注册为可取消时的`cancel`，对应既有查询和取消接口。不因失败或超时自动提供重放、释放或重启操作。

每个阶段事件保留原phase和wall时间，同时记录实际monotonic开始/完成时间和该阶段的detail证据。完整回执持久化后，轻量状态保留同一用例和结果字段。读取已结束的旧回执只补内存投影，不重写原文件；重启发现原在途请求仍沿原逻辑记为unknown并禁止自动重放。精确UC/SC步骤对齐、跨重启只读核验和恢复入口将在P2接入现有运维编排，当前保留明确缺口。

### I11/I12/I15 CaptureSession最小传输契约

2026-11-08 已发布的生命周期增量（限定验收见 recording-lifecycle-acceptance.json）：I11/I15 增加布尔
`ingress_closed`，由原 writer 在原始来源订阅已关闭、尾部边界已核验或明确记缺、已接纳数据
及分片 flush/fsync 后发布。该事实沿既有 Run、capture_session_id 与 writer 身份回传，
不新增身份或控制命令。缺失、未知或落盘失败不能解锁下一 Run。

Owner 和业务只在控制保持、旧 Run 取流关闭后允许下一次准备；序列完整性核验和回放导出
可以继续后台运行。`ingress_closed` 不代表 `sealed` 或 `writer_exited`，正常停记录服务仍要求
全部 writer 完成封存并退出。旧 Run 的迟到错误、封存回执和索引归属原 Run，不改变新 Run
的控制和指标。在线生产者和消费者同步升级，连接时拒绝旧组合，历史产物 schema 不变。

保持状态应用执行步数、去噪配置或运行时长时，按已有 request_id 串行执行旧 Run 结束取流、
原参数接口应用和新 Run 准备。执行步数仍核对 ControlPolicy expected_revision，模型仍绑定
精确 bundle_digest；Stop 撤销未完成的重新准备，任何失败都保持设备且不续跑旧动作。
UI 可编辑草稿并区分正在应用、准备成功与旧录像后台封存，踏板只在新 HOLD 准备成功后启用。

WAM-MEM 同样允许在 HOLD 下将执行长度更新为 1–32 步。模型仍返回完整 32 步预测，
Owner 只规划并执行前 N 步，再使用新观测推理；未执行的预测不作为动作写入记忆。
参数更新沿用上述 Run 更新流程及同模型／设备的记忆租约复用，不加载权重或重接 SDK。
WAM-MEM 去噪能力以已登记制品为准，当前支持 1–10 步、默认 2 步。仅更新步数时保留
权重、worker、同一独占记忆租约及其时钟／revision；不执行会重置记忆的合成预热。
`workbench.wam-memory-phase.v4` 的 leased `/prepare memory_sampling` 核对原绑定和新模型描述，
只接受 `inference_options` 变化，拒绝未结束请求、其他活跃任务或过期租约。服务与客户端原子
更新模型绑定，下一 Run 按新配置准入；旧绑定不能提交迟到推理。人工观测尾部先在原配置下
串行提交，更新不清空记忆或把未执行预测写成动作。SO 接入暂缓。

### 统一真实观测记忆与回放配置

v4 统一真实数据写入协议已通过隔离验收。Owner 在真实反馈上附加 `workbench.memory-activity.v1`，按反馈源时间和
command epoch 标记 MODEL_EXECUTING、TEACH 或等待／保持。业务端仅在前两种阶段推进
统一有效样本计数，选择第 0、32、64……个真实观测写入记忆，等待和切换期间保留实际时间差。
首次启动的真实观测建立初始记忆。相邻实测状态生成 q-next GT 审计，缺帧和等待不补造动作。

已核对的 T01、T02_1 训练快照均将历史 executed-control 条件掩码设为 false。适配器遵循
各制品原有归一化，将真实 RGB、当前状态和 episode 相对秒送入原 Policy context。
GT 审计通过 `memory_sample` 随 ModelInput 留存。预测调用复用请求级冻结 context，生成的
未来动作及重复查询均不提交持久记忆。真实写入按时序串行，显式清空以 revision 和 generation
隔离旧请求，参数更新沿用同一记忆租约与有效样本计数。

显式结束控制会话通过 `cancel.end_test=true` 表达测试结束。控制停止确认后清空记忆、关闭
该测试的记忆租约，下一次准备建立空租约并复用已加载权重。参数热更新内部的普通 cancel
保持原记忆。清空失败在业务状态中显示，下一次准备等待结束处理完成。
控制审计 DataEvent 的序号缺口计入 `control_event_gaps` 与 `last_control_gap`，并进入
现有流水线计时。统一采样与阶段判断使用配对 RobotState 的 `memory_sampling` 及 epoch。
原始观测来源变化、时间倒退、写入队列溢出仍使记忆不可用。
Owner 的状态订阅按 Runtime 实例常驻，推理绑定、测试结束和模型参数更新复用同一已就绪
输入。每轮模型绑定与请求仍独立。Runtime 身份变更时替换状态订阅，同一实例的描述变更
明确拒绝。首次就绪计时、订阅缓存与错误、发布端序列及丢弃数由现有诊断接口返回。

`GET /api/inference-replay-metadata?environment=...&run_id=...` 按所选封存 Run 返回
`workbench.inference-replay.v1`。录制绑定保存模型、后端、去噪步数及执行步数，已有录像
通过冻结的 ModelInput 和 ControlPolicy 还原。配置变化与 MODEL／TEACH 区间共同投影到
压缩回放时钟，页面在视频下显示当前配置并支持按片段跳转。TEACH 显示关联模型配置。
缺少历史字段时返回未知，缺少模式时间轴时明确提示。该接口按需读取，常规状态轮询不携带
完整配置时间轴。读取核对已登记回执和 manifest 身份，拒绝路径越界与读取期间的变更。

### 训练制品安装与采集期间回放编码

2026-11-12 用户批准统一模型安装入口、便携制品清单、采集期间浏览器编码及隔离验收和目标机发布验证。
新增 `model_install.py` 负责制品和安装事务，`scripts/install_model.py` 提供训练完成导出与目标安装入口。
模型目录、家族适配器、模型 HTTP 和 SDK Owner 继续通过现有接口消费已登记模型。

`workbench.model-release.v1` 包含完成标记、模型 ID、step、生产者身份、文件清单和后端声明。
每个文件声明相对目标路径、OSS 对象相对路径、大小及可用摘要。每个后端声明运行时 ID、
家族必需的制品角色和去噪能力。WAM-MEM 还声明源码相对目录、训练指令和文本缓存身份。
训练完成导出读取小文件并核对大权重大小，缺少大权重摘要时沿用生产者完成与传输 CRC 事实。

`workbench.model-install-target.v1` 由部署端提供资产根、目录根、主机/GPU 身份、OSS 内网配置
引用和已资格运行时。运行时 ID 解析为家族、后端及 Python 路径。机器路径与凭据引用保存在
目标配置中。Mac 入口只通过已配置 SSH 目标调用安装器，权重直接从 OSS 内网进入目标主机。

安装事务使用模型锁、分文件续传记录和共享资产缓存。完整文件与家族描述检查通过后一次性
发布全部后端目录，已安装的同一 release 重复调用返回复用回执，同模型 ID 的不同 release
明确报告冲突。目录发现完成与 GPU 加载完成分别记录，当前模型选择与控制权由原业务接口管理。
生产者入口从 WAM 的原生 COMPLETE/trainer_state/训练 contract 和冻结源码清单解析制品。
WAM-MEM 从 complete.json 或 weights-only export.json、config/run、依赖表及精确文本缓存
解析制品。checkpoint 提交与整作业完成分别记录。当前源码身份清单随制品保存，目标运行时
提供已资格的共享编码器资产。WAM 原始文本缓存由公共适配器按已声明长度补齐，保留有效
token 与原 attention mask。声明清单通过仅证明安装契约。

安装命令支持 `--source <OSS清单>` 或 `--producer <配置名> --family ... --experiment ...`
两种来源。后一入口依次调用原生完成记录解析、OSS 内网发布和目标安装。配置文件只保存
主机、运行时、路径与凭据位置引用。跨端复用既有 OSS 配置时通过进程输入与匿名内存文件
传递，发布清单在全部对象成功后提交。重复安装复用现有对象与本地资产。

ROS 录制器在每个原始分段提交时，通过其有界后台任务调用独立 CPU 编码进程，生成 H.264
`yuv420p` 浏览器分段。编码使用对应原始帧的活动区间，前置证据帧继续保存在原始分段中。
旁路 `workbench.browser-segment.v1` 保存原始 RGB/帧索引身份、入选帧号及浏览器文件身份。
结束时重新核对最终活动区间，复用匹配的分段，对缺失、内容变化或选帧变化的分段单独重建。

最终使用 FFmpeg packet `setts` 与 stream-copy 赋予微秒级 PTS/DTS 和末帧时长，随后封装 MP4。
该路径要求 FFmpeg 7 或更高版本的选项文件支持，录制进程在就绪前检查。已有未生成浏览器
旁路分段的录像继续支持原导出流程。三相机视频、MODEL/TEACH 配置区间沿用同一压缩回放时钟。
录制健康的各视频流 `browser_encoding` 提供已编码帧/段、待处理段、缓冲帧和失败信息，
计时接口记录 `recording.replay.encode`，封存清单记录复用与重建分段数。

用户进一步要求取消常规 Infer 保存时的全量原始记录校验。采集期间同步累计写入数量、字节
摘要和紧凑时间索引，关闭取流后生成 `workbench.written-capture-index.v1`，直接据此完成索引
和回放封装。最终 manifest 的 `raw_validation` 及模型输入事后审计字段明确记录
`not_requested`。采集进程报告的写入错误、队列溢出与终止水位结果仍进入最终录制结果。
带有写入索引的异常恢复复用同一封存入口，历史产物仍按其已有格式读取。
快速封存的浏览器产物通过封装包数量、PTS 和末帧时长核对，无 B 帧的输出无需再次逐帧解码。
后台编码启动前解析 FFmpeg 的绝对路径，使低优先级启动与子进程环境隔离兼容。

验收覆盖安装中断与并发重复、两种 Torch 家族目录/准备/参数更新、活动区间边界、源时间戳、
尾帧时长、旧录制恢复、浏览器分段损坏与重建，以及长录制期间控制和观测延迟。

### Infer 变更的非真机验收门

2026-11-11 用户要求：每次相关修改后，先完成以下验收再安排发布。应用、管理器、
模型 HTTP、ROS 与独立 Owner 使用实际组件，测试执行端必须是隔离的模拟后端。
独立命名空间、localhost ROS domain、真实 SDK 模块导入与 AF_CAN 拦截共同约束测试；
测试引入的真实控机调用必须为零。当前现场模型租约和活动控制会话不属于测试资源。

| 编号 | 必验内容 | 通过条件 |
| --- | --- | --- |
| A00 | 隔离 | 模拟执行器、独立数据与 IPC；无真实 SDK/CAN 调用 |
| A01 | 公共准备 | WAM Torch/SO、WAM-MEM Torch 的 1/8/20/32 步经 HTTP 与 Owner 接纳到 HOLD |
| A02 | Run/操作者 | MODEL/HOLD/TEACH 与回零入口准入一致；失败后无持续灰死或旧动作重放 |
| A03 | 去噪 | 两个 Torch 家族经管理器完成 2→4→10→2，worker 与加载次数不变；回执和页面反映实际复用 |
| A04 | 记忆 | 人工观测、参数变更后记忆连续；显式清空／fresh 才复位；未执行预测不写动作上下文 |
| A05 | 其他参数 | 执行长度、时长、录制选项更新后新 Run 生效，旧配置被冻结，SDK 不重建 |
| A06 | 竞争与失败 | 越界、短忙、重复请求、Stop 与迟到回包被正确处理；状态不永久锁定 |
| A07 | 规划 | 验证无倍数上限的完整重定时、2 rad/s策略、超过30秒的Owner传递与中途制动；显式有限诊断预算继续校验 |
| A08 | 连续切换 | 三次各 10 秒 TEACH 交替，再 60 秒 MODEL；检查断供、死锁与泄漏 |
| A09 | 录制 | 默认关闭无 Run 录像；开启三路完整；删除只影响选定的合成测试任务 |
| A10 | 发布 | 核对所有实际消费者与加载版本，尤其业务与 Owner；不打断现场活动会话 |

核心入口、参数与失败路径为必跑项；连续运行、录制和其他关联项按影响执行并说明不适用理由。
GPU 单模块与页面 DOM 通过不能代替上述链路通过。模型数值实现变化还须真实 GPU 对照；
复用未改变的既有数值资格时须标注来源和适用边界。SO 热更新与 WAM-MEM SO 暂缓，
不得把已有 SO 重载行为登记为热更新通过。

可重复入口为 `test_control_owner_process` 中两个 `prepare_sampling_and_short_infer_use_shared_owner`
用例（`WORKBENCH_NATIVE_ROS_TEST=1`、`ROS_LOCALHOST_ONLY=1`，执行环境须具有项目原生 ROS 依赖），
以及同文件 1/8/20/32 步录制用例；其余项目对应 `test_variable_inference_steps`、`test_execution_settings`、
`test_model_session`、`test_inference_transport`、`test_http_service`、`test_management`、`test_inference_history`
和 `test_web_ui.mjs`。候选、验收、发布各自保留回执，失败记录不覆盖。

用户已批准在`recording_process.py`中实现独立CPU记录进程，稳定身份绑定一次，事件只携带流、序列、源时间和有界数据/引用；沿用64 MiB事件队列与512 KiB消息上限。该增量是通用记录基础，不自动替换现有数采或infer录制，也不接入SDK或开启相机。

`CaptureSession.create(root, request_id=..., source_descriptors=..., purpose_profile=..., resource_budget=...)`创建候选，`prepare()`回读准备结果。`request_id`同时作为本候选的`capture_session_id`，不生成另一套随机身份；同一root和请求内容重复调用复用同一会话，内容不同则拒绝。记录目录由请求摘要定位，独占文件锁由实际writer继承；其他进程或重启后的调用只能读取既有候选，不能启动第二个writer或取得旧会话控制权。

每个来源描述固定`stream_id/kind/source_instance_id/source_clock_id/units/coordinate_frame/nominal_hz/required`。来源身份或时钟缺失明确表示不可用；`data_required`缺必需来源时不Ready，`infer_debug`返回degraded并留缺口。此处核验调用方提供的来源身份与本地存储条件，实际硬件健康和时钟对应关系仍由输入模块负责，不能凭描述中的非空字符串证明真机就绪。

`bind(run_id=..., business_kind=..., policy_hash=...)`在记录进程确认同一Run、业务和策略后返回；绑定后不接受身份改变。`start()`只开始记录。来源描述与上述稳定身份保存在会话文件和绑定文件中，不逐事件重传。

`try_publish(stream_id, sequence, source_time, payload)`只做有界JSON值快照和非阻塞入队，不在生产线程调用JSON编码、磁盘或编码器。默认队列64 MiB、单条消息512 KiB，可在准备时按既有上限收紧。计费保守覆盖快照对象与JSON编码上界。超限、队列满或来源不可用时返回`accepted:false`，记录本源拒收数；后续事件或`final_seq`使实际缺口显式可见。序列从1递增，源时间不得倒退。数值必须有限，图像字节和大张量走引用，不进入消息体。

`payload_ref`复用已有`path/size_bytes/sha256`制品描述。writer只读取普通文件、核对大小与摘要、流式复制到自身制品目录，然后写入记录引用；复制、散列与落盘均在子进程。记录不会依赖原始临时文件继续存活。源时间、worker接收时间、引用复制后的记录时间分开保留，不用接收时间代替采样时间。

`finish(final_seq_by_stream, control_terminal)`冻结后续发布并立即返回sealing；sender在已接纳队列排空后发送结束屏障。writer执行flush/fsync、逐流核验序列、制品摘要与末端边界，然后发布manifest。`recording_outcome`与控制终态分离：控制失败但记录齐全时，数据可为complete。缺流、缺序列、未绑定、缺结束屏障、损坏尾部或制品失效均不能报complete。

`snapshot()`返回实际worker状态、逐流received/committed、parent队列字节和拒收数。`recover()`只有writer文件锁已释放时才启动独立恢复进程，重新验证现有字节和序列；不补造控制终态、不重放动作、不截断原文件。每次封存保留独立`manifest-rNNNNNN.json`和对应缺口文件，`manifest.json`是最新回读入口。旧writer仍存在时只读，不抢占或停止它。记录失败时尽可能封存可读前缀；磁盘不可写时通过进程状态返回错误，后续再恢复。

当前CPU验证覆盖真实独立进程、重复请求/只读复查、准备与绑定、生产线程无JSON编码、可变输入的隔离快照、拒收和final_seq缺口、制品复制、异常退出与损坏尾部恢复。实际控制业务、I17输入描述、连续视频编码和I14模型流适配仍需后续接线；当前complete只证明声明的producer序列覆盖，不推断上游传感器本身无掉帧。

### I01 recording组与SC05装配

用户已批准将`recording`纳入同一份Owner策略。字段仅为`purpose_profile`和`resource_budget`；资源预算包含`event_queue_bytes/message_bytes`，默认及上限分别沿用64 MiB/512 KiB。数采、标定用途固定为`data_required`，infer固定为`infer_debug`；研究默认未选择，必须明确选择其中一种才能准备通用录制。固定业务的用途不出现在可编辑字段中，研究可通过原`control_policy`接口选择；资源值按`recording.resource_budget.<字段>`提交并整体校验。

所需流仍由本次绑定的`source_descriptors[].required`描述，失败动作按已确认用途推导，不另存一份列表或动作开关。预算验证与CaptureSession复用`execution_settings.py`中的同一实现和常量。旧持久化设置恢复时按业务补齐录制默认并记录来源；旧执行设置入口不会覆盖录制配置。新Run冻结的`policy_hash`包含录制策略，Guard和CaptureSession使用同一身份。摘要在预览准备时计算，实际接纳执行只消费已经冻结的值。

`operations.prepare_capture`实现SC05公共装配，输入为已有`request_id/run_id/business_kind/environment/policy/source_descriptors`与必需的`still_current`核验回调。先校验策略内容与摘要，再准备候选，按G35裁决并绑定Run。它返回`(session, receipt)`；receipt包含准备状态、是否Ready、实际绑定、同一策略摘要、裁决与缺失来源。它不启动控制、不启动来源，也不自动调用记录`start()`。

上下文在准备前、准备完成后及绑定后分别核验；失效的未运行候选异步结束留档，不能授予Ready。已有请求若绑定到其他Run，明确冲突并保留原录制；进程间只能只读访问的候选返回unknown而不接管。数据完整性用途缺流时拒绝绑定，infer调试返回degraded并允许绑定。测试覆盖实际Runtime冻结策略到Guard与CPU记录进程的相同hash，以及撤销、冲突、重复准备和研究用途未选择的拒绝路径。

该装配函数尚未替换现有业务的录制调用；下一步才接入真实来源描述、控制/模型事件与连续视频。CPU装配验证不代表这些生产适配已经完成。

### I13/I17已登记相机来源与结束边界

用户已批准相机来源引用、既有执行者身份和原始时间结束边界。`CommonCameraService.capture_sources()`只读取已运行服务，返回三路来源描述；不启动相机、重绑采集或取得机器人控制。视频描述增加可选`source_ref`，字段为`kind=common_camera/descriptor_path/camera/service_instance_id`。`source_instance_id`继续标识相机采集实例，`service_instance_id`标识相机服务进程实例，两者分别核验。凭据只从既有描述文件读取，不复制到逐帧事件或命令行。

存在自动相机来源时，`CaptureSession.bind(..., execution_owner=...)`要求既有`owner_pid/runtime_nonce/command_epoch`。该字段只参与原录制租约和帧关联核验，不授予SDK或机器人动作权。准备阶段只检查来源和占用；`start()`重新检查并申请本Run的相机录制租约。每路仍只有一个消费者，外部占用时拒绝申请；部分申请失败只清理已能证明属于本会话的租约。原相机服务不会随记录器退出而停止。

用户已批准独立记录租约作用域。通用记录器准备时通过既有服务identity回执核验`recording_run_lease_v1`能力，缺少能力的旧源按用途报告未就绪／缺口，不允许必需录制进入控制。服务描述同时声明该能力；不能仅凭客户端描述推定远端支持。绑定时租约仅增加可选`identity_scope=recording_run`，旧租约缺省仍为`execution_epoch`。fake帧仍须与绑定PID、runtime_nonce一致，采样epoch必须是原值或更新值；物理帧沿用原租约关联，不伪造同时刻Owner反馈。Run、token、源实例／时钟、单消费者、截止时刻和原结束边界继续校验。记录器照实保存Stop撤权后的新epoch，控制及模型输入的epoch校验保持严格；此作用域只使同一Run能够完成录像尾帧封存。未来真机发布需一并更新相机CPU服务与记录器，本轮未改动现有真机环境。

视频帧由记录子进程内的独立读取线程通过既有通道取得，复用`rgbd_segments.SegmentWriter`保存无损RGB/深度分段。控制事件和相机像素走独立入口，像素不穿过父进程的512 KiB事件消息。每帧保留来源时间、原始帧号及采集身份；归档`frame_ref`定位`camera/<stream>/<segment>.commit.json`和段内帧号。RGB与深度的像素完整性继续由原分段校验负责，元数据时间为权威时间，不用视频PTS替代源时间。

`finish(final_seq_by_stream, control_terminal, source_endpoints=...)`中，显式推送流仍提供最终序列；自动相机流改由记录器在排空尾帧后填写实际最终序列。`source_endpoints`为`stream_id -> {source_clock_id, source_time}`，时钟必须与绑定描述一致，单位沿流的源秒时间。请求仍立即返回sealing。记录器调用原`stop_after`关闭结束边界后的入流，保存覆盖该时刻的帧，再释放自己的租约；若边界未覆盖，沿用原5秒空尾等待预算并标为不完整，不放宽时间或伪造尾帧。

封存逐段核验commit文件、编码器输出、像素摘要和帧元数据关联；`segments_ref`指向本次版本的分段索引。相机线程的错误、末端覆盖及清理结果保存在`<stream>-source.json`。恢复时仅在原writer已退出后尝试释放可证明属于它的相机租约；其他租约保持不变。若中断发生在相机最终序列确认前，恢复结果保留该不确定性，不把已保存前缀标为完整过程。

该链路在Linux CPU假相机服务上的最终定向检查为7项相机契约和12项通用会话，19项全部通过，包括三路连续帧、RGB/深度解码逐字节一致、原始时间与尾帧、占用拒绝、缺尾、立即结束时排空已接纳帧及崩溃清理。证据为`.evidence/architecture-p1-20261105-v1/camera-tail-contract-receipt.json`。此前本机56项检查中53项通过、3项因无编解码器跳过。扩大Linux回归时，既有`segmented_playback.py`导出用例的fake/physical分支因系统FFmpeg4.4.2不支持`-fps_mode`而失败；该文件与此前基线一致，不能把这两项记为通过。未进行真机或生产业务切换验收。

### P1输入与轨迹规则归并（2026-11-05已批准）

G13/G15/G16使用I01现有`planning.velocity_limit_rad_s/acceleration_limit_rad_s2/jaw_velocity_limit_mm_s`，G21使用`execution.execute_steps`，G22核对单段时长为有限正数。Infer在接纳完整轨迹前统一调用I05，轨迹的峰值计算、重定时和SDK本地边界核验继续保留。`execution_settings.py`补入原`trajectory-check`文件映射，不新增策略身份或逐窗口完整配置。动态峰值沿用既有1e-10数值计算余量；步数必须为完整整数；普通在线Infer由G23核对整轮剩余时间，夹爪重定时读取已冻结的`planning.jaw_velocity_limit_mm_s`。

I01的`observation`组新增`state_max_age_s/max_camera_span_s`，默认均为0.1秒，仅可在既有已验证上限内收紧。前者由业务输入管理和Owner核验配对反馈年龄，后者由输入管理核验三相机时间跨度；`maximum_age_s`继续管理图像及整体观测年龄。G02/G03/G04使用现有I05字段，采样时刻与检查时刻分别保留。训练配对25/12ms仍仅诊断，本轮没有将它们变为拦截条件。

旧持久化策略恢复时补齐原默认值并记录`migrated_input_defaults`来源，下一Run冻结同一策略。配置中两项参数通过既有`protocol_features`协商`observation_policy_v1`；缺少能力的Owner在配置和控机前拒绝，不把回显未知字段当作实现证明。Owner的实时缓存读取只执行内存裁决，不发起策略RPC、散列或磁盘写入。

### I05绑定与结构化观测（2026-11-05已批准）

I05的下一增量已获用户批准：`Guard.bind(policy, context)`在已有Run/能力准备阶段保留冻结策略及实际消费的只读上下文；绑定后的`evaluate(signal, phase)`继续使用同一组信号键，允许声明的有限向量/结构化观测。旧数值入口委托同一实现，不生成另一套规则或身份。上下文不向SDK窗口传播。

当前绑定实现覆盖旋转关节限位向量、跟踪/到位误差、反馈与窗口饥饿的计算。真正的共用执行接线已完成G25/G26/G28/G29：MotionRunner复用Owner预先绑定的策略，保持主动臂、禁用夹爪、持续跟踪时段与到位资格的原语义；TargetInterpreter保留普通反馈0.1秒、积分路径0.05秒及原15ms饥饿余量，后者保持原减法顺序以免改变浮点边界。Owner接纳阶段不散列或验证策略对象；准备阶段完成绑定。已有控制诊断及参考时序回执携带对应裁决。G18/G19/G20仅引擎测试通过，位置调用链尚未全部迁移，不能据此宣称P1完成。

本增量的261项匹配源码LinuxCPU检查全部通过，见`.evidence/architecture-p1-20261105-v1/goal-bound-final-receipt.json`。此前输入/运维/UI增量的312项回执单独保留，不将重复测试相加为本版本的覆盖数量。两者均未部署ARX6或进行实机验收。

### P2异常事实与P4参数入口（既有接口内实现）

公共运维执行器在动作开始后遇到未分类异常、回包丢失或核验异常时保留`unknown`及已有result/阶段证据，不能据此自动重放。执行器使用显式`OperationError`区分已确认失败与未知结果；相机复位前的设备/配置不符、录制租约占用和进程资格不满足均是已知前置失败，未发起复位。真正复位后的异常继续使用原unknown语义。

工作台统一参数面板复用`GET /api/control-policy`和原`control_policy`动作。只按Owner返回的`editable_paths`构造原生输入，并提交`expected_revision/changes`；没有第二份参数存储或新的控制入口。读取不启动Run，保存后下一Run生效；版本冲突保留草稿，切换业务/设备后拒收旧读取回包。旧快捷设置继续投影到同一策略。标定、规则数采、研究任务的统一记录适配及其完整用例验收仍未完成。

### 实际Infer Run迁移与回放（2026-11-05已批准）

部署配置显式协商`recording_backend=capture_session_v1`，装配新工作台时默认启用该后端；旧Owner未声明同名能力时，在配置/控机前拒绝。保留显式旧后端兼容入口，`recording_enabled`不再决定新后端是否留存。新Run的`created`事件仅传一次已冻结`control_policy`（revision、policy_hash、parameters）；业务侧直接使用这一快照执行SC05，不重新读取或生成第二份策略。

准备、健康与结束的`inference_recording`回执复用`execution_instance_id`。Owner核验当前Run与桥接会话，上一Run迟到的就绪或结束信息不能改变新Run。控制事件在现有Owner事件通道排队后，经`inference_trace.CaptureRun`写入通用会话；最终序列屏障与控制终态分别保留。录制故障按同一冻结策略交给G38裁决，infer_debug可继续并标记缺口，原观测新鲜度与运动保护不变。

模型输入/输出沿用`InferenceEvidence`异步写入，CaptureRun按已有request_id关联并将数组、请求元数据、耗时和失败信息作为不可变引用归档。控制正常完成、失败、Stop均生成`debug-receipt.json`并进入现有结果索引；控制成功不意味着记录完整，记录完整也不意味着控制成功。视频派生失败单独呈现在已有错误字段中，不把完整的原始数据改判为丢失。

回放复用`rule_based_workbench/segmented_playback.py`。逐帧核验原分段元数据、采集身份、源时间及序号；仅导出已验证前缀。VFR编码使用兼容旧FFmpeg的`-vsync vfr`，微秒时间基与末帧时长继续由ffprobe逐帧核验；任何误差超出既有1微秒门槛均拒绝发布回放。此处只迁移Infer Run；标定/规则数采的生产入口仍待迁移，不代表ARX6已经部署或通过实机验收。

## UC步骤、共享子流程与实现追溯（2026-11-05）

追溯入口为 <http://10.17.0.27:18930/#trace?uc=UC05&step=UC05-S02>。UC描述用户目标，主流程和异常分支均使用稳定步骤ID。SC定义可复用行为，包含输入、前置条件、子步骤、完成条件、失败分支、取消/重试语义、编排负责组件及参与职责。接口描述组件间的调用和数据关系，角色绑定将具体行为定位到对应实现文件。

唯一维护源仍为 `web/architecture/catalog.json`。`use_cases[].main_flow` 从文字列表迁移为步骤对象，含 `id/title/inputs/completion/sc_refs/bindings/next`。异常分支保留原有触发和后续处理字段，并增加稳定ID、触发步骤 `from_steps` 及实现引用。重复执行显式记录 `loop_to/loop_condition`，条件调用记录 `condition`。`sc_steps` 可把某次引用限定到SC的具体子步骤，例如UC17分别引用准备与绑定阶段。

`shared_steps` 保存SC行为契约及其自身验收条件。原SC02细化为撤销输出并核验停止，新增SC07单独描述正常释放SDK与退出证明。SC08–SC11分别描述输入配对、模型调用、轨迹准备和执行反馈。现有6类共享行为得到扩展，共11个SC。UC05每个action chunk复用模型和输入行为，人工目标与研究轨迹复用公共准备/执行行为，模型坐标适配在UC05对应直接步骤单独映射。

`traceability.bindings` 记录接口ID、参与组件、具体实现角色、文件路径、状态与缺口。已有复用源码标记 `partial` 并说明待归并内容，目标入口尚未实现时标记 `planned` 并写清缺口。实际实现完成后更新为 `implemented`。文件采用开发Map中的同一路径身份。图中的每一行保留行为步骤和接口角色，避免通过组件级文件并集扩大依赖关系。反向查询从文件返回实际引用它的UC步骤及SC。

SC05样例在创建Run身份之后准备录制，编排、输入、记录和Guard分别承担请求、流身份、资源/会话和用途裁决。data_required所需条件不足时拒绝准备，infer_debug按已冻结用途策略返回可用、降级或拒绝结果。准备响应丢失时按原请求核验会话。SC06分别记录finish受理、异步封存及complete/incomplete结果。上述行为沿用此前已确认的用途策略。

新增目标契约I16定义模型选择、准备、状态、绑定、取消与释放，I17定义输入源描述和绑定。I05增加运维/业务对准备裁决的消费关系。接口仍按现有能力迁移和版本协商推进，部署资格以相应验证回执为准。

SC进度由 `python3 scripts/development_map.py set --kind sc --id SC05 --state active --note '<本次工作>' --next '<下一步>' --publish` 维护。标为 `verified` 时，所有角色绑定须已实现，回执必须包含 `scope`、当前 `source_fingerprint` 和覆盖该SC全部验收ID的 `cases` 对象，各值为 `passed`。源指纹覆盖SC行为、验收条件、接口、角色绑定和已映射传递依赖。实现或契约改变、回执丢失或摘要变化后，页面显示待重验。UI目录检查和图形检查单独作为追溯工具的证据，SC运行行为按自己的集成验收维护。

本版覆盖21个UC的102个主流程步骤、45个异常步骤，以及11个SC的47个子步骤/异常分支，共60项接口实现角色映射。检查入口为 `node scripts/check_traceability.mjs`，覆盖稳定ID、前后继、引用范围、组件接口角色、文件归属、明确缺口、双向导航及验收证据失效。原架构和Map检查同时保留。样例、校验、离线图形及发布证据位于 `.evidence/architecture-traceability-20261105-v1/`。实际浏览器布局验收受Work自动化设置限制。

## 开发 Map 与日常维护（2026-11-05）

开发导航集成在 <http://10.17.0.27:18930/#progress> 顶部，侧边栏入口为“开发 Map · 实施进度”。同页嵌入复用独立Map模块，三级导航和自动刷新均在该区域内使用，另外提供全屏入口 <http://10.17.0.27:18930/development-map.html>。功能模块、职责子域、源码文件三层均可切换，Python粒度止于文件。功能模块身份和接口关系引用 `web/architecture/catalog.json`。`web/architecture/development-map.json` 唯一维护本轮职责域、文件归属、开发状态、负责人、下一步及证据，文件以仓库相对路径去重。当前覆盖8个功能/支撑模块、22个职责域、105个独立文件，范围包括工作台运行时全部顶层Python模块和已核对的SDK/标定/数采接入边界、模型/部署入口及Map自身源码。

按用户确认，既有`rule_based_workbench/rgbd_segments.py`纳入“独立录制、编码与封存”职责域，并关联视频录制和封存步骤。导入图同时识别记录进程既有`PYTHONPATH`中的`rule_based_workbench`来源，因此`from rgbd_segments import ...`可定位到该文件；其变化会进入下游源码指纹。此项只补导航与映射，不修改编码器源码。前后摘要、CPU引用与图检查见`.evidence/architecture-p1-20261105-v1/encoder-map-validation.json`。

本轮优化状态与既有能力分别记录。灰色表示已有基线，紫色待开发，蓝色开发中，红色阻塞，黄色已实现待验收，绿色通过所列验证，橙色表示源码变化待重验。职责域的目标完成状态由验收证据维护，功能模块按职责域汇总并显示混合状态的数量。共享文件保留一份状态，父节点统计去重。模块图连线来源为设计接口，文件/职责图连线来源为静态Python导入，动态装配的调用关系继续依据接口规范。

每次开始、阻塞、实现完成或验证完成时执行 `python3 scripts/development_map.py set` 更新相应节点。每个逻辑变更结束后执行 `python3 scripts/development_map.py refresh --publish`。工具刷新文件摘要、已映射依赖及历史部署比对，发现顶层运行时新文件时列为待归属。新增职责或文件应先修改同一JSON映射，复用已有模块ID，避免另建进度事实源。历史部署快照保留核验时间，实时设备归属仍走原核验入口。

`set --kind domain --id <职责域>` 更新职责域，默认 `--kind file --id <相对路径>` 更新文件。必需字段为 `--state` 和 `--note`，建议同步 `--next`，负责变化时指定 `--owner`。`verified` 必须提供 `--evidence <仓库内回执路径>`，工具冻结证据摘要及源码/已映射传递依赖指纹。源码或证据变化后，页面转为待重验。该验证范围以回执为准，真机资格另外记录。

发布复用18930静态站点，以新release目录和原子symlink切换维护。发布前检查架构目录和Map，发布后逐文件比对HTTP字节摘要。页面每15秒读取已发布快照，保留当前图层、选中节点和筛选。网络失败显示上次快照与更新失败提示。刷新命令只更新本地，`--publish`或`publish`显式同步内网站点。

本次检查入口为 `node scripts/check_development_map.mjs` 和 `node scripts/check_architecture.mjs`，证据在 `.evidence/development-map-20261105-v1/`。检查覆盖层级、共享文件去重、筛选、键盘、路由、刷新、失效与恢复、证据绑定及原子写入。三层SVG已离线渲染核验，实际浏览器交互因Work设置禁用自动化而待用户确认。Map自身显示已实现待验收，控制链公共模块优化保留待开发状态。

## 第五模块与统一记录设计（2026-11-05）

用户明确要求将中间产物采集与保存列为独立公共模块，覆盖标定、规则数采、infer、控制研究及其他按统一协议接入的控制过程。当前设计版本为 `architecture-recorder-20261105-v3`，详细入口为 <http://10.17.0.27:18930/#recording>。

五个公共模块分别负责控制门槛、相机/state输入、模型推理、轨迹翻译执行，以及中间产物采集保存。记录服务采用独立进程，与控制链并行订阅数据。现有 `ProcessRecordingSession`、`InferenceTrace`、`InferenceEvidence` 和导出/质量核验能力作为迁移基础，通用会话与跨业务接口需要后续实现。

统一采集范围包括全部声明关节状态、控制点与原生窗口、可用实际输出与SDK回执、连续视频流、模型实际输入/输出、业务阶段、策略裁决及终态。每条流明确源能力、采样层级、频率、单位、坐标、sequence和source timestamp。相机采集实例唯一，完整视频与模型选帧通过帧ID关联。

新增I11–I15分别定义CaptureSession生命周期、ControlEvent非阻塞发布、连续视频FrameRef、模型产物引用，以及RecordingHealth/SealReceipt。控制面由应用层管理prepare/bind/start/finish/status，实时数据面使用固定上限事件与有界共享引用。编码、序列化、写盘、索引和恢复在记录进程及其受预算约束的worker内完成。运维事务负责请求幂等账本，记录服务接收其事件并统一归档。

每个CaptureSession冻结用途策略和资源预算。用户已确认D09，标定和规则数采要求数据完整，记录异常由公共Guard请求控制执行器受控结束。infer调试允许控制继续，并明确保存缺口和incomplete结果。该策略针对记录链本身，源相机和SDK异常由对应输入与控制规则裁决。控制研究按明确的测量验收或调试用途选择策略。

finish携带各源final_seq及控制终态，立即返回sealing状态。记录服务异步核对已提交分片与序列覆盖后发布最终manifest。停止运动、结束采集、完成封存分别有回执。进程退出、重复finish和迟到事件使用会话身份、序列与seal_revision处理，恢复时保留可用数据并明确缺口。

性能验收需比较录制开关前后的publish耗时、控制周期和Stop响应，同时覆盖慢盘、满盘、进程崩溃、编码拥塞、长时内存有界与全流吞吐。独立进程的CPU/内存/IO/编码资源与共享缓存生命周期都要纳入预算。三路640×480 RGB各30fps的原始吞吐约82.94 MB/s，两秒源帧驻留约158.2 MiB，仅作为容量演算，实际参数按设备profile测量后冻结。

新增UC17–UC19覆盖通用录制、故障处理和恢复封存，UC20–UC21覆盖标定及规则数采的全过程留存。已有Run、调姿、研究与交接用例接入相同准备/结束录制步骤。目录当前共有5个公共模块、21个用户用例、15份接口契约和38条现状/目标规则，目标规则明确标注设计阶段。

本次完成的是设计目录、图形、故障用途策略和接口约定。录制服务实际实现、设备上的性能预算及跨业务接入资格按P1–P5推进。本次证据位于 `.evidence/architecture-recorder-20261105-v3/`。后续章节保留此前版本的设计范围与历史核验时间。

## 用例驱动的架构与实施方案（2026-11-05）

用户要求先明确目标、典型交互与实施路径，再开始控制链路改造。当前设计入口为 <http://10.17.0.27:18930/#usecases>，目录版本为 `architecture-usecases-20261105-v2`。

目录建立了4个用户目标、16个用例、5条典型旅程和32项目标验收case。每个用例记录参与者、前置条件、主流程、成功后的状态、异常分支、实际影响、模块、契约、规则及源码依据。这些验收case描述目标行为，其执行证据按后续阶段补齐。

层级从用户用例进入应用层编排，再进入4个领域模块和基础设施。运维应用层复用现有 `OperationManager`、`Management` 和 `Runtime/Owner`，统一请求幂等、任务互斥、步骤回执、取消、后置核验与恢复点。它服务启动、接入、Stop、恢复、平台交接及发布等用例。四个领域模块继续负责共同门槛、输入管理、模型推理和翻译执行。

新增目标契约I10描述 `OperationIntent / OperationReceipt`，关联use_case_id、operation_id、目标身份、策略版本、步骤结果、当前实际状态和允许的下一步。unknown结果先进入核验，确认外部状态后再选择继续或可重复步骤。Stop具有独立优先级，既有授权按目标和动作范围复用，动作时点的事实按最新快照核验。

工作台退出用例UC09分别核验停止、必要维护、SDK释放、录制封存、自有资源退出和可选的正式网页恢复。若SDK已释放而数采网页启动失败，结果保留“设备已释放”这一事实，后续只处理已确认需要恢复的网页步骤。正式网页可访问、数采硬件manager就绪和录制开始分别记录。

后续变更以用例和接口为依据选择验证范围。目录提供V0目录/界面、V1局部逻辑、V2接口/用例集成、V3部署只读核验、V4有界真机验收，以及7种常见变更的影响示例。源码、配置和语义身份匹配的已有证据可以复用。控制语义或物理约束变化需完成受影响的实际硬件验收。

实施顺序按用户流程形成闭环，P0确认目标与用例，P1归并源码和共同契约，P2完成运维交互，P3贯通可解释infer与产物，P4复用参数和维护能力，P5汇总匹配证据并补齐端到端缺口。D06–D08补充用例先行、运维状态核验和影响驱动验证的设计决策，与原D01–D05一起审阅。

开发维护要求是在同次变更中更新受影响用例、接口、规则与状态，并运行 `node scripts/check_architecture.mjs`。检查覆盖所有模块、10份契约和33条规则的用例引用闭包，状态转移及参数关系。每个新增抽象需要说明服务哪个用户用例。

本次页面与目录证据位于 `.evidence/architecture-usecases-20261105-v2/`。硬件运行快照沿用带时间的历史核验，设计更新时间独立记录。

## 架构与统一策略审阅（2026-11-04）

用户要求先梳理架构和关键设计，再进行控制链路改造。当前工作台、共享模型与控制执行由本聊天统一负责。下文 A/B/C/D 分工及各日期的运行状态按历史范围使用。

架构页面位于 <http://10.17.0.27:18930/>。它从 `web/architecture/catalog.json` 读取四模块职责、33条规则、9份关键接口映射、开发状态、源码定位与目标设计。该目录是架构图和门槛盘点的维护源。当前 wire 契约以本规范及对应部署源码为依据，目标字段显式标记为待确认设计，随后通过 schema／feature 协商推进生产者与消费者。

2026-11-04 22:48 CST 只读核验显示 ARX6 工作台业务、Owner、模型已退出，正式数采8080运行，工作台运维网关仍在。架构页是独立静态文档服务。参数区支持草案演算与导出，实际策略提交、设备能力校验与生效回读列入P4阶段。

本次逐条核验的7个封存Run均使用执行设置 revision 6，实测速度保护为 **1.5 rad/s**，规划速度为 **1 rad/s**，代码默认实测保护仍为 **0.8 rad/s**。infer 的实际跟踪与到位判断均为关闭。此前概括为“实测保护1rad/s”的状态描述需按这些真实Run回执修正。完整性、时间配对、漂移、规划可行性、实际反馈与传输超时在目录中分别列出。

建议设计采用一个不可变 `ControlPolicy` 快照、统一 `GuardDecision` 和分阶段裁决。Owner实时检查及原生watchdog就地执行并绑定相同策略身份。关键待确认事项包括部署形态、下一Run生效语义、规则的observe／warn／enforce等级、失败动作分类，以及唯一SDK写入与兼容策略。实施顺序和验收条件在页面P0–P5列出。

后续开发应在同次变更中同步维护以下内容。

1. 组件职责、接口字段、门槛含义或失败动作变化时，更新本规范与 `web/architecture/catalog.json` 对应条目，保留明确生产者、消费者、单位、坐标、时钟和取消语义。
2. 每条规则使用唯一rule_id，标明当前来源、适用路径、可编辑范围、生效时机与实际检查点。当前事实附源码摘要或运行回执，设计与已发布状态分别填写。
3. 参数提交前检查设备能力、参数关系和revision，Run创建后固定policy_hash。所有下游结果回传同一策略身份。
4. 运行 `node scripts/check_architecture.mjs`，核验目录引用、参数派生关系、DOM入口与隔离的数据读取面。控制契约变更还需对应模块的回归及约定验收。
5. 发布后更新实际版本和带时间的核验快照，记录源码尚未归并、待验证及历史资格的适用范围。

本次源码核验与真实Run策略证据位于 `.evidence/architecture-review-20261104-v1/`。后续阶段接续时先读取该目录的交付索引与架构目录。

A 模型协议和 B 执行语义分别由其 owner 维护，C 不重定义：

- [A 模型服务及 ActionChunk](../../robot-workbench-model-session-pin/docs/model-session-shared-service-20261103.md)
- [B 控制执行契约](../../robot-workbench-maintenance-home/docs/control-execution-contract-20261103.md)

## 正常刹车余量的相关性修正（2026-11-04 14:15）

用户 2 秒 Run 的第一份预测在准备阶段被旧余量估算拒绝，设备保持健康静止。原轨迹按速度、加速度与夹爪限速可在 0.765 秒执行，旧算法将整条轨迹的最小限位距离与不同时刻的最大速度、加速度相加，估算需延长至 2.490 秒。限制轴为右 J2。

修正保留原快速充分条件；该条件无法在预算内通过时，使用同一时刻的 q(t)、v(t)、a(t) 构造刹车包络。刹车位置两个基函数分别落在 [0,T/2] 和 [0,T²/12]，速度基函数落在 [0,1] 和 [0,4T/27]，加速度基函数落在 [-1.5/T,0] 和 [-1/3,1]。对矩形顶点形成的各条多项式求全部极值，包住任意参考时刻和任意刹车时刻。必要时在原最大时长内做有界时间缩放搜索，最后重新核验包络。

同一原始预测重放后 1.5086 秒可满足证明。所有 15 个目标点、1 rad/s、40 rad/s²、40 mm/s、0.2 秒正常刹车、单片段 2 秒上限与关节限位保留。真正不可行的限位／时长输入仍拒绝。该改动只发布 `motion_reference.py`，本地和 Linux 各 41 项检查通过。

重新加载后使用新观测／新预测完成真实 2 秒预算 Run，完整 15 点实际执行 1.3986 秒，正常 END、反馈健康、无控制故障。旧预测只用于离线重放。原始证据、解析证明与发布身份见 [brake reserve delivery](../.evidence/brake-reserve-20261104-v1/delivery.json)。

## 当前持续推理部署（2026-11-04 13:51）

持续推理已在 ARX6 上发布并完成短时实机验收。当前默认 120 秒，支持 0.5、1、2、40、120 秒。工作台为 managed 生命周期，沿用正式 8080 数采交接、人工规划及启动配置。模型、SDK 与输入绑定已恢复，用户可沿 Run 入口测试长时任务。

本轮实机覆盖四次完整单片段、选择 40 秒预算后连续两段并手动 Stop、Stop 后新 Run、正式数采退出与工作台恢复。65 项组合检查、6 项接纳补验、18 项模型退出回归和前端检查通过，存在重叠，分别列证。原始回执见 [continuous v2 delivery](../.evidence/continuous-infer-20261104-v2/delivery.json)。本轮无录像，40/120 秒全时长、任务效果和 Triton 链路仍待验收。

Run 的后台作业等待本次 initial HOLD 完成后提交 MODEL，正常预算结束进入 HOLD，再提交 END。模型第一段未能被接纳时，即使错误原因是预算不足，也返回失败。模型运行预算从 MODEL 开始，包含观测、推理、CPU 准备、段间等待和运动；初始化保持与 END 收尾另计。0.5 秒选项沿用完整 15 点诊断，允许重定时至最多 2 秒。

managed 模型退出已改为先撤销本工作台绑定并关闭 pin，再停止模型服务。pin 关闭未知时保留服务供核验。实机确认 pin_release=released 后服务退出，重新加载成功，SDK 全程健康。

## 正式数采入口（2026-11-04 13:13）

用户指定正式数采为跳转对象，当前为 managed 工作台生命周期。`collection_handoff` 固定正式 unit、文件摘要和 loopback status API，`collection_handoff_url` 指定浏览器目标 `http://10.17.32.5:8080/`。完整退出先完成两臂回零和 SDK 释放，再退出工作台资源并恢复正式网页。

本机显式配置 `exit_home_keep_jaws=true`，回零采用原 calibration 人工单臂目标入口，顺序执行左、右臂 0.1 rad/s 回零；每个目标不含夹爪指令，原 MotionRunner 从 native begin 命令保留夹爪。两臂之间等待相同 Stop barrier 的在途调用清空和保持确认，再创建第二臂审阅。失败时沿既有保护链中止。

正式退出核验返回 `environment_exited`、`sdk_release_verified`、`collection_web_ready`。运维轮询仅公开这些布尔证明和 `next_url`，浏览器只在当前用户请求成功且三项证明齐全时跳转。共享模式的旧证明组合仍可兼容，但本机默认已不指向 18080。

本轮实际回零、释放、进程退出、正式网页启动已通过。原任务 1275、正式数据目录与 release342ac40 保留。回零参考冲突及临时 unit 消失导致的失败/未知回执、修复和后续成功均见 [正式交接回执](../.evidence/formal-collection-handoff-20261104-v1/delivery.json)。浏览器真实点击受当前工具设置限制。

## 人工规划启动与独立平台入口历史（2026-11-04 12:49）

用户指定后，工作台、共享执行与共享模型开发由本会话统一负责。既有模块协议继续复用，下方原 A/B/D 写权与部署责任描述按历史阶段理解。

人工规划随工作台业务启动，可信 business profile 必须提供当前 `calibration_config`。注册启动脚本在页面与模型服务可读后，读取 calibration 工作区并要求 `manual_queue.available=true`，成功回执新增 `manual_planning_ready=true`。模型预热仍异步，网页可以先访问。

external 模式可配置 `consumer_handoff_web={unit,files,status_url}`，其中 files 为已有 user unit/drop-in 的精确摘要。status URL 与 `consumer_handoff_url` 使用同主机和端口、路径 `/api/status`。用户点击“退出工作台并启动数采平台”后，管理器先核本消费者 detach 与共享 SDK 保持，再复用既有 `restore_collection_web` 启动或复用该独立网页，核实际 robot_vin、orphan 状态和就绪结果后才返回 `next_url`。平台录制由用户另行启动。

本轮已验业务停止后启动命令返回人工规划就绪、模型准备完成、真实人工姿态预览与粗球自碰撞检查，以及数采网页从 inactive 经按钮对应 API 启动到 ready 后恢复工作台。默认启动命令与证据见 [ARX6 运行说明](ARX6_RUNTIME_HANDOFF_20261103.md)及[交付回执](../.evidence/manual-startup-20261104-v1/delivery.json)。浏览器控制禁用，真实点击/导航体验待用户确认。整机重启后的全链冷启动另计。

## 可配置持续推理候选（2026-11-04 12:04，尚未部署）

用户现场完成旧四段 QA Run 后明确要求恢复完整可配置 infer。限制来自本轮配置生成器强制 `finite_chunk_count=4`，工作流据此锁定 0.5 秒并禁用时长选择。新候选移除该 QA 选择，复用已存在的共享 normal 同步执行路径，默认 120 秒，可选 0.5、1、2、40、120 秒。

1、2、40、120 秒表示从本次 HOLD→MODEL 开始的最长设备源时钟预算，包含观测、推理、准备、段间等待和运动。每段完整执行源 0.5 秒的 15 点，可减速至最多 2 秒。下一整段放不进剩余预算时保持，随后正常 END。0.5 秒选项沿既有单完整片段诊断语义。规划 1 rad/s、40 rad/s²、夹爪 40 mm/s、反馈/观测/供给门沿用现有契约。

`inference_run` 使用原后台作业等待本轮 `normal_initialize` 完成 HOLD，再提交一次带当前 session/nonce/epoch 的 MODEL。观察到本轮 MODEL 后完成 HOLD，再提交一次 END 并等待 READY。预算结束属于正常结果，过期、供给失败或轨迹不可行引起的提前 HOLD 在 END 后仍报告原失败原因。Stop 撤销后续 MODEL/END，未知请求只核对原回执。

可信 C profile 使用 `normal_initial_mode=HOLD`、`recording_required=false`、空 recorder_ids，当前关闭录制，无 normal_finish_policy。同消费者 Stop 后通过新 Run 重新审阅。用户明确退出或切到数采时，若自身 runner 已 STOPPED，manager 在消费者本地 `stop_consumer` 维护范围内先释放自己的模型 pin，再调用 `ops_detach_stopped_consumer`。业务侧调用 B 的 `detach(abort_stopped=True)`，要求 `consumer_abort_stopped_v1` 及 B 原有停止、静止、健康反馈、录制结束和自身持权核验。成功回执包含 `episode_aborted=true`、`safe_finish=false`、`ownership_retained=false`。默认 close 行为继续使用原接口。

组合 CPU/HTTP/Unix 检查 107 项通过，最终微调 6 项通过，web 和 operations 两组页面检查通过。B 执行器配合设备替身验证了正常预算至 END。D 是唯一远端部署与操作方，新 B 服务端能力需要正常加载新 Owner 并更新实际 consumer 绑定。冻结入口为 [持续推理组合回执](../.evidence/user-duration-infer-20261104-v1/receipt.json)，此时尚未取得该组合的实机部署回执。

## 已部署用户切换接口与验收进度（11:40 四段版本）

2026-11-04 11:40，D 完成 C 接入 → C 退出 → D 准备至 READY → D 返回 → C 再接入的五步实机 API 验收，每一步均 passed。全程共享 runtime nonce `5a6a5a82…`、Owner 实例 `4ac06c98…` 保持，最终反馈健康、C 持有控制、模型 ready、Run allowed，自动验收脚本已退出并交用户亲测。C 已读取本地实际回执并校验来源，D 负责现场操作。

首次接入失败原因是 Owner 相对脚本路径与 manager 权限隔离冲突。身份解析修复在 Linux 执行 23 项检查，22 通过、1 跳过；实际部署经正常维护将 Owner 改为绝对脚本入口并建立新绑定。模型 service/worker `2285559/2285682` 持续保留。完整往返使用上述新实例。

中途一次 C 返回请求收到 HTTP 409，原请求没有 retained job，后续对照发现目标 epoch 从 5 变 6。D 核对 fresh 状态及准入后发起新的明确请求成功，旧失败保留。初版验收脚本未保存该 409 的响应正文，后续已补保存。身份变化符合前置拒绝路径，具体业务错误码无法由现有证据确认。C 管理操作完成时间为首次接入 0.595 秒、退出 0.516 秒、再接入 0.577 秒，均取作业时间戳，页面加载和人工操作耗时另计。

交付时模型 `active_sessions=0`，C 在 Run 时获取执行 pin。模型 ready 与 Run allowed 表示当前准入状态，实际执行结果需由本轮 Run 回执确认。

本轮部署根为 `/home/wujie/work/feiyu.zhou/workbench-collection-switch-20261104-v1`，`provider/` 由 D 最终执行源叠加 C 21 文件组成。C 管理页与业务端口分别为 18868、18878，D 隔离数采入口为 18080。工作台使用 `workbench-C`，数采使用 `collector-D`，共享 Owner 与模型服务由 D 统一管理。原正式数采任务及数据由 D 保留。

1. C → D 在 C 空闲后点击运维页“切换到数采工作台”，沿 `exit_environment` 释放自己的 pin、关闭自己的业务并 detach。完成检查包含本消费者已释放、共享进程身份保持、同一非空 `runtime_nonce`、SDK 已连接且反馈健康。
2. 可信 manager 配置的可选 `consumer_handoff_url` 提供目的页面。只有当前用户请求的成功回执满足上述检查，页面才自动跳转。历史回执、未知结果或管理器身份变化均需明确核验。运维轮询返回 `next_url` 及 `consumer_detached`、`independent_services_preserved`、`shared_runtime_preserved`。
3. D 页点击“启动硬件并准备采集”接入该消费者。返回前沿正常结束与保存流程，D 的 `POST /api/collection-mode/handoff` 成功 detach 后导航至 C 运维页。
4. C 返回链接携带 `view=operations`，用户点击“接入本工作台控制”，附着当前共享 Owner 并恢复当前热模型绑定，然后进入推理页。业务后端缺失时从模型服务的实际 backend/bundle 取得绑定依据。external 模式的恢复使用本地模型准备维护范围。

共享服务身份核验支持 Python 相对脚本路径，通过 `/proc/PID/cwd` 严格解析实际文件，检查工作目录稳定性，并保留 PID/start_ticks/pgid/sid/uid/boot 核验。解析失败保持未知。该修复只需重新加载管理器代码。

C 本地组合 48 项 CPU 检查与两组 Node 页面检查通过。身份修复本机执行 23 项检查，21 通过、2 项 Linux 专属跳过，目标 Linux 结果见上文。浏览器控制当前在 Work 设置中禁用，现有证据覆盖 CPU 页面行为和现场 API，实际点击与现场踏板体验仍需单列验收。本轮切换准备未新增 infer 动作或录制，当前组合的用户 Run/DAgger 操作仍需现场验收。

冻结入口为 [C 组合回执](../.evidence/user-bidirectional-switch-20261104-v1/receipt.json)、[身份修复回执](../.evidence/user-bidirectional-switch-20261104-v1/relative-script-identity/receipt.json)与[当前实机证据索引](../.evidence/user-bidirectional-switch-20261104-v1/actual-delivery-index.json)。D 现场最终入口为 `/Users/wujie/Documents/ChatGPT/first_work/workbench-collection-switch-20261104/remote-evidence/user-ready-receipt.json`。以下旧阶段标题与数值用于历史追溯，本节优先说明当前部署。

## 历史资格与组件状态（2026-11-03）

本会话最后实机发布为 `source-control-fourvideo-20261103-v1`。旧 5 rad/s² 接纳门已修复；三段、随后用户要求的四段真实 infer 均完成。四段各完整15点、新观测/预测，时长0.549815、0.627849、0.645992、0.629804秒，独立Stop和8次异步写入确认，三视角视频完成。

23:39 C通过external manager正常关闭自己的业务消费者和模型pin，保留独立Owner/SDK、热模型和相机；23:40同一GPU记录已转给D，C旧GPU监控删除。SDK控制交接单独等待D/B候选。本轮新增的**共享执行Owner消费者生命周期接线仅离线通过，尚未部署**，不能把四段单消费者实机通过外推为C/D共享隔离实机通过。当前资源责任和交接证据见[运行环境交接](ARX6_RUNTIME_HANDOFF_20261103.md)。

实际来源与状态以[四段实际回执](../.evidence/arx6-four-segment-video-20261103-v1/actual-acceptance.json)、交接文档及canonical continuation指针为准。旧sync15三段失败、426ms及首包35.78ms均为历史测量，不代表当前交付仍失败，也不代表后续冷启动延迟已优化完毕。

| 组件 | 当前状态 | 已有证据 / 下一步 |
| --- | --- | --- |
| 模型准备、pin 与完整绑定 | 单消费者实机与交接已验证 | 三段/四段真实模型调用；C正常pin close及D独立pin0→1→0保持同热worker；不代表完整DAgger验收 |
| ActionChunk 与正常模式薄接线 | 最终 B 同步链的 CPU 组合已完成 | 实际本机 socket→Owner 假硬件模式往返；旧 Owner 缺能力时在 configure 前拒绝 |
| 共享模型退出、启停归属 | CPU已完成；真实pin/GPU责任交接已验证 | 共享服务未卸载；共享执行Owner隔离另见下行 |
| 共享执行Owner生命周期 | 离线候选通过，未部署 | 专属consumer descriptor、attach/detach、外部模式仅退出本工作台；D真实运动/录制时退出C仍待串行实机验证 |
| A/C 联合部署配置 | 离线准备完成 | 31 项包、manager/startup external 成套选择；未安装，B 新能力关闭 |
| 维护 finish 消费端 | 显式接口 CPU 已完成 | 初始 HOLD→END→保夹爪回零→独立原维护 release 已验；D 录制/开爪策略由 D 消费 |
| 中控组件图连接 | API 读写与回读已完成 | 7 个自有节点回填 revision 600；中控 API 消费确认 revision 602，不等于业务重新验收 |
| 时间戳近邻配对、异步原始证据、规划与有限段接线 | 三段及四段实机通过 | 接纳层1/40/40贯通，实测速度保护1；瞬时配对反馈100ms拒绝仍为P2，保留门槛 |

冻结证据：

- [C 消费者接线 v1](/Users/wujie/Developer/worktrees/robot-workbench-module-integration/.evidence/workbench-module-integration-20261103-v1/receipt.json)
- [共享生命周期 v2](../.evidence/workbench-shared-model-lifecycle-20261103-v2/receipt.json)；v1 全套与 v2 增量检查不相加。
- [A/C 联合候选](../.evidence/shared-infer-joint-candidate-20261103-v1/receipt.json)
- [最终 B 同步链的 C 组合](../.evidence/workbench-control-consumer-20261103-v2/receipt.json)：B owned manifest `60b809c6…`、B receipt `4ba823c4…`；纯服务包不被此控制组合替换。

## 1. C 配置入口

输入由可信部署 profile 提供，不能从模型响应建立校验依据。

| 字段 | 默认 | 行为 |
| --- | --- | --- |
| `inference.action_chunks` | `false` | 显式开启后冻结 acquired pin 的完整 binding，并使用 Owner 上下文回调；仅支持当前 remote Owner 路径 |
| `inference.normal_modes` | 不配置 | B 的显式参数原样交其验证；C 不分配制动时长、增益或执行时间 |
| `inference.normal_initial_mode` | `MODEL` | 可显式选 `HOLD`；需要 normal_modes，review/confirm 后只初始化 HOLD，收到新 MODEL 请求才调用模型 |
| `inference.normal_finish_kind` | 不配置 | C 只接受 `maintenance_home_v1`，不自动采用 D 的开爪顺序 |
| `inference.normal_finish_policy` | 不配置 | 与 kind 一起显式给出；五字段 jaw_mapping、execution_settings、max_velocity_rad_s、jaw_speed_mm_s、timeout_s 由 B 核验并冻结 |
| `preserve_begin_jaws` | `false` | 显式为 true 才传入 Runtime；不代替 SDK 能力与输出来源核验 |
| `inference.reference_planning` | 不配置 | 可信部署配置，经 B `infer_reference_policy` 验证、冻结并透传；不替代原 audit-only 动态字段 |
| `inference.finite_chunk_count` | 不配置 | 已发布QA验证接受3或4；要求上述规划策略、action_chunks、关闭录制及normal_modes；每轮固定完整15点，不作为通用DAgger接口 |

C 正常模式目前要求 `action_chunks=true`、工作台 `recording_enabled=false` 且 `normal_modes.recording_required=false`。
C 自带录制器没有 D 的 Raw gate，不能构造假的 closed 回执。D 的 gate-enabled 录制属于 D 接线。
新封套不能替代冻结研究参考；原 `predict` 和独立故障 Stop 保留。
`recording_context_policy=per_episode_review` 属于 D 的 Raw-gate 集成；C 不录制路径显式拒绝，不能静默忽略或伪造成员绑定。

## 2. C → A/B 预测接线

`bind_device` 在真实 pin acquire 后冻结：`session_id`、完整 `model_binding`、`conversion_id`、完整 `action_contract`、`task`。
契约保留 schema/history/sampling 等完整字段；不使用测试精简 fixture，不从待验 chunk 反向建立 binding。

对 B 的 `ControlRuntimeClient.configure_remote_inference` 提供可选：

```python
predict_chunk(request, consumer_context)
```

`consumer_context` 来自 Owner pending request，含 `epoch`、`clock_domain`、`valid_until_source_time`。
C 回调原样调用 `pinned.infer_chunk(request, **consumer_context)`；模型只调用一次。
输出为 A 的 `workbench.action-chunk.v1`，供 B 核验完整身份、源时间、有效期、重复与乱序，再走原 adapter 一次映射。
C 不重新分配执行原点、不排程、不用 RPC 到达时间重标观测，也不把名义 30 Hz 当硬件 deadline。

旧回调继续输出 `{request_id, action14, audit}`。旧 wire 和旧模型返回不强迁；未启用新选项时不加 `predict_chunk`。
B 客户端在 configure 前检查 Owner 的 `protocol_features`；新字段需要相应能力，旧 Owner 缺声明时拒绝，不允许静默忽略后继续控机。

## 3. 工作台正常模式动作入口

沿用现有工作台 HTTP、CSRF、业务控制权及维护窗口校验：

`POST /api/actions/inference_mode`

请求字段恰为路由 `environment`、`business="inference"`，加以下业务字段：

| 字段 | 来源与约束 |
| --- | --- |
| `session_id` | 当前 B runner 的 `mode_session_id` |
| `runtime_nonce` | 当前 Owner 实例 |
| `request_id` | 本次明确操作身份；未知结果查同一回执，不自动重发 |
| `epoch` | 当前请求代次，拒绝旧状态请求 |
| `mode` | `MODEL`、`HOLD`、`TEACH`；END/DISCARD 请求填 `HOLD` |
| `target` | `MODEL`、`HOLD`、`TEACH`、`END`、`DISCARD` |

C 不补字段、不隐式插入 HOLD、不排队，将请求交唯一 Owner。
本接口当前不接受 `recording_gate`，因为上述 C 正常模式限于不录制配置；D 直接按 B/D 契约接 gate。
TEACH→MODEL 拒绝，必须独立完成 HOLD 后再发新的 MODEL 请求。正常模式不得恢复故障或已终结 runner。

输出原样返回 B 的接纳/完成回执，至少区分 `accepted`、`completed`、`status`、请求 `epoch` 与实际 `command_epoch`。
`GET /api/state?...&business=inference` 的 `workflow.inference.normal_mode` 暴露 `mode`、`mode_session_id`、`mode_receipts`、`transition_stage`、`finish_capability`、`command_epoch`、`control_state`、`feedback_healthy`。
完成根据 Owner 回执判断，不用 HTTP 接纳当完成；`ControlCallUnknown` 保持未知，不自动重试。
`READY + FINISH_QUEUED/FINISH` 仍是忙态；不能把上一 episode 已结束误报为当前 finish 已完成。当前 epoch/state 与历史 receipt 分开，不拿旧 retained_hold 当当前确认。
本版本没有新增页面模式按钮；独立 Stop、原取消和保护链保持原入口。

END/DISCARD 只表示 B 固定 HOLD 与 episode 终结，**不包含回零、开爪、SDK 释放或环境退出**。

独立的 `POST /api/actions/inference_finish` 使用原 HTTP 路由与校验，业务字段恰为
`session_id/runtime_nonce/request_id/epoch/ending_request_id/policy_digest`。
其中 ending_request_id 必须指向本 runner 已完成的 END/DISCARD，policy_digest 取当前 `finish_capability`。
C 只在该 reviewed run 已配置 `maintenance_home_v1` 时向 B 转发 `normal_finish`；不接受 `raw_sealed`，不代替 D 录制器封存。
未配置、额外字段或旧身份拒绝；同请求回读、未知不自动重试均由原 Owner 账本负责。

完成回执位于同一个 `mode_receipts`，包含 `op=normal_finish`、`home_completed`、`jaw_open_completed=false`、`release_ready`、`sdk_released=false` 以及原 B 的阶段/输出/反馈证据。
SDK 释放仍是原维护 actor 的单独操作，必须另核 `status=released`、`sdk_close_returned` 和退出；finish 不自行释放或停止模型/相机。
本轮只增加显式薄入口，未改 manager 一键环境退出的原回零路由，也没有新增页面模式/finish 按钮。最终操作方案必须明确采用哪个入口，不把 CPU 维护 purpose 当现场扫掠或原一键退出的新资格。

## 4. 共享模型生命周期配置与输出

manager 的 `model_lifecycle` 只允许 `managed`（默认私有）或 `external`（共享服务管理员持有进程生命周期）。
external 下所有 consumer model start/stop/restart 进程操作被拒绝；模型准备可调用已有共享 API，由服务端准入控制冲突。
显式管理员停服属于 A/宿主运维，可使全部 pin 失效；没有“读 active_sessions=0 后原子停服”的承诺。

existing `stop_model` 运维操作在 external 模式显示为“释放本工作台模型绑定”，仅调用本地 invalidation/own pin close。
结果保留 `lifecycle="external"`、`process_lifecycle_changed=false`、`binding.pin_release.status`：

- `released`：本次 own pin release 返回经客户端核验。
- `not_acquired`：本实例无待释放 pin，且没有未确认释放记录。
- `unconfirmed`：本实例之前的释放响应未知；重复 invalidation 不将其变成成功。
- `unconfirmed_backend_absent`：业务后端已经缺失，未证明远端 pin release，等待服务 TTL/管理员处理。

工作台业务 stop/restart 先处理本 pin；环境退出只要求 C 托管角色停止，共享模型可以继续运行。
private managed 配置仍按原私有行为处理，不能把旧私有配置静默当共享。

注册启动脚本的共享配置必须显式带 `manager_profile`，且该 manager 中的外部模型为受支持的 `kind="user_unit"` 和非空合法 unit。
缺失、空值、非法 unit、docker/process 映射在任何 subprocess/HTTP 前拒绝；这个入口不猜 unit，也不新增 docker 支持。
共享 startup 的列表同时排除该模型 unit，代码与配置须成套冻结。另一 sealed-startup 路径按其既有 spec 校验并核 lifecycle 与 manager 一致。

## 5. mixed366 输入配对与原始证据写入（本版 CPU 候选）

仅 mixed366 物理输入使用有界历史：每相机最多 16 帧、反馈最多 32 份，并删除超过 500 ms 的历史。采集实例、serial、Runtime nonce 或 epoch 变化时清空历史，拒绝源时间倒退。

配对采用公共有效范围内最新的 30 Hz 毫秒时间栅格，规则为 `origin_ms + round(n * 1000 / 30)`；每路图像及机器人反馈选择绝对时间差最小的已保留样本，同距取较早样本。栅格相位属于本次观测历史，不假定与历史训练 episode 相同。没有未来帧预测，也不保证取回订阅端已漏掉的帧。

模型状态来自选中的真实反馈，图像保持原始像素及各自时间戳；`source_time` 仍为选中图像的最老时间，不改成栅格或回包时间。Owner 仍检查图像 500 ms、反馈 100 ms，工作台仍检查相机跨度 100 ms。公共历史尚未形成时只在业务准备阶段最多等待 500 ms，不在模型请求或 Owner tick 中等帧。

`frame_ids` 增加可审计的 `pairing_target_wall_ns`、`pairing_error_ns`、`subscriber_receive_ns`；原始 `host_receive_ns`、采集身份和配对反馈时间不变。现有 frame_ids 透传／身份摘要机制复用，外层 wire 字段不变。

`workflow.inference.observation_pairing` 提供栅格、各流误差、跨度和 `training_alignment_within_bounds`。训练 RGB 25 ms／机器人 12 ms 是对共同目标时刻的误差；本候选仅报告诊断，`training_bounds_enforced=false`，未静默新增硬门。冻结训练证据见 D 的 `/Users/wujie/Documents/ChatGPT/first_work/mixed366-training-alignment-readonly-20261103/findings.json`。逐流阈值不等于相机曝光同步或实测三机跨度分布。

mixed366 原始 input/request、output/audit/chunk 使用一个后台写线程和最多 8 项待写任务。调用线程只冻结不可变内存快照、非阻塞入队，不等待文件写完才 infer 或返回模型结果。输入／输出文件内容、请求身份与原目录规则保留；写线程记录排队、开始、完成时间。完成表示写函数已返回，不承诺 fsync 后的断电持久性。

`workflow.inference.input_evidence` 独立公开 pending/completed/error/status。队列满或磁盘失败锁存错误，不静默丢证据；后续新 Run 拒绝，正在进行的下一次预测入队也失败。已返回的模型结果不能被事后写盘失败撤销，不把未落盘宣称为已落盘，也不从写线程另发控制命令。Stop 不等待该线程；业务正常关闭在控制路径之外等待已入队写入完成。该队列属于原始推理证据，不替代正式录制器或 D 的 Raw gate。

用户最终确定：当前 infer 规划速度及实测速度保护均 1 rad/s，规划加速度 40 rad/s²、夹爪 40 mm/s。每轮完整执行模型源时长 0.5 秒的 15 点，允许整体减速后实际轨迹时长不超过 2 秒，连续 3 轮；同步观测／推理等待单列，不把总墙钟限定为 1.5 秒或 6 秒。此决定覆盖之前严格 0.5 秒执行的解释，不能再用减速参考前缀冒充完整 15 点。

候选透传 `reference_planning={schema:"workbench.infer-reference-policy.v1", curve:"cubic_c2_quintic_ends", velocity_limit_rad_s:1, acceleration_limit_rad_s2:40, jaw_velocity_limit_mm_s:40, boundary_scope:"complete_prepared_segment", timing_policy:"retime_allowed", max_execution_duration_s:2}`，以及 `finite_chunk_count=3, duration_s=0.5, execute_steps=15`；不设置旧 `bounded_live_prediction`。B Owner/client 必须声明并核验 `infer_reference_policy_v1` 与 `finite_sync_chunks_v1`，缺能力时拒绝，不能忽略字段后运行旧 prefix 逻辑。

`workflow.inference.finite_chunks` 原样提供 B 的 requested_count/completed_count/stage/source_duration_s/steps_per_chunk/completed_execution_s/active_execution_s/waiting_elapsed_s/total_waiting_s/current/completed；current 含 index/request_id/reference_id/observation_source_s/start_source_s/end_source_s/execution_duration_s/waiting_s。`active_execution_s` 是累计已执行时间（已完成加当前部分），不是单独当前段耗时。C 展示源时长、每段真实完成及等待，不从窗口下发次数推断完整片段完成。B 的 `finite_chunks_completed` 不替代独立 Stop 回执。

最终 B 四文件与 C 六实现文件、模型接口实际部署基线已完成 ARX6 隔离 CPU 组合 129 项检查，全部通过；明确排除无关 codec 案例。同机同保存输入的五轮暖 CPU 中位数：旧曲线／新限 17.730 ms，新曲线 2.369 ms；全路径极值 15.499 ms → 0.471 ms。原实际参考 metadata/reference_id 精确重建；新曲线完整 15 点约 0.692462 s、v 0.716427 rad/s、a 38.446751 rad/s²、jaw 37.631785 mm/s。曲线比采样点范围最大外扩约 0.133°，未越硬关节限；此值不是物理超调，也不使用起点 0.2° 容差授权它。CPU 数学与进程检查不资格实机端到端耗时、碰撞或到位精度。

本轮唯一发布／恢复／验收入口为 [组合候选计划](../.evidence/arx6-infer-combination-20261103-v1/publication-acceptance-plan.json)，其引用固定制品、完整源清单、前后 selector 哈希与回滚原文。实测速度保护必须在新连接健康空闲并交给 inference 后，经既有 revision-bound execution_settings API 仅改该业务的 limit_rad_s=1，回读实际生效；不热改当前故障实例，也不覆盖其他业务策略。

输入／输出写线程另保存 `pipeline-timing.json` 与 `input-write.json/output-write.json`：主机 monotonic 回调、适配、模型起止及写队列起止；这些记录用于拆分耗时，不作为替换原观测时间的来源。文件以临时文件写完再原子替换，后台完成仍不等于 fsync。

当前已部署上述候选，实测速度保护 1 rad/s 已经现有 revision API 生效并回读。尚未贯通的层是 `SingleArmBackend._queue_reference`：共有参考封包仍硬编码 joint dq≤2 rad/s、有限差分 ddq≤5 rad/s²；这些与模型审计的2/5只是数值相同，实际是独立硬门。此前门槛清单漏列该门，本次补为真实阻塞，不能仅删检查或全局放宽其他业务。

下一修复必须闭合 TargetInterpreter → SingleArmBackend → SDK/native 接纳的同一可信策略及单位、限位、时间和epoch检查，并补真实生产适配器的无设备回归。SDK实际配置速度上限从本批原生输出证据取得：每臂关节 `[5,5,5.5,5.5,5,5]` rad/s、夹爪0.3 SDK m/s；它不是Python封包门或当前实测保护。当前生效的分层值不能混写成单一“速度／加速度上限”。

C另有未部署的统计接线小修，区分 finite 结果已返回／参考已准备／窗口已接纳；原始证据中本批模型确有返回，不以旧UI returned_predictions=0推断模型没回包。该小修不控制设备。

## 6. 共享执行Owner消费者生命周期（离线候选，未部署）

唯一冻结入口：[consumer lifecycle receipt](../.evidence/workbench-control-consumer-lifecycle-20261103-v1/receipt.json)。基于已发布C四段源及B的`shared_consumers_v1`组合；只改C的装配、manager、本地维护、启动与显示，不改B执行源或普通模式物理参数。

- Business profile和manager profile同时设置`control_lifecycle: external`、相同`control_consumer_id`、同一专属`control_owner_descriptor`；manager同时要求`model_lifecycle: external`。管理员先提供`consumer_create`生成的私有descriptor。C核其`consumer_id`和`shared_consumers`，拒绝管理员descriptor和不匹配身份，禁止以consumer profile构造SDK Owner。
- 启动时仅尝试attach已连接的Owner，不acquire SDK；其他消费者持有时保持不可接入，不抢占。管理页“接入本工作台控制”复用自己的后端重载，重新创建attach capability；不恢复旧动作。
- 外部模式“退出本工作台”及停止后端只正常关闭自身模型pin、停止自身业务进程，由已有`ControlRuntimeClient.detach`释放自身claim。共享Owner/SDK/相机/模型不停止，不回零、不恢复原数采网页。若本消费者仍执行/收尾，入口拒绝；若detach保留claim或状态未知，结果明确未确认，不冒称退出成功。
- 本消费者模型prepare/stop的维护冻结仅作用于自己的工作流，不调用共享Owner全局maintenance。其他设备生命周期按钮禁用；直接共享进程启停也拒绝。独占部署默认仍为`managed`，原回零/释放/数采恢复语义不变。
- `start_registered_workbench.py`读取manager可信服务映射，跳过共享Owner、相机和模型unit；不扩展未知映射。设备管理员用的`workbench_deploy`启动清单入口拒绝external consumer配置，避免把消费者当设备管理员启动。配置片段见同证据目录`config-fragment.json`，占位路径必须替换并重新冻结部署清单。

资格：相关回归122项通过；最终增量检查38项、37通过及1项Linux boot专属跳过；数字不相加。Unix socket双consumer＋模拟SDK验证C退出时D继续运动、C空闲detach后D接入、C重载新capability、未确认claim拒绝成功；运维页面fake DOM通过。首次脚本缺静态资源和结果JSON跳过项序列化错误均保留日志，最终收据已通过，不作为实机证据。

部署优先级：**D首个独占消费者验收不必先部署本候选，条件是C旧业务及旧管理生命周期入口保持停用，D作为唯一操作者；若要启用C与D共用Owner，本候选及B共享消费者能力必须先组合部署。** C不为验证此候选重新占用ARX6。真实双消费者启动/退出/重载、D真实运动或录制时退出C、异常进程丢失与保留claim恢复均尚未真验，交D同一操作者后续串行核验。

## 7. 短暂历史配对未就绪的有界等待（离线小差量）

T-b766f378独立候选见[冻结回执](../.evidence/paired-feedback-wait-20261104-v1/receipt.json)。仅修改`inference_workflow.py`共享刷新路径及新增CPU检查，基线为已发布C-four SHA`8bbbf785…`，不含未部署统计改动。

当前相机身份及当前实测反馈先核验；只有历史选择结果的反馈年龄超过100ms时抛既有`PairingPending`，由原`_wait_observation`在原500ms总预算、5ms间隔内等待新配对。当前反馈过期/故障、设备或相机绑定变化、未来反馈时间仍为立即拒绝；相机500ms年龄门不变，不重标旧时间戳。原后台缓存循环保持原行为，不新增线程、推理请求或已下发动作重试。

旧冻结源已CPU复现立即ValueError；候选18项检查全部通过，包括旧配对后新样本继续、永久未就绪按原预算失败、等待期间发生当前硬件故障立即中止、绑定/相机身份变化与未来时间不进入等待、500ms图像门保持。尚未部署或真机重验；D可独立选入当前组合，不作为其他独立开发前置，也不宣称解决所有连续MODEL延迟问题。

## 8. D唯一踏板意图路由的启动选择（离线窄修）

B既定normal录制契约要求D唯一手势路由、禁止并存旧Owner.listener；C只实现此语义的启动映射，不修改B运行拒绝或物理保护。冻结入口：[external intents receipt](../.evidence/pedal-external-intents-20261104-v1/receipt.json)，基线为已发布C-four的`device_bootstrap.py` SHA`e52922a1…`，只含此产品文件和最小测试，不混入第6节external-control生命周期候选。

可信SDK Owner部署profile可显式设置`pedal_input_mode: external_intents`。该模式不导入或创建旧`PedalStopListener`；Runtime若意外已存在listener则启动进入FAULT且不调用backend.connect，不静默移除未知监听。D仍负责启动和核验自己的唯一手势路由及独立UI Stop，这个字段本身不创建外部reader。

默认省略或`legacy_stop`保持原监听器及键码逻辑（默认5，显式如46沿用）。`external_intents`与任何`pedal_stop_key_code`字段冲突，包括null、0和合法旧键码；必须删除该字段，不能通过无效key规避读取。非法mode在read_profile及factory构造前拒绝。此字段与`control_lifecycle: external`不同：前者选择SDK Owner的踏板输入解释，后者选择工作台消费者的服务管理责任。

Runtime的normal recording＋listener拒绝、独立Stop API、native Guard、SDK参数均保留不变。37项启动相关回归通过，最终6项专项全部通过（数目重叠不相加），包含默认行为、external无旧listener、非法/双重映射、无listener时真实Runtime Stop进入双臂模拟SDK保持。尚未远端部署或实际踏板验收；D在其最终normal/C-pairing/jaw组合中选入并走唯一操作者实机验收，旧远端SDK不提前改变。

## 9. RGB读者RELIABLE订阅：D实机修复、本地归仓

`prometheus_rgb.py`仅将订阅QoS的BEST_EFFORT改为RELIABLE，KEEP_LAST/depth1/VOLATILE、原publisher源时间和500ms检查不变。源码SHA从`e53d61f5…`变为`be06f71575ba7a87a84f168c04cf66cec46fe27f202b8a293ee421f36e662ebc`，与D冻结运行版本一致。C只合入本地源并复跑原4项CPU检查，未操作D部署，也未commit/push。

资格及证据统一见[RGB归仓回执](../.evidence/rgb-reliable-integration-20261104-v1/receipt.json)，索引D原始交付及运行数据。C重算12s同publisher对照：旧RGB头/左/右139/210/143帧，新订阅三路各360帧；新ROS接收源龄峰值13.01ms，最大源时间间隔33.65ms。正式唯一SHM writer替换后的600样本约12.08s，每路363个不同源时间戳，最大源龄46.28ms。相机Owner978254、publisher instance `ff44e9d1072745b48d74fcc806ccee65`保持；D提供的后续录制/模式矩阵passed另在回执中索引，不能代替未完成的全部human回归。

发布边界：同一SHM目录仅单writer，固定临时文件名不支持两个reader并发写。比较订阅器应独立取样或使用私有输出目录；正式替换先确认旧reader退出。reader进程身份需用PID/start/boot/cgroup/启动源记录，`capture_instance_id`继续表示真实publisher，不能伪造成reader身份。RGBClient无独立reader descriptor，逐次读取原子SHM文件；上层若有额外descriptor则真实重绑。此实机资格限于ARX6已对照的同publisher链路，不保证其他发布者QoS或长期负载；D继续独占设备和GPU。

## 维护与下一检查点

工作台代码写权：`inference_workflow.py`、页面/业务/装配与对应测试；A/B 模块由各 owner 改，C 只消费已冻结差异。
中控实时图入口为 [工作台节点](http://10.17.0.27:18920/#overview/workbench)，规范目录为 [api-workbench](http://10.17.0.27:18920/#contracts/api-workbench)。
更新只使用 [中控 API 唯一规范](/Users/wujie/Developer/work/anyverse/_context/tools/session-control-center/LIVE_ARCHITECTURE.md)及同目录 `architecture_client.py`，actor 为 `control`；仅改自有节点与本规范，提交后回读，冲突重读合并。
本源内容改变时同轮发布 `api-workbench` 的版本/摘要；阶段、阻塞和估时变化同轮更新相应节点。图/API 的实际回填证据见 [receipt](../.evidence/component-architecture-update-20261103-v1/receipt.json)。
下一检查点为D/B候选就绪后的独立SDK交接，以及共享消费者生命周期的目标Linux/真实隔离验收。生产5rad/s²旧门、三段与四段任务已完成；GPU责任已交D，不再因旧故障回执阻塞或由C继续资源操作。正常模式新物理参数由中控/B冻结，本候选不分配或覆盖这些参数。

### 2026-11-06 四项已批准的续接契约

来源：本聊天用户明确回复“四项按推荐”。继续仅在CPU候选中开发，不代表ARX6发布或真机资格。

- I13/I17：相机服务在绑定时声明可选`pixel_formats`，默认仍为`rgb8-depth16le`，RGB-only为`rgb8`。`rgb8_capture_v1`沿已有服务能力通道声明；实际identity中的格式与登记descriptor一致才能绑定。RGB-only的publish允许`depth=null`，编码产物用`rgb-segment-v1`，不生成深度。旧`rgbd-segment-v1`及读取保持兼容。原生订阅器先发布最新输入，再向有界后台桥接队列非阻塞提交；原始源时刻保留，新鲜度由已有G02/G03裁决，已知拥塞按用途形成缺口，不推断上游未观测到的完整性。
- I15：普通控制Run封存后，沿原Owner事件通道发出`control_capture_sealed`，仅携带`execution_instance_id/business_kind/seal_revision/receipt_ref/manifest_ref`。引用为原文件路径、字节数和SHA256。Run回执复用原`execution_context`中的设备/环境身份。业务端在已登记根内异步核验并登记，各业务复用有界SQLite分页；同修订幂等、旧修订不覆盖新修订，原封存文件不重写。索引与媒体查询均不需要SDK或模型，读取时拒绝路径越界、身份不符及篡改。原始记录与派生索引分离，索引重建不恢复控制。
- I10：现有阶段`detail`增加可选`step_refs`，引用已批准的UC/SC步骤。原phase、job/request身份及旧回执不变；未知阶段不编造引用。摘要过大时仍保留有界步骤引用，完整证据留在原操作回执。界面链接到架构Map；步骤引用和阶段时间戳本身均不构成完成证明，实际结果仍依据后置核验。
- UC09/P4：两种正常回零形状均使用现有`calibration`人工轨迹和冻结策略，录制用途固定`data_required`。`exit_home_keep_jaws`仅决定轨迹/夹爪构造。回零速度仍为0.1 rad/s，不改策略数值；使用当前标定的跟踪、到位和实测速度设置。执行、静止及本Run录制结束均确认后才允许正常释放，异常或Stop不重放旧动作。`research`自身仍要求显式选择用途。

### 推理录制文件管理（2026-11-10 用户确认）

I09 的文件管理以整条 Infer Run 为删除单位，包含全部相机视频、原始帧及关联推理结果。
页面显式刷新 `GET /api/inference-recordings?environment=...&business=inference`，返回
`run_id/started_wall_ns/state/deletable/reason`；普通状态轮询不扫描录制目录。
单选或最多 100 条多选后，页面展示具体任务并确认永久删除，再提交
`POST /api/actions/inference_delete_recordings`，字段为
`environment/business/request_id/run_ids/confirmed=true`。页面的选择与确认绑定设备和服务实例。
后端先核验整批目录归属、封存终态和 writer 锁；活动任务、封存未结束、仍有 writer 或身份未知时拒绝。
SQLite 历史索引 schema 2 从 schema 1 事务迁移，先持久化删除标记并撤销结果索引，再删除 Run 目录；
迟到封存通知和服务重启不能恢复已删除结果。文件删除失败逐项返回，可在刷新后显式重试。
该接口不接入或释放 SDK，不启动模型、相机或控制服务，也不删除其他业务的结果。

### 2026-11-06 常驻记录服务（CPU候选）

来源：用户要求保留原图的常驻进程设计，并批准薄入口、受管`recording`角色及本地Unix IPC；未授权真机发布。

- `scripts/serve_recording.py --root <私有服务目录> --data-root <已登记数据根>`启动独立CPU主进程；`--data-root`可重复。稳定服务根独立于Run目录，业务与Owner通过启动环境中的`WORKBENCH_RECORDING_SERVICE=<服务根>/service.json`登记同一入口。私有descriptor、socket及服务锁限定当前用户；会话目录必须位于登记数据根。接口继续使用既有request/Run/策略身份，不增加SDK字段。
- 主服务持有既有writer和编码器，业务端仅有有界事件队列及后台IPC线程；`try_publish`和`snapshot`不进行IPC或写盘。既有事件总预算在生产端和服务端队列间分配，消息上限不变。RGB/RGB-D取流、原始时间、final-sequence屏障与版本封存格式继续复用原实现。
- 业务进程退出不关闭会话或录像。Linux服务使用本地socket取得的进程身份识别生产端退出；没有final barrier时保存“未提交范围未知”的缺口，恢复不会将其判成完整。服务重启只恢复已登记的中断记录，原writer锁仍被持有时拒绝接管；所有恢复仅处理数据。未收到控制终态时保持unknown，不推断动作结果。
- I15报告常驻服务实例/PID、writer真实身份、活动会话、队列和缺口。服务自身可用与单Run的CaptureReady分别核验。受管启动先准备记录服务再启动消费者；正常停服务要求会话封存且writer实际退出。活跃会话拒绝SIGTERM及管理停服，不强杀；环境退出前先检查封存状态，再退出来源服务。
- 用户另行批准I11绑定时接收已有`control_policy/execution_context`。服务核验策略hash、用途、资源预算与Run目录身份，writer封存并确认退出后，由服务生成原格式`workbench.capture-run.v1`的`receipt-rNNNNNN.json`及最新入口。业务端消费封存回执并异步索引；Infer复用既有历史库，保留原调试结果与不可变版本，媒体回读继续核验pin。业务在提交结束后退出，不影响服务生成回执和重启索引。没有final barrier的生产端退出保留未知覆盖，不能补造控制终态。
- 服务仅在raw封存、writer真实退出、所需Run回执完成后将会话登记为完成；已完成历史的后续清理不等于活动录制故障，已完成request_id不能重新启动writer。用户于2026-11-06批准G34本轮仅做诊断验收：采用非阻塞发布、有界队列、发布耗时及队列年龄统计，不新增运行时硬停止门槛。真机控制周期与Stop性能须在P5发布前实测并冻结预算；CPU微基准不能替代真机性能资格。批准与证据见`g34-performance-stage-proposal.json`和`resident-performance.json`。最终资格与测试证据见`.evidence/architecture-p1-20261105-v1/progress.json`。
