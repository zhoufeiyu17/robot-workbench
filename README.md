# Robot Workbench

统一标定、数采与模型推理工作台。核心求解、采集、规划和执行复用原 C/D 实现；fake与physical共用设备Owner、业务调度、相机CPU服务、录制、存储和读取，仅采样/动作输出由设备适配器提供。完整fake与ARX6真机入口均已部署，具体功能与故障验收边界见下方阶段记录，不以部署成功代替完整生产资格。

维护者：工作台牵头任务；2026-10-21。接口、来源、执行策略或资格变化时更新本入口和证据；历史通过不自动资格新版本。

## 最新入口与使用状态

**2026-10-21：ARX6真机入口 <http://10.17.32.5:18868/>，独立Owner V14/env2已发布。**
标定、人工目标、回零、规则数采与Infer共用目标解释器和原生q/dq窗口，Owner生命周期与业务后端隔离；
共同相机、录制子进程及浏览器WebGL目标预览独立运行。18:34 CST阶段核验为连接、READY、反馈健康、自动执行关闭；
实际身份以任务本地 `continuation-state.json` 指向的实际发布与状态收据为准，不能由Git HEAD推断。

当前合同、组件／时序图、已恢复故障、验收数据与优先级见
[控制链路阶段状态](docs/control-runtime-status.md)，可移植统计和内存曲线随该文档入库。
V14发布前516项Linux CPU检查通过；30分钟真实录制／编码路径完成377轮、1131视频，机器人与模型为CPU替身。
本次Git收尾完成标准检查入口的CPU／前端覆盖：1663次单测执行（1627通过、36跳过）及6个Node检查；
复用未变的通过前缀与修正后的完整后缀，过程及适用范围见阶段验收摘要。
**历史索引仍全量常驻，全服务长期常量内存尚未通过；≤0.2°全关节精度目标也未全部完成。**
本次Git阶段收尾保留常驻服务与资源预留，不重新执行真机或数采。

## 历史运行记录（2026-10-15及以前）

以下版本、入口、计数及“未提交”等描述为对应日期的历史事实，不代表当前状态。

2026-10-15当时版本`persistent-20261015-173438/session-1-d888ccf2e9b2`，入口 **http://10.17.0.27:18866/** 。完整fake已连接、READY、反馈健康、标定持权，三相机可读。当前head v000003/wrist v000004、142条到位统计及30条历史视频保留；旧71条记录仅是历史基线。模型服务未自动加载或执行；使用Infer时先显式交接、加载与预热，再确认一轮40源秒。真机IP面板连接已部署的兼容设备服务，不会自动部署SDK、连接控制器或执行动作。

本版共用三相机1GiB缓存、二进制IPC与流式分段录制；Infer记录原预测、设备转换、位置投影、最终下发和实际完成history，速度/加速度仅记录，不做碰撞检查，停止/交接撤销迟到输出。40秒录制允许30ms覆盖缺口，实际时间不改；维护场景允许0.1%四元数模长误差，解析前归一化并留证。服务与GPU预留无自动截止，45分钟只读监控已指向本版。

验证：1077次完整CPU执行无失败（22项外部环境缺失跳过）、前端通过，49项远端真实codec/HTTP测试通过。固定Base379新进程strict load及两份历史输入输出误差均0，热HTTP约128/131ms。独立fake完成40源秒执行并生成三视频，但杯子未获得托盘支撑，任务失败；该轮使用历史复用标定，不冒充当前安装的任务资格。旧raw少2.5ms按新30ms口径复核接受，旧manifest不改；同批视频各1201帧/39.9975s与源时间保持，导出由约53.6s降至22.3s。未重复标定或完整数采。

当前状态、保留项与限制见[交付收据](.evidence/infer-unification-20261015/persistent-20261015-173438/delivery-receipt.json)和[实施契约](/Users/wujie/Developer/work/anyverse/_context/notes/unified-workbench-refactor-20261009/17-common-camera-and-policy-infer-20261015.md)。以下较早运行段落仅供追溯；实际真机部署/执行验收及模型任务成功率仍待完成。本轮没有Git提交或推送。

首次发布候选165202因实测四元数模长1.0000000298600251导致场景恢复失败，失败与清退证据保留；173438包含解析前规范化修复，实际启动、场景恢复、三相机及设备连接均已核验。原始快照未修改，修正记录在本run的`scene-restored.json`。

## 实现与来源

- `calibration/`：原标定工作流、求解器、结果库与 Owner；来源见 [calibration-source.json](migration/calibration-source.json)。头部空库使用名义 A1，双腕依赖本轮头解并只看贴纸。
- `rule_based_workbench/`：D174 运行文件及13个最新控制候选 Python，来源见 [collection-source.json](migration/collection-source.json)。30Hz冻结目标、100Hz Owner、数据故障与运动故障分离等既有策略保留。
- `robot_workbench/runtime.py`：每设备/环境唯一 Owner、业务凭证、显式交接、停止证据及安装变更撤销未执行审批。交接保留连接与保持；切栏目不交接。
- `robot_workbench/service.py`、`bridges.py`、`workflows.py`、`web/`：共享 HTTP、原业务适配和四栏目页面。结果查看与连接独立；候选 mesh/回放没有质量门；正式安装独立校验成组依赖。
- `robot_workbench/sample_clock.py`：wall/source 时钟区分；fake命令的发令时刻、最近反馈采样及实际物理应用步分别记录，不重标时间来掩盖延迟。
- `robot_workbench/physx.py`、`physx_camera.py`、`physx_task.py`：同一物理 plant 的命令、实测反馈、三相机 RGB-D 与原生任务观测。任务结果不替代运动或数据完整性。
- `robot_workbench/calibration_profile.py`、`collection_setup.py`：本轮安装转为新数采 profile/模型并冻结接入配置。估计零偏只应用一次，观察 plant 不因标定结果改变。
- `detector_adapter/`：两个薄 CLI/服务入口；检测运行时与权重继续作为外部只读依赖。来源见 [detector-source.json](migration/detector-source.json)。

原工作树和旧运行路径保持独立。原生 robot_sim、A1资产、Isaac、cuRobo、检测环境与硬件 SDK 是外部依赖，不复制权重、SDK二进制、密钥或私有服务配置进仓库。原生来源起点见 [native-dependencies.json](migration/native-dependencies.json)：129项历史provenance包含121项native和8项workbench helper，并非129项全部原生源码。

## 现场相机监控

2026-10-15：统一工作台新增独立的“现场相机”栏目，入口为 http://10.17.0.27:18866/?view=site-cameras 。可保存不同 ARX 的名称、内网 IPv4 和只读桥端口，选择头部与双腕或单相机放大，暂停/恢复刷新；画面陈旧或断线时明确标记。切换监控设备不切换控制设备或控制权。当前实际接入 ARX6（10.17.32.5:18873）；其他设备须先启用相同只读桥，不能仅填写机器人控制端口。

网关在 3090 station 提供页面与配置持久化，ARX 端 `serve_site_camera_bridge.py` 只代理既有相机预览的 `/streams` 和已知 JPEG 帧，不启动采集 SDK、修改控制或占用 GPU。独立入口 http://10.17.0.27:18866/site-cameras 在 fake 后端不可用时仍可访问。新增服务无自动退出时间；原临时预览页的限时不影响本入口。

2026-10-15 补充：ARX6 相机源已与数采任务解耦。`robot-workbench-camera-owner.service` 复用原 RealSense 发布器（RGB+IR、640×480@30、ROS domain 62），`robot-workbench-camera-preview.service` 复用原 ROS 预览器并监听本机18874，原只读桥18873转发该独立预览器。数采停止后相机仍运行；数采再次启动时，其相机子进程通过 `prometheus.session.shared_camera` 复用同一发布器，不重复打开硬件。该接入分支仅在 `~/.config/robot-workbench/camera-owner.json` 存在时启用；相机参数/域不匹配、心跳丢失或源实例重启会明确失败，不静默争抢设备。停用该部署前须先确认数采已停止，再清退独立服务并恢复发布入口；不能只删除状态文件。

新增 `robot_workbench/camera_owner.py` 管理原发布器的独立生命周期。设备端状态与网关连通性分别报告，页面区分“源相机已停止”“源画面未更新”和“网络或预览服务连接中断”。用户已明确授权本次原相机入口变更与仅相机启停验收；未启动 CAN、机器人控制、触觉或数采。两次实际相机复用客户端就绪/退出后，源实例与发布器PID均未变；另验证独立服务停止时原发布器和三worker退出、状态报告停止。首次启动三路出图成功，重启验收曾遇左腕USB 2.1缺少右红外profile且单RGB也收帧超时；用户允许接受USB2，本轮一次仅左腕软件复位后链路恢复USB3.2。最终沿用RGB+双红外640×480@30，未降低录制规格；相机源再次重启后三路实时恢复。独立源每次启动会重建自有预览订阅，修复旧订阅跨发布器重启后只留旧图的问题。USB2不是一律禁止条件，但不能把本次不支持的流组合或实际收帧超时当作可用。整批真实数采启停未执行；本轮浏览器自动化超时，接口验收不冒充最终页面视觉验收。证据见 [.evidence/site-cameras-20261015/independent-receipt.json](.evidence/site-cameras-20261015/independent-receipt.json)。

发布文件、服务身份、CPU/HTTP 与浏览器验收见 [.evidence/site-cameras-20261015/final-receipt.json](.evidence/site-cameras-20261015/final-receipt.json)。真实验收覆盖 ARX6 三路画面、切栏、暂停恢复与配置保存；双设备隔离由 CPU HTTP 测试覆盖，不冒充第二台实体设备验收。

## 本机 CPU 开发

Python 3.10–3.12，无需机器人、SDK、ROS或GPU：

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
sh scripts/check.sh
# 可选：约30秒的墙钟故障矩阵，使用新的证据目录，不能覆盖旧回执。
.venv/bin/python -B tests/test_l1_fault_matrix.py --evidence-dir .evidence/l1-current
PYTHONPATH=. .venv/bin/python -B -m robot_workbench.service \
  --protocol-fake --state-dir .evidence/local-device --port 18866
```

浏览器打开 `http://127.0.0.1:18866/`。这是明确的 **L1 协议 fake**：可验证连接、人工预览/执行、权限与停止，未配置 C/D 后端的能力会明确不可用，没有物理相机图像。它不能替代下方 L2。

设置 `DETECTOR_TEST_RUNTIME_ROOT` 为已核对的外部检测源码根，可启用真实catalog的CPU闭包正负例；默认缺该源码时会明确报告跳过，不代表通过。运行 `serve_physx_device.py`（含dry-run）或setup-only时则始终使用部署声明的detector解释器和源码验证实际发布包，不能跳过。

原 C 资产检查可显式配置 `AC_ONE_URDF`；默认未配置的fixture跳过会报告，不算通过。界面修改后需重新加载浏览器；HTTP/DOM测试不替代视觉验收。

## 历史阶段状态与验收

2026-10-15 相机专项收尾：**CLOSED_WITH_RESIDUALS**；整体重构仍 **NOT_CLOSED**。现场相机监控与独立发布/预览生命周期已上线，原数采相机客户端两次实际复用/退出、相机源重启后的三路恢复已验收。收尾只读检查三路仍live、源running、ARX6原采集任务stopped；3090网关与DSW转发active，174136 fake管理入口可达。当前相机源是RGB+双IR，不是共同RGB-D后端资格；共同相机/infer候选与已部署版本分别记录。后续由牵头任务补下一次正常数采整批启停、补新版页面视觉证据；不为收尾重复采集。相机代码仍在未提交工作树，未commit/push；当前HEAD不等于在线发布包。机器入口见[检查点](.evidence/s2/active-task-checkpoint.json)，相机回执见上节；下方10月14日热就绪/持权为历史测量，不当作本轮新验收。


2026-10-14 18:30：当前长期版本为 `persistent-20261014-174136/session-1-b1c49f9be908`，入口仍为 **http://10.17.0.27:18866/**；完整 fake 已连接、数采持权，检测/Scene/cuRobo 初始化与预热完成（本次478.31墙钟秒）。头/腕/安装清单字节SHA、71个到位样本统计及30条结果目录项保留。用户明确批准将杯子初始化到固定原点前+30/左−15mm；这次是场景重建，未执行抓取/数采，不能冒充此前失败动作的终点。只读 `/api/fake-scene?environment=fake&business=collection` 已连通，重建后的杯位与目标相差约0.001mm。旧130006的21项进程身份与两GPU清空后才释放；新预留和automation-2均已切换，无自动截止。

新增共享“执行误差设置”：标定与数采各自保存到位/跟踪策略，支持12关节到位阈值、夹爪阈值和连续跟踪超差时长；两项均“仅记录”时按既定时序结束，不因位置误差中断或等待收敛，采图静止、反馈健康、停止和位置边界仍独立有效。未自定义标定维持旧保护；页面明确说明旧被动轴门槛，保存后统一按自定义跟踪策略。配置只能持权空闲时更改，并撤销旧审批、绑定新执行、跨child保留。Fake可按种子为每次动作抽取12关节均匀偏置，0.5源秒平滑施加到真实plant驱动；名义命令、实际目标/读回、限位投影及偏置证据分别保留，默认关闭，物理环境不可注入。

本轮完整CPU检查退出0：983次unittest执行、其中19项外部环境缺失跳过，前端套件通过；最后增补返回场景12项、HTTP30项、管理代理11项通过。11项偏置测试包含真实Runtime/Owner/PhysxBackend与CPU plant串联：同偏置下仅记录完成，严格跟踪超差中断，关闭偏置正常完成；不冒充原生GPU运动资格。完整fake实际验证配置启用/恢复与落盘，目标未变且未创建动作实例。原失败`88c7394f...`为落桌期间接触相对速度80.250mm/s超过80mm/s后，16个金属接触失去容许条件；运动完成，过程验收失败。没有修改接触门槛或下放轨迹。本版补齐失败阶段/数值说明和失败后独立终点观测，保留首次失败、终点及重启场景，后者不提升失败资格。此前旧回执只记录落桌失败帧（相对固定原点前28.516/左−12.656mm），最终杯位无法追补。该段支撑→释放交接仍需按已有失败轨迹定点修正/复验；不重跑整套数采。证据见[本轮交付](.evidence/execution-settings-20261014/final-receipt.json)和[失败诊断](.evidence/scene-recovery-failure-20261014/diagnosis.json)。

以下130006为本次升级前的历史状态。

2026-10-14 14:20：当前为 `persistent-20261014-130006/session-1-6fa1e2c2dd00`，长期入口 **http://10.17.0.27:18866/** 已恢复完整 fake、三相机与数采热就绪，保持数采控制权。本次首次初始化473.94秒已结束；没有替用户开始新的采集或动作。标定组、71个到位样本、24条已验收历史视频保留；原3条失败录制的L3诊断文件仍保留但历史目录入口尚待补齐。监控automation-2已跟随新run，无自动退出。

新增“恢复场景”：仅输入相对固定原始杯位的前后/左右偏移，各轴±50mm，向前（远离机器人）和机器人左侧为正。点击后自动生成当前计划并弹一次确认；修改偏移、切换上下文、核验环境或过期都会撤销旧审批。恢复采用最新成功放杯轨迹，原始杯位不随循环累加；父轨迹与原点落盘，服务早停重启也保留回退源。零有效位移沿原轨迹反向；非零位移先在悬空位置对齐，固定XY/姿态落杯，释放撤手后回位。仅限fake；没有新的碰撞通过声明，位置、动态和原生接触验收门槛保持不变。

当前原点0、+30mm及四轴/四角±50mm共10项真实URDF CPU检查通过；非零构造0.996–1.248秒、FK重验0.294–0.336秒。独立PhysX实际前+30/左−20mm取放通过：目标误差2.614mm，稳定桌面支撑、释放、撤手、回位均通过，录制调用0；动作43.67源秒。首次同时平移/下降方案的接触失败证据保留，修复后未放宽阈值。最终93项后端定向回归、前端双套回归与页面输入视觉验收通过。两轮父轨迹晋级和原点保持经过CPU集成验证，未重复完整数采，也不将单次实际恢复外推为所有多轮循环或真机资格。另一个人为移动原点后的−100mm压力目标越出当前UI绝对区间，在撤退段被IK门拒绝，未发布为可执行计划。详见[功能交付回执](.evidence/scene-recovery-20261014/final-receipt.json)、[当前界面](.evidence/scene-recovery-20261014/ui-recovery-ready.png)。

以下为已结束运行的历史证据，当前运行状态以上方及动态检查点为准。

历史，2026-10-14 03:21 维护发布：`persistent-20261014-031356/session-1-d88a99e51f67` 当时完整 fake 已连接、数采持权、三相机恢复，原标定组与71个到位样本保留。录制改为三相机共用 **1 GiB** 字节预算，移除每路16帧上限；租约建立后即消费预录制帧，保留最早帧，最终执行时间线只绑定一次，不以编码就绪作为运动门。旧数据按原墙钟节奏回灌：1170命令、3902反馈、每相机1181组RGB-D的像素/深度/时间戳一致，17项真实codec与恢复测试通过，末输入至封存5.013秒、总缓冲峰值6,169,198字节。原8.638秒首帧阻塞缺少细分日志，不能归因于单一组件；本次补齐分阶段耗时。见[录制回执](.evidence/online-recording-overflow-20261014/remote-cpu-final-receipt.json)。

同版增加显式取杯维护回放：固定成功父轨迹、当前安装组、实际场景和新Owner反馈，反向派生1127目标、37.57仿真秒、原97%时序，夹杯金属间距65mm、放杯83mm；不重新碰撞规划、不录制训练数据，仍保留一次明确执行确认。取回发布实测3.22秒；当前仅生成并审阅，**尚未执行取回**。审批凭证仍为30秒，过期需重新生成。部署先前024746因把杯在咖啡机上的恢复状态送入新任务初态检查而失败，现改为正常环境初始化后恢复原场景，并保留失败证据。24条已验收历史视频仍可访问；上一失败录制的3条L3诊断视频文件已核验保留，但当前历史导入仅接受完整成功记录，尚未重新出现在新页面目录。见[当前回执](.evidence/cup-return-assessment-20261014/final-receipt.json)、[取回页面](.evidence/cup-return-assessment-20261014/persistent-20261014-031356/return-review.png)。本轮没有重复完整数采、重标定或真机动作。

以下001822为上一版本记录，已停机清退；当前身份以上方及动态检查点为准。

2026-10-14维护发布：内网入口不变，当前为 `persistent-20261014-001822/session-1-241fc1f62f09`，已恢复完整 fake 热就绪。151%与123%两次旧失败均为路径加速度超限，原碰撞验收通过；123%原路径100%已达4.997765/5 rad/s²。现在每段最终路径生成后立即按请求速度校验完整前缀，超限即停止后续规划；页面明确显示阶段、左右关节、数值、上限及处理建议，原始错误仍保留。30%～170%是可选倍率，具体路径仍受原2 rad/s与5 rad/s²上限约束。HTTP内网兼容使用安全随机源生成UUID，显示/普通动作/服务管理三入口共用；实际整机视角渲染已验收。此次只复算旧数据、发布并预热，没有新观测、规划GPU作业、动作或采集。标定组、71样本统计值及24视频保留，历史Range206通过；自动内存相机登记修复已随本版发布。本次初始化483.63秒已完成；磁盘缓存复用不代表重启可保留进程内热状态。监控automation-2已跟随新run。详见[本轮回执](.evidence/collection-plan-failure-20261013-2347/final-receipt.json)。

以下2026-10-13入口迁移记录为历史；实时运行身份以 `.evidence/s2/current-full.json` 为准。

2026-10-13内网入口最新交付：**http://10.17.0.27:18866/**（管理页`/session`）。3090station仅运行CPU网页代理与专用SSH传输，GPU/仿真/数据仍在734的181854/child3；Mac旧18866转发已停用。检测、Scene、cuRobo已预热，当前可由用户从“2. 观测与规划”开始，速度草稿99%，未替用户采新观测或执行轨迹。最后一次初始化488.071秒已完成，LAN三路图像/同源POST/幂等准备/历史Range播放均通过，浏览器截图已核验。连接级TCP MSS1000缓解长SSH中断；未改系统路由或加密验证。见[热就绪与迁移回执](.evidence/hot-ready-20261013/final-receipt.json)、[部署/故障/恢复说明](/Users/wujie/Developer/work/anyverse/_context/notes/unified-workbench-refactor-20261009/16-station-entry-and-hot-ready-20261013.md)。当前冻结后端通过运行metadata补齐内存相机登记；本地自动登记修复待下次冻结发布，不热改当前运行源码。新监控`automation-2`每45分钟只读核验，无自动运动或退出。

以下为本次迁移前的存储交付与历史验收，当前入口/状态以上方和动态检查点为准。

2026-10-13最新：`persistent-20261013-181854`已部署有界内存、流式无损RGB-D分段、兼容读取及只读中断恢复。每相机16帧/32MiB队列、每段120帧、后台原子提交；源帧/时间/身份保留，溢出明确失败。使用141541同一旧批1170命令、3902反馈、每相机1181组RGB-D按原墙钟节奏回灌，全部RGB/深度像素、源时间、命令反馈一致，仅四个新录制租约字段改变。末输入到完整封存**4.081894秒**，末反馈到完成4.123901秒；浏览器三视频派生**另需26.800621秒**。新raw131文件/400,453,724字节，队列峰值每相机10/16。通过的是原实际到达节奏的持续吞吐，不是三相机各30墙钟fps上限或一次新采集实测；本轮没有重跑规划、动作、标定或完整采集。

最终实际codec/集成18项及真实SIGKILL恢复1项通过；908次完整CPU检查是异步提交修改前的版本，后续定向验收不冒充最终全量重跑。管理器及fake现已READY、持权数采、空闲，三路图像经18866核验，原安装结果、71样本和24视频保留；数采初始化/预热仍需显式执行。原生首次启动曾停滞，保留现场并复用同版本缓存后第三child恢复；不宣称全部根因已确定。浏览器自动化受此前网络错误页影响，最终截图未验收，API及图像证据单独记录。长期入口无自动截止，资源监控`fake`已恢复每45分钟核验。见[交付回执](.evidence/recording-streaming-20261013/final-receipt.json)、[实现/读取/恢复说明](/Users/wujie/Developer/work/anyverse/_context/notes/unified-workbench-refactor-20261009/15-streaming-rgbd-storage-20261013.md)。

历史（本轮输入来源）：`persistent-20261013-141541`已交付连续有效3源秒观测（至少9帧、缺检或杯心跨度≥6mm重置、异常之后首帧重新计时），边采边检测并导入冻结结果，避免二次检测。规划保留碰撞验证；同plant执行移除逐物理步重复几何检查，保留真实接触和任务判定。一次正常页面审阅的99%完整fake通过：动作完成、数据有效、任务成功，1170命令/3902反馈/每相机1181组RGB-D，三源视频与独立辅助L3保存；原18视频、71到位样本和安装结果保留，无新标定/cold/L4。

本次热观测到计划106.795墙钟秒，完整执行保存805.190秒；执行反馈覆盖326.910秒、末反馈至raw封存344.601秒、视频导出97.701秒、其他35.977秒。相比旧基准执行更快，但后台RGB-D写入积压成为主要尾部等待；不是严格同路径A/B。详见[交付回执](.evidence/collection-observation-20261013/final-receipt.json)、[实际分账](.evidence/collection-observation-20261013/141541-execution-audit.json)、[完成页面](.evidence/collection-observation-20261013/141541-completed.jpg)。CPU全套覆盖883次执行：876通过/7既有fixture跳过，初次exact-kwargs测试修正后完整后缀通过，原失败日志保留。长期18866和两GPU无自动截止，缺失的后台监控仍待恢复确认；动态状态以检查点为准。

2026-10-11：已按用户要求完成中间状态梳理，并完成同一冻结版本下的 **3/3 新空库完整 fake 验收，每例独立9/9通过**。本组冷启动验收到此结束。整体重构仍为 **NOT_CLOSED**：部分专项交互/恢复资格及正式真机部署仍未完成（长期fake入口已于10月12日交付），不能标为生产就绪或“95%完成”。

冷验收冻结产品为 **757b290c12dde26732474cb58cf99b5a62cbc914**，375项源码与验收driver固定不变。当前代码、数据和证据的机器入口是[验证摘要](migration/validation-summary.json)，三例联合核验为[总回执](.evidence/s2/cold-suite-v7-qualified-20261011.json)。动态运行与资源见[续接检查点](.evidence/s2/active-task-checkpoint.json)。

| 已完成范围 | 证据与边界 |
| --- | --- |
| 共同入口与控制 | 四栏目、单Owner、显式交接、原C/D业务、结果绑定和诊断已实现；26项旧功能对照与已批准实现补齐完成，不等于每项全部浏览器/实际后端分支已验收 |
| 冷验收时CPU/L1 | 791次测试执行：790通过、1旧fixture跳过；L1故障矩阵27/27；软件stop P95 4.128ms/max4.660ms，仅本机软件资格 |
| 姿态显示预览 | warm-145124指定profile：mesh关23.17–23.52FPS/P95≤173.5ms，开22.30–22.58FPS/P95≤183.7ms；不外推其他主机、负载或PhysX慢执行画面 |
| 封存与导出 | 同一完整raw总耗时576.222→309.245s，降低46.33%；解码帧、PTS、MP4 SHA一致，限实测输入/主机/存储 |
| 到位统计证据 | 已复核实际浏览器截图、文本和当前两例各71个到位样本及相关源码；原C12缺证据项可关闭，不当作实体精度证明 |
| 预动作恢复CPU | 现成恢复脚本已单独补跑通过，覆盖安装/场景变化、未知/部分执行拒绝及零命令恢复；未冒充浏览器或GPU故障恢复 |
| 显式显示渲染器恢复 | 在第三例原始终态和9项审计保存后，单独恢复显示进程19.052s；旧PID退出、新目标JPEG成立、Owner绑定与三相机源时钟保持；仅健康实例显式恢复，不是故障注入或浏览器点击资格 |

[当前CPU/L1](.evidence/s2/readiness-diagnostics-full-cpu-20261011-v1/receipt.json)、[UI对照回执](.evidence/s2/parity-ui-20261010T174002Z-verified/receipt.json)、[封存/导出对照](.evidence/s2/cpu-export-perf2-20261011-0115/comparison.json)、[专项证据复核](.evidence/s2/targeted-evidence-reconciliation-20261011.json)、[预动作恢复检查](.evidence/s2/targeted-resume-cpu-20261011-1443.json)、[显示恢复回执](.evidence/s2/full-scene-xy-20261011-134701-display-recovery-v2/receipt.json)均保留。

## 已结束的三例完整 fake

三例均从空结果库重新采集头34张＋双腕29张，求解并成组安装，随后以本轮标定构造新模型/观测/规划，完成同plant Owner执行、原生任务、完整RGB-D、源录像和独立L3辅助回放。L3不承担实际任务成功证明。

| 案例 | 本轮run | 命令数 | 反馈数 | 每相机RGB-D帧数 | 独立审计 |
| --- | --- | ---: | ---: | ---: | --- |
| 受控左右J1零偏±0.003rad | full-device-j1-20261011-110144 | 3215 | 10718 | 3227 | 9/9 |
| 名义场景 | full-20261011-122428 | 3234 | 10782 | 3246 | 9/9 |
| 杯子[+.01,-.01,0]m、咖啡机[-.01,+.01,0]m | full-scene-xy-20261011-134701 | 3624 | 12082 | 3635 | 9/9 |

三例均原driver正常完成，未在失败后续跑来计通过；设备真值仅供plant/evaluator，不进入求解与规划。三例原始终态、审计、native回执及影像已保存，清退前后native/raw/三源视频各五项散列保持。

最后案例的[实际头相机终帧](.evidence/s2/full-scene-xy-20261011-134701-final-cameras/head.jpg)、[源录像](.evidence/s2/full-scene-xy-20261011-134701-source-head.mp4)、[独立审计](.evidence/s2/full-scene-xy-20261011-134701-audit.json)可直接查看。内容证据不冒充新的浏览器版式验收。

## 两个业务工作区与当前入口

2026-10-11：根据旧版体验审计及用户决定，已保留共享Owner/权限/数据契约，恢复两个业务工作区（d9b0a2a）：

- **标定**：度数滑块和实测值并列，输入自动只读预览；七阶段与质量/有效帧摘要；逐姿态前后浏览；固定帧/相机更换同批次候选。人工目标草稿按设备/环境/业务保存，恢复草稿不会恢复执行审批。
- **数采**：准备、观测规划、审阅执行三个动作及速度设置同时可见；显示原阶段耗时、下一步、环境、来源和录制/封存/任务错误。后端按完整执行身份解释状态，前端不重复推断任务成功。
- **结果**：标定候选与数采记录分别查看；历史按任务分组，同任务同相机双侧播放，刷新保留选择。原生任务结果与L3辅助回放仍分开。

主体完整CPU796次执行（789通过、7明确跳过），最后45项历史/HTTP/serve/resume定向检查通过，Node交互检查通过。[代码验证回执](.evidence/ui-workspaces-20261011/final-code-validation.json)。CPU保存图像fixture验证5°换算、草稿恢复及同帧A/B切换，不冒充物理渲染资格。

### 当前完整fake与服务管理

当前运行见上方181854交付及[动态检查点](.evidence/s2/active-task-checkpoint.json)，以下为历史速度验收。

**历史：2026-10-13 11:12**：`persistent-20261013-104958` / `session-1-b4539bbdf346` 已连接READY、持权数采、预热完成、无自动动作。数采用户速度区间已为30%–170%，100%基准、固定等待与2rad/s/5rad/s²目标上限保持；速度变化仍需新审阅。真实025402冻结轨迹完整CPU重定时30%/100%/170%分别120.533/38.367/24.067源秒，170%所有控制检查通过；实际浏览器两端点可达，测试后恢复99%草稿，未执行新轨迹。旧非MotionGen fallback>100%明确拒绝，不能冒充支持提速。原标定、71样本与18历史媒体身份保持。[回执](.evidence/collection-speed-20261013/final-receipt.json)、[CPU原始检查](.evidence/collection-speed-20261013/cpu-speed-receipt.json)、[实际170%页面](.evidence/collection-speed-20261013/ui-speed-170.png)。

[最新耗时拆分及边界](/Users/wujie/Developer/work/anyverse/_context/notes/unified-workbench-refactor-20261009/13-collection-speed-and-latency-20261013.md)：131秒包括9帧/8源秒新观测、重复环境预检、检测与接触几何；971秒中747.626秒确为执行反馈覆盖、87.550秒末反馈到raw封存、99.395秒视频导出，余36.421秒。数采每物理步同步原生任务验收不同于标定单姿态；尚未测出各块墙钟占比，不宣称性能瓶颈已解决。本次只做读日志解释和速度范围交付，没有170%实际运动、完整数采重跑或物理资格。

下方031906及更早运行属于历史交付，以本段和动态检查点为准。

**2026-10-13 凌晨热复用交付**：`persistent-20261013-031906` / `session-1-3d8bea31002e` 在[18866](http://127.0.0.1:18866/)已连接READY、持权数采、无自动任务；原无自动退出约定保持。新增单一“初始化与预热”按钮，准备资产、检测器、常驻观测场景和规划模型。安装标定只提示待准备；普通杯子/咖啡机移动、任务中断及复位不重建健康模型，重新观测后热规划。真正进程退出、模型或标定变化会明确失效，需要再次显式准备。

实际移动杯子XY[+.01,−.01]m、咖啡机XY[−.01,+.01]m后完整fake成功：观测→计划131.005秒，其中规划与碰撞检查48.492秒；确认执行→全部保存970.992秒，合计18分22秒，不含首次初始化和人工审阅等待。1168命令、3895反馈、每相机1179组RGB-D，运动完成、数据有效、原生任务成功，源录像/L3均保存。场景/模型/引擎确实复用，模型build/init/warmup均0；动态更新0.03884秒，同一worker身份、不同观测与首帧SHA已核验。99%源轨迹38.933秒不等于墙钟实时；L3辅助回放仍按批独立启动，与同plant执行并行。

全链验收版025402与当前031906仅差准备计时及其测试/合同；核心业务代码逐字节相同，未为计时重复采集。最终准备总计508.25秒，阶段内实时增长，重复按钮核验attempt仍1且完整warmup回执保持。原头/腕标定ID、71到位样本、18段历史视频实际HTTP SHA/size均保持（含本批6段）。875项CPU逻辑覆盖868通过/7明确跳过，另有断言和Node；这是分段接续检查，原失败保留，不是一次无中断全绿。没有重新资格历史冷3/3或实体L4，代码仍为未提交增量。

[设计与边界](/Users/wujie/Developer/work/anyverse/_context/notes/unified-workbench-refactor-20261009/12-collection-warmup-20261013.md)、[最终回执](.evidence/collection-warmup-20261013/final-receipt.json)、[完整计时](.evidence/collection-warmup-20261013/025402-benchmark-receipt.json)、[准备界面](.evidence/collection-warmup-20261013/ui-final-warmup-ready.png)、[历史结果](.evidence/collection-warmup-20261013/ui-benchmark-history-preserved.png)为本次证据。资源记录`unified-persistent-20261013-031906`，唯一动态源仍为GPU使用表。原fake监控工具返回不存在，恢复询问尚无回复；开发中已做实际核验，不宣称后台监控生效。下一步先对plant执行/导出分段计时，再补共享故障恢复入口；其余业务分支与真机L4仍独立保留。

以下10月12日及更早部署、PID和“当前”均为历史交付，不代表最新服务：

**2026-10-12 长期入口已恢复并验收**：[工作台](http://127.0.0.1:18866/) · [手动启停](http://127.0.0.1:18866/session)。用户已明确取消自动退出时间；CPU入口、完整fake、显示renderer、检测/规划worker和本机SSH转发均无到期退出。页面显示“长期运行（无自动退出时间）”。单次任务超时和运动保护继续保留；停止或故障不会自动重启仿真或续跑动作。

当前冻结部署 `persistent-20261012-211606` / `session-1-c2cf7b981c6a`，C/D就绪、fake连接READY、无自动任务；三相机、目标图与mesh通过检查，mesh已关闭。原头/腕/几何ID、71个到位样本及6历史视频SHA保持，没有重复标定或数采。当前代码为独立分支上的未提交增量，运行文件按发布包SHA冻结；当前[验收回执](.evidence/manual-click-20261012/final-receipt.json)与[CPU回执](.evidence/manual-click-20261012/final-cpu-receipt.json)优先于下方首次常驻记录。

**人工调姿检查已改为内接粗球**：只检查双臂与机身自碰撞，排除外部场景；7,590球/181组件预生成缓存，全部通过原外壳包含性核验。保留一次动作确认。实际同一78点路径的两次检查合计约1.0–1.2秒，Owner请求约2.4ms；204054的浏览器点击到弹窗2.015秒，用户已接受。211606保持相同前端和两次检查，仅补统一位置边界容差，原始反馈/计划不改。额外单次检查和快速轮询候选已撤回，不在运行源码中。

确认/取消和到期均未发运动，执行代次、下发序号与关节姿态保持；[实际确认窗](.evidence/manual-click-20261012/confirmation-fast.jpg)来自同前端204054，[当前工作台](.evidence/manual-click-20261012/workbench-delivered.jpg)来自211606。当前浏览器审阅出现后到期关闭，自动化未捕获该短窗口，因此不伪报新的点击时延。原调姿草稿已恢复。完整检查脚本按成功前段、已修复测试项和后缀覆盖，最终逻辑套件852项、845通过/7明确跳过，另有断言脚本和Node；最终受影响调用链69/69。两个旧测试的墙钟/清理竞态仅修测试，生产预算不变。粗球仍会漏掉部分薄边接触，不是精细网格或连续路径保证；详见[设计与边界](/Users/wujie/Developer/work/anyverse/_context/notes/unified-workbench-refactor-20261009/11-manual-inscribed-self-collision-20261012.md)。

首次常驻部署102524的840次CPU执行（833通过、7跳过）、无期限资源登记8项测试，以及101430的null剩余秒数错误和37历史PID清退证据保留为历史，不冒充当前重复验收。

资源记录 `unified-persistent-20261012-211606` 使用 `--end manual`，45分钟heartbeat请求真实核验；本机离线或调度延迟的空窗如实记录，不补记为已检查。Mac用户级LaunchAgent `work.anyverse.robot-workbench.tunnel` 维护原127.0.0.1:18866→734/43759转发，网络短暂断开后重连；它不启动或恢复机器人任务。启动用 `serve_fake_session.py --persistent` 和明确persistent/null期限合同与lease。Mac/DSW离线仍会影响访问，不把无到期当作宿主故障免疫。首次常驻的[历史回执](.evidence/persistent-fake-20261012/final-receipt.json)、[管理页](.evidence/persistent-fake-20261012/management-ready.png)及[CPU证据](.evidence/persistent-fake-20261012/cpu-final-receipt.json)保留。本次未重做历史冷3/3，也不替代实体L4。

下列均为历史有限时段记录，其中“当前”和截止只对应各自标注时刻；旧011619已到期清退52个历史PID、双GPU及原SSH26602。

2026-10-12 01:31最新核验：当前冻结版本 `dev-20261012-011619` / `session-1-055e0318a5d7`，运行源码与 `ef7a776`一致，C/D就绪、fake连接READY、无自动动作。下述234651及213856均为历史记录。入口和原资源截止不变。

**审批恢复与路径加速已交付**：30秒TTL不变，Owner排队确认过期时返回绑定当次轨迹的“未执行”回执，保留保持，不构造runner或误置硬件FAULT；确认窗倒计时归零后明确提示重新点击执行，不自动续签。完整fake已验证过期零运动及重新执行；最后还修复恒定SDK上限插值产生1 ULP越界，往返实测READY，命令限位未放宽。

支撑检测改为批量SAT与分层bounds；18组实际A1新旧返回完全一致，独立CPU回零检查约下降40%–47%。三次相同目标、近同起点的HTTP准备耗时中位数3.385→2.929s，包含轮询及负载变化，不作P95保证。通用几何源码保持原样，未改标定身份或重写结果。完整CPU832次执行：825通过、7明确跳过，Node通过。[最终回执](.evidence/manual-optim-20261012/final-receipt.json)、[实际过期界面](.evidence/manual-optim-20261012/expired-ready.jpg)、[几何等价对照](.evidence/manual-optim-20261012/geometry-support-only-receipt.json)、[HTTP对照](.evidence/manual-optim-20261012/http-comparison.json)。003955因通用源码指纹变化启动失败、005333往返测试的插值错误均保留；前者已撤回导致身份变化的部分，后者已修复。未重复标定/数采，原头/腕/几何ID与71统计保持。

2026-10-11 23:55：当前冻结候选为 `dev-20261011-234651`（child `session-1-3111ae19812f`），已连接READY、持权标定、无自动动作；数采后台按新实例重新准备，动态状态以[检查点](.evidence/s2/active-task-checkpoint.json)为准。入口仍为[18866](http://127.0.0.1:18866/)，设备栏“开发服务”进入[管理页](http://127.0.0.1:18866/session)。管理页独立于仿真进程，在已授权时段内显式启动、停止和重启；每秒显示阶段、截止、失败原因与清退状态。没有自动重启、连接设备或续跑动作。

人工操作已合并：拖动只看目标图像，点击“执行到目标姿态”或“执行整队”后在后台检查，完成后自动弹出一次本次动作确认。确认窗绑定请求job_id和冻结审批；取消、编辑、换环境、停止、权限变化或过期不执行、不重新弹旧审批。fake夹爪接受0–83mm命令，0mm在仿真机械闭合位置饱和，实测仍保留0.008mm偏移。静止、到位、碰撞与实体SDK策略未改。

196项定向CPU及Node检查通过；中间冻结232236实际完成单姿态、双爪0mm闭合与两点整队执行，取消前后实测不变且零下发；最终234651只再加重连控件缓存修复，浏览器实际断开/连接后12关节输入和草稿恢复。头/腕ID、71个到位样本保持。阶段耗时及原输入落在每个run的 `data/calibration-workbench/diagnostics/manual-preview-<id>.json`；实测单姿态准备3.367s、整队9.920s，主要耗在路径检查，不外推原142s失败。[验收回执](.evidence/manual-preview-20261011/final-receipt.json)、[确认窗](.evidence/manual-preview-20261011/confirmation-zero-jaws.jpg)、[阶段原记录](.evidence/manual-preview-20261011/full-fake-phase-traces.json)。一次30s审批临界过期导致既有FAULT的原始失败保留，未放宽TTL；及时确认的独立测试通过，这种边缘拒绝的恢复体验仍待优化。未重跑标定或数采，也未重新宣称3/3冷验收。

历史管理器 `dev-20261011-213856`、child `session-2-abf2e493bb01`已清退。该版实际完成一次完整fake启动→停止（双GPU C/G核空、页面仍可访问）→再次启动；新实例最初未连接，标定ID/版本、71个到位样本和6视频完整SHA保持，随后相机/target/mesh检查通过。另实际终止自有显示renderer后，通过浏览器恢复成功；旧PID退出、新renderer与目标图成立，Owner绑定及相机源时钟保持。该历史生命周期验收没有重新采集/求解/安装或执行轨迹。

[管理页截图](.evidence/fake-lifecycle-20261011/full-management-ready.jpg)、[停止后仍可访问](.evidence/fake-lifecycle-20261011/full-management-stopped.jpg)、[重启保留检查](.evidence/fake-lifecycle-20261011/restart-preservation.json)、[实际渲染故障恢复](.evidence/fake-lifecycle-20261011/preview-fault/verified.json)可核验。CPU整套按成功前缀和完整后缀接续完成，825次执行818通过/7跳过；旧录制测试wall时序依赖已以确定性fixture修复（aa0ad13），原失败日志保留，生产录制策略/容量未改。[CPU证据](.evidence/fake-lifecycle-20261011/cpu-receipt.json)。部署运行代码与f7e0ae9一致，只有该测试在冻结后变化。

用户明确批准本次734双GPU至 **2026-10-12 01:46:58 CST**，CPU管理器和原18866本机SSH转发至 **01:52** 清退。GPU1计算/GPU0图形；child停止会清空自己的GPU进程，但保留本Session时段预留以便再次启动。45分钟heartbeat真实核验；本机转发另有按PID/UTC启动时间/完整命令的截止关闭守卫。旧193413在14自有PID及双GPU核空后已release，18867保持关闭。动态状态见[检查点](.evidence/s2/active-task-checkpoint.json)。本入口没有新的时段自授权能力，不等于无限常驻。

管理员在实际仿真主机用 `scripts/serve_fake_session.py --help` 核对固定profile入口：冻结contract/base/nominal、独立state目录、派生资源lease、GPU绝对截止与独立CPU截止；UI不能提供命令/路径/GPU/期限。每次child新目录，原数据不删除；历史catalog只读复用，未确认清退不能启动。lease只是已有唯一GPU登记的派生许可，不是另一份使用账本。

### 完整fake复用启动

在持有原run的主机，用 `scripts/calibration_resume_source.py --source-run <原run绝对路径> --source-base <原base-manifest.json绝对路径>` 生成只读SHA规格；将该JSON作为 `serve_physx_device.py --calibration-resume-source <规格JSON>` 的独立参数，配合正常base/nominal/新output和有界collection合同启动。它不是warm诊断模式，正常C/D操作保留；`--dry-run`会验证同虚拟设备、模型、相机、完整文件和成组依赖。不得把来源规格中的plant/truth核验材料传给求解与规划。

完成记录可单独用 `scripts/collection_history_source.py --source-run <原run> --job <完整job路径>` 生成规格，再传入 `--collection-history-source <JSON>`。该入口只导入原生回执核验成功的已完成fake任务，校验SHA/size/原robot-clock-install-execution身份；它是只读历史，不能恢复动作或checkpoint，暂不承担通用失败记录迁移。

导入图像、结果和统计各自独立复制；原版本ID、原采样时钟和原事件身份不改。新采样使用新时钟；精确批次散列支持旧头＋新腕及重新求解。恢复不计空库验收。启动代码与76项定向CPU检查及两组完整CPU配置链见[定向回执](.evidence/s2/dev-resume-targeted-cpu-20261011.json)；实际浏览器切换、D就绪和之后生命周期以检查点的实测时间为准。

反复执行来自三case空库要求，以及此前共享Owner能力、缓存和wall/source就绪差异修复后的产品换版。当前3/3已达到有限退出条件，不再继续追加正常case。文档更新不影响运行源码；后续变更先判断受影响的证据和最小重验范围，不盲目把整套结果归零，也不降低已批准覆盖。

日常开发优先复用有效fake标定与匹配证据；只有要证明无旧标定依赖的场景才使用空库验收。当前完整验收采用30%执行档位，PhysX源时间与墙钟分开；30Hz采样不代表墙钟30FPS。normal-v31仍按当前模型/观测动态生成目标，不导入S0旧q/标定；129/170/90/29条名义预算、开爪v3的45条/1.5s、6条/0.2s保持及原物理门不变。2rad/s、5rad/s²是软件目标上限，不是真机关节实测极限。

## 剩余工作（按优先级）

1. **定点补验已有功能**：完整人工队列、仅腕/分步操作、清空/单批重解、逐相机/逐帧/低质量候选/下载、不同速度实际执行、其他故障后的显式恢复和重新审阅；本轮一种实际renderer故障与浏览器恢复已经通过。C12与预动作恢复CPU已补，不重列待办；实际历史head mesh、50%审阅和30%执行只覆盖各自子范围。本轮已新增上述浏览器证据，其他实际后端分支仍按范围补验，不把页面成功冒充采集/执行资格。
2. **正式真机部署与L4验收**：SDK/支撑与停止、时延/载荷、真实相机及单writer切换独立完成；fake结果不替代现场资格。当前没有待用户裁决的新算法或UI行为决定。

本轮两个业务工作区恢复、封存导出优化和3/3空库验收不再回到实现待办；体验与实际分支资格按证据边界继续补齐。独立纠正数据生产由原任务负责，不并入本工作台范围。

正式安装对外路径仍为`/var/lib/anyverse/calibration/hardware/active/{head.json,wrist.json,manifest.json}`；fake使用隔离身份和安装根。真机支撑与逐批运动审阅门保留。

跨仓权威入口为[统一工作台设计入口](/Users/wujie/Developer/work/anyverse/_context/notes/unified-workbench-refactor-20261009/README.md)；06为运行时/时钟契约，07保留历史与当前证据。维护责任为牵头Session `01a0e689-ddf5-7410-98d5-743920bfa8f7`；实现、来源、run阶段/终态或资源变化时更新，运行快照不作永久状态。
