# 真机工作台部署与二次启动规范

Owner：Robot Workbench 部署维护者；2026-10-29。父入口：[控制链路阶段状态](control-runtime-status.md)。
适用于已有明确使用授权的 Linux X5 台架；目前真实安装对象是 ARX13。机器、SDK、组件版本、
夹爪映射、服务依赖或独占方式改变时重新执行交付检查并封存新清单；历史验收不自动继承。
本规范统一部署流程，不授权运动、故障恢复、修改原数采环境或接管其他任务。

**2026-10-31 03:45 CST 最新部署及交接：** 管理／业务／网页为`source-environment-exit-v9`、
`run-environment-exit-v9`，启动清单`startup/release-environment-exit-20261031-v9.json`；
启动工具仍为`deployment-tools-model-v6r5/workbench_deploy.py`。Owner／模型endpoint／相机版本未改。
已真实点击“一键退出工作台环境”，完成正常0.1 rad/s回零、SDK释放、模型／相机／Triton CPU／
业务／Owner退出，恢复原数采Web；独立管理页保留。成功job：`b9b2db8bc4704aa58c93bcd23c494ff0`。
随后原数采入口只调用一次，任务1251、批次`咖啡打包-20261031_033312-0`已ready，
机器人、三相机与踏板正常，心跳持续；本轮没有录制episode或上传，不替代完整数据落盘验收。
交接时设备由原人工数采持有，工作台不自动接回，模型已退出。后续09:32踏板USB断连导致原数采退出；
10:36只读观测仍为failed，认领有效，未自动恢复。此为交付后的运行事件，恢复前以实时状态为准。
本次启动配置保留退出前选择的Triton，不以历史Torch默认值覆盖用户选择。
验收及失败原件见`.evidence/arx13-exit-collection-20261031-v1/delivery.json`。

**2026-10-31 00:53 CST 历史部署：** 管理／业务／网页为`source-model-startup-v7`、`run-model-startup-v7`；
Owner仍为`source-reclaim-control-v3`、`run-reclaim-control-v3`。新增已登记模型容器
`pi05-arx13-resident-workbench-v6`（18925），endpoint代码为`source-model-startup-v6`。
启动工具为`deployment-tools-model-v6r5/workbench_deploy.py`，清单为`release-model-startup-20261031-v7r5.json`。
一键脚本必须完成模型服务启动与预热并返回`model_ready=true`；infer全退出后单次脚本约58.84秒通过，
Torch/Triton固定输入真实GPU回包均为有限60×14，最终恢复Torch；已有SDK身份与停止保持保留。
日常模型服务采用本机注册的operator准入与原生GPU独占锁；Agent使用仍需独立资源登记。
模型映射未验收时页面Run明确禁用；不会用模型ready代替机器人动作资格。最新入口见[使用说明](WORKBENCH_GUIDE.md)。

**2026-10-30 23:19 CST 历史部署：** Owner／业务当时保持 `source-reclaim-control-v3`、
`run-reclaim-control-v3`，管理／网页为 `source-cold-preview-v4`；启动工具为
`deployment-tools-cold-v4/workbench_deploy.py`，选择器为 `release-cold-preview-20261030-v5.json`。
五服务（含独立 Triton CPU）全部退出后单次脚本冷启动约4.02秒通过；其后仅更新网页加载竞态，
没有重复整套冷启动。当时SDK按启动契约保持断开，三相机／目标预览／人工规划正常。
日常操作、二次开发入口与证据以 [工作台使用说明](WORKBENCH_GUIDE.md) 为统一入口。

**2026-10-30 22:05 CST 历史部署：** Owner／业务／管理当时为 `source-reclaim-control-v3`、
`run-reclaim-control-v3`；启动器为 `deployment-tools-reclaim-v3/workbench_deploy.py`，
选择器绑定 `release-reclaim-control-20261030-v3.json`。已验收“清理现有控制并接入”、
幂等启动和四栏目共用人工控制约1秒后Stop。当时SDK连接、健康静止、停止保持，三相机正常；
原数采Web已正常暂停。踏板3绑定KEY_C/46，实际踩踏Stop待用户补验。
日常操作、范围和最新证据统一见 [工作台使用说明](WORKBENCH_GUIDE.md)。
以下旧发布及计数保留为历史记录，不代表当前设备状态。

2026-10-30较早阶段：当时Owner/业务/管理使用`source-dm-equivalent-v1`与
`run-dm-B0-final-v1`；相机仍为原独立组件。已通过DM等价单位真实Stop/回退及七轮完整
1.5×研究，未晋级D/FF参数；临时2°起点准入已撤掉，默认运动仍base-v1。
当时启动器调用`deployment-tools-dm-v1/workbench_deploy.py`，`startup/current-release.json`
选择`release-dm-equivalent-20261029-v1.json`（995项静态输入）；旧清单、旧启动器及SDK保留。
交付时SDK已正常释放，工作台相机正常停用、原数采网页恢复且idle；下次研究按下方
`--handoff-collection`显式交接。详见控制状态最新节及
`.evidence/arx13-inner-loop-20261029-v1/`。下文首次标准化测试数字是历史证据，不覆盖本次版本。

## 交付标准

首次部署完成必须同时交付：隔离运行环境、机器绑定、完整 UI/规划/相机模块、固定的服务配置、
SDK CPU 预检、真实控制验收记录、版本化启动清单、一键脚本、回滚指向和未通过项目。
“网页可访问”“SDK 能 import”“单测通过”均不等于这份交付已完整。

二次启动恢复已封存服务，顺序为 **Owner → 相机 → 已配置的模型endpoint → 业务后端 → Triton CPU → 管理入口**，
然后通过同一运维操作加载／预热已选模型，等待模型与业务绑定就绪。
启动 Owner 只提供控制接口；**不连接 SDK、不运动、不清锁、不重放审批或任务**。
已登记模型会同步启动与预热；健康已加载模型直接复用，不在每次启动时重复加载。
健康已运行服务复用；异常服务保留原状态并报告，不通过循环 restart 或 kill 掩盖故障。

## SDK 更新与模型目录迁移

SDK 更新后，在发布前用 Owner 实际配置执行统一预检，并将同一命令配置到 Owner 的 `ExecStartPre`。
`--profile` 必须与 Owner 的 `ExecStart --profile` 指向同一配置，发布封存工具会核对该绑定。

```sh
python3 scripts/check_sdk_solver.py --profile '<实际 Owner 配置>' --output '<预检回执>'
```

预检加载所选原生 SDK，复用接入链路的模型内容校验，并构造名义／双臂标定模型的纯 CPU solver，
检查 FK 和重力结果有限。检查期间不构造硬件 controller。模型目录可迁移，模型内容、机型、
重力向量和标定来源仍须一致。回归覆盖同内容迁移通过、内容变更拒绝，以及预检误用旧配置拒绝。

维护触发为 SDK、模型目录、标定绑定或 Owner 配置更新。该预检在发布和 Owner 启动时执行。

## ARX13 已安装的一键入口

在 ARX13 终端执行：

```sh
/home/wujie/work/feiyu.zhou/arx13-control-20261029-v1/start-workbench.sh
```

从 Mac 执行同一个入口：

```sh
ssh -i ~/.ssh/id_ed25519_new_server -o IdentitiesOnly=yes -o BatchMode=yes \
  wujie@10.17.32.3 \
  /home/wujie/work/feiyu.zhou/arx13-control-20261029-v1/start-workbench.sh
```

启动完成访问 <http://10.17.32.3:18868/?environment=physical&business=research>。
原数采网页在开机时可能自动启动。确认其采集已结束、操作人员停止使用后，显式交接：

```sh
/home/wujie/work/feiyu.zhou/arx13-control-20261029-v1/start-workbench.sh --handoff-collection
```

该标志授权正常结束**清单中的**数采及其网页：先GET确认机器与会话，活动采集通过原
`/api/collect/stop`停止（可能回零），等待`manager_alive=false`、`collect_status=idle/stopped`
及适用的`safe_ok=true`，再暂停网页并核实cgroup子进程退出。状态未知、孤儿／遥操作仍活动、
停止未确认、会话被替换或工作台控机连接／故障未处理时拒绝。数据保存结果沿用数采自己的回执。
原数采没有共同的原子设备锁，HTTP 空闲检查不能排除另一操作者同时发起采集；因此这个选项
需要操作者先结束数采操作。本版没有修改原数采代码、Conda、SDK、CAN、udev或开机策略。

只检查而不启动：

```sh
/home/wujie/work/feiyu.zhou/arx13-control-20261029-v1/start-workbench.sh --check
```

输出区分 `integrity_passed`、`startup_ready`、`services_ready` 和 `control`，不能把文件完整性
通过当作机器人已可执行。`check` 仍创建自己的互斥锁文件，不改服务/设备。
退出码：0=请求的检查/服务启动完成（check仍须读取startup_ready，start读取control_ready）；
2=前置、配置或启动失败。服务已起但控制故障时明确报告`control_ready=false`，不自动清锁或重试。
每次 start 在 `startup/` 保存唯一 intent 和结果，部分成功也保留；文件中不保存 token。

机器人恢复使用仍走页面“接入/检查”和控制权交接。重复运行脚本不会更换现有 SDK 身份。
结束研究后，在独立运维页点击“一键退出工作台环境”并确认：正常Stop→健康静止→正常规划回零
（0.1 rad/s、保留夹爪开度）→正常SDK释放→模型／相机／Triton CPU／业务／Owner退出→
核验全部资源停止→恢复原`prometheus-arx13-web.service`。管理页保留，不自动接入。
任一步失败就停在该阶段，不强杀、不恢复数采、不盲目重放。退出成功后业务心跳不可读属于预期，
以成功退出回执和资源状态为准。按钮不会调用`/api/collect/start`。
操作者再在ARX13本机`http://127.0.0.1:18980/`点击“启动硬件并准备采集”，见ready与三路LIVE后
按原流程示教；原数采仅监听loopback，不能把工作台18868当数采入口。
本脚本不提供一键强制停止或清锁；不能用它绕过维护前置。

## 本次困难、根因和防复发措施

| 实际问题 | 根因/证据 | 标准化处理 |
|---|---|---|
| arx6 配置不能直接运行在 arx13 | CAN 名称、相机序列号、夹爪端点不同 | 机器清单逐项读取；左右角色、物理单位及绑定哈希在 Owner/业务配置间一致 |
| Python 3.12 数采 SDK 与研究 Python 3.11 不兼容 | ABI、依赖和部署路径不同 | 研究 Python/SDK/依赖独立；不安装进原 Conda，不以改系统库解决问题 |
| `os.memfd_create`/封印常量缺失 | Conda Python 构建暴露能力不同 | 保留已实现 libc memfd 封印回退；目标 Linux CPU 检查，不只在 Mac 测试 |
| SDK import 成功但接入报 `AMENT_PREFIX_PATH` | Solver 构造才加载 ament 索引和 URDF parser 插件 | Owner 的 ExecStartPre 构造纯 CPU solver、验证 FK/重力；固定 AMENT 和动态库路径 |
| 接入时左夹爪 80.271 mm 被拒绝 | 指令上限误用于真实反馈及 Stop 保持 | 左命令0–80、反馈−3–83；右命令0–89、反馈−3–92；保留原始反馈，保持指令独立投影 |
| 关节初始小越界阻塞规划 | SDK 限位、URDF 名义限位和指令噪声容差混用 | 显示/规划用机器 SDK 限位；起点观测±2°余量；指令不随之扩限 |
| DM 速度解释可能不一致 | 设备寄存器和编码范围需要现场证明 | 按序列号及CAN角色绑定只读 PMAX/VMAX/TMAX 回执；不能套旧机回执 |
| Owner 正常退出后 token 妨碍重启 | 控制连接与进程/凭据生命周期混杂 | 保留已修正常关闭清理；二次启动只归档**前一次 boot**遗留文件，同boot异常不自动清理 |
| 部分接入失败后维护票据/旧Stop妨碍退出 | 半初始化 Runtime 与维护 actor 状态未同步 | 保留已修复的 adopt/退出证明；SDK存活时禁止强退Owner；发布只从已验证释放态进行 |
| 目标浏览、人工规划漏装 | 初始部署只配 SDK/研究入口/相机，没有完整能力清单 | seal 要求 calibration_config + target_model_manifest；启动验收12轴栏、模型和三相机 |
| UI夹爪83mm写死、离线负角度草稿失败 | 机型适配停在控制端，显示/模型/碰撞检查未同步 | 绑定统一夹爪范围与SDK限位，重绑几何指纹；离线和在线两种状态都验收 |
| V8/V10、研究配置和标定动作的P/D混淆 | 执行曲线profile兼任增益选择，业务配置未覆盖普通动作 | 默认运动gain profile显式normal_X5；记录冻结增益/原生输出/Stop保持三者，不用草稿推断实态 |
| 0.8/2/5 rad/s各业务策略混读 | 顶层速度观察服从持权业务，保存策略按栏目区分 | 启动不改参数；动作前记录当前holder及Owner的settings_snapshot；封存历史策略不强制覆盖用户新设置 |
| 真Stop成功，脚本却报失败 | 测试硬等`FAULT`，实际枚举为`FAULT_STOP_UNCONFIRMED` | 检查Stop返回、连接、反馈、撤销未来队列及显式恢复；状态名不单独代表物理停止结论 |
| 回零完成但±0.3°助手退出 | 功能完成与最终静态精度验收混用 | 回零轨迹完成、静止和精度分列；不能自动放宽阈值，也不能把未达精度写成硬件故障 |
| 记录器启动失败 | 子进程未继承项目PYTHONPATH | 固定解释器/env/工作目录，依赖检查在运动前；失败身份独立保存，未知结果不重放 |
| localhost访问拒绝 | 管理器明确绑定LAN IP，误用127.0.0.1 | 启动清单保存准确URL；业务loopback与对用户LAN入口分开检查 |
| 服务进程active但IPC尚未出现 | systemd Type=simple 的active不等于应用ready | 真实systemd冷启动测试已抓到并修复：等本实例Owner descriptor/IPC和相机descriptor，再启动依赖者 |
| 初始发布记录落后于后续source-v8 | 多轮临时脚本、组件独立更新但缺统一封存 | 按实际unit/manager读取版本，哈希封存各组件；允许Owner/相机/UI版本不同但全部明确绑定 |
| 数采网页与工作台可能同时控机/占相机 | 原环境没有共同锁，且开机自动启动 | 默认只报告冲突；显式空闲交接，不碰其他任务、CAN、数采数据；退出后按原配置归还 |
| 高频振动/到位误差仍存在 | 参数、单位契约、硬件内环与负载尚需辨识 | 属于控制研究未完成项；部署标准化不宣称消除振动，启动不自动做研究轨迹 |

证据：`.evidence/arx13-deployment-20261029-v1/`、`arx13-commissioning-20261029-v1/`、
`arx13-default-pd-20261029-v1/`、`arx13-base-v1-20261029/`，及本次
`.evidence/arx13-deployment-standardization-20261029-v1/`。失败原件不覆盖。
部署中遇到的CAN拥塞、主机调度/文件系统阻塞等历史控制问题，沿已有诊断证据处理，
不可凭本轮启动检查将它们归因成部署缺陷或宣布解决。

## 首次部署标准顺序与停止门

| 阶段 | 必须产出/检查 | 未通过时 |
|---|---|---|
| 1. 只读盘点 | 主机、账号、端口、systemd/进程、原数采状态、设备角色、保护文件哈希 | 不停止归属未知进程，不直接复用旧IP/配置 |
| 2. 冻结输入 | 独立任务根；代码快照及SHA、Python ABI、SDK build/lib哈希、实际设备清单 | 缺少来源/绑定就不启动硬件 |
| 3. CPU运行能力 | 固定env；SDK导入+solver构造；memfd；单位/夹爪边界；依赖单测 | 保持惰性，不以循环接SDK找依赖缺失 |
| 4. 完整服务装配 | Owner、相机、业务、管理、名义模型、人工校验；服务可写范围仅任务根 | 不把缺显示/规划称为部署完成 |
| 5. 离线UI与观测 | SDK断开时12轴滑杆/目标预览；三路实际图像；草稿不触发控制 | 前后核对SDK身份/epoch，保持操作者输入 |
| 6. 受控实机资格 | 接入→反馈→低速Stop→未来撤销→显式恢复→释放→再接入；独立TX/RX | 原生/通信异常停止，不自动重试；单独记录未验收精度 |
| 7. 隔离与交付 | 原环境哈希复核；数采网页恢复；当前实际版本/状态收据；seal+一键入口 | 不凭Git HEAD或历史成功宣布交付 |
| 8. 后续变更 | 先候选→CPU→按维护门发布→更新收据→重新seal | 旧清单发现漂移即拒绝启动，不静默改成新版本 |

第一轮在新机器上部署仍需明确提供/核验机器绑定，脚本不是自动猜测机型的万能安装器。
`device_bootstrap.read_profile` 当前对ARX13和原设备有明确CAN角色约束；新增设备必须显式适配、
测试并验收。名义URDF可用于显示/粗自碰撞，不等于这台机的实测外参、关节零偏或负载辨识。

## 清单封存与升级

工具源：`scripts/workbench_deploy.py`，纯标准库编排，复用 `ManagedService`、`Deployment`、
只读 `ControlChannel`。不新增包管理器或第二套控机接口。将工具复制到目标机器任务专属目录后：

```sh
"$TASK_ROOT/env/bin/python" -B "$TASK_ROOT/deployment-tools-v1/workbench_deploy.py" seal \
  --root "$TASK_ROOT" \
  --manager-profile "$TASK_ROOT/run-base-v1-preset/manager.json" \
  --manager-unit robot-workbench-arx13-manager.service \
  --url http://10.17.32.3:18868 \
  --output "$TASK_ROOT/startup/release-v4.json" \
  --collection-unit prometheus-arx13-web.service \
  --collection-status-url http://127.0.0.1:18980/api/status \
  --triton-unit robot-workbench-arx13-triton-control.service
```

`TASK_ROOT=/home/wujie/work/feiyu.zhou/arx13-control-20261029-v1`；首次使用时明确赋值。
上面目录名是早期示例；实际升级应采用新的工具／run／清单路径。已配置`triton_preview`时，
必须显式绑定其CPU unit并核验脚本、端口和凭据路径；不启动GPU模型或转换任务。
`seal`只创建新清单，不改unit、不启动、不伪造实机验收。检查机器ID、双方配置、独占服务、
隔离unit、入口脚本、CPU prestart和显示/规划配置；固定代码/配置/SDK/ament/几何哈希及unit/drop-in。
solver-only URDF仅需运动学/惯量，目标显示/碰撞URDF的所有mesh必须完整，二者资格分开。
模型服务须在manager.services.model登记为受限`docker_model`，同时配置model_port与business.inference；
封存时检查端口、认证文件路径一致，原生制品在加载时继续校验。启动等待管理器观察到**本次模型PID身份及业务实例**，
再提交唯一预热操作；不靠固定sleep或失败后盲目重试。服务进程、模型warm、设备映射和实际动作分别验收。
运行descriptor、凭据和可变研究历史不封存为静态版本文件；不将凭据写进回执。

当前安装的 `start-workbench.sh` 来自 `scripts/start_workbench.sh`，读取
`startup/current-release.json`；最新选择器及工具路径见本文开头的版本更新，权限0600。
新版本须用新清单路径，旧清单保留；manager unit身份回执按release清单独立保存，不能跨版本复用。
校验完成后，以0600临时文件加原子rename将新清单内容
发布到 `current-release.json`；如工具版本变更，同步更新脚本路径并重新seal。
这是显式发布动作；不按目录时间或`source-v*`最大编号自动切换。新清单的手工交接服务drop-in
变化也必须重新核验，不能把“原数采文件有变化”自动解释为本任务可以覆盖它。
清单只校验，不替代可信来源和操作授权；同用户主动改写所有清单与程序不在其防护模型内。

## 失败处理、回滚与验证边界

- 启动前完整性、unit身份、CAN存在/up、服务启动磁盘空间和端口冲突失败：零服务变更。
- 同时运行两次启动：flock只允许一个执行。另一调用退出，不追加启动请求。
- 部分启动成功后超时：保留已启动进程及回执，报告失败；不自动回滚，因为期间可能已有人工接入。
- 前一boot的Owner socket/token：确认unit无进程、私有文件和旧boot身份后改名归档；不删除证据。
- 同boot Owner异常、PID/归属不明、维护仍在执行：停止；从页面维护和原始日志定位，禁止强删锁。
- 活跃进程不在预期script/uid/cgroup、unit配置漂移：拒绝重用/操作，不kill、不重命名CAN。
- 版本回滚仍须核对控制状态；Owner更换遵守先正常回零/释放。显示独立回滚不应牵连SDK。
- 真实无控机systemd冷启动夹具和重复调用证明服务编排；不冒充真机器人断电重启验收。
- 生产现场的故障锁和原数采占用必须如实报告，不能为让一键脚本返回0而代替用户清锁。

本次验证命令：

```sh
PYTHONPATH=. .venv/bin/python -m unittest tests.test_workbench_deploy
# 仅目标Linux，临时unit名称唯一，无真实SDK/相机：
python tests/check_workbench_startup_systemd.py \
  --root <新的任务内临时目录> --source <已安装源码根> --tool <workbench_deploy.py路径>
```

本次12项Linux测试全部通过（Mac上前一boot真实/proc用例跳过，已在Linux补验）。
真实systemd夹具的 `cold-test-02` 完成全停→按序启动→重复启动，四个PID/身份不变，
临时unit全部正常退出并移除；这不是机器人断电测试。当前封存1442个静态文件。
生产入口的 `--check` 完整性通过，显示/人工规划/三相机就绪，但 Owner 已退出且遗留同boot文件，
`startup_ready=false`。本轮普通启动和显式交接选项均拒绝继续，前后生产unit、PID和控制状态未改。
当前故障恢复不属于此次部署标准化的自动动作，仍留给控机恢复流程。

测试覆盖冷启动顺序、重复启动身份保持、故障保留、忙态拒绝、数采交接门、文件漂移、
前一boot残留归档、并发互斥、部分失败不自动回滚。真实主机夹具只停/删除自己创建的临时unit，
保留其日志与测试结果。真实机器人电源重启留到下一次自然开机时验收，不为部署规范化中断现场工作。
