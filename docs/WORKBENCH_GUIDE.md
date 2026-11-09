# Robot Workbench 使用与部署说明

维护者：工作台控制／部署任务（W2，Session `01a16327-7545-7332-a361-0123ddad811d`）。
最近发布与验收：2026-11-09，ARX6；ARX13 的 11-08 记录保留历史时点。机器、发布版本、接口、启动方式或验收结果变化时更新。
当前身份与待办以 [当前进度](../.evidence/infer-pedal2-start-20261107-v1/progress.json) 及其中的
current_runtime/latest_resource 为准；开发工作树和历史成功记录不能代替线上状态核验。

## ARX6 桌面启动（2026-11-09）

同日补修“准备控制与踏板”：首次绑定时规划速度 1 rad/s 与旧实测保护默认 0.8 rad/s 冲突，
旧实现先提交配置、后在控制 tick 校验，导致 Owner 故障及后续旧 epoch 观测报错。
现在绑定与旧设置入口都在提交前原子校验完整 ControlPolicy；未绑定的物理规划默认值
按当前实测保护范围约束，已有显式规划策略继续按原值核验。
用户随后将共享实测速度保护默认值指定为 **2.5 rad/s**，唯一来源为
`execution_settings.DEFAULT_MEASURED_SPEED_LIMIT_RAD_S`，网页读取实际 Owner 值。
ARX6 已实测准备 HOLD、完整三视频/状态/控制/踏板录制，模型请求为 0；结束后 1.20 秒
解除忙态，后台封存 outcome=complete、无缺失或错误，SDK 保留健康静止保持。
回执见 [准备控制修复进度](../.evidence/arx6-infer-prepare-20261109-v1/progress.json)。

用户于 2026-11-09 明确将 ARX6 桌面入口改为自动接管 CAN：启动脚本即确认现场可以接入，
不再弹出第二次确认。配置 `connect_control: true` 时，脚本复用运维 `reclaim_control`：
正常结束已登记的旧控制／数采，核验退出，接入工作台并保持当前位置，不续跑旧模型轨迹。
已有健康静止的工作台连接直接复用；未开启该配置的其他消费者仍只启动软件。

网页打开与服务准备分开：桌面脚本先运行 `--open-page-only`，启动配置中的 `ui_unit`，
管理网页可访问后立即调用系统浏览器；该步骤在准备锁之前，目标为 5 秒内打开网页。
随后继续正常接管、输入准备及模型加载，页面显示真实准备进度；全部就绪后启动脚本才返回成功。
`--no-browser` 仅省略打开页面，仍执行配置中的正常接管与完整服务准备。

源码、配置与 unit/drop-in 文件摘要只保留为首次登记的部署证据，运行期间不反复扫描，
也不因文件版本更新阻塞启动或运维。模型与 SDK 制品首次加载的身份校验保留；页面版本
与实际管理器握手、服务进程/cgroup、控制归属、退出结果、实时反馈及协议核验继续执行。

本次同时修复两个启动问题：来源登记服务恢复后，旧 ROS 消费者仍需通过既有重绑操作恢复；
原数采的 `safe_ok` 是单条采集复位结果，未完成 episode 时可以为空。为空时必须由完整的
v1 组件终态、无错误和真实进程退出共同核验交接。另允许已明确在 episode 前失败的启动：
robot 已停止，其他组件仅 waiting/stopped/disabled，manager/orphan/teleop 均已退出；
正常停 Web 后仍核对整个 cgroup 退出，原 failed/error 保留。其余失败、未知组件、孤儿进程
及退出不明继续拒绝，不强杀。SDK 重力标定源路径已随实际 SDK 迁移，模型内容与标定偏移不变；
原生求解器前检同时核验该绑定。桌面 GPU 进程重启后，通过已登记可执行文件、同用户的
桌面 session cgroup 和实际 GPU 子进程身份核验，不再依赖旧 PID。

踏板消费者与录制子进程使用部署声明的 ROS 网络范围，不强制改为 localhost-only。
启动成功条件加入实际踏板心跳。候选通过 148 项启动／交接检查与 94 项踏板／录制检查
（含实际隔离 ROS 进程），实机发布与接管回执见
[本轮进度](../.evidence/arx6-startup-rebind-20261109-v1/progress.json)。

实机通过正常回零、完整录制、SDK 释放与原数采 Web 恢复；原失败录制封存保留。
最终冷启动回执为同目录 `delivery-acceptance.json`：网页 **0.317 秒**打开，当时模型尚未
就绪，全部服务 **70.929 秒**就绪；重复启动 **0.349 秒**且 SDK/模型身份不变。
前置是七个登记服务全部停止、18868
不可访问。共享硬件相机发布器为既有外部依赖，保留其归属；其工作台登记服务参与冷启动。
桌面 `.desktop` 原入口自动打开 Chrome，以启动输出、实际 API 和窗口标题核验；浏览器
自动化不可用，未将这些证据描述为截图或按钮实际点击验收。

下方是 2026-11-08 软件部署阶段的验收；其中“只启服务、网页接入”的选择已被上述新指令替代。

本轮目标是在 ARX6 桌面提供一个入口，启动已登记的 Owner 接口、ROS 输入与踏板、
常驻记录器、模型目录服务、业务后端和管理页。启动完成必须核验真实相机帧、目标资产、
人工规划能力以及所选模型的加载／预热，不能仅以 HTTP 可达作为成功。
冷启动、重复启动、模型独立回包、完整录制与原数采网页切换分别保留实际回执；候选准备不代表这些验收已通过。
当前进度与安装身份见 [ARX6 本轮验收记录](../.evidence/arx6-one-click-20261108-v1/progress.json)。

入口为 <http://10.17.32.5:18868/?environment=physical&business=inference>，ARX6 桌面入口为
`~/桌面/真机工作台.desktop`，脚本位于 `/home/wujie/work/feiyu.zhou/arx6-one-click-20261108-v1/start-workbench.sh`。
本轮已验证七个登记服务全部退出后的冷启动，约 65.6 秒返回所选模型 ready；重复启动约
0.09 秒且进程身份不变；CUDA 库隔离后的复验约 72.7 秒。原共享相机发布器继续运行，
本轮没有停用它。E03/E04_2 的 SO/Torch 四个模型包均通过单模块 GPU 回包，输出为有限的
32×14；coffee999/Torch、mixed366/Torch 和 mixed366/Triton 的输出为有限的 60×14。
目录共 43 个配置。三路实图、目标资产及人工规划组件已核验，目标二进制长度／摘要与
12 轴描述一致；人工路径校验实际返回成功，未下发控制。

用户已确认 ARX6 交接，并选择**脚本只启服务，保留网页接入 SDK**。脚本会通过原正常接口
暂停空闲的数采 Web，等待其真实退出；活动数采会在启动工作台服务前被拒绝，不自动停机。
随后启动全部登记服务，等待真实相机与模型就绪再打开网页。重复／并发启动使用同一入口；
原生文件锁串行化启动，已运行服务复用。用户在网页确认现场条件并接入 SDK 后再执行 Infer。
SDK 断开时的数采切换已验证能从原数采 Web 完全停止的状态恢复它，返回正确网页地址；
后台没有开始采集或改变任务认领。本次软件部署验收没有接入 SDK 或执行机器人动作。
最终使用真实 `.desktop` 入口从七服务全停状态启动，约 **72.1 秒**达到模型就绪并打开 Chrome，
空闲数采 Web 正常退出，共享相机实例未改变。标准检查记录 2,395 次测试执行，其中 2,326 通过、
69 项因外部环境条件跳过；另行执行的 54 项原生 ROS 检查与最终 73 项启动／交接检查通过。
新环境中的四条 WAM 与三条 π 后端均通过真实 GPU 模型回包；未把这些单模块结果当作实机动作资格。
已通过桌面系统打开两个网页并核对 Chrome 窗口标题；浏览器自动化被 Work 设置禁用，
未把接口与前端用例通过写成真实按钮点击或视觉截图验收。

ARX6 复用原独立相机发布器，工作台只登记其原始 ROS 图像来源；退出工作台不停止这个
共享硬件发布器。正式数采任务、数据及同事的推理面板保持各自归属。
“切回数采”沿既有正常回零／释放流程退出工作台，核验原数采 Web 就绪后自动跳转到配置的网页；
它不自动开始数采、录制 episode 或上传。SDK 接入仍必须使用真实的现场确认与正常运维接口。

## 当前 Infer 使用方式（2026-11-08）

入口为 <http://10.17.32.3:18868/?environment=physical&business=inference>。更新后刷新页面。
本轮发布后的 SDK 已连接并保持；使用前继续核对页面的实际反馈、持权者和故障状态。

1. 选择模型家族、后端和 checkpoint，使用既有“准备模型”操作。当前 ARX13 已登记
   E03/E04_2 的 SO、Torch 及各自去噪 1–10，共 40 个配置。目录每 5 秒扫描，也可手动刷新。
   包目录为 `/home/wujie/work/feiyu.zhou/wam-arx13-20261107-v1/model-catalog`；新增同族包使用
   `<checkpoint>/<backend>/manifest.json`，完整资产校验在加载前执行。
2. 设置执行步数、运行时长、seed 和可选 WAM trick，网页“准备控制与踏板”一次。
   准备成功只进入当前位置 HOLD 并记录。HOLD 中双踩踏板 1 才开始 Infer，或使用页面 Run。
3. 已准备的 HOLD 中仍可修改执行步数和去噪并点击应用；运行时长输入完成后自动应用。
   系统关闭旧 Run 的取流、应用参数并准备新 HOLD，SDK 保持连接。Stop 撤销未完成的重新准备。
   seed 和 trick 在该会话内冻结；MODEL、TEACH 或回零期间不应用新参数。
4. “结束控制会话并封存”先停止控制、确认尾部数据落盘和取流关闭，然后允许下一轮操作。
   原录像的校验和回放导出继续在后台进行。三次实机 HOLD 用例的解锁时间为 1.49–1.51 秒，
   这是本轮测量，不是所有负载下的上限。后台记录仍显示原 Run 身份及错误；停记录服务仍须
   等全部 writer 封存并退出。

踏板 1 单踩停止；MODEL 中踏板 1/2 停止、3 无效。双踩资格按第一脚时状态固定，不能在
同一双踩中停止后又重新启动。TEACH 先通过踏板 1 或 2 回到 HOLD，再用独立双踩 1 启动。
HOLD 中踏板 2 进入重力补偿拖动，再踩 2 保持当前位置；踏板 3 使用原生回零。
来源 250 ms 去抖不变，双踩窗口 0.5 秒；最新双踩规则的实体验收按用户要求留待方便时完成。

WAM trick 默认关闭，准备前勾选。暂按夹爪标定全行程的 5% 作为静止跨度，当前左/右为
4.0/4.45 mm；关节跨度为 0.02 rad，窗口为 30 Hz 下 90 个唯一已执行模型步。
停滞时循环两个顶部黑屏请求和一个真实画面请求，逐请求随机 seed，运动后恢复。
原始图像、实际模型输入与 seed 均留存；此单位定义是本项目假设，实机脱困成功率尚未测量。

v28 的 [封存与参数验收](../.evidence/infer-pedal2-start-20261107-v1/recording-lifecycle-acceptance.json)
覆盖 32→15 步、5→7 秒、去噪 2→1→2，以及三次完整录制；Run 内无 MODEL 请求。
最终恢复 E04_2/Torch、去噪 2、执行 32 步。当前 Owner 使用 source-ros-clock-retirement-v37，
业务与页面源码使用 source-source-health-v38（运行 unit 为 pedal-rebind-v43），记录器源码
使用 source-capture-ready-v41；当前管理配置为 manager-pedal-rebind-v43.json，
模型服务仍为 wam-catalog-model-v4.service。
v32 修复了业务侧等待封存超过 90 秒就误报结束的问题，后台追踪现在持续到记录服务给出
真实终态；71 项 CPU 录制／ROS检查通过，正常回零重接后策略与原热模型核对一致。
详见 [封存追踪修复](../.evidence/infer-pedal2-start-20261107-v1/background-seal-acceptance.json)。

v34 解除关闭的 ROS 节点、旧推理绑定和录制状态对象的循环引用。关闭循环 GC 的原生 CPU
用例中，8 个关闭节点、一个完整 Run 后的旧绑定/录制对象及随后 16 次配置退役均无残留；
这不等于全天 RSS 已有界。扩展组合 152 项中 151 项通过，另一次踏板进入 MODEL 前等待
新观测 500 ms 超时，模型调用为 0 并保持；该完整用例独立连续 3 次通过，但原因仍待查，
没有放宽门槛。见 [退役与回归证据](../.evidence/infer-pedal2-start-20261107-v1/ros-retirement-acceptance.json)。

v37 修复了以较旧的控制 RPC 投影时间判断 ROS state 新鲜度的问题：真机使用当前同机单调
时钟，fake 使用原 ROS 源时钟；原始采样时间与 100 ms 门槛不变。确定性边界与 RPC 时间
落后 250 ms 的完整 WAM 用例通过，尚不能证明原间歇 500 ms 观测超时全部由此引起。
销毁订阅时也清除残留回调，外部保留已销毁句柄不再保留旧节点。
组合检查曾出现跨业务尾部屏障失败；后续十次通过不关闭该间歇问题。
见 [时钟与退役检查](../.evidence/infer-pedal2-start-20261107-v1/ros-feedback-clock-acceptance.json)。

v38 修复踏板断开后仍显示 ready，以及单个来源尾部缺失时误报所有相机缺失的问题。
踏板来源退出或心跳超时会显示不可用；录制保留真实缺口并指出具体来源，分别判断相机
端点。原失败 Run 的唯一未达端点是踏板，原件保留。29 项 Linux CPU 检查通过；踏板设备
恢复后正常重启来源并重新绑定，ARX13 HOLD 复验 1.528 秒解锁、三视频完整封存、0 模型
调用，SDK 与模型 worker 均未重接。详见 [来源健康验收](../.evidence/infer-pedal2-start-20261107-v1/source-health-acceptance.json)。

v39 修复跨 Run 的控制序列屏障：唯一 Owner 使用全局事件序号，旧 Run 的后台封存通知
也会占用序号。记录器现保留绑定窗口内的全部原始事件及各自 Run ID，不再过滤出虚假
序列缺口；模型输入／结果仍按 Run 绑定。控制流继续按最终序号闭合，不改成时间戳截止。
完整 CPU 数采用例已稳定复现修复前的 control 屏障超时，并验证修复后完整录制与旧 Run
身份保留；实机 HOLD 1.573 秒解锁、三视频完整、0 模型调用，SDK／业务／模型保留。
扩展组合 30 项中 29 项通过，另一次在进入测试尾部前触发独立的 1 秒录制准备超时；
该问题未关闭。见 [全局控制水位验收](../.evidence/infer-pedal2-start-20261107-v1/control-watermark-acceptance.json)。

v40 的只读时序诊断将线上观测间歇阻塞定位到大图 ROS 交付：原始 state 到达约 4 ms，
相机发布调用约 1.6 ms，但图像到订阅端最慢约 278 ms，合格观测间隔达到 561 ms。
相机发布进程原 Fast DDS 共享内存段仅约 512 KiB，单张 RGB 像素为 900 KiB。
隔离 A/B 的 16 MiB 段配置把三路图像最大交付延迟从约 13 ms 降到 2.8 ms。
当前只对本次相机发布服务配置 `FASTRTPS_DEFAULT_PROFILES_FILE`，文件为部署根目录的
`config/dds-image-shm16.xml`，已纳入管理服务 publication_files 摘要；未修改系统网络设置。
相机正常重启并重新绑定后，60 秒只读检查收到 2,665 个合格观测，最大发布间隔 74 ms、
配对反馈最大年龄 84 ms，无 G03 过期；三路图像交付最大 8.9 ms。100 ms 新鲜度与
500 ms 供给门槛不变。HOLD 复验 1.401 秒解锁、完整封存、0 模型调用，SDK 保留。
这是已捕获传输问题的修复与有限时长资格，不代替后续完整 MODEL 长时/精度验收，也不
把历史未带诊断的 CPU 超时全部归为同一原因。见 [图像传输验收](../.evidence/infer-pedal2-start-20261107-v1/image-transport-acceptance.json)。

v41 去掉录制来源首次就绪后等待下一次 5 Hz 心跳的额外延迟。每个原始来源的首个有效
样本只唤醒已有健康发布线程，就绪判断仍核验全部来源；DDS 接收回调不负责序列化或
落盘，周期心跳与原 1 秒准备预算保持不变。延后周期心跳的完整标定用例在修复前超时，
修复后通过；录制和四业务组合回归通过。六次正常准备测量为 505–937 ms，不作为任何
负载下的上限。实机 HOLD 1.481 秒解锁、完整封存、0 模型调用；SDK、相机和模型保留。
参见 [就绪通知验收](../.evidence/infer-pedal2-start-20261107-v1/capture-ready-acceptance.json)。

2026-11-08 已在用户授权的模型独占窗口完成 E04_2 step002902／2 步去噪的 SO 与 Torch
单模块对照，未执行机械臂轨迹。三个真实录制请求的原始图像与有效 BF16 state 重建校验
通过，权重、统计量、文本缓存和记录的七项输入张量摘要一致；两种后端各自重复均逐位
一致。两者之间仍有稳定差异：最大关节输出差 0.978°、最大夹爪坐标差 0.568 mm，
有效维度中 987/1344 个值完全相同，差异最大为 8 个 BF16 ULP；具体层／算子尚未定位，
不宣称数值等价或任务成功率等价。

排除每后端 3 次预热，各测 30 次：Torch／SO 模型本体 P50 为 198.0／133.6 ms，
P95 为 200.3／134.6 ms；单模块 HTTP 请求 P50 为 215.7／152.2 ms，P95 为
219.8／156.5 ms。模型加载和整机执行另计，测量时隔离 CPU 录制任务已退出。
原 Torch 配置与设备观测绑定已恢复，SDK nonce/控制 epoch 未变。踏板来源于 16:24
另因 ENODEV 退出；模型测试期间保持其不可用事实。后续确认同一设备重新枚举、无其他
控制程序读取后，正常重启自有来源并重载业务绑定，当前已恢复 ready；未重接 SDK，
相机与热模型进程保留。实体踩踏仍按用户要求暂缓。
详见 [模型延迟与精度对照](../.evidence/infer-pedal2-start-20261107-v1/model-comparison-acceptance.json)。

600 秒全分辨率 RGB-D 隔离负载已完成：每路 18,017 帧、240,176 个控制事件，无序号
缺口；约 138.25 MB/s 原始像素流，队列峰值约 15 MiB。录制 writer 采样期 RSS 最大
185.2 MiB，末 120 秒为 182.9–185.2 MiB。这是该 writer 的有限时长证据，不能证明
全部服务无限时长内存恒定。原 CLI 的 120 秒封存等待先到期，但原 writer 随后完整封存
并退出，未重跑。测试 CLI 现已改为确认 ingress_closed 后停止合成来源，再等待真实封存
与 writer 退出；`--seal-timeout-s` 为独立观察预算，默认 1800 秒。到期报告 pending、
保留原会话与服务定位，不再把未完成封存称为数据不完整。完整延迟封存用例与 CLI 回归
通过，生产控制门槛不变。参见 [满分辨率持续负载](../.evidence/infer-pedal2-start-20261107-v1/full-resolution-endurance-acceptance.json)
及 [观察等待修复](../.evidence/infer-pedal2-start-20261107-v1/load-observer-acceptance.json)。

隔离 CPU 的三路 640×480 RGB-D／100 Hz state／400 Hz 控制事件检查在取消测试核数限制后
连续 60 秒完整通过，RGB 和深度原始像素摘要一致；限制到四个效率核时会丢失接收数据并
触及 128 MiB 队列上限，按设计记录 incomplete。线上服务未设该四核限制。
这是容量与故障可见性证据，不是实机长时或 G34 正式预算资格；早先单次 RGB 控制事件缺口
仍未确定原因。参见 [容量检查](../.evidence/infer-pedal2-start-20261107-v1/rgbd-capacity-acceptance.json)。
CPU/Node、实际 HTTP 与资源摘要均已核验；本机浏览器自动化关闭，本轮没有页面截图。

## 历史交接与部署记录（2026-10-31）

下列状态、目录及启动命令是 2026-10-31 的历史部署，不是当前 Infer 的启动入口。
恢复历史数采部署前须重新核验归属、实际进程和部署契约。

**03:45交接验收通过：工作台环境已退出，原人工数采交接时已启动且硬件就绪。**
独立运维页保留；SDK、Owner、业务、工作台相机、模型和 Triton CPU 均已退出。
原数采任务1251“咖啡打包”，交接验收没有录制或上传。
**后续09:32踏板USB两次断连／重新识别，原数采因踏板进程退出而停止；10:36只读观测仍未恢复。**
当前原数采认领仍有效、硬件进程已退出，工作台保持退出，未自动重启。先现场检查踏板USB再恢复；
内核`EMI?`是诊断提示，物理原因尚未确定。使用前核对实时状态，不要仅为查看页面而运行启动脚本。

## 1. 从哪里打开、怎样启动

- 工作台：<http://10.17.32.3:18868/?environment=physical&business=research>
- 独立运维页：<http://10.17.32.3:18868/operations?environment=physical>
- 主机：`wujie@10.17.32.3`（ARX13）。同一内网可直接访问，无需 Mac 转发。

在 **ARX13 本机**执行：

```sh
cd /home/wujie/work/feiyu.zhou/arx13-control-20261029-v1
./start-workbench.sh --check  # 检查版本、配置和运行前置
./start-workbench.sh          # 启动／复用工作台服务
```

默认启动会准备已登记的服务，**同步启动模型 infer 服务并完成模型加载／预热**；
模型为 Base566 epoch008／step67408，按当前发布配置选择 Torch 或 Triton；本次退出前选择为 Triton。
历史首次部署默认 Torch，不能据此推断后续选择。
**不新接 SDK、不下发机器人动作、不清故障锁**。已就绪时复用服务、模型 worker 和 SDK 身份。
输出中的 `integrity_passed`、`startup_ready`、`services_ready`、`model_ready`、`control_ready`
分别说明文件完整性、启动前置、服务、模型及设备控制条件。本版启动成功要求 `model_ready=true`；
模型 ready 不代表本机控制映射已通过，实际 Run 仍独立检查。

数采尚占用时，优先在运维页使用 **“清理现有控制并接入”**。仅需将服务从数采交给工作台、
暂不接 SDK 时，可显式执行：

```sh
./start-workbench.sh --handoff-collection
```

该选项调用已登记数采的正常停止接口（其流程可能回零），核实硬件与网页进程退出后启动服务；
不删除数据、不强杀、不修改数采环境。保存是否成功看数采自己的保存回执。

## 2. 接入、动作、停止和交接

**“清理现有控制并接入”是一项受核验的接管流程：**

1. 若工作台已经连接，先 Stop，撤销旧任务，等待保持返回与健康静止反馈；健康 SDK 可直接复用。
2. 若已登记数采仍运行，调用其正常停止接口，等待 `safe_ok`、网页及子进程退出，再接入工作台。
3. 必要时走既有中断恢复／重接入口；需要撤掉保持时，页面展示相应支撑确认。
4. 返回当前实例与核验结果，**不会续跑旧动作**。归属冲突、停止未确认或真实故障会明确拒绝。

它覆盖已登记的工作台和数采进程，不会强杀任意外部控制器，因此不能保证在断网、设备离线或
未知进程占用时仍能接入。不要同时从原数采启动入口另开控制器。

日常操作：

1. 选择 Physical，核对设备 ARX13、反馈、相机及当前控制权。
2. 显式“交接控制权”到需要的栏目；**切换栏目只改变页面，不改变持权者**。
3. 人工规划拖动／填写目标只形成草稿与预览；点击执行，预检通过后审阅并确认动作。
4. 研究动作同样是“保存配置 → 准备 → 审阅执行”。修改参数后须重新准备，不自动套用旧审阅。
5. 页面 Stop 走独立 Owner 通道，取消未来指令并请求当前姿态保持。停止不等于释放 SDK。
6. 中断恢复只恢复符合条件的控制状态；旧动作不续跑，下一次动作须重新准备。

### 一键交回人工数采

1. 打开独立运维页，在“接入与控制”中点击 **“一键退出工作台环境”**，审阅后确认一次。
2. 等待 **“工作台环境已退出”**：按钮依次 Stop、通过正常规划以0.1 rad/s回零（保留夹爪开度）、
   正常释放SDK、停止工作台模型／相机／Triton CPU／业务／Owner，确认进程退出后恢复原数采网页。
3. 在 **ARX13 本机**打开原数采页 `http://127.0.0.1:18980/`，沿用原任务，点击
   “启动硬件并准备采集”。看到“硬件就绪，等待踏板开始”和三路 LIVE 后再开始人工示教。
   该地址只监听ARX13本机；从其他电脑不要直接把127.0.0.1当作ARX地址，沿用原有数采入口。

退出按钮只恢复数采网页，**不自动启动数采硬件、不录制、不上传**。这次验收在按钮成功后，
另外调用原数采启动入口一次，已实际到达ready。原数采代码、环境、配置和既有数据均未改动。
独立管理页保留，用来查看回执；它不占用机器人、相机或GPU，也不会自动重接。
退出后业务已停，控机心跳／Runtime栏可能显示未知；以“环境已退出”的成功回执和各资源已停止为准，
不要因此点击中断恢复。关闭浏览器或只点Stop不能替代环境退出。

回零、释放或任一资源退出未确认时，流程停在具体失败阶段，**不会继续恢复数采或强杀进程**。
先查看同一操作ID的完整回执，不因页面超时重复提交。重新交回工作台时，先结束人工数采操作，
再用“清理现有控制并接入”，或仅需服务时执行 `start-workbench.sh --handoff-collection`。

### ARX13 踏板 3

第三键实测为 **Linux `KEY_C`，键码 46**，Owner 与业务配置均已绑定。第一次按下触发软件停止；
既有逻辑中，松开后再次按下可能进入手动接管，不能把连续踩踏理解成反复 Stop。
它不是硬件断电急停。已完成键码实测和运行时绑定检查；**SDK 接入后的现场踩踏 Stop 验收待补**，
用户本轮暂时无法操作。软件 Stop 的实机验收已完成。

## 3. 功能和当前可用范围

| 模块 | 功能 | 本版实际边界 |
|---|---|---|
| 共用控制与人工规划 | 12 关节、夹爪目标、预览、动作审阅、交接、Stop | 已真实控机；角度显示为度，下发为 rad |
| 标定 | 头部／双腕采样、求解、候选查看及安装 | 入口和共用控机可用；本轮未做完整标定求解 |
| 控制研究 | 参数预设、完整 1.5× 研究轨迹、数值记录与中间结论 | 已配置；研究按钮自身不自动录像，且不代表精度／平滑度已达标 |
| 观察与目标浏览 | 三路实拍、目标姿态、人工规划草稿 | 三路实拍正常；名义模型预览不等于实机标定资格 |
| 工作台自动数采 | 规则动作、录制和结果入口 | ARX13 `collection_setup` 尚未装配，完整自动数采未验收 |
| 原人工数采 | Prometheus人工遥操作／示教录制 | 一键退出后已实际启动，机器人／相机／踏板ready；本轮未录制episode |
| 推理 | 统一 Torch／Triton 模式、checkpoint 选择、模型服务与预热 | 一键启动、两模式固定输入推理均已实测；机器人 Run 因本机映射未验收而禁用 |
| 结果与回放 | 按业务查看历史结果、视频、曲线和回执 | 历史结果不是当前控制状态 |
| 服务运维 | 独立状态、接入／释放／恢复、服务与相机管理 | 接管与环境退出均已真实点击验收；退出后原人工数采启动通过 |

模型控机还需要本机关节零偏／符号及训练夹爪行程映射资格。原生夹爪端点已绑定，不等于
训练模型坐标映射已完成。此前 Torch checkpoint 不可见由 inference/catalog 未装配导致，
本版已修复，两种模式均能看到 epoch008。人工动作或固定输入模型回包不能代替模型控机验收。

## 4. 部署位置与计算依赖

本地开发仓库：`/Users/wujie/Developer/work/anyverse/github/zhoufeiyu17/robot-workbench`。
远端以下相对路径均以 `/home/wujie/work/feiyu.zhou/arx13-control-20261029-v1` 为根：

| 路径 | 用途 |
|---|---|
| `source-reclaim-control-v3/` | 当前独立 Owner 的冻结代码 |
| `source-environment-exit-v9/` | 当前管理／网页及业务代码，包含环境退出与手工规划读回同步修复 |
| `source-model-startup-v6/scripts/serve_base566_model.py` | 已验收的常驻模型 endpoint，复用外部冻结的模型数值实现 |
| `run-reclaim-control-v3/` | Owner 配置与描述文件；本轮未替换 Owner |
| `run-environment-exit-v9/` | 当前业务、管理配置和启动入口 |
| `run-model-startup-v6/model/` | 模型 registry、CID、ready 和进程身份回执；凭据不共享 |
| `startup/current-release.json` | 当前启动版本选择器；本次为 `release-environment-exit-20261031-v9.json` |
| `deployment-tools-model-v6r5/workbench_deploy.py` | 已安装启动／封存工具，仓库源为 `scripts/workbench_deploy.py` |
| `env/` | 工作台独立 Python 环境 |
| `sdk-dm-equivalent-v1/`、`legacy-pedal-mapped-v1/` | 绑定的 SDK 与工作台专用适配副本 |
| `source-v2/scripts/serve_camera_sources.py` | 独立相机组件，本轮保持既有版本 |
| `storage-v2/`、`references/` | 业务存储及研究参考；与冻结代码分开 |
| `reclaim-control-20261030-v3/` | 发布、启动和本轮实机验收记录 |
| `cold-start-20261030-v1/` | 五服务真实全退出／冷启动证据 |
| `model-startup-20261031-v1/` | 模型服务退出／冷启动、目录、固定输入回包与最终交付记录 |
| `exit-collection-20261031-v1/` | 一键退出真实点击、回零／释放和原人工数采启动／只读核验 |
| `docs/WORKBENCH_GUIDE.md` | 本说明的 ARX 本机副本，放在冻结源码之外 |

网页、管理、业务、Owner、相机、Triton CPU 工具和模型 infer 的部署计算均在 **ARX13**；
不依赖 Mac 或 3090 station 转发／计算。浏览器 WebGL 使用访问者设备。模型使用 ARX13 的
RTX 4090；本轮退出后模型容器和worker已确认退出，GPU用途已交回原人工数采。
等待人工录制时GPU compute为空是正常现象，不能据此抢占数采资源。一键启动不依赖旧实验的临时 GPU 窗口；
日常 operator 服务按固定主机／UUID及同一 GPU 文件锁准入，保留显存与已有 compute 进程检查。
进程互斥不能代替资源预留登记；Agent 触发的加载／推理仍须先取得唯一 GPU 表中的有效预留。

用户级服务前缀为 `robot-workbench-arx13-`：`manager.service`（18868 内网入口）、
`business.service`（127.0.0.1:18878）、`owner.service`、`cameras.service`。
`triton-control.service` 独立监听 127.0.0.1:18924，已纳入一键脚本启动清单。原数采网页为 `prometheus-arx13-web.service`，
监听 127.0.0.1:18980；不能用其 LAN 端口访问失败推断数采已退出。
本次没有修改原数采代码、环境、CAN 配置或数据。

真正的模型服务是 Docker 容器 `pi05-arx13-resident-workbench-v6`，监听 127.0.0.1:18925；
18924 仅负责转换等工具任务。模型数值源码与制品只读复用
`/home/wujie/work/feiyu.zhou/manipulation-arx-pi05-20261029`，未修改冻结模型实现。
常驻适配器只改变 operator 资源准入口径，保持原 `triton-control/gpu-use.lock` 的同一 inode，
从加载到 worker 确认退出全程持锁。进程身份同时绑定 Docker CID、真实宿主 PID 与启动时刻。

目标姿态用浏览器 WebGL 渲染，没有另一个目标预览主进程；实拍 mesh 预览是业务后端内的
CPU 线程，只有选择合格标定组才可使用。本轮缺完整标定组，不能把线程存在等同叠加已验收。

## 5. 核心控制逻辑

```text
浏览器草稿／研究配置
  → 业务预检、冻结计划、一次审阅
  → 唯一 ControlOwner：持权、实例、命令代次和保护核验
  → 共用目标解释器：平滑参考与 q/dq 时间窗口
  → SDK 原生输出 → CAN → 电机内环
  ← 关节反馈、速度与状态 ← SDK／独立 Guard

Stop → 独立 Owner 通道 → 撤销旧代次与未来队列 → 请求保持 → 核验返回及静止反馈
相机／录像／目标显示独立于控制输出；页面断开不会停止 Owner。
```

普通动作采用 SDK 默认运动增益 `normal_X5`，I=0；研究配置只有保存并绑定新动作后生效。
停止保持使用独立增益，不能拿它推断运动中的 P/D。当前保留 0.8 rad/s 关节速度保护及
独立 100 ms 反馈／心跳保护；保护故障不自动重试。
反馈允许的边界余量与命令限位分开：夹爪左命令 0–80 mm、反馈 −3–83 mm；右命令
0–89 mm、反馈 −3–92 mm，关节反馈有 2° 边界余量；这些余量不扩大下发命令范围。

## 6. 常见异常与处理

| 现象 | 优先检查与处理 |
|---|---|
| 网页打不开 | 核对 ARX13 电源／网络；本机执行 `--check`，再按结果运行启动脚本；不先重启 SDK |
| 页面能看但不能执行 | 看当前持权、反馈和业务依赖；显式交接；数采／推理未配置不靠重新接 SDK 解决 |
| 接入被数采占用阻塞 | 用“清理现有控制并接入”；等待正常停止回执，退出不明时不另开 SDK |
| `FAULT_STOP_UNCONFIRMED` | 表示缺少停止确认，不等于已安全停住；看具体 reason、保持回执和新鲜反馈，再按恢复入口要求处理 |
| Owner／CAN 心跳失联 | 不续跑旧动作；保留故障记录。无健康反馈时先确认现场支撑，再使用适用的恢复流程 |
| 踏板断连／ENODEV | 查看 USB 与踏板健康；本版锁存动作故障后继续真实反馈读取，避免旧分支跳读引发二次心跳退出；不自动清锁 |
| 相机陈旧／无图 | 区分具体相机、源帧龄和服务状态，查 USB；先处理相机组件，不重启整个控制进程 |
| `target changed`／部署身份变了 | 刷新页面后重新预检／审阅，不重复提交旧请求 |
| 启动清单或 unit 哈希不符 | 核对实际版本并重新封存合格候选；不跳过校验，不覆盖原数采文件 |
| 操作中断／结果未知 | 查同一操作 ID 的回执及实际状态；不要因浏览器超时重放动作 |

只读日志命令（ARX13 本机）：

```sh
systemctl --user status robot-workbench-arx13-manager.service robot-workbench-arx13-business.service robot-workbench-arx13-owner.service robot-workbench-arx13-cameras.service --no-pager
journalctl --user -u robot-workbench-arx13-owner.service -n 100 --no-pager
```

## 7. 本版验收与后续

当前管理发布身份：`64cb428b021aadb854013c8c41917f612218b3d54d672051add03a2adea64ca5`。
下述真实控机验收阶段身份为 `3c2d3d425488e3de0ccec1a6cf768af1219913c021cb94002c47c542e9076270`。
基础候选 300 项 Linux 检查通过；增加踏板绑定后的相关 190 项通过（不能相加当作互不重复的覆盖）。
真实浏览器已验收数采退出后接入、健康 SDK 停止后复用两种路径；重复启动不换 SDK，SDK acquisitions=0。

四栏目各经共用人工入口执行左 J1 小角度动作，其余目标保持，约 1 秒后 Stop：

| 栏目 | 运动起点到 Stop 接受 | Stop 接受到 SDK 保持返回 | J1 实际位移 |
|---|---:|---:|---:|
| 标定 | 1.0011 s | 13.22 ms | −0.743° |
| 控制研究 | 1.0012 s | 9.84 ms | +0.699° |
| 数采 | 1.0017 s | 5.08 ms | −0.699° |
| 推理 | 1.0007 s | 7.70 ms | +0.721° |

随后均健康静止、自动动作关闭，未触发速度保护。上述毫秒值是 SDK 调用返回时间，**不是机械
停止耗时**；不是完整业务或控制精度验收。早期轮询计时样本在 1.545 秒后才 Stop，原件保留，
不计入上表。该阶段结束时为已接入 `STOPPED_DAMPING_HOLD`。之后为冷启动验收，按正常链
0.1 rad/s 回零并释放 SDK；回零残差保留，未将其当作精度通过。

### 环境退出与人工数采启动（2026-10-31 最新）

真实浏览器点击退出及确认各一次，成功操作ID为`b9b2db8bc4704aa58c93bcd23c494ff0`；
03:23 CST完成正常回零、SDK释放及五项注册资源退出，管理页保留，原数采网页恢复idle。
03:33 CST单次调用原`/api/collect/start`，约5秒后到达ready。03:45只读复核为同一会话、
心跳序号持续增长、无孤儿进程、robot/cameras/pedal均ready、event已启动，三路预览LIVE。
任务1251，新批次`咖啡打包-20261031_033312-0`；新批录制0条，原4条已上传记录不变。
**通过的是环境交接和硬件启动，不是完整episode录制、保存、上传或质检验收。**

Linux相关132项检查与后续104项共享链／竞态回归通过（覆盖重叠，不相加）。首次按钮尝试在
规划前因Stop交接后的反馈缓存代次拒绝而失败，未下发回零；修复为有界只读同步且保留审阅撤销。
原失败回执保留。数采启动验收脚本误要求ready阶段存在recorder，因此曾超时；核对原实现后，
仅补只读验收，没有重发启动。recorder只在开始一条录制时创建。

### 模型 infer 冷启动（2026-10-31 00:53 历史）

正常停止模型后，核对旧容器、服务与 GPU worker 全退出；单次 `start-workbench.sh` 用时
**58.84 秒**重新创建模型容器并加载／预热。返回 `model_configured=true`、`model_ready=true`、
`sdk_acquisitions=0`，新服务 PID 与 worker PID 均已核验。再次执行脚本为 `actions=[]`、
`model_preparation.reused=true`，不重复加载。已有 SDK 身份与停止保持全程保留。

Torch 和 Triton 均使用同一份已保存的训练域输入完成真实 GPU 推理，得到有限的 **60×14** 输出；
这不是机器人动作。该阶段验收结束恢复 Torch；其后用户切换为 Triton，本次发布保留该选择。浏览器实测两模式均有 epoch008 checkpoint，
模型显示“已加载”，Run 按未验收的本机映射明确禁用。

本次同时修复：infer 服务未注册／未启动、模型与业务认证文件不一致，以及管理器尚未观察到
新模型 PID 就提前提交预热的时序问题。首次失败原件保留，启动器不自动重放未知操作。
本轮未重启独立 Owner、未重新接 SDK。此前五服务整体冷启动记录见下节；不将分段验收表述为
本轮再次完成所有服务／主机电源的冷启动。

### 五服务整体冷启动（2026-10-30 历史）

2026-10-30：管理、业务、相机、Owner、Triton CPU 五项服务及其 cgroup 子进程全部正常退出，
核对旧 PID 消失；仅调用一次 `start-workbench.sh`，约 **4.02 秒**启动五项服务，全部主 PID 更新。
顺序为 Owner → 相机 → 业务 → Triton CPU → 管理。三相机新帧、12轴人工规划、浏览器真实
WebGL目标图像、Triton目录均已核验。没有启动GPU任务，没有接SDK；最终为 `DISCONNECTED`。
这是同一主机正常退出后的进程冷启动，**不是断电／整机重启测试**。原验收脚本最后误取断开态
不存在的 `automatic_enabled` 字段而报错，原件保留；随后只读核验成功，没有重复启动。

冷启动之前的一键脚本漏掉独立 Triton CPU 服务，已通过可信 unit／入口／端口／凭据路径绑定
补入清单，并保留其 GPU 任务准入。新增相关 Linux 16 项检查通过；旧 Owner、控制代码和增益未改。
随后仅更新管理／网页，修复模型下载期间点击预览误发 CPU 备用请求的问题。故意延迟模型下载后
立即点击已复验：本地目标渲染成功、没有备用请求或前端错误；该显示更新只补起管理进程，
不冒充再次执行了五服务冷启动。

## 8. 给二次开发者的入口

代码仓库：<https://github.com/zhoufeiyu17/robot-workbench>。当前本地分支为
`fix/shared-workspace-tools`，上一已提交基线为 `e87aafc791fc85c94b4b0eb8e9bf62adf60424de`。
2026-10-31源码整理提交为 `11454b92b44d9056c3dbc06995647c97175ffafc`，完整检查见[验收摘要](acceptance/workbench-20261031.json)；SDK原生库、模型制品、机器标定和私有运行配置仍是外部依赖，
clone源码不代表已具备真机部署和控机资格。实际部署的冻结源码
在 ARX13 的 `source-reclaim-control-v3/`（Owner）、`source-environment-exit-v9/`（管理／业务／网页）
及 `source-model-startup-v6/`（已验收模型 endpoint），最新启动工具在 `deployment-tools-model-v6r5/`；
二次开发先取得这些只读快照及发布清单，再建立独立开发分支。不要直接编辑线上冻结目录，
也不要整树覆盖本地既有工作。

| 要修改的功能 | 优先阅读的仓库文件 |
|---|---|
| 页面与人工规划／目标显示 | `web/app.js`、`web/operations.js`、`web/target_viewer.js` |
| HTTP与业务调度 | `robot_workbench/service.py`、`workflows.py` |
| 运维接管与生命周期 | `robot_workbench/management.py`、`operations.py`、`managed_services.py` |
| 唯一Owner与控制状态 | `robot_workbench/control_owner.py`、`runtime.py` |
| 参考曲线与SDK窗口 | `robot_workbench/target_interpreter.py`、`motion_reference.py`、`sdk_process.py` |
| 机器配置与保护 | `robot_workbench/device_bootstrap.py`、`arx_output_guard.py`、`joint_bounds.py` |
| 相机／录像／预览 | `scripts/serve_camera_sources.py`、`robot_workbench/rgbd_service.py`、`recording_process.py`、`physical_preview.py` |
| 启动与部署 | `scripts/start_workbench.sh`、`scripts/workbench_deploy.py`；已安装入口的工具目录由发布版本绑定 |
| 常驻模型接入 | `scripts/serve_base566_model.py`、`robot_workbench/docker_model_service.py`；资源准入与数值模型实现分开 |
| 推理候选 | `robot_workbench/inference_workflow.py`、`inference_transport.py`、`triton_control.py`；本地候选与已部署版本分开看 |

在独立开发目录进行无真机检查，不使用原数采 Conda：

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
sh scripts/check.sh
PYTHONPATH=. .venv/bin/python -B -m robot_workbench.service --protocol-fake --state-dir .evidence/local-device --port 18866
```

开发入口为 `http://127.0.0.1:18866/`。Protocol fake 只验证接口／状态逻辑，不包含真实物理／相机；
外部资源缺失而跳过的检查不算通过。修改控机链后，在目标 Linux 做相关回归，使用新发布目录与
新清单；走正常回零／释放维护，再核验停止／恢复。部署命令参考下列规范；新版本需要重新绑定
Owner、业务、SDK、相机、模型及 unit 身份，不能只改 `current-release.json` 伪装升级。
源码提交与远端状态以Git记录为准；此次Git整理不重启线上服务，也不扩大既有实机资格。

剩余顺序：① 补踏板 3 实际触发 Stop（用户可操作时）；② 补本机模型映射后验收模型动作，由
W2 与模型部署任务协作；③ W2/W3 补完整自动数采。人工数采环境交接、模型服务启动与模式选择已通过。
控制精度和平滑度研究另行继续，
本版不宣称解决原有抖动。

证据入口：[模型冷启动](../.evidence/arx13-model-startup-20261031-v1/cold-start-r4.json)，
[模型真实回包](../.evidence/arx13-model-startup-20261031-v1/model-smoke.json)，
[模型页面验收](../.evidence/arx13-model-startup-20261031-v1/infer-torch-ready.png)，
[五服务冷启动独立核验](../.evidence/arx13-cold-start-20261030-v1/cold-start-readonly-verification.json)，
[冷启动预览截图](../.evidence/arx13-cold-start-20261030-v1/target-preview-verified.png)，
[本轮控机验收汇总](../.evidence/arx13-reclaim-startup-20261030-v1/acceptance-summary.json)，
[实际发布记录](../.evidence/arx13-reclaim-startup-20261030-v1/publication.json)，
[10月30日只读状态](../.evidence/arx13-reclaim-startup-20261030-v1/final-state.json)，
[桌面运维截图](../.evidence/arx13-reclaim-startup-20261030-v1/operations-final.png)。
项目续接权威索引为 `.evidence/next-optimization-20261017/continuation-state.json`。
详细首次部署、封存及回滚流程见 [部署规范](physical-deployment-runbook.md)；其中旧版本段落按日期理解。

本次退出与数采交接收据：`.evidence/arx13-exit-collection-20261031-v1/delivery.json`；
真实点击、原启动及只读核验原件随同目录保存。既有模型启动与冷启动回执按日期保留。
