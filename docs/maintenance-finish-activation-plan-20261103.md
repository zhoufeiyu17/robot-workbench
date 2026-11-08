# 本轮 C 维护 finish：启用差异、操作与回退方案

Owner：C `01a16327-7545-7332-a361-0123ddad811d`。日期：2026-11-03，Asia/Shanghai。
状态：**离线待审方案；没有可执行的激活 profile，未部署或控机。**
本文件是本轮操作方案的统一入口；字段规范只引用 [C 唯一接口源](WORKBENCH_INTEGRATION_INTERFACE.md)与其中的 B 契约，不维护第二份协议。
参数、native 身份、现场事实或本轮范围改变时更新本方案；原交付和失败日志保留。

## 选定范围

本轮采用 **初始 HOLD → END → 显式 `maintenance_home_v1` → 原维护 release**。
不发 MODEL/TEACH 意图，不采用 D 的四段开爪策略，不写正式 Raw。
HOLD 本身仍有 SDK 输出和增益切换，不能把“没有模型调用”称为只读验收。

原“一键环境退出”的旧 home 路由不升级，不用该按钮代替新入口；本轮也不新增页面按钮。
SDK 释放与环境退出分开，末态不自动恢复数采、关闭共享模型或重接设备。
A 的纯服务验证包独立继续，原 A/C 31 项包不因本方案改动。

依据：[最终 C/B 消费回执](../.evidence/workbench-control-consumer-20261103-v2/receipt.json)、[组合选择](../.evidence/workbench-control-consumer-20261103-v2/candidate/composition.json)。
本轮结构化审阅差异及缺项见 [activation-review.json](../.evidence/workbench-maintenance-activation-plan-20261103-v1/activation-review.json)，其 `executable=false`，不是部署配置。

## 启用差异

相对已冻结但关闭新控制能力的组合候选，未来经审阅后需要以下成套变化；本轮没有应用：

| 位置 | 当前候选 | 拟启用值 / 待审内容 |
| --- | --- | --- |
| Owner 与 business 的 `preserve_begin_jaws` | false | true；两端一致，不能只改业务端 |
| `inference.action_chunks` | false | true，使用真实 acquired binding 与 Owner context |
| `inference.recording_enabled` | false | 保持 false |
| `inference.normal_initial_mode` | 未配置，默认 MODEL | 显式 HOLD |
| `inference.normal_finish_kind` | 未配置 | 显式 `maintenance_home_v1` |
| `normal_modes.recording_required / recorder_ids / recording_clock_id` | 未配置 | false / 空列表 / `host_wall_time_ns`，不伪造 gate |
| normal_modes 其余五字段 | 未配置 | 制动时长、速度/加速度上界、增益过渡时长、teach_kd_scale **均未选值** |
| normal_finish_policy | 未配置 | 真实 jaw mapping pin、完整 execution_settings、回零速度、jaw 速度界、超时 **待冻结** |

已保存的 `motion_gain_profile="normal_X5"`、`target_interpreter_profile="x5_control50_damping_v1"`、
`native_reference_api="preserve_reference_v1"`、双爪启用和共享 model external 归属保留。
前两者职责不同：运动增益已有显式 normal_X5，不能仅因参考解释器名称不同就擅自改名或改 PD。
`initial_hold_mode` 在记录中缺省，启动代码默认 manual_gravity；这不等于已经完成 normal 初始 HOLD。

本轮不执行 TEACH，但 B 的完整 normal_modes schema 仍要求显式 teach_kd_scale；不抄 CPU fixture 填空。
同理，保夹爪 home 的 jaw_speed_mm_s 仍须是明确合法上界，不能因不主动开爪而随意填 0。
代码允许范围只是输入校验，不是推荐的物理参数。

## 必须补齐的具体事实

| 缺项 | 已知事实 | 补齐方式与边界 |
| --- | --- | --- |
| 正常模式参数及阶段预算 | CPU 接线已验；没有本轮物理预算 | 明确 normal_modes 五字段、preview.duration_s、HOLD 观察期与各操作 deadline，不用测试值代替批准值 |
| finish 设置 | 源码要求 arrival/tracking/speed protection 全开 | 从可信当前设置形成完整 policy，保留原保护；核实际 SDK 的关节/夹爪速度上界，不自动放宽限位/跟踪/限速 |
| 最终 native 来源 | module SHA `f17ddb10…`，build `0d5e5989…`；归档 SO 与之不同 | 核 `/home/wujie/work/feiyu.zhou/arx6-live-infer-20261102-v1/evidence/native-arx6-source.json` 及当前二进制/源文件；不能用 CPU fixture 顶替 |
| 当前夹爪命令与增益 | endpoint pin `0d256b2b…`，mapping 路径已记录 | 新鲜核 mapping SHA、SDK native calibration identity，以及同 epoch 的完整 q/dq/jaw 命令、逐臂 gains/sequence/time/instance；反馈 gap 不代替命令 |
| 指尖、负载和回零扫掠 | 只换指尖的历史事实；24 份名义 mesh 记录一致 | 补新指尖 CAD/安装关系或保守尺寸/偏置/误差界、当前负载，以及当前姿态到双臂零位的具体扫掠与相关障碍；不要求无依据重建整臂 |
| 当前设备归属与接入条件 | 历史已交回数采，不能代表现在 | 执行方核 D 的 manager/recorder/SDK 退出与当前控制归属，再按现有接入规则核现场条件；旧授权条件不变时不重复泛问 |
| 当前软件与模型实例 | 主机/端口/unit/目录有历史记录 | 核现时 host/boot/source/config/unit、新目录冲突及 A 的持久共享端点/worker/bundle；临时 port0 验证实例不能代替固定18925部署身份 |

83 mm 是记录坐标绑定，不是新指尖完整外形认证。旧内接球代理没有完整外包络、连续碰撞或外部环境证明。
旧 09:07 故障、10:24 释放及早先空场确认都是带时间的证据，不填作当前姿态/负载/健康状态。
初次接入或故障后的恢复由执行方结合已有授权核验；本方案不在离线阶段再次索要现场批准。

## 操作入口与顺序

使用现有工作台客户端、CSRF/目标身份和唯一 Owner。以下是正常 API 流程，不代表每一步再向用户索要批准；具体真实执行范围由中控统一对齐。

1. **准入与准备。** 先核不需 SDK 的软件、归属、静态限值和可预先审定的参数。只有接入后才有的完整当前输出、增益、姿态及据此计算的扫掠，放在有授权且满足现场条件的接入后核验；不伪造未连接时的反馈，也不形成“未接入先回零”的循环。strict Owner 能力须在本次首次候选加载时就位；若后续只调整业务参数，走保留 Owner PID/nonce 的原独立业务维护链，不为此反复重接 SDK。确认 A 持久共享服务及所选模型就绪后，C 再用模型准备/绑定入口取得三相机绑定及 pin；不使用旧 nonce/token/worker 身份，不调用临时验证服务替代目标实例。
2. **新 review/confirm，初始 HOLD。** 通过 `inference_prepare`、`inference_preview` 与原 `execute` 审阅链。读取新 `mode_session_id`、runtime nonce、当前 epoch 与 `finish_capability`。初始 HOLD 完成应有 `normal_initialize` 证据、稳定健康反馈、coherent 输出/增益，且本轮没有 policy prediction_requested 事件；共享服务其他消费者的请求不计作本轮。
3. **明确 END。** 通过 `POST /api/actions/inference_mode` 发一次 END（mode=HOLD），用当前身份。即使已经 HOLD，也须得到真实 terminal completed，而非 no_op；记录其 request_id 与完成 `command_epoch`。
4. **明确维护 finish。** 通过 `POST /api/actions/inference_finish`，使用上述 ending_request_id、新鲜当前 epoch 和当前 `finish_capability.policy_digest`。policy_digest 不是简单的配置文件 SHA；不自行计算替代。等待同 request/session/nonce 的完成回执，并确认对应当前代次。
5. **独立原维护 release。** 只有合格 home 完成且原维护前置满足，才沿管理 API `/api/operations/action` 的 `release_control`（或原维护 actor 的等价入口）释放，使用本次维护票据并查询该操作结果。按原规则使用真实授权依据，不把接口的支撑确认字段伪造成新的现场事实。单独取得 released/close/退出证据后结束本轮，保留共享模型与页面供检查。

Mixed366 的 **0.5 秒 preview 会进入旧 bounded diagnostic，不能与 normal_modes 组合**；本轮须另行选择支持的非0.5 duration。
该 duration 也不能被解读成 HOLD 的自动超时保障；阶段观察期、操作 deadline 与终止条件需分别明确。
模型 pin session_id 与控制 runner 的 mode_session_id 不同，不能互换。
未知结果先查原 mode_receipts/操作回执；不得换 ID 重放，也不能用新 epoch 改写原请求载荷冒充查询。

## 验收口径

- 配置/身份：A、B、C 来源与实际选择一致，新能力明确启用，未连接不支持该协议的旧 Owner；原保护均保留。
- 初始 HOLD：完整当前命令及增益来源可追踪，夹爪目标来自输出而非反馈，按本轮已选观察预算核实稳定；无本轮模型输出调用。
- END：当前实例/代次的 terminal completed，实际 HOLD，尚未 home 或 release。
- finish：`op=normal_finish`、`completed=true`、`home_completed=true`、`jaw_open_completed=false`、`release_ready=true`、`sdk_released=false`；阶段 reference/SDK窗口、到位/跟踪/速度及新鲜静止反馈能关联，夹爪目标保持。
- release：另有 `status=released`、`sdk_close_returned=true`，Runtime/自有 worker 退出确认。SDK 返回、native 输出和反馈分别记录，不声称 motor ACK。

这些通过只资格本轮明确配置与路径，不自动达到一格精度、无振荡研究目标、连续训练节奏或完整任务成功。
诊断证据与正式 Raw 分开；不为了本轮补录 D 数据、加做 P/D 扫描或恢复旧1.5×研究。

## 失败与回退

- 未安装/未接入：保持关闭新 flags 的候选与原现场；无设备回退动作，保留文件证据。
- 已换软件但未接 SDK：核自有进程身份后恢复匹配的消费者选择；共享模型仍归 A。不能启动旧 private manager 接管仍被使用的共享模型。
- 已接入且健康：仅合格的新显式 finish 完成后沿原维护 release；正常步骤失败、超时或身份未知不报“安全收尾”，不自动放宽参数或重试。
- 故障、失联或未知控制结果：原 Stop/Guard 立即优先，不等 Raw/模型；保存原 request/epoch/反馈/输出证据，停止依赖“先回零”的维护。按既有恢复/支撑规则处理，不强杀、重接或撤保持掩盖故障。
- 全量退回私有部署：须先协调全部消费者准入及 A/admin 共享服务退役窗口，再恢复成套原 unit/launcher/config；不能用一次 active_sessions 查询承诺原子安全停服。

当前交付只包括本方案和结构化审阅差异。未创建可加载的激活 profile，未安装、启动、控机、调用模型或修改任何原数采环境。
