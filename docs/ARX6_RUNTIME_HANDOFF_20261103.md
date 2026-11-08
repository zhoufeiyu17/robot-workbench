# ARX6 工作台 → DAgger/replay 运行环境交接

## 阶段收尾（2026-11-04 16:43，CLOSED_WITH_RESIDUALS）

本阶段功能由用户确认“没啥问题”。执行长度支持 1–60 步、默认 15 步；源时长为 N/30 秒，单段规划及刹车余量的最长时长为源时长×4。完整执行所选 N 点，无法满足速度、加速度、限位或刹车余量时保持并结束，不截断轨迹。关节速度 1 rad/s、加速度 40 rad/s²、夹爪 40 mm/s 及观测新鲜度保护保留。32 项 CPU 检查通过；用户随后实际修改步数并运行真机 infer，现场效果以用户验收为准。

当前运行状态已交给正式数采，不是热模型待用：16:37 网页 `exit_environment` 操作 `e7361825803f4dfaaa4991e61cf71f40` 成功，回执确认环境退出、SDK 正常释放、正式 8080 网页恢复。16:43 SSH 复核工作台业务、Owner、模型、RGB 主进程均已退出，GPU 仅剩原桌面 Chrome；本会话 GPU 记录 `arx6-continuous-infer-20261104` 已释放，监控 `arx6-gpu-2` 已不存在。不自动恢复工作台；正式数采硬件准备和录制由用户在正式平台操作。RGB unit 留有退出失败状态，但进程已退出，本次未重启或清除该状态。

源码、差异补丁、测试和实机证据入口为 [本阶段 delivery](../.evidence/variable-steps-20261104-v1/delivery.json) 与 [progress](../.evidence/variable-steps-20261104-v1/progress.json)。实际部署源码仍保存在 `/home/wujie/work/feiyu.zhou/workbench-collection-switch-20261104-v1/provider`。本地 `feature/workbench-shared-lifecycle` 分支存在跨阶段未提交改动，且未设置 upstream；部署快照与工作树未完全归并，本次未 stage、commit 或 push。

剩余事项按优先级：① 下一轮 Git 交付前归并已部署源码、核对既有改动并完成针对性回归；② 对齐中途撤销时的调试回执与延迟封存 trace：运行 `145d476a…` 的 trace 最终完整且哈希匹配，但 Run 回执仍标 incomplete，不能据此声称所有 Run 产物均完整；③ 保留“预测准备期间设备发生移动”的既有诊断，按新日志复现后再决定是否优化稳定等待。本阶段不继续控机或扩展修复。

维护触发：重新启动工作台、下一次发布、上述遗留完成或运行归属变化时，先核验实时状态并更新本段。以下各节均为历史记录，不用其中 PID、固定 15 点／2 秒或早期分工推断当前状态。

## 当前刹车余量修复（2026-11-04 14:15）

用户遇到的第一段准备失败已修复并重新验收。旧全局余量估算把同一预测判为需要 2.490 秒；保留 q/v/a 时间相关性的解析包络证明使同一预测在 1.5086 秒满足原有门槛。单段 2 秒上限、0.2 秒刹车和 1/40/40 动态限制保留。

修复发布后，新的实时观测与预测在 2 秒运行预算下完整执行 15 点，轨迹时长 1.3986 秒，正常 END，反馈健康。当前 Owner/SDK 为 3949125/3971376，runtime nonce `df5fde07…`；模型 service/worker 为 3949128/3960806，设备已连接、健康静止、无自动动作。下一次操作仍需读实时状态。

本地和 Linux 各 41 项检查通过。旧预测仅用于离线复现；实际控机使用新请求。证明、旧失败和新验收统一见 [brake reserve delivery](../.evidence/brake-reserve-20261104-v1/delivery.json)。用户可以刷新推理页继续测试，原资源记录和只读监控继续维护。

## 当前持续推理环境（2026-11-04 13:51）

工作台已恢复，用户可在 [推理页](http://10.17.32.5:18868/?environment=physical&business=inference)选择 40 或 120 秒并 Run。当前为 Torch mixed366-nohistory epoch010 / step34250，模型 loaded/ready，设备 connected、反馈健康静止、holder=inference，Run allowed。自动验收动作已结束。

固定四段配置已移除。默认 120 秒，支持 0.5、1、2、40、120 秒；40/120 是从 MODEL 开始的运行预算，包含观测、推理、准备、等待和执行。初始化保持与 END 收尾另外计时。每段完整执行模型源 0.5 秒/15 点，允许重定时到最多 2 秒；剩余预算不能容纳下一整段时正常结束。0.5 秒为单完整片段诊断。

本轮实际完成四次单片段 Run，轨迹时长依次为 0.9023、1.5762、1.1771、1.6425 秒，各 15 点完整且正常 END。另一次选择 40 秒的持续 Run，在两段完整执行后由验收驱动发送 Stop，停止回执、健康静止和随后再次 Run 均通过。40/120 秒全时长与具体咖啡任务效果由用户现场验收。

新版本还完成正式 8080 退出与恢复工作台。过程中发现 managed 模式先停止模型服务、后关闭 pin 的顺序错误，已改为先释放绑定再停服务；实机确认 pin_release=released，模型正常退出并重新加载。首轮验收观察器漏识别 Application 的 completed 终态，已修正并保留原记录及独立完成回执。

当前 Owner/SDK PID 为 3551029/3556023，runtime nonce `ecb98444…`；模型 service/worker 为 3579240/3581684。具体身份、SHA 与原始回执见 [本轮 delivery](../.evidence/continuous-infer-20261104-v2/delivery.json)。GPU 新记录为 `arx6-continuous-infer-20261104`，由本会话维护；`arx6-gpu-2` 已指向本记录。此前记录保持已释放。

本轮为无录像的控制验收，原始推理输入、输出和控制回执已保存。当前组合的 Triton 转换/实机推理、40/120 秒完整运行、浏览器实际按钮体验仍待验收。

## 当前正式数采交接（2026-11-04 13:13）

用户明确要求按钮跳转正式数采平台。当前入口为 [正式数采网页](http://10.17.32.5:8080/)，服务 `prometheus-web.service`、原发布 `342ac40`、原数据目录 `/home/wujie/workspace/new_data_collection/PrometheusV2/data`，任务 1275 和原 claim 保留。18080 试用网页已停止。

运维按钮为“退出工作台并启动正式数采”。本轮已实际完成双臂 0.1 rad/s 回零、SDK 释放、工作台业务与模型退出，正式网页启动及身份核验。最终回执包含 `environment_exited=true`、`sdk_release_verified=true`、`collection_web_ready=true`，返回地址为 8080。数采硬件准备和录制仍在正式平台内操作。

正式数采独立接管 SDK，因此当前工作台采用已存在的 managed 生命周期及管理员描述文件。默认启动配置和生成器已同步选择正式服务，Owner unit 已补为持久定义。人工规划默认配置及启动检查保留。

本次修正保留两个过程问题的证据。初次串行回零在第二臂遇到旧原生参考未退出，经过已确认的软件恢复，在两臂之间加入正常 Stop 及回执屏障后通过。随后旧临时 Owner unit 在进程退出后消失，管理器等待超时；核实所有旧进程已退出、补齐持久 unit、经原维护恢复入口核销未知状态后，新的完成请求成功。原失败与未知回执保持。

当前实际索引为 [正式交接 delivery](../.evidence/formal-collection-handoff-20261104-v1/delivery.json)。GPU 模型进程已退出，本会话旧试用记录已释放，`arx6-gpu-2` 监控已暂停。浏览器工具禁用，服务端按钮路径和页面导航逻辑已验证，实际点击体验待用户确认。40/120 秒持续 infer 仍为未发布候选。

## 当前部署与启动（2026-11-04 12:49）

工作台、共享控制执行和共享模型服务统一由本会话 `01a16327-7545-7332-a361-0123ddad811d` 负责。GPU 记录 `arx6-user-switch-trial-20261104` 已实际转入本会话，45 分钟只读监控为 `arx6-gpu-2`。下方原分工与早期运行状态均为历史。

ARX6 上继续使用以下启动命令，入口已指向当前部署。

```sh
/home/wujie/work/feiyu.zhou/arx6-workbench-20261102-v1/start-workbench.sh
```

人工规划随业务后端默认装配。启动回执核验 `manual_planning_ready=true`，模型已热时返回 `model_status=ready`，模型准备期间页面可先访问。缺人工规划时启动脚本明确报错。实际业务停止后重新启动、真实人工姿态预览、自碰撞检查均通过，模型 ready、反馈健康，consumer 为 workbench-C。验证期间共享 SDK 与模型持续运行，未执行人工规划的运动。

本次补齐推理专用 profile 漏掉的 `calibration_config`。使用当前部署 `config/calibration-manual-v1.json`，保留既有标定存储目录及对应业务 unit 写权限。自碰撞代理在原几何、网格与 83 mm 夹爪包络完全一致的核验下更新了夹爪映射文件绑定，球数组保持一致，证据保存在交付索引中。

两个平台分别访问工作台 18868 与隔离数采 18080。运维按钮现为“退出工作台并启动数采平台”，它先结束工作台业务并交还自身控制权，再启动或复用登记的数采网页，核验 `/api/status` 的设备身份和就绪结果后给出跳转地址。录制和硬件准备由用户在数采平台操作。该网页已登记为持久 user unit `prometheus-arx6-switch-web.service`，启动流程核对 unit 文件摘要。

已将数采网页正常停止，沿按钮对应 API 验证 `started=true/ready=true`，随后用上述命令恢复工作台。浏览器控制在 Work 设置中禁用，实际按钮点击和页面导航体验仍待用户验证。当前 SDK nonce 仍为 `5a6a5a82…`，Owner/SDK 为 `2329764/2329861`。这是业务服务重启与数采网页冷启动验收，整机重启后的全链冷启动另行核验。

当前实际交付为 [manual-startup delivery](../.evidence/manual-startup-20261104-v1/delivery.json)。40/120 秒持续推理仍是未发布候选，线上保留 finite4。原持续推理冻结包之后已有本次管理器与启动修复，后续发布须根据 `user-duration-infer-20261104-v1/post-manual-integration.json` 重建清单，避免覆盖本次修复。

## 当前亲测入口（2026-11-04 11:40）

D 已完成 C 接入 → C 退出 → D 准备至 READY → D 返回 → C 再接入的五步真实 API 验收。C 本地核验保存回执与 SHA 后更新本入口。以下 05:04 与 11 月 3 日内容均保留为历史。

- [工作台推理](http://10.17.32.5:18868/?environment=physical&business=inference)当前 C 持有控制，模型 ready、Run allowed，自动验收脚本已退出，用户开始亲测。
- C 空闲时，在[运维页](http://10.17.32.5:18868/?environment=physical&business=inference&view=operations)点击“切换到数采工作台”，核验成功后转到[隔离数采页](http://10.17.32.5:18080/)，点击“启动硬件并准备采集”。测试任务明确标注“ARX6现场亲测_DAgger_仅本地”，原正式任务与数据由 D 保留。
- D 完成当前记录的正常结束和保存后，点击“返回工作台”，确认 detach 后转回 C 运维页，再点击“接入本工作台控制”。

本轮实际部署根为 `/home/wujie/work/feiyu.zhou/workbench-collection-switch-20261104-v1`。Owner PID `2329764`，实例 `4ac06c98a6ea06e50880d8dd134b9a56`，SDK worker `2329861`，runtime nonce `5a6a5a82d2cf25eb6c30dcad3e35f6da08493dd21361a89f`。五步切换中实例保持且反馈健康。模型 service/worker `2285559/2285682` 保持运行，实测速度保护 1 rad/s。

D 会话 `01a18153-fb63-7a31-82e7-eeddf12c87ee` 为唯一设备、服务与 GPU 操作方，C 负责本地工作台集成。当前用户 Run 阶段由用户操作。旧 C GPU 记录和监控继续保持结束状态。

统一验收索引为 [actual-delivery-index.json](../.evidence/user-bidirectional-switch-20261104-v1/actual-delivery-index.json)，其中索引 D 的 `user-ready-receipt.json`、五步原始回执与中途拒绝记录。当前接口源为 [WORKBENCH_INTEGRATION_INTERFACE.md](WORKBENCH_INTEGRATION_INTERFACE.md)。本入口在现场状态或负责方改变时更新。

现有资格覆盖真实 API 切换与 CPU 页面行为。本轮准备未新增 infer 动作或录制，页面实际点击、当前组合的用户 infer/DAgger、踏板和手拖体验待现场验收。浏览器工具在 Work 设置中禁用。中途一次 HTTP409 原始正文未被初版验收脚本保存，已保留失败请求、目标 epoch 差异和 fresh 请求成功证据。

## 历史终态（2026-11-04 05:04）

> Latest registered terminal state (D observation2026-11-04 05:04:21+08:00; C checked the saved receipt hashes locally): original production human collection at http://10.17.32.5:8080 is READY. Human manager1962778 and SDK1962790 intentionally hold the device; task1275 and the original claim were preserved field-for-field. READY/not-recording does not mean the SDK is free.
>
> Sole final-state authority: `/Users/wujie/Documents/ChatGPT/first_work/dagger-formal-launch-20261104/final-delivery/receipt.json`. C index: `.evidence/arx6-sdk-handoff-20261104-v1/production-human-terminal-index.json`. The indexed production-human, claim-preservation, GPU-retirement and automation receipts were SHA-checked; C did not SSH or operate any service/device for this update.
>
> D private test Web/Owner and model exited normally; the transferred GPU record was released, D arx6-gpu and hub dagger-infer monitors are PAUSED. C's earlier arx6-infer monitor remains deleted. Camera978254 / publisher instanceff44e9d1072745b48d74fcc806ccee65 and reliable RGB reader1756873 were preserved.
>
> This state supersedes earlier C/D handoff windows and CAN-empty snapshots. The retired D test nonce f469ca6e32aaf4ff630cd613514f96518379419165370f1a and prior C nonce cannot authorize or bind a new connection. Before a future authorized operation, fresh-check production task/claim/recording/SDK and normally hand out that human SDK; do not treat it as a leftover process or restart old test services.
>
> Automatic DAgger/replay/human acceptance is complete within D's receipt scope. Formal C+D concurrent operation, onsite physical pedal/drag checks, and reviewed commit/push remain separate. Candidate source is not committed/pushed and new DAgger was not promoted to production8080. The older sections below are historical evidence, not current deployment instructions. This update grants no new device/deployment/GPU operation.

Owner: C 工作台，Session `01a16327-7545-7332-a361-0123ddad811d`。2026-11-03，company-internal。

## 当前交接结论

C 的四段真实 infer 与视频已全部完成，无剩余需要占用设备的必需验收。23:27 左右只读检查：没有活动 job、自动动作、录制或 SDK 在途调用；后台8次写入完成；SDK连接、健康静止，holder=inference，STOPPED_DAMPING_HOLD，epoch4。
C 从交接准备开始不再启动控机/发布；现有服务保持运行，GPU记录与监控在接收方完成transfer前仍归C。**本文件不是已完成转交回执**。

人类接续授权已直接核验于中控聊天 `01a1805e-2df9-71f1-97d8-7b8d09516652`：
- `01a1a1fb-55cd-78f3-bf5a-9bd1fa85b6d4`：开始推进DAgger/replay，要求真实短轨迹验收。
- `01a1a1fc-af5f-78a0-bd82-300e85f83de4`：等待C最后四段用完后再接真机。
因此无需另外等待“用户亲测结束”。接收方为D `01a18153-fb63-7a31-82e7-eeddf12c87ee`；同一时段仅一方控机/管理GPU。

## 实际设备及发布

| 项目 | 实际值 |
|---|---|
| 主机 | `wujie@10.17.32.5`，hostname `wujie-ThinkStation-P3-Tower` |
| boot | `9f649604-2757-4b93-adbe-2614d5b7dd66` |
| GPU | `GPU-3b0e6216-017c-ffae-3a10-ba7882776a16` |
| UI | <http://10.17.32.5:18868/?environment=physical&business=inference> |
| 控制/业务/管理源 | `/home/wujie/work/feiyu.zhou/arx6-live-infer-20261102-v1/source-control-fourvideo-20261103-v1` |
| 模型源 | `/home/wujie/work/feiyu.zhou/arx6-mixed366-20261102-v1/source-shared-20261103-v1` |
| Owner/Business/Manager PID | `862713 / 862714 / 862716` |
| Guard/native PID | `865175 / 865192` |
| runtime nonce | `118d91ec8b08587f12613da9659bef6360caa9dcaba4bfca` |
| 热模型 service/worker | `650927 / 659381`，worker start_ticks `35147177` |
| 模型cgroup | `/user.slice/user-1000.slice/user@1000.service/app.slice/robot-workbench-arx6-mixed366-model.service` |
| 相机 Owner / RGB subscriber | `978254 / 650930`，保留运行 |
| 原数采网页 | `prometheus-web.service`，按此前交接正常inactive |
| GPU记录 | `arx6-workbench-user-infer-20261103`，当前仍归C |

SSH沿用 `~/.ssh/id_ed25519_new_server`，不复制凭据。模型/status为18925；凭据只由远端已登记token文件读取，不打印。模型registry：`/home/wujie/work/feiyu.zhou/arx6-mixed366-20261102-v1/config/registry-user-infer-20261103-v1.json`。

控制Owner descriptor：`/home/wujie/work/feiyu.zhou/arx6-live-infer-20261102-v1/state/owner-user-infer-20261103-v1/control-owner.json`。
Owner profile仍为同目录部署根下`config/owner-profile-user-infer-20261103-v1.json`；business/manager/backend-launch/startup配置后缀均为`fourvideo-20261103-v1.json`。启动入口仍是`/home/wujie/work/feiyu.zhou/arx6-workbench-20261102-v1/start-workbench.sh`，**接收前不要为了检查执行启动脚本**。

## 已验证范围和不可外推项

- 四段分别0.549815、0.627849、0.645992、0.629804秒，每段完整15点/源0.5秒、新观测/新预测；总执行2.453460秒。独立Stop确认、反馈健康静止。
- 规划与实测速度保护1rad/s、规划加速度40rad/s²、夹爪40mm/s；每段≤2秒。100ms反馈/500ms观测与供应门未变。
- 20项针对性CPU检查通过。当前fourvideo仅比已验nativeadmit增加3文件的3/4段接纳和配置选择4；精确patch及SHA见下方交付。
- infer_native_admission_v1能力已加载；没有部署B尚在开发的shared_consumers。有限3/4段QA策略不能当作正常DAgger/replay通用接口。
- 模型/夹爪单位和校准不变；native末端输出核验不是固件ACK。
- 瞬时配对反馈>100ms可能使preview/control_ready短暂失败，新Run会重新取观测，也可能在动作前拒绝。保留门槛，不自动重试。中控P2 `T-b766f378`。
- 当前网页manual_preview的标定规划服务未配置。本次维护通过已有Owner `preview_workbench` →审阅→confirm完成0.1rad/s双臂回零、保持81mm夹爪目标，再正常release；没有强杀或跳过保护。接收方重接/部署继续按AGENTS正常回零维护或具体故障恢复门。
- 视频为三路已有RGB预览读者，CPU编码、原速、非严格同步。录制PID879211已退出；无活动编码器。

## 统一证据入口

所有本轮证据位于 `/Users/wujie/Developer/worktrees/robot-workbench-shared-lifecycle/.evidence/arx6-four-segment-video-20261103-v1/`：
- `delivery.json`：交付索引；`actual-acceptance.json`：逐段执行/Stop/写入结果。
- `publication-plan.json`、`publication-result.json`、`source-delta.json`、`candidate.patch`：实际代码及切换身份。
- `final-service-state.json`、`final-state.json`：进程、GPU、运行态。
- `home-result.json`：回零收据；`live-fourvideo-231836-a7eb9f0d/`：真实动作前后状态、请求、Owner事件。
- `original-video/`：源视频、逐帧时间/相机身份；用户交付视频 `/Users/wujie/Documents/Codex/outputs/arx6-four-segment-infer-20261103/infer-four-segments.mp4`。
远端同一组证据：`/home/wujie/work/feiyu.zhou/arx6-live-infer-20261102-v1/evidence/user-infer-20261103-v1/four-segment-video-v1/`。

## 最小接收顺序

1. A确认热模型registry/admission是否与旧Session/GPU记录绑定，以及新consumer pin的必要条件；A已核代码和已存现场证据：mixed366不绑定C聊天Session或GPU台账owner；transfer不会释放进程持有的GPU文件锁或改变descriptor，可保留热worker。责任登记、监控、C自己的pin和D的新consumer绑定应分别交接。来源：A聊天`01a19faf-909c-7281-aed7-e019547e736e` 23:31前回执；D仍须接收前fresh核验。
2. D准备好接收后，只读核同一boot/GPU、worker PID/start/父链/cgroup，以及无新动作/录制；不能按旧PID或idle直接抢设备。
3. 在C明确交接窗口内，由D以自己的Session调用唯一`gpu-usage transfer --session D --from-session C --id arx6-workbench-user-infer-20261103 --authorization <上述真实人类指示> --evidence <fresh核验>`。不要新建第二记录或冒用C身份。
4. D确认transfer成功且接管小时监控后，C核回读并删除自己的`arx6-infer`监控，保存双方交接回执。转交不代表卸载模型或改动相机。失败/归属未知时保持原状态，不强停。
5. 后续DAgger/replay由约定的唯一操作方串行验。C已完成QA不代表D入口/共享consumer/录制状态转移已通过；按各自具体候选验收，不复用旧Run request ID，不接第二SDK。

当前剩余：D接收与资源transfer/监控接管；A最终交接证据路径补索引。交接准备不授权提前关闭任何共享服务。

## Raw反馈源文件补充

已只读取得B请求的实际legacy文件，未导入或启动SDK：
`/Users/wujie/Developer/worktrees/robot-workbench-shared-lifecycle/.evidence/arx6-four-segment-video-20261103-v1/legacy-arx5-backend/arx5_backend.py`。
SHA256 `38cfa9b3b327a4a17f765dcf9d166c48b7555e3322f7a4a66b4d5286b8d07a25`，19215字节。同目录`arx5_backend.provenance.json`记录远端`/home/wujie/work/feiyu.zhou/coffee-rule-harness-20260916/source-owner-v6-home/infra/arx_env/backends/arx5_backend.py`。供B/D核验同一SDK反馈快照与FK路径，不等于新的Raw采样语义已通过。

## Final model-owner confirmation and receiver readiness

A confirmed at23:32: the deployed mixed366/model_session/inference_transport/serve_mixed366_model manifest matches its audited source. Registry and descriptor do not bind the Codex Session or GPU record owner. The service holds R/gpu-use.lock independently; gpu-usage transfer does not affect this lock, process, current deadline or model binding.

At the agreed receiving window, C closes only its own consumer pin through the existing external-manager flow; no ordinary prepare/unload or model-unit stop. Zero pins do not unload the model. D creates its own RemoteBaseClient.pin using ready_only/status/acquire and the full current status binding. D must not reuse C capability or substitute PID for fact.instance_id. GPU transfer does not transfer SDK ownership automatically.

Historical final-service-state.configuration.registry was hardcoded to registry-v6. The live unit selector points to registry-user-infer-20261103-v1.json, SHA256 daddbbda48469e286c4008c6fc1f5c50d8b28b570ef28d579552b0226af6ac73. Current bundle digest is c2c3cf7b8fdab120f7324e9fde552fcf72dd9fb89a0d5266f5754f48a9ccd683. Use the actual unit selector and full model binding; do not reload because of the stale report field. The future read-only probe now follows that selector; historical evidence remains intact.

D acknowledged the handoff window but is NOT ready to receive yet: its relay still requires B's coherent Raw snapshot, normal physical budgets and general0.5s source-segment policy. Therefore no resource transfer, C pin closure, service retirement or new D GPU record is performed now. C continues the existing GPU monitor until D reports successful transfer and monitoring takeover. Remaining: D candidate readiness, agreed receiving window, then C pin close / D transfer and new pin / monitor takeover.

## SDK receiving-window plan (2026-11-04, prepared only)

D has requested preparation, not execution. Exact plan: `/Users/wujie/Developer/worktrees/robot-workbench-shared-lifecycle/.evidence/arx6-sdk-handoff-20261104-v1/plan.json`.

After D explicitly declares the final candidate ready, C will fresh-check the old source/unit/runtime identities and absence of activity; normally stop the old C manager to isolate obsolete lifecycle buttons; keep the C business already stopped; use the existing tested Owner preview_workbench route to home at0.1rad/s while preserving confirmed81mm jaw targets; obtain a fresh review and submit once; verify READY and healthy stationary feedback; then use the original administrator maintenance capability for normal release. Only after matching close receipt, old native/Guard lifetime exit and empty CAN registrations may the released old Owner service be normally stopped and D admitted to its new Owner. Model and camera services stay running. Unknown results are queried by original receipt, never resubmitted. Any new fault or unresolved home/release stops the affected sequence.

No remote access, motion, release, process operation or GPU operation was performed to prepare this plan. C does not resume GPU ownership. The planned81mm targets must match the actual held commands at the receiving window, not noisy measured openings. The old webpage manual planner remains unconfigured; the plan reuses the already-tested independent Owner path instead of restarting C business or creating a planner.
