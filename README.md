# Robot Workbench

统一标定与数采工作台，当前仍是隔离开发候选。核心求解、采集、规划和执行策略复用原 C/D 实现，新代码负责共同入口、单设备控制权与 fake 接入。尚未替换旧工作台或连接真机，不能标记为生产就绪或“95%完成”。

维护者：工作台牵头任务；2026-10-10。接口、来源、执行策略或资格变化时更新本入口和证据；历史通过不自动资格新版本。

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

## 驻留物理 fake 与当前资格

`scripts/serve_physx_device.py` 只用于有界、任务独占的GPU环境。运行前必须冻结本次代码/输入/资源/输出合同，通过preflight和dry-run，并登记实际GPU占用。服务只绑定loopback；不通过页面输入任意远端地址或fallback到其他实例。

功能代码基线为 **900673e**；后续状态文档提交不代表重新运行测试。本轮中间状态整理只更新文档与资源记录，不改变代码或抓取策略。新工作台仍为隔离候选，未替换旧真机入口。

| 范围 | 已有资格 | 尚缺资格 |
| --- | --- | --- |
| 统一页面、控制权与停止 | 四栏目已确认；HTTP/DOM、当前源码故障矩阵27/27通过 | 最新状态修复的实际DSW页面复验、完整旧能力对照、实体停止 |
| 头部/双腕标定与安装 | full-123915完成34头部＋29双腕采集、求解、成组安装；左/右腕留出1.417/1.051px | 最终版本的完整空库案例 |
| 模型衔接与数采运动 | 安装→新D模型/观测/规划已接通；历史warm-153813完整执行3692条指令 | 最新计划的原生任务成功与完整数据，不可用运动完成替代 |
| 图像与预览 | warm-145124固定1280×720/DPR1、每模式60秒×3：mesh关23.17–23.52FPS/P95 169.7–173.5ms，开22.30–22.58FPS/P95 180.8–183.7ms；8267派发全呈现 | 不外推所有主机/负载 |
| 持续录制与回放 | warm-173414实际首帧覆盖通过；随后位置、提前租约、L3派生、IO与状态修复均有CPU证据 | 最新修复的实际完整raw、原生任务、实际相机及L3完整回放 |

**完整冷启动验收仍0/3。** 这指三个预声明案例都必须完成空库标定→安装→数采任务→完整数据与回放；不表示标定子流程未成功。warm明确只读复用旧C来源，创建新plant/Owner/时钟和D观测，不安装当前C store，fresh_C_completed=false，永不计入空库案例；truth仅用于准入，不进入求解或D规划。

最新修复分组：

- 4c8c740：fake 0.005rad／physical 0.01rad位置边界、转换和编码容差；审阅前合法化并记录原值/修正，L3仅独立派生初始化前缀。其他阈值保持。
- 7d1a5dd：fake确认后最多1wall秒准备本批相机租约，再进入原0.3source秒静止核验；准备失败仍执行，数据标无效；取消与续租保留执行身份。
- a8665bc／403ee29／900673e：接收请求即发布busy与运行身份、隔离旧刷新/审批、原生事件正确使用run_status，以及64条事件公平写入与fake尾帧归档。这三组尚未新DSW全链复验。

当前CPU完整日志755次unittest执行（非去重测试总数），751过、4skip；其中C479次（475过、4skip），后续A1补测补足3个缺失项，旧模型规划fixture仍未测。完整L1 27/27、320个Python指纹与当前匹配，软件停止屏障P95 2.289ms/max3.095ms；不代表真机止动。

warm-173414失败已保存：首帧分别早于首命令0.410/0.377/0.343源秒，但持续raw队列溢出。原生事件字段错误掩盖的接触高度越界已由精确源码与失败帧重建；计划胶垫下缘仅约0.47mm余量，标定映射残差约1.27mm。新增至少2mm抓取高度规划余量是**待对齐的新策略，未实施**；原35–100mm/0.1mm接触验收不变。不是把整条轨迹直接抬高2mm，也不是把原失败改成通过。

该run已清退并释放资源，中间状态整理再次核验旧PID均不存活、两卡核验时为空、原始产物仍在。18867临时隧道已关闭；18866保留本机L1协议fake，无GPU相机或后台自动任务。详细当前字段与历史证据分开放在 [validation-summary.json](migration/validation-summary.json)。

真机仍须单独核验SDK、增益/踏板、实际止动和设备侧时延，并遵守现场支撑/运动审阅门。正式安装对外路径保持 `/var/lib/anyverse/calibration/hardware/active/{head.json,wrist.json,manifest.json}`；fake使用隔离的身份与安装根。

## 剩余验收

1. 对齐是否在本轮修改抓取姿态选择策略；2mm是建议的待验证初值。这个决策影响完整任务验收，不否定或阻塞已独立完成的工程修复。
2. 新冻结版本完成实际fake持续raw、原生任务与两类回放，再完成三个预声明空库全链case及旧能力在新入口的对照；warm不能代替。
3. 后续真机SDK、现场止动、时延与载荷资格，以及按设备单writer切换仍需独立验收。

跨仓设计和用户决策的本地权威入口为 Anyverse `_context/notes/unified-workbench-refactor-20261009/README.md`；其中06记录时钟/安装契约，07记录行为对照、故障归因及当前资格。
