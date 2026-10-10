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

- 当前功能分支已推送到 **7d1a5dd**；仍为隔离候选，没有接管真机。S0 同 Owner→PhysX 实测反馈与三相机因果检查通过；完整冷启动验收仍 **0/3**。
- full-20261010-123915 完成34头部、29双腕采集/求解并成组安装，左/右腕留出1.417/1.051px；随后新D模型/观测/检测/scene通过，规划在杯子参考URDF绑定失败。历史失败、修复和各run资格见 [validation-summary.json](migration/validation-summary.json)。
- 最近 warm-20261010-153813 的新D观测、规划、改速与 Owner 3692条指令完成，但完整raw、原生任务验收及L3均失败。杯子材料发布、录制新鲜度水位及首帧准备、L3初始化转换问题已有修复；局部完成不等于全链通过。
- 新修复：4c8c740统一 fake 0.005rad／physical 0.01rad 的位置边界、转换和编码容差，在审阅前归一化并保留原值/修正。D按模型与Owner合法域的交集编码；L3仅显式派生初始化前缀。7d1a5dd在fake确认后最多1墙钟秒准备本批相机保存租约，再进入原0.3源秒静止核验；准备失败仍执行、数据标无效，取消及续租保留执行身份。静止、到位、跟踪、夹爪和碰撞口径未随位置噪声调整。
- 新功能完整CPU检查exit0，C共479项（4项跳过），显式A1补充4项通过，旧模型规划fixture未跑。最终录制准备9项、续租/时钟14项及相关专项通过；L1故障矩阵 **27/27**、source_unchanged=true，停止软件屏障P95 1.233ms/max1.252ms，仅资格本Mac软件流程。
- 预览已在 warm-145124、1280×720/DPR1、每模式60秒×3实际通过：mesh关闭23.17–23.52FPS/P95 169.7–173.5ms；开启22.30–22.58FPS/P95 180.8–183.7ms；8267次派发全呈现，最大635ms保留。资格限该host/profile，原始证据已SHA归档，不能外推其他忙负载或真机。
- warm诊断固定输入一致时只读复用已验C结果，保留旧身份，创建新的plant/Owner/时钟与D计划；fresh_C_completed=false，不导入当前C store、不运行安装，永不计入空库验收。truth准入信息不进入D输入图。
- 当前新运行 warm-20261010-173414 冻结7d1a5dd，已通过依赖SHA与dry-run并开始准备新D环境，尚无新运动/raw/回放资格。734两卡为本任务独占组，计算GPU1、GPU0仅Isaac图形上下文；实际占用以统一GPU表和run为准。
- 用户已确认四栏目布局。18866仅L1；已验证边界容差内的人工预览可生成并可撤销，未在该UI测试执行。18867为已批准的临时本机转发，只在有界完整fake运行期间可用。图像/实时叠加资格与完整业务资格分别记录。

真机仍须单独核验SDK、增益/踏板、实际止动和设备侧时延，并遵守现场支撑/运动审阅门。正式安装对外路径保持 `/var/lib/anyverse/calibration/hardware/active/{head.json,wrist.json,manifest.json}`；fake使用隔离的身份与安装根。

## 剩余验收

1. 验证当前修复版同plant D执行、原生任务结果、首帧与完整RGB-D/raw、实际相机回放及L3辅助回放，分别给出结果。
2. 完成三个预声明的空库标定→安装→新模型/新观测→交接→数采全链case及旧能力在新入口的行为对照；warm不能代替。
3. 后续真机SDK、现场止动、时延与载荷资格，以及按设备单writer切换仍需独立验收。

跨仓设计和用户决策的本地权威入口为 Anyverse `_context/notes/unified-workbench-refactor-20261009/README.md`；其中06记录时钟/安装契约，07记录行为对照、故障归因及当前资格。
