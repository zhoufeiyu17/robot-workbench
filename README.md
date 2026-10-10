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

`scripts/serve_physx_device.py`用于有界独占GPU环境；先冻结代码/输入/输出/资源合同，通过preflight/dry-run并登记GPU占用，服务仅绑定loopback。

当前产品代码 **e67cf7e**，已完成本轮抓取余量实现及一次实际warm数采验收。warm-20261010-223638原workflow因视频时间戳累计取整而导出失败，清退后终态stopped；运动、原生任务和完整raw通过，三路源录像在新CPU run中修复后独立恢复。历史失败保留。**完整冷启动仍0/3，真机未接入，新工作台仍为隔离开发候选。** 状态复核：2026-10-11T00:08:30.522492+08:00。

| 范围 | 已有证据 | 尚缺资格 |
| --- | --- | --- |
| 统一入口/控制权/停止 | 四栏目、实际GPU页面实时指令计数与分项终态；最终L1 27/27 | 完整旧能力对照、实体止动 |
| 标定/安装 | full-123915已34头部＋29双腕，求解并成组安装；腕部只看贴纸，左/右留出1.417/1.051px | 最终版本三个空库全链案例 |
| 抓取/放杯/回零 | 3669/3669命令，原生任务success，29355次观测，对齐误差1.7905mm | 其他预声明场景与真机资格 |
| 持续数据/回放 | raw3669命令、12232反馈、各3681帧；独立完整性通过，L3完整；新CPU源视频各122.700s | 同一新版本一次端到端完成；封存/导出性能整理 |
| 预览 | warm-145124限定1280×720/DPR1、每模式60秒×3，mesh关23.17–23.52FPS/P95 169.7–173.5ms；开22.30–22.58FPS/P95 180.8–183.7ms | 不外推所有主机/负载 |

抓取策略：在既有姿态域选择潜在胶垫接触区域37–98mm，至少2mm余量；本轮选中43.790–85.625mm。原生35–100mm/0.1mm验收、0.016m深度不变，新姿态重新绑定几何与闭爪证明；不整体抬高轨迹，不用真实plant或实测1.265mm残差调参。93a5a01只保留实际取得的释放窗口，后续接触/力/稳定/分离和最终回零/接液仍逐步检查。

f14daf7要求合同显式声明独占快盘camera_spool_root，正式raw/视频/manifest仍在CPFS。e67cf7e按绝对时间偏移量化concat时间，原2ms视频门槛未改。恢复run replay-20261011-000106只用CPU，三路各3681帧，单帧PTS误差最大0.333334ms；原failed run未改成passed。warm复用旧C、不写当前安装库，fresh_C_completed=false，永不计入空库验收。

最终CPU：[验证摘要](migration/validation-summary.json)，760次执行759通过/1旧模型fixture跳过，371源码与native/detector未变；L1 27/27，软件stop P95 2.248ms/max3.592ms，仅本机软件证据。实际原始产物独立审计、源录像恢复和清理回执分别为.evidence/s2/warm-223638-offline-archive-audit-20261011-001.json、replay-000106-recovery.json、warm-223638-spool-cleanup.json。旧失败与中断预算尝试见行为记录，未覆盖。

GPU驻留已退出：10/10 23:47:54核验14个PID/start_ticks退出，两GPU C/G空；23:49:40记录release。18867临时转发关闭，18866保留本机L1协议fake。只清理本run已核实晚于完整episode的额外spool及可重建PNG，CPFS原始产物和新恢复视频保留。本轮没有连接真机。

## 剩余验收

1. 完成最终版本三个预声明空库全链案例；warm复用旧标定，不能计入0/3。
2. 补完新入口的旧功能对照及导出/封存性能整理。
3. 单独完成真机SDK、现场止动、时延、载荷与单writer正式切换。

正式安装对外路径仍为`/var/lib/anyverse/calibration/hardware/active/{head.json,wrist.json,manifest.json}`；fake隔离身份和安装根。真机支撑与逐批运动审阅门保留。

跨仓权威入口为Anyverse `_context/notes/unified-workbench-refactor-20261009/README.md`；06为运行时/时钟契约，07保留详细故障、修复与证据；[validation-summary.json](migration/validation-summary.json)为机器可读状态。
