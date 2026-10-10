# Robot Workbench

统一标定与数采工作台，当前仍是隔离开发候选。核心求解、采集、规划和执行策略复用原 C/D 实现，新代码负责共同入口、单设备控制权与 fake 接入。尚未替换旧工作台或连接真机，不能标记为生产就绪或“95%完成”。

维护者：工作台牵头任务；2026-10-11。接口、来源、执行策略或资格变化时更新本入口和证据；历史通过不自动资格新版本。

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

## 当前资格与运行

已推送产品 **96e7f71**，仍是隔离开发候选；**完整冷启动0/3，真机未接入**。本节记录2026-10-11 06:41 CST事实节点；后续阶段、终态和资源以[续接检查点](.evidence/s2/active-task-checkpoint.json)、[机器验证摘要](migration/validation-summary.json)及所引原始证据核对。

| 范围 | 已完成 | 当前边界 |
| --- | --- | --- |
| 旧能力与UI | b5ab340的26项对照及已批准UI范围完成CPU/实际浏览器验收 | 不是全部浏览器分支或真机资格 |
| 封存/导出性能 | af6cfa9完整三相机raw对照总576.222→309.245s；解码帧、PTS和MP4 SHA一致 | 限实测输入、主机/负载/存储；该项已完成 |
| 动态动作配方 | 与仿真任务对齐normal-v31，按当前模型/观测重算目标；固定原输入GPU规划、warm实际执行通过 | 不复制S0旧q/标定；不能外推新场景 |
| 标定/安装 | full-040630新采头34＋腕29并成组安装，head image_passed=true，左右腕留出1.266434/1.070071px | 该run的D规划失败保留，不计完整冷案例 |
| warm全链 | warm-050624原workflow完成，独立8/8审计通过；3425命令、11418反馈、每相机3436 RGB-D；native task成功、源录像及辅助回放通过 | 只读复用040630标定，不能计空库通过 |
| 最终CPU/L1 | 96e7f71回归783次执行，782通过/1旧fixture跳过；L1 27/27；源码及native/detector前后不变 | 软件stop P95 4.122ms/max4.550ms，不代表实体止动 |

normal-v31动态阶段为合并抬撤、连续移杯降高、插入、实际落杯；固定名义预算129/170/90/29条30Hz命令，超预算拒绝。开爪v3固定45条/1.5s，release hold为6条/0.2s，UI改速保留开爪时长；原2mm/0.5°及物理验收门不变；12轴目标速度2rad/s、目标加速度5rad/s²是规划/控制协议上限，不代表各关节真机实测极限，L4尚未验收。

612408a修复实时预览反馈和正常tick竞争；c06ae64在源坐标计算凸面后变换半空间，未放宽几何阈值。96e7f71将共享Owner能力固定在snapshot边界发布，并持续显示标定预览发布错误，直到显式重生成成功；取消/新批次使旧回调失效。详见[行为记录](/Users/wujie/Developer/work/anyverse/_context/notes/unified-workbench-refactor-20261009/07-behavior-parity.md)。

full-060418已采头34/34并求解，腕预览发布失败且页面清掉错误；诊断重生成捕获“updated owner required”，CPU复现能力缓存窗口可触发同错，但不能证明现场调用一定落在该窗口。原driver已终止，没有成功终态，没有腕/D运动；9个PID退出、双GPU清空并释放。本失败不计通过。warm-050624也已清退14个PID并释放资源，原始产物校验保留。

新suite-v5冻结375项源和原三案例定义，full-064034已通过preflight/dry-run并启动新的nominal空库验证；未产生通过资格。734双GPU窗口06:41–08:36 CST，110分钟驻留加5分钟清退，不延长；以检查点核验实际进度与资源。18867临时loopback转发仍在，18866 L1未动。浏览器自动化连续两次失败后已停止重试；[warm实际头相机图像](.evidence/s2/warm-20261011-050624-final-cameras/head.jpg)与[源录像](.evidence/s2/warm-20261011-050624-source-head.mp4)是画面内容证据，不是新run浏览器验收。

[最终CPU/L1回执](.evidence/s2/capability-review-full-cpu-20261011-v1/receipt.json)、[warm独立审计](.evidence/s2/warm-20261011-050624-audit.json)和[三案例冻结](.evidence/s2/cold-suite-freeze-20261011-v5.json)分别保留资格范围。旧失败不改写为通过。GPU使用表更新成功，自动shadow核账仍有已记录的非阻断问题，不宣称治理全绿。

## 剩余验收

1. 以suite-v5新跑nominal、scene-xy、device-j1三个空库全链案例，逐例核验原始终态、安装、实际任务、raw、源录像与辅助回放。
2. 完成运行终态文档及自有资源清退；中断或诊断续跑不能计冷启动通过。
3. 独立完成L4真机SDK、现场止动、时延、载荷与单writer正式切换。

正式安装对外路径仍为`/var/lib/anyverse/calibration/hardware/active/{head.json,wrist.json,manifest.json}`；fake使用隔离身份和安装根。真机支撑与逐批运动审阅门保留。

跨仓权威入口为[统一工作台设计入口](/Users/wujie/Developer/work/anyverse/_context/notes/unified-workbench-refactor-20261009/README.md)；06为运行时/时钟契约，07保留历史与当前证据。维护责任为牵头Session `01a0e689-ddf5-7410-98d5-743920bfa8f7`；实现、来源、run阶段/终态或资源变化时更新，运行快照不作永久状态。
