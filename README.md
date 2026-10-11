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

## 当前状态与验收

2026-10-11：已按用户要求完成中间状态梳理，并完成同一冻结版本下的 **3/3 新空库完整 fake 验收，每例独立9/9通过**。本组冷启动验收到此结束。整体重构仍为 **NOT_CLOSED**：稳定完整fake日常入口、部分专项交互/恢复资格及正式真机部署仍未完成，不能标为生产就绪或“95%完成”。

本轮冻结产品为 **757b290c12dde26732474cb58cf99b5a62cbc914**，375项源码与验收driver固定不变。当前代码、数据和证据的机器入口是[验证摘要](migration/validation-summary.json)，三例联合核验为[总回执](.evidence/s2/cold-suite-v7-qualified-20261011.json)。动态运行与资源见[续接检查点](.evidence/s2/active-task-checkpoint.json)。

| 已完成范围 | 证据与边界 |
| --- | --- |
| 共同入口与控制 | 四栏目、单Owner、显式交接、原C/D业务、结果绑定和诊断已实现；26项旧功能对照与已批准实现补齐完成，不等于每项全部浏览器/实际后端分支已验收 |
| 当前CPU/L1 | 791次测试执行：790通过、1旧fixture跳过；L1故障矩阵27/27；软件stop P95 4.128ms/max4.660ms，仅本机软件资格 |
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

## 当前入口、资源与重验口径

2026-10-11 18:00：已恢复完整fake后端 `dev-20261011-175603`，控制READY/已连接、三实际相机、姿态预览、已安装头解mesh均通过API与实际JPEG核验；D正在初始化，尚未把这一阶段写成D就绪。[恢复回执](.evidence/s2/dev-20261011-175603-availability/receipt.json)。本轮无新标定或数采轨迹。

- **18866**：暂仍是已连接的本机协议fake。完整fake位于734本机43759；申请将18866切换为仅本机SSH转发，尚待用户批准。最新路由状态查[检查点](.evidence/s2/active-task-checkpoint.json)，不把后端健康冒充浏览器已切换。
- **18867**：原临时验收转发已关闭，未复开。
- **734 GPU**：当前run独占双卡；计算GPU1，GPU0为Isaac图形上下文。服务截止约21:57 CST，资源登记至22:02用于清退。`fake` heartbeat每45分钟真实核验；不自动延长或重启。CPU协议入口数据保留，未接真机。
- **来源**：复用qualified nominal `full-20261011-122428`，保留头v000001/腕v000002、34+29张历史采样和71个到位样本，原source只读，当前独立复制。新Owner/clock/凭据，旧审批和动作不恢复。
- **修复**：驻留寿命与D单任务预算分开，D仍最多120分钟；显示镜像接受既有−2～85mm反馈容差、记录原值与显示修正，显示模型限制在0～83mm，用户目标范围不变。最初174752启动尝试保留为失败诊断，9个自有PID与双GPU核空后已释放。
- **历史与纠错**：3/3冷验收仍绑定757b290/suite-v7，不重新运行，也不当作本次新增恢复功能资格。device-j1提前登记release的纠错及治理非阻断一致性警告继续保留。

### 完整fake复用启动

在持有原run的主机，用 `scripts/calibration_resume_source.py --source-run <原run绝对路径> --source-base <原base-manifest.json绝对路径>` 生成只读SHA规格；将该JSON作为 `serve_physx_device.py --calibration-resume-source <规格JSON>` 的独立参数，配合正常base/nominal/新output和有界collection合同启动。它不是warm诊断模式，正常C/D操作保留；`--dry-run`会验证同虚拟设备、模型、相机、完整文件和成组依赖。不得把来源规格中的plant/truth核验材料传给求解与规划。

导入图像、结果和统计各自独立复制；原版本ID、原采样时钟和原事件身份不改。新采样使用新时钟；精确批次散列支持旧头＋新腕及重新求解。恢复不计空库验收。启动代码与76项定向CPU检查及两组完整CPU配置链见[定向回执](.evidence/s2/dev-resume-targeted-cpu-20261011.json)；实际浏览器切换、D就绪和之后生命周期以检查点的实测时间为准。

反复执行来自三case空库要求，以及此前共享Owner能力、缓存和wall/source就绪差异修复后的产品换版。当前3/3已达到有限退出条件，不再继续追加正常case。文档更新不影响运行源码；后续变更先判断受影响的证据和最小重验范围，不盲目把整套结果归零，也不降低已批准覆盖。

日常开发优先复用有效fake标定与匹配证据；只有要证明无旧标定依赖的场景才使用空库验收。当前完整验收采用30%执行档位，PhysX源时间与墙钟分开；30Hz采样不代表墙钟30FPS。normal-v31仍按当前模型/观测动态生成目标，不导入S0旧q/标定；129/170/90/29条名义预算、开爪v3的45条/1.5s、6条/0.2s保持及原物理门不变。2rad/s、5rad/s²是软件目标上限，不是真机关节实测极限。

## 剩余工作（按优先级）

1. **完整fake入口与生命周期交付**：恢复代码与后端图像核验已完成，优先完成D初始化、已申请的18866本机转发及现场页面确认；之后补用户自助启停/重启交互和实际故障恢复。当前有界服务不等于无限常驻或D7全部完成。
2. **定点补验已有功能**：完整人工队列、仅腕/分步操作、清空/单批重解、逐相机/逐帧/低质量候选/下载、不同速度实际执行、故障后的显式恢复和重新审阅。C12与预动作恢复CPU已补，不重列待办；实际历史head mesh、50%审阅和30%执行只覆盖各自子范围。浏览器自动化此前连续失败，需按已有工具边界补可执行的人工/浏览器证据，不伪造视觉通过。
3. **正式真机部署与L4验收**：SDK/支撑与停止、时延/载荷、真实相机及单writer切换独立完成；fake结果不替代现场资格。当前没有待用户裁决的新算法或UI行为决定。

已完成的UI对照实现、封存导出优化和3/3空库验收不再回到开发待办。独立纠正数据生产由原任务负责，不并入本工作台范围。

正式安装对外路径仍为`/var/lib/anyverse/calibration/hardware/active/{head.json,wrist.json,manifest.json}`；fake使用隔离身份和安装根。真机支撑与逐批运动审阅门保留。

跨仓权威入口为[统一工作台设计入口](/Users/wujie/Developer/work/anyverse/_context/notes/unified-workbench-refactor-20261009/README.md)；06为运行时/时钟契约，07保留历史与当前证据。维护责任为牵头Session `01a0e689-ddf5-7410-98d5-743920bfa8f7`；实现、来源、run阶段/终态或资源变化时更新，运行快照不作永久状态。
