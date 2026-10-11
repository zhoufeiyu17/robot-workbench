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

已推送产品 **96d3c7a**，仍是隔离开发候选；**当前suite-v6完整冷启动1/3，真机未接入**。历史suite-v5的nominal已完整通过，但不代替新版本资格。本节记录2026-10-11 10:08 CST事实节点；动态进度与资源见[续接检查点](.evidence/s2/active-task-checkpoint.json)及[机器验证摘要](migration/validation-summary.json)。

| 范围 | 已完成 | 当前边界 |
| --- | --- | --- |
| 旧能力与UI | b5ab340的26项对照及已批准UI范围完成CPU/实际浏览器验收 | 不是全部浏览器分支或真机资格 |
| 封存/导出性能 | af6cfa9完整三相机raw对照总576.222→309.245s；解码帧、PTS和MP4 SHA一致 | 限实测输入、主机/负载/存储；该项已完成 |
| 动态动作配方 | 对齐normal-v31，按当前模型/观测重算目标；固定输入GPU规划、warm及历史nominal实际全链通过 | 不复制S0旧q/标定；不能外推新场景 |
| 历史suite-v5 nominal | full-064034原工作流及独立9/9审计通过；头34＋腕29新采安装、3207命令/10692反馈、各3218 RGB-D、源任务成功与源录像/L3通过 | 资格归属96e7f71；当前suite-v6须重新验证 |
| 当前suite-v6 scene-xy | full-scene-xy-083941原工作流及独立9/9通过；头34＋腕29新采安装、3537命令/11792反馈、每相机3548 RGB-D、源任务与源录像/L3通过 | 仅位置变化案例通过；device-j1/nominal尚待当前版本验收 |
| 当前CPU/L1 | 96d3c7a回归785次执行，784通过/1旧fixture跳过；L1 27/27；源码及native/detector前后不变 | 软件stop P95 4.229ms/max4.264ms，不代表实体止动 |

normal-v31阶段为合并抬撤、连续移杯降高、插入、实际落杯；固定名义预算129/170/90/29条30Hz命令，超预算拒绝。开爪v3固定45条/1.5s，release hold为6条/0.2s，UI改速保留开爪时长；原2mm/0.5°及物理验收门不变。12轴目标速度2rad/s、目标加速度5rad/s²是规划/控制协议上限，不代表真机各关节实测极限或墙钟实时吞吐。

96e7f71消除共享Owner能力声明窗口并保留预览发布错误。随后scene-xy-080456头部完成，腕部预览已发布，但确认执行在旧UI缓存可用性门拒绝，未下发腕动作；原失败和终态保留，9个PID及双GPU清退，08:22:50释放。事后Owner健康不能证明失败时具体哪项缓存为false，混合缓存误拒绝路径有确定性CPU复现。

96d3c7a让标定预览/确认各执行一次只读新鲜状态和反馈检查，复用原监控采样与双SDK推进逻辑；保留0.5s/2s及Owner100ms、token/epoch/pedal门，不自动重试动作。8类live故障在下发前拒绝；停止/新计划撤销晚预览和页面锁不阻塞采样检查保留。完整首跑fixture缺初始化与一次子脚本环境遗漏日志均保留，最终一致环境检查通过。

[suite-v6](.evidence/s2/cold-suite-freeze-20261011-v6.json)冻结375项源及原三案例定义。scene-xy-083941已完整通过，raw10655文件/3889404054字节，三源视频各3548帧/118.266s，最大PTS误差0.333334ms，沿用2ms门。14个自有PID/start ticks及双GPU已清退，10:06:07释放；清退前后native/raw/三源视频SHA一致。[独立审计](.evidence/s2/full-scene-xy-20261011-083941-audit.json)、[实际终帧](.evidence/s2/full-scene-xy-20261011-083941-final-cameras/head.jpg)和[源录像](.evidence/s2/full-scene-xy-20261011-083941-source-head.mp4)保留。

新full-device-j1-20261011-100608已通过预检、原空库检查和头部预览/确认，正在头部采集；尚无本例完整C/D资格。它只在plant加入预声明左右J1±0.003rad偏置，真值不进入求解或规划输入。734双GPU预留10:06:54–12:01:54，10:07:34登记启动；驻留截止11:57:01，随后清退，不延长，下一资源核验10:55。18867临时loopback转发仍在，18866 L1未动。

[当前CPU/L1](.evidence/s2/fresh-control-full-cpu-20261011-v2/receipt.json)、[历史nominal独立审计](.evidence/s2/full-20261011-064034-audit.json)、[nominal实际头相机终帧](.evidence/s2/full-20261011-064034-final-cameras/head.jpg)及[源录像](.evidence/s2/full-20261011-064034-source-head.mp4)保留。历史nominal清退14个PID后，native/raw/三源视频SHA一致；每路视频3218帧/107.266s，最大PTS误差0.333334ms，沿用2ms门。warm-050624复用C的8/8通过、所有旧失败及修复详见[行为记录](/Users/wujie/Developer/work/anyverse/_context/notes/unified-workbench-refactor-20261009/07-behavior-parity.md)。

浏览器自动化连续两次失败后已停止重试；实际相机/录像是画面内容证据，不能代替新浏览器验收。GPU使用表更新成功，自动shadow核账仍有已记录的非阻断问题，不宣称治理全绿。

## 剩余验收

1. 完成suite-v6剩余device-j1（运行中）和nominal新空库全链；scene-xy已通过。继续核验原始终态、安装、实际任务、raw、源录像与辅助回放，不借用旧版本成功。
2. 同步最终文档与验收证据，核验并清退自有资源；中断或诊断续跑不计完整冷启动通过。
3. 独立完成L4真机SDK、现场止动、时延、载荷与单writer正式切换。

正式安装对外路径仍为`/var/lib/anyverse/calibration/hardware/active/{head.json,wrist.json,manifest.json}`；fake使用隔离身份和安装根。真机支撑与逐批运动审阅门保留。

跨仓权威入口为[统一工作台设计入口](/Users/wujie/Developer/work/anyverse/_context/notes/unified-workbench-refactor-20261009/README.md)；06为运行时/时钟契约，07保留历史与当前证据。维护责任为牵头Session `01a0e689-ddf5-7410-98d5-743920bfa8f7`；实现、来源、run阶段/终态或资源变化时更新，运行快照不作永久状态。
