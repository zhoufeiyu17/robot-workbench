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

本次中间盘点以2026-10-11 14:09实际检查为基准，整体仍为 **NOT_CLOSED**。这是状态与证据整理，不是新增开发或验收轮次；已有scene-xy原driver继续，整理期间不新起GPU运行。14:09实际核验其双腕采集24/29、734自有进程健康，14:12头部与双腕成组安装已完成；Mac 18867仅本机SSH转发和当前fake API正常，**18866无监听、HTTP拒绝，退出原因尚未查明**。此前“18866未动”不代表该服务仍在线。来源：[中间盘点证据](.evidence/s2/interim-closeout-20261011-1409.json)，后续运行阶段见续接检查点。

目前实现包括共同四栏目、单Owner与显式交接、原C/D业务接入、结果绑定和诊断。26项旧能力对照与已批准的实现补齐已经完成；**这不等于26项所有浏览器和实际后端分支都完成了验收**。姿态显示预览在warm-145124指定profile达标：mesh关23.17–23.52FPS/P95≤173.5ms，开22.30–22.58FPS/P95≤183.7ms；该实测不外推其他主机或PhysX慢执行画面。封存导出优化已完成，见下表。

反复标定/数采来自两个不同目的：三个声明case分别从空库验证新设备无旧标定依赖；此前修复Owner能力声明、缓存可用性及反馈wall/source就绪差异后，正式产品/suite发生变化，因此v5/v6成功未混加到v7。当前固定产品与driver，三case各一次原始完整流程及九项独立审计通过即结束本轮冷启动验收。文档更新不改变产品源散列，不重跑；新故障先定点诊断、说明影响和必需重验范围，不自动增加无限轮次或降低已批准覆盖。

已推送产品 **757b290**，仍是隔离开发候选；**当前suite-v7完整冷启动2/3，真机未接入**。本轮device-j1与nominal已通过，scene-xy运行中。历史suite-v5的nominal和suite-v6的scene-xy不代替新版本资格。本节记录2026-10-11 13:50 CST事实节点；动态进度与资源见[续接检查点](.evidence/s2/active-task-checkpoint.json)及[机器验证摘要](migration/validation-summary.json)。

| 范围 | 已完成 | 当前边界 |
| --- | --- | --- |
| 旧能力与UI | b5ab340的26项对照及已批准UI范围完成CPU/实际浏览器验收 | 不是全部浏览器分支或真机资格 |
| 封存/导出性能 | af6cfa9完整三相机raw对照总576.222→309.245s；解码帧、PTS和MP4 SHA一致 | 限实测输入、主机/负载/存储；该项已完成 |
| 动态动作配方 | 对齐normal-v31，按当前模型/观测重算目标；固定输入GPU规划、warm及历史nominal实际全链通过 | 不复制S0旧q/标定；不能外推新场景 |
| 历史suite-v5 nominal | full-064034原工作流及独立9/9审计通过；头34＋腕29新采安装、3207命令/10692反馈、各3218 RGB-D、源任务成功与源录像/L3通过 | 资格归属96e7f71；当前suite-v7须重新验证 |
| 历史suite-v6 scene-xy | full-scene-xy-083941原工作流及独立9/9通过；头34＋腕29新采安装、3537命令/11792反馈、每相机3548 RGB-D、源任务与源录像/L3通过 | 资格归属96d3c7a；当前suite-v7须重新验证 |
| 当前CPU/L1 | 757b290回归791次执行，790通过/1旧fixture跳过；L1 27/27；源码及native/detector前后不变 | 软件stop P95 4.128ms/max4.660ms，不代表实体止动 |

normal-v31阶段为合并抬撤、连续移杯降高、插入、实际落杯；固定名义预算129/170/90/29条30Hz命令，超预算拒绝。开爪v3固定45条/1.5s，release hold为6条/0.2s，UI改速保留开爪时长；原2mm/0.5°及物理验收门不变。12轴目标速度2rad/s、目标加速度5rad/s²是规划/控制协议上限，不代表真机各关节实测极限或墙钟实时吞吐。

96e7f71消除共享Owner能力声明窗口并保留预览发布错误。随后scene-xy-080456头部完成，腕部预览已发布，但确认执行在旧UI缓存可用性门拒绝，未下发腕动作；原失败和终态保留，9个PID及双GPU清退，08:22:50释放。事后Owner健康不能证明失败时具体哪项缓存为false，混合缓存误拒绝路径有确定性CPU复现。

96d3c7a让标定预览/确认各执行一次只读新鲜状态和反馈检查，复用原监控采样与双SDK推进逻辑；保留0.5s/2s及Owner100ms、token/epoch/pedal门，不自动重试动作。8类live故障在下发前拒绝；停止/新计划撤销晚预览和页面锁不阻塞采样检查保留。完整首跑fixture缺初始化与一次子脚本环境遗漏日志均保留，最终一致环境检查通过。

[历史suite-v6](.evidence/s2/cold-suite-freeze-20261011-v6.json)的scene-xy-083941已完整通过，raw10655文件/3889404054字节，三源视频各3548帧/118.266s，最大PTS误差0.333334ms，沿用2ms门。14个自有PID/start ticks及双GPU已清退，10:06:07释放；清退前后native/raw/三源视频SHA一致。[独立审计](.evidence/s2/full-scene-xy-20261011-083941-audit.json)、[实际终帧](.evidence/s2/full-scene-xy-20261011-083941-final-cameras/head.jpg)和[源录像](.evidence/s2/full-scene-xy-20261011-083941-source-head.mp4)保留。

device-j1-100608头部34/34已解，但腕部确认因`host_sample_stale_or_clock_skew`在下发前拒绝，无腕部或D动作；原失败终态保留，9个PID及双GPU已清退，10:28:35释放。失败瞬间的精确年龄未记录，不能断言是超过0.5s还是负年龄；之后20次健康读取不能反证原失败。已证实页面按source时间判可执行，而读取器还要求wall年龄合格，二者就绪口径不一致。

757b290统一host采样年龄判据，在反馈过期或来自未来时同步禁用页面与runtime执行入口；状态查询不增加HTTP/SDK读取。前置检查失败会保存含原始时间戳、请求边界、年龄分支及进程/时钟身份的独立诊断文件。没有增加动作等待、重试、阈值放宽或时间戳改写。

[suite-v7](.evidence/s2/cold-suite-freeze-20261011-v7.json)冻结375项产品源及原三案例定义，并固定验收driver/auditor源哈希。driver只在页面primary与runtime都明确允许时提交一次确认，已提交动作失败仍终止，不重试；旧driver和旧验收记录保留。

full-device-j1-20261011-110144原始空库工作流及[独立9/9审计](.evidence/s2/full-device-j1-20261011-110144-audit.json)已通过：头34＋腕29新采安装，3215条命令、10718条反馈、三相机各3227帧RGB-D；同plant任务成功，raw、源录像与L3均通过。预声明左右J1±0.003rad偏置仅加入plant，真值未进入求解或规划输入。[实际终帧](.evidence/s2/full-device-j1-20261011-110144-final-cameras/head.jpg)、[源录像](.evidence/s2/full-device-j1-20261011-110144-source-head.mp4)及[原始终态](.evidence/s2/full-device-j1-20261011-110144-terminal.json)已保存。12:23:15实际核验14个PID及双GPU全空，清退前后5项制品SHA保持。资源表12:23:04提前登记释放是牵头执行顺序错误，旧结束行不可改；[纠正记录](.evidence/s2/full-device-j1-20261011-110144-resource-release-correction.json)与下一记录notes保留差异，无两轮GPU重叠，不把提前登记时间当作核验完成时间。

nominal full-20261011-122428原始空库工作流及[独立9/9审计](.evidence/s2/full-20261011-122428-audit.json)已通过：头34＋腕29新采安装，3234条命令、10782条反馈、三相机各3246帧RGB-D；同plant任务成功，raw、源录像与L3通过。[实际终帧](.evidence/s2/full-20261011-122428-final-cameras/head.jpg)和[源录像](.evidence/s2/full-20261011-122428-source-head.mp4)保留。13:46:27实际核验14个自有PID及双GPU全空，五项制品SHA在清退前后保持，之后13:46:45登记释放。

最后一例scene-xy full-scene-xy-20261011-134701已通过预检、dry-run、空库检查及头部预览/确认，13:50:02头部采集0/34；本例尚未资格。杯子位移[+0.01,-0.01,0]m，咖啡机位移[-0.01,+0.01,0]m，其余声明保持。734双GPU预留13:47:48–15:42:48，13:48:35登记启动；驻留截止15:37:54，随后清退，不延长，下一资源核验14:30。服务初始化时一次本机只读状态连接reset，服务就绪后原driver正常进入C，未构成动作失败。18867临时loopback转发仍在，18866 L1未动。

[当前CPU/L1](.evidence/s2/readiness-diagnostics-full-cpu-20261011-v1/receipt.json)、[历史nominal独立审计](.evidence/s2/full-20261011-064034-audit.json)、[nominal实际头相机终帧](.evidence/s2/full-20261011-064034-final-cameras/head.jpg)及[源录像](.evidence/s2/full-20261011-064034-source-head.mp4)保留。历史nominal清退14个PID后，native/raw/三源视频SHA一致；每路视频3218帧/107.266s，最大PTS误差0.333334ms，沿用2ms门。warm-050624复用C的8/8通过、所有旧失败及修复详见[行为记录](/Users/wujie/Developer/work/anyverse/_context/notes/unified-workbench-refactor-20261009/07-behavior-parity.md)。

浏览器自动化连续两次失败后已停止重试；实际相机/录像是画面内容证据，不能代替新浏览器验收。GPU使用表更新成功，自动shadow核账仍有已记录的非阻断问题，不宣称治理全绿。

## 剩余验收

1. 完成suite-v7最后一例scene-xy（运行中）新空库全链；device-j1与nominal已通过。继续核验原始终态、安装、实际任务、raw、源录像与辅助回放，不借用旧版本成功。
2. 交付稳定fake开发入口及运行生命周期：先查明18866离线状态、复用既有启动方式，明确18866协议fake和18867有时限完整fake的区别。当前临时验收入口不等于D7常驻部署，3/3也不替代日常可用性；中间整理不自动扩大GPU常驻成本。按[旧功能矩阵](.evidence/s2/parity-ui-20261010T170940Z/parity-matrix.json)定点补尚缺证据的人工队列、仅腕/分步采集、清空/单批重解、候选逐帧回放、到位统计和故障恢复路径，并保留不同速度实际执行与renderer生命周期的证据边界。已实现能力不重新列为未实现，不因此重跑整套标定数采；浏览器自动化连续两次失败的限制需明确，不伪造视觉通过。
3. 同步最终文档与验收证据，核验并清退自有资源；中断或诊断续跑不计完整冷启动通过。独立纠正数据生产继续由原任务负责，不并入本工作台待办。
4. 独立完成L4真机SDK、现场止动、时延、载荷与单writer正式切换；fake资格不能替代正式真机部署。当前没有待用户裁决的新行为决策。

正式安装对外路径仍为`/var/lib/anyverse/calibration/hardware/active/{head.json,wrist.json,manifest.json}`；fake使用隔离身份和安装根。真机支撑与逐批运动审阅门保留。

跨仓权威入口为[统一工作台设计入口](/Users/wujie/Developer/work/anyverse/_context/notes/unified-workbench-refactor-20261009/README.md)；06为运行时/时钟契约，07保留历史与当前证据。维护责任为牵头Session `01a0e689-ddf5-7410-98d5-743920bfa8f7`；实现、来源、run阶段/终态或资源变化时更新，运行快照不作永久状态。
