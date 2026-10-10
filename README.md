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

- S0已验证同Owner→PhysX实测响应、同场景RGB-D、停止/失联保护及连续物理步反馈更新，历史摘要见 [validation-summary.json](migration/validation-summary.json)。
- 最新完整L1故障矩阵绑定399d007，27/27通过；30个正常SDK样本的HTTP请求起点→软件停止屏障P95为1.019ms、最大1.190ms。仅资格本Mac墙钟控制流，不外推实体止动或后续产品增量。后续HTTP限额与观测时钟修复分别有回归和实际图像证据。
- L2历史失败原样保留：短测在双腕22/29时达到1100s自设期限；`full-20261010-033728`头解失败。后者经独立冷启动A/B定位为新scene贴纸共面，0.1mm打印面分离修复不改变求解器或验收阈值。
- `full-20261010-045936`已完成34张头部、29张双腕的空库标定并安装：头部留出贴纸RMS1.086px、mesh3.255px/92.625%，左/右腕RMS1.352/1.108px；仅安装于该fake run。D自动准备因跨Python运行时环境串扰失败，后续隔离环境的实际GPU转换及URDF/USD/rig/binding核验通过；尚无完整D执行/raw/回放资格。该run进程已实际清退、GPU1已释放。
- `77f31cb`已修复数采准备阶段/失败重试、旧审批撤销及服务重启页面恢复；同源码L1矩阵27/27通过。`full-20261010-061606`也完整标定并安装（腕部1.291/1.041px），转换通过后D初始化发现IPC目录清单混入file pins；8ac2b48已修复。该run/诊断已清退释放，尚无D运动/raw资格。
- 既有detector配置曾引用原实例/tmp release；已与原D任务核对，在隔离CPFS形成完全相同digest的持久包，并将真实CPU闭包校验前置到GPU/C之前。full-071034完成C安装、workers ready后，首次观测因新Runtime漏派生physical_feedback失败；399d007现复用原CollectionOwner缓存字段，不进SDK/recorder锁。原run与失败均保留、已清退。
- full-080258冻结399d007：34头+29腕再次通过并安装，新D profile/URDF/USD/rig与安装绑定独立核验通过。但首观测混用30墙钟秒超时和8仿真源秒覆盖要求，未到D规划/执行/raw/replay；该run完整L2失败，08:46核验全部自有进程和GPU1清空、08:47释放。
- 7e7b74a按源时间完成fake观测，保留9组/8秒要求、30墙钟秒无进展保护及固定resident清退余量；physical原30秒语义不变。独立候选读取同一plant实际RGB-D，在33.304墙钟秒取得9组/8.133源秒并通过原validator，无控制或新GPU进程，不能追溯资格完整D。049ce80另恢复原D接口16MiB计划请求范围。最终全CPU检查exit0（C469执行/4skip，显式A1/release fixture另验），最新详情见validation-summary。
- 用户已人工确认四栏目布局；随后浏览器验证了协议fake单姿态审批/执行，以及切栏不交接、显式交接保留保持。完整业务界面与目标预览性能仍待验收。

真机仍须单独核验SDK、增益/踏板、实际止动和设备侧时延，并遵守现场支撑/运动审阅门。正式安装对外路径保持 `/var/lib/anyverse/calibration/hardware/active/{head.json,wrist.json,manifest.json}`；fake使用隔离的身份与安装根。

## 剩余验收

1. 完成旧能力通过新入口的实际行为验收；仅腕重做、批次重算、质量详情、回放、目标/mesh及恢复入口已接入，实际GPU/完整浏览器资格仍待核验。
2. 在同一驻留plant完成空库标定→安装→新规划模型→新观测→显式交接→数采→完整RGB-D/raw→回放；三预声明正常case仍未通过。
3. 测量目标预览/mesh及实际浏览器交互预算，完成设备切换准备；不能用非实时PhysX wall FPS替代姿态显示预览性能。

跨仓设计和用户决策的本地权威入口为 Anyverse `_context/notes/unified-workbench-refactor-20261009/README.md`；其中06记录时钟/安装契约，07记录行为对照、故障归因及当前资格。
