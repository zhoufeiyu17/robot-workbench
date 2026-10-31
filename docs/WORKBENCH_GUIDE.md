# Robot Workbench 使用与部署说明

维护者：工作台控制／部署任务（W2，Session `01a16327-7545-7332-a361-0123ddad811d`）。
交接验收：2026-10-31 03:45 CST；最近只读观测：2026-10-31 10:36 CST，ARX13。机器、发布版本、接口、启动方式或验收结果变化时更新。
本文说明当前实际部署；开发工作树与旧 README 的历史状态不代表线上版本。
实时状态以页面及接口为准，发布与验收索引见文末。

**03:45交接验收通过：工作台环境已退出，原人工数采交接时已启动且硬件就绪。**
独立运维页保留；SDK、Owner、业务、工作台相机、模型和 Triton CPU 均已退出。
原数采任务1251“咖啡打包”，交接验收没有录制或上传。
**后续09:32踏板USB两次断连／重新识别，原数采因踏板进程退出而停止；10:36只读观测仍未恢复。**
当前原数采认领仍有效、硬件进程已退出，工作台保持退出，未自动重启。先现场检查踏板USB再恢复；
内核`EMI?`是诊断提示，物理原因尚未确定。使用前核对实时状态，不要仅为查看页面而运行启动脚本。

## 1. 从哪里打开、怎样启动

- 工作台：<http://10.17.32.3:18868/?environment=physical&business=research>
- 独立运维页：<http://10.17.32.3:18868/operations?environment=physical>
- 主机：`wujie@10.17.32.3`（ARX13）。同一内网可直接访问，无需 Mac 转发。

在 **ARX13 本机**执行：

```sh
cd /home/wujie/work/feiyu.zhou/arx13-control-20261029-v1
./start-workbench.sh --check  # 检查版本、配置和运行前置
./start-workbench.sh          # 启动／复用工作台服务
```

默认启动会准备已登记的服务，**同步启动模型 infer 服务并完成模型加载／预热**；
模型为 Base566 epoch008／step67408，按当前发布配置选择 Torch 或 Triton；本次退出前选择为 Triton。
历史首次部署默认 Torch，不能据此推断后续选择。
**不新接 SDK、不下发机器人动作、不清故障锁**。已就绪时复用服务、模型 worker 和 SDK 身份。
输出中的 `integrity_passed`、`startup_ready`、`services_ready`、`model_ready`、`control_ready`
分别说明文件完整性、启动前置、服务、模型及设备控制条件。本版启动成功要求 `model_ready=true`；
模型 ready 不代表本机控制映射已通过，实际 Run 仍独立检查。

数采尚占用时，优先在运维页使用 **“清理现有控制并接入”**。仅需将服务从数采交给工作台、
暂不接 SDK 时，可显式执行：

```sh
./start-workbench.sh --handoff-collection
```

该选项调用已登记数采的正常停止接口（其流程可能回零），核实硬件与网页进程退出后启动服务；
不删除数据、不强杀、不修改数采环境。保存是否成功看数采自己的保存回执。

## 2. 接入、动作、停止和交接

**“清理现有控制并接入”是一项受核验的接管流程：**

1. 若工作台已经连接，先 Stop，撤销旧任务，等待保持返回与健康静止反馈；健康 SDK 可直接复用。
2. 若已登记数采仍运行，调用其正常停止接口，等待 `safe_ok`、网页及子进程退出，再接入工作台。
3. 必要时走既有中断恢复／重接入口；需要撤掉保持时，页面展示相应支撑确认。
4. 返回当前实例与核验结果，**不会续跑旧动作**。归属冲突、停止未确认或真实故障会明确拒绝。

它覆盖已登记的工作台和数采进程，不会强杀任意外部控制器，因此不能保证在断网、设备离线或
未知进程占用时仍能接入。不要同时从原数采启动入口另开控制器。

日常操作：

1. 选择 Physical，核对设备 ARX13、反馈、相机及当前控制权。
2. 显式“交接控制权”到需要的栏目；**切换栏目只改变页面，不改变持权者**。
3. 人工规划拖动／填写目标只形成草稿与预览；点击执行，预检通过后审阅并确认动作。
4. 研究动作同样是“保存配置 → 准备 → 审阅执行”。修改参数后须重新准备，不自动套用旧审阅。
5. 页面 Stop 走独立 Owner 通道，取消未来指令并请求当前姿态保持。停止不等于释放 SDK。
6. 中断恢复只恢复符合条件的控制状态；旧动作不续跑，下一次动作须重新准备。

### 一键交回人工数采

1. 打开独立运维页，在“接入与控制”中点击 **“一键退出工作台环境”**，审阅后确认一次。
2. 等待 **“工作台环境已退出”**：按钮依次 Stop、通过正常规划以0.1 rad/s回零（保留夹爪开度）、
   正常释放SDK、停止工作台模型／相机／Triton CPU／业务／Owner，确认进程退出后恢复原数采网页。
3. 在 **ARX13 本机**打开原数采页 `http://127.0.0.1:18980/`，沿用原任务，点击
   “启动硬件并准备采集”。看到“硬件就绪，等待踏板开始”和三路 LIVE 后再开始人工示教。
   该地址只监听ARX13本机；从其他电脑不要直接把127.0.0.1当作ARX地址，沿用原有数采入口。

退出按钮只恢复数采网页，**不自动启动数采硬件、不录制、不上传**。这次验收在按钮成功后，
另外调用原数采启动入口一次，已实际到达ready。原数采代码、环境、配置和既有数据均未改动。
独立管理页保留，用来查看回执；它不占用机器人、相机或GPU，也不会自动重接。
退出后业务已停，控机心跳／Runtime栏可能显示未知；以“环境已退出”的成功回执和各资源已停止为准，
不要因此点击中断恢复。关闭浏览器或只点Stop不能替代环境退出。

回零、释放或任一资源退出未确认时，流程停在具体失败阶段，**不会继续恢复数采或强杀进程**。
先查看同一操作ID的完整回执，不因页面超时重复提交。重新交回工作台时，先结束人工数采操作，
再用“清理现有控制并接入”，或仅需服务时执行 `start-workbench.sh --handoff-collection`。

### ARX13 踏板 3

第三键实测为 **Linux `KEY_C`，键码 46**，Owner 与业务配置均已绑定。第一次按下触发软件停止；
既有逻辑中，松开后再次按下可能进入手动接管，不能把连续踩踏理解成反复 Stop。
它不是硬件断电急停。已完成键码实测和运行时绑定检查；**SDK 接入后的现场踩踏 Stop 验收待补**，
用户本轮暂时无法操作。软件 Stop 的实机验收已完成。

## 3. 功能和当前可用范围

| 模块 | 功能 | 本版实际边界 |
|---|---|---|
| 共用控制与人工规划 | 12 关节、夹爪目标、预览、动作审阅、交接、Stop | 已真实控机；角度显示为度，下发为 rad |
| 标定 | 头部／双腕采样、求解、候选查看及安装 | 入口和共用控机可用；本轮未做完整标定求解 |
| 控制研究 | 参数预设、完整 1.5× 研究轨迹、数值记录与中间结论 | 已配置；研究按钮自身不自动录像，且不代表精度／平滑度已达标 |
| 观察与目标浏览 | 三路实拍、目标姿态、人工规划草稿 | 三路实拍正常；名义模型预览不等于实机标定资格 |
| 工作台自动数采 | 规则动作、录制和结果入口 | ARX13 `collection_setup` 尚未装配，完整自动数采未验收 |
| 原人工数采 | Prometheus人工遥操作／示教录制 | 一键退出后已实际启动，机器人／相机／踏板ready；本轮未录制episode |
| 推理 | 统一 Torch／Triton 模式、checkpoint 选择、模型服务与预热 | 一键启动、两模式固定输入推理均已实测；机器人 Run 因本机映射未验收而禁用 |
| 结果与回放 | 按业务查看历史结果、视频、曲线和回执 | 历史结果不是当前控制状态 |
| 服务运维 | 独立状态、接入／释放／恢复、服务与相机管理 | 接管与环境退出均已真实点击验收；退出后原人工数采启动通过 |

模型控机还需要本机关节零偏／符号及训练夹爪行程映射资格。原生夹爪端点已绑定，不等于
训练模型坐标映射已完成。此前 Torch checkpoint 不可见由 inference/catalog 未装配导致，
本版已修复，两种模式均能看到 epoch008。人工动作或固定输入模型回包不能代替模型控机验收。

## 4. 部署位置与计算依赖

本地开发仓库：`/Users/wujie/Developer/work/anyverse/github/zhoufeiyu17/robot-workbench`。
远端以下相对路径均以 `/home/wujie/work/feiyu.zhou/arx13-control-20261029-v1` 为根：

| 路径 | 用途 |
|---|---|
| `source-reclaim-control-v3/` | 当前独立 Owner 的冻结代码 |
| `source-environment-exit-v9/` | 当前管理／网页及业务代码，包含环境退出与手工规划读回同步修复 |
| `source-model-startup-v6/scripts/serve_base566_model.py` | 已验收的常驻模型 endpoint，复用外部冻结的模型数值实现 |
| `run-reclaim-control-v3/` | Owner 配置与描述文件；本轮未替换 Owner |
| `run-environment-exit-v9/` | 当前业务、管理配置和启动入口 |
| `run-model-startup-v6/model/` | 模型 registry、CID、ready 和进程身份回执；凭据不共享 |
| `startup/current-release.json` | 当前启动版本选择器；本次为 `release-environment-exit-20261031-v9.json` |
| `deployment-tools-model-v6r5/workbench_deploy.py` | 已安装启动／封存工具，仓库源为 `scripts/workbench_deploy.py` |
| `env/` | 工作台独立 Python 环境 |
| `sdk-dm-equivalent-v1/`、`legacy-pedal-mapped-v1/` | 绑定的 SDK 与工作台专用适配副本 |
| `source-v2/scripts/serve_camera_sources.py` | 独立相机组件，本轮保持既有版本 |
| `storage-v2/`、`references/` | 业务存储及研究参考；与冻结代码分开 |
| `reclaim-control-20261030-v3/` | 发布、启动和本轮实机验收记录 |
| `cold-start-20261030-v1/` | 五服务真实全退出／冷启动证据 |
| `model-startup-20261031-v1/` | 模型服务退出／冷启动、目录、固定输入回包与最终交付记录 |
| `exit-collection-20261031-v1/` | 一键退出真实点击、回零／释放和原人工数采启动／只读核验 |
| `docs/WORKBENCH_GUIDE.md` | 本说明的 ARX 本机副本，放在冻结源码之外 |

网页、管理、业务、Owner、相机、Triton CPU 工具和模型 infer 的部署计算均在 **ARX13**；
不依赖 Mac 或 3090 station 转发／计算。浏览器 WebGL 使用访问者设备。模型使用 ARX13 的
RTX 4090；本轮退出后模型容器和worker已确认退出，GPU用途已交回原人工数采。
等待人工录制时GPU compute为空是正常现象，不能据此抢占数采资源。一键启动不依赖旧实验的临时 GPU 窗口；
日常 operator 服务按固定主机／UUID及同一 GPU 文件锁准入，保留显存与已有 compute 进程检查。
进程互斥不能代替资源预留登记；Agent 触发的加载／推理仍须先取得唯一 GPU 表中的有效预留。

用户级服务前缀为 `robot-workbench-arx13-`：`manager.service`（18868 内网入口）、
`business.service`（127.0.0.1:18878）、`owner.service`、`cameras.service`。
`triton-control.service` 独立监听 127.0.0.1:18924，已纳入一键脚本启动清单。原数采网页为 `prometheus-arx13-web.service`，
监听 127.0.0.1:18980；不能用其 LAN 端口访问失败推断数采已退出。
本次没有修改原数采代码、环境、CAN 配置或数据。

真正的模型服务是 Docker 容器 `pi05-arx13-resident-workbench-v6`，监听 127.0.0.1:18925；
18924 仅负责转换等工具任务。模型数值源码与制品只读复用
`/home/wujie/work/feiyu.zhou/manipulation-arx-pi05-20261029`，未修改冻结模型实现。
常驻适配器只改变 operator 资源准入口径，保持原 `triton-control/gpu-use.lock` 的同一 inode，
从加载到 worker 确认退出全程持锁。进程身份同时绑定 Docker CID、真实宿主 PID 与启动时刻。

目标姿态用浏览器 WebGL 渲染，没有另一个目标预览主进程；实拍 mesh 预览是业务后端内的
CPU 线程，只有选择合格标定组才可使用。本轮缺完整标定组，不能把线程存在等同叠加已验收。

## 5. 核心控制逻辑

```text
浏览器草稿／研究配置
  → 业务预检、冻结计划、一次审阅
  → 唯一 ControlOwner：持权、实例、命令代次和保护核验
  → 共用目标解释器：平滑参考与 q/dq 时间窗口
  → SDK 原生输出 → CAN → 电机内环
  ← 关节反馈、速度与状态 ← SDK／独立 Guard

Stop → 独立 Owner 通道 → 撤销旧代次与未来队列 → 请求保持 → 核验返回及静止反馈
相机／录像／目标显示独立于控制输出；页面断开不会停止 Owner。
```

普通动作采用 SDK 默认运动增益 `normal_X5`，I=0；研究配置只有保存并绑定新动作后生效。
停止保持使用独立增益，不能拿它推断运动中的 P/D。当前保留 0.8 rad/s 关节速度保护及
独立 100 ms 反馈／心跳保护；保护故障不自动重试。
反馈允许的边界余量与命令限位分开：夹爪左命令 0–80 mm、反馈 −3–83 mm；右命令
0–89 mm、反馈 −3–92 mm，关节反馈有 2° 边界余量；这些余量不扩大下发命令范围。

## 6. 常见异常与处理

| 现象 | 优先检查与处理 |
|---|---|
| 网页打不开 | 核对 ARX13 电源／网络；本机执行 `--check`，再按结果运行启动脚本；不先重启 SDK |
| 页面能看但不能执行 | 看当前持权、反馈和业务依赖；显式交接；数采／推理未配置不靠重新接 SDK 解决 |
| 接入被数采占用阻塞 | 用“清理现有控制并接入”；等待正常停止回执，退出不明时不另开 SDK |
| `FAULT_STOP_UNCONFIRMED` | 表示缺少停止确认，不等于已安全停住；看具体 reason、保持回执和新鲜反馈，再按恢复入口要求处理 |
| Owner／CAN 心跳失联 | 不续跑旧动作；保留故障记录。无健康反馈时先确认现场支撑，再使用适用的恢复流程 |
| 踏板断连／ENODEV | 查看 USB 与踏板健康；本版锁存动作故障后继续真实反馈读取，避免旧分支跳读引发二次心跳退出；不自动清锁 |
| 相机陈旧／无图 | 区分具体相机、源帧龄和服务状态，查 USB；先处理相机组件，不重启整个控制进程 |
| `target changed`／部署身份变了 | 刷新页面后重新预检／审阅，不重复提交旧请求 |
| 启动清单或 unit 哈希不符 | 核对实际版本并重新封存合格候选；不跳过校验，不覆盖原数采文件 |
| 操作中断／结果未知 | 查同一操作 ID 的回执及实际状态；不要因浏览器超时重放动作 |

只读日志命令（ARX13 本机）：

```sh
systemctl --user status robot-workbench-arx13-manager.service robot-workbench-arx13-business.service robot-workbench-arx13-owner.service robot-workbench-arx13-cameras.service --no-pager
journalctl --user -u robot-workbench-arx13-owner.service -n 100 --no-pager
```

## 7. 本版验收与后续

当前管理发布身份：`64cb428b021aadb854013c8c41917f612218b3d54d672051add03a2adea64ca5`。
下述真实控机验收阶段身份为 `3c2d3d425488e3de0ccec1a6cf768af1219913c021cb94002c47c542e9076270`。
基础候选 300 项 Linux 检查通过；增加踏板绑定后的相关 190 项通过（不能相加当作互不重复的覆盖）。
真实浏览器已验收数采退出后接入、健康 SDK 停止后复用两种路径；重复启动不换 SDK，SDK acquisitions=0。

四栏目各经共用人工入口执行左 J1 小角度动作，其余目标保持，约 1 秒后 Stop：

| 栏目 | 运动起点到 Stop 接受 | Stop 接受到 SDK 保持返回 | J1 实际位移 |
|---|---:|---:|---:|
| 标定 | 1.0011 s | 13.22 ms | −0.743° |
| 控制研究 | 1.0012 s | 9.84 ms | +0.699° |
| 数采 | 1.0017 s | 5.08 ms | −0.699° |
| 推理 | 1.0007 s | 7.70 ms | +0.721° |

随后均健康静止、自动动作关闭，未触发速度保护。上述毫秒值是 SDK 调用返回时间，**不是机械
停止耗时**；不是完整业务或控制精度验收。早期轮询计时样本在 1.545 秒后才 Stop，原件保留，
不计入上表。该阶段结束时为已接入 `STOPPED_DAMPING_HOLD`。之后为冷启动验收，按正常链
0.1 rad/s 回零并释放 SDK；回零残差保留，未将其当作精度通过。

### 环境退出与人工数采启动（2026-10-31 最新）

真实浏览器点击退出及确认各一次，成功操作ID为`b9b2db8bc4704aa58c93bcd23c494ff0`；
03:23 CST完成正常回零、SDK释放及五项注册资源退出，管理页保留，原数采网页恢复idle。
03:33 CST单次调用原`/api/collect/start`，约5秒后到达ready。03:45只读复核为同一会话、
心跳序号持续增长、无孤儿进程、robot/cameras/pedal均ready、event已启动，三路预览LIVE。
任务1251，新批次`咖啡打包-20261031_033312-0`；新批录制0条，原4条已上传记录不变。
**通过的是环境交接和硬件启动，不是完整episode录制、保存、上传或质检验收。**

Linux相关132项检查与后续104项共享链／竞态回归通过（覆盖重叠，不相加）。首次按钮尝试在
规划前因Stop交接后的反馈缓存代次拒绝而失败，未下发回零；修复为有界只读同步且保留审阅撤销。
原失败回执保留。数采启动验收脚本误要求ready阶段存在recorder，因此曾超时；核对原实现后，
仅补只读验收，没有重发启动。recorder只在开始一条录制时创建。

### 模型 infer 冷启动（2026-10-31 00:53 历史）

正常停止模型后，核对旧容器、服务与 GPU worker 全退出；单次 `start-workbench.sh` 用时
**58.84 秒**重新创建模型容器并加载／预热。返回 `model_configured=true`、`model_ready=true`、
`sdk_acquisitions=0`，新服务 PID 与 worker PID 均已核验。再次执行脚本为 `actions=[]`、
`model_preparation.reused=true`，不重复加载。已有 SDK 身份与停止保持全程保留。

Torch 和 Triton 均使用同一份已保存的训练域输入完成真实 GPU 推理，得到有限的 **60×14** 输出；
这不是机器人动作。该阶段验收结束恢复 Torch；其后用户切换为 Triton，本次发布保留该选择。浏览器实测两模式均有 epoch008 checkpoint，
模型显示“已加载”，Run 按未验收的本机映射明确禁用。

本次同时修复：infer 服务未注册／未启动、模型与业务认证文件不一致，以及管理器尚未观察到
新模型 PID 就提前提交预热的时序问题。首次失败原件保留，启动器不自动重放未知操作。
本轮未重启独立 Owner、未重新接 SDK。此前五服务整体冷启动记录见下节；不将分段验收表述为
本轮再次完成所有服务／主机电源的冷启动。

### 五服务整体冷启动（2026-10-30 历史）

2026-10-30：管理、业务、相机、Owner、Triton CPU 五项服务及其 cgroup 子进程全部正常退出，
核对旧 PID 消失；仅调用一次 `start-workbench.sh`，约 **4.02 秒**启动五项服务，全部主 PID 更新。
顺序为 Owner → 相机 → 业务 → Triton CPU → 管理。三相机新帧、12轴人工规划、浏览器真实
WebGL目标图像、Triton目录均已核验。没有启动GPU任务，没有接SDK；最终为 `DISCONNECTED`。
这是同一主机正常退出后的进程冷启动，**不是断电／整机重启测试**。原验收脚本最后误取断开态
不存在的 `automatic_enabled` 字段而报错，原件保留；随后只读核验成功，没有重复启动。

冷启动之前的一键脚本漏掉独立 Triton CPU 服务，已通过可信 unit／入口／端口／凭据路径绑定
补入清单，并保留其 GPU 任务准入。新增相关 Linux 16 项检查通过；旧 Owner、控制代码和增益未改。
随后仅更新管理／网页，修复模型下载期间点击预览误发 CPU 备用请求的问题。故意延迟模型下载后
立即点击已复验：本地目标渲染成功、没有备用请求或前端错误；该显示更新只补起管理进程，
不冒充再次执行了五服务冷启动。

## 8. 给二次开发者的入口

代码仓库：<https://github.com/zhoufeiyu17/robot-workbench>。当前本地分支为
`fix/shared-workspace-tools`，上一已提交基线为 `e87aafc791fc85c94b4b0eb8e9bf62adf60424de`。
2026-10-31源码整理提交为 `11454b92b44d9056c3dbc06995647c97175ffafc`，完整检查见[验收摘要](acceptance/workbench-20261031.json)；SDK原生库、模型制品、机器标定和私有运行配置仍是外部依赖，
clone源码不代表已具备真机部署和控机资格。实际部署的冻结源码
在 ARX13 的 `source-reclaim-control-v3/`（Owner）、`source-environment-exit-v9/`（管理／业务／网页）
及 `source-model-startup-v6/`（已验收模型 endpoint），最新启动工具在 `deployment-tools-model-v6r5/`；
二次开发先取得这些只读快照及发布清单，再建立独立开发分支。不要直接编辑线上冻结目录，
也不要整树覆盖本地既有工作。

| 要修改的功能 | 优先阅读的仓库文件 |
|---|---|
| 页面与人工规划／目标显示 | `web/app.js`、`web/operations.js`、`web/target_viewer.js` |
| HTTP与业务调度 | `robot_workbench/service.py`、`workflows.py` |
| 运维接管与生命周期 | `robot_workbench/management.py`、`operations.py`、`managed_services.py` |
| 唯一Owner与控制状态 | `robot_workbench/control_owner.py`、`runtime.py` |
| 参考曲线与SDK窗口 | `robot_workbench/target_interpreter.py`、`motion_reference.py`、`sdk_process.py` |
| 机器配置与保护 | `robot_workbench/device_bootstrap.py`、`arx_output_guard.py`、`joint_bounds.py` |
| 相机／录像／预览 | `scripts/serve_camera_sources.py`、`robot_workbench/rgbd_service.py`、`recording_process.py`、`physical_preview.py` |
| 启动与部署 | `scripts/start_workbench.sh`、`scripts/workbench_deploy.py`；已安装入口的工具目录由发布版本绑定 |
| 常驻模型接入 | `scripts/serve_base566_model.py`、`robot_workbench/docker_model_service.py`；资源准入与数值模型实现分开 |
| 推理候选 | `robot_workbench/inference_workflow.py`、`inference_transport.py`、`triton_control.py`；本地候选与已部署版本分开看 |

在独立开发目录进行无真机检查，不使用原数采 Conda：

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
sh scripts/check.sh
PYTHONPATH=. .venv/bin/python -B -m robot_workbench.service --protocol-fake --state-dir .evidence/local-device --port 18866
```

开发入口为 `http://127.0.0.1:18866/`。Protocol fake 只验证接口／状态逻辑，不包含真实物理／相机；
外部资源缺失而跳过的检查不算通过。修改控机链后，在目标 Linux 做相关回归，使用新发布目录与
新清单；走正常回零／释放维护，再核验停止／恢复。部署命令参考下列规范；新版本需要重新绑定
Owner、业务、SDK、相机、模型及 unit 身份，不能只改 `current-release.json` 伪装升级。
源码提交与远端状态以Git记录为准；此次Git整理不重启线上服务，也不扩大既有实机资格。

剩余顺序：① 补踏板 3 实际触发 Stop（用户可操作时）；② 补本机模型映射后验收模型动作，由
W2 与模型部署任务协作；③ W2/W3 补完整自动数采。人工数采环境交接、模型服务启动与模式选择已通过。
控制精度和平滑度研究另行继续，
本版不宣称解决原有抖动。

证据入口：[模型冷启动](../.evidence/arx13-model-startup-20261031-v1/cold-start-r4.json)，
[模型真实回包](../.evidence/arx13-model-startup-20261031-v1/model-smoke.json)，
[模型页面验收](../.evidence/arx13-model-startup-20261031-v1/infer-torch-ready.png)，
[五服务冷启动独立核验](../.evidence/arx13-cold-start-20261030-v1/cold-start-readonly-verification.json)，
[冷启动预览截图](../.evidence/arx13-cold-start-20261030-v1/target-preview-verified.png)，
[本轮控机验收汇总](../.evidence/arx13-reclaim-startup-20261030-v1/acceptance-summary.json)，
[实际发布记录](../.evidence/arx13-reclaim-startup-20261030-v1/publication.json)，
[10月30日只读状态](../.evidence/arx13-reclaim-startup-20261030-v1/final-state.json)，
[桌面运维截图](../.evidence/arx13-reclaim-startup-20261030-v1/operations-final.png)。
项目续接权威索引为 `.evidence/next-optimization-20261017/continuation-state.json`。
详细首次部署、封存及回滚流程见 [部署规范](physical-deployment-runbook.md)；其中旧版本段落按日期理解。

本次退出与数采交接收据：`.evidence/arx13-exit-collection-20261031-v1/delivery.json`；
真实点击、原启动及只读核验原件随同目录保存。既有模型启动与冷启动回执按日期保留。
