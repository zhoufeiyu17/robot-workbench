# ARX13 Triton 转换与输出预览

2026-10-30。工作台 UI/API、CPU任务服务及硬件交接由本任务维护；模型转换、数值资格和GPU使用由「梳理 π0.5 infer 现状」维护。入口：[控制阶段状态](control-runtime-status.md)；跨仓统一记录：[π0.5 ARX13 部署状态](/Users/wujie/Developer/work/anyverse/_context/notes/pi05-arx13-deployment-20261029.md)。模型、接口、资源责任或设备状态变化时同步更新。

**已从实际页面完成两次独立提交：转换成功并成为 READY；选择该模型后，实时 Infer 成功返回60×14预测。没有执行预测动作，未放宽数值门，旧失败结果保留。**

## 使用入口与本次结果

[打开工作台 Triton 模式](http://10.17.32.3:18868/?environment=physical&business=inference&infer_mode=triton)。也可在“推理 / Infer”栏目选择“Triton · 模型输出预览”。

1. 选择已登记的源模型，点击 **Triton 权重转换**。当前登记 Base566 epoch008 / step67408；打包、同输入数值校验、部署制品与完整哈希核验通过后，模型才进入 READY 选择栏。
2. 选择已转换模型，点击 **Infer · 开始推理**。读取现有三相机与关节反馈，加载模型，返回一次60步、每步14维的输出预览。页面提供逐维曲线、逐步数值和原始计时；任务结束清退模型worker和自有容器。

本模式使用项目内的 Triton kernels 与 cuBLAS/SDPA/CUDA Graph 路径，不是 NVIDIA Triton Inference Server。本次新精度方案是显式混合执行路径，不能沿用旧候选约55 ms的计时。

| 验收 | 实际结果 |
| --- | --- |
| 转换任务 | `116c2db142b24caf83fa9cefd9b06428`，READY；页面仅提交1次 |
| 数值对照 | 3份输入 × history 0/3/60，共9组；normalized、物理关节、夹爪最大差均为0 |
| 部署SHA256 | `d28c5407150822c71546ab0541168ad17f82d683c5c6cb039fcdd5d6c7f448f9` |
| 实时推理任务 | `cc8f3c674b7446b99820b6d203c480b1`，SUCCEEDED；页面仅提交1次 |
| 本次实时计时 | worker请求 **74.19 ms**；另有加载/预热 **49.26 s**，不把74 ms当成点击到出结果的总耗时 |
| 输出与输入资格 | 有限60×14；`live_nominal_unqualified`，ARX13模型坐标/夹爪映射仍未完成标定资格 |
| 收尾 | 两任务均确认退出、自有容器不存在、GPU compute为空；GPU预留继续由负责Session维护，不能称为已释放预留 |

READY表示转换数值与制品资格，不代表真机任务成功、控制精度或预测动作可以执行。本次 `hardware_control=false`、`command_publication=false`，不自动执行、回零或恢复旧动作。

## 当前硬件与服务状态

用户在协作聊天中明确授权将控制交回工作台。交接前确认原Prometheus数采idle、无manager/orphan/teleop，待上传批次为review而非上传；随后正常暂停其网页、启动工作台相机并接入现有Owner。没有执行额外轨迹。

本次最终核验：holder为 **calibration**，epoch **0**，SDK为READY、健康静止保持，三相机ready；连接身份与Infer前一致。SDK接入本身会发保持命令，因此这一步依据用户新增授权执行，不归为纯读。后续模型只读消费现有输入，不接第二个SDK、不交接机器人业务控制权。

**SDK和相机保留连接，原数采网页暂时停用；不自动释放或归还。** 原18869/18870标定服务PID3363/3362保持。用户提出的“控制权管理小模块”只登记为后续待办，本轮未扩建。

工作台任务根：`/home/wujie/work/feiyu.zhou/arx13-control-20261029-v1`。

| 组件 | 当前来源 |
| --- | --- |
| 18868管理/API/页面 | `source-triton-ui-v3`；`run-triton-ui-v1/manager.json` |
| 原业务与Owner | `source-dm-equivalent-v1` / `run-dm-B0-final-v1`，本轮页面发布未替换这两项 |
| CPU任务服务 | `source-triton-control-v3`；`robot-workbench-arx13-triton-control.service`，仅监听127.0.0.1:18924，已启用用户级开机启动 |
| 启动封存 | `startup/release-triton-ui-20261030-final.json`；当前选择器已更新，`--check`完整性与startup_ready通过，检查不发机器人命令 |

同时修复了启动封存把动态 `backend/current.json` 和 `ExecStartPre --output model-prestart.json` 当成静态输入的问题：按字段与命令参数的生产/消费语义区分，仍封存脚本、profile与unit；并非从清单手工删文件绕过检查。新增回归与13项Linux启动检查通过。CPU任务服务独立启停，不由主工作台启动脚本加载模型。

最终UI仅在Triton模式隐藏不适用的旧控机业务“尚未配置”引导，保留设备状态、故障与Stop；该独立页面修订保持SDK nonce/epoch、业务后端与其他服务不变。

## 接口、资源责任与失败行为

CPU任务根：`/home/wujie/work/feiyu.zhou/manipulation-arx-pi05-20261029/triton-control`。token只保存在该目录的私有0600文件，不下发浏览器。浏览器通过18868同源代理，保留Host、CSRF和部署身份校验。

| CPU接口 | 契约 |
| --- | --- |
| `GET /v1/catalog` | sources、models、recent_jobs；FAILED/INVALID不可用于Infer |
| `POST /v1/conversions` | 固定source_id与唯一request_id，不接受任意路径或命令 |
| `POST /v1/infer` | READY model_id与唯一request_id；一次实时输出预览 |
| `GET /v1/jobs/{job_id}` | 阶段、终态、错误、输出和退出证明 |
| `GET /v1/health` | CPU服务可用性，不代表模型READY或现场输入有效 |

同request_id同参数返回原任务，不同参数拒绝；同时只允许一个GPU任务。提交超时保留请求身份并查回执，不自动再提交。页面可从recent_jobs恢复请求，真实成功输出也已通过该只读恢复路径验证。

GPU唯一使用记录继续归属Session **`01a136ba-6810-7262-b354-3358b3fac8f5`**，由其真实核验、更新台账和监控。本服务只读其导出的 `workspace/gpu-allocation.json`：必须匹配session_id/GPU UUID/run_id，更新时间不超过60分钟、剩余至少1200秒，且 `jobs_enabled=true`。派生文件不是第二份预留台账，也不能自行续租。手工GPU诊断前应先关闭job入口并确认busy=false。窗口失效时按钮拒绝启动，不把旧记录当有效授权。

实时输入缺失返回INPUT_UNAVAILABLE，不自动接SDK或相机。任务超时/关闭只清退精确job容器名/CID及其子进程；退出不明、服务重启遗留活动任务会锁住新任务，需维护者核验，不自动重试或影响别人的进程。

## 验收证据与待办

证据根：[arx13-triton-ui-20261030-v1](/Users/wujie/Developer/work/anyverse/github/zhoufeiyu17/robot-workbench/.evidence/arx13-triton-ui-20261030-v1)。保留真实按钮request/job、转换报告、部署制品JSON、实际预测、私有服务拒绝、硬件交接和最终退出证明；没有下载模型权重到Mac。

- [真实成功桌面页面](/Users/wujie/Developer/work/anyverse/github/zhoufeiyu17/robot-workbench/.evidence/arx13-triton-ui-20261030-v1/triton-ready-output-desktop.png) · [移动端](/Users/wujie/Developer/work/anyverse/github/zhoufeiyu17/robot-workbench/.evidence/arx13-triton-ui-20261030-v1/triton-ready-output-mobile.png) · [实际延迟回执](/Users/wujie/Developer/work/anyverse/github/zhoufeiyu17/robot-workbench/.evidence/arx13-triton-ui-20261030-v1/triton-live-timing.png)
- [转换按钮证据](/Users/wujie/Developer/work/anyverse/github/zhoufeiyu17/robot-workbench/.evidence/arx13-triton-ui-20261030-v1/button-convert.json) · [Infer按钮与实际输出](/Users/wujie/Developer/work/anyverse/github/zhoufeiyu17/robot-workbench/.evidence/arx13-triton-ui-20261030-v1/button-infer.json) · [最终进程/控制状态证明](/Users/wujie/Developer/work/anyverse/github/zhoufeiyu17/robot-workbench/.evidence/arx13-triton-ui-20261030-v1/final-proof.json)

本轮两按钮、真实转换与名义实时输出预览已完成。后续优先级：

1. GPU负责人持续维护服务窗口与真实核验，避免长期入口依赖过期回执。
2. 补齐ARX13模型坐标/夹爪映射标定后，再谈已标定实时输入与动作资格；本轮不执行模型动作。
3. 按用户后续要求启动控制权管理模块；目前仅记录，不提前开发。
