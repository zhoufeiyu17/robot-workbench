# ARX6 共享 infer：A/C 联合配置候选

维护者：C 工作台会话 `01a16327-7545-7332-a361-0123ddad811d`。日期：2026-11-03，Asia/Shanghai。
父入口：[跨会话统一交接](/Users/wujie/Developer/work/anyverse/_context/notes/shared-infer-control-handoff-20261103.md)。
本文是离线候选的审阅索引。来源、配置、B/D 接线或真实验收变化时更新；历史回执原件保留。

**状态：A 与 C 来源及共享生命周期配置已冻结；B 最终差异待接入。未上传、安装、启动或切换现场配置，不能当作完整控机候选。**

## 本次制品

证据根目录：`../.evidence/shared-infer-joint-candidate-20261103-v1/`。

- [publication-plan.json](../.evidence/shared-infer-joint-candidate-20261103-v1/publication-plan.json)：具体来源、文件选择、归属、退出、回滚及缺项。
- [configuration.diff](../.evidence/shared-infer-joint-candidate-20261103-v1/configuration.diff)：相对已保存部署配置的逐行差异。
- [receipt.json](../.evidence/shared-infer-joint-candidate-20261103-v1/receipt.json)：本轮检查与制品 SHA。
- [payload.tar.gz](../.evidence/shared-infer-joint-candidate-20261103-v1/payload.tar.gz)：31 个代码覆盖、配置、unit/启动选择及回滚文件；无模型权重或凭据值。
- `manifests/`：model/business 的完整预期源文件清单及模型 runtime 输入。
- `build_candidate.py` / `validate_candidate.py`：离线生成与检查脚本；没有远端发布逻辑。

每个目标源目录须从该角色自己的旧部署源码复制，再只覆盖清单文件；不能同步整份工作树。
旧源码事实取自 **03:14:32 CST** 的冻结 SSH 回执；并非当前现场核验。

## 明确复用的部署事实

目标仍是 `wujie@10.17.32.5`，保存的 hostname 为 `wujie-ThinkStation-P3-Tower`。
网页仍为 `http://10.17.32.5:18868`，业务 HTTP 为本机 `18878`，模型 HTTP 为 `127.0.0.1:18925`。
本方案只准备 ARX6 同机 C/D 消费端，不新增 ARX13 跨机监听、代理或端口。

| 对象 | 既有 unit | 本候选归属与选择 |
| --- | --- | --- |
| 共享模型 | `robot-workbench-arx6-mixed366-model.service` | A/服务管理员持有生命周期；换到 model 的新源目录及 registry |
| 工作台业务 | `robot-workbench-arx6-staged-business.service` | C；换到新业务源码与 launcher 配置 |
| 工作台 manager | `robot-workbench-arx6-staged-manager.service` | C；源码与配置同时换到 external-aware 候选 |
| 控制 Owner | `robot-workbench-arx6-staged-owner.service` | 保留旧选择；B 完整版本尚未纳入 |
| RGB 订阅器 | `robot-workbench-arx6-native-rgb.service` | 保留旧选择；本轮不启停相机 |
| 原人工数采 | `prometheus-web.service` | D；不改其配置、环境或当前状态 |

新目录名均为**提案**，尚未检查远端是否存在：

- model：`/home/wujie/work/feiyu.zhou/arx6-mixed366-20261102-v1/source-shared-20261103-v1`
- business/manager：`/home/wujie/work/feiyu.zhou/arx6-live-infer-20261102-v1/source-shared-20261103-v1`
- 配置在原角色的 `config/` 下，文件名带 `shared-20261103-v1`；状态也用对应的新子目录。
- 两个既有 `start-workbench*.sh` 的路径保留，候选内容同时改为新脚本源与新 startup JSON。

## 共享生命周期选择

manager 显式设置 `model_lifecycle="external"`。startup JSON 显式指向该 manager profile，unit 列表也移除模型 unit。
脚本还会核对外部模型的 `kind=user_unit` 及确切 unit；缺失、空值或不支持的映射在任何进程操作前拒绝。

共享服务应先由 A/服务管理员准备。C 的启动命令启动自己的页面及已登记服务，复用外部模型；外部模型缺失时不会越权拉起它。
原无新字段的私有配置仍保留原行为，**不能将旧 startup JSON 原样视为共享配置**。

C 的“释放本工作台模型绑定”和退出仅释放自己的 pin；业务停启先处理自身绑定，不停止共享模型进程。
退出判断仅要求 C 托管角色停止。backend 已缺失或释放回执未知时保留“不确认/等待服务 TTL”的事实。
显式管理员停服会使所有 pin 失效；本方案不承诺“查询零 pin 后停服”的原子性。

## 固定来源与 B 边界

A 采用已封存的 `shared-infer-owner-20261103-v1`，继承原 canonical pin 入口。
C 合并 `workbench-module-integration-20261103-v1` 与 `workbench-shared-model-lifecycle-20261103-v2` 的所属文件。
model 覆盖 6 个文件，business 覆盖 10 个文件；源 SHA 逐项绑定原回执。

B 核心文件保持各角色原部署清单，未复制 B 正在变化的工作树。`action_chunks=false`，`normal_modes` 未配置，strict begin 未启用。
这些关闭项防止不完整接口被默认激活，不代表新 B 已验或原 0.502 秒资格自动沿用。
B 最终源码/回执、Owner/SDK 能力与原生二进制来源、finish 接口和受影响的消费者回归仍须补齐。

新模型身份为计划值，未加载：

| 身份 | SHA256 |
| --- | --- |
| runtime | `2e0c4b61df9cedb5b11063aa37ece9c962f351390943daad8f1f9a9e70d729b8` |
| Torch bundle | `c2c3cf7b8fdab120f7324e9fde552fcf72dd9fb89a0d5266f5754f48a9ccd683` |
| Triton bundle | `ca7441bcb9eefddb42d529c1d8b46ed13343b80f5e7a977eaf5a54c384d3dcc2` |

117 项 runtime 输入重新计算；训练制品与 native Triton 文件引用保持原值。本轮没有读取或复制权重字节。

## 发布前、退出与回滚

发布前重新核当前主机/boot、源码、unit、配置及数采归属，并检查新路径冲突。旧 C 的 GPU 记录和 heartbeat 保持释放；新共享用途由 A 按真实使用登记。
本次只备好文件，不包含远端安装脚本或执行许可。中控取得联合候选后另行提出具体真实验收方案。

消费者单独回滚时，若 D 仍用共享模型，应保持受影响的 C 消费端停止，或保留 external-aware manager。
不能启动旧私有 manager 去管理仍被共享使用的模型。恢复整套私有部署须统一停止消费者准入，由 A 在明确窗口退役共享服务，再一起恢复匹配的 unit、launcher、配置和 bundle。
回滚字节与权限来自原冻结配置；未知退出不强杀，不自动重接 SDK、续跑 episode 或启动数采。

本轮通过：生成结果重复一致、31 项 payload SHA/语法、ServiceSpec、旧回滚字节、B 核心未混入，以及真实候选启动脚本的拦截式检查。
拦截式检查只产生四个预期的模拟 unit 启动调用，模型调用数为零；实际 HTTP/systemd 调用数均为零。
这些是新增配置组合检查，不是模型、Owner、完整工作台或真机回归。

剩余按优先级：B/D 最终差异与联合配置冻结；新鲜部署/资源/归属核验；具体真实模型双消费者与工作台非回归；完整控制与 finish 验收。
