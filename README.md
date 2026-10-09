# Robot Workbench

统一标定与数采工作台。当前是隔离重构候选，尚未替换现有工作台或连接真机。S0 已验证左右臂 Owner→PhysX 实测响应、六份三相机 RGB-D、停止/失联保护及48个连续物理步的反馈更新。完整 L2 业务资格尚未完成，具体边界见 [验证摘要](migration/validation-summary.json)。

维护者：工作台牵头任务；2026-10-10。修改接口、执行策略、来源或资格时更新本入口及对应证据，不能将历史通过结果直接算作新版本通过。

## 代码与来源

- `calibration/`：标定工作流、Owner、求解器和已有页面，来自冻结标定源码。身份见 [calibration-source.json](migration/calibration-source.json)。
- `rule_based_workbench/`：数采工作流、30 Hz 连续执行策略、相机/录制及 GPU 服务接口。174 文件运行基线与 13 Python 候选增量独立记录在 [collection-source.json](migration/collection-source.json)。
- `robot_workbench/runtime.py`：同一 Owner 上的业务权限、停止证据、保持交接与设备独占。现阶段由嵌入服务显式注册业务身份，尚未接入新 HTTP/UI。
- `robot_workbench/physx.py`：同 Owner 命令出口到 PhysX 实测反馈的候选适配器，控制 100 Hz、物理 240 Hz 各自显式计时；不证明真实 X5 增益/踏板或实体止动。
- `scripts/probe_owner_physx.py`：有界 GPU 可接性探针。必须先核验资源并实例化运行合同，不能在本机运行 Isaac。

原工作树与旧运行路径不修改。原生 robot_sim、A1 USD/相机/几何、Isaac、cuRobo、检测环境与 SDK 均是外部依赖；不复制权重、SDK 二进制、密钥或私有服务配置进仓库。外部身份起点见 [native-dependencies.json](migration/native-dependencies.json)。该清单含历史录制 provenance，原生源码与 workbench helper 必须分别解析，不把全部 129 项都视为原生目录文件。

## 本地 CPU 检查

Python 3.10–3.12；无需 SDK、ROS、相机、DSW 或机器人。使用仓库内虚拟环境：

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
sh scripts/check.sh
```

检查涵盖原标定测试、真实 Playground plan 调用链（只替换相机/网络/GPU边界）、控制时钟、共享 Owner 交接和固定竞态排列。CPU fixture 的通过不等于原生 PhysX、真实 RGB-D 或硬件资格。

## 接入边界与未完成项

已有实现保留两种业务策略：标定逐点到位/静止采图；数采 30 Hz 冻结目标，普通偏差只记录，数据错误不转为运动故障。停止、连接释放、控制权交接和运动审阅是不同动作。物理连接/释放及运动仍遵循既有现场确认。

跨仓设计与用户决策统一入口：本地 Anyverse `_context/notes/unified-workbench-refactor-20261009/README.md`；当前证据在被忽略的 `.evidence/s0/`，GPU原始数据留在对应 task-owned run。正式安装对外继续保持 `/var/lib/anyverse/calibration/hardware/active/{head.json,wrist.json,manifest.json}`。

剩余交付：新 HTTP/UI 与媒体接入；完整空库标定→安装→交接→数采→回放；L1/L2 故障覆盖与性能目标；真实设备单 writer 切换。当前仓库不可凭小探针结果标记为生产就绪。
