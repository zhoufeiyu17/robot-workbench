# GPU 准入与模型切换

模型加载和转换共用 `robot_workbench/gpu_admission.py`。工作台按 GPU UUID 获取运行互斥锁，按实时进程身份判定占用，并在退出已确认后释放锁。跨 Session 的资源预留继续由 GPU 使用台账管理。

## 入口与责任

| 入口 | 准入与生命周期 |
| --- | --- |
| WAM、WAM-MEM、Mixed366 | 共享 UUID 锁覆盖 worker 启动、预热、运行与退出；旧导入入口继续转发到公共模块 |
| Base566 原生适配器 | 复用原客户端的加载与退出流程，GPU 获取阶段使用公共模块 |
| 转换与输出预览 | 复用业务授权检查，公共 UUID 锁覆盖任务与子进程清退 |
| ModelService | 新候选的制品、依赖和资源预检通过后退出旧 worker；按请求指纹加载并预热 |

锁文件默认位于 `/tmp/robot-workbench-gpu-<uid>/<GPU-UUID>.lock`，目录和文件只允许当前服务用户访问。锁由操作系统持有，文件中的 PID、模型 ID 仅用于诊断。部署可通过 `WORKBENCH_GPU_LOCK_DIR` 统一指定锁目录，各入口使用同一目录。

## 部署环境策略

环境策略默认读取 `~/.config/robot-workbench/gpu-admission.json`，可通过 `WORKBENCH_GPU_POLICY` 指定。文件为当前用户所有的普通文件，权限为 `0600`。

```json
{
  "schema": "workbench.gpu-admission.v1",
  "desktop_executables": ["/path/to/trusted/browser"]
}
```

桌面 GPU 进程需要同时满足已登记可执行文件、当前用户、GPU 子进程类型、桌面 cgroup 和稳定进程出生身份。支持桌面 session 服务及 `app-*.scope`。环境策略存在时，各 checkpoint 共用该策略；旧部署的模型登记字段继续提供兼容输入。

模型登记中的 `gpu_uuid` 和 `min_free_memory_mib` 分别声明目标设备与所需显存。替换预检仅计入当前服务已持有锁、且仍存活的自有 worker 将释放的显存。实际获取锁时重新核验设备、剩余显存与计算进程。

## 切换与失败回执

1. 核对请求中的模型与制品指纹。
2. 预检候选制品、固定依赖环境和 GPU 占用。大权重复用生产者身份与文件大小，小文件核对内容摘要。
3. 预检失败时保留当前模型，记录 `last_prepare_error` 与 `last_prepare_admission`；当前已加载模型的准入信息由 `gpu_admission` 提供。
4. 正常退出旧 worker，获取共同 GPU 锁，加载候选并完成预热。
5. 加载失败时记录实际错误与 worker 状态；加载成功后发布新模型身份和就绪事实。

`GPU_BUSY`、`GPU_MEMORY`、`GPU_IDENTITY` 回执包含对应 GPU、显存要求和占用进程信息。进程信息使用 PID、用途类别、可执行文件和 cgroup，不保存完整私密命令行。

## 验收

回归覆盖跨模型目录互斥、失败释放锁、桌面重启与新 cgroup、外部计算进程拒绝、候选预检失败保留旧模型。制品接入验收使用独立测试模型 ID，硬链接复用实际权重，追加标识性元数据生成新指纹。

同一测试模型在元数据 revision 更新后再次生成不同指纹，分别验证首次发现和加载、相同指纹复用、同 ID 新指纹重新加载，以及原模型恢复。测试包保留来源标识，测试完成后将目录条目移至验收证据目录。

2026-11-14 的实现、原始回执和实际加载结果位于 `.evidence/gpu-admission-20261114-v1/`。该验收通过模型服务入口进行加载与预热，保留当前 SDK 实例和控制代次。
