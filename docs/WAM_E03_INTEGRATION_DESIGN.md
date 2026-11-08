# WAM E03 接入设计与验收计划

状态：WAM E03 的ARX13有界三段、运动中Stop、原始ROS录制及输入重建已通过；普通持续/变步数现场覆盖、正式性能预算和任务精度仍待验收。日期：2026-11-07。负责：本聊天。
用户已确认新增 WAM 支持、保留 π0.5，首个候选为 E03 step_003190、2 步去噪。
本设计不授权接管同事控制服务，也不复用其面板的控机、复位或自动回传逻辑。
当前 ROS 迁移继续推进；WAM 的接口、UI 与用例先审阅再实现。

## 2026-11-07 已批准的模型家族与双后端

同日追加E04_2 / step_002902上线。已核验H32/30Hz、RTC关闭、embodiment_id=2、
夹爪m，以及prepared manifest的materialized_next_observation语义；自身统计量SHA为
`e60f56427879ed3830c4da5c89d7f365962c45628b6581b5acea9b50ca133706`，不复用E03统计。
用户进一步批准通用WAM模型描述：每个checkpoint独立绑定权重、统计、文本和配置，
Owner/翻译消费现有action_contract及原生坐标绑定，model_id只用于身份关联。
不增加SDK窗口字段，不改阈值；同步升级生产者/消费者，旧在线组合在绑定时拒绝，
历史产物按文件schema读取。发布遵守既有受控维护流程，不热改活动Owner。
当前E03原生Torch已用原训练快照在CPU严格恢复1653项模块状态并逐项核对数值，
未导入anyinfer、未初始化CUDA；GPU前向与服务接入仍待完成。

用户确认页面按“模型家族 → 执行后端 → checkpoint”组织：π 使用 Torch / Triton，
WAM 使用 Torch / SO 算子加速。SO 的协议值继续为 `anyinfer`，不因显示名称改变身份。
I03 保留 E03 模型 ID，以不同 `bundle_digest` 冻结 Torch / anyinfer 的运行时与制品；
WAM 描述中的 `backend` 允许 `torch` 或 `anyinfer`。后端身份在模型绑定时确认，
Owner/SDK 窗口不增加字段。切换继续使用现有模型生命周期，不并行占用同一GPU。

Torch 使用原生 PyTorch 前向，不导入 anyinfer 扩展；SO 使用现有已登记引擎。
两者复用三相机/状态、统计量、文本、seed、2步去噪、32×14输出、首点1/30秒、
2.5秒观测/结果有效期和现有翻译/录制。有效性规则与阈值保持。
当前实际控机资格属于已部署SO路径，不能据此宣称Torch后端已通过。

开发顺序：先更新目录和W14–W17用例，再实现两级UI与目录过滤；随后完成Torch
严格加载及固定输入验证，比较同输入下两后端的输出与首次/热调用耗时，最后发布。
验收覆盖家族/后端不串选、同模型跨后端身份与迟到结果隔离、Torch不依赖SO、
实际网络输入重建、固定输入数值与时延。新增模型预测不扩大本次真机动作预算。

2026-11-07用户批准WAM原生绑定：Infer从冻结部署一次构造
`workbench.wam-native-coordinates.v1`，只核验robot_id、sdk_module字节摘要、
gripper_calibration及三路camera_serials。profile_provider仍返回(profile,binding)，
binding复用calibration_profile_digest，不新增身份或SDK窗口字段；外参不参与WAM转换。
profile中的gripper.max_opening_mm按left/right保存各侧当前标定范围，Infer和Owner
核验相同标定；适配器与Guard均沿用2mm裕量。ARX13当前范围80/89mm，对应模型
命令上限78/87mm；输入夹爪state不按模型命令上限截断。π0.5原有标定与83mm旧边界
保持原语义。W13用例覆盖完整依赖绑定、双侧投影、SDK/设备/相机身份拒绝；完整
WAM32 Run和推理中Stop继续使用独立Owner与常驻录制服务进行CPU验证。
同日用户明确确认现场双臂支撑，授权本次旧Owner/SDK异常退出及重接；该次保存
exceptional_retirement=true、sdk_close_confirmed=false，不冒充正常SDK释放。

当前实现与资格入口为 `.evidence/wam-arx13-acceptance-20261107-v1/progress.json`；
开发Map按实际源码与回执更新。用户已恢复ARX13有界验收并确认task1274正常交接：
规则轨迹逐关节幅度不超过3°，FK/场景核验后选向上、向前方向；模型最多执行前3源秒，
执行前检查实际预测和现场。首次录制拒绝和旧writer占位的失败原件保留；用户确认
现场支撑后完成了一次明确标记的异常退役，没有把它记作正常SDK释放。后续有限段
供给故障同样保留；用户另行批准当前位置正常释放/重接，左右quiesce/retire与guard
disarm已确认。当前修复Owner健康静止、自动输出关闭；热模型/录制服务保留给用户试用。

本轮ARX13实际三段Run为`c0fd801fb705942759b149973307069b`：每段15步、0.5源秒，
实际运动0.6365/0.5000/0.5347秒，段间等待共1.9327秒，均满足单段源时长×4。
Stop Run为`b1b5c590f1dc74cb28ad6b9a047be486`：首段执行约50ms时请求网页Stop，
约59.9ms观察到SDK保持确认，之后600ms下发次数保持3，反馈健康静止；不宣称有
firmware ACK或外部空间停距测量。此前失败Run仍标原始incomplete，没有改写其结论。
两条新Run均完整封存：三段Run保存429条state/1407控制事件/3组模型输入输出，
Stop Run保存166条state/276控制事件/1组模型输入输出；三相机各有连续原始段与回放。
加上首个失败Run，共6次实机调用的42组网络输入张量可从原始样本逐位重建，输出
反归一化与保存的action14完全一致；此核验只用CPU，不重跑模型、不改原manifest。
结果列表3条模型Run、9条MP4 Range接口已验证206。

SDK端点目标与投影后的模型末点误差小于1e-12 rad/SDKm。段尾首个反馈最大关节
误差0.39–0.41°，夹爪约0.13–0.20mm；这些是指定时刻的编码器误差，不是最终停稳
或外部指尖/杯子精度。本轮未通过“所有关节≤0.2°”资格，也未验收抓杯任务成功率。
SDK JointController没有forward_kinematics，现有可选FK字段如实标无效；运动前的
名义检查使用固定X5 URDF并与独立SDK求解器对照，不能冒充外部实测位置。

G34按批准保留诊断：三段全录制区间state周期P99约12.38ms/max26.78ms；正式预算
尚未冻结，没有新增运行时硬停门槛。当前ARX13候选配置仍是3×15步；普通持续模式、
1–32可变步数的现场覆盖及正式运维代理配置不因本次有界通过而自动获得资格。
实际资格、源码角色、所有失败、资源与遗留统一见
`.evidence/wam-arx13-acceptance-20261107-v1/final-bounded-acceptance.json`及开发Map。

7组网络输入已通过原部署cu130运行时CPU运算与独立CPU构建的逐位对照；
独立Owner、实际InferenceWorkflow与常驻录制器的WAM32完整CPU Run通过，模型为CPU替身。
证据为 `cross-build-input-parity.json`、`wam-current-native-run.log`，均在
`.evidence/wam-design-20261106-v1/`。
ARX13真实E03使用原正式录制第1组chunk1，32次调用重复输出误差为0；加载约26.7秒、
首次请求约679毫秒，31次重复请求中位137毫秒、P95约140毫秒。同机独立运行原面板
公式，7组输入摘要与raw80/action14输出均完全一致。与历史录制输出的最大差异为
关节0.009886rad、夹爪0.421mm，不能称跨部署历史输出逐位一致；此次同机对照证明
适配器保留原公式。证据为当前进度目录的 `gpu-fixed-input.json` 与
`same-gpu-reference.json`。上述测量没有下发模型动作；运动中Stop/G34仍未验收。
I03按批准仅在模型绑定时传入原始 `normalization_json`，以既有SHA核验；
业务侧预处理独占消费，Owner/SDK及命令窗口不接收统计量副本。

## 已核验来源

ARX6 上的部署根为 `/home/wujie/work/liu.zhecheng`，不是本机的 coffee_make。
模型包为 `infer/anyinfer-3090-9838bce-e03`；面板模板为其
`runs/e03-10step-panel.p_bm0391/live-template`。该来源核验在ARX6只读进行；后续GPU及
有界控机试验使用ARX13。部署包不包含训练源码；训练仓库及原始Run已单独核实，见下方时间语义。

| 内容 | 核验结果 |
| --- | --- |
| 模型配置 | `model_config.json` 明确 E03/step_003190 与归一化 SHA |
| 去噪 | 包与面板模板 YAML 相同，`num_denoise_steps: 2` |
| 权重 | joint_model.pt完整SHA与原训练COMPLETE一致；三编码器与制品注册表摘要一致，验收文件留在ARX13 |
| 归一化 | `phoenix.joint14.normalization.v1`，state/action 各14维 mean/std，夹爪单位 m |
| 原始图像顺序 | 头部、左腕、右腕；BGR 原图先转 RGB，再 bicubic 到256×256 |
| 网络输入 | 两路 `[1,3,3,1,256,256]` BF16 图像，80维 proprio，文本特征与 mask，video/action noise |
| 有效关节槽 | `[0,1,2,3,4,5,16,40,41,42,43,44,45,56]`；其余维度为零 |
| 输出 | 网络 `[1,32,80]`；抽取14维后以 action std/mean 反归一化为 `[32,14]` |
| 文本 | task_context.pt 为缓存特征；原文未知，不能在UI伪造提示词文字 |
| 计算图 | 示例支持图；实际面板 policy.py 当前明确 `use_cuda_graph=False`，不能宣称已在线复用图 |
| 时间 | 包 README 的2步合成输入测量约260ms；用户提供的约957ms是10步历史口径，均不能替代本次真实输入测量 |

轻量身份：YAML SHA `070745fb834558c46203334d4a0fd1b1e9cc68b5805a1558f386c5290160aa30`；
归一化 SHA `b44b06204d6e49b771e02f1ad43c945877eeec49d95d0e3eb082ccc79019dc33`；
提示词缓存 SHA `e7b950c5e45302bf5aab4da4148445966dbda5d7daa2c9e2659df196170d18ac`；
policy.py SHA `6587daab4c35c0f8c913e914bb284c70be7cc11c260682bd3a721eb23782433b`。

### E03训练来源与首点时间（2026-11-06已核实）

用户提供的原始Run位于3090可读的
`/mnt/nas_data/zhecheng.liu/05_phoenix_mwa/exp/20261103/coffee_stage1_c1c2_1epoch_zero1_c3norm_v1`。
`resolved_config.json`引用的prepared manifest声明30Hz、`action_abs_joint`和
`materialized_next_observation`；训练contract固定embodiment_id=2、32步输出和相同归一化。
step_003190的COMPLETE与训练contract一致，manifest摘要为
`a2b9ab978ef5d9671f83578cfaace57cfd426ae14c941ee82a03f179c01ee4dd`。
ARX6上joint_model.pt完整SHA256为
`0aa639846bc277eb1104f811aa82bb122b1b2c466c78aea2e808bc51bc2daf7d`，
与COMPLETE中的weights.pt一致，大小均为12045734252字节；归一化摘要也一致。

抽查3个episode、1287对相邻样本，`action[t] == state[t+1]`的最大绝对误差为0，
间隔约1/30秒。因此该E03微调模型的action[0]是观测后1/30秒的目标，action[i]对应
`(i+1)/30`秒；15步覆盖0.5秒。不能把第0个输出当成当前观测状态，也不能再在已物化
标签上增加一行偏移。此结论绑定本Run和上述权重，不推广到main的其他数据profile。
本轮仅做文件摘要和数据抽查，没有加载模型、运行推理或调用GPU。轻量回执见
`.evidence/wam-design-20261106-v1/e03-training-timing.json`。

## 建议边界和执行顺序

1. 在现有 Infer 模块内增加 WAM provider，复用共享模型生命周期、ROS 输入、翻译、Guard 和常驻记录。π0.5模型目录和服务实现保留；GPU装载按现有独占与模型选择约定执行。
2. 将模型能力中的 action horizon 从固定60改为已绑定的H；π0.5仍为60，WAM为32。返回数组必须正好为H×14，不补零到60，也不截掉模型原始输出。
3. 复用执行步数入口，建议默认仍为15；WAM可选1至32，π0.5可选1至60。源时长仍为执行步数÷30，单段最长仍为源时长×4。是否改WAM默认值需审阅确认。
4. WAM provider 完成BGR/RGB、256缩放、两路BF16图像、14→80槽位、state标准化、噪声与文本绑定；输出侧只抽槽并反归一化一次。后续仍由共享翻译模块做设备单位与已批准规划。
5. 先完成CPU契约、固定输入与录制重建验证，再在已分配GPU上做固定输入数值/耗时验收，最后才准备由用户开始的真机验收。

公共ROS输入已经输出RGB，因此WAM从RGB开始缩放，不能再执行面板中的BGR反转。
WAM声明 `history_steps=0`，π0.5保留各checkpoint既有历史需求。
噪声使用请求中明确记录的seed，按参考的video_noise、完整32×80 action_noise顺序生成，再清零无效槽；不在provider内部额外叠加面板chunk序号。

## 接口变更候选

| 接口 | 差异、消费者与用途 | 传递频率和兼容性 |
| --- | --- | --- |
| 模型目录／I03 | family、后端与checkpoint/config/norm/text身份，H=32、30Hz、14轴顺序/单位 | 选择与绑定时一次确认；复用bundle摘要，不增加重复模型身份 |
| I02 | 输入管理仍提供同一原始图像/state引用；模型专属预处理由Infer承担 | 每请求选择；不为WAM另开原始相机或state通道 |
| I03／ROS ModelResult | 现有 rows/columns 消费模型H；60×14容量可容纳32×14，去掉消费者固定60的判断 | 每结果；π0.5仍严格要求60×14，不能把变长当成任意形状 |
| I03／模型输入 | 保持ModelInput的14维有效标准化BF16值，绑定确定的80维槽位映射，剩余维度必须为零；不扩大公共state维度 | 绑定声明布局；请求保存实际有效值与变换身份，可重建80维proprio，不能误称原始设备state |
| I01／执行 | execute_steps 上限从模型能力派生；denoise_steps=2为模型配置，不是控制速度或轨迹长度 | 下一Run冻结，运行中不修改；原门槛数值不自动放宽 |
| I14／录制 | 保存原图引用、实际预处理身份、BF16输入校验、80维布局、seed、去噪步数、原始/反归一化输出 | 记录器唯一落盘；优先验证确定性位精确重建，失败时另审派生tensor载荷方案，不能悄悄复制原图或接受近似重建 |
| I03／取消 | 延续request/Run/epoch与观测期限，旧模型迟到结果不得控机 | 每请求；同事面板的ROS控制owner与自动回传不进入工作台链路 |

2026-11-06用户确认WAM观测/结果新鲜度上限采用2.5秒；π0.5继续使用0.5秒。该差异随模型绑定冻结，运行中不切换；实时关节反馈年龄、相机配对跨度和运动保护继续保持原值。分别测输入配对、预处理、传输、模型、翻译与落盘队列，不把2.5秒当作控制反馈允许陈旧2.5秒。

## UI候选

- 模型选择显示π0.5和WAM；WAM条目标明E03/step_003190、2步去噪、32步输出和资格状态。未完成验收不能显示可控机。
- 执行步数与模型输出长度分别显示；切换模型后重新绑定并核验已有执行步数。
- 文本显示“已绑定缓存特征”及摘要，不编造未知原始提示词。
- 显示输入年龄、配对跨度、首次/重复模型耗时及结果年龄，避免混用等待时间和纯模型计算时间。
- 每Run结果显示模型身份、输入/输出/实际下发/未下发点与停止结果。切换回π0.5不复用WAM的统计量、文本或迟到结果。

## 验收用例（定义完成，尚未执行WAM模型）

| ID | 场景与通过条件 |
| --- | --- |
| W01 | E03权重、YAML、norm、text、运行库缺失或摘要不符时不可绑定；错误指向具体项 |
| W02 | π0.5与WAM目录共存；反复切换后H、统计量、文本和身份不串用 |
| W03 | WAM只接受32×14，π0.5只接受60×14；NaN、维度、单位或关节顺序错误均拒绝 |
| W04 | WAM执行1/15/32步均保留完整原始输出，实际选择数量一致，单段预算分别按N/30×4派生 |
| W05 | BGR→RGB、相机顺序、resize、BF16舍入、state标准化和14→80槽位与参考逐项一致 |
| W06 | 按原图引用重建实际输入，BF16位模式、mask、噪声/seed与参考一致；未通过则不能宣布输入可复现 |
| W07 | 固定真实输入、seed与2步去噪对比参考输出；首次/热调用耗时分开统计，记录具体运行版本 |
| W08 | WAM观测/结果超过2.5秒、π0.5超过0.5秒，或来源重启、模型身份变化均拒绝；实时反馈门槛不变，不把缺失数据改取最新帧 |
| W09 | 计算中Stop、模型切换或epoch撤销后，迟到输出不产生后续SDK命令 |
| W10 | 成功、失败、Stop均保存单一Run产物，模型目标、实际下发、未发点及反馈能按request/point关联 |
| W11 | UI正确显示两种模型能力、执行范围和资格；旧模型及历史结果可继续读取 |
| W12 | GPU不足、首次加载失败、并发装载与服务退出均保留可核验状态；不接管同事控制服务 |

## 开发文件定位候选

模型能力、目录与验证：inference_model.py、inference_transport.py、model_session.py。
WAM内部provider拟新增 `robot_workbench/wam.py`，服务启动优先复用既有入口，不另建控制面板。
变长结果与选择：ros_inference.py、inference.py、inference_owner.py、inference_workflow.py、motion_reference.py。
产物：ros_recording.py、inference_trace.py；UI：web/app.js、web/index.html。
新增文件与跨文件契约在本设计审阅后再进入实现Map，不创建空占位模块。

## 尚未闭合的事实

- 训练的action[0]时间语义已由原始Run核实为t+1/30；真实控制的发令和到位延迟仍须在接入后单独测量，不能把它们等同于训练标签时间。
- 2步真实输入的首次、热调用、P95/P99和完整观测年龄待测；2步合成基准不能资格真实Run。
- BF16预处理位精确重建与完整权重摘要待验；原始文本不可恢复时继续绑定已有缓存，不替换语义。
- 编译包代码基线可读说明为9838bce；完整Phoenix-MWA训练/采样源码尚未获取。
- 训练仓库可通过`git@gitlab-ssh.cloud.anyverse.work:fm-manipulation/phoenix-mwa.git`读取，本地只读checkout位于`/Users/wujie/Developer/work/anyverse/fm-manipulation/phoenix-mwa`。main的F15/F16包含不同标签约定；本次已依据实际E03 Run确认其微调标签，而非依据面板立即下发行为或泛化的main配置猜测。
