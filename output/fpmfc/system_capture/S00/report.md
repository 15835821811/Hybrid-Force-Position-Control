# S00 统一契约、基线适配与证据索引

状态：implementation = PASS。本轮只执行 S00，从 `007abc48fe3d804a96d4feee0a1c304444048669` 建立 `codex/system-s00-contracts`，默认仅本地提交。

## 修改与边界

新增 `v6_mujoco/system_capture` 的数据契约、注册、组合适配、时钟调度、归档重放、剖析和证据报告。旧控制器/观测器/模型不移动、不改写；临时运行目录装载各轨迹原有源码的精确字节，不修改全局旧模块，不用当前版本冒充历史版本。原命令入口、碰撞/工具、执行器、载荷、任务门和所有旧结果/媒体保留。

`FrameConvention` 固定点线速度在前、角速度在后，wrench为力在前、力矩在后，SI、主动列向量旋转。旧基座广义速度单独拆为世界系线速度和体坐标角速度；需要世界系 twist 时显式旋转。SensorPacket 的机器人采样与送达/控制时刻保留，目标观测各自带采样/送达时刻；旧日志没有独立生成时刻，使用 null，不虚构时间。状态协方差、数组及嵌套快照使用独立不可变字节缓冲。

ScenarioSpec分开硬件、真值、先验、传感器、算法、任务、数值。控制器工厂只接受不含truth/场景标识的ControllerSetup和公开packet。静态源码接口与差分测试是软件边界证据，不是OS级隔离证明。四类门明确区分，8 mm预测跟踪门是算法接受条件，不能当物理碰撞裕量。

## 执行和结果

新物理尝试 **0**；两条既有轨迹的重放 **4/4**；合成/接口测试 **23**；预选同批控制状态 **16/16**；静态/单状态预算上界 **41/100**。记录的总过程CPU为 **450.469 s / 14400 s**，包含重放内的预测，父进程与子进程分开汇总。代码编辑和只读shell查询不在过程CPU统计范围内。

两种重放分别验收。执行器重放仅t0初始化一次，重施记录力矩/锁紧事件，检查全时域qpos/qvel和接触掩码。决策重放中，适配器每包调用一次原归档控制器；逐包比较原返回与统一proposal，并对独立原记录核对tau、触发、状态机、估计、参考、guard、进度、参数和终止原因。不存在为比较而额外运行两条新完整决策轨迹。详细误差和源码身份见 baseline_equivalence.json。

S01保持NO_VERIFIED_CONTROL。其已知无效t0协方差日志别名仅按原TS_AUDIT使用同tick独立trace核对；原失败验证和原control_states文件保持不变。新快照隔离测试实际修改旧滤波器内部P，证明先前统一快照不随之改变。

distance gradient、HQP、state filter、predictor在预先登记的同一批状态原调用中剖析，失败/无效返回也计入。p50/p95/max、调用数量、环境在runtime_profile.json；嵌套inclusive耗时不能相加，含插桩开销，不能作为实时就绪证据。

## 证据与未支持结论

legacy_inventory.csv区分在用组件、仅被继承的工具函数和不在线的历史接触导纳。source_traceability.csv与reproduction_map.csv对齐梁斌论文的公式、图表、等价条件与迁移差异。型Ⅰ为基座姿态、型Ⅱ为臂形，均不是惯性估计。原文式(27)的末端零空间不自动推出基座无反作用；式(34)存在需澄清的导数/位移量纲约定。paper_compat_srs与paper_compat_reserved仅注册，返回NOT_IMPLEMENTED。

paper/system_framework建立三类证据矩阵和按来源/待验证问题组织的空论文骨架。当前鲁棒改进NOT_SUPPORTED、辨识控制收益NOT_DEMONSTRATED、硬件和实时NOT_EVALUATED。原N073仅支持固定完整方法规划轨迹上的执行级消融结论；N209仍为PARTIAL_OPERATING_DOMAIN_MISMATCH。S00不重新审计这些问题，也不将重放通过写成物理任务成功或方法收益。

## 验收与交接

逐要求核对见completion_audit.json。implementation、physical_model、reproduction、task、method_benefit、sensor_domain、parameter_identification、control_benefit与next_stage_admission分别写入qualification.json。下一阶段仅推荐S01，等待新授权。handoff.json保存输入提交和输出内容哈希；最终本地提交SHA存入Git公共目录旁路索引，避免Git提交包含自身SHA。未推送、未合并、未执行S01-S08。

## 报告修正记录

首次报告发现原失败验证应引用 D03_V1_S01/validation.json，而非不存在的 R03 文件。仅修正报告引用与显式报告身份核验；控制器、契约、重放器及已执行测试源码均未改变，四次重放保留原冻结身份，不补跑。原报告/核验源码与修改身份保存在 reporting_amendment/。接近预测实测计数 79300 步；制动预测按原始安全候选日志和未变循环推导 30060 步，合计 109360 步；推导部分不冒充直接插桩计数。详见 prediction_accounting.json。
