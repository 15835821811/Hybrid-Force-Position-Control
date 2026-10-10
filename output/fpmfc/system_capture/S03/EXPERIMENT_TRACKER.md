# S03 实验执行记录

所有原始失败保留；B00是严格重放复用，不是新增独立样本。

|运行|状态|实际终止(s)|锁紧(s)|物理步|CPU(s)|
|---|---|---:|---|---:|---:|
|B01|NO_VERIFIED_CONTROL|5.900|None|2950|196.438|
|D00|IMPLEMENTATION_ERROR|6.440|None|3220|139.844|
|D00_R1|PREDICTION_REJECTED|7.240|None|3620|172.016|
|D01|PREDICTION_REJECTED|6.440|None|3220|149.234|
|P00|PREDICTION_REJECTED|8.040|None|4020|191.469|
|P01|PREDICTION_REJECTED|7.240|None|3620|176.875|
|P02|PREDICTION_REJECTED|6.440|None|3220|158.922|
|P03|PREDICTION_REJECTED|6.440|None|3220|156.594|

B00：复用 S02/E0_C2，7.914s 锁紧，27.914s 完成，13957步；两类严格重放通过。
D02/D03：未使用；两版实现额度用于V1与仅I/O修复的V2，只有一组科学参数。
R00/H1：名义和H2 B2均未获完整任务准入，按条件跳过。
候选、预测步和总CPU见最终 compute_profile.json；重放状态见 qualification.json。
