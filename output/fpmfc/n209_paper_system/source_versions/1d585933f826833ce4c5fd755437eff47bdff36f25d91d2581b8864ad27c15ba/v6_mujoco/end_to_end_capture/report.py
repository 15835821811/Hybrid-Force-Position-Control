"""Evidence-based report, static historical geometry diagnostic, scientific figures."""
from pathlib import Path
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from v6_mujoco.model import default_model_spec
from v6_mujoco.postgrasp.physics import joint_slices
from v6_mujoco.collision import signed_distance
from v6_mujoco.fpmfc.target import target_from_config
from .common import *
from .adapter import compile_model,initialize,addresses,extra_pairs

def historical_terminal_geometry():
    # Read-only static diagnostic in isolated MjData. NEVER an N205 executed state.
    source=arrays(C1/"precontact/C1/trace.npz");m=compile_model();d=initialize(m);oldmodel=default_model_spec().compile_model();ids=addresses(m)
    for name in ids:
        if name=="target_free_joint":continue
        q,v=joint_slices(oldmodel,name);d.qpos[ids[name]["qpos"]]=source["qpos"][-1,q];d.qvel[ids[name]["qvel"]]=source["qvel"][-1,v]
    sample=target_from_config(preconfig()).sample(8);quat=Rotation.from_matrix(sample.center_rotation_world).as_quat()
    d.qpos[ids["target_free_joint"]["qpos"]]=np.r_[sample.center_position_world_m,quat[3],quat[:3]]
    mujoco.mj_forward(m,d)
    result={"scope":"isolated static replay of HISTORICAL C1 terminal geometry at 8 s; zero integration steps; NOT the unexecuted N205 t=8 state and never injected into its plant","source_trace_sha256":digest(C1/"precontact/C1/trace.npz"),"distances_m":{p.name:signed_distance(m,d,p) for p in extra_pairs(m)},"diagnostic_identity":identity([__file__])}
    save(ROOT/"historical_terminal_geometry.json",result);return result

def figures(a,pairs):
    out=ROOT/"figures";out.mkdir(exist_ok=True);t=a["time_s"];paths=[]
    plt.rcParams.update({"font.size":11,"axes.titlesize":12})
    def finish(fig,name):
        for ext in ("png","pdf"):
            path=out/(name+"."+ext);fig.savefig(path,dpi=160,bbox_inches="tight");paths.append(path)
        plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True,constrained_layout=True)
    axes[0].plot(t,1000*a["approach_position_error_m"]);axes[0].set_ylabel("Flange reference error [mm]")
    axes[1].plot(t,np.rad2deg(a["approach_rotation_error_rad"]));axes[1].set_ylabel("Reference SO(3) error [deg]")
    axes[2].plot(t,np.rad2deg(a["approach_shape_error_rad"]));axes[2].set_ylabel("Arm-shape error [deg]");axes[2].set_xlabel("Absolute time [s]")
    for ax in axes:ax.grid(alpha=.25)
    fig.suptitle("Actual approach tracking to stop; no postgrasp holding interval")
    finish(fig,"approach_tracking_errors")
    fig,axes=plt.subplots(2,1,figsize=(12,8),constrained_layout=True)
    for i,pair in enumerate(pairs):
        axes[0].plot(t,1000*a["extra_pair_distances_m"][:,i],alpha=1 if "link7_collision__tumbling" in pair.name else .28,label="Link 7 / target cube" if "link7_collision__tumbling" in pair.name else None)
    axes[0].set(ylim=(-1,100),ylabel="Robot-target distance [mm]",title="All added pairs; highlighted trigger pair");axes[0].legend();axes[0].grid(alpha=.25)
    i=[p.name for p in pairs].index("link7_collision__tumbling_target_geom");mask=t>7.2
    axes[1].plot(t[mask],1000*a["extra_pair_distances_m"][mask,i],label="Measured distance")
    axes[1].axhline(1,color="#b54343",ls="--",label="Frozen margin: 1 mm");axes[1].axhline(.99,color="#d9883f",ls=":",label="Stop threshold incl. 10 um tolerance")
    axes[1].scatter(t[-1],1000*a["extra_pair_distances_m"][-1,i],color="#b54343")
    axes[1].set(xlabel="Absolute time [s]",ylabel="Distance [mm]",title="Stopped at the first failing physical sample, before penetration");axes[1].legend();axes[1].grid(alpha=.25)
    finish(fig,"target_clearance_stop")
    fig,axes=plt.subplots(2,1,figsize=(12,7),constrained_layout=True)
    for key,label in [("target_omega_world_rad_s","Real free target"),("base_omega_world_rad_s","Free base"),("target_base_relative_omega_world_rad_s","Target minus base")]:axes[0].plot(t,np.rad2deg(np.linalg.norm(a[key],axis=1)),label=label)
    axes[0].set(ylabel="Angular speed [deg/s]",title="Approach only: detumbling stage was not reached");axes[0].legend();axes[0].grid(alpha=.25)
    axes[1].plot(t,a["contact_peak_force_n"],label="Pad / plate force");axes[1].set(xlabel="Absolute time [s]",ylabel="Contact force [N]",ylim=(-.1,1),title="No pad/plate contact detected before stop");axes[1].legend();axes[1].grid(alpha=.25)
    finish(fig,"angular_speed_and_contact")
    fig,axes=plt.subplots(2,1,figsize=(12,7),constrained_layout=True)
    for ax,key,limit,label in zip(axes,["linear_momentum_world_kg_m_s","angular_momentum_about_center_world_kg_m2_s"],[1e-4,1e-5],["P drift [kg m/s]","H drift [kg m^2/s]"]):
        value=np.linalg.norm(a[key]-a[key][0],axis=1);ax.plot(t,value);ax.set_yscale("symlog",linthresh=1e-16);ax.set_ylim(0,limit*3);ax.axhline(limit,color="#b54343",ls="--",label="Original gate");ax.set_ylabel(label);ax.grid(alpha=.25);ax.legend()
    axes[1].set_xlabel("Absolute time [s]");fig.suptitle("Whole recorded horizon, always referenced to t=0")
    finish(fig,"momentum")
    fig=plt.figure(figsize=(14,7));gs=fig.add_gridspec(1,2,wspace=.35,left=.04,right=.97);ax=fig.add_subplot(gs[0],projection="3d")
    for key,label,style in [("flange_position_world_m","Actual flange","-"),("approach_reference_position_world_m","Frozen C1 reference","--")]:
        xyz=a[key];ax.plot(*xyz.T,style,label=label)
    ax.set(xlabel="World x [m]",ylabel="World y [m]",zlabel="World z [m]",title="Approach trajectory");ax.legend()
    ax2=fig.add_subplot(gs[1]);error=1000*(a["flange_position_world_m"]-a["approach_reference_position_world_m"])
    for i,label in enumerate("xyz"):ax2.plot(t,error[:,i],label=label)
    ax2.set(xlabel="Absolute time [s]",ylabel="Signed flange reference error [mm]",title="Tracking error; NOT grasp holding error");ax2.grid(alpha=.25);ax2.legend();finish(fig,"end_effector_trajectory")
    fig,axes=plt.subplots(2,1,figsize=(12,7),constrained_layout=True)
    for i in range(7):axes[0].plot(t,a["joint_velocity_rad_s"][:,i],label=f"J{i+1}")
    axes[0].legend(ncol=7);axes[0].set_ylabel("Joint velocity [rad/s]");axes[0].grid(alpha=.25)
    for key,label in [("actuator_power_w","Actuator"),("passive_power_w","Passive"),("all_constraint_power_w","Constraint")]:axes[1].plot(t,a[key],label=label)
    axes[1].set(xlabel="Absolute time [s]",ylabel="Power [W]",title="Approach actively does work; no postgrasp dissipation claim");axes[1].legend();axes[1].grid(alpha=.25);finish(fig,"joints_and_power")
    save(out/"manifest.json",{"source_trace_sha256":digest(ROOT/"E_nominal/trace.npz"),"derived_identity":identity([__file__]),"files":identity(paths)})

def build():
    verify_design();ledger=read(ROOT/"run_ledger.json");results={};replays={}
    for item in ledger["formal_runs"]:
        s=item["scenario"];out=ROOT/("E_"+s);r=read(out/"metrics.json");check_identity(r["implementation_identity"])
        assert digest(out/"trace.npz")==r["trace_sha256"]
        from .runner import summarize
        a=arrays(out/"trace.npz");assert all(r[k]==v for k,v in summarize(a,r["status"]).items())
        v=read(out/"validation.json");check_identity(v["verification_identity"]);assert v["passed"] and v["trace_sha256"]==r["trace_sha256"]
        results[s]=r;replays[s]=v
    a=arrays(ROOT/"E_nominal/trace.npz");r=results["nominal"];m=compile_model();pairs=extra_pairs(m);i=[p.name for p in pairs].index(r["reason"])
    history=historical_terminal_geometry();figures(a,pairs)
    allscenes=config()["scenario_order"];per=lambda field:{s:results[s][field] if s in results else "NOT_RUN_UPSTREAM_FAILURE" for s in allscenes}
    matrix={"legacy_full_boundary_contract":"FAILED","restricted_fixture_domain":"inherited_verified_scope","precontact_reexecuted_from_home":True,"full_8s_approach_completed":False,"physical_target_from_t0":True,"single_continuous_plant":True,"no_terminal_state_injection":True,"fixed_latch_frames":True,"continuous_18s_completed":per("continuous_18s_completed"),"capture_gate_at_8s":{s:"NOT_REACHED" if s in results else "NOT_RUN_UPSTREAM_FAILURE" for s in allscenes},"model_latch_and_holding":{s:"NOT_EVALUATED" for s in allscenes},"detumbling_window_16_18s":per("performance"),"whole_horizon_momentum_and_safety":{s:"RECORDED_HORIZON_MOMENTUM_PASS_SAFETY_FAILED" if s in results else "NOT_RUN" for s in allscenes},"independent_replay":{s:replays[s]["passed"] if s in replays else "NOT_RUN" for s in allscenes},"end_to_end_model_validation":per("end_to_end_passed"),"zero_added_control_comparison":"NOT_RUN","hardware_load_rating_validated":False,"physical_gripper_capture_validated":False,"perception_robustness_validated":False,"real_time_ready":False,"campaign_outcome":"PHYSICAL_GEOMETRY_GATE_FAILED; dependent experiments correctly not admitted"}
    save(ROOT/"qualification_matrix.json",matrix);save(ROOT/"comparison.json",{"results":results,"C1_comparison":read(ROOT/"E_nominal/c1_comparison.json"),"historical_terminal_geometry":history,"attempt_count":len(ledger["formal_runs"]),"remaining_attempt_allowance_is_not_permission_to_change_physics":True})
    distance=1000*a["extra_pair_distances_m"][-1,i];old_distance=1000*history["distances_m"][r["reason"]]
    report=f'''# N205：连续接近—等效锁紧—消旋集成验证

## 实际结论

**本轮端到端验证未通过。** E_nominal 从机器人 home 和自由目标 t=0 连续积分到 **{r['end_time_s']:.3f} s**，在 `link7_collision` 与 `tumbling_target_geom` 的距离安全门停止。没有到达 8 s 捕获检查、没有激活锁紧，没有运行 10 s 消旋，也没有评价 [16,18] s。

触发样本距离 **{distance:.9f} mm**；前一物理样本距离 **{1000*a['extra_pair_distances_m'][-2,i]:.9f} mm**。新目标对的 1 mm 余量及 10 μm 数值容差已在首次正式积分前冻结。实际停止阈值为 0.99 mm。停止时距离仍为正，故这是**非预期几何接近安全失败，不是已经发生物理碰撞的证据**。目标立方体和这些机械臂对采用有符号距离监测，未宣称具有物理碰撞响应。

| 条件 | 真实时域 | t=8 捕获 | 18 s / [16,18] | 记录输入重放 |
|---|---|---|---|---|
| E_nominal | 0–{r['end_time_s']:.3f} s | 未到达 | 未完成 / NOT_EVALUATED | 通过，qpos/qvel/wrench差为0 |
| E_fine | 未运行 | 未评价 | 未评价 | 未运行 |
| E_light | 未运行 | 未评价 | 未评价 | 未运行 |
| E_heavy | 未运行 | 未评价 | 未评价 | 未运行 |

已用 **{len(ledger['formal_runs'])}/6** 次正式尝试。真实几何失败后，按任务规则不启动后续场景，不改变参考、捕获时刻、site frame、安全门或初态，不消耗“实现修复”额度寻找有利结果。可选 E_nominal_Z 未运行。没有重跑旧19项夹具或复制旧42个视频。

## 连续系统与控制适配

模型从已提交候选2复制，只恢复原 pad/plate 接触掩码并将命名 `postgrasp_latch` 初始关闭。质量、惯量、site局部坐标、solref/solimp/torquescale和原阻尼系数保持。目标自由关节与机器人自由基座从0存在，模型 nq=21、nv=19、nu=7。目标初始世界线速度和体坐标角速度按原配置设置一次。随后主 MjData 没有 qpos/qvel、time 或参考几何重写。

接近轨迹直接调用原 C1 轨迹生成器及 pairing manifest；T=8 s 和终端臂形来自源字段。原 HQP、反作用映射、20 ms速度斜坡、2 ms reference_q/逆动力学伺服保持。完整维度矩阵按名称索引，目标自由度对应的关节反作用映射行是零。所有优化、有限差分、servo 的 mj_forward 和观测均在复制数据上运行。正式植物只收到原七个 actuator ctrl，然后 mj_step 连续积分。

预定义后续状态为 APPROACH→CAPTURE_CHECK→LATCH→GRASP_VERIFY→DAMP_TRANSFER→HOLD。锁紧仅允许绝对 t=8 且捕获门通过，50 ms确认，最终窗口固定16–18 s；本次只执行了APPROACH，后续状态机尚无连续运行成功证据。静态事件单元测试不计作端到端锁紧成功。

## 接触、几何和源 C1 对照

原 pad/plate 从0起具有原材料和掩码。枚举证明关闭这两个geom掩码仅移除它们互相的交互；其余监测不变。原24对非接触检查仍采用40 mm门；新增19个机器人—目标距离对，包括全部 link0..7、基座及pad与目标立方体/板。只有pad—cube因与计划板正面重合，采用原2 mm接触穿透门；没有整段末端或目标豁免。

整个已记录时域没有检测到 pad/plate 几何接触、非零接触力或0.2 N阈值接触。目标解析诊断与实际自由运动的位置差为0，姿态误差在约1e-13 rad量级。全系统动量漂移峰值 P={r['max_P_drift']:.9g} kg·m/s，H={r['max_H_drift']:.9g} kg·m²/s，分别低于1e-4与1e-5门，始终以t=0为基准。

3681个共同时间样本中，新旧命名机器人位置最大差5.5881e-11、速度最大差8.1701e-10；没有超过1e-8/1e-7诊断阈值的首次偏离。369次HQP均成功。因此当前证据将停止定位到**新增的目标几何覆盖**，而非自由目标回填、接近跟踪失效或接触后动量交换。

额外只读静态诊断：将**历史**C1的t=8几何放入隔离数据对象，link7—cube距离为 **{old_distance:.6f} mm**，link7—plate也约−0.293 mm。这显示旧接触关闭参考的终端几何存在干涉。它不是本轮未执行的t=8真实状态，也没有被注入正式植物；不据此宣称本轮已经撞入或完成捕获。

## 测试与证据身份

两个短单元批次分别69个物理步及9步记录输入重放：验证全维度/命名地址、目标自由传播和初速度响应、初始weld关闭、隔离计算不改真实状态、原C1参考、捕获失败拒绝、静态事件零状态跳变、掩码交互清单、控制节拍以及短样本不能通过最终窗口。第二批增加原终端门观测后再次验证；不属于正式轨迹或参数搜索。

E_nominal独立重放从本次t=0完整integration state初始化一次，仅施加记录ctrl与明确事件，之后只mj_step；没有调用控制器选择，没有每步覆盖记录qpos/qvel。全部严格状态、wrench、动量和距离比较通过，并在同一末样本复现同一安全失败。未执行的fine和密度场景无重放或步长敏感性结论。

预注册设计、模型/初态/接触对契约、每次实际实现身份、运行账本、trace、任务残差、events、switch_audit、validation及历史诊断均在 `output/fpmfc/n205_end_to_end/`。`switch_audit`明确为NOT_REACHED，没有伪造t=8快照。模型/引擎定义依据 [MuJoCo 3.3.2 Computation](https://mujoco.readthedocs.io/en/3.3.2/computation/) 与 [API Functions](https://mujoco.readthedocs.io/en/3.3.2/APIreference/APIfunctions.html)。

## 八项最终回答

1. **是否从home重算8 s？** 从home真实重算至7.362 s；完整8 s未完成，未播放或导入末态。
2. **目标是否从0为自由动力学对象？** 是；全程由引擎积分，解析运动只作独立诊断。
3. **是否连续植物、无注入和重标定？** 已执行区间是一套真实model/data，无末态注入或site重拟合；锁紧阶段未到达。
4. **提前接触/非预期碰撞？** 未检测到计划接触；link7—目标距离触发安全门，仍正间隙，未称已碰撞。
5. **8 s捕获、锁紧、50 ms确认？** 均未到达、未评价。
6. **16–18 s消旋保持？** NOT_EVALUATED，没有使用短样本最大值判通过。
7. **全流程安全、P/H、重放？** 已记录区间动量及独立重放通过，距离安全失败；完整18 s未验证。
8. **密度场景和验证边界？** fine/light/heavy按准入规则未运行，不能继承旧L/V作为本轮连续验证；硬件、感知、真实夹爪和硬实时均false。

## 媒体、复现与下一项缺失

视频只从本次已重放trace绘制，长度约7.362 s，画面标注实际绝对时间、未锁紧局部时间N/A、APPROACH/FAILED、真实目标角速度、接近参考误差；不补齐18 s、不拼接旧抓后视频。交付总览、接口近景、五视角合成、随基座体侧镜头，以及独立末端轨迹/误差和安全科学图。图中的接近跟踪误差与尚未适用的抓后保持误差区分。

统一入口：`--prepare`、`--scenario nominal`、`--all --resume`、`--validate-only`、`--report`；另有 `python -m v6_mujoco.end_to_end_capture.render`。resume只读取本次已存在身份并保留物理失败，不隐式重试。新增验证调用仅增加重放账本。

下一项缺失首先是**末段轨迹/工具与目标几何兼容性**。这需要下一独立设计任务决定；本轮不调整姿态、臂形、T或放宽距离门。旧满边界资格仍FAILED，受限夹具资格仅继承原范围。

离线物理循环耗时{r['elapsed_wall_s']:.3f} s；控制计算平均{1000*r['control_latency_s']['mean']:.3f} ms、p99 {1000*r['control_latency_s']['p99']:.3f} ms、最大{1000*r['control_latency_s']['max']:.3f} ms，未满足硬实时证据要求。原始日志不作硬件计时承诺。

按用户附件末尾明确要求上传独立N205分支；不合并、不移动旧实验分支。
'''
    (PROJECT_ROOT/"paper/N205_END_TO_END_CAPTURE_DETUMBLING_REPORT.md").write_text(report,encoding="utf-8",newline="\n")
    print(json.dumps({"outcome":matrix["campaign_outcome"],"formal_attempts":len(ledger["formal_runs"])}),flush=True)

if __name__=="__main__":build()
