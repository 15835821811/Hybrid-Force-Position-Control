"""Cited inventory and evidence index; historical conclusions are not requalified."""
import csv
from .common import PROJECT,OUT,OLD,REGISTRY,read,save,sha

def csv_write(path,rows):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('w',encoding='utf-8',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

def write_evidence():
    inventory=[]
    entries=[
      ('v6_mujoco/adaptive_capture/plant.py','true physical plant, one initialization','YES via archived N209','flexiv_system','MjModel/MjData','seven actual inputs and preserved passive bodies','hardware equivalence'),
      ('v6_mujoco/adaptive_capture/sensors.py','declared sensor frontend','historical generator; replay reads its packets','flexiv_system','legacy SensorPacket','recorded channel and sample/arrival protocol','robust navigation certification'),
      ('v6_mujoco/adaptive_capture/state_estimator.py','N208 estimator and shared propagation','base propagation retained; update overridden in N209','flexiv_system','legacy StateEstimate','original estimator implementation','current S01 robustness'),
      ('v6_mujoco/feasible_capture/estimator_adapter.py','measurement-time estimator','YES via archived N209','flexiv_system','legacy StateEstimate','same-packet conditional estimator findings','new closed-loop benefit'),
      ('v6_mujoco/adaptive_capture/controller.py','baseline controller and seven-torque servo integration','YES superclass; n208 registry factory','flexiv_system','tau7/latch and control history','archived algorithm semantics','all domains passed'),
      ('v6_mujoco/feasible_capture/controller_adapter.py','N209 control extension and diagnostic HQP','YES selected runs','flexiv_system','tau7/latch/QP records','explicit scalar-governor prototype','robust improvement'),
      ('v6_mujoco/feasible_capture/progress_governor.py','finite prior lookahead','YES approach task ticks','flexiv_system','candidate log/accepted fraction','finite sampled model decisions','recursive safety or guaranteed fallback'),
      ('v6_mujoco/feasible_capture/reference_shaper.py','independent shaped target reference','YES; smoothing enabled in S01','flexiv_system','TrajectorySample','declared command-state filter','smoothed state may approve latch'),
      ('v6_mujoco/adaptive_capture/relative_reference.py','target transport plus fixed path','YES superclass','flexiv_system','TrajectorySample','relative path kinematic convention','target stops when progress stops'),
      ('v6_mujoco/fpmfc/controller.py','hierarchical velocity least squares','YES through GeometryHQP','flexiv_system','FPMFCResult','primary soft task and hard inequalities','all six base DOFs and shape independently commanded'),
      ('v6_mujoco/hierarchical_qp.py','shared QP and bounded joint commands','YES','flexiv_system','QPResult','solver feasibility in recorded scope','QP solvability equals physical success'),
      ('v6_mujoco/geometry_capture/planning.py','distance derivative and geometry HQP','gradient/HQP YES; planning runner NO','flexiv_system','distance rows/kinematic rollouts','isolated same-time distance gradient','new planning or experiment in S00'),
      ('v6_mujoco/fpmfc/shape.py','type-II arm shape coordinate','YES','Flexiv adapted from SRS concept','ArmShapeSample','declared arm shape','mass/inertia estimation is type II'),
      ('v6_mujoco/fpmfc/contact.py','legacy force/normal-admittance implementation','NO in selected N209 controller path','historical contact variants','ContactWrench/NormalAdmittanceState','component exists in historical branches','N209 already integrates force feedback compliance'),
      ('v6_mujoco/adaptive_capture/governor.py','post-latch prior braking governor','YES according to archived mission flags','flexiv_system','alpha/prediction records','original damping/loading behavior','new S00 braking performance'),
      ('v6_mujoco/adaptive_capture/inertial_estimator.py','causal inertial identification','YES shadow only','flexiv_system','pi/rank/local scaled covariance','finite-data observable directions','full-parameter certainty or control benefit'),
      ('v6_mujoco/adaptive_capture/information_gate.py','posterior-to-controller gate','YES; parameter_feedback false in selected runs','flexiv_system','model_pi/trust flags','feedback separation','identified trajectory is better'),
      ('v6_mujoco/adaptive_capture/known_model.py','independent prior model','YES','flexiv_system','new model with public prior','no plant inertia handle in controller construction','prior predictions exact'),
      ('v6_mujoco/feasible_capture/benchmark.py','historical N209 runner/validator','NO new campaign; rules read only','flexiv_system','NPZ/JSON/gzip','original replay tolerances and TS_AUDIT','run old experiment matrix'),
      ('models/flexiv_rizon4s_n206_tool_scene.xml','frozen physical geometry','YES','flexiv_system','MuJoCo XML','selected tool and robot geometry','hardware approval'),
      ('output/fpmfc/n209_paper_system/qualification_matrix.json','authoritative prior qualification','EVIDENCE ONLY','flexiv_system','JSON','PARTIAL_OPERATING_DOMAIN_MISMATCH','S00 wrapper changes performance verdict'),
      ('output/fpmfc/n209_paper_system/qp_failure_diagnosis.json','historical H2/S01 hard-set classification','EVIDENCE ONLY','flexiv_system','JSON','frozen linear hard set conflict','all future planners impossible'),
      ('output/fpmfc/n209_paper_system/uncertainty_floor.json','conditional Riccati/guard bound','EVIDENCE ONLY','specified old sensor/filter','JSON','implemented gate allowance incompatibility','universal sensor precision limit'),
      ('output/fpmfc/n209_paper_system/sensor_reference_audit.json','same-packet S01 estimator statistics','EVIDENCE ONLY','specified old sensor/filter','JSON','read-only changed estimator comparison','independent robot success'),
      ('paper/N073_CONTROLLER_ABLATION_REPORT.md','original fixed-plan ablation report','EVIDENCE ONLY','historical Flexiv precontact','Markdown + linked raw summary','shape term mattered on its frozen candidates','replanned end-to-end superiority; base-reaction benefit'),
      ('output/fpmfc/n206_geometry_capture/selected_design.json','tool geometry contract','EVIDENCE ONLY','flexiv_system','JSON','20 mm extension, 19.8 mm front offset','full-boundary hardware qualification'),
    ]
    for path,role,online,domain,fmt,supported,excluded in entries:
        inventory.append({'file':path,'sha256':sha(PROJECT/path),'responsibility':role,'online_use':online,'model_dependency':domain,'output_format':fmt,'supported_conclusion':supported,'cannot_inherit':excluded})
    csv_write(OUT/'legacy_inventory.csv',inventory)
    pdf=next((PROJECT/'reference_papers').glob('*.pdf'));pdfrel=pdf.relative_to(PROJECT).as_posix()
    mappings=[
      ('P1 Eq10-17 pp961-962','v6_mujoco/hierarchical_qp.py','momentum-consistent generalized Jacobian, declared frame and momentum assumptions','Flexiv MuJoCo mass matrix and measured drift, not source SRS parameters','DECLARED_ADAPTATION','reaction_velocity_map; type I is base attitude'),
      ('P1 Eq18-25 pp962-963; Fig2/3','v6_mujoco/fpmfc/shape.py','nonsingular shoulder-elbow-wrist geometry','Flexiv joint layout and numerical model-specific Jacobian','DECLARED_ADAPTATION','type II is arm shape; neither type denotes inertial estimation'),
      ('P1 Eq26 p964','v6_mujoco/fpmfc/target.py','capture-point velocity and lever arm at same instant','current N209 uses a frozen relative C1 path; no new capture-time optimization','SOURCE_LIMITED','geometric objective alone does not prove impact/capture success'),
      ('P1 Eq27-31 p964','v6_mujoco/fpmfc/controller.py','primary task nullspace under rank assumptions','hierarchical QP with hard bounds and finite tolerances','SOURCE_AMBIGUITY','Jg*N=0 preserves endpoint velocity; it does not by itself prove base reaction A*N=0'),
      ('P1 Eq32-33 pp964-965','v6_mujoco/fpmfc/contact.py','matching wrench/pose coordinates and discretized impedance','historical normal-admittance component is NOT online in selected N209','DECLARED_ADAPTATION','position correction is not a velocity command; S04 integration pending'),
      ('P1 Eq34 p965','v6_mujoco/fpmfc/contact.py','consistent derivative order and units required','source displayed velocity/position increment combination requires S01 clarification','SOURCE_AMBIGUITY','not silently implemented as an equivalent equation'),
      ('P1 Eq35 p966; Fig6 p965','v6_mujoco/fpmfc/controller.py','capture time and terminal arm-shape optimization on source model','N209 scalar progress is a different extension; no source objective equivalence claimed','NEW_EXTENSION','objective weights and implemented units must be frozen before S01'),
      ('P1 Tables2/3 pp965-966','v6_mujoco/system_capture/registry.py','public DH/mass/inertia with explicit missing COM/frame assumptions','paper_compat_srs reserved NOT_IMPLEMENTED; Flexiv values not substituted','SOURCE_LIMITED','no claim of reproducing 15.6 s or 0.2686 rad'),
      ('P1 Fig9/10 pp966-967; conclusion p968','paper/N073_CONTROLLER_ABLATION_REPORT.md','same-model matched comparisons and failure denominator','N073 fixed planned trajectories; no end-to-end reoptimization per ablation','DECLARED_ADAPTATION','source numerical study explicitly limited to precontact; no inherited collision/detumbling result'),
    ]
    rows=[{'source_locator':loc,'source_file':pdfrel,'source_sha256':sha(pdf),'code_file':code,'equivalence_conditions':cond,'implementation_difference':diff,'classification':kind,'boundary':boundary,'evidence_file':code if code.startswith('paper/') else 'output/fpmfc/n209_paper_system/qualification_matrix.json'} for loc,code,cond,diff,kind,boundary in mappings]
    csv_write(OUT/'source_traceability.csv',rows);csv_write(OUT/'reproduction_map.csv',rows)
    paper=PROJECT/'paper/system_framework';paper.mkdir(parents=True,exist_ok=True);csv_write(paper/'reproduction_map.csv',rows)
    claims=[
      ('SRC_PM','SOURCE_REPRODUCTION','paper_compat_srs','SOURCE_LIMITED','type I=base attitude; type II=arm shape; source numerical study is precontact',pdfrel,'pp962-968','Source read; no SRS reproduction performed in S00'),
      ('ADAPT_N073','DECLARED_ADAPTATION','historical_flexiv','SUPPORTED_IN_RECORDED_FIXED_PLANS','shape removal affects frozen full-method candidates; base-reaction benefit not established','paper/N073_CONTROLLER_ABLATION_REPORT.md','Sections3-4','no independent replanning of each ablation'),
      ('EXT_ROBUST','NEW_EXTENSION','flexiv_system','NOT_SUPPORTED','robust capture improvement remains unsupported','output/fpmfc/n209_paper_system/qualification_matrix.json','C1_improved_robust_capture','seen nominal/H1 success cannot remove H2/S01 failures'),
      ('EXT_CONTINUOUS','NEW_EXTENSION','flexiv_system','SUPPORTED_ONLY_IN_RECORDED_IDEAL_SEEN_RUNS','continuous capture and stabilization in limited prior ideal cases','output/fpmfc/n209_paper_system/qualification_matrix.json','C2_continuous_task','not new success samples or blind validation'),
      ('EXT_ID_CONTROL','NEW_EXTENSION','flexiv_system','NOT_DEMONSTRATED','inertia identification control benefit not demonstrated','output/fpmfc/n209_paper_system/qualification_matrix.json','C3_identification_control_benefit','shadow identification and feedback benefit are distinct'),
      ('EXT_SENSOR','NEW_EXTENSION','flexiv_system','CONDITIONAL_IMPLEMENTED_GUARD_INCOMPATIBILITY','old sensing/filter model has a conditional guard allowance floor','output/fpmfc/n209_paper_system/uncertainty_floor.json','minimum_velocity_3sigma_lower_bound_m_s; assumptions','not universal sensing impossibility'),
      ('EXT_HARDWARE','NEW_EXTENSION','flexiv_system','NOT_EVALUATED','real hardware/interface certification absent','output/fpmfc/n209_paper_system/qualification_matrix.json','hardware_qualification','full 50N/2Nm boundary failure retained'),
      ('EXT_REALTIME','NEW_EXTENSION','flexiv_system','NOT_EVALUATED','real-time readiness unverified','output/fpmfc/n209_paper_system/qualification_matrix.json','realtime_ready','instrumented S00 profiling is not real-time certification'),
    ]
    csv_write(paper/'claim_evidence.csv',[{'claim_id':i,'evidence_type':kind,'model_domain':domain,'status':status,'claim':claim,'evidence_file':file,'field_or_locator':field,'sha256':sha(PROJECT/file),'limitation':lim} for i,kind,domain,status,claim,file,field,lim in claims])
    sections=[('Problem and evidence boundaries','P1 pp958-960; N209 manuscript; claim_evidence.csv','Which operating domain can be supported after independent validation?'),
      ('Source reproduction and declared adaptations','reproduction_map.csv; P1 Tables2/3; N073 report','Which SRS parameters and Eq34 conventions are missing or ambiguous? S01 only after authorization.'),
      ('Models, clocks and data contracts','system_capture/contracts.py; model/algorithm registry; S00 phase manifest','How are frames, points, information permissions and separate gates enforced?'),
      ('Estimation and feasibility planning','N209 sensor_reference_audit and uncertainty_floor; H2 diagnosis','S02/S03: can sensing requirements and motion feasibility be made compatible? NOT_EVALUATED here.'),
      ('Force feedback compliance and continuous integration','P1 Eq32-33; legacy contact module; current controller inventory','S04/S05: does measured force actually modify the reference and improve safe capture? NOT_EVALUATED.'),
      ('Identification and optional decision value','N209 qualification C3; shadow posterior records','S06 optional: does a posterior improve held-out prediction or change useful actions? NOT_DEMONSTRATED.'),
      ('Frozen independent experiments and results','S00 replay results only; prior seen-case evidence index','S07 not authorized; no new task success or method superiority is asserted.'),
      ('Limitations and conclusion','claim_evidence.csv and S00 qualification','Retain H2/S01 failures, hardware/full-boundary/real-time gaps and source ambiguities; no positive conclusion prefilled.')]
    text='# System framework manuscript skeleton\n\nStatus: S00 evidence scaffold only. No new controller performance.\n\n'
    for i,(title,source,question) in enumerate(sections,1):text+=f'## {i}. {title}\n\nSources: {source}.\n\nPending question: {question}\n\n'
    (paper/'manuscript.md').write_text(text.rstrip()+'\n',encoding='utf-8',newline='\n')
    (paper/'README.md').write_text('# System framework evidence panel\n\nCurrent phase: S00. [Stage report](../../output/fpmfc/system_capture/S00/report.md), [qualification](../../output/fpmfc/system_capture/S00/qualification.json), [handoff](../../output/fpmfc/system_capture/S00/handoff.json).\n\n[Claims](claim_evidence.csv) separate source reproduction, declared adaptation and new extension. [Reproduction map](reproduction_map.csv) records source gaps. [Manuscript](manuscript.md) is an unfilled question-based scaffold. S01-S08 are not executed or authorized by this panel.\n',encoding='utf-8',newline='\n')
    save(OUT/'source_reading.json',{'paper':pdfrel,'sha256':sha(pdf),'read_scope':'full extracted text; page964 rendered for Eq26-32; source page/equation maps above; no new literature reproduction',
        'type_I':'base attitude','type_II':'arm shape; not inertia identification','source_study_scope':'precontact, per conclusion p968','source_ambiguities':['Eq27 endpoint-nullspace is not by itself base reaction cancellation','Eq34 derivative/position increment units require clarification','Table3 does not resolve every COM/inertial-frame implementation detail'],
        'N209_H2_classification':read(OLD/'qp_failure_diagnosis.json')['runs']['H2_prior']['classification'],
        'N209_scope':read(OLD/'qualification_matrix.json')['status'],
        'S01_statistics_scope':read(OLD/'sensor_reference_audit.json')['scope']})
