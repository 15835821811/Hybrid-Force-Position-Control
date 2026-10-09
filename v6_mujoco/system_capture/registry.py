"""Explicit model/algorithm factories; reserved entries cannot create a run."""
from dataclasses import dataclass
from .contracts import ModelProfile,ScenarioSpec,ControllerSetup,NamedConstraint,plain
from .common import PROJECT,OLD,sha,read

class NotImplementedStage(RuntimeError): pass

@dataclass(frozen=True)
class AlgorithmEntry:
    algorithm_id: str
    domain: str
    status: str
    module: str | None
    class_name: str | None
    scope: str

ALGORITHMS={
 'n208_baseline':AlgorithmEntry('n208_baseline','flexiv_system','AVAILABLE','v6_mujoco.adaptive_capture.controller','Controller','Existing fixed relative reference and declared legacy options; no fresh N208 performance claim'),
 'n209_scalar_governor':AlgorithmEntry('n209_scalar_governor','flexiv_system','AVAILABLE','v6_mujoco.feasible_capture.controller_adapter','FeasibleController','Archived N209 scalar governor; no retuning'),
 'paper_compat_reserved':AlgorithmEntry('paper_compat_reserved','paper_compat_srs','NOT_IMPLEMENTED',None,None,'S01 requires original-source parameter mapping; never runs Flexiv under this label'),
}

def models():
    return {
      'flexiv_system':ModelProfile('flexiv_system','flexiv_system','AVAILABLE',
          {'active_joint_count':7,'base_actuators':False,'target_actuators':False,'gravity':'local_microgravity','tool_extension_m':.020,'tool_face_from_flange_m':.0198,'joint_names':['joint'+str(i) for i in range(1,8)],'interface':'calibrated software weld; not hardware certification'},
          {p:sha(PROJECT/p) for p in ['models/flexiv_rizon4s_n206_tool_scene.xml','configs/n206_geometry_capture.yaml']}),
      'paper_compat_srs':ModelProfile('paper_compat_srs','paper_compat_srs','NOT_IMPLEMENTED',
          {'active_joint_count':7,'base_actuators':False,'target_actuators':False,'missing_parameters':'S01 source contract pending; no inherited Flexiv values'},{}),
    }

def scenario(name):
    cfg=read(OLD/'runs'/name/'config.json')
    mission=cfg['mission']
    return ScenarioSpec(name,'flexiv_system',plain(models()['flexiv_system'].hardware),cfg['truth_evaluation_only'],cfg['prior'],cfg['sensors'],
                        {'id':'n209_scalar_governor','legacy_mission':mission},
                        {k:mission[k] for k in ['capture','approach_deadline_s','post_duration_s','evaluation_width_s']},
                        {'dt':cfg['dt'],'servo_s':mission['servo_s'],'task_s':mission['task_s'],'solver_tolerance':mission['solver_tolerance']})

def controller(algorithm_id, setup, model_id='flexiv_system'):
    import importlib
    from .adapters import ControllerAdapter
    entry=ALGORITHMS[algorithm_id]
    if entry.status!='AVAILABLE':raise NotImplementedStage('NOT_IMPLEMENTED: '+algorithm_id)
    if entry.domain != model_id:raise ValueError('model/algorithm domain mismatch')
    if not isinstance(setup,ControllerSetup):raise TypeError('ControllerSetup only; never ScenarioSpec/truth')
    cls=getattr(importlib.import_module(entry.module),entry.class_name)
    return ControllerAdapter(cls(plain(setup.prior),plain(setup.algorithm_config)))

def thresholds(cfg):
    """Separate physical requirements, statistical gates, design tests and numerics."""
    import numpy as np
    values=[
      ('capture_translation','task_safety','m',1e-4,cfg['capture']['translation_m'],'mission.capture'),
      ('capture_rotation','task_safety','rad',np.deg2rad(.05),np.deg2rad(cfg['capture']['rotation_deg']),'mission.capture'),
      ('capture_linear_speed','task_safety','m/s',.001,cfg['capture']['linear_m_s'],'mission.capture'),
      ('capture_angular_speed','task_safety','rad/s',np.deg2rad(.2),np.deg2rad(cfg['capture']['angular_deg_s']),'mission.capture'),
      ('actual_load_fraction','task_safety','1',1.,1.,'norm(F)/50N + norm(M)/2Nm'),
      ('holding_translation','task_safety','m',.0005,.0005,'N209 inherited holding gate'),
      ('holding_rotation','task_safety','rad',np.deg2rad(.1),np.deg2rad(.1),'N209 inherited holding gate'),
      ('stale_age','sensor_gate','s',cfg['stale_limit_s'],cfg['stale_limit_s'],'mission.stale_limit_s'),
      ('uncertainty_multiplier','sensor_gate','1',1.,cfg['margin_sigma'],'mission.margin_sigma; not confidence certification'),
      ('predicted_tracking','algorithm_acceptance','m',.008,cfg.get('n209',{}).get('tracking_error_limit_m',.008),'N209 finite predictor; NOT a physical collision limit'),
      ('solver_tolerance','numerical','solver_internal',1.,cfg['solver_tolerance'],'MuJoCo solver, not safety margin'),
      ('replay_qpos','numerical','mixed_m_rad',1.,1e-10,'original tagged validator'),
      ('replay_qvel','numerical','mixed_m_s_rad_s',1.,1e-9,'original tagged validator'),
    ]
    return tuple(NamedConstraint(n,r,u,s,source,limit=float(v)) for n,r,u,s,v,source in values)
