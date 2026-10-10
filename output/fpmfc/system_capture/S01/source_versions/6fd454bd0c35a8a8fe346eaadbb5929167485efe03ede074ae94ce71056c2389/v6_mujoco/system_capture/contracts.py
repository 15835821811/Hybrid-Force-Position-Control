"""SI contracts. Arrays own immutable byte buffers, including nested snapshots."""
from dataclasses import dataclass, fields, is_dataclass
from collections.abc import Mapping
from types import MappingProxyType
import math
import numpy as np


def require(condition, message):
    if not condition:
        raise ValueError(message)


def scalar(x, name, lower=None):
    require(not isinstance(x,(bool,np.bool_)) and isinstance(x, (int, float, np.number)) and math.isfinite(float(x)), name+' must be finite')
    require(lower is None or x >= lower, name+' below bound')


def array(x, shape, name):
    a = np.array(x, dtype=float, copy=True)
    require(a.shape == shape and np.isfinite(a).all(), name+' shape/finite mismatch')
    return a


def rotation(x):
    a = array(x, (3, 3), 'rotation')
    require(np.max(abs(a.T @ a-np.eye(3))) < 1e-8 and abs(np.linalg.det(a)-1) < 1e-8, 'rotation must be SO(3)')


def covariance(x, n):
    a = array(x, (n, n), 'covariance')
    require(np.max(abs(a-a.T)) <= 1e-9, 'covariance not symmetric')
    require(np.linalg.eigvalsh((a+a.T)/2).min() >= -1e-10, 'covariance not PSD')


def freeze(x):
    if isinstance(x, np.ndarray):
        a = np.ascontiguousarray(x)
        return np.frombuffer(a.tobytes(), dtype=a.dtype).reshape(a.shape)
    if isinstance(x, Mapping):
        return MappingProxyType({str(k): freeze(v) for k, v in x.items()})
    if isinstance(x, (list, tuple)):
        return tuple(freeze(v) for v in x)
    return x


def plain(x):
    if is_dataclass(x):
        return {f.name: plain(getattr(x, f.name)) for f in fields(x)}
    if isinstance(x, Mapping):
        return {k: plain(v) for k, v in x.items()}
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (tuple, list)):
        return [plain(v) for v in x]
    if isinstance(x, np.generic):
        return x.item()
    return x


def encode(x):
    """Tagged JSON values; decoding reruns each constructor's validation."""
    if is_dataclass(x):return {'$type':type(x).__name__,'fields':{f.name:encode(getattr(x,f.name)) for f in fields(x)}}
    if isinstance(x,Mapping):return {k:encode(v) for k,v in x.items()}
    if isinstance(x,(tuple,list)):return [encode(v) for v in x]
    return plain(x)


def decode(x):
    if isinstance(x,list):return [decode(v) for v in x]
    if isinstance(x,dict):
        if '$type' in x:
            cls=globals().get(x['$type'])
            require(isinstance(cls,type) and issubclass(cls,Snapshot) and is_dataclass(cls),'unknown contract type')
            require(set(x)=={'$type','fields'},'invalid tagged contract')
            return cls(**{k:decode(v) for k,v in x['fields'].items()})
        return {k:decode(v) for k,v in x.items()}
    return x


class Snapshot:
    def seal(self, arrays=()):
        for name in arrays:
            object.__setattr__(self, name, np.asarray(getattr(self, name), dtype=float))
        for f in fields(self):
            object.__setattr__(self, f.name, freeze(getattr(self, f.name)))


@dataclass(frozen=True)
class FrameConvention(Snapshot):
    frame: str
    point: str
    velocity_order: str = 'linear_then_angular'
    wrench_order: str = 'force_then_moment'
    rotation_convention: str = 'R_parent_from_child_active_column'
    units: tuple = ('m', 'rad', 's', 'N', 'N m', 'kg')

    def __post_init__(self):
        require(bool(self.frame) and bool(self.point), 'frame/point required')
        require(self.velocity_order == 'linear_then_angular', 'convert spatial twist explicitly')
        require(self.wrench_order == 'force_then_moment', 'wrench order mismatch')
        require(self.rotation_convention == 'R_parent_from_child_active_column', 'rotation convention mismatch')
        require(tuple(self.units) == ('m', 'rad', 's', 'N', 'N m', 'kg'), 'SI units required')
        self.seal()


@dataclass(frozen=True)
class PoseObservation(Snapshot):
    t_sample: float
    t_arrival: float
    p: np.ndarray
    R: np.ndarray
    P: np.ndarray
    convention: FrameConvention
    t_generated: float | None = None

    def __post_init__(self):
        scalar(self.t_sample, 'sample', 0); scalar(self.t_arrival, 'arrival', 0)
        require(self.t_sample <= self.t_arrival+1e-10, 'future observation')
        if self.t_generated is not None:
            require(self.t_sample <= self.t_generated <= self.t_arrival+1e-10, 'generation timestamp')
        array(self.p, (3,), 'pose p'); rotation(self.R); covariance(self.P, 6)
        require(self.convention == FrameConvention('W', 'target_geometry'), 'pose is visible origin, not COM')
        self.seal(('p', 'R', 'P'))


@dataclass(frozen=True)
class RobotObservation(Snapshot):
    base_pose_wxyz: np.ndarray
    base_linear_velocity_W: np.ndarray
    base_angular_velocity_B: np.ndarray
    q7: np.ndarray
    dq7: np.ndarray
    actuator_tau7: np.ndarray

    def __post_init__(self):
        for name, n in [('base_pose_wxyz', 7), ('base_linear_velocity_W', 3), ('base_angular_velocity_B', 3), ('q7', 7), ('dq7', 7), ('actuator_tau7', 7)]:
            array(getattr(self, name), (n,), name)
        require(abs(np.linalg.norm(np.asarray(self.base_pose_wxyz)[3:])-1) < 1e-8, 'base quaternion normalization')
        self.seal(tuple(f.name for f in fields(self)))

    def base_twist_world(self):
        from scipy.spatial.transform import Rotation
        R = Rotation.from_quat(self.base_pose_wxyz[[4, 5, 6, 3]]).as_matrix()
        return np.r_[self.base_linear_velocity_W, R @ self.base_angular_velocity_B]


@dataclass(frozen=True)
class SensorPacket(Snapshot):
    t_sample: float
    t_arrival: float
    t_control: float
    robot: RobotObservation
    poses: tuple
    wrench_target_at_grasp_W: np.ndarray
    contact: bool
    wrench_convention: FrameConvention = FrameConvention('W', 'target_grasp')

    def __post_init__(self):
        for t in (self.t_sample, self.t_arrival, self.t_control): scalar(t, 'packet time', 0)
        require(self.t_sample <= self.t_arrival+1e-10 <= self.t_control+2e-10, 'packet causality')
        require(isinstance(self.robot, RobotObservation), 'robot observation type')
        require(all(isinstance(p, PoseObservation) and p.t_arrival <= self.t_control+1e-10 for p in self.poses), 'pose delivery causality/type')
        array(self.wrench_target_at_grasp_W, (6,), 'wrench')
        require(self.wrench_convention == FrameConvention('W', 'target_grasp'), 'wrench point/frame')
        self.seal(('wrench_target_at_grasp_W',))


@dataclass(frozen=True)
class StateEstimate(Snapshot):
    t_measurement: float
    t_state: float
    p: np.ndarray
    R: np.ndarray
    v: np.ndarray
    w: np.ndarray
    P: np.ndarray
    bias_bound: np.ndarray
    valid: bool
    process_mode: str
    convention: FrameConvention = FrameConvention('W', 'target_geometry')
    covariance_order: str = 'dp_W,dtheta_W,dv_W,domega_W'

    def __post_init__(self):
        scalar(self.t_measurement, 'measurement', -1); scalar(self.t_state, 'state', 0)
        require(self.t_measurement <= self.t_state+1e-10 and (not self.valid or self.t_measurement >= 0), 'estimate causality')
        for n in ('p', 'v', 'w'): array(getattr(self, n), (3,), n)
        rotation(self.R); covariance(self.P, 12); b=array(self.bias_bound, (12,), 'bias')
        require((b >= 0).all() and bool(self.process_mode), 'bias/process mode')
        require(self.convention == FrameConvention('W', 'target_geometry') and self.covariance_order == 'dp_W,dtheta_W,dv_W,domega_W', 'state frame/order')
        self.seal(('p', 'R', 'v', 'w', 'P', 'bias_bound'))


@dataclass(frozen=True)
class ParameterEstimate(Snapshot):
    pi: np.ndarray
    covariance: np.ndarray
    covariance_scope: str
    rank: int
    physical: bool
    prediction_score: Mapping
    feedback_used: bool
    order: str = 'm,hx,hy,hz,IOxx,IOyy,IOzz,IOxy,IOxz,IOyz'

    def __post_init__(self):
        array(self.pi, (10,), 'pi'); covariance(self.covariance, 10)
        require(type(self.rank) is int and 0 <= self.rank <= 10, 'rank')
        require(bool(self.covariance_scope) and self.order == 'm,hx,hy,hz,IOxx,IOyy,IOzz,IOxy,IOxz,IOyz', 'parameter convention')
        if self.physical: require(self.pi[0] > 0, 'physical mass')
        self.seal(('pi', 'covariance'))


@dataclass(frozen=True)
class MotionReference(Snapshot):
    t_state: float
    T: np.ndarray
    v: np.ndarray
    w: np.ndarray
    a: np.ndarray
    alpha: np.ndarray
    s: float
    sdot: float
    sddot: float
    psi: float
    psidot: float
    valid_until: float
    convention: FrameConvention = FrameConvention('W', 'flange')

    def __post_init__(self):
        T=array(self.T, (4, 4), 'T'); rotation(T[:3,:3])
        require(np.array_equal(T[3], [0,0,0,1]), 'homogeneous transform bottom row')
        for n in ('v','w','a','alpha'): array(getattr(self,n),(3,),n)
        for n in ('t_state','s','sdot','sddot','psi','psidot','valid_until'): scalar(getattr(self,n),n)
        require(0 <= self.s <= 1+1e-10 and self.valid_until >= self.t_state and self.t_state >= 0, 'reference progress/time')
        require(self.convention == FrameConvention('W','flange'), 'reference frame/point')
        self.seal(('T','v','w','a','alpha'))


@dataclass(frozen=True)
class NamedConstraint(Snapshot):
    name: str
    role: str
    units: str
    scale: float
    source: str
    value: float | None = None
    limit: float | None = None
    relation: str = '<='

    def __post_init__(self):
        require(self.role in ('task_safety','sensor_gate','algorithm_acceptance','numerical'), 'threshold role')
        require(bool(self.name) and bool(self.units) and bool(self.source), 'named constraint provenance')
        scalar(self.scale,'scale'); require(self.scale > 0,'positive scale')
        if self.value is not None: scalar(self.value,'value')
        if self.limit is not None: scalar(self.limit,'limit')
        require(self.relation in ('<=','>=','=='), 'constraint relation')
        self.seal()


@dataclass(frozen=True)
class ControlProposal(Snapshot):
    t_control: float
    tau7: np.ndarray
    phase: str
    solver_statuses: Mapping
    named_constraints: tuple
    latch_request: bool
    fallback_kind: str
    decision: str
    actuation_valid: bool

    def __post_init__(self):
        scalar(self.t_control,'control',0); array(self.tau7,(7,),'tau7')
        require(bool(self.phase) and bool(self.decision), 'phase/decision')
        require(self.fallback_kind in ('NONE','LEGACY_OBSERVE_ZERO','SIM_ABORT'), 'unverified fallback')
        require(all(isinstance(c,NamedConstraint) for c in self.named_constraints), 'named constraints type')
        require(self.actuation_valid or (self.fallback_kind == 'SIM_ABORT' and not self.latch_request), 'aborted action cannot latch')
        self.seal(('tau7',))


@dataclass(frozen=True)
class ModelProfile(Snapshot):
    model_id: str
    domain: str
    status: str
    hardware: Mapping
    source_identity: Mapping

    def __post_init__(self):
        require(self.domain in ('paper_compat_srs','flexiv_system'), 'model domain')
        require(self.status in ('AVAILABLE','NOT_IMPLEMENTED'), 'model availability')
        require(self.hardware.get('active_joint_count') == 7, 'seven actual actuators')
        require(not self.hardware.get('base_actuators') and not self.hardware.get('target_actuators'), 'passive free bodies')
        self.seal()


@dataclass(frozen=True)
class ControllerSetup(Snapshot):
    prior: Mapping
    algorithm_config: Mapping

    def __post_init__(self):
        forbidden={'truth','truth_evaluation_only','scenario','scenario_id','future_packets','scene_id'}
        def check(x):
            if isinstance(x,Mapping):
                require(not forbidden.intersection(x),'private input at controller boundary')
                for v in x.values(): check(v)
            elif isinstance(x,(list,tuple)):
                for v in x: check(v)
        check(self.prior); check(self.algorithm_config); self.seal()


@dataclass(frozen=True)
class ScenarioSpec(Snapshot):
    scenario_id: str
    model_id: str
    hardware: Mapping
    truth_evaluation_only: Mapping
    prior: Mapping
    sensors: Mapping
    algorithm: Mapping
    task: Mapping
    numerical: Mapping

    def __post_init__(self):
        require(bool(self.scenario_id) and self.model_id in ('flexiv_system','paper_compat_srs'), 'scenario model identity')
        require(all(isinstance(getattr(self,f.name),Mapping) for f in fields(self) if f.name not in ('scenario_id','model_id')), 'separate scenario namespaces')
        self.seal()

    def controller_setup(self):
        return ControllerSetup(self.prior,self.algorithm['legacy_mission'])


@dataclass(frozen=True)
class RunManifest(Snapshot):
    phase: str
    input_commit: str
    source_identity: Mapping
    data_identity: Mapping
    environment: Mapping
    task_identity: str
    evaluation_identity: str

    def __post_init__(self):
        require(self.phase == 'S00' and len(self.input_commit) == 40, 'phase/baseline')
        require(all(bool(getattr(self,f.name)) for f in fields(self)), 'complete execution identity')
        self.seal()


@dataclass(frozen=True)
class PhaseResult(Snapshot):
    implementation: str
    physical_model: str
    reproduction: str
    task: str
    method_benefit: str
    sensor_domain: Mapping
    parameter_identification: str
    control_benefit: str
    next_stage_admission: bool | str
    no_new_controller_performance: bool = True

    def __post_init__(self):
        for name,choices in {'implementation':('PASS','FAIL','BLOCKED'),'physical_model':('VERIFIED_IN_SCOPE','FAILED','NOT_EVALUATED'),'reproduction':('SOURCE_COMPATIBLE','DECLARED_ADAPTATION','SOURCE_LIMITED','NOT_APPLICABLE'),'task':('PASSED_IN_CASES','FAILED','NOT_EVALUATED'),'method_benefit':('SUPPORTED_IN_CASES','NOT_SUPPORTED','NOT_EVALUATED'),'parameter_identification':('UNOBSERVABLE','PARTIAL','ACCURATE_IN_TESTED_CASES')}.items():
            require(getattr(self,name) in choices,name+' status')
        require(self.next_stage_admission in (True,False,'conditional') and self.no_new_controller_performance, 'S00 scope')
        self.seal()


def shift_point_velocity(v, w, r_old_to_new):
    return np.asarray(v)+np.cross(w,r_old_to_new)


def shift_wrench(wrench, r_old_to_new):
    W=array(wrench,(6,),'wrench')
    return np.r_[W[:3],W[3:]-np.cross(r_old_to_new,W[:3])]


def spatial_to_point_velocity(spatial, p):
    V=array(spatial,(6,),'spatial velocity')
    return np.r_[V[:3]+np.cross(V[3:],p),V[3:]]
