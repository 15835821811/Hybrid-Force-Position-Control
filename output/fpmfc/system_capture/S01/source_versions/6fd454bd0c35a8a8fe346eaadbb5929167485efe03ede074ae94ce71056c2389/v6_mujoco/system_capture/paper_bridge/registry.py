"""S01 registry extension; reserved S00 entries retain their historical meaning."""
from ..contracts import ModelProfile,ControllerSetup
from ..registry import AlgorithmEntry
from .common import PROJECT,CONFIG,sha
from .model import MODEL,compile_model

def models():return {'paper_compat_srs':ModelProfile('paper_compat_srs','paper_compat_srs','AVAILABLE',
  {'active_joint_count':7,'base_actuators':False,'target_actuators':False,'base':'free','gravity':'zero','scope':'precontact inertial model under explicit assumptions; physical clearance source-limited'},
  {p.relative_to(PROJECT).as_posix():sha(p) for p in [MODEL,CONFIG/'source_parameters.yaml',CONFIG/'assumptions.yaml']})}
def algorithms():return {key:AlgorithmEntry(key,'paper_compat_srs','AVAILABLE','v6_mujoco.system_capture.paper_bridge.control',cls,scope) for key,cls,scope in [
  ('source_nullspace','source_velocity','Eq18 and residual interpretation of Eq29-31, independently derived DH/momentum'),
  ('project_hqp','SRSHQP','Actual project two-level HQP, SRS shape and declared common bounds; no Flexiv geometry')]}
def model(model_id='paper_compat_srs',dt=None):
    if model_id!='paper_compat_srs':raise ValueError('S01 model domain mismatch')
    return compile_model(dt)
def controller(algorithm_id,setup,model):
    if not isinstance(setup,ControllerSetup):raise TypeError('ControllerSetup required')
    if setup.prior.get('domain')!='paper_compat_srs':raise ValueError('SRS public model setup required')
    from .control import SRSHQP,source_velocity
    if algorithm_id=='project_hqp':return SRSHQP(model)
    if algorithm_id=='source_nullspace':return source_velocity
    raise KeyError(algorithm_id)
