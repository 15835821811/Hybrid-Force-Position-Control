"""Thin composition; B0 constructs the unchanged C2 control object."""
from ..contracts import ControllerSetup,plain
from ..adapters import ControllerAdapter
from ..estimation.controller import AccelerationController

def controller(cfg,replay=False):
    setup=ControllerSetup(cfg['prior'],cfg['mission'])
    prior,mission=plain(setup.prior),plain(setup.algorithm_config)
    if mission.get('s03_method','B0_scalar')=='B0_scalar':return ControllerAdapter(AccelerationController(prior,mission))
    from .planner import PlanningController,PlanningAdapter
    return PlanningAdapter(PlanningController(prior,mission,replay=replay))
