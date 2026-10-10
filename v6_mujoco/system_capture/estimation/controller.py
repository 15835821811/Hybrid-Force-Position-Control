"""Versioned S02 composition; archived control, shaping and capture stay intact."""
from v6_mujoco.feasible_capture.controller_adapter import FeasibleController
from .filter import AccelerationEstimator
from .governor import SnapshotProgressGovernor

class AccelerationController(FeasibleController):
    def __init__(self,prior,cfg):
        super().__init__(prior,cfg)
        self.state_filter=AccelerationEstimator(cfg)
        self.predictor=SnapshotProgressGovernor(self)
        self.guard_calls=[]

    def gate(self,packet,e):
        import copy
        passed=super().gate(packet,e)
        self.guard_calls.append(dict(time=packet.time,passed=passed,**copy.deepcopy(self.last_gate)))
        return passed
