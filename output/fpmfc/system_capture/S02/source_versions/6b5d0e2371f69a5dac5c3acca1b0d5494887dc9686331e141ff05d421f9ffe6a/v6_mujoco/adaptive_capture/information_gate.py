"""Conservative parameter-use permission based only on causal heldout blocks."""
import numpy as np
from .momentum_regressor import physical

class InformationGate:
    def __init__(self,prior):
        self.prior=prior;self.model_pi=None;self.reason='shadow only; no independent prediction blocks';self.used=False
    def update(self,estimator,dt,enabled,grasp_verified):
        if self.model_pi is None:self.model_pi=estimator.pi0.copy()
        recent=estimator.predict[-10:]
        valid=[x for x in recent if x['rank']>=3]
        trust=bool(enabled and grasp_verified and estimator.accepted and estimator.absolute_excitation and estimator.rank>=3 and len(valid)>=5 and
            np.mean([x['posterior']**2 for x in valid])<.8*np.mean([x['prior']**2 for x in valid]) and np.mean([x['posterior']**2 for x in valid])<36 and physical(estimator.pi,self.prior))
        if trust:
            delta=(estimator.pi-self.model_pi)/estimator.scales;norm=np.linalg.norm(delta)
            candidate=self.model_pi+estimator.scales*delta*min(1.,.05*dt/max(norm,1e-12))
            if physical(candidate,self.prior):self.model_pi=candidate;self.used=True;self.reason='causal prediction improvement; physical scaled update <=0.05/s'
        else:
            # Withdraw unsupported parameter corrections with the same bounded
            # rate; do not silently hold the last estimate while claiming prior.
            delta=(estimator.pi0-self.model_pi)/estimator.scales;norm=np.linalg.norm(delta)
            self.model_pi+=estimator.scales*delta*min(1.,.05*dt/max(norm,1e-12))
            self.reason='prior retained' if np.linalg.norm((self.model_pi-estimator.pi0)/estimator.scales)<1e-12 else 'information insufficient; bounded return to prior <=0.05/s'
        return self.model_pi.copy(),trust
