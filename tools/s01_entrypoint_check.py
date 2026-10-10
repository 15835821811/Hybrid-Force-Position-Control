"""Routing and resume contract checks without requesting another physical run."""
import contextlib
import io
import sys
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from v6_mujoco.system_capture.cli import main
from v6_mujoco.system_capture.registry import phase_models,phase_algorithms
from v6_mujoco.system_capture.paper_bridge.common import OUT,save,sha

def run():
    before=set(OUT.parent.iterdir());capture=io.StringIO()
    with contextlib.redirect_stdout(capture):code=main(['--phase','S02','--mode','run'])
    assert code==2 and 'NOT_IMPLEMENTED' in capture.getvalue() and before==set(OUT.parent.iterdir())
    calls=[]
    with patch('v6_mujoco.system_capture.paper_bridge.phase.main',side_effect=lambda mode,resume:calls.append((mode,resume)) or 0):
        for mode in ['prepare','test','run','replay','report']:
            assert main(['--phase','S01','--mode',mode,'--resume'])==0
    assert calls==[(mode,True) for mode in ['prepare','test','run','replay','report']]
    # S00 dispatch stays directed to the existing implementation. Its frozen verification
    # must be executed from its historical acceptance checkout, not relabeled as S01.
    with patch('v6_mujoco.system_capture.phase.prepare',return_value=None) as prep:
        assert main(['--phase','S00','--mode','prepare'])==0
        prep.assert_called_once_with()
    models=phase_models('S01');algorithms=phase_algorithms('S01')
    assert 'paper_compat_srs' in models
    result={'passed':True,'S02_remains_unimplemented':True,'S01_modes_forwarded':calls,'S00_prepare_dispatch_unchanged':True,
      'model_ids':list(models),'algorithm_ids':list(algorithms),'physics_steps':0,'new_attempts':0,
      'historical_S00_reproduction':'Use commit 80ac0f69faa258cba61a2623a4f55961e7ffbebb; its archived selftest expects S01 not implemented. Do not rerun or overwrite historical S00 evidence from this S01 checkout.',
      'scope':'Dispatch uses explicit local mocks only in this integration check; production execution is not patched','validator_sha256':sha(Path(__file__))}
    save(OUT/'entrypoint_check.json',result);print(result)

if __name__=='__main__':run()
