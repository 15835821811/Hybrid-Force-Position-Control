"""V1 D00 prefix replay with the recorded external I/O fault at its exact callsite."""
import sys
import time
import importlib.util
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from v6_mujoco.system_capture.planning.contracts import OUT,PROJECT,Context,read,save,sha,clean
from v6_mujoco.system_capture.estimation.replay import replay_one,deterministic
from v6_mujoco.system_capture.estimation.persistence import records,validate
from v6_mujoco.system_capture.contracts import decode,plain
from v6_mujoco.system_capture.replay import compare
from v6_mujoco.system_capture.scheduler import ClockSpec

def main():
    name='D00';folder=OUT/'runs'/name;done=read(folder/'completion.json');cfg=read(folder/'config.json')
    if not (folder/'actuator_replay.json').exists():
        if not replay_one(name,'actuator',Context())['passed']:raise RuntimeError('actuator replay failed')
    root=OUT/'versions/V1';identity=read(root/'identity.json')
    for rel,h in identity['source'].items():assert sha(root/rel)==h
    files=['contracts','trajectory_segment','parameterization','screening','prediction','planner','controller_adapter']
    for name in files:
        module='v6_mujoco.system_capture.planning.'+name;p=root/'v6_mujoco/system_capture/planning'/(name+'.py')
        spec=importlib.util.spec_from_file_location(module,p);obj=importlib.util.module_from_spec(spec);sys.modules[module]=obj;spec.loader.exec_module(obj)
    p=root/'v6_mujoco/system_capture/estimation/runner.py';module='v6_mujoco.system_capture.estimation.runner'
    spec=importlib.util.spec_from_file_location(module,p);runner=importlib.util.module_from_spec(spec);sys.modules[module]=runner;spec.loader.exec_module(runner)
    old=sys.modules['v6_mujoco.system_capture.planning.contracts'];ctx=old.Context();w=ctx.controller(cfg,replay=True)
    original=w.legacy.predictor.budget.register;faults=[]
    def inject(method,t,theta,model):
        if abs(t-done['end_time_s'])<1e-10:
            faults.append(dict(time=t,callsite='Budget.register -> save -> os.replace',model=model,theta=theta))
            raise PermissionError('recorded WinError5 external sharing fault reproduced at Budget.register')
        return original(method,t,theta,model)
    w.legacy.predictor.budget.register=inject
    cpu,wall=time.process_time(),time.perf_counter();clock=ClockSpec(.002,.002,.02);passed=True;maxerr=0.;packets=0;expected_exception=False;actual=None;mismatches=[]
    for r in records(folder/'raw'):
        if r['kind']=='packet_received':
            p=decode(r['packet']);packets+=1
            try:actual=w.update(p,clock.task_tick(p.t_control))
            except PermissionError:
                if abs(p.t_control-done['end_time_s'])>1e-10:raise
                expected_exception=True
        elif r['kind']=='proposal':
            for kind,a,b in [('proposal',plain(actual),plain(decode(r['proposal']))),('snapshot',deterministic(runner.snapshot(w)),deterministic(r['snapshot']))]:
                ok,err=compare(a,b);passed &= ok;maxerr=max(maxerr,err)
                if not ok and len(mismatches)<8:mismatches.append(dict(time=r['time'],kind=kind,error=err))
        elif r['kind']=='exception':passed &= expected_exception and 'PermissionError' in r['reason']
    result=dict(run='D00',kind='decision',passed=bool(passed and expected_exception and len(faults)==1),packets=packets,
        max_intermediate_error=maxerr,mismatches=mismatches,original_runtime_sha256=identity['source'],
        terminal_external_fault_injection=faults,scope='exact V1 scientific prefix and recorded external I/O fault at original callsite; OS sharing failure is not claimed to recur spontaneously',
        original_failure_retained=True,physics_steps=0,predictive_steps=w.legacy.predictor.physics_steps,
        cpu_s=time.process_time()-cpu,wall_s=time.perf_counter()-wall,raw_validation=validate(folder/'raw'))
    save(folder/'decision_replay.json',result);b=read(OUT/'run_ledger.json');b['replays'].append(dict(run='D00',kind='decision',passed=result['passed'],cpu_s=result['cpu_s'],wall_s=result['wall_s'],predictive_steps=result['predictive_steps'],physics_steps=0));save(OUT/'run_ledger.json',b)
    print({k:v for k,v in result.items() if k not in ['original_runtime_sha256','raw_validation']},flush=True)

if __name__=='__main__':main()
