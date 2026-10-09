"""Observe existing calls on preregistered packet indices; no function replacement."""
import sys
import time
import numpy as np

class SampledProfiler:
    def __init__(self,indices):
        self.selected=set(indices);self.records=[];self.calls=[];self.active={};self.current=None
    def identify(self,frame):
        path=frame.f_code.co_filename.replace('\\','/');name=frame.f_code.co_name
        if path.endswith('/geometry_capture/planning.py') and name=='full_distance_gradient':return 'distance_gradient'
        if path.endswith('/feasible_capture/controller_adapter.py') and name=='solve_fpmfc':return 'HQP'
        if path.endswith('/feasible_capture/estimator_adapter.py') and name=='update':return 'state_filter'
        if path.endswith('/feasible_capture/progress_governor.py') and name=='predict':return 'predictor'
        return None
    def trace(self,frame,event,arg):
        module=self.identify(frame)
        if module is None:return
        if event=='call':self.active[id(frame)]=(time.perf_counter(),module)
        elif event=='return' and id(frame) in self.active:
            start,module=self.active.pop(id(frame));bad=arg is None
            if module=='predictor' and isinstance(arg,dict):bad=not arg['verified']
            elif module=='HQP' and arg is not None:bad=not arg.success
            elif module=='state_filter' and arg is not None:bad=not arg.valid
            self.calls.append({'module':module,'wall_s':time.perf_counter()-start,'rejected_invalid_or_exception':bool(bad)})
    def begin(self,index,t):
        self.current=None
        if index in self.selected:
            if sys.getprofile() is not None:raise RuntimeError('external profiler already installed')
            self.current={'packet_index':index,'time_s':t};self.calls=[];self.active={};sys.setprofile(self.trace)
    def end(self):
        if self.current is None:return
        sys.setprofile(None)
        self.current['calls']=self.calls;self.records.append(self.current);self.current=None

def summarize(records):
    result={}
    for module in ('distance_gradient','HQP','state_filter','predictor'):
        calls=[c for r in records for c in r['calls'] if c['module']==module]
        z=np.array([c['wall_s'] for c in calls])
        result[module]={'calls':len(calls),'rejected_invalid_or_exception_calls':sum(c['rejected_invalid_or_exception'] for c in calls),
          'sampled_states':len(records),'states_with_calls':sum(any(c['module']==module for c in r['calls']) for r in records),
          'p50_s':float(np.quantile(z,.5)) if len(z) else None,'p95_s':float(np.quantile(z,.95)) if len(z) else None,'max_s':float(z.max()) if len(z) else None}
    return result
