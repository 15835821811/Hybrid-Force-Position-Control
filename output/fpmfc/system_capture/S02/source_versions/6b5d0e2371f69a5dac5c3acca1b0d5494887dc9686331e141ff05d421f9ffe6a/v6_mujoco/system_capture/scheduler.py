"""Integer clocks and injected packet/controller/event functions, no algorithm branches."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class ClockSpec:
    physics_s: float
    servo_s: float
    task_s: float

    def __post_init__(self):
        for x in (self.physics_s,self.servo_s,self.task_s):
            if not math.isfinite(x) or x<=0:raise ValueError('positive finite clocks')
        for x,y in ((self.servo_s,self.physics_s),(self.task_s,self.servo_s)):
            if abs(x/y-round(x/y))>1e-10:raise ValueError('clocks must have integer strides')

    def task_tick(self,time):
        i=round(time/self.servo_s)
        if abs(time-i*self.servo_s)>1e-8:raise ValueError('time off servo grid')
        return i%round(self.task_s/self.servo_s)==0

def decisions(packets,controller,clocks,observer,profiler=None):
    """Stream input only; observer stores evidence and cannot alter the proposal."""
    for i,packet in enumerate(packets):
        if profiler:profiler.begin(i,packet.t_control)
        try:proposal=controller.update(packet,clocks.task_tick(packet.t_control))
        finally:
            if profiler:profiler.end()
        observer(i,packet,proposal,controller)
        if not proposal.actuation_valid:break
