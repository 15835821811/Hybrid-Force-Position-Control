"""Figure 4 includes every authorized condition, including explicitly unexecuted ones."""
import numpy as np
from .common import OUT,read
from .plot_style import plt,COLORS,savefig
def main():
    fig,ax=plt.subplots(1,2,figsize=(7.4,3.7),layout='constrained');names=['E0','E0_C2','E1','E2','E3']
    attempts={r['name']:r for r in read(OUT/'run_ledger.json')['attempts']}
    durations=[];loads=[]
    for i,name in enumerate(names):
        path=OUT/'runs'/name/'metrics.json'
        if not path.exists():
            label='NOT RUN' if name not in attempts else 'ATTEMPTED; NO SUMMARY'
            if name=='E3' and name not in attempts:label+='\nOmitted for repair budget'
            ax[0].text(.2,i,label,va='center',fontsize=7.5);continue
        r=read(path);passed=r['continuous_task_completed'] and r['postgrasp_detumbling'] and not r['actual_safety_violations']
        color=COLORS[0] if passed else COLORS[1]
        durations.append(r['end_time_s']);loads.append(r['max_load_fraction'])
        ax[0].barh(i,r['end_time_s'],height=.5,color=color)
        label='Task passed' if passed else {'IMPLEMENTATION_ERROR':'Implementation error','NO_VERIFIED_CONTROL':'Control rejected'}.get(r['status'],r['status'].replace('_',' ').title())
        ax[0].text(r['end_time_s']+.3,i,f"{r['end_time_s']:.3f} s\n{label}",va='center',fontsize=7)
        ax[1].scatter(r['max_load_fraction'],i,color=color,marker='o' if passed else 'x',s=40)
    extent=max(durations,default=1.)
    ax[0].set_xlim(0,extent+max(10,.75*extent));ax[0].set_xlabel('Actual duration (s)');ax[1].set_xlabel('Maximum actual load fraction')
    ax[1].axvline(1,color='k',lw=.6,ls='--');ax[1].set_xlim(-.03,max(1.1,1.15*max(loads,default=0.)))
    for a in ax:a.set_yticks(range(5),names);a.set_ylim(4.6,-.6)
    for a,label in zip(ax,('a','b')):a.text(-.12,1.03,'('+label+')',transform=a.transAxes,fontweight='bold')
    savefig(fig,'f4_task_outcomes')
if __name__=='__main__':main()
