"""Figure 3: fixed first-contact +/-100 ms diagnostic, no selected success window."""
import numpy as np
from .common import OUT,read
from .plot_style import plt,COLORS,savefig
from .persistence import records
def main():
    d=OUT/'runs/E0_C2'
    if not (d/'trace.npz').exists():return
    with np.load(d/'trace.npz') as a:z={k:a[k] for k in a.files}
    ids=np.flatnonzero(z['intentional_contact_count']>0)
    if not len(ids):return
    t0=z['time_s'][ids[0]];mask=abs(z['time_s']-t0)<=.1;t=z['time_s'][mask]-t0
    fig,ax=plt.subplots(3,1,figsize=(7.4,6.9),layout='constrained')
    actual=z['target_geometric_twist_world'][:,:3]
    refs={}
    for row in records(d/'raw'):
        if row['kind']=='proposal':
            e=row['snapshot']['controller']['reference_target']
            if e is not None:refs[round(row['time'],9)]=e['v']
    reference=np.array([refs.get(round(t,9),[np.nan]*3) for t in z['time_s']])
    estimate=z['estimated_v'].copy()
    estimate[~z['estimate_current_tick'].astype(bool)]=np.nan
    for values,name,c,ls in [(actual,'Actual target',COLORS[0],'-'),(estimate,'Causal estimate',COLORS[1],'--'),(reference,'Target reference (shaping disabled)',COLORS[2],':')]:
        ax[0].plot(t,np.linalg.norm(values[mask],axis=1)*1000,label=name,color=c,ls=ls)
    ax[0].set_ylabel('Target speed (mm/s)');ax[0].legend(frameon=False)
    error=np.linalg.norm(z['estimate_error'][:,6:9],axis=1)*1000
    error[~z['estimate_current_tick'].astype(bool)]=np.nan
    ax[1].plot(t,error[mask],color=COLORS[1],label='Linear estimate error')
    ax[1].set_ylabel('Error norm (mm/s)');ax[1].legend(frameon=False)
    for a in ax[:2]:a.axvline(0,color='k',lw=.6,ls=':');a.set_xlabel('Time from first contact (s)')
    full=error.copy();full[~z['estimate_valid'].astype(bool)]=np.nan
    ax[2].plot(z['time_s'],full,color=COLORS[1],label='All valid samples',lw=.7)
    peak=int(np.nanargmax(full));ax[2].scatter(z['time_s'][peak],full[peak],s=16,color=COLORS[1])
    ax[2].annotate(f"Peak {full[peak]:.3f} mm/s at {z['time_s'][peak]:.3f} s",
        xy=(z['time_s'][peak],full[peak]),xytext=(12,0),textcoords='offset points',fontsize=8,va='center')
    ax[2].set_ylim(-.01,max(.02,np.nanmax(full)*1.18))
    ax[2].set_xlabel('Full trajectory time (s)');ax[2].set_ylabel('Error norm (mm/s)')
    for a,label in zip(ax,('a','b','c')):a.text(-.07,1.03,'('+label+')',transform=a.transAxes,fontweight='bold')
    savefig(fig,'f3_contact_motion')
if __name__=='__main__':main()
