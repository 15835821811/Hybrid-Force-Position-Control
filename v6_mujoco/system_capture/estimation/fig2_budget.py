"""Figure 2: unchanged guard budget in archived free and synthetic contact motion."""
import numpy as np
from .common import OUT
from .plot_style import plt,COLORS,savefig
def main():
    fig,ax=plt.subplots(2,2,figsize=(7.4,5.2),layout='constrained')
    for row,prefix in enumerate(('archived_R03_V2_S01_','validation_bias_noise_')):
        for j,m in enumerate(('CV','CA')):
            with np.load(OUT/'offline'/(prefix+m)/'comparison.npz') as z:
                for col,(k,scale,label) in enumerate(((2,1000,'Remaining linear budget (mm/s)'),(3,180/np.pi,'Remaining angular budget (deg/s)'))):
                    ax[row,col].plot(z['time'],z['legacy_remaining_budget'][:,k]*scale,color=COLORS[j],ls='--' if j==0 else '-',lw=.7,label=m+' legacy guard')
                    if m=='CA':ax[row,col].plot(z['time'],z['full_remaining_budget'][:,k]*scale,color=COLORS[2],ls=':',lw=.7,label='CA full geometry')
                    ax[row,col].set_ylabel(label);ax[row,col].set_xlabel(('Archived noisy' if row==0 else 'Synthetic transition')+' time (s)')
                    ax[row,col].set_yscale('symlog',linthresh=.2);ax[row,col].axhline(0,color='k',lw=.5)
                    if j==0:
                        first=z['time'][np.flatnonzero(z['valid'])[0]]
                        ax[row,col].axvspan(0,first,color='.5',alpha=.25,zorder=-2)
                    ticks=[-1000,-100,-10,-1,-.2,0,.2,.6] if col==0 else [-1000,-100,-10,-1,-.2,0,.1,.2]
                    lo,hi=ax[row,col].get_ylim()
                    kept=[x for x in ticks if lo<=x<=hi]
                    ax[row,col].set_yticks(kept,[str(x) for x in kept])
        for a in ax[row]:
            a.legend(frameon=False,loc='lower right')
            if row:a.axvspan(1.2,4.2,color='.8',alpha=.25,zorder=-2)
            if row:a.axvline(2,color='.4',lw=.6,ls='-.')
    for a,label in zip(ax.flat,('a','b','c','d')):
        a.text(-.14,1.04,'('+label+')',transform=a.transAxes,fontweight='bold')
    savefig(fig,'f2_guard_budget')
if __name__=='__main__':main()
