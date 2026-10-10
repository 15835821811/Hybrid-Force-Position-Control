"""Figure 1: actual causal state errors and blind forecasts from saved arrays."""
import numpy as np
from .common import OUT,read
from .plot_style import plt,COLORS,savefig
def main():
    fig,ax=plt.subplots(2,2,figsize=(7.4,5.2),layout='constrained')
    for j,m in enumerate(('CV','CA')):
        with np.load(OUT/'offline'/('archived_R03_V2_S01_'+m)/'comparison.npz') as z:
            for col,(k,scale,label) in enumerate(((6,1000,'Linear velocity error (mm/s)'),(9,180/np.pi,'Angular velocity error (deg/s)'))):
                ax[0,col].plot(z['time'],np.linalg.norm(z['error'][:,k:k+3],axis=1)*scale,color=COLORS[j],lw=.65,label=m,ls='-' if j else '--')
                ax[0,col].set_ylabel(label);ax[0,col].set_xlabel('Archived noisy time (s)');ax[0,col].set_yscale('symlog',linthresh=.05);ax[0,col].legend(frameon=False)
                if j==0:
                    first=z['time'][np.flatnonzero(z['valid'])[0]]
                    ax[0,col].axvspan(0,first,color='.5',alpha=.25,zorder=-2)
                    ax[0,col].annotate('Initialization <24 ms',xy=(first,1),xycoords=('data','axes fraction'),
                        xytext=(15,-12),textcoords='offset points',fontsize=7,va='top')
        rows=read(OUT/'synthetic_validation.json')
        horizons=sorted(float(h) for h in rows['bias_noise'][m]['blind_prediction'])
        for col,(k,scale,label) in enumerate(((6,1000,'Blind linear error RMS (mm/s)'),(9,180/np.pi,'Blind angular error RMS (deg/s)'))):
            vals=[np.linalg.norm(np.array(rows['bias_noise'][m]['blind_prediction'][str(h)]['component_rms'])[k:k+3])*scale for h in horizons]
            ax[1,col].plot(np.array(horizons)*1000,vals,'o-' if j else 's--',color=COLORS[j],label=m,ms=4)
            ax[1,col].set_xlabel('Blind horizon (ms)');ax[1,col].set_ylabel(label);ax[1,col].legend(frameon=False)
            ax[1,col].set_xticks([0,6,12,16])
    for a,label in zip(ax.flat,('a','b','c','d')):
        a.text(-.14,1.04,'('+label+')',transform=a.transAxes,fontweight='bold')
    savefig(fig,'f1_causal_errors')
if __name__=='__main__':main()
