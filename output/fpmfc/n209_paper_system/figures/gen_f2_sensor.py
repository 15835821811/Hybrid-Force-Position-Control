from paper_plot_style import *
z=np.load(OLD/'runs/S01_noise_delay/trace.npz');a=read(ROOT/'diagnostics/S01_noise_delay/new_estimator_redecision.json')
t=np.array([x['time'] for x in a]);E=np.array([x['error'] for x in a]);fig,axes=plt.subplots(2,1,figsize=(7.1,4.3),sharex=True,constrained_layout=True)
for ax,sl,scale,label in zip(axes,[slice(6,9),slice(9,12)],[1000,180/np.pi],['Velocity error norm (mm/s)','Angular velocity error norm (deg/s)']):
    mask=(z['time_s']>=1)&(z['time_s']<=7);new=(t>=1)&(t<=7)
    ax.plot(z['time_s'][mask],scale*np.linalg.norm(z['estimate_error'][mask,sl],axis=1),color=COLORS[0],lw=.7,label='N208 on archived S01')
    ax.plot(t[new],scale*np.linalg.norm(E[new,sl],axis=1),color=COLORS[1],ls='--',lw=.8,label='N209 estimator; same packets')
    ax.set_ylabel(label)
axes[0].legend(frameon=False,loc='lower center',bbox_to_anchor=(.5,1.01),ncol=2)
axes[-1].set_xlabel('Absolute time (s)');savefig(fig,'f2_same_packet_estimation')
