from paper_plot_style import *
p=ROOT/'runs/D01_V1_nominal';z=np.load(p/'trace.npz');m=read(p/'metrics.json');cfg=read(p/'config.json')['mission'];t=z['time_s'];latch=m['latch_time_s']
fig,axes=plt.subplots(2,2,figsize=(8.2,5),constrained_layout=True)
axes[0,0].semilogy(t,np.maximum(1e-7,np.rad2deg(np.linalg.norm(z['target_omega_world_rad_s'],axis=1))),color=COLORS[0],label='World')
axes[0,0].semilogy(t,np.maximum(1e-7,np.rad2deg(np.linalg.norm(z['target_base_relative_omega_world_rad_s'],axis=1))),color=COLORS[1],ls='--',label='Relative to base')
axes[0,0].axhline(cfg['performance']['world_deg_s'],color=COLORS[0],lw=.6,ls=':');axes[0,0].axhline(cfg['performance']['relative_deg_s'],color=COLORS[1],lw=.6,ls=':');axes[0,0].set_ylabel('Angular speed (deg/s)');axes[0,0].legend(frameon=False)
axes[0,1].plot(t,z['load_fraction'],color=COLORS[0]);axes[0,1].axhline(1,color=COLORS[1],ls=':');axes[0,1].set_ylabel(r'Actual load utilization $\rho$')
for ax,key,label in [(axes[1,0],'linear_momentum_world_kg_m_s','Linear momentum drift (kg m/s)'),(axes[1,1],'angular_momentum_about_center_world_kg_m2_s','Angular momentum drift (kg m²/s)')]:
    ax.plot(t,np.linalg.norm(z[key]-z[key][0],axis=1),color=COLORS[0]);ax.set_ylabel(label);ax.ticklabel_format(axis='y',style='sci',scilimits=(-2,2))
for ax in axes.flat:
    ax.axvline(latch,color='gray',ls='--',lw=.8);ax.axvspan(*m['evaluation_window_s'],color='gray',alpha=.1);ax.set_xlabel('Absolute time (s)')
savefig(fig,'f3_nominal_continuous_task')
