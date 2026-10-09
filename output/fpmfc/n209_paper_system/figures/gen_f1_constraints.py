from paper_plot_style import *
fig,axes=plt.subplots(1,3,figsize=(9,2.7),constrained_layout=True)
for ax,name,label in zip(axes,['H2_prior','S01_noise_delay','D05_final_nominal'],['H2','S01','D05']):
    rows=read(ROOT/'diagnostics'/name/'snapshots.json');t=np.array([x['time'] for x in rows]);z=[x['qp'][0]['phase_one']['z'] for x in rows]
    ax.plot(t,z,color=COLORS[0],marker='.',label=label);ax.axvline(t[-1],color=COLORS[1],ls='--',lw=.8)
    ax.set_xlabel('Absolute time (s)');ax.set_ylabel('Phase-I z (dimensionless)');ax.legend(frameon=False);ax.ticklabel_format(axis='y',style='sci',scilimits=(-2,2))
savefig(fig,'f1_historical_constraint_failure')
