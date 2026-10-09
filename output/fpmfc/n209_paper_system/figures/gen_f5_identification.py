from paper_plot_style import *
fig,axes=plt.subplots(2,1,figsize=(6.8,4.2),sharex=True,constrained_layout=True)
for name,color,ls in [('H1_prior',COLORS[0],'-'),('H3_prior',COLORS[1],'--')]:
    a=read(ROOT/'diagnostics'/name/'identification_history.json');t=[x['time'] for x in a]
    axes[0].plot(t,[x['com_error_mm'] for x in a],color=color,ls=ls,label=name.split('_')[0]);axes[1].step(t,[x['rank'] for x in a],color=color,ls=ls,where='post')
axes[0].axhline(5,color='gray',ls=':',label='5 mm target');axes[0].set_ylabel('COM error (mm)');axes[0].legend(frameon=False)
axes[1].set_ylabel('Data numerical rank');axes[1].set_xlabel('Absolute time (s)');axes[1].set_yticks([0,5,10]);savefig(fig,'f5_historical_identification')
