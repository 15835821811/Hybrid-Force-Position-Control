import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from .common import OUT
plt.rcParams.update({'font.family':'serif','font.serif':['Times New Roman','DejaVu Serif'],
    'font.size':10,'axes.labelsize':10,'legend.fontsize':8,'xtick.labelsize':9,'ytick.labelsize':9,
    'axes.spines.top':False,'axes.spines.right':False,'savefig.dpi':300,'mathtext.fontset':'stix'})
FIG=OUT/'figures'
COLORS=['#0072B2','#D55E00','#009E73']
def savefig(fig,name):
    FIG.mkdir(exist_ok=True)
    for ext in ('pdf','png'):fig.savefig(FIG/(name+'.'+ext),bbox_inches='tight',pad_inches=.08)
    plt.close(fig)
