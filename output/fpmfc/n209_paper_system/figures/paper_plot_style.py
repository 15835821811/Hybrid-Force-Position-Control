from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
matplotlib.rcParams.update({'font.size':10,'font.family':'serif','font.serif':['Times New Roman','DejaVu Serif'],
    'axes.labelsize':10,'xtick.labelsize':9,'ytick.labelsize':9,'legend.fontsize':8,
    'savefig.dpi':300,'savefig.bbox':'tight','savefig.pad_inches':.08,'axes.spines.top':False,'axes.spines.right':False,
    'mathtext.fontset':'stix','pdf.fonttype':42,'ps.fonttype':42})
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent;PROJECT=HERE.parents[3];OLD=PROJECT/'output/fpmfc/n208_adaptive_capture'
COLORS=['#0072B2','#D55E00','#009E73','#CC79A7']
def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def savefig(fig,name):
    fig.savefig(HERE/(name+'.pdf'));fig.savefig(HERE/(name+'.png'));plt.close(fig)
