"""S03 scientific comparison figures. Saved evidence only; no simulation."""
import argparse
import csv
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from s02_visualization import style,COLORS,STYLES
from v6_mujoco.system_capture.planning.contracts import OUT,PROJECT,read,save,sha
from v6_mujoco.system_capture.planning.report import folder_for
DEST=OUT/'visualizations/figures'

def write(fig,name,caption,sources):
    DEST.mkdir(parents=True,exist_ok=True)
    for ext in ['png','pdf']:fig.savefig(DEST/(name+'.'+ext),bbox_inches='tight')
    plt.close(fig)
    return dict(title=name.replace('_',' '),caption=caption,source_sha256={p.relative_to(PROJECT).as_posix():sha(p) for p in sources})

def authority_signed():
    source=OUT/'diagnostics/authority_signed_margins.csv'
    with source.open(encoding='utf-8',newline='') as f:rows=list(csv.DictReader(f))
    times=sorted({float(x['time_s']) for x in rows});dims=list(read(OUT/'candidate_authority.json')['effects'])
    pairs=['link2_to_link6','link3_to_link7','link6_collision__tumbling_target_geom','link7_collision__tumbling_target_geom','gripper_contact_pad__tumbling_target_geom','n206_spacer__tumbling_target_geom']
    labels=['Link 2 / link 6','Link 3 / link 7','Link 6 / target','Link 7 / target','Pad / target','Spacer / target']
    arrays=[]
    for t in times:
        arrays.append(np.array([[next(float(r['delta_margin_m'])*1000 for r in rows if float(r['time_s'])==t and r['dimension']==d and r['sign']==sign and r['pair']==p) for d in dims for sign in ['-','+']] for p in pairs]))
    bound=max(float(np.max(abs(a))) for a in arrays);fig,axs=plt.subplots(2,2,figsize=(10,6),layout='constrained')
    for ax,t,a in zip(axs.flat,times,arrays):
        im=ax.imshow(a,cmap='RdBu',vmin=-bound,vmax=bound,aspect='auto')
        ax.set_yticks(range(len(labels)),labels,fontsize=8);ax.set_xticks(range(10),[f'{d}{s}' for d in ['Time','Normal','Tangent','Mid','End'] for s in ['−','+']],rotation=55,ha='right',fontsize=8)
        ax.set_ylabel(f'State at {t:.2f} s')
    fig.colorbar(im,ax=axs,label='Candidate minus baseline minimum margin (mm)',shrink=.8)
    return write(fig,'authority_signed','Signed changes for six named pairs at four preregistered B01 states, each using the same 0.6 s prior forecast. Columns change one dimension with its negative/positive perturbation. Positive means increased predicted minimum distance, not task success. All finite pairs and thresholds are in authority_signed_margins.csv; absent-distance sentinels are excluded.',[source])

def common_prefix():
    comp=read(OUT/'same_model_comparison.json');fig,axs=plt.subplots(2,2,figsize=(9,5.8),layout='constrained');sources=[]
    for col,(scene,names) in enumerate([('Nominal',['B00','P00','P01']),('H2',['B01','P02','P03'])]):
        end=min(comp['runs'][n]['end_time_s'] for n in names)
        for i,n in enumerate(names):
            f=folder_for(n);sources.append(f/'trace.npz')
            with np.load(f/'trace.npz') as z:
                mask=z['time_s']<=end+1e-9;t=z['time_s'][mask];active=z['approach_reference_applicable'][mask].astype(bool)
                axs[0,col].plot(t,np.rad2deg(np.linalg.norm(z['base_omega_world_rad_s'][mask],axis=1)),STYLES[i],label=f'{n} / B{i}',color=COLORS[i])
                axs[1,col].plot(t[active],z['approach_position_error_m'][mask][active]*1000,STYLES[i],color=COLORS[i])
        axs[0,col].set_ylabel(f'{scene}: base speed (deg/s)');axs[1,col].set_ylabel(f'{scene}: tracking error (mm)');axs[1,col].set_xlabel('Common executed time (s)');axs[0,col].legend(fontsize=8)
    return write(fig,'common_prefix','B0/B1/B2 actual base angular speed and active-reference tracking error, truncated to the shortest executed interval in each scene. This figure excludes later baseline capture and does not establish complete-task benefit, risk reduction or postgrasp performance. Runs are deterministic seen-condition comparisons, not independent statistical samples.',sources)

def forecast_errors():
    names=['P00','P01','P02','P03'];fig,axs=plt.subplots(2,2,figsize=(9,5.6),layout='constrained');sources=[]
    keys=['q_error_rad','dq_error_rad_s','p_error_m','R_error_rad'];labels=['Max joint-position deviation (rad)','Max joint-speed deviation (rad/s)','Target-position deviation (mm)','Target-attitude deviation (deg)'];scales=[1,1,1000,180/np.pi]
    for i,n in enumerate(names):
        f=folder_for(n)/'planner_cache_errors.json';rows=read(f);sources.append(f)
        for ax,k,label,scale in zip(axs.flat,keys,labels,scales):
            ax.plot([r['time'] for r in rows],[r[k]*scale for r in rows],STYLES[i],color=COLORS[i],label=n);ax.set(xlabel='Executed time (s)',ylabel=label)
    axs[0,0].legend(ncol=2,fontsize=8)
    return write(fig,'forecast_errors','Public joint measurements and CA18 target estimates against the selected independent-prior cached forecast at actual cache-validation ticks. Traces end at the last cache check before termination (P00/P01/P02/P03: 8.02/7.22/6.42/6.42 s). Replanning resets the forecast horizon. These are cache-domain diagnostics, not a global prediction error guarantee; truth is not fed to the planner.',sources)

def main(which=None):
    style();funcs={'authority_signed':authority_signed,'common_prefix':common_prefix,'forecast_errors':forecast_errors}
    manifest=read(DEST/'scientific_manifest.json') if (DEST/'scientific_manifest.json').exists() else {}
    for k,fn in funcs.items():
        if which is None or which==k:manifest[k]=fn()
    save(DEST/'scientific_manifest.json',manifest)
    snippets=[]
    for k,r in manifest.items():
        cap=r['caption'].replace('_',r'\_')
        snippets.append('\\begin{figure}[t]\n\\centering\n\\includegraphics[width=0.95\\linewidth]{'+k+'.pdf}\n\\caption{'+cap+'}\n\\label{fig:s03-'+k.replace('_','-')+'}\n\\end{figure}\n')
        (DEST/('gen_'+k+'.py')).write_text('"""Run from any directory in this checked-out repository."""\nimport runpy\nfrom pathlib import Path\np=Path(__file__).resolve()\nroot=next(x for x in p.parents if (x/"v6_mujoco").is_dir())\nmod=runpy.run_path(str(root/"tools/s03_scientific_figures.py"))\nmod["main"]("'+k+'")\n',encoding='utf-8')
    (DEST/'latex_includes.tex').write_text('\n'.join(snippets),encoding='utf-8')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--figure');main(p.parse_args().figure)
