"""Four trace-derived S01 scientific figure groups; no simulation steps."""
import html
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from .common import OUT,PROJECT,read,save,sha,source,assumptions
from .kinematics import IndependentSRS

DEST=OUT/'figures'
COLORS=['#0072B2','#D55E00','#009E73','#CC79A7','#444444','#E69F00']
LINES=['-','--','-.',':','-','--']
TITLES={
 'model_and_frames':'SRS 模型与坐标',
 'mathematics_and_hierarchy':'数学核验与层级残差',
 'optimization_and_base_comparison':'优化与基座扰动比较',
 'tracking_and_dynamics':'实际轨迹跟踪与动力学'}
CAPTIONS={
 'model_and_frames':'Initial SRS centerlines in three world-coordinate projections, from the published DH table and separately recorded joint angles. S, E and W denote shoulder, elbow and wrist. The DH0 mount is the declared Rx(pi) assumption; initial base and world frames coincide. Graphics are schematic, not collision envelopes or mass-support geometry. Published masses total 524 kg; link COMs and inertia axes require declared assumptions.',
 'mathematics_and_hierarchy':'Eleven preregistered configurations: index 0 is the initial state, and indices 1-10 follow the fixed sine-perturbation rule in s01.json. (a) Each residual is normalized by its corresponding numerical tolerance; unity is the acceptance boundary. FK position is maximum component error (tolerance 1e-10 m); full Jacobian is maximum entry error (1e-8 in the stated mixed linear/angular coordinates); zero-momentum map is maximum entry residual (1e-10 in its stated SI coordinates); A/B shape rate is absolute difference (1e-6 rad/s). (b) Eq. (23) under two readings of its undefined vector l, versus the independent derivative of Eq. (18). (c,d) Translational and angular base motion per unit nullspace speed, shown separately with zero as the reactionless reference. Values below 1e-17 in logarithmic displays are plotted at 1e-17, a display floor only.',
 'optimization_and_base_comparison':'Three strategies (fixed shape 0, fixed shape pi/2, and joint time/shape optimization) and two fixed seeds (240601,240602) receive 120 kinematic evaluations each. G is squared world base angular-speed peak normalized by 5 deg/s plus squared grasp-direction alignment normalized by pi, with weights 1/1; definitions and gates are frozen in assumptions.yaml. (a) Feasible-only best objective by generation; gaps mean no feasible incumbent. (b) Selected time and shape; numbers refer to the legend, with a gray x for an infeasible candidate. (c,d) Actual dynamic base angular-speed peaks and attitude drift for the common-time protocol A. An x denotes an unqualified trajectory, annotated with its actual end time; its statistic uses that recorded interval only and is not an admissible benefit comparison. Two seeds provide descriptive results only; reduced-budget stagnation does not establish convergence.',
 'tracking_and_dynamics':'Actual torque-driven SRS traces at the author-reported point (15.6 s, 0.2686 rad; source-nullspace and adapted HQP) and at the preregistered common-time joint result. Position, rotation and shape errors, world base speed, separate P/H drift about the fixed world origin, actual applied torque and actual joint speed are shown. Curves end at the true stopping time: x marks an unqualified run and a circle a qualified run. Horizontal dotted lines are frozen path gates (5 mm, 1 deg, 2 deg), P/H drift gates (1e-6 kg m/s and 1e-6 kg m²/s), and torque/joint-speed limits (50 N m, 0.8 rad/s). There is no fabricated terminal segment. Plotted failed trajectories do not establish source reproduction.'}

def style():
    plt.rcParams.update({'font.family':'serif','font.serif':['Times New Roman','DejaVu Serif'],
       'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'savefig.dpi':300,
       'mathtext.fontset':'stix','pdf.fonttype':42,'ps.fonttype':42,'legend.fontsize':8})
def archive(key):
    with np.load(OUT/'dynamics_runs'/key/'trace.npz',allow_pickle=False) as z:return {k:z[k] for k in z.files}
def finish(fig,name,inputs,detail=''):
    DEST.mkdir(parents=True,exist_ok=True)
    for ax in fig.axes:ax.grid(alpha=.15)
    for ext in ['png','pdf']:fig.savefig(DEST/(name+'.'+ext),bbox_inches='tight')
    plt.close(fig)
    entry={'title':TITLES[name],'caption':CAPTIONS[name]+detail,'inputs':{str(p.relative_to(PROJECT)).replace('\\','/'):sha(p) for p in inputs},
      'files':{ext:sha(DEST/(name+'.'+ext)) for ext in ['png','pdf']},'generator_sha256':sha(Path(__file__))}
    save(DEST/(name+'.json'),entry)
    return entry

def model_and_frames():
    q=np.deg2rad(source()['initial']['joint_angles_deg']);k=IndependentSRS();f=k.fk(q);s=k.shape(q)
    pts=np.vstack(([0.,0,0],f.ends))
    fig,axs=plt.subplots(1,3,figsize=(9,3.5),layout='constrained')
    for ax,(i,j) in zip(axs,[(0,1),(0,2),(1,2)]):
        ax.plot(pts[:,i],pts[:,j],'-o',color=COLORS[0],ms=3)
        for label,point,mark in [('S',s.S,'s'),('E',s.E,'D'),('W',s.W,'^')]:
            ax.scatter(point[i],point[j],marker=mark,s=28,color=COLORS[1])
            offset=(-17,-13) if label=='S' and (i,j)==(0,2) else (5,5)
            ax.annotate(label,point[[i,j]],xytext=offset,textcoords='offset points')
        ax.scatter(0,0,marker='+',color='black',s=75,label='Base / world origin')
        ax.set(xlabel='World '+'XYZ'[i]+' (m)',ylabel='World '+'XYZ'[j]+' (m)')
        ax.set_aspect('equal',adjustable='datalim');ax.margins(.18)
    axs[0].legend(loc='upper left',bbox_to_anchor=(0,1.15))
    return finish(fig,'model_and_frames',[PROJECT/'models/paper_compat/srs.xml',PROJECT/'configs/system_capture/paper_compat/source_parameters.yaml',PROJECT/'configs/system_capture/paper_compat/assumptions.yaml'])

def mathematics_and_hierarchy():
    data=read(OUT/'equation_tests.json');rows=data['states'];x=np.arange(len(rows));tol=data['tolerances']
    fig,axs=plt.subplots(2,2,figsize=(9,6),layout='constrained')
    for idx,(key,t,label) in enumerate([('fk_position_m',tol['fk_m_rad'],'FK position'),('full_jacobian_max_abs',tol['jacobian'],'Full Jacobian'),('momentum_zero_map',tol['momentum_map'],'Zero-momentum map'),('AB_shape_velocity_error',tol['hierarchy'],'A/B shape rate')]):
        axs[0,0].semilogy(x,np.maximum([r[key]/t for r in rows],1e-17),LINES[idx],color=COLORS[idx],label=label)
    axs[0,0].axhline(1,color='.3',ls=':');axs[0,0].set_ylabel('Residual / numerical tolerance');axs[0,0].legend(ncol=2)
    for idx,(key,label) in enumerate([('eq23_l_equals_w_cross_V_error','l = w × V'),('eq23_l_equals_projected_LV_error','l = (w × V) × unit(w)')]):
        axs[0,1].semilogy(x,np.maximum([r[key] for r in rows],1e-17),LINES[idx],color=COLORS[idx],label=label)
    axs[0,1].set_ylabel('Shape derivative residual (rad/rad)');axs[0,1].legend()
    v=np.array([r['A_N_world_twist'] for r in rows])
    for ax,col,label in [(axs[1,0],slice(0,3),'Base translation per unit null speed (m/rad)'),(axs[1,1],slice(3,6),'Base rotation per unit null speed (rad/rad)')]:
        ax.scatter(x,np.linalg.norm(v[:,col],axis=1),s=18,color=COLORS[0]);ax.axhline(0,color='.4',ls=':');ax.set_ylim(bottom=0);ax.set_ylabel(label)
    for i,ax in enumerate(axs.flat):ax.set_xlabel('Preregistered configuration');ax.text(0,1.03,'('+chr(97+i)+')',transform=ax.transAxes,va='bottom')
    return finish(fig,'mathematics_and_hierarchy',[OUT/'equation_tests.json'])

def optimization_and_base_comparison():
    files=sorted((OUT/'optimization_runs').glob('*/result.json'));cells=[read(p) for p in files]
    fixed=read(OUT/'fixed_time_comparison.json');fig,axs=plt.subplots(2,2,figsize=(9,6.5),layout='constrained')
    for idx,c in enumerate(cells):
        hist=np.array(c['history'],dtype=float)
        valid=np.array(c['history_feasible'],dtype=bool)
        y=np.where(valid,hist,np.nan)
        label=str(idx+1)+': '+str(c['seed'])+' / '+c['strategy'].replace('fixed_','ψ=').replace('pi2','π/2')
        axs[0,0].plot(np.arange(1,len(hist)+1),y,LINES[idx],color=COLORS[idx],marker=['o','s','^','D','v','P'][idx],ms=3,label=label)
        if c['best_rollout']['feasible']:
            axs[0,1].scatter(*c['best_position'],marker=['o','s','^','D','v','P'][idx],s=50,facecolors='none',edgecolors=COLORS[idx])
        else:axs[0,1].scatter(*c['best_position'],marker='x',s=50,color='.5')
        axs[0,1].annotate(str(idx+1),c['best_position'],xytext=(5,3+idx%2*8),textcoords='offset points',fontsize=8)
    axs[0,0].set(xlabel='Generation (20 evaluations each)',ylabel='Best feasible objective G')
    handles,legend_labels=axs[0,0].get_legend_handles_labels()
    fig.legend(handles,legend_labels,loc='upper center',bbox_to_anchor=(.5,1.10),ncol=3,fontsize=8)
    axs[0,1].set(xlabel='Selected time (s)',ylabel='Selected terminal shape (rad)')
    axs[0,1].margins(x=.18,y=.18)
    labels=[{'psi0':'ψ = 0','psi90':'ψ = π/2','joint':'Joint'}[r['label']] for r in fixed['rows']]
    for ax,key,ylabel in [(axs[1,0],'base_peak_angular_speed_rad_s','Base angular-speed peak (deg/s)'),(axs[1,1],'base_max_attitude_drift_rad','Base attitude drift maximum (deg)')]:
        for idx,r in enumerate(fixed['rows']):
            ax.scatter(idx,np.rad2deg(r['dynamics'][key]),marker='o' if r['dynamics']['feasible'] else 'x',color=COLORS[idx],s=60)
            if not r['dynamics']['feasible']:ax.annotate('end '+format(r['dynamics']['end_time_s'],'.2f')+' s',(idx,np.rad2deg(r['dynamics'][key])),xytext=(2,8),textcoords='offset points',fontsize=8)
        ax.set_xticks(range(3),labels);ax.set_ylabel(ylabel);ax.set_xlabel('Common-time protocol A')
        ax.set_ylim(0,1.12*max(np.rad2deg(r['dynamics'][key]) for r in fixed['rows']))
    for idx,ax in enumerate(axs.flat):ax.text(0,1.03,'('+chr(97+idx)+')',transform=ax.transAxes,va='bottom')
    detail=' Protocol A uses the joint result of seed 240601 by the preregistered rule, independent of which seed has the lower objective. Qualification covers the declared inertial model and gates, not physical collision clearance or contact. Common time = '+format(fixed['common_T_s'],'.6f')+' s. '+'; '.join(r['label']+': '+r['dynamics']['status']+', qualified='+str(r['dynamics']['feasible']) for r in fixed['rows'])+'.'
    return finish(fig,'optimization_and_base_comparison',files+[OUT/'fixed_time_comparison.json'],detail)

def tracking_and_dynamics():
    author=read(OUT/'author_point_check.json');joint=read(OUT/'fixed_time_comparison.json')['rows'][2]
    cases=[('Author / source',author['run_keys']['source']),('Author / HQP',author['run_keys']['hqp']),('Common / joint',joint['dynamics_run'])]
    fig,axs=plt.subplots(4,2,figsize=(10,11),layout='constrained');a=assumptions();inputs=[];details=[]
    for idx,(label,key) in enumerate(cases):
        z=archive(key);t=z['time_s'];m=read(OUT/'dynamics_runs'/key/'metrics.json')
        inputs.extend([OUT/'dynamics_runs'/key/'trace.npz',OUT/'dynamics_runs'/key/'metrics.json'])
        vals=[np.linalg.norm(z['position_error_m'],axis=1)*1000,np.rad2deg(np.linalg.norm(z['rotation_error_rad'],axis=1)),
          np.rad2deg(abs(z['shape_error_rad'])),np.rad2deg(np.linalg.norm(z['base_angular_velocity_world_rad_s'],axis=1)),
          np.linalg.norm(z['momentum_world_origin'][:,:3]-z['momentum_world_origin'][0,:3],axis=1),
          np.linalg.norm(z['momentum_world_origin'][:,3:]-z['momentum_world_origin'][0,3:],axis=1),
          np.max(abs(z['applied_ctrl']),axis=1),np.max(abs(z['qvel'][:,6:]),axis=1)]
        for j,(ax,v) in enumerate(zip(axs.flat,vals)):
            tt=t[:-1] if j==6 else t;ax.plot(tt,v,LINES[idx],color=COLORS[idx],label=label,lw=1)
            ax.scatter(tt[-1],v[-1],marker='o' if m['feasible'] else 'x',s=20,color=COLORS[idx])
        details.append(label+': '+m['status']+', end='+format(t[-1],'.6f')+' s')
    labels=['Position error (mm)','Rotation error (deg)','Shape error (deg)','Base angular speed (deg/s)','P drift (kg m/s)','H drift (kg m²/s)','Max actual applied torque (N m)','Max actual joint speed (rad/s)']
    gates=[a['gates']['path_position_error_m']*1000,np.rad2deg(a['gates']['path_rotation_error_rad']),np.rad2deg(a['gates']['path_shape_error_rad']),None,a['gates']['linear_momentum_drift_kg_m_s'],a['gates']['angular_momentum_drift_world_origin_kg_m2_s'],a['controller']['torque_limit_nm'],a['controller']['joint_velocity_limit_rad_s']]
    for i,(ax,label,gate) in enumerate(zip(axs.flat,labels,gates)):
        ax.set(xlabel='Physical time (s)',ylabel=label)
        if gate is not None:ax.axhline(gate,color='.4',ls=':',lw=.8)
        if i in [4,5,6]:ax.set_yscale('symlog',linthresh=1e-12 if i<6 else .01)
        ax.text(0,1.03,'('+chr(97+i)+')',transform=ax.transAxes,va='bottom')
    handles,legend_labels=axs[0,0].get_legend_handles_labels()
    fig.legend(handles,legend_labels,loc='upper center',bbox_to_anchor=(.5,1.025),ncol=3)
    return finish(fig,'tracking_and_dynamics',inputs,' Qualification covers the declared inertial model and gates, not physical collision clearance or contact. '+'; '.join(details)+'.')

def run(only=None):
    style();entries={}
    for name in TITLES:
        if only is None or only==name:entries[name]=globals()[name]()
    if only:return entries
    snippets=[]
    for name,item in entries.items():
        snippets.append('\\begin{figure}[t]\n\\centering\n\\includegraphics[width=0.95\\textwidth]{'+name+'.pdf}\n\\caption{'+item['caption'].replace('%','\\%').replace('_','\\_')+'}\n\\label{fig:s01-'+name.replace('_','-')+'}\n\\end{figure}\n')
        (DEST/('gen_'+name+'.py')).write_text("from pathlib import Path\nimport sys\nsys.path.insert(0,str(Path(__file__).resolve().parents[5]))\nfrom v6_mujoco.system_capture.paper_bridge.figures import run\nif __name__ == '__main__': run("+repr(name)+")\n",encoding='utf-8')
    (DEST/'latex_includes.tex').write_text('\n'.join(snippets),encoding='utf-8')
    (DEST/'paper_plot_style.py').write_text('from v6_mujoco.system_capture.paper_bridge.figures import style\nstyle()\n',encoding='utf-8')
    save(DEST/'manifest.json',entries)
    md='# S01 四组核心图\n\n仅接触前 SRS；来源限制与失败保留。\n\n'
    body='<meta charset="utf-8"><title>S01 scientific figures</title><style>body{max-width:1100px;margin:30px auto;font:16px/1.6 system-ui}img{width:100%}</style><h1>S01 四组核心图</h1><p>接触前 SRS 基准；早停轨迹不延长。</p>'
    for name,item in entries.items():
        md+='## '+item['title']+'\n\n[PDF]('+name+'.pdf)\n\n!['+item['title']+']('+name+'.png)\n\n'+item['caption']+'\n\n'
        body+='<h2>'+item['title']+'</h2><a href="'+name+'.pdf">PDF</a><img src="'+name+'.png"><p>'+html.escape(item['caption'])+'</p>'
    (DEST/'README.md').write_text(md.rstrip()+'\n',encoding='utf-8');(DEST/'index.html').write_text(body,encoding='utf-8')
    return entries
