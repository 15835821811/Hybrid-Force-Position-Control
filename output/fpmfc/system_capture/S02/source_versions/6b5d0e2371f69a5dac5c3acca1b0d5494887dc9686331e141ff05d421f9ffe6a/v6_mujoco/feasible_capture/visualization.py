"""Post-process every saved N209 attempt; never advance a physical simulation."""
import argparse
import html
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from .common import ROOT, PAPER, PROJECT_ROOT, read, save, sha

OUT = ROOT / 'visualizations'
SELECTED = 'R01_V2_nominal'
COLORS = ['#0072B2', '#D55E00', '#009E73', '#CC79A7', '#E69F00', '#444444', '#56B4E9']
STYLES = ['-', '--', '-.', ':', (0, (5, 1, 1, 1)), (0, (3, 1, 1, 1, 1, 1)), (0, (1, 2))]
CAPTIONS = {
 'trajectory': ('末端实际与参考轨迹', 'World XY/XZ paths of the actual flange and the recorded live reference, only while the approach reference is applicable. Equal spatial scales.'),
 'tracking': ('末端轨迹跟踪误差', 'Actual flange minus the recorded live reference during approach: world position components, position norm, and SO(3) rotation error. No inactive post-latch reference is extended.'),
 'reference_progress': ('参考进度与治理候选', 'Virtual path time in seconds, virtual-time rate, acceleration and selected candidate. A red x means no verified candidate, not an accepted zero-speed command.'),
 'state_estimation': ('状态估计误差与测量年龄', 'Geometric-origin estimation errors at valid estimator-output ticks. Shading is the maximum of three marginal 3-sigma values, not joint coverage or a safety certificate.'),
 'capture_and_load': ('实际捕获量与接口载荷', 'Independent physical capture quantities and actual load. Dashed pose lines denote capture gates; dotted post-latch lines denote holding gates. Values are not corrected by subtracting numerical residuals.'),
 'detumbling': ('世界与相对角速度', 'Target world and target-base relative angular-speed norms. The fixed final two-second window is shaded only for completed full-horizon runs.'),
 'momentum_energy': ('动量、能量与功', 'Separate linear/angular momentum drift, kinetic/relative energy and integrated work. Momentum and actual safety quantities retain their original SI definitions.'),
 'joints': ('七关节状态与实际力矩', 'All seven joint positions, velocities and actual actuator forces from the saved physical trajectory. Line style and color identify each joint.'),
 'geometry': ('碰撞距离与接口几何', 'Original and added robot-clearance pairs, other target pairs and intended interface contact are shown with their separate original thresholds.'),
 'identification': ('影子惯性辨识', 'Shadow posterior parameters and finite-data rank; evaluation-only truth is dashed in the matching component color. The logarithmic singular-value display floors values below 1e-12; this floor is not measured excitation. Parameter feedback remains disabled. Numerical rank is not parameter accuracy or control benefit.'),
 'base_motion': ('被动基座漂移与角速度', 'Passive-base translation and geodesic attitude change relative to its initial pose, followed by world angular-velocity components. No base actuator is introduced.'),
}

def load_run(name):
    if name not in [x['name'] for x in read(ROOT/'run_ledger.json')['attempts']]:
        raise ValueError('Unknown recorded attempt: '+name)
    folder=ROOT/'runs'/name; metrics=read(folder/'metrics.json'); cfg=read(folder/'config.json')
    validation=read(folder/'validation_timestamp_audited.json'); digest=sha(folder/'trace.npz')
    assert validation['actuator_replay_passed'] and validation['decision_replay_passed']
    assert digest==metrics['trace_sha256']==validation['trace_sha256']
    with np.load(folder/'trace.npz', allow_pickle=False) as archive:
        arrays={k:archive[k] for k in archive.files}
    return folder, arrays, metrics, cfg, digest

def style():
    plt.rcParams.update({'font.family':'serif','font.serif':['Times New Roman','DejaVu Serif'],
        'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'savefig.dpi':300,
        'mathtext.fontset':'stix','pdf.fonttype':42,'ps.fonttype':42,
        'axes.prop_cycle':matplotlib.cycler(color=COLORS)})

def no_data(ax, message):
    ax.text(.5,.5,message,ha='center',va='center',transform=ax.transAxes,color='.35')

def plot_run(name, only=None):
    style(); folder,a,m,cfg,digest=load_run(name); t=a['time_s']; latch=m['latch_time_s']
    dest=OUT/'runs'/name; dest.mkdir(parents=True,exist_ok=True)
    active=a['approach_reference_applicable'].astype(bool)
    valid=a['estimate_valid'].astype(bool)&a['estimate_current_tick'].astype(bool)
    manifest=read(dest/'figure_manifest.json') if only and (dest/'figure_manifest.json').exists() else {}
    checks={'position_error_max_difference_m':0.,'all_times_match_saved_trace':True}
    if active.any():
        e=a['flange_position_world_m'][active]-a['approach_reference_position_world_m'][active]
        checks['position_error_max_difference_m']=float(np.max(abs(np.linalg.norm(e,axis=1)-a['approach_position_error_m'][active])))
        assert checks['position_error_max_difference_m']<1e-10
    for key,(title,caption) in CAPTIONS.items():
        if only and only!=key: continue
        count={'trajectory':2,'tracking':3,'reference_progress':4,'state_estimation':5,'capture_and_load':6,
               'detumbling':2,'momentum_energy':4,'joints':3,'geometry':2,'identification':6,'base_motion':3}[key]
        fig,axs=plt.subplots(1,2,figsize=(7.3,4.0),layout='constrained') if key=='trajectory' else plt.subplots(count,1,figsize=(7.3,2.05*count),sharex=True,layout='constrained')
        axs=np.atleast_1d(axs)
        if key=='trajectory':
            for ax,j in zip(axs,[1,2]):
                if active.any():
                    for col,ls,label in [('flange_position_world_m','-','Actual flange'),('approach_reference_position_world_m','--','Live reference')]:
                        p=a[col][active];ax.plot(p[:,0],p[:,j],ls,label=label);ax.scatter(p[[0,-1],0],p[[0,-1],j],s=12)
                else: no_data(ax,'No active approach reference before stop')
                ax.set(xlabel='World X (m)',ylabel=f'World {"XYZ"[j]} (m)');ax.set_aspect('equal',adjustable='box')
            if active.any(): axs[0].legend(loc='upper center',bbox_to_anchor=(.5,1.18),ncol=2,fontsize=8)
        elif key=='tracking':
            if active.any():
                e=(a['flange_position_world_m']-a['approach_reference_position_world_m'])*1000
                for j in range(3):axs[0].plot(t[active],e[active,j],STYLES[j],label='XYZ'[j],lw=.9)
                axs[0].legend(ncol=3);axs[1].plot(t[active],a['approach_position_error_m'][active]*1000)
                axs[2].plot(t[active],np.rad2deg(a['approach_rotation_error_rad'][active]))
            else:
                for ax in axs:no_data(ax,'NOT_EVALUATED: no active reference')
            for ax,label in zip(axs,['Position components (mm)','Position norm (mm)','Rotation error (deg)']):ax.set_ylabel(label)
        elif key=='reference_progress':
            axs[0].plot(t,a['progress_s']);axs[0].set_ylabel('Virtual path time (s)')
            axs[1].plot(t,a['reference_rate']);axs[1].set_ylabel('Virtual-time rate (s/s)')
            axs[2].plot(t,a['reference_acceleration']);axs[2].set_ylabel('Virtual acceleration (1/s)')
            g=read(folder/'progress_governor.json')
            for x in g:
                y=x['chosen_fraction'];axs[3].plot(x['time'],y if y is not None else -.08,'o' if y is not None else 'x',color=COLORS[0] if y is not None else COLORS[1],ms=3)
            axs[3].set(ylabel='Accepted demand fraction',ylim=(-.15,1.08))
            if not g:no_data(axs[3],'No governor evaluation recorded')
        elif key=='state_estimation':
            labels=['Position error (mm)','Rotation error (deg)','Velocity error (mm/s)','Angular rate error (deg/s)']
            for j,(ax,scale,label) in enumerate(zip(axs[:4],[1000,180/np.pi,1000,180/np.pi],labels)):
                for k in range(3):ax.plot(t[valid],a['estimate_error'][valid,3*j+k]*scale,STYLES[k],lw=.8,label='XYZ'[k])
                if valid.any():
                    bound=3*np.sqrt(np.max(np.diagonal(a['estimate_covariance'][valid,3*j:3*j+3,3*j:3*j+3],axis1=1,axis2=2),axis=1))*scale
                    ax.fill_between(t[valid],-bound,bound,color='.82',alpha=.5,label='max marginal 3-sigma')
                ax.set_ylabel(label)
            axs[0].legend(ncol=4,fontsize=8);axs[4].plot(t[valid],a['measurement_age_s'][valid]*1000);axs[4].set_ylabel('Measurement age (ms)')
        elif key=='capture_and_load':
            vals=[a['interface_translation_error_m']*1000,a['interface_rotation_error_deg'],np.linalg.norm(a['interface_relative_twist_world'][:,:3],axis=1)*1000,np.rad2deg(np.linalg.norm(a['interface_relative_twist_world'][:,3:],axis=1))]
            for ax,v,gate,label in zip(axs[:4],vals,[.1,.05,1.,.2],['Interface gap (mm)','Relative angle (deg)','Relative speed (mm/s)','Relative rate (deg/s)']):
                ax.plot(t,v);ax.axhline(gate,color='.35',ls='--');ax.set(ylabel=label,yscale='symlog');ax.set_yscale('symlog',linthresh=gate);ax.set_ylim(bottom=0)
            if latch is not None:
                axs[0].plot([latch,t[-1]],[.5,.5],':',color='.3');axs[1].plot([latch,t[-1]],[.1,.1],':',color='.3')
            axs[4].plot(t,a['contact_peak_force_n'],label='Contact peak');axs[4].plot(t,np.linalg.norm(a['force_grasp_n'],axis=1),'--',label='Interface resultant');axs[4].set_ylabel('Actual force (N)');axs[4].legend(fontsize=8)
            axs[5].plot(t,a['load_fraction']);axs[5].axhline(1,color='.35',ls='--');axs[5].set_ylabel('Actual load fraction')
        elif key=='detumbling':
            for ax,k,gate,label in zip(axs,['target_omega_world_rad_s','target_base_relative_omega_world_rad_s'],[.1,.02],['World rate (deg/s)','Relative rate (deg/s)']):
                ax.plot(t,np.maximum(1e-12,np.rad2deg(np.linalg.norm(a[k],axis=1))));ax.axhline(gate,color='.35',ls='--');ax.set(ylabel=label,yscale='log')
                if m['full_window_evaluated']:ax.axvspan(*m['evaluation_window_s'],color='.85',alpha=.6)
        elif key=='momentum_energy':
            for ax,k,gate,label in zip(axs[:2],['linear_momentum_world_kg_m_s','angular_momentum_about_center_world_kg_m2_s'],[1e-4,1e-5],['P drift (kg m/s)','H drift (kg m²/s)']):
                ax.plot(t,np.linalg.norm(a[k]-a[k][0],axis=1));ax.axhline(gate,color='.35',ls='--');ax.set_ylabel(label);ax.set_yscale('symlog',linthresh=1e-12);ax.set_ylim(bottom=0)
            axs[2].plot(t,a['kinetic_energy_j'],label='Kinetic');axs[2].plot(t,a['relative_energy_j'],'--',label='Relative');axs[2].set_ylabel('Energy (J)');axs[2].legend()
            axs[3].plot(t,a['kinetic_energy_j']-a['kinetic_energy_j'][0],label='Kinetic change')
            for k,label,ls in [('actuator_power_w','Actuator','--'),('passive_power_w','Passive',':'),('all_constraint_power_w','Constraint','-.')]:
                axs[3].plot(t,np.r_[0,np.cumsum(.5*(a[k][1:]+a[k][:-1])*np.diff(t))],ls,label=label)
            axs[3].set_ylabel('Energy / work (J)');axs[3].legend(ncol=4,fontsize=8)
        elif key=='joints':
            for ax,k,label in zip(axs,['joint_position_rad','joint_velocity_rad_s','actuator_force_nm'],['Position (rad)','Velocity (rad/s)','Actual torque (N m)']):
                for j in range(7):ax.plot(t,a[k][:,j],ls=STYLES[j],label=str(j+1),lw=.8)
                ax.set_ylabel(label)
            axs[0].legend(ncol=7,fontsize=8)
        elif key=='geometry':
            from v6_mujoco.geometry_capture.design import compile_model,extra_pairs
            pairs=extra_pairs(compile_model())
            for ax,category,label,ls in [(axs[0],'tool_robot','Added tool pairs','--'),(axs[1],'target_distance','Other target pairs','-'),(axs[1],'intended_surface','Intended contact',':')]:
                ids=[j for j,p in enumerate(pairs) if p.category==category];ax.plot(t,np.min(a['extra_pair_distances_m'][:,ids],axis=1)*1000,ls,label=label)
            axs[0].plot(t,a['minimum_noncontact_clearance_m']*1000,label='Original pairs');axs[0].axhline(40,color='.35',ls=':');axs[0].set_ylabel('Robot clearance (mm)')
            axs[1].axhline(1,color='.35',ls='--');axs[1].axhline(-2,color='.35',ls=':');axs[1].set_ylabel('Target clearance (mm)')
            for ax in axs:ax.legend(fontsize=8)
        elif key=='identification':
            from v6_mujoco.adaptive_capture.momentum_regressor import parameters
            truth=cfg['truth_evaluation_only'];pi=a['parameter_pi'];truthpi=parameters(truth['mass'],truth['com'],truth['inertia'])
            axs[0].plot(t,pi[:,0]);axs[0].axhline(truth['mass'],color='.35',ls='--');axs[0].set_ylabel('Shadow mass (kg)')
            for ax,cols,truthvals,label,names in [(axs[1],pi[:,1:4]/pi[:,:1]*1000,np.array(truth['com'])*1000,'COM in T (mm)',['x','y','z']),(axs[2],pi[:,4:7],truthpi[4:7],'Origin inertia (kg m²)',['xx','yy','zz']),(axs[4],pi[:,7:10],truthpi[7:10],'Inertia products (kg m²)',['xy','xz','yz'])]:
                for j in range(3):
                    ax.plot(t,cols[:,j],color=COLORS[j],ls=STYLES[j],label=names[j])
                    ax.axhline(truthvals[j],color=COLORS[j],ls='--',lw=.6,label=names[j]+' truth')
                ax.set_ylabel(label);ax.legend(ncol=3,fontsize=8)
            axs[3].step(t,a['parameter_rank'],where='post');axs[3].set(ylabel='Data rank / 10',ylim=(-.2,10.2))
            posterior=read(folder/'posteriors.json')
            if posterior:
                pt=np.array([x['time'] for x in posterior]);sv=np.array([x['singular_values'] for x in posterior]);threshold=np.maximum(cfg['mission']['information_singular_min'],sv[:,0]*cfg['mission']['information_relative_min'])
                for j in range(10):axs[5].semilogy(pt,np.maximum(sv[:,j],1e-12),lw=.5,color='.7')
                axs[5].semilogy(pt,np.maximum(sv[:,0],1e-12),label='Largest');axs[5].semilogy(pt,np.maximum(sv[:,-1],1e-12),':',label='Smallest');axs[5].semilogy(pt,threshold,'k--',label='Rank threshold');axs[5].legend(fontsize=8,loc='center right')
                if not np.any(sv):
                    axs[5].text(.02,.92,'All recorded singular values are zero; display floor = 1e-12',transform=axs[5].transAxes,va='top',fontsize=8,color='.35')
            else:no_data(axs[5],'No posterior update before stop')
            axs[5].set_ylabel('Scaled singular value')
        elif key=='base_motion':
            base=a['qpos'][:,:7];rots=Rotation.from_quat(base[:,[4,5,6,3]])
            axs[0].plot(t,np.linalg.norm(base[:,:3]-base[0,:3],axis=1)*1000);axs[0].set_ylabel('Base displacement (mm)')
            axs[1].plot(t,np.rad2deg((rots[0].inv()*rots).magnitude()));axs[1].set_ylabel('Base attitude drift (deg)')
            for j in range(3):axs[2].plot(t,np.rad2deg(a['base_omega_world_rad_s'][:,j]),ls=STYLES[j],label='XYZ'[j])
            axs[2].set_ylabel('Base angular rate (deg/s)');axs[2].legend(ncol=3)
        for ax in axs:
            ax.grid(alpha=.15)
            if key!='trajectory':
                if latch is not None:ax.axvline(latch,color='black',ls=':',lw=.8)
                if m['status']!='COMPLETED':ax.axvline(t[-1],color=COLORS[1],ls=':',lw=.8)
        if key!='trajectory':axs[-1].set_xlabel('Absolute physical time (s)')
        for ext in ['pdf','png']:fig.savefig(dest/(key+'.'+ext),bbox_inches='tight')
        plt.close(fig)
        window_status='EVALUATED' if m['full_window_evaluated'] else 'NOT_EVALUATED (full post-capture horizon absent)'
        context=f" Run {name}; {m['status']}; actual interval {t[0]:.3f}–{t[-1]:.3f} s; latch {'none' if latch is None else f'{latch:.3f} s'}; final window {window_status}. All runs are seen development/regression."
        manifest[key]={'title':title,'caption':caption+context,'trace_sha256':digest,'run':name,
            'png_sha256':sha(dest/(key+'.png')),'pdf_sha256':sha(dest/(key+'.pdf')),'source_fields_time_range_s':[float(t[0]),float(t[-1])]}
    save(dest/'figure_manifest.json',manifest);save(dest/'tracking_consistency.json',checks)
    print('N209 figures',name,len(manifest),flush=True)
    return manifest

def page(title,body):
    return '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+html.escape(title)+'</title><style>body{max-width:1240px;margin:32px auto;padding:0 24px;font:16px/1.65 system-ui;color:#173044;background:#f6f8fa}a{color:#0969aa}h1{font-size:30px}h2{font-size:23px}section,.card{background:white;padding:24px;margin:20px 0;border:1px solid #d7e1e8;border-radius:12px}img,video{max-width:100%;height:auto;background:#fff}video{width:100%}table{border-collapse:collapse;width:100%}td,th{padding:8px;border-bottom:1px solid #d7e1e8;text-align:left}.note{border-left:4px solid #c98226;padding:12px 18px;background:#fff4df}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(270px,1fr));gap:16px}.small{font-size:13px;color:#546c7c}nav{position:sticky;top:0;padding:12px;background:#f6f8faf2;border-bottom:1px solid #ccd6dd}summary{cursor:pointer;font-weight:600}figure{margin:16px 0}figcaption{font-size:14px;color:#4a6274}</style>'+body+'</html>'

def build_gallery(selected):
    names=[x['name'] for x in read(ROOT/'run_ledger.json')['attempts']];rows=[]
    for name in names:
        m=read(ROOT/'runs'/name/'metrics.json');figs=read(OUT/'runs'/name/'figure_manifest.json')
        window_status='EVALUATED' if m['full_window_evaluated'] else 'NOT_EVALUATED（缺少完整抓后时域）'
        body=f'<p><a href="../../index.html">← 全部 N209 结果</a></p><h1>{name}</h1><p class="note">{m["status"]} · 实际结束 {m["end_time_s"]:.3f} s · 抓后完整窗：{window_status}。早停数据在实际停止处结束。</p>'
        body+='<nav>'+ ' · '.join(f'<a href="#{k}">{v["title"]}</a>' for k,v in figs.items())+'</nav>'
        for key,item in figs.items():body+=f'<section id="{key}"><h2>{item["title"]}</h2><a href="{key}.pdf">矢量 PDF</a> · <a href="{key}.png">PNG</a><figure><img loading="lazy" src="{key}.png" alt="{item["title"]}"><figcaption>{html.escape(item["caption"])}</figcaption></figure></section>'
        (OUT/'runs'/name/'index.html').write_text(page(name,body),encoding='utf-8')
        latch_text='—' if m['latch_time_s'] is None else f"{m['latch_time_s']:.3f}"
        rows.append(f'<tr><td><a href="runs/{name}/index.html">{name}</a></td><td>{m["status"]}</td><td>{m["end_time_s"]:.3f}</td><td>{latch_text}</td><td>{"完整" if m["full_window_evaluated"] else "NOT_EVALUATED"}</td></tr>')
    for key in CAPTIONS:
        for ext in ['pdf','png']:shutil.copyfile(OUT/'runs'/selected/(key+'.'+ext),OUT/(key+'.'+ext))
        wrapper=f"from pathlib import Path\nimport sys\nsys.path.insert(0,str(Path(__file__).resolve().parents[4]))\nfrom v6_mujoco.feasible_capture.visualization import plot_run\nif __name__ == '__main__':\n    plot_run(sys.argv[1] if len(sys.argv)>1 else {selected!r}, {key!r})\n"
        (OUT/('gen_'+key+'.py')).write_text(wrapper,encoding='utf-8')
    videos=read(OUT/'video_manifest.json');body='<h1>N209 · 连续捕获与不确定性证据</h1>'
    body+='<p class="note"><strong>PARTIAL_OPERATING_DOMAIN_MISMATCH</strong>：8 条已见尝试，3 条完成、5 条早停。当前视频展示选定 V2 名义回归；H2/S01 未通过，独立盲测、消融和细步长未准入。视频不是新的性能验证。</p>'
    body+='<p><a href="../../../../paper/N209_FEASIBILITY_AND_UNCERTAINTY_REPORT.md">完整报告</a> · <a href="../../../../paper/system_paper/manuscript_v1.md">论文 V1</a> · <a href="../qualification_matrix.json">资格矩阵</a> · <a href="refresh_manifest.json">本次刷新清单</a></p>'
    body+='<nav><a href="#videos">同步视频</a> · <a href="#runs">全部运行</a> · <a href="#figures">论文主图</a> · <a href="runs/'+selected+'/index.html">完整名义诊断图</a></nav>'
    body+=f'<section id="videos"><h2>同步视频 · {selected}</h2><p>从 t=0 到实际结束，30 fps 原速展示；所有画面来自已通过双重重放的同一 trace。红色为实际末端轨迹，蓝色为有效参考。体侧相机连续跟随基座姿态。</p>'
    body+='<p>合成总览为 2880 × 1200；请全屏或按原始尺寸查看每个面板的数值，亦可打开下方独立镜头。</p>'
    for key in ['overview_five_views_and_body_side','body_side','interface_closeup']:
        item=videos['videos'][key];body+=f'<h3>{item["title"]}</h3><video controls preload="metadata" poster="{key}_last.png" src="{key}.mp4"></video><p><a href="{key}.mp4">下载 MP4</a></p>'
    body+='<details><summary>五个独立固定视角</summary><div class="grid">'
    for key in ['isometric','front','right','top','rear']:
        item=videos['videos'][key];body+=f'<div><h3>{item["title"]}</h3><video controls preload="none" poster="{key}_last.png" src="{key}.mp4"></video><a href="{key}.mp4">下载</a></div>'
    body+='</div></details></section><section id="runs"><h2>全部 8 条记录</h2><table><tr><th>运行 / 图库</th><th>结果</th><th>结束 (s)</th><th>锁紧 (s)</th><th>末窗</th></tr>'+''.join(rows)+'</table></section>'
    body+='<section id="figures"><h2>论文五组主图</h2>'
    for p in sorted((ROOT/'figures').glob('f[1-5]_*.png')):body+=f'<figure><a href="../figures/{p.stem}.pdf">矢量 PDF</a><img loading="lazy" src="../figures/{p.name}" alt="{p.stem}"></figure>'
    body+='</section><p class="small">仅仿真；不声明硬件或实时就绪。N208 及更早结果保留为历史资料。</p>'
    (OUT/'index.html').write_text(page('N209 可视化',body),encoding='utf-8')
    (OUT/'README.md').write_text(f'# N209 current visualizations\n\nSelected video run: {selected}. Open index.html locally after Git LFS hydration.\n\nEight synchronized videos, 11 diagnostic figure types for each of all eight attempts, and five paper figure groups. Every video uses the same saved continuous trace; rendering advances no simulation.\n\nRebuild: `python -m v6_mujoco.feasible_capture --visualize`. Failed attempts remain stopped and do not receive a fabricated post-capture window.\n',encoding='utf-8')

def run(selected=SELECTED, resume=False):
    start=time.time();cpu=time.process_time();OUT.mkdir(parents=True,exist_ok=True)
    before={str(p.relative_to(PROJECT_ROOT)):sha(p) for p in [ROOT/'run_ledger.json',PAPER/'manuscript_v1.md']}
    for script in sorted((ROOT/'figures').glob('gen_f*.py')):subprocess.run([sys.executable,str(script)],check=True)
    for attempt in read(ROOT/'run_ledger.json')['attempts']:
        plot_run(attempt['name'])
    from .render import run as render
    if resume and (OUT/'video_manifest.json').exists():
        previous=read(OUT/'video_manifest.json')
        assert previous['run']==selected, 'Existing video uses another run; rebuild without --resume'
        assert previous['trace_sha256']==sha(ROOT/'runs'/selected/'trace.npz'), 'Video trace changed; rebuild without --resume'
        assert all(sha(PROJECT_ROOT/p)==digest for p,digest in previous['source_identity'].items()), 'Renderer changed; rebuild without --resume'
        assert all(sha(OUT/(key+'.mp4'))==item['sha256'] for key,item in previous['videos'].items()), 'Video bytes changed; rebuild without --resume'
    else:render(selected)
    assert read(OUT/'video_manifest.json')['run']==selected
    build_gallery(selected)
    assert all(sha(PROJECT_ROOT/p)==digest for p,digest in before.items())
    artifacts={p.relative_to(OUT).as_posix():sha(p) for p in OUT.rglob('*') if p.is_file() and p.suffix in ['.mp4','.png','.pdf','.html']}
    save(OUT/'refresh_manifest.json',{'selected_video_run':selected,'all_run_names':[x['name'] for x in read(ROOT/'run_ledger.json')['attempts']],
        'figure_types':list(CAPTIONS),'run_figure_count':len(CAPTIONS)*len(read(ROOT/'run_ledger.json')['attempts']),
        'video_count':len(read(OUT/'video_manifest.json')['videos']),'new_physical_attempts':0,'physics_steps':0,
        'preserved_inputs':before,'artifacts':artifacts,'source_identity':{p:sha(PROJECT_ROOT/p) for p in ['v6_mujoco/feasible_capture/visualization.py','v6_mujoco/feasible_capture/render.py']},
        'wall_s':time.time()-start,'orchestrator_process_cpu_s':time.process_time()-cpu,'resource_scope':'Additional display refresh; not a new experiment or a reopened experiment ledger. Video encoding CPU is separate in video_manifest.json.'})
    print(json.dumps({'visualization_refresh':'complete','figures':len(CAPTIONS)*8,'videos':8,'selected':selected}))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--name',default=SELECTED);p.add_argument('--figure',choices=list(CAPTIONS));p.add_argument('--resume',action='store_true');args=p.parse_args()
    plot_run(args.name,args.figure) if args.figure else run(args.name,args.resume)
