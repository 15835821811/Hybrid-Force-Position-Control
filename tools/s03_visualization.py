"""S03 recorded-data plots and multi-view videos, no new physical integration."""
import argparse
import html
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from v6_mujoco.system_capture.planning.contracts import OUT as ROOT,PROJECT,read,save,sha,charge
from v6_mujoco.system_capture.planning.report import folder_for
from s02_visualization import plot_run,page,style
OUT=ROOT/'visualizations'

def load_run(name):
    folder=folder_for(name);m=read(folder/'metrics.json');cfg=read(folder/'config.json')
    for k in ['actuator','decision']:
        assert read(ROOT/'runs'/name/(k+'_replay.json'))['passed'], name+' '+k+' replay'
    with np.load(folder/'trace.npz',allow_pickle=False) as z:a={k:z[k] for k in z.files}
    return folder,a,m,cfg,sha(folder/'trace.npz')

def authority():
    a=read(ROOT/'candidate_authority.json');style();dest=OUT/'figures';dest.mkdir(parents=True,exist_ok=True)
    labels=['Time','Normal','Tangent','Shape midpoint','Shape endpoint'];values=list(a['effects'].values())
    fig,axs=plt.subplots(1,2,figsize=(7.3,3.2),layout='constrained')
    for ax,k,scale,y in zip(axs,['max_action_delta_rad_s','max_margin_delta_m'],[1,1000],['Max joint-speed change (rad/s)','Max named-margin change (mm)']):
        ax.bar(np.arange(5),[v[k]*scale for v in values],color=['#0072B2','#D55E00','#009E73','#CC79A7','#E69F00']);ax.set_xticks(range(5),labels,rotation=30,ha='right');ax.set_ylabel(y)
    for ext in ['png','pdf']:fig.savefig(dest/('candidate_authority.'+ext),bbox_inches='tight')
    plt.close(fig)
    return dict(caption='Maximum absolute paired change over four preregistered states with 0.6 s independent-prior forecasts. Each maximum may belong to a different state, pair and candidate; they are not simultaneous improvements. See the signed-margin table and figure. This demonstrates local authority, not continuous-task success. Shape alters joint motion without altering the commanded end-effector pose.',source_sha256=sha(ROOT/'candidate_authority.json'))

def planning_plot(name):
    f,a,m,cfg,digest=load_run(name);g=read(f/'progress_governor.json');dest=OUT/'runs'/name
    if not any('selected' in r for r in g):return
    style();fig,axs=plt.subplots(5,1,figsize=(7.3,9.0),sharex=True,layout='constrained');times=[];theta=[];valid=[]
    for row in g:
        if row.get('selected') is None:continue
        cand=next(c for c in row['candidates'] if c['index']==row['selected']);times.append(row['time']);theta.append(cand['theta']);valid.append(row['valid_until'])
    if times:
        t=np.array(times);v=np.array(theta);arrivals=t+v[:,0]
        # The final accepted plan remains selected through the executed prefix.
        # Its certificate endpoint is held between discrete replans, not ramped.
        if t[-1]<m['end_time_s']:
            t=np.r_[t,m['end_time_s']];v=np.vstack([v,v[-1]]);arrivals=np.r_[arrivals,arrivals[-1]];valid=[*valid,valid[-1]]
        axs[0].step(t,arrivals,where='post',label='Selected arrival');axs[0].step(t,valid,where='post',ls=':',label='Verified until');axs[0].set_ylabel('Absolute time (s)');axs[0].legend()
        axs[1].step(t,v[:,1]*1000,where='post',label='Normal');axs[1].step(t,v[:,2]*1000,where='post',ls='--',label='Fixed tangent');axs[1].set_ylabel('Registered path theta (mm)');axs[1].legend()
        axs[2].step(t,v[:,3],where='post',label='Midpoint bump');axs[2].step(t,v[:,4],where='post',ls='--',label='Endpoint correction');axs[2].set_ylabel('Registered shape theta (rad)');axs[2].legend()
    shape=read(ROOT/'diagnostics'/('shape_'+name+'.json'))['samples']
    for k,ls,label in [('actual_rad','-','Actual'),('reference_rad','--','Task reference')]:axs[3].plot([r['time_s'] for r in shape],[r[k] for r in shape],ls,label=label)
    axs[3].set_ylabel('Unwrapped arm shape (rad)');axs[3].legend()
    active=a['approach_reference_applicable'].astype(bool);axs[4].plot(a['time_s'][active],a['approach_shape_error_rad'][active]);axs[4].set(ylabel='Absolute shape error (rad)',xlabel='Physical time (s)')
    for ax in axs:ax.grid(alpha=.15)
    fig.suptitle('Theta panels: candidate metadata, not the complete executed reference.\nContinuation preserves saved segment coefficients even when metadata is zero.',fontsize=8)
    for ext in ['png','pdf']:fig.savefig(dest/('local_planning.'+ext),bbox_inches='tight')
    plt.close(fig)

def index(names):
    q=read(ROOT/'qualification.json');rows=[]
    for name in names:
        f=folder_for(name);m=read(f/'metrics.json');dest=OUT/'runs'/name;parts=[];manifest=read(dest/'figure_manifest.json')
        if (dest/'video_manifest.json').exists():
            for key,v in read(dest/'video_manifest.json')['videos'].items():parts.append(f'<section><h2>{html.escape(v["title"])}</h2><video controls preload="metadata" poster="{key}_last.png" src="{key}.mp4"></video></section>')
        if (dest/'local_planning.png').exists():parts.append('<section><h2>局部时间、路径与臂形</h2><img src="local_planning.png"><p>记录的到达时间、有效期及候选 theta 元数据；实际臂形与跟踪误差单列。幅度面板不是完整执行参考：index 0 继续旧段时以保存的 segment 系数为准，登记零值不表示实际偏移或臂形参考瞬间归零。</p></section>')
        for key,v in manifest.items():parts.append(f'<section><h2>{html.escape(v["title"])}</h2><img loading="lazy" src="{key}.png"><p>{html.escape(v["caption"])}</p><a href="{key}.pdf">矢量PDF</a></section>')
        body=f'<a href="../../index.html">← S03 总览</a><h1>{name} · {m["status"]}</h1><p class="note">实际时域0–{m["end_time_s"]:.3f}s；完整抓后窗口：{m["final_window_status"]}。原始失败保留；显示不是新物理尝试。</p>'+''.join(parts)
        (dest/'index.html').write_text(page('S03 '+name,body),encoding='utf-8')
        (dest/'README.md').write_text(f'# S03 {name}\n\n状态：{m["status"]}；实际终止 {m["end_time_s"]:.3f}s。\n\n[全部图表和视频](index.html) · [五视角与体侧总览](overview_five_views_and_body_side.mp4) · [连续体侧](body_side.mp4) · [末端跟踪误差](tracking.png)\n',encoding='utf-8')
        rows.append(f'<tr><td><a href="runs/{name}/index.html">{name}</a></td><td>{m["status"]}</td><td>{m["end_time_s"]:.3f}</td><td>{m["postgrasp_detumbling"]}</td></tr>')
    body='<h1>S03 · 时间—路径—臂形规划</h1><p class="note">理想 SensorPacket + 冻结 CA18。非理想测量 NOT_ADMITTED；非实时仿真。失败前缀不代表完整任务收益。</p>'
    body+=f'<p>B2名义：{q["nominal_continuous_task"]}；H2：{q["H2_ideal_task"]}；双重放：{q["dual_replay"]}。</p><table><tr><th>运行</th><th>状态</th><th>实际终止(s)</th><th>完整消旋合格</th></tr>'+''.join(rows)+'</table>'
    body+='<section><h2>候选实际控制权限</h2><img src="figures/candidate_authority.png"><p>同状态候选改变关节动作与命名裕量，不能据此认定完整捕获成功。</p></section><p><a href="../report.md">科学报告</a> · <a href="../same_model_comparison.json">完整/共同前缀对照</a> · <a href="../../S02/visualizations/index.html">S02归档可视化</a></p>'
    if (OUT/'figures/scientific_manifest.json').exists():
        for key,v in read(OUT/'figures/scientific_manifest.json').items():body+=f'<section><h2>{html.escape(v["title"])}</h2><img src="figures/{key}.png"><p>{html.escape(v["caption"])}</p><a href="figures/{key}.pdf">PDF</a></section>'
    (OUT/'index.html').write_text(page('S03 可视化',body),encoding='utf-8')
    (OUT/'README.md').write_text('# S03 可视化\n\n[统一图表与视频入口](index.html)\n\n'+ '\n'.join(f'- [{n}](runs/{n}/README.md)' for n in names)+'\n\n只显示保存的已执行轨迹，未捕获运行不延长成完整任务。\n',encoding='utf-8')

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--plots',action='store_true');parser.add_argument('--videos',action='store_true');parser.add_argument('--name');a=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True);names=[a.name] if a.name else ['B00']+[e['name'] for e in read(ROOT/'run_ledger.json')['attempts']]
    cpu,wall=time.process_time(),time.perf_counter();child_cpu=0.;prerendered=[];rendered_new=[]
    if a.plots:
        for n in names:plot_run(n,loader=load_run,output=OUT,stage='S03');planning_plot(n)
        save(OUT/'figures/authority_caption.json',authority())
        from s03_scientific_figures import main as science_figures
        science_figures()
    if a.videos:
        from s02_render import run
        for n in names:
            path=OUT/'runs'/n/'video_manifest.json'
            if path.exists():
                v=read(path)
                assert v['trace_sha256']==load_run(n)[-1]
                for k,r in v['videos'].items():assert sha(path.parent/(k+'.mp4'))==r['sha256']
                prerendered.append((n,v))
            else:
                run(n,root=ROOT,loader=load_run,stage='S03');child_cpu+=read(path)['encoder_process_cpu_s'];rendered_new.append(n)
    index(names);charge('visualizations',cpu,wall,run_names=names,rendered_new_names=rendered_new,encoder_cpu_s=child_cpu,new_physics_steps=0)
    if child_cpu:
        b=read(ROOT/'run_ledger.json');b['operations'].append(dict(kind='video_encoder_children',cpu_s=child_cpu));save(ROOT/'run_ledger.json',b)
    if prerendered:
        b=read(ROOT/'run_ledger.json')
        for n,v in prerendered:
            if not any((x.get('kind')=='prerendered_media' and x.get('run')==n) or n in x.get('rendered_new_names',[]) for x in b['operations']):
                b['operations'].append(dict(kind='prerendered_media',run=n,cpu_s=v['parent_process_cpu_s']+v['encoder_process_cpu_s'],wall_s=v['wall_s']))
        save(ROOT/'run_ledger.json',b)

if __name__=='__main__':main()
