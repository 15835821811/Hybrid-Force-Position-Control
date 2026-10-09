"""Check evidence completeness without turning failed trials into successes."""
import argparse
import html
from pathlib import Path
from .common import ROOT,read,save,digest,init_ledger

def deliver(selected):
    out=ROOT/'runs'/selected;metrics=read(out/'metrics.json');q=read(ROOT/'qualification_matrix.json');ledger=init_ledger();viz=ROOT/'visualizations';figures=read(viz/'figure_manifest.json');videos=read(viz/'manifest.json');trace=digest(out/'trace.npz')
    checks={};checks['no_running_attempts']=not any(x['status']=='RUNNING' for x in ledger['attempts']);checks['attempt_budget']=len(ledger['attempts'])<=16 and sum(x['category']=='development' for x in ledger['attempts'])<=6;checks['cpu_budget']=read(ROOT/'resource_accounting.json')['budget_charge_s']<=28800
    required=['requirements_and_parameter_policy.md','literature_to_implementation.md','identifiability_report.json','comparison.json','qualification_matrix.json','truth_access_audit.json','run_ledger.json','commands.md','tests.log','momentum_feasibility.json']
    checks['required_artifacts']=all((ROOT/f).exists() for f in required);checks['selected_trace_verified']=read(out/'validation.json')['actuator_replay_passed'] and read(out/'validation.json')['decision_replay_passed']
    checks['figure_provenance']=all(x['trace_sha256']==trace and x['run']==selected and digest(viz/(k+'.png'))==x['png_sha256'] and digest(viz/(k+'.pdf'))==x['pdf_sha256'] for k,x in figures.items())
    from .figures import CAPTIONS
    checks['figure_set_complete']=set(figures)==set(CAPTIONS)
    checks['visual_review_recorded']='Final rendered review completed' in (ROOT/'visual_quality_review.md').read_text(encoding='utf-8')
    checks['comparison_figure_sources']=all(digest(ROOT/'runs'/run/'trace.npz')==sha for item in figures.values() for run,sha in item.get('source_run_trace_sha256',{}).items())
    checks['video_provenance']=videos['trace_sha256']==trace and len(videos['videos'])==2 and all(digest(viz/(k+'.mp4'))==v['sha256'] for k,v in videos['videos'].items())
    checks['hardware_claims_remain_false']=all(q[k] is False for k in ['hardware_load_rating_validated','physical_gripper_capture_validated','real_time_ready'])
    statuses=[]
    for attempt in ledger['attempts']:
        p=ROOT/'runs'/attempt['name'];hastrace=(p/'trace.npz').exists();valid=read(p/'validation.json') if (p/'validation.json').exists() else {}
        statuses.append({'name':attempt['name'],'status':attempt['status'],'trace_exists':hastrace,'actuator_replay_passed':valid.get('actuator_replay_passed'),'decision_replay_passed':valid.get('decision_replay_passed')})
    checks['attempts_preserved']=all((ROOT/'runs'/x['name']/'config.json').exists() and (ROOT/'runs'/x['name']/'metrics.json').exists() for x in ledger['attempts'])
    checks['all_available_traces_replayed']=all(not x['trace_exists'] or (x['actuator_replay_passed'] and x['decision_replay_passed']) for x in statuses)
    planned={'holdout':6,'fine':1,'pressure':2};unexecuted={k:n-sum(x['category']==k for x in ledger['attempts']) for k,n in planned.items()}
    if ledger['frozen']:
        from .campaign import verify_freeze
        checks['frozen_execution_unchanged']=verify_freeze();checks['independent_campaign_executed']=all(n==0 for n in unexecuted.values())
    else:checks['unfrozen_validation_not_claimed']=not any(x['category']!='development' for x in ledger['attempts'])
    report={'delivery_complete':all(checks.values()),'checks':checks,'selected':selected,'attempts':statuses,'unexecuted_planned_categories':unexecuted,'qualification':q,'visual_review':'See visual_quality_review.md; automated hashes/dimensions do not replace rendered inspection.','physical_success_not_required_for_honest_delivery':True,'remote_push_verified':False}
    save(ROOT/'completion_audit.json',report)
    title='N208 actual simulation evidence — '+selected
    body=['<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>'+html.escape(title)+'</title><style>body{max-width:1100px;margin:32px auto;padding:0 20px;font:16px/1.6 system-ui;background:#fafafa;color:#182431}img,video{max-width:100%;background:white}section{padding:20px 0;border-bottom:1px solid #ccd}a{color:#125ea6}small{color:#52606d}</style>',f'<h1>{html.escape(title)}</h1>',f'<p>Outcome: {metrics["status"]}. Continuous task: {metrics["continuous_task_completed"]}; fixed-window detumbling: {metrics["postgrasp_detumbling"]}. Natural-task full inertia identification: unvalidated.</p>','<p>Simulation sensors, passive target, equivalent software latch. No hardware load, physical gripper, real-vision or real-time claim. Historical N206 results remain separate.</p>','<p><a href="../../../../paper/N208_PASSIVE_TARGET_ESTIMATION_CAPTURE_REPORT.md">Report</a> · <a href="../comparison.json">All attempts</a> · <a href="../qualification_matrix.json">Qualification</a></p>']
    for key in videos['videos']:body.append(f'<section><h2>{html.escape(key.replace("_"," "))}</h2><video controls preload="metadata" src="{key}.mp4"></video></section>')
    for key,item in figures.items():body.append(f'<section><h2>{html.escape(key.replace("_"," "))}</h2><a href="{key}.pdf">Vector PDF</a><p>{html.escape(item["caption"])}</p><img loading="lazy" src="{key}.png" alt="{html.escape(key)}"></section>')
    (viz/'index.html').write_text('\n'.join(body),encoding='utf-8')
    repo=ROOT.parents[2];readme=repo/'README.md';text=readme.read_text(encoding='utf-8');start='<!-- N208 CURRENT START -->';end='<!-- N208 CURRENT END -->'
    if start in text:
        left,right=text.split(start,1);_,right=right.split(end,1);text=left+right.lstrip('\n')
    text=text.replace('## 当前分支 N206：','## 历史 N206：',1)
    head,tail=text.split('\n',1)
    block=f'''{start}
## 当前分支 N208：被动目标估计与事件捕获

选中运行 `{selected}`：连续任务 **{metrics['continuous_task_completed']}**，固定末窗消旋 **{metrics['postgrasp_detumbling']}**。共记录 {len(ledger['attempts'])} 次完整机器人尝试，所有早停保留。自然任务全惯性辨识尚未验证；任务完成与参数识别分别判定。

- [本轮报告](paper/N208_PASSIVE_TARGET_ESTIMATION_CAPTURE_REPORT.md) · [全部运行比较](output/fpmfc/n208_adaptive_capture/comparison.json) · [资格矩阵](output/fpmfc/n208_adaptive_capture/qualification_matrix.json)
- [本轮全部科学图与两条视频](output/fpmfc/n208_adaptive_capture/visualizations/index.html)
- [五个固定视角与连续体侧合成视频](output/fpmfc/n208_adaptive_capture/visualizations/overview_five_views_and_body_side.mp4) · [接口近景](output/fpmfc/n208_adaptive_capture/visualizations/interface_closeup.mp4)
- [末端实际/参考轨迹](output/fpmfc/n208_adaptive_capture/visualizations/trajectory.png) · [末端跟踪误差](output/fpmfc/n208_adaptive_capture/visualizations/tracking.png)

仿真传感器、软件锁紧与有限场景证据；无真实夹爪、硬件承载或实时就绪声明。下列 N110—N206 内容保留为历史记录。
{end}
'''
    readme.write_text(head+'\n\n'+block+tail,encoding='utf-8');return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('selected');a=p.parse_args();r=deliver(a.selected);print({'delivery_complete':r['delivery_complete'],'checks':r['checks']})
