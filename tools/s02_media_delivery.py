"""Build current S02 navigation and audit display derivatives without changing evidence."""
import html
import json
import re
import shutil
import subprocess
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlsplit
sys.path.insert(0,str(Path(__file__).resolve().parent))
from s02_visualization import ROOT,OUT,PROJECT_ROOT,SELECTED,CAPTIONS,read,save,sha,page

BRANCH='codex/system-s02-estimation-capture-contract'
SNAPSHOT='7198178ad798624f6ea163244e03641b78a56a3c'
KEYS=['overview_five_views_and_body_side','body_side','interface_closeup','isometric','front','right','top','rear']
TITLES=['五视角与连续体侧合成总览','连续基座体侧','接口近景','等轴','正面','右侧','俯视','后方']

def video_html(prefix):
    s=''
    for key,title in zip(KEYS,TITLES):
        s+=f'<details'+(' open' if key==KEYS[0] else '')+f'><summary>{title}</summary><video controls preload="none" poster="{prefix}{key}_last.png" src="{prefix}{key}.mp4"></video><a href="{prefix}{key}.mp4">下载 MP4</a></details>'
    return s

def build():
    names=[x['name'] for x in read(ROOT/'run_ledger.json')['attempts']]
    rows=[];mdrows=[]
    for name in names:
        d=OUT/'runs'/name;m=read(ROOT/'runs'/name/'metrics.json');figs=read(d/'figure_manifest.json')
        window='已评价' if m['full_window_evaluated'] else 'NOT_EVALUATED'
        note=f'{name}：{m["status"]}；实际结束 {m["end_time_s"]:.3f} s；抓后末窗 {window}。'
        body=f'<p><a href="../../index.html">返回 S02 全部结果</a></p><h1>{name}</h1><p class="note">{note}</p><h2>同一记录的同步视频</h2>'+video_html('')
        md=f'# {name}\n\n[返回全部结果](../../README.md) · [本地交互图集](index.html)\n\n{note}\n\n'
        md+=' · '.join(f'[{title}]({key}.mp4)' for key,title in zip(KEYS,TITLES))+'\n\n'
        for key,item in figs.items():
            body+=f'<section id="{key}"><h2>{item["title"]}</h2><a href="{key}.pdf">矢量 PDF</a><figure><img loading="lazy" src="{key}.png" alt="{item["title"]}"><figcaption>{html.escape(item["caption"])}</figcaption></figure></section>'
            md+=f'## {item["title"]}\n\n[PDF]({key}.pdf)\n\n![{item["title"]}]({key}.png)\n\n{item["caption"]}\n\n'
        (d/'index.html').write_text(page('S02 '+name,body),encoding='utf-8');(d/'README.md').write_text(md.rstrip()+'\n',encoding='utf-8')
        rows.append(f'<tr><td><a href="runs/{name}/index.html">{name}</a></td><td>{m["status"]}</td><td>{m["end_time_s"]:.3f}</td><td>{window}</td></tr>')
        mdrows.append(f'| [{name}](runs/{name}/README.md) | {m["status"]} | {m["end_time_s"]:.3f} | {window} |')
    intro='S02：ACCURACY_IMPROVED_GUARD_NOT_COMPATIBLE。E0_C2 完成完整捕获与抓后消旋；原 E0 实现失败、E1/E2 噪声条件控制拒绝均保留，E3 未运行。估计 RMS 改善不等于噪声捕获合格，启动峰值反而变大。'
    details='全部媒体读取保存的实际轨迹，显示过程不推进动力学。五个固定世界镜头、连续随基座姿态旋转的体侧镜头及接口近景采用相同时间索引；30 fps 原速，末帧为实际停止状态，视频封装时长最多增加两个帧周期。体侧是相机定义，不是柔性连续体机器人。合成分辨率 2880×1200，独立镜头 960×600。'
    body=f'<h1>S02 · 全部可视化</h1><p class="note">{intro}</p><p>{details}</p><p><a href="../report.md">实验报告</a> · <a href="../qualification.json">资格</a> · <a href="refresh_manifest.json">媒体来源</a> · <a href="visualization_audit.json">媒体核验</a></p>'
    body+='<h2>完整名义任务 E0_C2</h2><p>锁紧 7.914 s，真实运行至 27.914 s；此视频不代表 E1/E2 成功。</p>'+video_html('runs/E0_C2/')
    body+='<h2>四条实际运行</h2><table><tr><th>运行 / 图集</th><th>结果</th><th>结束 (s)</th><th>末窗</th></tr>'+''.join(rows)+'</table><p>E3：NOT_RUN，没有伪造轨迹或视频。</p>'
    body+='<h2>末端轨迹与跟踪误差</h2>'
    for key in ['trajectory','tracking']:
        body+=f'<section><h3>{CAPTIONS[key][0]}</h3><img src="runs/E0_C2/{key}.png" alt="{key}"><p><a href="runs/E0_C2/{key}.pdf">PDF</a> · <a href="runs/E0_C2/README.md">全部11类诊断图与图注</a></p></section>'
    body+='<h2>四组科学主图</h2><p><a href="core_figures/README.md">完整定义与图注</a></p>'
    for p in sorted((OUT/'core_figures').glob('f[1-4]_*.png')):body+=f'<figure><a href="core_figures/{p.stem}.pdf">PDF</a><img loading="lazy" src="core_figures/{p.name}" alt="{p.stem}"></figure>'
    body+='<h2>历史资料</h2><p><a href="../../S01/visualizations/README.md">S01 SRS</a> · <a href="../../../n209_paper_system/visualizations/README.md">历史 N209 完整图集</a></p>'
    (OUT/'index.html').write_text(page('S02 全部可视化',body),encoding='utf-8')
    md=f'# S02 当前版本完整可视化\n\n{intro}\n\n[本地交互图集](index.html) · [实验报告](../report.md) · [媒体清单](refresh_manifest.json) · [核验](visualization_audit.json)\n\n{details}\n\n## 完整名义任务 E0_C2\n\n锁紧7.914 s，实际结束27.914 s。\n\n'
    md+='\n'.join(f'- [{title}](runs/E0_C2/{key}.mp4)' for key,title in zip(KEYS,TITLES))+'\n\n![五视角实际终帧](runs/E0_C2/overview_five_views_and_body_side_last.png)\n\n'
    md+='## 全部运行：44组诊断图与32个视频\n\n|运行|结果|实际结束(s)|抓后末窗|\n|---|---|---|---|\n'+'\n'.join(mdrows)+'\n\nE3未运行。每条运行11类图均有PNG与矢量PDF；失败记录只显示实际前缀。\n\n'
    md+='## 末端轨迹与跟踪误差\n\n[实际/参考轨迹](runs/E0_C2/trajectory.png) · [轨迹PDF](runs/E0_C2/trajectory.pdf) · [误差PDF](runs/E0_C2/tracking.pdf)\n\n![末端跟踪误差](runs/E0_C2/tracking.png)\n\n位置误差为实际法兰减有效法兰参考，姿态误差采用SO(3)；参考在锁紧后停用，抓后接口保持量在capture_and_load中单独显示。\n\n'
    md+='## 四组科学主图\n\n[全部科学主图及严格图注](core_figures/README.md)\n\n'
    md+=f'## 复现与来源\n\n```powershell\npython tools/s02_refresh_media.py --plots\npython tools/s02_refresh_media.py --videos\npython tools/s02_media_delivery.py\n```\n\n发布分支 `{BRANCH}`。本次按用户新增授权刷新媒体并上传 GitHub；原实验验收快照为 `{SNAPSHOT}`，其报告、handoff、原始数据及资源账本保持原样。原文“未推送”描述验收快照产生时的状态。此显示工作单独记账，不是新物理实验。\n\n克隆后执行 `git lfs pull` 获取轨迹和视频，然后在本地打开 index.html。GitHub Markdown 可预览PNG，MP4通过文件页下载。\n\n[显示复核](visual_quality_review.md) · [历史S01](../../S01/visualizations/README.md) · [历史N209](../../../n209_paper_system/visualizations/README.md)\n'
    (OUT/'README.md').write_text(md,encoding='utf-8')
    for key in CAPTIONS:
        wrapper=f"from pathlib import Path\nimport sys\nsys.path.insert(0,str(Path(__file__).resolve().parents[5]/'tools'))\nfrom s02_visualization import plot_run\nif __name__=='__main__':plot_run(sys.argv[1] if len(sys.argv)>1 else 'E0_C2', {key!r})\n"
        (OUT/('gen_'+key+'.py')).write_text(wrapper,encoding='utf-8')
    (OUT/'.gitattributes').write_text('*.mp4 filter=lfs diff=lfs merge=lfs -text\n',encoding='utf-8')
    readme=PROJECT_ROOT/'README.md';text=readme.read_text(encoding='utf-8')
    text=re.sub(r'<!-- S02 CURRENT START -->.*?<!-- S02 CURRENT END -->\s*','',text,flags=re.S)
    text=text.replace('## 当前版本：S01 原文兼容 SRS 基准与完整可视化','## 历史 S01：原文兼容 SRS 基准与可视化').replace('未运行S02—S08。','该历史验收时未运行S02—S08。')
    prefix='output/fpmfc/system_capture/S02/visualizations'
    block=f'<!-- S02 CURRENT START -->\n## 当前版本：S02 估计器与捕获门兼容性\n\n{intro}\n\n分支：`{BRANCH}`。已刷新48组图（44组逐运行诊断＋4组科学主图）和32个同步视频。\n\n- [全部可视化]({prefix}/README.md) · [本地交互图集]({prefix}/index.html) · [核验与来源]({prefix}/refresh_manifest.json)\n- [五视角总览]({prefix}/runs/E0_C2/overview_five_views_and_body_side.mp4) · [连续体侧视频]({prefix}/runs/E0_C2/body_side.mp4) · [接口近景]({prefix}/runs/E0_C2/interface_closeup.mp4)\n- [末端轨迹]({prefix}/runs/E0_C2/trajectory.png) · [轨迹跟踪误差]({prefix}/runs/E0_C2/tracking.png) · [S02报告](output/fpmfc/system_capture/S02/report.md)\n\n视频来自E0_C2：7.914 s锁紧，实际运行至27.914 s；全部失败运行另有图表和视频，绝不补齐缺失时域。下方S01/N209为独立历史证据。\n\n![S02 五视角与体侧终帧]({prefix}/runs/E0_C2/overview_five_views_and_body_side_last.png)\n![S02 末端跟踪误差]({prefix}/runs/E0_C2/tracking.png)\n<!-- S02 CURRENT END -->\n\n'
    head,tail=text.split('\n',1);readme.write_text(head+'\n\n'+block+tail.lstrip(),encoding='utf-8')
    unified=ROOT.parent/'visualizations'
    (unified/'README.md').write_text('# 当前版本可视化入口\n\n[S02完整图集](../S02/visualizations/README.md) · [本地交互图集](../S02/visualizations/index.html)\n\n当前S02：48组图、32个视频，包含五视角、连续体侧、接口近景、末端轨迹与跟踪误差。\n\n历史资料：[S01图表](../S01/figures/README.md) · [S01视频](../S01/visualizations/README.md) · [N209图集](../../n209_paper_system/visualizations/README.md)。各自保留原实验来源。\n',encoding='utf-8')
    (unified/'index.html').write_text(page('当前版本可视化','<h1>当前版本：S02</h1><p><a href="../S02/visualizations/index.html">打开48组图与32个视频</a></p><p><a href="../S01/visualizations/README.md">历史S01</a> · <a href="../../n209_paper_system/visualizations/index.html">历史N209</a></p>'),encoding='utf-8')

def audit():
    errors=[];checked=0;probes={};tracking={}
    original=read(ROOT/'handoff.json')['evidence_sha256']
    for rel,digest in original.items():
        checked+=1
        if sha(PROJECT_ROOT/rel)!=digest:errors.append('Original evidence changed: '+rel)
    for name in [x['name'] for x in read(ROOT/'run_ledger.json')['attempts']]:
        d=OUT/'runs'/name;v=read(d/'video_manifest.json');figs=read(d/'figure_manifest.json');trace=sha(ROOT/'runs'/name/'trace.npz')
        for rel,digest in v['source_identity'].items():
            if sha(PROJECT_ROOT/rel)!=digest:errors.append('Stale renderer '+name)
        with np.load(ROOT/'runs'/name/'trace.npz') as a:
            active=a['approach_reference_applicable'].astype(bool)
            position=np.linalg.norm(a['flange_position_world_m'][active]-a['approach_reference_position_world_m'][active],axis=1)
            angle=Rotation.from_matrix(a['flange_rotation_world'][active]@np.transpose(a['approach_reference_rotation_world'][active],(0,2,1))).magnitude()
            tracking[name]={'position_max_difference_m':float(np.max(abs(position-a['approach_position_error_m'][active]))),
                            'rotation_max_difference_rad':float(np.max(abs(angle-a['approach_rotation_error_rad'][active])))}
            if tracking[name]['position_max_difference_m']>1e-10 or tracking[name]['rotation_max_difference_rad']>1e-8:errors.append('Tracking definition '+name)
            if v['indices'][0]!=0 or v['indices'][-1]!=len(a['time_s'])-1 or abs(v['physical_times_s'][-1]-a['time_s'][-1])>1e-10:errors.append('Video endpoint '+name)
        if v['trace_sha256']!=trace or set(figs)!=set(CAPTIONS):errors.append('Trace or figure set '+name)
        if v['body_camera_max_adjacent_forward_angle_rad']>.05 or v['body_camera_max_orthogonality_error']>1e-10:errors.append('Camera continuity '+name)
        if v['physics_steps'] or v['new_physical_attempts']:errors.append('Unexpected simulation '+name)
        for key,item in figs.items():
            for ext in ['png','pdf']:
                if sha(d/(key+'.'+ext))!=item[ext+'_sha256']:errors.append('Figure hash '+name+'/'+key)
            if item['trace_sha256']!=trace:errors.append('Figure provenance '+name+'/'+key)
        for key,item in v['videos'].items():
            p=d/(key+'.mp4')
            if sha(p)!=item['sha256']:errors.append('Video hash '+str(p))
            probe=json.loads(subprocess.check_output([shutil.which('ffprobe'),'-v','error','-threads','1','-count_frames','-select_streams','v:0','-show_entries','stream=nb_read_frames,width,height,codec_name,pix_fmt,avg_frame_rate:format=duration','-of','json',str(p)],creationflags=subprocess.CREATE_NO_WINDOW))
            stream=probe['streams'][0]
            if int(stream['nb_read_frames'])!=v['frames'] or stream['codec_name']!='h264':errors.append('Decode '+str(p))
            if abs(float(probe['format']['duration'])-v['physical_end_time_s'])>2/v['fps']+.001:errors.append('Duration '+str(p))
            probes[name+'/'+key]=probe
        print('Media audit',name,flush=True)
    class Links(HTMLParser):
        def handle_starttag(self,tag,attrs):
            for k,v in attrs:
                if k in ['href','src','poster'] and v and not urlsplit(v).scheme:
                    target=(self.base/unquote(v.split('#')[0])).resolve()
                    expected=[OUT/'refresh_manifest.json',OUT/'visualization_audit.json']
                    if not target.exists() and target not in expected:errors.append('Broken link '+str(self.base)+' '+v)
    for p in list(OUT.rglob('*.html'))+[ROOT.parent/'visualizations/index.html']:
        parser=Links();parser.base=p.parent;parser.feed(p.read_text(encoding='utf-8'))
    for p in list(OUT.rglob('*.md'))+[ROOT.parent/'visualizations/README.md']:
        for ref in re.findall(r'\[[^\]]*\]\(([^)]+)\)',p.read_text(encoding='utf-8')):
            if urlsplit(ref).scheme:continue
            target=(p.parent/unquote(ref.split('#')[0])).resolve()
            if not target.exists() and target not in [OUT/'refresh_manifest.json',OUT/'visualization_audit.json']:errors.append('Broken Markdown link '+str(p)+' '+ref)
    sources={p.relative_to(PROJECT_ROOT).as_posix():sha(p) for p in (PROJECT_ROOT/'tools').glob('s02_*media*.py')}
    sources.update({f'tools/{name}':sha(PROJECT_ROOT/'tools'/name) for name in ['s02_visualization.py','s02_render.py']})
    artifacts={p.relative_to(OUT).as_posix():sha(p) for p in OUT.rglob('*') if p.is_file() and p.name not in ['refresh_manifest.json','visualization_audit.json'] and '__pycache__' not in p.parts}
    save(OUT/'visualization_audit.json',{'passed':not errors,'errors':errors,'original_evidence_hash_checks':checked,'tracking_definition_checks':tracking,'all_32_videos_decoded':probes,'physics_steps':0,'new_physical_attempts':0})
    save(OUT/'refresh_manifest.json',{'branch':BRANCH,'original_experiment_commit':SNAPSHOT,'original_handoff_sha256':sha(ROOT/'handoff.json'),
        'selected_video_run':SELECTED,'all_run_names':['E0','E0_C2','E1','E2'],'run_figure_count':44,'core_figure_count':4,'video_count':32,
        'new_physical_attempts':0,'physics_steps':0,'source_identity':sources,'artifacts':artifacts,'audit_sha256':sha(OUT/'visualization_audit.json'),
        'scope':'Current S02 display derivatives from unchanged archived traces; historical S01/N209 media retain their own provenance. User authorized separate GitHub branch upload after experiment completion.'})
    assert not errors,errors

if __name__=='__main__':build();audit()
