"""Publish current local navigation and verify regenerated visual evidence."""
import json
import re
import shutil
import subprocess
import sys
from html.parser import HTMLParser
from pathlib import Path, PureWindowsPath
from urllib.parse import unquote
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from v6_mujoco.feasible_capture.common import ROOT,PAPER,PROJECT_ROOT,read,save,sha
from v6_mujoco.feasible_capture.visualization import OUT,CAPTIONS

videos=read(OUT/'video_manifest.json');selected=videos['run'];refresh=read(OUT/'refresh_manifest.json')
errors=[];run_rows=[]
for name in refresh['all_run_names']:
    folder=OUT/'runs'/name;figs=read(folder/'figure_manifest.json');metrics=read(ROOT/'runs'/name/'metrics.json');trace=sha(ROOT/'runs'/name/'trace.npz')
    if set(figs)!=set(CAPTIONS):errors.append('incomplete figure set '+name)
    for key,item in figs.items():
        if item['trace_sha256']!=trace or item['run']!=name:errors.append('mixed figure provenance '+name+'/'+key)
        for ext in ['png','pdf']:
            if sha(folder/(key+'.'+ext))!=item[ext+'_sha256']:errors.append('figure digest '+name+'/'+key+'.'+ext)
    if read(folder/'tracking_consistency.json')['position_error_max_difference_m']>=1e-10:errors.append('tracking definition '+name)
    run_page=f'# {name} 可视化\n\n[返回全部运行](../../README.md) · [本地 HTML 图集](index.html)\n\n'
    window_status='EVALUATED' if metrics['full_window_evaluated'] else 'NOT_EVALUATED（缺少完整抓后时域）'
    run_page+=f'结果：{metrics["status"]}；实际结束 {metrics["end_time_s"]:.3f} s；末段窗口：{window_status}。\n\n'
    for key,item in figs.items():
        run_page+=f'## {item["title"]}\n\n[矢量 PDF]({key}.pdf)\n\n![{item["title"]}]({key}.png)\n\n{item["caption"]}\n\n'
    (folder/'README.md').write_text(run_page.rstrip()+'\n',encoding='utf-8')
    run_rows.append(f'| [{name}](runs/{name}/README.md) | {metrics["status"]} | {metrics["end_time_s"]:.3f} | [跟踪误差](runs/{name}/tracking.png) · [全部图表](runs/{name}/README.md) |')
probes={}
for key,item in videos['videos'].items():
    p=OUT/(key+'.mp4')
    if sha(p)!=item['sha256']:errors.append('video digest '+key)
    probe=json.loads(subprocess.check_output([shutil.which('ffprobe'),'-v','error','-threads','1','-count_frames','-select_streams','v:0','-show_entries','stream=nb_read_frames,width,height,codec_name','-of','json',str(p)],creationflags=subprocess.CREATE_NO_WINDOW))
    s=probe['streams'][0];probes[key]=s
    expected=item['probe']['streams'][0]
    if int(s['nb_read_frames'])!=videos['frames'] or any(s[k]!=expected[k] for k in ['width','height','codec_name']):errors.append('video decode '+key)
if videos['body_camera_max_adjacent_forward_angle_rad']>.05:errors.append('body camera orientation discontinuity')
if videos['body_camera_max_orthogonality_error']>1e-10:errors.append('body camera basis')
if videos['physics_steps']!=0 or videos['new_physical_attempts']!=0:errors.append('unexpected physical run')
for key,digest in read(PAPER.parent/'PAPER_CLAIM_AUDIT.json')['audited_input_hashes'].items():
    p=Path(key) if PureWindowsPath(key).is_absolute() else PAPER.parent/key
    if 'sha256:'+sha(p)!=digest:errors.append('paper-audit input changed '+key)
for key,digest in refresh['preserved_inputs'].items():
    if sha(PROJECT_ROOT/key)!=digest:errors.append('experiment input changed '+key)
for identity in [refresh['source_identity'],videos['source_identity']]:
    for key,digest in identity.items():
        if sha(PROJECT_ROOT/key)!=digest:errors.append('visual generator changed '+key)

markdown=f'''# N209 当前可视化

**PARTIAL_OPERATING_DOMAIN_MISMATCH**。全部 8 条已见尝试：3 条完成、5 条早停；独立盲测、消融及细步长未准入。

视频选取最终 V2 名义回归 `{selected}`，从 t=0 连续展示到 27.992 s，7.992 s 锁紧。画面是已验证 trace 的显示，不是新仿真或新的性能证据。

[交互式本地图集](index.html)（下载并执行 `git lfs pull hybrid` 后在浏览器打开） · [刷新清单](refresh_manifest.json) · [视频来源](video_manifest.json) · [完整报告](../../../../paper/N209_FEASIBILITY_AND_UNCERTAINTY_REPORT.md)

## 同步视频

GitHub 文件页可下载 MP4；HTML 视频播放需本地打开图集。合成总览为 2880 × 1200，请全屏或按原始尺寸读取面板中的数值。

- [五个固定视角与连续体侧合成总览](overview_five_views_and_body_side.mp4)
- [独立连续基座体侧视角](body_side.mp4)
- [接口近景](interface_closeup.mp4)
- 独立固定镜头：[等轴](isometric.mp4) · [正面](front.mp4) · [右侧](right.mp4) · [俯视](top.mp4) · [后方](rear.mp4)

![当前 N209 五视角与体侧终帧](overview_five_views_and_body_side_last.png)

## 全部运行

每条记录有 11 类诊断图，均提供 PNG 和矢量 PDF。早停运行在实际时间终止，缺失抓后窗标为 NOT_EVALUATED。

|运行|结果|实际结束 (s)|可视化|
|---|---|---|---|
'''+ '\n'.join(run_rows)+'\n\n## 当前名义回归的完整诊断图\n\n'
for key,(title,_) in CAPTIONS.items():markdown+=f'### {title}\n\n[矢量 PDF]({key}.pdf) · [全部运行中的对应图](runs/{selected}/{key}.png)\n\n![{title}]({key}.png)\n\n'
markdown+='## 复现与发布范围\n\n```powershell\npython -m v6_mujoco.feasible_capture --visualize\npython tools/n209_visualization_delivery.py\n```\n\n`--visualize --resume` 重建图表，并在核对 trace、渲染器和视频哈希后复用已有视频。原始实验、失败记录、审计输入和历史 N208 图均保留。\n\n本次是用户在原 N209 任务完成后另行要求的可视化刷新与 GitHub 独立分支上传。原 `commands.md` 的“本地提交、不推送”描述原任务范围；本次新增授权允许推送 `codex/n209-paper-ready-feasible-capture`，不合并主分支。显示刷新不重启实验账本、不产生新候选或新机器人尝试。\n\n[显示质量复核](visual_quality_review.md) · [完整解码及文件核验](visualization_audit.json)\n'
(OUT/'README.md').write_text(markdown,encoding='utf-8')
readme=PROJECT_ROOT/'README.md';text=readme.read_text(encoding='utf-8');start='<!-- N209 CURRENT START -->';end='<!-- N209 CURRENT END -->'
if start in text:
    left,right=text.split(start,1);_,right=right.split(end,1);text=left+right.lstrip('\n')
text=text.replace('## 当前分支 N208：','## 历史 N208：',1)
head,tail=text.split('\n',1)
prefix='output/fpmfc/n209_paper_system/visualizations'
block=f'''{start}
## 当前分支 N209：估计不确定性与参考可实现性

**PARTIAL_OPERATING_DOMAIN_MISMATCH**：8 条已见开发/回归中，3 条完成、5 条早停；全部统一双重重放通过。H2/S01 未完成，独立验证未准入。论文数据审计为 WARN，无实质数值不符；这不是鲁棒性能或硬件就绪结论。

- [完整报告](paper/N209_FEASIBILITY_AND_UNCERTAINTY_REPORT.md) · [论文 V1](paper/system_paper/manuscript_v1.md) · [审计](paper/PAPER_CLAIM_AUDIT.md)
- [当前全部可视化：8 个视频、88 组逐运行诊断图和5组论文主图]({prefix}/README.md) · [本地图集]({prefix}/index.html)
- [五视角与连续体侧总览]({prefix}/overview_five_views_and_body_side.mp4) · [独立连续体侧视频]({prefix}/body_side.mp4) · [接口近景]({prefix}/interface_closeup.mp4)
- [末端实际/参考轨迹]({prefix}/trajectory.png) · [跟踪误差]({prefix}/tracking.png) · [所有8条运行]({prefix}/README.md#全部运行)

视频来自最终 V2 名义回归 `{selected}`：7.992 s 锁紧，完整运行至27.992 s。参考在锁紧后停用，抓后保持量单独展示。所有失败记录仍可在图集中查看。

![N209 当前五视角与体侧终帧]({prefix}/overview_five_views_and_body_side_last.png)
![N209 当前末端轨迹跟踪误差]({prefix}/tracking.png)

下面 N208 及更早内容为历史记录，不作为当前 N209 的新增验证。
{end}
'''
readme.write_text(head+'\n\n'+block+tail,encoding='utf-8')
class Links(HTMLParser):
    def __init__(self):super().__init__();self.links=[]
    def handle_starttag(self,tag,attrs):
        self.links.extend(v for k,v in attrs if k in ['href','src','poster'] and v)
for p in OUT.rglob('*.html'):
    parser=Links();parser.feed(p.read_text(encoding='utf-8'))
    for link in parser.links:
        if link.startswith(('http:','https:','#','data:')):continue
        if not(p.parent/unquote(link.split('#')[0])).exists():errors.append('broken HTML link '+str(p)+': '+link)
refresh['artifacts']={p.relative_to(OUT).as_posix():sha(p) for p in OUT.rglob('*') if p.is_file() and p.suffix in ['.mp4','.png','.pdf','.html']}
refresh['navigation_sha256']={'README.md':sha(readme),'visualizations/README.md':sha(OUT/'README.md')}
refresh['run_readme_sha256']={p.relative_to(OUT).as_posix():sha(p) for p in (OUT/'runs').glob('*/README.md')}
refresh['quality_review_sha256']=sha(OUT/'visual_quality_review.md')
refresh['delivery_generator_sha256']=sha(Path(__file__));save(OUT/'refresh_manifest.json',refresh)
result={'passed':not errors,'errors':errors,'run_count':len(refresh['all_run_names']),'diagnostic_figures':refresh['run_figure_count'],
    'videos_decoded':len(probes),'decoded_streams':probes,'paper_audit_inputs_unchanged':not any('paper-audit' in e for e in errors),
    'new_physical_attempts':0,'selected_video_run':selected,'scope':'visual regeneration, exact source identity and complete video decoding; no new performance qualification',
    'validator_sha256':sha(Path(__file__))}
save(OUT/'visualization_audit.json',result);print(json.dumps(result,ensure_ascii=False));raise SystemExit(bool(errors))
