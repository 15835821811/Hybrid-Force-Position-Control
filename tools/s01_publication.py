"""Build the current branch gallery without changing any scientific outcome."""
import html
import json
import re
import subprocess
import sys
import time
from pathlib import Path
from html.parser import HTMLParser
from urllib.parse import unquote
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from v6_mujoco.system_capture.paper_bridge.common import OUT,PROJECT,read,save,sha,git
from v6_mujoco.system_capture.paper_bridge.report import finalize

def run():
    start=time.time();branch=git('branch','--show-current');q=read(OUT/'qualification.json')
    old=PROJECT/'output/fpmfc/n209_paper_system/visualizations'
    gallery=OUT.parent/'visualizations';gallery.mkdir(parents=True,exist_ok=True)
    errors=[]
    for path in [OUT/'figures/manifest.json',old/'visualization_audit.json',OUT/'visualizations/video_manifest.json']:
        if not path.exists():raise RuntimeError('Missing publication input '+str(path))
    if not read(old/'visualization_audit.json')['passed']:raise RuntimeError('N209 visual audit failed')
    manifest=read(OUT/'figures/manifest.json')
    for name,row in manifest.items():
        for ext,h in row['files'].items():
            if sha(OUT/'figures'/(name+'.'+ext))!=h:errors.append('S01 figure hash '+name)
        for rel,h in row['inputs'].items():
            if sha(PROJECT/rel)!=h:errors.append('S01 figure input '+rel)
    video=read(OUT/'visualizations/video_manifest.json')
    if sha(OUT/'visualizations'/video['video'])!=video['video_sha256']:errors.append('SRS video digest')
    prefix='output/fpmfc/system_capture'
    block=f"""<!-- S01 CURRENT START -->
## 当前版本：S01 原文兼容 SRS 基准与完整可视化

发布分支：`{branch}`。数学检查11/11；来源参数为 **SOURCE_LIMITED**。
C2动力学合格 **{q['dynamic_feasibility']['C2_qualified']}/{q['dynamic_feasibility']['C2_attempts']}**；可用C2记录均独立重放通过，但C1有1次不可恢复的原始输入记录缺口，全批重放为 **PARTIAL**。

- [统一可视化入口]({prefix}/visualizations/README.md) · [本地交互图集]({prefix}/visualizations/index.html)
- [S01报告]({prefix}/S01/report.md) · [资格字段]({prefix}/S01/qualification.json) · [逐项验收]({prefix}/S01/completion_audit.json) · [复现章节](paper/system_framework/reproduction_section.md)
- [SRS五视角与连续体侧视频]({prefix}/S01/visualizations/{video['video']}) · [SRS实际跟踪误差]({prefix}/S01/figures/tracking_and_dynamics.png)
- [完整捕获五视角与连续体侧总览](output/fpmfc/n209_paper_system/visualizations/overview_five_views_and_body_side.mp4) · [独立体侧视频](output/fpmfc/n209_paper_system/visualizations/body_side.mp4)
- [完整捕获末端轨迹](output/fpmfc/n209_paper_system/visualizations/trajectory.png) · [跟踪误差](output/fpmfc/n209_paper_system/visualizations/tracking.png)

本次刷新4组S01核心图及1段SRS视频，同时重新生成保存的N209全部88组诊断图、5组论文主图及8段同步视频。新SRS为接触前基准，N209视频为已有Flexiv连续捕获记录；各自的模型、数据来源和失败结论在图集中明确标注。未运行S02—S08。

![S01实际轨迹跟踪与动力学]({prefix}/S01/figures/tracking_and_dynamics.png)

<!-- S01 CURRENT END -->

"""
    readme=PROJECT/'README.md';text=readme.read_text(encoding='utf-8')
    text=re.sub(r'<!-- S01 CURRENT START -->.*?<!-- S01 CURRENT END -->\s*','',text,flags=re.S)
    title,rest=text.split('\n',1);readme.write_text(title+'\n\n'+block+rest.lstrip(),encoding='utf-8')
    md='# 当前分支完整可视化\n\n[本地交互图集](index.html) · [S01报告](../S01/report.md) · [GitHub仓库首页](../../../../README.md)\n\n'
    md+='当前 S01：独立 SRS 接触前基准；历史 N209：已有 Flexiv 连续捕获记录。两者不混作同一实验。\n\n## S01 SRS\n\n'
    md+='[五个固定视角与连续体侧合成视频](../S01/visualizations/'+video['video']+')\n\n![真实停止帧](../S01/visualizations/srs_last.png)\n\n'
    body='<h1>当前版本 · 完整可视化</h1><p>当前 S01 为 SRS 接触前基准；保存的 N209 为 Flexiv 连续捕获。失败和来源限制均保留。</p><nav><a href="#srs">S01</a> · <a href="#capture">连续捕获</a> · <a href="#allruns">全部诊断图</a></nav>'
    body+='<section id="srs"><h2>S01 SRS</h2><p>数学检查11/11；SOURCE_LIMITED；C1记录缺口使全批重放为PARTIAL。作者报告点在实际停止时间结束。</p>'
    body+='<video controls preload="metadata" poster="../S01/visualizations/srs_last.png" src="../S01/visualizations/'+video['video']+'"></video>'
    for name,row in manifest.items():
        md+='### '+row['title']+'\n\n[PDF](../S01/figures/'+name+'.pdf)\n\n!['+row['title']+'](../S01/figures/'+name+'.png)\n\n'
        body+='<h3>'+row['title']+'</h3><a href="../S01/figures/'+name+'.pdf">PDF</a><img loading="lazy" src="../S01/figures/'+name+'.png"><p>'+html.escape(row['caption'])+'</p>'
    body+='</section><section id="capture"><h2>已有 N209 完整捕获视频</h2><p>R01_V2_nominal：7.992 s锁紧，真实运行到27.992 s。8条已见记录中3条完成、5条早停。H2/噪声工作域未通过；本次仅从原trace再生成显示。</p>'
    md+='## 已有 N209 连续捕获\n\n8段同步视频、88组逐运行诊断图、5组论文主图全部重新生成。\n\n'
    for key,label in [('overview_five_views_and_body_side','五视角与连续体侧总览'),('body_side','连续基座体侧'),('interface_closeup','接口近景'),('isometric','等轴'),('front','正面'),('right','右侧'),('top','俯视'),('rear','后方')]:
        rel='../../n209_paper_system/visualizations/'+key+'.mp4'
        md+='- ['+label+']('+rel+')\n'
        body+='<details'+(' open' if key=='overview_five_views_and_body_side' else '')+'><summary>'+label+'</summary><video controls preload="none" poster="../../n209_paper_system/visualizations/'+key+'_last.png" src="'+rel+'"></video></details>'
    body+='</section><section id="allruns"><h2>末端轨迹、误差和全部运行</h2>'
    for key,label in [('trajectory','实际与参考末端轨迹'),('tracking','末端轨迹跟踪误差'),('base_motion','基座运动'),('momentum_energy','动量与能量')]:
        path='../../n209_paper_system/visualizations/'+key+'.png'
        md+='\n### '+label+'\n\n!['+label+']('+path+')\n'
        body+='<h3>'+label+'</h3><img loading="lazy" src="'+path+'">'
    md+='\n[全部8条运行、全部诊断类别与来源](../../n209_paper_system/visualizations/README.md) · [N209本地图集](../../n209_paper_system/visualizations/index.html)\n\n'
    body+='<p><a href="../../n209_paper_system/visualizations/index.html">全部8条运行的88组诊断图及5组论文主图</a></p></section>'
    md+='下载后先执行 `git lfs pull hybrid`，再用浏览器打开index.html。GitHub中可浏览PNG/PDF与下载MP4；HTML互动播放需本地打开。\n'
    (gallery/'README.md').write_text(md,encoding='utf-8')
    (gallery/'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>S01 与连续捕获可视化</title><style>body{max-width:1250px;margin:30px auto;padding:0 20px;font:16px/1.65 system-ui;color:#173044;background:#f6f8fa}section{padding:24px;background:white;margin:24px 0;border:1px solid #d7e1e8;border-radius:12px}img,video{width:100%;height:auto}a{color:#086aa4}summary{cursor:pointer}nav{position:sticky;top:0;background:#f6f8faf2;padding:12px}</style>'+body+'</html>',encoding='utf-8')
    class Links(HTMLParser):
        def __init__(self):super().__init__();self.links=[]
        def handle_starttag(self,tag,attrs):self.links.extend(v for k,v in attrs if k in ['href','src','poster'] and v)
    for path in list(gallery.glob('*.html'))+list((OUT/'figures').glob('*.html')):
        parser=Links();parser.feed(path.read_text(encoding='utf-8'))
        for ref in parser.links:
            if ref.startswith(('http:','https:','#','data:')):continue
            if not(path.parent/unquote(ref.split('#')[0])).exists():errors.append('broken link '+ref)
    # Root navigation legitimately changes after the N209-only delivery audit.
    refresh=read(old/'refresh_manifest.json');refresh['navigation_sha256']['README.md']=sha(readme)
    refresh['publication']['current_gallery']='output/fpmfc/system_capture/visualizations/README.md';save(old/'refresh_manifest.json',refresh)
    artifacts={p.relative_to(PROJECT).as_posix():sha(p) for root in [OUT/'figures',OUT/'visualizations',gallery] for p in root.rglob('*') if p.is_file() and p.suffix in ['.png','.pdf','.mp4','.html','.md','.json']}
    for p in [old/'refresh_manifest.json',old/'video_manifest.json',old/'visualization_audit.json',PROJECT/'README.md']:artifacts[p.relative_to(PROJECT).as_posix()]=sha(p)
    save(OUT/'publication_manifest.json',{'branch':branch,'repository':'15835821811/Hybrid-Force-Position-Control','remote':'hybrid',
      'user_authorization':'Current version on a separate GitHub branch; refresh all current visualizations including five views, continuous body-side videos and end-effector tracking errors.',
      'merged':False,'display_refresh_new_physical_attempts':0,'srs_figure_groups':4,'srs_videos':1,'retained_n209_figure_groups':88,'retained_n209_paper_figure_groups':5,'retained_n209_videos':8,
      'byte_identical_outputs_are_valid_regeneration':True,'s00_acceptance_unchanged':True,'links_and_hashes_passed':not errors,'errors':errors,
      'remote_confirmation':'Compare remote branch HEAD with containing Git commit after push; never embed a self-referential commit SHA',
      'artifacts_sha256':artifacts,'generator_sha256':sha(Path(__file__)),'wall_s':time.time()-start})
    if errors:raise RuntimeError(str(errors))
    finalize()
    print({'publication_gallery':'ready','branch':branch,'errors':errors},flush=True)

if __name__=='__main__':run()
