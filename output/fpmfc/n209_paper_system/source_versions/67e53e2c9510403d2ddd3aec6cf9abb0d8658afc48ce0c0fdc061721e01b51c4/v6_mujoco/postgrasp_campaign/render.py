"""Render verified saved integration states; never advance simulation dynamics."""
import argparse
import json
import shutil
import subprocess
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from v6_mujoco.model import PROJECT_ROOT
from v6_mujoco.visualize import VIEWS, _append_coordinate_frame
from v6_mujoco.postgrasp.physics import digest
from .io import ROOT, read, save, identity
from .engine import make_model, location, STATE_SPEC, interface_geoms, PHASES
from .report import collect

OUT = ROOT / "visualizations"
FONT = ImageFont.truetype("C:/Windows/Fonts/consola.ttf", 17)
FFMPEG = shutil.which("ffmpeg") or str(next(Path("C:/Users/admin/AppData/Local/Microsoft/WinGet/Packages").glob("Gyan.FFmpeg*/ffmpeg-*/bin/ffmpeg.exe")))
FFPROBE = str(Path(FFMPEG).with_name("ffprobe.exe"))


def probe(path):
    result = subprocess.run([FFPROBE,"-v","error","-select_streams","v:0","-show_entries","stream=width,height,nb_frames,codec_name:format=duration","-of","json",str(path)],capture_output=True,text=True,check=True,creationflags=subprocess.CREATE_NO_WINDOW)
    info=json.loads(result.stdout)
    assert int(info["streams"][0]["nb_frames"])==300 and abs(float(info["format"]["duration"])-10)<.001
    return info


class Encoder:
    def __init__(self,path,width,height):
        self.path=path; self.temporary=path.with_suffix(".partial.mp4"); self.log=path.with_suffix(".ffmpeg.log").open("wb")
        self.process=subprocess.Popen([FFMPEG,"-y","-loglevel","error","-f","rawvideo","-pix_fmt","rgb24","-s",f"{width}x{height}","-r","30","-i","-","-an","-c:v","libx264","-threads","2","-preset","fast","-crf","20","-pix_fmt","yuv420p","-movflags","+faststart",str(self.temporary)],stdin=subprocess.PIPE,stderr=self.log,stdout=subprocess.DEVNULL,creationflags=subprocess.CREATE_NO_WINDOW)

    def write(self,frame): self.process.stdin.write(np.asarray(frame,dtype=np.uint8).tobytes())

    def close(self):
        self.process.stdin.close(); code=self.process.wait(); self.log.close()
        if code: raise RuntimeError(f"video encoding failed: {self.path}")
        probe(self.temporary); self.temporary.replace(self.path)


def stamp(frame,name,title,t):
    im=Image.fromarray(frame); draw=ImageDraw.Draw(im)
    draw.rectangle((0,0,640,47),fill="#122434")
    draw.text((12,5),f"{name} | {title}",font=FONT,fill="white")
    draw.text((12,25),f"t={t:6.3f} s | saved-state playback",font=FONT,fill="#8bd3df")
    return im


def telemetry(name,a,i):
    im=Image.new("RGB",(640,360),"#122434"); draw=ImageDraw.Draw(im)
    phase={v:k for k,v in PHASES.items()}.get(int(a["phase_code"][i]),"UNKNOWN")
    lines=[f"{name} / {phase}",f"time             {a['time_s'][i]:.3f} s",
        f"target spin      {np.rad2deg(np.linalg.norm(a['target_omega_world_rad_s'][i])):.5f} deg/s",
        f"target-base      {np.rad2deg(np.linalg.norm(a['target_base_relative_omega_world_rad_s'][i])):.5f} deg/s",
        f"actual rho       {a['load_fraction'][i]:.6f} / 1",
        f"holding error    {1e6*a['interface_translation_error_m'][i]:.4f} um",
        f"SO(3) error      {a['interface_rotation_error_deg'][i]:.6f} deg",
        "Fixed acceptance window: 8..10 s", "Equivalent latch / model-level evidence",
        "Seven-joint Flexiv; no hardware capture claim"]
    for j,line in enumerate(lines): draw.text((18,16+32*j),line,font=FONT,fill="white" if j<7 else "#8bd3df")
    return im


def render_one(name,result,a,preview=False):
    folder=OUT/name; folder.mkdir(parents=True,exist_ok=True)
    record=folder/"manifest.json"
    source_id=identity([Path(__file__), PROJECT_ROOT/"v6_mujoco/visualize.py"])
    if not preview and record.exists():
        old=read(record)
        if old["render_identity"]==source_id and old["trace_sha256"]==result["trace_sha256"] and all(digest(folder/k)==v["sha256"] for k,v in old["videos"].items()):
            print(f"verified existing render: {name}",flush=True); return old
        raise RuntimeError(f"changed render identity: {name}; preserve old evidence explicitly")
    model=make_model(result["timestep_s"],result["scenario"]); data=mujoco.MjData(model)
    renderer=mujoco.Renderer(model,height=360,width=640)
    base=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"base")
    if base<0: base=int(model.jnt_bodyid[0])
    gids=interface_geoms(model)
    indices=np.rint(np.linspace(0,len(a["time_s"])-1,300)).astype(int)
    encoders={}
    keys=[v.key for v in VIEWS]+["body_side","five_views"]
    if not preview: encoders={k:Encoder(folder/(k+".mp4"),1920 if k=="five_views" else 640,720 if k=="five_views" else 360) for k in keys}
    try:
        for frame_index,i in enumerate(indices[:1] if preview else indices):
            mujoco.mj_setState(model,data,a["integration_state"][i],STATE_SPEC)
            model.geom_contype[gids],model.geom_conaffinity[gids]=a["interface_geom_masks"][i]
            mujoco.mj_forward(model,data)
            visible=model.geom_bodyid>0
            radii=model.geom_rbound[visible,None]
            lower=np.min(data.geom_xpos[visible]-radii,axis=0); upper=np.max(data.geom_xpos[visible]+radii,axis=0)
            center=(lower+upper)/2
            distance=max(2.8,1.15*np.linalg.norm(upper-lower)/2/np.sin(np.deg2rad(model.vis.global_.fovy/2)))
            frames=[]
            for n in range(6):
                cam=mujoco.MjvCamera(); cam.lookat[:]=center
                if n<5:
                    view=VIEWS[n]; title=view.title; cam.azimuth=view.azimuth_deg;cam.elevation=view.elevation_deg;cam.distance=distance
                else:
                    direction=data.xmat[base].reshape(3,3)@np.array([0,-1,.2]);title="Continuous body-side"
                    cam.azimuth=np.rad2deg(np.arctan2(direction[1],direction[0]));cam.elevation=-np.rad2deg(np.arctan2(direction[2],np.linalg.norm(direction[:2])));cam.distance=distance
                renderer.update_scene(data,camera=cam)
                _append_coordinate_frame(renderer.scene,a["tool_position_world_m"][i],a["tool_rotation_world"][i],.065,1.,.002)
                im=stamp(renderer.render(),name,title,float(a["time_s"][i])); frames.append(im)
                if not preview: encoders[keys[n]].write(im)
            composite=Image.new("RGB",(1920,720))
            for j,im in enumerate(frames[:5]+[telemetry(name,a,i)]): composite.paste(im,(640*(j%3),360*(j//3)))
            if not preview: encoders["five_views"].write(composite)
            if frame_index in (0,150,299):
                composite.save(folder/f"preview_{frame_index:03d}.png")
                frames[5].save(folder/f"body_side_{frame_index:03d}.png")
            if frame_index%100==0: print(f"{name}: rendered {frame_index+1}/300 frames",flush=True)
        for encoder in encoders.values(): encoder.close()
    except BaseException:
        for encoder in encoders.values():
            if encoder.process.poll() is None: encoder.process.kill();encoder.process.wait();encoder.log.close()
        raise
    finally: renderer.close()
    if preview: return None
    videos={k+".mp4":{"sha256":digest(folder/(k+".mp4")),"bytes":(folder/(k+".mp4")).stat().st_size,"probe":probe(folder/(k+".mp4"))} for k in keys}
    record_data={"name":name,"trace_sha256":result["trace_sha256"],"render_identity":source_id,"frames":300,"fps":30,"duration_s":10,"physical_state_indices":indices.tolist(),"physical_times_s":a["time_s"][indices].tolist(),"new_physics_steps":0,"method":"nearest saved integration state; mj_forward only; base-following side camera; no interpolation","videos":videos}
    save(record,record_data); return record_data


def build_index(records):
    cards=[]
    for name in records:
        links=" | ".join(f'<a href="{name}/{key}.mp4">{key}</a>' for key in [v.key for v in VIEWS]+["body_side","five_views"])
        cards.append(f'<section><h2>{name}</h2><video controls preload="none" poster="{name}/preview_150.png" src="{name}/five_views.mp4"></video><p>{links}</p><video class="side" controls preload="none" poster="{name}/body_side_150.png" src="{name}/body_side.mp4"></video></section>')
    plots="".join(f'<figure><img src="../figures/{p.stem}.png"><figcaption>{p.stem}</figcaption></figure>' for p in sorted((ROOT/"figures").glob("*.png")))
    html='<!doctype html><html lang="zh"><meta charset="utf-8"><title>抓持与消旋 / 已验证轨迹</title><style>body{font:17px system-ui;background:#edf3f5;color:#173346;margin:30px auto;max-width:1400px;padding:20px}section,figure{background:white;padding:20px;border-radius:14px;margin:24px 0}video,img{width:100%;display:block}.side{max-width:800px}a{color:#087f92}p{line-height:1.7}</style><h1>C1 后等效锁紧、保持与消旋</h1><p>名义、16 kg、24 kg 三个明确场景通过。原始 C1 速度保留；仅七关节阻尼。固定评价窗口 8–10 s。视频来自已验证完整状态；未新增动力学试验。</p><p>P_Z 保留为 6 ms 诊断误停，无完整视频。没有验证真实夹爪承载、完整接近抓捕或一般鲁棒性。连续体侧镜头跟随基座；模型是七关节 Flexiv。</p><p><a href="../../../../paper/POSTGRASP_DETUMBLING_CAMPAIGN_REPORT.md">完整报告</a> | <a href="../qualification_matrix.json">状态矩阵</a></p>'+"".join(cards)+plots+'</html>'
    (OUT/"index.html").write_text(html,encoding="utf-8",newline="\n")
    save(OUT/"manifest.json",{"conditions":records,"incomplete_condition":"P_Z: 0.006 s, not extended","new_physics_steps":0})


def main():
    parser=argparse.ArgumentParser();parser.add_argument("--preview",action="store_true");args=parser.parse_args()
    _,_,results,traces=collect();records={}
    for name in ["L_nominal","P_D","P_D_fine","L_fine","V_light","V_heavy"]:
        if results[name]["performance_passed"]:
            records[name]=render_one(name,results[name],traces[name],args.preview)
            if args.preview: break
    if not args.preview: build_index(records)


if __name__=="__main__": main()
