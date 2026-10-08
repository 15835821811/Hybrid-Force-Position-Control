"""Saved-state media of the ACTUAL stopped horizon; no appended postgrasp video."""
import json
import subprocess
import shutil
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from v6_mujoco.visualize import VIEWS,_append_coordinate_frame,_append_connector
from v6_mujoco.model import body_id,geom_id
from v6_mujoco.postgrasp_campaign.engine import STATE_SPEC
from .common import *
from .adapter import compile_model,pads

OUT=ROOT/"visualizations"
FONT=ImageFont.truetype("C:/Windows/Fonts/consola.ttf",16)
FFMPEG=shutil.which("ffmpeg") or str(next(Path("C:/Users/admin/AppData/Local/Microsoft/WinGet/Packages").glob("Gyan.FFmpeg*/ffmpeg-*/bin/ffmpeg.exe")))

def run():
    verify_design();out=ROOT/"E_nominal";r=read(out/"metrics.json");validation=read(out/"validation.json");assert validation["passed"] and validation["trace_sha256"]==digest(out/"trace.npz")
    OUT.mkdir(exist_ok=True);a=arrays(out/"trace.npz");n=int(np.ceil(a["time_s"][-1]*30))+1;indices=np.rint(np.linspace(0,len(a["time_s"])-1,n)).astype(int)
    m=compile_model();m.vis.global_.offwidth=800;m.vis.global_.offheight=480
    cube=geom_id(m,"tumbling_target_geom");m.geom_matid[cube]=-1;m.geom_rgba[cube]=[.1,.65,.85,.30] # Display-only transparency exposes the monitored interface.
    d=mujoco.MjData(m);renderer=mujoco.Renderer(m,height=480,width=800);gids=pads(m);base=body_id(m,"base_link_0")
    encoders={};logs={};sizes={"overview":(800,480),"interface_closeup":(800,480),"five_views":(2400,960),"body_side":(800,480)}
    for key,(w,h) in sizes.items():
        logs[key]=(OUT/(key+".ffmpeg.log")).open("wb")
        encoders[key]=subprocess.Popen([FFMPEG,"-y","-loglevel","error","-f","rawvideo","-pix_fmt","rgb24","-s",f"{w}x{h}","-r","30","-i","-","-an","-c:v","libx264","-threads","2","-preset","fast","-crf","20","-pix_fmt","yuv420p","-movflags","+faststart",str(OUT/(key+".partial.mp4"))],stdin=subprocess.PIPE,stderr=logs[key],stdout=subprocess.DEVNULL,creationflags=subprocess.CREATE_NO_WINDOW)
    def panel(im,i,title):
        pic=Image.fromarray(im);dr=ImageDraw.Draw(pic);dr.rectangle((0,0,800,72),fill="#132739")
        state="FAILED: clearance gate" if i==indices[-1] else "APPROACH"
        dr.text((12,5),f"N205 | {title} | {state}",fill="white",font=FONT)
        dr.text((12,27),f"absolute t={a['time_s'][i]:.3f} s | post-latch time: N/A",fill="#8ad5dc",font=FONT)
        dr.text((12,49),f"target {np.rad2deg(np.linalg.norm(a['target_omega_world_rad_s'][i])):.3f} deg/s | ref error {a['approach_position_error_m'][i]*1000:.4f} mm",fill="#8ad5dc",font=FONT)
        return pic
    try:
        for frame,i in enumerate(indices):
            mujoco.mj_setState(m,d,a["integration_state"][i],STATE_SPEC);m.geom_contype[gids],m.geom_conaffinity[gids]=a["interface_geom_masks"][i];mujoco.mj_forward(m,d)
            visible=m.geom_bodyid>0;radii=m.geom_rbound[visible,None];lo=np.min(d.geom_xpos[visible]-radii,axis=0);hi=np.max(d.geom_xpos[visible]+radii,axis=0);center=(lo+hi)/2;distance=max(2.8,1.15*np.linalg.norm(hi-lo)/2/np.sin(np.deg2rad(m.vis.global_.fovy/2)))
            frames=[]
            for view in VIEWS:
                cam=mujoco.MjvCamera();cam.lookat[:]=center;cam.distance=distance;cam.azimuth=view.azimuth_deg;cam.elevation=view.elevation_deg
                renderer.update_scene(d,camera=cam);frames.append(panel(renderer.render(),i,view.title))
            close=mujoco.MjvCamera();close.lookat[:]=.5*(a["tool_position_world_m"][i]+a["target_grasp_position_world_m"][i]);close.distance=max(.70,2*np.linalg.norm(a["tool_position_world_m"][i]-a["target_grasp_position_world_m"][i]))
            R=a["target_grasp_rotation_world"][i];outward=-R[:,2]+.7*R[:,0]+.4*R[:,1]
            close.azimuth=np.rad2deg(np.arctan2(outward[1],outward[0]));close.elevation=-np.rad2deg(np.arctan2(outward[2],np.linalg.norm(outward[:2])))
            renderer.update_scene(d,camera=close)
            segment=np.zeros(6);mujoco.mj_geomDistance(m,d,geom_id(m,"link7_collision"),geom_id(m,"tumbling_target_geom"),2.,segment)
            if np.linalg.norm(segment[3:]-segment[:3])>1e-10:_append_connector(renderer.scene,segment[:3],segment[3:],.001,np.array([1,.15,.1,1]))
            _append_coordinate_frame(renderer.scene,a["target_grasp_position_world_m"][i],a["target_grasp_rotation_world"][i],.035,.8,.0008)
            closepic=panel(renderer.render(),i,"Interface / red = link7-cube gap")
            side=mujoco.MjvCamera();side.lookat[:]=center;direction=d.xmat[base].reshape(3,3)@np.array([0,-1,.2]);side.azimuth=np.rad2deg(np.arctan2(direction[1],direction[0]));side.elevation=-np.rad2deg(np.arctan2(direction[2],np.linalg.norm(direction[:2])));side.distance=distance
            renderer.update_scene(d,camera=side);sidepic=panel(renderer.render(),i,"Continuous body-side camera")
            info=Image.new("RGB",(800,480),"#132739");dr=ImageDraw.Draw(info)
            textlines=["ACTUAL RECORDED HORIZON ONLY",f"Stopped at {r['end_time_s']:.3f} s before t=8 latch",f"Frame absolute time: {a['time_s'][i]:.3f} s","No pad/plate contact before stop","Capture / holding / detumbling: NOT EVALUATED","No appended 10 s postgrasp playback","Reference error shown = approach tracking","Seven-joint Flexiv; free physical target","Source trace replay verified; no new physics"]
            for j,line in enumerate(textlines):dr.text((20,25+42*j),line,font=FONT,fill="white" if j<5 else "#8ad5dc")
            composite=Image.new("RGB",(2400,960))
            for j,pic in enumerate(frames+[info]):composite.paste(pic,(800*(j%3),480*(j//3)))
            pictures={"overview":frames[0],"interface_closeup":closepic,"body_side":sidepic,"five_views":composite}
            for key,pic in pictures.items():encoders[key].stdin.write(np.asarray(pic,dtype=np.uint8).tobytes())
            if frame in (0,n//2,n-1):
                for key,pic in pictures.items():pic.save(OUT/(key+f"_{frame:03d}.png"))
            if frame%100==0:print(f"N205 saved-state media: {frame+1}/{n}",flush=True)
        for key,proc in encoders.items():
            proc.stdin.close();code=proc.wait();logs[key].close()
            if code:raise RuntimeError("encoding failed: "+key)
            (OUT/(key+".partial.mp4")).replace(OUT/(key+".mp4"))
    finally:
        for proc in encoders.values():
            if proc.poll() is None:proc.kill();proc.wait()
        renderer.close()
    videos={}
    for key,(w,h) in sizes.items():
        path=OUT/(key+".mp4");probe=json.loads(subprocess.check_output([str(Path(FFMPEG).with_name("ffprobe.exe")),"-v","error","-select_streams","v:0","-show_entries","stream=width,height,nb_frames:format=duration","-of","json",str(path)],creationflags=subprocess.CREATE_NO_WINDOW))
        assert int(probe["streams"][0]["nb_frames"])==n and probe["streams"][0]["width"]==w
        videos[key]={"sha256":digest(path),"probe":probe}
    save(OUT/"manifest.json",{"trace_sha256":r["trace_sha256"],"render_identity":identity([__file__]),"physical_end_time_s":r["end_time_s"],"frames":n,"fps":30,"physical_indices":indices.tolist(),"physical_times_s":a["time_s"][indices].tolist(),"new_physics_steps":0,"display_only_changes":"800x480 framebuffer and blue translucent target cube, no geometry/physical-state change","video_duration_policy":"ceil(duration*30)+1 frames, nearest archived states incl stop; encoding time differs by under two frames, no fabricated physical samples","videos":videos})
    cards=''.join(f'<section><h2>{key}</h2><video controls preload="none" poster="{key}_{n-1:03d}.png" src="{key}.mp4"></video></section>' for key in sizes)
    figs=''.join(f'<figure><img src="../figures/{p.name}"><figcaption>{p.stem}</figcaption></figure>' for p in sorted((ROOT/"figures").glob("*.png")))
    html='<!doctype html><html lang="zh"><meta charset="utf-8"><title>N205 连续动力学：安全早停</title><style>body{font:17px system-ui;background:#edf3f5;color:#163246;max-width:1300px;margin:30px auto;padding:20px}section,figure{background:white;padding:20px;border-radius:12px;margin:24px 0}img,video{width:100%}p{line-height:1.7}</style><h1>N205：从 home 连续积分，7.362 s 安全早停</h1><p>link7 与目标的距离低于冻结余量。尚未接触、尚未到达 8 s 捕获检查；未验证锁紧与消旋。以下为真实停止时域，未拼接旧抓后视频。</p><p><a href="../../../../paper/N205_END_TO_END_CAPTURE_DETUMBLING_REPORT.md">报告</a> · <a href="../qualification_matrix.json">状态矩阵</a></p>'+cards+figs+'</html>'
    (OUT/"index.html").write_text(html,encoding="utf-8",newline="\n")

if __name__=="__main__":run()
