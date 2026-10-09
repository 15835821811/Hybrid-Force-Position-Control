"""Recorded-state display only: fixed world views and a continuous body camera."""
import argparse
import ctypes
from ctypes import wintypes
import json
import shutil
import subprocess
import time
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from v6_mujoco.visualize import VIEWS,_append_connector,_append_coordinate_frame
from v6_mujoco.model import body_id,geom_id
from v6_mujoco.postgrasp_campaign.engine import STATE_SPEC
from v6_mujoco.geometry_capture.audit import ordered_box_witness
from v6_mujoco.geometry_capture.design import NEW_MODEL
from v6_mujoco.adaptive_capture.plant import Plant,TruthConfig
from v6_mujoco.adaptive_capture.evaluation import PHASES
from v6_mujoco.end_to_end_capture.adapter import pads
from .common import ROOT,PROJECT_ROOT,save,sha
from .visualization import OUT,SELECTED,load_run

WIDTH,HEIGHT,FPS=960,600,30
TITLES={'isometric':'等轴视角','front':'正面视角','right':'右侧视角','top':'俯视视角','rear':'后方视角',
        'overview_five_views_and_body_side':'五个固定视角 + 连续基座体侧总览','body_side':'独立连续基座体侧视角','interface_closeup':'接口与实际几何近景'}

def child_cpu(proc):
    values=[wintypes.FILETIME() for _ in range(4)]
    fn=ctypes.windll.kernel32.GetProcessTimes
    fn.argtypes=[wintypes.HANDLE]+[ctypes.POINTER(wintypes.FILETIME)]*4;fn.restype=wintypes.BOOL
    if not fn(wintypes.HANDLE(int(proc._handle)),*(ctypes.byref(x) for x in values)):return None
    return sum((x.dwHighDateTime<<32)+x.dwLowDateTime for x in values[2:])/1e7

def run(name=SELECTED, samples_only=False):
    start=time.time();cpu=time.process_time();folder,a,result,cfg,digest=load_run(name)
    OUT.mkdir(parents=True,exist_ok=True);ffmpeg=shutil.which('ffmpeg');ffprobe=shutil.which('ffprobe')
    if not ffmpeg or not ffprobe:raise RuntimeError('ffmpeg and ffprobe are required')
    t=a['time_s'];n=int(np.ceil(t[-1]*FPS))+1;presentation=np.minimum(np.arange(n)/FPS,t[-1])
    right=np.minimum(np.searchsorted(t,presentation),len(t)-1);left=np.maximum(0,right-1)
    indices=np.where(abs(t[right]-presentation)<abs(t[left]-presentation),right,left);indices[-1]=len(t)-1
    assert indices[0]==0 and indices[-1]==len(t)-1 and np.all(np.diff(indices)>=0)
    m=Plant(TruthConfig(**cfg['truth_evaluation_only']),cfg['dt'],cfg['mission'].get('solver_tolerance')).model
    m.vis.global_.offwidth=WIDTH;m.vis.global_.offheight=HEIGHT;d=mujoco.MjData(m);renderer=mujoco.Renderer(m,height=HEIGHT,width=WIDTH)
    gids=pads(m);base=body_id(m,'base_link_0');cube=geom_id(m,'tumbling_target_geom')
    m.geom_matid[cube]=-1;m.geom_rgba[cube]=[.1,.65,.35,.3]
    visible=m.geom_bodyid>0;radii=m.geom_rbound[visible,None];lo=np.full(3,np.inf);hi=-lo.copy()
    for i in indices:
        mujoco.mj_setState(m,d,a['integration_state'][i],STATE_SPEC);m.geom_contype[gids],m.geom_conaffinity[gids]=a['interface_geom_masks'][i];mujoco.mj_forward(m,d)
        lo=np.minimum(lo,np.min(d.geom_xpos[visible]-radii,axis=0));hi=np.maximum(hi,np.max(d.geom_xpos[visible]+radii,axis=0))
    fixed_center=(lo+hi)/2;fixed_distance=max(2.8,1.18*np.linalg.norm(hi-lo)/2/np.sin(np.deg2rad(m.vis.global_.fovy/2)))
    sizes={v.key:(WIDTH,HEIGHT) for v in VIEWS};sizes.update({'overview_five_views_and_body_side':(3*WIDTH,2*HEIGHT),'body_side':(WIDTH,HEIGHT),'interface_closeup':(WIDTH,HEIGHT)})
    phases={v:k for k,v in PHASES.items()};font=ImageFont.truetype('C:/Windows/Fonts/consola.ttf',18)
    encoders={};logs={};encoder_cpu={};camera_checks=[]
    preview=OUT/'previews';preview.mkdir(exist_ok=True)
    requested={0,n-1,n//2}
    if result['latch_time_s'] is not None:requested.add(int(np.argmin(abs(presentation-result['latch_time_s']))))
    frames_to_draw=sorted(requested) if samples_only else range(n)
    if not samples_only:
        for key,(w,h) in sizes.items():
            logs[key]=(OUT/(key+'.ffmpeg.log')).open('wb')
            encoders[key]=subprocess.Popen([ffmpeg,'-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{w}x{h}','-r',str(FPS),'-i','-','-an','-c:v','libx264','-threads','1','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(OUT/(key+'.partial.mp4'))],stdin=subprocess.PIPE,stderr=logs[key],stdout=subprocess.DEVNULL,creationflags=subprocess.CREATE_NO_WINDOW)
    def panel(image,i,title,close=False):
        pic=Image.fromarray(image);draw=ImageDraw.Draw(pic);draw.rectangle((0,0,WIDTH,103),fill='#132739')
        phase=phases[int(a['phase_code'][i])];local=f"{a['latch_local_s'][i]:.3f}s" if a['eq_active'][i,0] else 'N/A'
        error=f"tracking {a['approach_position_error_m'][i]*1000:.4f} mm" if a['approach_reference_applicable'][i] else f"grasp gap {a['interface_translation_error_m'][i]*1000:.4f} mm"
        if i==len(t)-1:phase=result['status']
        overlay='red: distance witness; axes: true grasp' if close else ('red: actual flange; blue: active reference' if a['approach_reference_applicable'][i] else 'reference inactive after latch; actual geometry shown')
        lines=[f'N209 | {name} | {title}',f't={t[i]:.3f}s | post-latch {local} | {phase}',f"target {np.rad2deg(np.linalg.norm(a['target_omega_world_rad_s'][i])):.4f} deg/s | {error}",f'{overlay} | simulation only']
        for j,line in enumerate(lines):draw.text((10,3+24*j),line,font=font,fill='white' if j<2 else '#8ad5dc')
        return pic
    def trail(i):
        ids=np.unique(np.linspace(max(0,i-700),i,min(45,i+1)).astype(int))
        for key,color in [('flange_position_world_m',[.9,.2,.1,1]),('approach_reference_position_world_m',[.1,.4,1,1])]:
            for l,r in zip(ids[:-1],ids[1:]):
                if a['approach_reference_applicable'][l] and a['approach_reference_applicable'][r] and np.linalg.norm(a[key][r]-a[key][l])>1e-10:
                    _append_connector(renderer.scene,a[key][l],a[key][r],.0015,np.array(color))
    try:
        for frame in frames_to_draw:
            i=indices[frame];mujoco.mj_setState(m,d,a['integration_state'][i],STATE_SPEC)
            m.geom_contype[gids],m.geom_conaffinity[gids]=a['interface_geom_masks'][i];mujoco.mj_forward(m,d)
            frames={}
            for view in VIEWS:
                cam=mujoco.MjvCamera();cam.lookat[:]=fixed_center;cam.distance=fixed_distance;cam.azimuth=view.azimuth_deg;cam.elevation=view.elevation_deg
                renderer.update_scene(d,camera=cam);trail(i);frames[view.key]=panel(renderer.render(),i,view.title)
            local_lo=np.min(d.geom_xpos[visible]-radii,axis=0);local_hi=np.max(d.geom_xpos[visible]+radii,axis=0)
            center=(local_lo+local_hi)/2;R=d.xmat[base].reshape(3,3)
            forward=R@np.array([0.,1.,-.20]);forward/=np.linalg.norm(forward)
            up=R@np.array([0.,.20,1.]);up/=np.linalg.norm(up)
            distance=max(2.8,1.18*np.linalg.norm(local_hi-local_lo)/2/np.sin(np.deg2rad(m.vis.global_.fovy/2)))
            cam=mujoco.MjvCamera();cam.lookat[:]=center;cam.distance=distance
            renderer.update_scene(d,camera=cam)
            # Both OpenGL eyes use a continuously rotated body-frame basis,
            # including base roll; no Euler-angle wrap or viewpoint switching.
            for eye in renderer.scene.camera:
                eye.pos[:]=center-distance*forward;eye.forward[:]=forward;eye.up[:]=up
            camera_checks.append({'time_s':float(t[i]),'forward_world':forward.tolist(),'up_world':up.tolist(),'orthogonality_error':float(abs(forward@up))})
            trail(i);frames['body_side']=panel(renderer.render(),i,'Continuous base-body side')
            composite=Image.new('RGB',(WIDTH*3,HEIGHT*2))
            for j,key in enumerate([v.key for v in VIEWS]+['body_side']):composite.paste(frames[key],(WIDTH*(j%3),HEIGHT*(j//3)))
            frames['overview_five_views_and_body_side']=composite
            cam=mujoco.MjvCamera();cam.lookat[:]=.5*(a['tool_position_world_m'][i]+a['target_grasp_position_world_m'][i]);cam.distance=max(.58,2*np.linalg.norm(a['tool_position_world_m'][i]-a['target_grasp_position_world_m'][i]))
            graspR=a['target_grasp_rotation_world'][i];direction=-graspR[:,2]+.7*graspR[:,0]+.4*graspR[:,1]
            cam.azimuth=np.rad2deg(np.arctan2(direction[1],direction[0]));cam.elevation=-np.rad2deg(np.arctan2(direction[2],np.linalg.norm(direction[:2])))
            renderer.update_scene(d,camera=cam);segment=np.zeros(6);gap=mujoco.mj_geomDistance(m,d,geom_id(m,'link7_collision'),cube,2,segment);segment,_,_=ordered_box_witness(m,d,'tumbling_target_geom',segment)
            if np.linalg.norm(segment[3:]-segment[:3])>1e-10:_append_connector(renderer.scene,segment[:3],segment[3:],.001,np.array([1,.1,.1,1]))
            _append_coordinate_frame(renderer.scene,a['target_grasp_position_world_m'][i],graspR,.035,.8,.0008)
            frames['interface_closeup']=panel(renderer.render(),i,f'Interface | link7 gap {gap*1000:.3f} mm',True)
            for key,pic in frames.items():
                if not samples_only:encoders[key].stdin.write(np.asarray(pic,dtype=np.uint8).tobytes())
                if frame in requested:pic.save(preview/(key+f'_{frame:04d}.png'))
                if frame==n-1:pic.save(OUT/(key+'_last.png'))
            if frame%100==0 or samples_only:print(f'N209 render {frame+1}/{n}',flush=True)
        for key,proc in encoders.items():
            proc.stdin.close();code=proc.wait();encoder_cpu[key]=child_cpu(proc);logs[key].close()
            if code:raise RuntimeError('FFmpeg failed: '+key)
            (OUT/(key+'.partial.mp4')).replace(OUT/(key+'.mp4'))
    finally:
        for proc in encoders.values():
            if proc.poll() is None:proc.kill();proc.wait()
        for handle in logs.values():
            if not handle.closed:handle.close()
        renderer.close()
    if samples_only:return
    videos={}
    for key,(width,height) in sizes.items():
        path=OUT/(key+'.mp4');probe=json.loads(subprocess.check_output([ffprobe,'-v','error','-select_streams','v:0','-show_entries','stream=width,height,nb_frames,codec_name,pix_fmt,avg_frame_rate:format=duration','-of','json',str(path)],creationflags=subprocess.CREATE_NO_WINDOW))
        stream=probe['streams'][0];assert int(stream['nb_frames'])==n and stream['width']==width and stream['height']==height and stream['codec_name']=='h264'
        videos[key]={'title':TITLES[key],'sha256':sha(path),'probe':probe,'encoder_process_cpu_s':encoder_cpu[key]}
    max_angle=0.
    for l,r in zip(camera_checks[:-1],camera_checks[1:]):max_angle=max(max_angle,float(np.arccos(np.clip(np.dot(l['forward_world'],r['forward_world']),-1,1))))
    save(OUT/'body_camera_samples.json',camera_checks)
    save(OUT/'video_manifest.json',{'run':name,'trace_sha256':digest,'validation_sha256':sha(folder/'validation_timestamp_audited.json'),
        'model_sha256':sha(NEW_MODEL),'source_identity':{str(Path(__file__).relative_to(PROJECT_ROOT)):sha(Path(__file__))},
        'physical_end_time_s':float(t[-1]),'latch_time_s':result['latch_time_s'],'physics_steps':0,'new_physical_attempts':0,
        'fps':FPS,'frames':n,'indices':indices.tolist(),'physical_times_s':t[indices].tolist(),'videos':videos,
        'body_camera_max_adjacent_forward_angle_rad':max_angle,'body_camera_max_orthogonality_error':max(x['orthogonality_error'] for x in camera_checks),
        'body_camera':'continuous base-rotation forward/up vectors including roll; scene center framing; no angle wrap',
        'display_only':'recorded integration states restored for display; no mj_step, no new physics, no historical splicing; target transparency and trajectory overlays only',
        'parent_process_cpu_s':time.process_time()-cpu,'encoder_process_cpu_s':sum(x for x in encoder_cpu.values() if x is not None),'wall_s':time.time()-start})
    print(json.dumps({'videos':len(videos),'frames':n,'run':name}),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--name',default=SELECTED);p.add_argument('--samples-only',action='store_true');args=p.parse_args();run(args.name,args.samples_only)
