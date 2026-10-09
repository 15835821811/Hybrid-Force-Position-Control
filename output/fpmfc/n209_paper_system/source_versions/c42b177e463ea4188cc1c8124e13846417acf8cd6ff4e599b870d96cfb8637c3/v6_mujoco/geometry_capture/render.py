"""Two videos, generated only from this campaign's independently replayed trace."""
import json
import subprocess
import shutil
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from v6_mujoco.visualize import VIEWS,_append_connector,_append_coordinate_frame
from v6_mujoco.model import body_id,geom_id
from v6_mujoco.postgrasp_campaign.engine import STATE_SPEC
from v6_mujoco.postgrasp_campaign.io import read,save,identity
from v6_mujoco.end_to_end_capture.common import arrays,digest
from .audit import ROOT,ordered_box_witness
from .adapter import compile_model,pads

OUT=ROOT/'visualizations';FONT=ImageFont.truetype('C:/Windows/Fonts/consola.ttf',16)
FFMPEG=shutil.which('ffmpeg') or str(next(Path('C:/Users/admin/AppData/Local/Microsoft/WinGet/Packages').glob('Gyan.FFmpeg*/ffmpeg-*/bin/ffmpeg.exe')))

def run():
    out=ROOT/'nominal';r=read(out/'metrics.json');validation=read(out/'validation.json');assert validation['passed'] and validation['trace_sha256']==digest(out/'trace.npz')
    a=arrays(out/'trace.npz');OUT.mkdir(exist_ok=True);fps=30;n=int(np.ceil(a['time_s'][-1]*fps))+1;indices=np.rint(np.linspace(0,len(a['time_s'])-1,n)).astype(int)
    m=compile_model();m.vis.global_.offwidth=800;m.vis.global_.offheight=480;d=mujoco.MjData(m);renderer=mujoco.Renderer(m,height=480,width=800);gids=pads(m);base=body_id(m,'base_link_0')
    cube=geom_id(m,'tumbling_target_geom');m.geom_matid[cube]=-1;m.geom_rgba[cube]=[.1,.65,.35,.28]
    encoders={};logs={};sizes={'overview_five_views_and_body_side':(2400,960),'interface_closeup':(800,480)}
    for key,(w,h) in sizes.items():
        logs[key]=(OUT/(key+'.ffmpeg.log')).open('wb');encoders[key]=subprocess.Popen([FFMPEG,'-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{w}x{h}','-r',str(fps),'-i','-','-an','-c:v','libx264','-threads','2','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(OUT/(key+'.partial.mp4'))],stdin=subprocess.PIPE,stderr=logs[key],stdout=subprocess.DEVNULL,creationflags=subprocess.CREATE_NO_WINDOW)
    phases={0:'APPROACH',1:'CAPTURE_CHECK',2:'LATCH',3:'GRASP_VERIFY',4:'DAMP_TRANSFER',5:'HOLD',6:'FAILED'}
    def panel(im,i,title):
        pic=Image.fromarray(im);dr=ImageDraw.Draw(pic);dr.rectangle((0,0,800,94),fill='#132739');phase=phases[int(a['phase_code'][i])]
        if i==len(a['time_s'])-1 and r['status']!='COMPLETED':phase='FAILED '+r['status']
        local=f"{a['time_s'][i]-8:.3f}s" if a['eq_active'][i,0] else 'N/A'
        lines=[f'N206 SIM TOOL +20mm | {title}',f"t={a['time_s'][i]:.3f}s | latch local {local} | {phase}",f"target {np.rad2deg(np.linalg.norm(a['target_omega_world_rad_s'][i])):.3f} deg/s | flange err {a['approach_position_error_m'][i]*1000:.4f} mm",'Recorded-input replay verified | seven joints | hardware NOT validated']
        for j,line in enumerate(lines):dr.text((10,3+22*j),line,font=FONT,fill='white' if j<2 else '#8ad5dc')
        return pic
    def trail(i):
        ids=np.linspace(max(0,i-700),i,min(36,i+1)).astype(int)
        for key,color in [('flange_position_world_m',[.9,.2,.1,1]),('approach_reference_position_world_m',[.1,.4,1,1])]:
            for left,right in zip(ids[:-1],ids[1:]):
                if a['time_s'][right]<=8+1e-9 and np.linalg.norm(a[key][right]-a[key][left])>1e-10:_append_connector(renderer.scene,a[key][left],a[key][right],.0015,np.array(color))
    try:
        for frame,i in enumerate(indices):
            mujoco.mj_setState(m,d,a['integration_state'][i],STATE_SPEC);m.geom_contype[gids],m.geom_conaffinity[gids]=a['interface_geom_masks'][i];mujoco.mj_forward(m,d)
            visible=(m.geom_bodyid>0);radii=m.geom_rbound[visible,None];lo=np.min(d.geom_xpos[visible]-radii,axis=0);hi=np.max(d.geom_xpos[visible]+radii,axis=0);center=(lo+hi)/2;dist=max(2.8,1.15*np.linalg.norm(hi-lo)/2/np.sin(np.deg2rad(m.vis.global_.fovy/2)))
            frames=[]
            for view in VIEWS:
                cam=mujoco.MjvCamera();cam.lookat[:]=center;cam.distance=dist;cam.azimuth=view.azimuth_deg;cam.elevation=view.elevation_deg;renderer.update_scene(d,camera=cam);trail(i);frames.append(panel(renderer.render(),i,view.title))
            cam=mujoco.MjvCamera();cam.lookat[:]=center;direction=d.xmat[base].reshape(3,3)@np.array([0,-1,.2]);cam.azimuth=np.rad2deg(np.arctan2(direction[1],direction[0]));cam.elevation=-np.rad2deg(np.arctan2(direction[2],np.linalg.norm(direction[:2])));cam.distance=dist
            renderer.update_scene(d,camera=cam);trail(i);frames.append(panel(renderer.render(),i,'Continuous base-body side'))
            composite=Image.new('RGB',(2400,960))
            for j,pic in enumerate(frames):composite.paste(pic,(800*(j%3),480*(j//3)))
            close=mujoco.MjvCamera();close.lookat[:]=.5*(a['tool_position_world_m'][i]+a['target_grasp_position_world_m'][i]);close.distance=max(.58,2*np.linalg.norm(a['tool_position_world_m'][i]-a['target_grasp_position_world_m'][i]));R=a['target_grasp_rotation_world'][i];direction=-R[:,2]+.7*R[:,0]+.4*R[:,1];close.azimuth=np.rad2deg(np.arctan2(direction[1],direction[0]));close.elevation=-np.rad2deg(np.arctan2(direction[2],np.linalg.norm(direction[:2])));renderer.update_scene(d,camera=close)
            seg=np.zeros(6);gap=mujoco.mj_geomDistance(m,d,geom_id(m,'link7_collision'),cube,2,seg);seg,_,_=ordered_box_witness(m,d,'tumbling_target_geom',seg)
            if np.linalg.norm(seg[3:]-seg[:3])>1e-10:_append_connector(renderer.scene,seg[:3],seg[3:],.001,np.array([1,.1,.1,1]))
            _append_coordinate_frame(renderer.scene,a['target_grasp_position_world_m'][i],R,.035,.8,.0008)
            closepic=panel(renderer.render(),i,f'Interface | link7 gap {gap*1000:.3f} mm')
            pics={'overview_five_views_and_body_side':composite,'interface_closeup':closepic}
            for key,pic in pics.items():encoders[key].stdin.write(np.asarray(pic,dtype=np.uint8).tobytes())
            if frame in [0,n//2,n-1]:
                for key,pic in pics.items():pic.save(OUT/(key+f'_{frame:03d}.png'))
            if frame%100==0:print(f'N206 media {frame+1}/{n}',flush=True)
        for key,proc in encoders.items():
            proc.stdin.close();code=proc.wait();logs[key].close()
            if code:raise RuntimeError('encoding failed '+key)
            (OUT/(key+'.partial.mp4')).replace(OUT/(key+'.mp4'))
    finally:
        for proc in encoders.values():
            if proc.poll() is None:proc.kill();proc.wait()
        renderer.close()
    videos={}
    for key,(w,h) in sizes.items():
        file=OUT/(key+'.mp4');probe=json.loads(subprocess.check_output([str(Path(FFMPEG).with_name('ffprobe.exe')),'-v','error','-select_streams','v:0','-show_entries','stream=width,height,nb_frames:format=duration','-of','json',str(file)],creationflags=subprocess.CREATE_NO_WINDOW));assert int(probe['streams'][0]['nb_frames'])==n and probe['streams'][0]['width']==w;videos[key]={'sha256':digest(file),'probe':probe}
    save(OUT/'manifest.json',{'source':'N206 nominal actual continuous physics; independent recorded-input replay passed','trace_sha256':digest(out/'trace.npz'),'model_sha256':digest(__import__('v6_mujoco.geometry_capture.design',fromlist=['NEW_MODEL']).NEW_MODEL),'render_identity':identity([__file__]),'physical_end_time_s':float(a['time_s'][-1]),'physics_steps':0,'fps':fps,'frames':n,'physical_indices':indices.tolist(),'physical_times_s':a['time_s'][indices].tolist(),'videos':videos,'views_in_overview':['isometric','front','right','top','rear','continuous_base_body_side'],'display_only':'800x480 framebuffer; target alpha/color; red actual and blue reference trails up to recorded current time','no_historical_state_splicing':True})
    print({'videos':list(videos),'physical_end_time_s':float(a['time_s'][-1])})

if __name__=='__main__':run()
