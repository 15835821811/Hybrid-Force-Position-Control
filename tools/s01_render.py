"""One optional SRS six-panel video, from the preregistered author-point trace."""
import json
import shutil
import subprocess
import sys
import time
import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from scipy.spatial.transform import Rotation
from v6_mujoco.system_capture.paper_bridge.common import OUT,PROJECT,read,save,sha
from v6_mujoco.system_capture.paper_bridge.model import MODEL
from v6_mujoco.visualize import VIEWS,_append_connector
from v6_mujoco.feasible_capture.render import child_cpu

def run():
    start=time.perf_counter();cpu=time.process_time();key=read(OUT/'author_point_check.json')['run_keys']['source']
    folder=OUT/'dynamics_runs'/key;metrics=read(folder/'metrics.json')
    with np.load(folder/'trace.npz',allow_pickle=False) as z:a={k:z[k] for k in z.files}
    dest=OUT/'visualizations';dest.mkdir(parents=True,exist_ok=True)
    tree=ET.fromstring(MODEL.read_text(encoding='utf-8'))
    asset=ET.SubElement(tree,'asset')
    ET.SubElement(asset,'texture',type='skybox',builtin='gradient',rgb1='.98 .98 .98',rgb2='.86 .90 .93',width='512',height='3072')
    render_xml=ET.tostring(tree,encoding='unicode')
    m=mujoco.MjModel.from_xml_string(render_xml);d=mujoco.MjData(m);W,H,FPS=960,600,30
    m.vis.headlight.ambient[:]=[.6,.6,.6];m.vis.headlight.diffuse[:]=[.7,.7,.7]
    m.geom_rgba[:,3]=1
    m.vis.global_.offwidth=W;m.vis.global_.offheight=H;renderer=mujoco.Renderer(m,height=H,width=W)
    times=a['time_s'];n=int(np.ceil(times[-1]*FPS))+1;pt=np.minimum(np.arange(n)/FPS,times[-1])
    right=np.minimum(np.searchsorted(times,pt),len(times)-1);left=np.maximum(0,right-1)
    ids=np.where(abs(times[right]-pt)<abs(times[left]-pt),right,left);ids[-1]=len(times)-1
    lo=np.full(3,np.inf);hi=-lo.copy()
    for i in ids:
        d.qpos[:]=a['qpos'][i];d.qvel[:]=a['qvel'][i];mujoco.mj_forward(m,d)
        lo=np.minimum(lo,np.min(d.geom_xpos-m.geom_rbound[:,None],axis=0))
        hi=np.maximum(hi,np.max(d.geom_xpos+m.geom_rbound[:,None],axis=0))
    center=(lo+hi)/2;distance=max(1.5,np.linalg.norm(hi-lo)*.65/np.sin(np.deg2rad(m.vis.global_.fovy/2)))
    font=ImageFont.truetype('C:/Windows/Fonts/consola.ttf',18)
    outfile=dest/'srs_author_point_five_views_and_body_side.mp4';partial=outfile.with_suffix('.partial.mp4')
    log=(dest/'encoding.log').open('wb')
    enc=subprocess.Popen([shutil.which('ffmpeg'),'-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',str(3*W)+'x'+str(2*H),'-r',str(FPS),'-i','-','-an','-c:v','libx264','-threads','1','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(partial)],stdin=subprocess.PIPE,stderr=log,stdout=subprocess.DEVNULL,creationflags=subprocess.CREATE_NO_WINDOW)
    cameras=[]
    def trail(i):
        ix=np.unique(np.linspace(0,i,min(65,i+1)).astype(int))
        for field,color in [('flange_position_world_m',[.9,.2,.1,1]),('reference_position_world_m',[.1,.4,1,1])]:
            for l,r in zip(ix[:-1],ix[1:]):
                if np.linalg.norm(a[field][r]-a[field][l])>1e-10:_append_connector(renderer.scene,a[field][l],a[field][r],.002,np.array(color))
    try:
        for frame,i in enumerate(ids):
            d.qpos[:]=a['qpos'][i];d.qvel[:]=a['qvel'][i];d.time=times[i];mujoco.mj_forward(m,d)
            panels=[]
            for v in list(VIEWS)+[None]:
                camera=mujoco.MjvCamera();camera.lookat[:]=center;camera.distance=distance
                if v is not None:camera.azimuth=v.azimuth_deg;camera.elevation=v.elevation_deg
                renderer.update_scene(d,camera=camera)
                if v is None:
                    R=Rotation.from_quat(a['qpos'][i,[4,5,6,3]]).as_matrix()
                    fw=R@np.array([0.,1.,-.2]);fw/=np.linalg.norm(fw);up=R@np.array([0.,.2,1.]);up/=np.linalg.norm(up)
                    for eye in renderer.scene.camera:eye.pos[:]=center-distance*fw;eye.forward[:]=fw;eye.up[:]=up
                    cameras.append(fw)
                trail(i);pic=Image.fromarray(renderer.render());draw=ImageDraw.Draw(pic);draw.rectangle((0,0,W,125),fill='#132739')
                title=v.title if v is not None else 'Continuous base-body side'
                lines=['S01 SRS | author point / source | '+title,
                  't='+format(times[i],'.3f')+' s / '+format(metrics['T_s'],'.3f')+' s requested',
                  'position error '+format(np.linalg.norm(a['position_error_m'][i])*1000,'.4f')+' mm | precontact only',
                  'red: actual flange | blue: reference | schematic geometry',
                  'FINAL: '+metrics['status'] if frame==n-1 else 'Recorded dynamics display; no contact or state-advance step']
                for j,line in enumerate(lines):draw.text((10,3+24*j),line,font=font,fill='white' if j<2 else '#9bdddf')
                panels.append(pic)
            composite=Image.new('RGB',(W*3,H*2))
            for j,panel in enumerate(panels):composite.paste(panel,(W*(j%3),H*(j//3)))
            enc.stdin.write(np.asarray(composite).tobytes())
            if frame in [0,n//2,n-1]:composite.save(dest/('srs_frame_'+str(frame)+'.png'))
            if frame==n-1:composite.save(dest/'srs_last.png')
            if frame%60==0:print('S01 render '+str(frame+1)+'/'+str(n),flush=True)
        enc.stdin.close();enc.wait()
        encoder_cpu=child_cpu(enc)
        if enc.returncode:raise RuntimeError('ffmpeg failed; see encoding.log')
        partial.replace(outfile)
    finally:
        renderer.close();log.close()
        if enc.poll() is None:enc.terminate();enc.wait()
    probe=json.loads(subprocess.check_output([shutil.which('ffprobe'),'-v','error','-threads','1','-count_frames','-select_streams','v:0','-show_entries','stream=width,height,nb_read_frames,codec_name','-of','json',str(outfile)],creationflags=subprocess.CREATE_NO_WINDOW))
    assert int(probe['streams'][0]['nb_read_frames'])==n
    c=np.asarray(cameras);angle=np.arccos(np.clip(np.sum(c[1:]*c[:-1],axis=1),-1,1))
    save(dest/'video_manifest.json',{'run':key,'selection_rule':'Preregistered author reported point, source controller; independent of outcome','result':metrics['status'],
      'trace_sha256':sha(folder/'trace.npz'),'video':outfile.name,'video_sha256':sha(outfile),'frames':n,'fps':FPS,'first_recorded_time_s':float(times[ids[0]]),'last_recorded_time_s':float(times[ids[-1]]),
      'requested_horizon_s':metrics['T_s'],'display_only':True,'physics_steps':0,'new_physical_attempts':0,'continuous_body_camera_max_adjacent_angle_rad':float(angle.max()),
      'display_changes':'Gradient skybox, headlight and opaque schematic geometry only; frozen dynamic model unchanged','model_sha256':sha(MODEL),'render_xml_sha256':hashlib.sha256(render_xml.encode()).hexdigest(),
      'probe':probe,'frame_indices':ids.tolist(),'generator_sha256':sha(Path(__file__)),'parent_cpu_s':time.process_time()-cpu,'encoder_cpu_s':encoder_cpu,'wall_s':time.perf_counter()-start})
    (dest/'README.md').write_text('# S01 作者报告点视频\n\n[五个固定视角与连续体侧合成视频]('+outfile.name+')\n\n![实际停止帧](srs_last.png)\n\n运行 '+key+'，'+metrics['status']+'。仅到实际停止时刻 '+format(times[-1],'.3f')+' s；要求时域 '+str(metrics['T_s'])+' s 未完成。中心线为示意，视频不代表接触或完整捕获。全部画面来自同一真实积分记录；显示过程不推进动力学。连续体侧指随基座连续旋转的镜头。\n',encoding='utf-8')
    print({'S01_video_complete':True,'frames':n},flush=True)

if __name__=='__main__':run()
