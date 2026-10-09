"""Reproducible N206 geometry sections and actual-horizon scientific figures."""
from pathlib import Path
import copy
import numpy as np
import mujoco
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull
from v6_mujoco.model import geom_id,body_id
from v6_mujoco.postgrasp_campaign.io import read,save,identity
from v6_mujoco.end_to_end_capture.common import arrays,digest
from v6_mujoco.end_to_end_capture.adapter import compile_model as old_model
from .adapter import compile_model,extra_pairs
from .audit import ROOT,OLD,ideal,bodyT,world_vertices,ordered_box_witness

OUT=ROOT/'figures';COLORS=['#0072B2','#D55E00','#009E73','#CC79A7','#E69F00','#56B4E9','#222222']
plt.rcParams.update({'font.family':'serif','font.serif':['DejaVu Serif'],'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'savefig.bbox':'tight'})
CAPTIONS={}
WHITEBOX={'facecolor':'white','edgecolor':'none','alpha':1,'pad':2}
def finish(fig,name,caption):
    OUT.mkdir(exist_ok=True)
    for ext in ['png','pdf']:fig.savefig(OUT/(name+'.'+ext),dpi=220,bbox_inches='tight')
    plt.close(fig);CAPTIONS[name]=caption

def section_segments(vertices,faces,z=0):
    lines=[]
    for tri in vertices[faces]:
        points=[]
        for a,b in zip(tri,np.roll(tri,-1,axis=0)):
            if (a[2]-z)*(b[2]-z)<0:points.append(a+(b-a)*(z-a[2])/(b[2]-a[2]))
            if abs(a[2]-z)<1e-12:points.append(a)
        if len(points)>=2:lines.append(np.array(points[:2]))
    return lines

def target_vertices(m,d,name):
    T=bodyT(m,d,'tumbling_target');return (world_vertices(m,d,name)-T[:3,3])@T[:3,:3]

def geometry():
    old=old_model();new=compile_model();od=ideal(old);nd=ideal(new)
    fig,axs=plt.subplots(1,2,figsize=(12,4.6),sharex=True,sharey=True,constrained_layout=True)
    for ax,m,d,label in zip(axs,[old,new],[od,nd],['(a) Original fixed mating','(b) +20 mm simulation tool candidate']):
        v=target_vertices(m,d,'link7_collision');g=geom_id(m,'link7_collision');mid=m.geom_dataid[g];f=m.mesh_face[m.mesh_faceadr[mid]:m.mesh_faceadr[mid]+m.mesh_facenum[mid]]
        def proj(x):return np.c_[x[:,0]*1000,-(x[:,1]+.15)*1000]
        ax.add_collection(LineCollection([proj(s) for s in section_segments(v,f)],colors=COLORS[0],lw=1.8,label='STL surface section'))
        ax.add_collection(LineCollection([proj(s) for s in section_segments(v,ConvexHull(v).simplices)],colors=COLORS[1],lw=.8,linestyles='--',label='Collision hull section'))
        ax.axhspan(-5,0,facecolor=COLORS[2],alpha=.2,label='Target solid (below face)');ax.plot([-50,50],[0,0],color=COLORS[2],lw=2,label='Plate front')
        T=bodyT(m,d,'tumbling_target')
        for name,col in [('gripper_contact_pad',COLORS[4]),('n206_spacer',COLORS[5])]:
            g=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,name)
            if g<0:continue
            # Normal-aligned tool section; frozen interface tilt <0.002 deg.
            R=d.geom_xmat[g].reshape(3,3);p=d.geom_xpos[g];rad,h=m.geom_size[g,:2];local=np.array([[-rad,0,-h],[rad,0,-h],[rad,0,h],[-rad,0,h],[-rad,0,-h]])
            q=proj((local@R.T+p-T[:3,3])@T[:3,:3]);ax.fill(q[:,0],q[:,1],color=col,alpha=.55,label=name.replace('_',' '))
        ax.text(.02,.96,label,transform=ax.transAxes,va='top',bbox={'facecolor':'white','edgecolor':'none','alpha':.85});ax.set(xlim=(-60,60),ylim=(-4,45),xlabel='Target x [mm]');ax.grid(alpha=.15)
    axs[0].set_ylabel('Outward distance from target face [mm]');axs[1].legend(fontsize=8,loc='upper left',bbox_to_anchor=(1.01,1.))
    inset=axs[0].inset_axes([.31,.44,.4,.28]);v=target_vertices(old,od,'link7_collision');g=geom_id(old,'link7_collision');mid=old.geom_dataid[g];f=old.mesh_face[old.mesh_faceadr[mid]:old.mesh_faceadr[mid]+old.mesh_facenum[mid]]
    for faces,color,style in [(f,COLORS[0],'-'),(ConvexHull(v).simplices,COLORS[1],'--')]:inset.add_collection(LineCollection([np.c_[s[:,0]*1000,-(s[:,1]+.15)*1000] for s in section_segments(v,faces)],colors=color,linestyles=style,lw=1.2))
    inset.axhspan(-.7,0,color=COLORS[2],alpha=.2);inset.axhline(0,color=COLORS[2]);inset.set(xlim=(-15,15),ylim=(-.6,.3));inset.tick_params(labelsize=7);inset.text(.02,1.04,'Depth enlarged; coordinates [mm]',transform=inset.transAxes,fontsize=7);inset.text(.04,.08,'3-D gap: -0.292937 mm',transform=inset.transAxes,fontsize=7)
    finish(fig,'terminal_sections','Ideal mating in the target frame; STL and convex-hull sections at target z=0. Tool solids projected into that plane (interface tilt is under 0.002 deg). Original global 3-D link7 distance -0.292936604 mm; 20 mm candidate gives about +19.707063 mm. These query distances are not inferred from the 2-D section. Inset magnifies the depth coordinate. New 9.8 mm spacer and 10 mm pad use a rigid circular-seat simulation assumption, not approved hardware. Old/new main panels use identical axes and original STL scale.')
    # Actual failed N205 geometry, closest pair at positive gap.
    a=arrays(OLD/'E_nominal/trace.npz');d=mujoco.MjData(old);d.qpos[:]=a['qpos'][-1];d.qvel[:]=a['qvel'][-1];mujoco.mj_forward(old,d)
    g=geom_id(old,'link7_collision');mid=old.geom_dataid[g];v=target_vertices(old,d,'link7_collision');f=old.mesh_face[old.mesh_faceadr[mid]:old.mesh_faceadr[mid]+old.mesh_facenum[mid]]
    box=geom_id(old,'tumbling_target_geom');seg=np.zeros(6);gap=mujoco.mj_geomDistance(old,d,g,box,2,seg);seg,_,_=ordered_box_witness(old,d,'tumbling_target_geom',seg);T=bodyT(old,d,'tumbling_target');s=(seg.reshape(2,3)-T[:3,3])@T[:3,:3]
    fig=plt.figure(figsize=(12,5.2));ax=fig.add_subplot(121,projection='3d');display=np.c_[v[:,0]*1000,v[:,2]*1000,-(v[:,1]+.15)*1000]
    ax.add_collection3d(Poly3DCollection(display[f],facecolor=COLORS[0],edgecolor=COLORS[0],alpha=.13,linewidth=.3))
    ax.add_collection3d(Poly3DCollection([np.array([[-60,-60,0],[60,-60,0],[60,60,0],[-60,60,0]])],facecolor=COLORS[2],alpha=.2))
    sp=np.c_[s[:,0]*1000,s[:,2]*1000,-(s[:,1]+.15)*1000];ax.plot(*sp.T,color=COLORS[1],lw=3,marker='o');ax.set(xlim=(-65,65),ylim=(-65,65),zlim=(-3,80),xlabel='Target x [mm]',ylabel='Target z [mm]',zlabel='Outward [mm]');ax.view_init(22,-65)
    ax.text2D(.02,.97,'N205 actual t=7.362 s — local view',transform=ax.transAxes,fontsize=9,bbox=WHITEBOX)
    inset=ax.inset_axes([.06,.60,.60,.26]);inset.axhspan(-.2,0,color=COLORS[2],alpha=.2);inset.plot([0,0],sp[:,2],color=COLORS[1],lw=2);inset.scatter([0],[sp[0,2]],marker='^',color=COLORS[1]);inset.scatter([0],[sp[1,2]],marker='s',color=COLORS[2]);inset.text(.15,sp[0,2],'link7 hull witness',fontsize=7,va='center');inset.text(.15,sp[1,2],'cube witness',fontsize=7,va='center');inset.set(xlim=(-.2,2),ylim=(-.2,1.3),xticks=[],ylabel='Outward [mm]');inset.tick_params(labelsize=7);inset.text(.02,1.04,f'Closest segment: +{gap*1000:.6f} mm',transform=inset.transAxes,fontsize=8,bbox=WHITEBOX)
    bx=fig.add_subplot(122);audit=read(ROOT/'geometry_audit.json');labels=['N205\n7.360 s','N205\n7.362 s','Historical C1\n8 s','Ideal fixed\nmating'];values=[x['pairs'][-3]['signed_distance_m']*1000 for x in audit['states']]
    bx.bar(labels,values,color=[COLORS[0],COLORS[0],COLORS[1],COLORS[1]]);bx.axhline(.99,color='black',ls=':',label='Frozen stop threshold 0.99 mm');bx.axhline(0,color='black',lw=.7)
    for i,val in enumerate(values):bx.text(i,val+(.04 if val>=0 else -.07),f'{val:.6f}',ha='center',va='bottom' if val>=0 else 'top',fontsize=9)
    bx.set(ylabel='Signed link7–cube distance [mm]',ylim=(-.65,1.35));bx.legend(loc='upper right',fontsize=8);fig.subplots_adjust(wspace=.35,bottom=.15)
    finish(fig,'failure_witness','Left: N205 actual 7.362 s saved state; supplied STL surface triangles, target face and geometrically labelled closest points. Gap is positive +0.989619610 mm; no collision asserted. Right: actual N205 samples and separate historical/ideal static queries. Historical and ideal negative distances also have 29 triangle/box crossing witnesses; this is not solely convex-hull filling. Geometry is supplied collision STL, not certified CAD.')

def planning():
    screen=read(ROOT/'trajectory_screening.json');fig,axs=plt.subplots(3,1,figsize=(10,8),sharex=True,constrained_layout=True)
    for n,r in enumerate(screen['candidates']):
        a=arrays(ROOT/'planning'/r['name']/'kinematic_trace.npz');j=[x['pair'] for x in r['pair_minima']].index('link7_collision__tumbling_target_geom');t=a['time_s']
        axs[0].plot(t,a['distance_m'][:,j]*1000,label=r['name'],color=COLORS[n]);axs[1].plot(t,a['position_error_m']*1000,color=COLORS[n]);axs[2].plot(t,np.rad2deg(a['shape_error_rad']),color=COLORS[n])
    axs[0].axhline(2,color='black',ls='--',label='Planning margin 2 mm');axs[0].legend(ncol=2,fontsize=8);axs[0].set_ylabel('Link7–cube gap [mm]');axs[1].set_ylabel('Kinematic position error [mm]');axs[2].set(ylabel='Arm-shape error [deg]',xlabel='Planning time [s]')
    finish(fig,'planning_screening','Three frozen references, all sampled over 0–8 s with added target/tool pairs in hard distance constraints. These are kinematic planning integrations, not true closed-loop dynamics. Same-path adaptive interval checks use 0.5 ms for standard C1 and 0.25 ms for other candidates near pad contact; no arbitrary perturbation guarantee. Standard C1 selected without reading postgrasp results.')
    fig,axs=plt.subplots(4,1,figsize=(10,10),sharex=True,constrained_layout=True)
    for n,r in enumerate(screen['candidates']):
        a=arrays(ROOT/'planning'/r['name']/'kinematic_trace.npz');t=a['time_s'];rows=r['pair_minima'];old=[j for j,x in enumerate(rows) if j<24];target=[j for j,x in enumerate(rows) if j>=24 and 'gripper_contact_pad__tumbling' not in x['pair'] and ('tumbling_target_geom' in x['pair'] or 'target_contact_plate' in x['pair'])];pad=[j for j,x in enumerate(rows) if x['pair']=='gripper_contact_pad__tumbling_target_geom'][0]
        toolrobot=[j for j,x in enumerate(rows) if j>=24 and j not in target and j!=pad]
        for ax,idx in [(axs[0],old),(axs[1],target),(axs[3],toolrobot)]:ax.plot(t,np.min(a['distance_m'][:,idx],axis=1)*1000,label=r['name'],color=COLORS[n],ls=['-','--',':'][n])
        axs[2].plot(t,a['distance_m'][:,pad]*1000,color=COLORS[n],ls=['-','--',':'][n])
    for ax,planning_gate,actual_gate,label in zip(axs,[45,2,-2,45],[40,1,-2,40],['Original min [mm]','Target min [mm]','Intended pad–cube [mm]','Tool–robot min [mm]']):
        ax.axhline(planning_gate,color='black',ls='--');ax.axhline(actual_gate,color='gray',ls=':');ax.set_ylabel(label);ax.text(.02,.16,f'Planning {planning_gate:g} / execution {actual_gate:g} mm',transform=ax.transAxes,fontsize=8,bbox=WHITEBOX)
    axs[0].legend(fontsize=8);axs[3].set_xlabel('Planning time [s]')
    finish(fig,'planning_pair_classes','Pointwise minimum within each pair class for all frozen kinematic planning candidates: original 24 pairs, unintended target pairs, intended pad–cube and added tool–robot/obstacle pairs. No real closed-loop dynamics shown. Dashed black lines: planning gates; dotted grey: unchanged execution gates before numeric tolerance. Candidate curves overlap before 6 s. Per-pair conservative interpolation bounds and adaptive checks are in trajectory_screening.json. Stated tool simulation assumption only, not hardware verification.')
    fig,ax=plt.subplots(figsize=(8,4.8));dims=read(ROOT/'selected_design.json')['parameters'];L=dims['spacer_length_m']*1000;end=dims['pad_front_z_m']*1000;r=dims['radius_m']*1000
    ax.fill([0,L,L,0],[-r,-r,r,r],color=COLORS[5],alpha=.5,label='Rigid spacer');ax.fill([L,end,end,L],[-r,-r,r,r],color=COLORS[4],alpha=.6,label='10 mm pad');ax.axvline(0,color='black',lw=2,label='Flange seat z=0');ax.axvline(-.2,color='gray',ls=':',label='Original tool face z=-0.2 mm')
    E=np.array(read(ROOT/'selected_design.json')['T_flange_tool']);ax.scatter(E[2,3]*1000,E[0,3]*1000,color=COLORS[3],marker='x',s=70,label='Frozen translated weld site')
    for a,b,y,label in [(0,L,42,'9.8 mm spacer'),(L,end,50,'10 mm pad'),(-.2,end,-44,'20 mm installation increment')]:
        ax.annotate('',(a,y),(b,y),arrowprops={'arrowstyle':'<->','color':'black'});ax.text((a+b)/2,y+2,label,ha='center',fontsize=9)
    ax.set(xlim=(-8,30),ylim=(-53,61),xlabel='Flange local z [mm] (+z toward target)',ylabel='Flange local x [mm]');ax.legend(loc='center left',bbox_to_anchor=(1.01,.5),fontsize=8)
    ax.text(.02,.96,'Dimension schematic: horizontal / vertical scales differ',transform=ax.transAxes,fontsize=8,va='top',bbox=WHITEBOX);ax.text(-.2,-49,'Old face -0.200 mm',fontsize=8,ha='center',bbox=WHITEBOX);ax.text(end,-49,'New face 19.800 mm',fontsize=8,ha='center');ax.text(end+1,7,f'Site z={E[2,3]*1000:.6f} mm',fontsize=8,rotation=90)
    finish(fig,'tool_dimension_drawing','Simulation dimension drawing in the flange frame. Radius 35 mm; 9.8 mm positive-length spacer contiguous with the 10 mm pad; 20 mm is installation increment, not spacer thickness. Added aluminium-equivalent density 2700 kg/m³, total mass 0.205738 kg, explicit positive inertia. Entire circular rigid seat assumed; no bolt pattern, manufacturing CAD, strength or hardware approval. Target geometry and robot flange/link7 unchanged.')

def actual():
    if not (ROOT/'nominal/trace.npz').exists():return
    a=arrays(ROOT/'nominal/trace.npz');r=read(ROOT/'nominal/metrics.json');validation=read(ROOT/'nominal/validation.json');assert validation['passed'];t=a['time_s'];end=float(t[-1]);post=a['holding_error_applicable'].astype(bool);valid='[16,18] s evaluated' if r['full_window_evaluated'] else '[16,18] s NOT EVALUATED'
    fig,axs=plt.subplots(3,1,figsize=(10,8),sharex=True,constrained_layout=True)
    for key,label in [('target_omega_world_rad_s','Target world'),('base_omega_world_rad_s','Base world'),('target_base_relative_omega_world_rad_s','Target relative to base')]:axs[0].plot(t,np.rad2deg(np.linalg.norm(a[key],axis=1)),label=label)
    axs[0].axhline(.1,color='gray',ls='--');axs[0].axhline(.02,color='gray',ls=':');axs[0].set_ylabel('Angular speed [deg/s]');axs[0].legend(fontsize=8)
    axs[1].plot(t,a['load_fraction']);axs[1].axhline(1,color=COLORS[1],ls='--');axs[1].set_ylabel('Actual interface load fraction')
    axs[2].plot(t,a['contact_peak_force_n']);axs[2].set(ylabel='Pad–plate force [N]',xlabel='Absolute physical time [s]')
    finish(fig,'angular_speed_load_contact',f'N206 nominal actual continuous dynamics through {end:.3f} s. {valid}. Angular speed gates are fixed at 0.1 and 0.02 deg/s. Load fraction uses the actual physical wrench and original 50 N / 2 Nm envelope; no old postgrasp curves inherited.')
    mask=t<=8+1e-9;fig=plt.figure(figsize=(12,7));gs=fig.add_gridspec(3,2,wspace=.35,hspace=.4);ax=fig.add_subplot(gs[:,0],projection='3d')
    for key,label,style in [('flange_position_world_m','Actual flange','-'),('approach_reference_position_world_m','Geometry-derived reference','--')]:ax.plot(*a[key][mask].T,style,label=label)
    ax.set(xlabel='World x [m]',ylabel='World y [m]',zlabel='World z [m]');ax.legend(fontsize=8)
    for j,(key,factor,label) in enumerate([('approach_position_error_m',1000,'Flange error [mm]'),('approach_rotation_error_rad',180/np.pi,'SO(3) error [deg]'),('approach_shape_error_rad',180/np.pi,'Shape error [deg]')]):
        ax=fig.add_subplot(gs[j,1]);ax.plot(t[mask],factor*a[key][mask]);ax.set_ylabel(label)
        if j==2:ax.set_xlabel('Absolute physical time [s]')
    finish(fig,'end_effector_tracking',f'N206 actual flange path and approach tracking errors from home through min(8,{end:.3f}) s. Reference comes from new fixed flange–tool and target–grasp transforms; it does not fit an old actual terminal state. These errors are distinct from postgrasp holding errors.')
    fig,axs=plt.subplots(3,1,figsize=(10,8),sharex=True,constrained_layout=True)
    if post.any():
        axs[0].plot(t[post],a['interface_translation_error_m'][post]*1000);axs[1].plot(t[post],a['interface_rotation_error_deg'][post]);axs[2].plot(t[post],np.linalg.norm(a['force_grasp_n'][post],axis=1),label='Force [N]');axs[2].plot(t[post],np.linalg.norm(a['moment_grasp_nm'][post],axis=1),label='Moment [Nm]');axs[2].legend()
    else:
        for ax in axs:ax.set_xlim(8,18);ax.text(.5,.65,'No latched samples — NOT EVALUATED',ha='center',transform=ax.transAxes,bbox=WHITEBOX)
    axs[0].axhline(.5,color=COLORS[1],ls='--');axs[1].axhline(.1,color=COLORS[1],ls='--');axs[0].set_ylabel('Holding translation [mm]');axs[1].set_ylabel('Physical SO(3) [deg]');axs[2].set(ylabel='Physical interface wrench',xlabel='Absolute physical time [s]')
    finish(fig,'interface_holding',f'Only actual latched samples are plotted; nominal ended at {end:.3f} s. No holding curves fabricated for an unexecuted stage. Original 0.5 mm / 0.1 deg limits and physical wrench reference remain fixed.')
    fig,axs=plt.subplots(3,1,figsize=(10,8),sharex=True,constrained_layout=True)
    for j,(key,gate,label) in enumerate([('linear_momentum_world_kg_m_s',1e-4,'P drift [kg m/s]'),('angular_momentum_about_center_world_kg_m2_s',1e-5,'H drift [kg m²/s]')]):
        axs[j].plot(t,np.linalg.norm(a[key]-a[key][0],axis=1));axs[j].axhline(gate,color=COLORS[1],ls='--');axs[j].set_yscale('symlog',linthresh=1e-14);axs[j].set_ylabel(label)
    for key in ['actuator_power_w','passive_power_w','all_constraint_power_w']:axs[2].plot(t,a[key],label=key.replace('_',' '))
    axs[2].set(ylabel='Power [W]',xlabel='Absolute physical time [s]');axs[2].legend(fontsize=8)
    finish(fig,'momentum_power','Whole physical robot + target system, referenced to actual t=0. The kinematically prescribed visualization marker is excluded from COM and locked-inertia accounting without altering the MuJoCo plant. Actuator, passive and constraint powers remain physical quantities.')
    fig,axs=plt.subplots(3,1,figsize=(11,8),sharex=True,constrained_layout=True)
    for i in range(7):
        axs[0].plot(t,np.rad2deg(a['joint_position_rad'][:,i]),label=f'J{i+1}',color=COLORS[i]);axs[1].plot(t,a['joint_velocity_rad_s'][:,i],color=COLORS[i]);axs[2].plot(t,a['actuator_force_nm'][:,i],color=COLORS[i])
    axs[0].legend(ncol=7,fontsize=8);axs[0].set_ylabel('Joint angle [deg]');axs[1].set_ylabel('Joint velocity [rad/s]');axs[2].set(ylabel='Actuator torque [Nm]',xlabel='Absolute physical time [s]')
    finish(fig,'joints_and_torque','All seven driven joints during N206 actual nominal dynamics. Torque is measured actuator force in joint units; individual original limits are checked numerically in validation, with no retuning.')
    m=compile_model();pairs=extra_pairs(m);fig,axs=plt.subplots(2,1,figsize=(10,7),sharex=True,constrained_layout=True)
    for j,p in enumerate(pairs):
        if p.category=='target_distance':axs[0].plot(t,a['extra_pair_distances_m'][:,j]*1000,alpha=.9 if 'link7_' in p.name or 'spacer' in p.name else .2,label=p.name if 'link7_' in p.name or 'spacer' in p.name else None)
    axs[0].axhline(1,color='black',ls='--',label='Actual gate 1 mm (10 µm tolerance)');axs[0].set(ylabel='Non-intended target gap [mm]',ylim=(0,150));axs[0].legend(fontsize=7,ncol=2)
    axs[1].plot(t,a['minimum_noncontact_clearance_m']*1000);axs[1].axhline(40,color=COLORS[1],ls='--');axs[1].set(ylabel='Original 24-pair minimum [mm]',xlabel='Absolute physical time [s]')
    finish(fig,'distance_safety',f'Actual N206 distances through {end:.3f} s. Link7 and spacer remain monitored during every phase. Original 24 pairs keep 40 mm execution / 45 mm planning margins; new target pairs use 1 mm execution / 2 mm planning. Pad–cube intended-face exception is recorded separately, never applied to link7 or spacer.')
    if (ROOT/'capture_failure_analysis.json').exists():
        failure=read(ROOT/'capture_failure_analysis.json');fig,axs=plt.subplots(1,2,figsize=(12,4.8),constrained_layout=True);fields=['translation_m','rotation_deg','relative_linear_speed_m_s','relative_angular_speed_deg_s'];ratio=[failure['actual'][k]/failure['gates'][k] for k in fields]
        axs[0].bar(['Translation','Rotation','Linear speed','Angular speed'],ratio,color=COLORS[1]);axs[0].axhline(1,color='black',ls='--');axs[0].set_ylabel('Actual / frozen capture limit');axs[0].tick_params(axis='x',rotation=20)
        for i,value in enumerate(ratio):axs[0].text(i,value+.04,f'{value:.3f}',ha='center')
        mask=t>=7.5;axs[1].plot(t[mask],a['target_prediction_position_error_m'][mask]*1000,label='Target center error [mm]');axs[1].plot(t[mask],np.rad2deg(a['target_prediction_rotation_error_rad'][mask]),label='Target rotation error [deg]');axs[1].axvline(failure['first_intended_contact_s'],color='gray',ls=':',label='First pad contact');axs[1].set(xlabel='Absolute physical time [s]',ylabel='Actual target vs prescribed reference');axs[1].legend(fontsize=8)
        finish(fig,'capture_gate_failure','Actual t=8 capture metrics normalized by unchanged limits: 0.1 mm, 0.05 deg, 1 mm/s, 0.2 deg/s. All four fail and latch remains inactive. Right: real target divergence after first detected intended contact; mixed units explicitly labelled. Temporal sequence is evidence from the one actual trace, not a counterfactual causal experiment. No new reference or controller tuning followed.')
    fig,ax=plt.subplots(figsize=(10,3.6));case_names=['nominal','fine','light','heavy'];ax.axvspan(16,18,color=COLORS[2],alpha=.12,label='Fixed evaluation window');ax.axvline(8,color='gray',ls='--',label='Fixed capture time')
    for i,name in enumerate(case_names):
        file=ROOT/name/'metrics.json'
        if file.exists():
            rr=read(file);ax.barh(i,rr['end_time_s'],height=.5,color=COLORS[0]);ax.text(rr['end_time_s']+.2,i,rr['status'],va='center',fontsize=8)
        else:ax.text(.15,i,'NOT RUN — nominal failure',va='center',fontsize=9)
    ax.set(yticks=range(4),yticklabels=['E_'+x+'_v2' for x in case_names],xlim=(0,18),xlabel='Actual physical coverage [s]');ax.set_ylim(3.5,-.5);ax.legend(loc='lower right',fontsize=8)
    finish(fig,'run_coverage','Only actual integrated duration is filled. Unexecuted fine/light/heavy cases are labelled NOT RUN and are not represented as zero-valued measurements. Capture time stays 8 s and final evaluation window stays [16,18] s, even when the nominal trajectory stops.')

def run():
    geometry();planning();actual();save(OUT/'captions.json',CAPTIONS)
    (OUT/'latex_includes.tex').write_text('\n\n'.join('\\begin{figure}[t]\n\\centering\n\\includegraphics[width=\\linewidth]{'+name+'.pdf}\n\\caption{'+caption.replace('_','\\_')+'}\n\\end{figure}' for name,caption in CAPTIONS.items()),encoding='utf-8')
    save(OUT/'manifest.json',{'generator_identity':identity([__file__]),'geometry_source_identity':identity([ROOT/'geometry_audit.json',ROOT/'selected_design.json',ROOT/'trajectory_screening.json']),'actual_trace_sha256':digest(ROOT/'nominal/trace.npz') if (ROOT/'nominal/trace.npz').exists() else None,'files':identity(list(OUT.glob('*.png'))+list(OUT.glob('*.pdf')))})
    print({'figures':list(CAPTIONS)})

if __name__=='__main__':run()
