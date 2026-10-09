"""Reproducible N208 scientific panels; --figure selects one independent figure."""
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from .common import ROOT,read,save,digest

CAPTIONS={
 'campaign_identification':'Independent target cases: final parameter errors and data rank for prior/identified control modes. COM error is Euclidean distance; tensor error is relative Frobenius error of COM-centered inertia. Dashed lines are preregistered accuracy targets; full rank does not guarantee those errors pass.',
 'trajectory':'World XY/XZ projections of actual flange and live estimated-target reference, only while approach reference is applicable. Equal spatial scales; solid actual, dashed reference.',
 'tracking':'Actual flange versus live estimated-target reference during approach; reference is inactive after latch.',
 'state_estimation':'Geometric-frame state estimation errors under the declared simulation sensor. Shading shows the maximum of three marginal 3-sigma model bounds, not a certified safety probability.',
 'capture_and_load':'Independent true capture quantities and physical interface load. Capture gates are unchanged; event latch is marked.',
 'detumbling':'Target world and target-base relative angular speeds, with the fixed post-latch evaluation window. A weld alone does not establish detumbling.',
 'momentum_energy':'Separate SI linear/angular momentum drift and kinetic/relative energy. Numerical residuals are not subtracted from actual momentum.',
 'joints':'Seven measured joint positions, velocities and applied motor torques throughout the actual run.',
 'geometry':'Minimum monitored clearances using the fixed N206 collision geometry. Intended contact and other target pairs retain separate gates.',
 'identification':'Causal inertial posterior and data rank. Truth overlays are evaluation-only; parameter identification and task success remain distinct.'}

def run(name,only=None):
    out=ROOT/'runs'/name;metrics=read(out/'metrics.json');validation=read(out/'validation.json');assert validation['actuator_replay_passed'] and validation['trace_sha256']==digest(out/'trace.npz')
    with np.load(out/'trace.npz') as z:a={k:z[k] for k in z.files}
    t=a['time_s'];latch=metrics['latch_time_s'];figdir=ROOT/'visualizations';figdir.mkdir(exist_ok=True);cfg=read(out/'config.json')
    manifest=figdir/'figure_manifest.json'
    if only and manifest.exists():assert all(x['trace_sha256']==digest(out/'trace.npz') for x in read(manifest).values()),'cannot mix runs in incremental figure refresh'
    plt.rcParams.update({'font.family':'serif','font.serif':['Times New Roman','DejaVu Serif'],'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'savefig.dpi':300,'mathtext.fontset':'stix'})
    entries={}
    for key in CAPTIONS:
        if only and key!=only:continue
        n=6 if key in ['capture_and_load','identification'] else 4 if key in ['state_estimation','momentum_energy','campaign_identification'] else 3 if key=='joints' else 2
        if key=='trajectory':fig,axes=plt.subplots(1,2,figsize=(7.1,4.3),layout='constrained')
        else:fig,axes=plt.subplots(n,1,figsize=(7.1,2.0*n),sharex=True,layout='constrained')
        axes=np.atleast_1d(axes)
        sources={name:digest(out/'trace.npz')}
        if key=='campaign_identification':
            cases=read(ROOT/'identifiability_report.json')['runs'];names=[case+'_'+mode for case in ['H1','H2','H3'] for mode in ['prior','identified'] if case+'_'+mode in cases];x=np.arange(len(names));sources={n:digest(ROOT/'runs'/n/'trace.npz') for n in names}
            for ax,error,scale,gate,label in zip(axes[:3],['mass_relative','com_m','inertia_com_frobenius_relative'],[100,1000,100],[5,5,10],['Mass error (%)','COM error (mm)','Tensor error (%)']):
                vals=[cases[n]['truth_error_evaluation_only'][error]*scale for n in names];ax.bar(x,vals,color=['#466f95' if n.endswith('_prior') else '#b47042' for n in names],width=.7);ax.axhline(gate,color='.3',ls='--');ax.set_ylabel(label)
            axes[3].bar(x,[cases[n]['rank'] for n in names],color='.5');axes[3].set_ylim(0,10.5);axes[3].set_ylabel('Data rank / 10');axes[3].set_xticks(x,[n.replace('_','\n')+('\nno latch' if n.startswith('H2') else '') for n in names],fontsize=8)
        elif key=='trajectory':
            active=a['approach_reference_applicable'].astype(bool)
            for ax,j,label in zip(axes,[1,2],['Y (m)','Z (m)']):
                for col,style,text in [('flange_position_world_m','-','Actual flange'),('approach_reference_position_world_m','--','Estimated-target reference')]:
                    p=a[col][active];ax.plot(p[:,0],p[:,j],style,label=text);ax.scatter(p[[0,-1],0],p[[0,-1],j],s=12)
                ax.set_aspect('equal',adjustable='box');ax.set_xlabel('X (m)');ax.set_ylabel(label)
            axes[0].legend(fontsize=8,loc='upper center',bbox_to_anchor=(.5,1.17))
        elif key=='tracking':
            active=a['approach_reference_applicable'].astype(bool)
            axes[0].plot(t[active],a['approach_position_error_m'][active]*1e3);axes[0].set_ylabel('Position error (mm)')
            axes[1].plot(t[active],np.rad2deg(a['approach_rotation_error_rad'][active]));axes[1].set_ylabel('Rotation error (deg)')
        elif key=='state_estimation':
            mask=a['estimate_valid'].astype(bool)
            if 'estimate_current_tick' in a:mask &= a['estimate_current_tick'].astype(bool)
            scales=[1000,180/np.pi,1000,180/np.pi];labels=['Position error (mm)','Rotation error (deg)','Velocity error (mm/s)','Angular rate error (deg/s)']
            for j,ax in enumerate(axes):
                for k in range(3):
                    q=3*j+k;ax.plot(t[mask],a['estimate_error'][mask,q]*scales[j],label='xyz'[k],lw=.8,ls=['-','--',':'][k])
                bound=3*np.sqrt(np.max(np.diagonal(a['estimate_covariance'][mask,3*j:3*j+3,3*j:3*j+3],axis1=1,axis2=2),axis=1))*scales[j]
                ax.fill_between(t[mask],-bound,bound,color='.8',alpha=.5,label='max marginal 3-sigma');ax.set_ylabel(labels[j])
            axes[0].legend(ncol=4,fontsize=8,loc='upper right')
        elif key=='capture_and_load':
            axes[0].plot(t,a['interface_translation_error_m']*1000);axes[0].axhline(.1,color='.3',ls='--');axes[0].set_ylabel('Interface gap (mm)')
            axes[1].plot(t,a['interface_rotation_error_deg']);axes[1].axhline(.05,color='.3',ls='--');axes[1].set_ylabel('Relative angle (deg)')
            axes[2].plot(t,np.linalg.norm(a['interface_relative_twist_world'][:,:3],axis=1)*1000);axes[2].axhline(1,color='.3',ls='--');axes[2].set_ylabel('Relative speed (mm/s)')
            axes[3].plot(t,np.rad2deg(np.linalg.norm(a['interface_relative_twist_world'][:,3:],axis=1)));axes[3].axhline(.2,color='.3',ls='--');axes[3].set_ylabel('Relative rate (deg/s)')
            axes[4].plot(t,a['contact_peak_force_n'],label='contact peak');axes[4].plot(t,np.linalg.norm(a['force_grasp_n'],axis=1),ls='--',label='interface resultant');axes[4].set_ylabel('Force (N)');axes[4].legend(fontsize=8)
            axes[5].plot(t,a['load_fraction']);axes[5].axhline(1,color='.3',ls='--');axes[5].set_ylabel('Load fraction')
            if latch is not None:
                axes[0].plot([latch,t[-1]],[.5,.5],':',color='.5');axes[1].plot([latch,t[-1]],[.1,.1],':',color='.5')
            for ax,scale in zip(axes[:4],[.1,.05,1.,.2]):ax.set_yscale('symlog',linthresh=scale);ax.set_ylim(bottom=0)
        elif key=='detumbling':
            for ax,col,gate,label in zip(axes,['target_omega_world_rad_s','target_base_relative_omega_world_rad_s'],[.1,.02],['World rate (deg/s)','Relative rate (deg/s)']):
                ax.plot(t,np.rad2deg(np.linalg.norm(a[col],axis=1)));ax.axhline(gate,color='.3',ls='--');ax.set_ylabel(label)
                ax.set_yscale('log')
                if metrics['full_window_evaluated']:ax.axvspan(*metrics['evaluation_window_s'],color='.85',alpha=.6)
        elif key=='momentum_energy':
            P=np.linalg.norm(a['linear_momentum_world_kg_m_s']-a['linear_momentum_world_kg_m_s'][0],axis=1);H=np.linalg.norm(a['angular_momentum_about_center_world_kg_m2_s']-a['angular_momentum_about_center_world_kg_m2_s'][0],axis=1)
            axes[0].plot(t,P);axes[0].set_ylabel('P drift (kg m/s)')
            axes[1].plot(t,H);axes[1].set_ylabel('H drift (kg m²/s)')
            axes[2].plot(t,a['kinetic_energy_j'],label='kinetic');axes[2].plot(t,a['relative_energy_j'],ls='--',label='relative');axes[2].set_ylabel('Energy (J)');axes[2].legend()
            axes[0].axhline(1e-4,color='.3',ls='--');axes[1].axhline(1e-5,color='.3',ls='--')
            axes[0].set_yscale('symlog',linthresh=1e-12);axes[1].set_yscale('symlog',linthresh=1e-12);axes[0].set_ylim(bottom=0);axes[1].set_ylim(bottom=0)
            axes[3].plot(t,a['kinetic_energy_j']-a['kinetic_energy_j'][0],label='kinetic change')
            for col,label,style in [('actuator_power_w','actuator','--'),('passive_power_w','passive',':'),('all_constraint_power_w','constraint','-.')]:
                power=a[col];work=np.r_[0,np.cumsum(.5*(power[1:]+power[:-1])*np.diff(t))];axes[3].plot(t,work,style,label=label)
            axes[3].set_ylabel('Energy / work (J)');axes[3].legend(ncol=4,fontsize=8)
        elif key=='joints':
            for ax,col,label in zip(axes,['joint_position_rad','joint_velocity_rad_s','ctrl_nm'],['Position (rad)','Velocity (rad/s)','Torque (N m)']):
                for j in range(7):ax.plot(t,a[col][:,j],label=str(j+1),lw=.8,ls=['-','--','-.',':',(0,(5,1,1,1)),(0,(3,1,1,1,1,1)),(0,(1,2))][j])
                ax.set_ylabel(label)
            axes[0].legend(ncol=7,fontsize=8)
        elif key=='geometry':
            from v6_mujoco.geometry_capture.design import compile_model,extra_pairs
            pairs=extra_pairs(compile_model());target=[i for i,p in enumerate(pairs) if p.category=='target_distance'];tool=[i for i,p in enumerate(pairs) if p.category=='tool_robot'];intended=[i for i,p in enumerate(pairs) if p.category=='intended_surface']
            axes[0].plot(t,a['minimum_noncontact_clearance_m']*1000,label='original pairs');axes[0].plot(t,np.min(a['extra_pair_distances_m'][:,tool],axis=1)*1000,label='new tool pairs');axes[0].axhline(40,ls='--',color='.3');axes[0].set_ylabel('Robot clearance (mm)');axes[0].legend(fontsize=8)
            axes[1].plot(t,np.min(a['extra_pair_distances_m'][:,target],axis=1)*1000,label='other target pairs');axes[1].plot(t,np.min(a['extra_pair_distances_m'][:,intended],axis=1)*1000,label='intended surface');axes[1].axhline(1,ls='--',color='.3');axes[1].axhline(-2,ls=':',color='.3');axes[1].set_ylabel('Target clearance (mm)');axes[1].legend(fontsize=8)
        elif key=='identification':
            truth=read(out/'config.json')['truth_evaluation_only'];pi=a['parameter_pi'];axes[0].plot(t,pi[:,0]);axes[0].axhline(truth['mass'],color='.3',ls='--');axes[0].set_ylabel('Mass (kg)')
            for j in range(3):axes[1].plot(t,pi[:,j+1]/pi[:,0]*1000,label='xyz'[j]);axes[1].axhline(truth['com'][j]*1000,color='.5',ls='--',lw=.6)
            axes[1].set_ylabel('COM in B (mm)');axes[1].legend(ncol=3)
            from .momentum_regressor import parameters
            truth_pi=parameters(truth['mass'],truth['com'],truth['inertia'])
            for j in range(3):axes[2].plot(t,pi[:,4+j],label='xyz'[j]);axes[2].axhline(truth_pi[4+j],color='.5',ls='--',lw=.6)
            axes[2].set_ylabel('Origin inertia (kg m²)');axes[2].legend(ncol=3);axes[3].step(t,a['parameter_rank'],where='post');axes[3].set_ylim(-.2,10.2);axes[3].set_ylabel('Data rank / 10')
            for j,label in enumerate(['xy','xz','yz']):axes[4].plot(t,pi[:,7+j],label=label);axes[4].axhline(truth_pi[7+j],color='.5',ls='--',lw=.6)
            axes[4].set_ylabel('Inertia products (kg m²)');axes[4].legend(ncol=3)
            posterior=read(out/'posteriors.json');pt=np.array([x['time_s'] for x in posterior]);sv=np.array([x['singular_values'] for x in posterior]);threshold=np.maximum(cfg['mission']['information_singular_min'],sv[:,0]*cfg['mission']['information_relative_min'])
            for j in range(10):axes[5].semilogy(pt,np.maximum(sv[:,j],1e-12),lw=.6,color='.65')
            axes[5].semilogy(pt,np.maximum(sv[:,0],1e-12),lw=.9,label='largest singular value');axes[5].semilogy(pt,np.maximum(sv[:,-1],1e-12),lw=.9,ls=':',label='smallest singular value')
            axes[5].semilogy(pt,threshold,'k--',label='data-rank threshold');axes[5].set_ylabel('Scaled singular value');axes[5].legend(fontsize=8)
        for ax in axes:
            if latch is not None and key not in ['trajectory','campaign_identification']:ax.axvline(latch,color='black',ls=':',lw=.8)
            ax.grid(alpha=.15)
        if key not in ['trajectory','campaign_identification']:axes[-1].set_xlabel('Actual time (s)')
        for ext in ['pdf','png']:fig.savefig(figdir/(key+'.'+ext),bbox_inches='tight')
        latch_text='none' if latch is None else f'{latch:.3f} s'
        caption=CAPTIONS[key]+f" Run {name}; sensor {cfg['sensor_mode']}; actual interval {t[0]:.3f}–{t[-1]:.3f} s; latch {latch_text}; outcome {metrics['status']}; full evaluation window {metrics['full_window_evaluated']}; parameter feedback used {metrics['parameter_feedback_used']}."
        if key=='campaign_identification':
            outcomes=[]
            for case in ['H1','H2','H3']:
                cm=read(ROOT/'runs'/(case+'_prior')/'metrics.json');outcomes.append(f"{case}: {cm['status']} at {cm['end_time_s']:.3f} s, "+('no latch' if cm['latch_time_s'] is None else 'latched'))
            caption=CAPTIONS[key]+' Source runs: '+', '.join(sources)+'. '+ '; '.join(outcomes)+'. H2 never used parameter feedback. Errors use evaluation-only truth; unequal actual horizons are retained. Control-mode names do not imply parameter use; consult comparison.json.'
        if key=='capture_and_load':caption+=' Dashed pose lines are capture gates; dotted post-latch lines are holding gates.'
        if key=='state_estimation':caption+=' World-expressed geometric-origin errors at estimator output ticks; shading is maximum of three marginal bounds.'
        if key=='identification':caption+=' Singular values below 1e-12 are displayed at the log-axis floor.'
        plt.close(fig);entries[key]={'caption':caption,'run':name,'trace_sha256':digest(out/'trace.npz'),'source_run_trace_sha256':sources,'png_sha256':digest(figdir/(key+'.png')),'pdf_sha256':digest(figdir/(key+'.pdf'))}
        wrapper='from pathlib import Path\nimport sys\nsys.path.insert(0,str(Path(__file__).resolve().parents[4]))\nfrom v6_mujoco.adaptive_capture.figures import run\nfrom v6_mujoco.adaptive_capture.common import ROOT,read\nif __name__ == "__main__":\n    run(sys.argv[1] if len(sys.argv)>1 else read(ROOT/"experiment_manifest.json")["selected_development_run"],'+repr(key)+')\n'
        (figdir/('gen_'+key+'.py')).write_text(wrapper,encoding='utf-8')
    existing=read(manifest) if only and manifest.exists() else {};existing.update(entries);save(manifest,existing)
    (figdir/'latex_includes.tex').write_text('\n'.join('\\begin{figure}\n\\centering\n\\includegraphics[width=.95\\textwidth]{'+key+'.pdf}\n\\caption{'+item['caption'].replace('_','\\_')+'}\n\\end{figure}\n' for key,item in existing.items()),encoding='utf-8')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--figure',choices=list(CAPTIONS));a=p.parse_args();run(a.name,a.figure)

