"""Two registered seeds, three equal-budget strategies, then matched dynamics."""
import json
import numpy as np
from ...fpmfc.optimizer import particle_swarm_optimize
from .common import OUT,PROTOCOL,read,save,source,assumptions,sha
from .phase import evaluation,dynamic,book

def run():
    protocol=read(PROTOCOL);s=source();a=assumptions();author=s['author_reported'];T=author['time_s'];psi=author['shape_rad']
    check=evaluation('author_reported_point',T,psi)
    keys={};keys['source'],dm=dynamic('author_point_source',T,psi);keys['hqp'],dh=dynamic('author_point_hqp',T,psi,'project_hqp')
    save(OUT/'author_point_check.json',{'classification':'AUTHOR_REPORTED_POINT_CHECK','not_a_found_optimum':True,'parameters':{'T_s':T,'psi_f_rad':psi},'kinematic':check.to_dict(),'dynamics':{'source':dm,'hqp':dh},'run_keys':keys})
    cells={}
    for seed in protocol['seeds']:
        for strategy in protocol['optimization_strategies']:
            key=f'{seed}_{strategy}';dest=OUT/'optimization_runs'/key;dest.mkdir(parents=True,exist_ok=True)
            if (dest/'result.json').exists():cells[key]=read(dest/'result.json');continue
            bounds=np.array([a['optimization']['time_bounds_s'],a['optimization']['shape_bounds_rad']],dtype=float)
            if strategy!='joint':bounds[1]=0 if strategy=='fixed_0' else np.pi/2
            counter=[0]
            def evaluate(tc,ps):
                i=counter[0];counter[0]+=1
                return evaluation(key+f'_{i:03d}',tc,ps)
            def progress(g,best):
                print(f'S01 PSO {key} generation {g}/6 evaluations={counter[0]} feasible={best.feasible} G={best.metrics["objective_G"]:.7g}',flush=True)
                save(dest/'progress.json',{'generation':g,'evaluations':counter[0],'best':best.to_dict()})
            result=particle_swarm_optimize(evaluate,bounds,seed=seed,population=protocol['pso']['population'],generations=protocol['pso']['generations'],
              inertia=s['pso']['inertia'],cognitive=s['pso']['cognitive'],social=s['pso']['social'],velocity_fraction=s['pso']['velocity_range_fraction'],
              convergence_tolerance=s['pso']['adjacent_best_threshold'],stagnation_generations=protocol['pso']['early_convergence_streak'],progress=progress)
            data=result.to_dict();data['strategy']=strategy;data['budget_status']='REDUCED_BUDGET';data['author_max_iterations']=1000
            data['adjacent_best_threshold_met_last_pair']=bool(len(result.history)>1 and abs(result.history[-1]-result.history[-2])<=s['pso']['adjacent_best_threshold'])
            data['convergence_statement']='Six generations do not establish PSO convergence; adjacent-best stagnation alone is not convergence evidence.'
            save(dest/'result.json',data);np.savez_compressed(dest/'best_trace.npz',**result.best_rollout.trace);cells[key]=data
    first=cells[f'{protocol["seeds"][0]}_joint'];common_T=float(first['best_position'][0]);common_psi=float(first['best_position'][1])
    fixed=[]
    for label,value in [('psi0',0.),('psi90',np.pi/2),('joint',common_psi)]:
        kin=first['best_rollout'] if label=='joint' else evaluation('common_time_'+label,common_T,value).to_dict()
        run_key,metrics=dynamic('common_time_'+label,common_T,value)
        fixed.append({'label':label,'T_s':common_T,'psi_f_rad':value,'kinematic':kin,'dynamics_run':run_key,'dynamics':metrics})
    paired=[]
    for seed in protocol['seeds']:
        for strategy in protocol['optimization_strategies']:
            cell=cells[f'{seed}_{strategy}'];tc,ps=cell['best_position'];key,metrics=dynamic(f'seed{seed}_{strategy}',tc,ps)
            paired.append({'seed':seed,'strategy':strategy,'cell':f'{seed}_{strategy}','evaluations':cell['evaluations'],'T_s':tc,'psi_f_rad':ps,'kinematic':cell['best_rollout'],'dynamics_run':key,'dynamics':metrics})
    fine_key,fine=dynamic('author_point_source_half_dt',T,psi,dt=a['controller']['physics_dt_s']/2)
    kin_fine=evaluation('author_point_kinematic_half_step',T,psi,dt=a['optimization']['planning_dt_s']/2)
    save(OUT/'numerical_refinement.json',{'prespecified_case':'author reported point, regardless of success','coarse_dynamic_run':keys['source'],'fine_dynamic_run':fine_key,
      'coarse':dm,'fine':fine,'coarse_kinematic':check.to_dict(),'fine_kinematic':kin_fine.to_dict(),'task_period_unchanged_s':a['controller']['task_period_s'],'no_extra_rollout_for_comparison':True})
    save(OUT/'fixed_time_comparison.json',{'protocol':'A','classification':'DECLARED_ADAPTATION','common_time_rule':protocol['common_time_rule'],'common_T_s':common_T,'rows':fixed,'comparison_admissible':all(x['dynamics']['feasible'] for x in fixed),'failure_denominator':len(fixed)})
    save(OUT/'equal_budget_comparison.json',{'protocol':'B','classification':'NEW_EXTENSION_ON_SOURCE_COMPATIBLE_MODEL','per_cell_budget':120,'seeds':protocol['seeds'],'rows':paired,'comparison_admissible_by_seed':{str(seed):all(x['dynamics']['feasible'] for x in paired if x['seed']==seed) for seed in protocol['seeds']},'failure_denominator':len(paired),'statistical_scope':'two optimization seeds are not population robustness samples; time ticks are not independent replicates'})
    print({'S01_run_finished':True,'kinematic_evaluations':len(book()['evaluations']),'dynamics_attempts':len(book()['dynamics'])},flush=True)
