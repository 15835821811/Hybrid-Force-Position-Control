"""Read-only evidence/package checks; never creates a plant or calls mj_step."""
from html.parser import HTMLParser
from pathlib import Path
import subprocess
import numpy as np
from v6_mujoco.model import PROJECT_ROOT
from v6_mujoco.postgrasp_campaign.io import read,save,identity,check_identity
from v6_mujoco.end_to_end_capture.common import arrays,digest,verify_design as verify_n205
from .common import ROOT,verify_design

def run():
    checks={}
    def check(name,value):
        checks[name]=bool(value)
        assert value,name
    verify_n205();manifest=verify_design();check_identity(manifest['runtime_identity_at_admission'])
    checks['original_n205_and_frozen_n206_identities']=True
    ledger=read(ROOT/'run_ledger.json');r=read(ROOT/'nominal/metrics.json');v=read(ROOT/'nominal/validation.json');a=arrays(ROOT/'nominal/trace.npz')
    check_identity(r['implementation_identity']);check_identity(v['execution_identity']);check_identity(v['verification_identity'])
    check('one_formal_attempt',len(ledger['formal_runs'])==1 and ledger['formal_runs'][0]['scenario']=='nominal')
    check('nominal_failure_skips_dependents',set(ledger['skipped'])=={'fine','light','heavy'} and all(x=='NOT_RUN_NOMINAL_FAILURE' for x in ledger['skipped'].values()))
    check('actual_fixed_capture_failure',r['status']=='CAPTURE_GATE_FAILED' and abs(r['end_time_s']-8)<1e-9 and not a['eq_active'].any())
    check('continuous_clock',len(a['time_s'])==4001 and a['time_s'][0]==0 and np.allclose(np.diff(a['time_s']),.002,atol=1e-12,rtol=0))
    check('final_window_not_claimed',r['performance']=='NOT_EVALUATED' and not r['continuous_18s_completed'] and not r['full_window_evaluated'])
    check('independent_replay',v['passed'] and v['steps']==4000 and v['state_restore_count']==1 and v['controller_update_calls']==0 and max(v['max_errors'].values())==0)
    check('actual_trace_hash',r['trace_sha256']==v['trace_sha256']==digest(ROOT/'nominal/trace.npz'))
    check('unchanged_failure_event',read(ROOT/'nominal/events.json')[0]['capture_passed'] is False)
    check('bounded_tests_recorded',ledger['accounting_totals']['short_robot_physics_steps_including_failure']==21 and len(ledger['failed_short_unit_attempts'])==1)
    check('affected_fixture_tests',read(ROOT/'interface_tests/result.json')['passed'] and len(ledger['interface_tests'])==19)
    check('new_unit_tests',read(ROOT/'unit_tests.json')['passed'])
    check('three_predeclared_planning_candidates',len(list((ROOT/'planning').glob('*/kinematic_trace.npz')))==3 and read(ROOT/'trajectory_screening.json')['selected']=='standard_C1')
    q=read(ROOT/'qualification_matrix.json')
    check('qualification_boundaries',q['original_terminal_geometry_feasible'] is False and q['hardware_design_assumption_changed'] is True and q['real_gripper_and_hardware_validated'] is False and q['capture_gate_at_8s']['nominal'] is False)
    figures=read(ROOT/'figures/manifest.json');check_identity(figures['generator_identity']);check_identity(figures['geometry_source_identity']);check_identity(figures['files'])
    check('thirteen_figure_groups',len(read(ROOT/'figures/captions.json'))==13 and len(list((ROOT/'figures').glob('*.png')))==13 and len(list((ROOT/'figures').glob('*.pdf')))==13)
    video=read(ROOT/'visualizations/manifest.json');check_identity(video['render_identity'])
    check('exactly_two_actual_videos',len(video['videos'])==2 and len(list((ROOT/'visualizations').glob('*.mp4')))==2 and video['physics_steps']==0 and video['trace_sha256']==r['trace_sha256'])
    for name,entry in video['videos'].items():
        check('video_'+name,entry['sha256']==digest(ROOT/'visualizations'/f'{name}.mp4') and int(entry['probe']['streams'][0]['nb_frames'])==241)
    check('video_actual_time_range',video['physical_times_s'][0]==0 and abs(video['physical_times_s'][-1]-8)<1e-9 and video['physical_indices'][-1]==4000)
    class Links(HTMLParser):
        def __init__(self):super().__init__();self.links=[]
        def handle_starttag(self,tag,attrs):
            self.links.extend(value for key,value in attrs if key in ['href','src','poster'])
    index=ROOT/'visualizations/index.html';parser=Links();parser.feed(index.read_text(encoding='utf-8'))
    check('html_local_links',all((index.parent/x).resolve().is_file() for x in parser.links if not x.startswith(('https:','http:','#'))))
    package=read(ROOT/'delivery_manifest.json');check_identity(package['files']);check_identity(package['report_identity'])
    checks['delivery_file_identities']=True
    def git(*args):return subprocess.check_output(['git',*args],cwd=PROJECT_ROOT,text=True).strip()
    base='f2c4cf5cc12dd299589dc4e0a3f32d52ed085403'
    check('separate_branch',git('branch','--show-current')=='codex/n206-terminal-geometry-feasibility')
    check('exact_source_ancestry',git('merge-base','HEAD',base)==base)
    check('historical_branch_unchanged',git('rev-parse','codex/n205-end-to-end-capture-detumbling')==base)
    changed=git('diff','--name-only',base).splitlines()
    allowed=lambda p:p in ['.gitattributes','README.md','configs/n206_geometry_capture.yaml','models/flexiv_rizon4s_n206_tool_scene.xml','paper/N206_GEOMETRY_COMPATIBILITY_REPORT.md'] or p.startswith(('v6_mujoco/geometry_capture/','output/fpmfc/n206_geometry_capture/'))
    check('tracked_historical_files_unchanged',all(allowed(p) for p in changed))
    save(ROOT/'completion_audit.json',{'evidence_package_validated':True,'physics_outcome':'CAPTURE_GATE_FAILED','checks':checks,'audit_identity':identity([__file__]),'formal_attempts':1,'replays':1,'new_physics_steps_in_audit':0,'source_commit':base,'branch':'codex/n206-terminal-geometry-feasibility','limits':['No 18 s completion or fixed-window detumbling evidence.','No real gripper, hardware design, CAD or load certification.','Finite planning bounds and perturbations do not prove robustness for every continuous disturbance.'],'delivery_manifest_sha256':digest(ROOT/'delivery_manifest.json')})
    print({'evidence_package_validated':True,'checks':len(checks),'physics_outcome':r['status']})

if __name__=='__main__':run()
