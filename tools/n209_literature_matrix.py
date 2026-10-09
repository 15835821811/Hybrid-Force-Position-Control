import csv
import json
from pathlib import Path

root=Path(__file__).resolve().parents[1]/'paper/system_paper'
keys=['wang2018integrated','zhan2022reactionless','gong2024double','vijayan2025unified','uchida2025inertia','wensing2024observability']
rows=[];bib=[]
notes=[
 ['ABSTRACT_ONLY','Institutional abstract and Crossref journal metadata','Abstract; no equation attribution','NOT_VERIFIED','Noncooperative; active target inputs NOT_VERIFIED','Post-capture; detailed contact model NOT_VERIFIED','Mass properties from measured target force/torque','Full controller/stability assumptions NOT_VERIFIED','7-DoF simulation (abstract)','Discussion only; no numeric reproduction','https://portal.fis.tum.de/en/publications/an-integrated-control-scheme-for-space-robot-after-capturing-non-/'],
 ['ABSTRACT_ONLY','Publisher abstract and Crossref journal metadata','Abstract; no equation attribution','Free-floating per title; complete actuation assumptions NOT_VERIFIED','Detumbling target; details NOT_VERIFIED','Post-capture','Dynamic uncertainties; exact estimator NOT_VERIFIED','Complete reactionless-control theorem assumptions NOT_VERIFIED','Publisher abstract; detailed setup NOT_VERIFIED','Discussion only; no numeric reproduction','https://www.sciencedirect.com/science/article/abs/pii/S0273117722007037'],
 ['ABSTRACT_ONLY','Publisher abstract and Crossref journal metadata','Abstract/highlights; no equation attribution','Free-floating per title; detailed actuation NOT_VERIFIED','Already captured target','Post-capture; detailed contact model NOT_VERIFIED','Concurrent learning identification advertised','No persistent excitation wording does not waive finite-data rank conditions; theorem not read','Publisher abstract; detailed setup NOT_VERIFIED','Discussion only; no numeric reproduction','https://www.sciencedirect.com/science/article/pii/S0016003223008153'],
 ['FULL_TEXT_TARGETED_READ','DLR accepted preprint January2025; final volume30(6) metadata','pp1-2; pp5-7; Eqs20-25; Algorithm1','Thrusters and reaction wheels explicitly controlled','Uncooperative passive client','Approach/grasp/post-grasp; interface force limitation','Not used here as inertia-estimation evidence','Coordinated externally actuated dynamics; proof not transferable to passive base','Simulation and DLR OOS-Sim HIL','Actuator-incompatible direct baseline; discussion only','https://elib.dlr.de/216816/1/root_copyright.pdf'],
 ['FULL_TEXT_TARGETED_READ','arXiv2512.21886v1,26Dec2025; conference DOI verified','SecII-B/II-C,III,IV-A1; Eqs12-19','Free-floating orbital case without external force','Rigidly grasped target from initial time','Rendezvous and detumbling precede estimation; ideal tracking study','Target10 inertial parameters; momentum regression and log-det regularization','Rigid model; excitation and regressor dependence; identifiability separate from control performance','MuJoCo numerical experiments','Conceptual momentum/physicality comparison; not reproduced','https://arxiv.org/html/2512.21886v1'],
 ['FULL_TEXT_TARGETED_READ','Journal2024; arXiv1711.03896v3 revised20Sep2023','Sec4.4 Theorem1,Sec6.4 Theorem2,AppendixC','General articulated/floating models; no capture controller','Not a target-capture study','No compliant capture assumption used here','Structural identifiability of inertial transfers','Gravity-free serial chain main theorem; extensions handled separately','Geometric analysis/algorithm and examples','Distinguish structural nullspace from finite-data SVD; not an online algorithm replication','https://journals.sagepub.com/doi/10.1177/02783649241258215']]
for i,(key,note) in enumerate(zip(keys,notes),1):
    m=json.loads((root/'literature_metadata'/f'{i}.json').read_text(encoding='utf-8'))['message']
    authors=' and '.join(x.get('given','')+' '+x['family'] for x in m['author'])
    if i==6:authors='Patrick M. Wensing and Günter Niemeyer and Jean-Jacques E. Slotine'
    year={1:2018,2:2022,3:2024,4:2025,5:2025,6:2024}[i]
    row=dict(zip(['fulltext_status','read_version','locations','servicer_actuation','target_drive','contact_grasp','unknown_parameters','theoretical_conditions','validation','portable_scope','source_url'],note))
    row.update(key=key,title=m['title'][0],authors=authors,year=year,doi=m['DOI'],arxiv='2512.21886v1' if i==5 else '1711.03896v3' if i==6 else '',classification='DISCUSSION_ONLY_NOT_IMPLEMENTED',access_date='2026-10-09')
    rows.append(row)
    fields={'title':m['title'][0],'author':authors,'year':year,'booktitle' if i==5 else 'journal':m['container-title'][0],'doi':m['DOI'],'url':note[-1]}
    for f in ['volume','page','issue']:
        if f in m:fields[{'page':'pages','issue':'number'}.get(f,f)]=m[f].replace('-','--') if f=='page' else m[f]
    if row['arxiv']:fields.update(eprint=row['arxiv'],archivePrefix='arXiv')
    bib.append('@'+('inproceedings' if i==5 else 'article')+'{'+key+',\n'+',\n'.join('  '+k+' = {'+str(v)+'}' for k,v in fields.items())+'\n}\n')
extras=[
dict(key='khorshidi2025physical',title='Physically-Consistent Parameter Identification of Robots in Contact',authors='Shahram Khorshidi and Murad Dawood and Benno Nederkorn and Maren Bennewitz and Majid Khadiv',year=2025,doi='',arxiv='2409.09850v2',fulltext_status='FULL_TEXT_TARGETED_READ',read_version='arXivv2,18Mar2025; ICRA2025 author version',locations='SecIII-C Eqs2a-2b; SecIV-B/IV-C; SecV',servicer_actuation='Underactuated legged floating base; joint torque sensing',target_drive='Not a space target study',contact_grasp='Rigid non-slipping environment contacts for projection',unknown_parameters='Whole-body physical/geometric inertial parameters',theoretical_conditions='Contact Jacobian and rigid no-slip projection; convex physical consistency',validation='Solo12 simulation and Spot hardware locomotion',portable_scope='Physical constraints as context; contact projection is not this soft-weld implementation',source_url='https://arxiv.org/html/2409.09850v2',classification='DISCUSSION_ONLY_NOT_IMPLEMENTED',access_date='2026-10-09'),
dict(key='gautam2026cerg',title='Compliant Explicit Reference Governor for Contact Friendly Robotic Manipulators',authors='Yaashia Gautam and Gilberto Briscoe-Martinez and Adhitya Mohan and Nataliya Nechyporenko and Alessandro Roncone and Marco M. Nicotra',year=2026,doi='',arxiv='2504.09188v2',fulltext_status='FULL_TEXT_TARGETED_READ',read_version='arXivv2,19May2026; accepted IFAC World Congress2026 per author metadata',locations='Sec2 Eq1; Sec3 Eq4-12; Sec4 Definition1/Lemma1/Proposition1',servicer_actuation='Fully actuated manipulator',target_drive='Environment contact; not uncontrolled tumbling satellite',contact_grasp='Disjunctive contact/energy condition with compliance',unknown_parameters='Not an inertial-identification contribution',theoretical_conditions='Fully actuated prestabilized PD+gravity; exact predicted dynamics and DSM properties',validation='MATLAB/Drake simulation and Franka Panda hardware',portable_scope='Reference-governor structural motivation only; its safety proof does not apply here',source_url='https://arxiv.org/html/2504.09188v2',classification='LITERATURE_INSPIRED_STRUCTURE_ONLY',access_date='2026-10-09')]
for row in extras:
    rows.append(row);bib.append('@misc{'+row['key']+',\n  title={'+row['title']+'},\n  author={'+row['authors']+'},\n  year={'+str(row['year'])+'},\n  eprint={'+row['arxiv']+'},\n  archivePrefix={arXiv},\n  url={'+row['source_url']+'}\n}\n')
fields=['key','title','authors','year','doi','arxiv','read_version','fulltext_status','locations','servicer_actuation','target_drive','contact_grasp','unknown_parameters','theoretical_conditions','validation','portable_scope','classification','source_url','access_date']
with (root/'related_work_matrix.csv').open('w',encoding='utf-8',newline='') as f:
    w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
(root/'references.bib').write_text('\n'.join(bib),encoding='utf-8')
(root/'literature_metadata/verification_notes.md').write_text('''# Metadata verification notes

Six journal/conference DOI records were retrieved directly from Crossref.
The first three works have only publisher/institutional abstracts available in
this task; their formulas and actuator details remain NOT_VERIFIED.
Five accessible full texts were read selectively at the locations in the CSV.
No external numerical controller is implemented or assigned results.

Uchida's arXivv1 author line and IEEE-deposited Crossref record both spell
the second author's given name "Antonine"; the manuscript email uses "antoine".
The bibliography preserves the verified published metadata spelling. An email
is not a justification to silently change an author's name.

Wensing's Crossref given-name encoding is malformed. Publisher title page
and arXiv metadata both explicitly provide "Günter Niemeyer"; that is the
bibliographic spelling used. The2024 journal and2017/2023 arXiv dates are not
conflated. Gautam is cited as the actually read2026v2 with its six-author list,
not the2025v1 seed listing. No final IFAC DOI is invented.

Bibliographic existence, source assumption compatibility and successful
algorithm reproduction are separate statuses. Full-text availability alone
does not authorize claims of reproduced equations, proofs or experiments.
''',encoding='utf-8')
print('wrote8 verified seed entries')
