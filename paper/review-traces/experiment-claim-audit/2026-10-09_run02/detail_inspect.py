from audit_inspect import *
ROOT=PAPER.parent
base=ROOT/'output/fpmfc/n209_paper_system'
for f in ['runs/D03_V1_S01/validation_timestamp_audited.json','runs/D03_V1_S01/validation.json','runs/D01_V1_nominal/metrics.json','runs/D03_V1_S01/events.json','tests_1791536217331844300.json']:
    d=read(base/f)
    for k in list(d):
        if 'identity' in k or k=='stdout': d[k]='OMITTED_HASH_MAP_OR_TEST_OUTPUT'
    print('\nFILE',f,'\n',json.dumps(d,ensure_ascii=False))
for name in ['H2_prior','S01_noise_delay','D05_final_nominal']:
    d=read(base/'diagnostics'/name/'snapshots.json')[-1]
    print('\nSNAPSHOT',name)
    print(json.dumps({k:d[k] for k in ['time','row_names','row_units','raw_lower','raw_upper']},ensure_ascii=False))
    print('QP',json.dumps(d['qp'],ensure_ascii=False))
for name in ['D02_V1_H2','D04_V2_H2','R02_V2_H2','D03_V1_S01']:
    d=read(base/'runs'/name/'progress_governor.json')[-1]
    print('TERMINAL_GOV',name,json.dumps(d))
for name in []:
    d=read(base/'runs'/name/'config.json')
    print('CONFIG',name,json.dumps({k:v for k,v in d.items() if k!='identity'},ensure_ascii=False))
