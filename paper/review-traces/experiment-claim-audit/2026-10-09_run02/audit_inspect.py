import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import json, hashlib, pathlib, collections
HERE=pathlib.Path(__file__).parent
manifest=json.loads((HERE/'inputs.json').read_text(encoding='utf-8-sig'))
PAPER=pathlib.Path(manifest['paper_dir'])
KEYS=manifest['declared_input_hashes']
FILES={k: pathlib.Path(k) if pathlib.Path(k).is_absolute() else PAPER/k for k in KEYS}
def read(path):
    p=pathlib.Path(path)
    assert p in FILES.values(), f'undeclared {p}'
    return json.loads(p.read_text(encoding='utf-8-sig'))
def inspect():
    result=[]
    for k,p in FILES.items():
        h=hashlib.sha256()
        with p.open('rb') as f:
            while b:=f.read(1024*1024): h.update(b)
        actual='sha256:'+h.hexdigest()
        result.append({'path':k,'bytes':p.stat().st_size,'actual':actual,'match':actual==KEYS[k]})
    (HERE/'hash_verification.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('HASHES',len(result),'mismatches',[r for r in result if not r['match']])
    for r in result:
        p=pathlib.Path(r['path'])
        print(str(p).replace(str(PAPER.parent),'ROOT'),r['bytes'])
if __name__=='__main__': inspect()
