"""Recover exact ancillary V1 bytes when their archived SHA matches."""
import hashlib
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from v6_mujoco.system_capture.planning.contracts import OUT,PROJECT,read,save,sha

def main():
    cfg=read(OUT/'runs/D00/config.json');prefix='v6_mujoco/system_capture/planning/'
    source=(PROJECT/(prefix+'tests.py')).read_bytes()
    a=source.index(b'    def test_atomic_sharing_retry');b=source.index(b'    def test_quintic_and_bump_jets',a)
    candidates={'tests.py':[source[:a]+source[b:]]}
    p=(PROJECT/(prefix+'phase.py')).read_text(encoding='utf-8')
    a=p.index('                    repaired=');b=p.index('                continue',a)
    p=p[:a]+"                    raise RuntimeError('implementation failure retained; repair dependency before continuing')\n"+p[b:]
    a=p.index("            cfg=read(OUT/'runs'/e['name']/'config.json')");b=p.index("            for kind in ['actuator','decision']:",a)
    p=p[:a]+p[b:]
    candidates['phase.py']=[p.encode(),p.replace('\n','\r\n').encode()]
    recovered=[]
    for n,options in candidates.items():
        for raw in options:
            if hashlib.sha256(raw).hexdigest()==cfg['source_identity'][prefix+n]:
                dest=OUT/'versions/V1'/(prefix+n);dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(raw);recovered.append(n);break
    result={}
    for name in ['B01','D00','D00_R1','D01','P00','P01','P02','P03']:
        f=OUT/'runs'/name/'config.json'
        if not f.exists():continue
        c=read(f);changed=[]
        for rel,h in c['source_identity'].items():
            if (PROJECT/rel).exists() and sha(PROJECT/rel)==h:continue
            archives=[v for v in (OUT/'versions').glob('*/'+rel) if sha(v)==h]
            recovered_path=OUT/'versions/source_by_sha'/h/Path(rel).name
            if recovered_path.exists() and sha(recovered_path)==h:archives.append(recovered_path)
            changed.append(dict(path=rel,expected_sha256=h,recovery=archives[0].relative_to(PROJECT).as_posix() if archives else None,
                scope='auxiliary registration/test/diagnostic source; not part of this run controller runtime' if rel.endswith(('/tests.py','/phase.py','/diagnostics.py')) else 'runtime'))
        result[name]=changed
    save(OUT/'source_recoverability.json',dict(recovered_V1_ancillary=recovered,changed_files_by_run=result,
        runtime='V1/V2 scientific runtime copies plus byte-identical files in containing commit; baseline B0 core unchanged',
        identity_note='config source_identity includes a superset of invoked code; any missing ancillary bytes are listed, not fabricated'))
    print(result)

if __name__=='__main__':main()
