"""Recover source bytes from this task's recorded file patches, SHA-checked.

Only tool-call patches are read; no conversation text or reasoning is exported.
The optional task transcript is a local recovery aid, never a runtime dependency.
"""
import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from v6_mujoco.system_capture.planning.contracts import OUT,PROJECT,BASE,read,save

def main(transcript):
    expected={}
    for cfg in (OUT/'runs').glob('*/config.json'):
        for rel,h in read(cfg)['source_identity'].items():
            if rel.startswith('v6_mujoco/system_capture/'):expected.setdefault(rel,set()).add(h)
    current={};recovered={};skipped=[]
    def capture(rel,source):
        for s in [source,source.replace('\n','\r\n')]:
            raw=s.encode('utf-8');h=hashlib.sha256(raw).hexdigest()
            if h in expected[rel]:
                dest=OUT/'versions/source_by_sha'/h/Path(rel).name
                dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(raw)
                recovered[h]=dict(path=rel,archive=dest.relative_to(PROJECT).as_posix())
    for line in Path(transcript).read_text(encoding='utf-8').splitlines():
        event=json.loads(line);p=event.get('payload',{})
        if event.get('type')!='response_item' or p.get('type')!='custom_tool_call':continue
        text=p.get('input','');match=re.search(r'tools\.apply_patch\(("(?:[^"\\]|\\.)*")\)',text)
        if not match:continue
        patch=json.loads(match.group(1));parts=re.split(r'^\*\*\* (Add|Update|Delete) File: (.+)\n',patch,flags=re.M)
        for i in range(1,len(parts),3):
            kind,path,body=parts[i:i+3]
            try:rel=Path(path).relative_to(PROJECT).as_posix()
            except ValueError:continue
            if rel not in expected:continue
            body=body.split('*** End Patch')[0].rstrip('\n')+'\n'
            if kind=='Add':current[rel]='\n'.join(x[1:] for x in body.splitlines() if x.startswith('+'))+'\n'
            elif kind=='Update':
                if rel not in current:
                    try:current[rel]=subprocess.check_output(['git','show',BASE+':'+rel],cwd=PROJECT).decode('utf-8').replace('\r\n','\n')
                    except subprocess.CalledProcessError:continue
                for hunk in re.split(r'^@@[^\n]*\n',body,flags=re.M)[1:]:
                    lines=hunk.splitlines();old='\n'.join(x[1:] for x in lines if x[:1] in [' ','-']);new='\n'.join(x[1:] for x in lines if x[:1] in [' ','+'])
                    if old not in current[rel]:skipped.append(dict(path=rel,event=event.get('timestamp'),reason='hunk context unavailable'));break
                    current[rel]=current[rel].replace(old,new,1)
            capture(rel,current[rel])
    save(OUT/'versions/source_by_sha/recovery_manifest.json',dict(recovered=recovered,skipped=skipped,
        provenance='Exact task file-patch history, accepted only when SHA256 equals a recorded run config. No transcript text is exported.'))
    print(dict(recovered=len(recovered),skipped=len(skipped)))

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('transcript');main(a.parse_args().transcript)
