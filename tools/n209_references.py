"""Primary bibliographic metadata only; never uploads unpublished research."""
import json
import urllib.request
from pathlib import Path

root=Path(__file__).resolve().parents[1]/'paper/system_paper/literature_metadata'
root.mkdir(parents=True,exist_ok=True)
dois=['10.1016/j.actaastro.2018.04.016','10.1016/j.asr.2022.07.070','10.1016/j.jfranklin.2023.12.044',
      '10.1109/TMECH.2025.3545201','10.1109/iSpaRo66239.2025.11437302','10.1177/02783649241258215']
for i,doi in enumerate(dois):
    path=root/(str(i+1)+'.json')
    if path.exists():continue
    try:
        req=urllib.request.Request('https://api.crossref.org/works/'+doi,headers={'User-Agent':'N209-bibliographic-verification/1.0'})
        with urllib.request.urlopen(req,timeout=30) as f:obj=json.load(f)
        path.write_text(json.dumps(obj,ensure_ascii=False,indent=2),encoding='utf-8')
        m=obj['message'];print(doi,m.get('title'),m.get('author'),m.get('published'),flush=True)
    except Exception as ex:print(doi,repr(ex),flush=True)
