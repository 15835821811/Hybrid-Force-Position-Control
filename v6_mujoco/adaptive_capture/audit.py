"""Explicit access audit and dependency inventory, not a security sandbox proof."""
import ast
from pathlib import Path
from .common import ROOT,save,runtime_identity

def run():
    package=Path(__file__).parent
    online=['contracts.py','state_estimator.py','momentum_regressor.py','inertial_estimator.py','information_gate.py','relative_reference.py','controller.py','risk.py','governor.py','known_model.py']
    violations=[];inventory={}
    for name in online:
        tree=ast.parse((package/name).read_text(encoding='utf-8'));imports=[];attrs=[]
        for n in ast.walk(tree):
            if isinstance(n,ast.ImportFrom):
                imports.append(n.module)
                if n.module and (n.module.endswith('plant') or n.module.endswith('evaluation') or n.module.endswith('sensors')):violations.append([name,n.lineno,'private module import'])
            if isinstance(n,ast.Attribute):
                if n.attr in ['qacc','body_mass','body_inertia','body_ipos','body_iquat','qfrc_constraint','xfrc_applied','qfrc_applied']:
                    attrs.append([n.lineno,n.attr])
                    if name not in ['known_model.py','governor.py']:violations.append([name,n.lineno,n.attr])
        inventory[name]={'imports':imports,'sensitive_attributes':attrs}
    result={'static_access_passed':not violations,'violations':violations,'inventory':inventory,
        'allowed_known_model_access':'known_model sums known robot/tool bodies only; target prior written from independent YAML before control. governor mutates independent predictive models only.',
        'sensor_boundary':'frontend owns copy of true solver; only pose of geometric B, robot navigation/encoders, actuator torque, ideal target interface F/T and contact flag leave it.',
        'evaluation_boundary':'true capture and safety checked independently; false estimated capture is retained as failure, not corrected. Safety can terminate simulation only.',
        'legacy_dependency_boundary':'N206 nominal reference is a fixed geometric path template; live target transform is always estimated. Inherited robot/template configs are public design priors, not validation scenario truth.',
        'runtime_test_evidence':'selftests isolation_tests; full sensor replay additionally required per executed run',
        'limitations':'source and behavioral audit, not an OS-enforced capability sandbox; no learned vision or physical sensor validation','identity':runtime_identity()}
    save(ROOT/'truth_access_audit.json',result);return result

if __name__=='__main__':
    r=run();print({'static_access_passed':r['static_access_passed'],'violations':r['violations']})
