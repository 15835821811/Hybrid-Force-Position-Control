from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[4]))
from v6_mujoco.adaptive_capture.figures import run
from v6_mujoco.adaptive_capture.common import ROOT,read
if __name__ == "__main__":
    run(sys.argv[1] if len(sys.argv)>1 else read(ROOT/"experiment_manifest.json")["selected_development_run"],'identification')
